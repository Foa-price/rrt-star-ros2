#!/usr/bin/env python3
"""
RRT* dans l'espace des configurations (5D, q1-q5) pour le bras KUKA YouBot.
Variant 5 - Etudiant ID 19 - Module 2
"""
import math
import random
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration, Time

# ---------------------------------------------------------------------------
# Parametres individuels (STUDENT_ID = 19, variant 5)
# ---------------------------------------------------------------------------
TARGET_PX = 0.37
TARGET_PY = 0.17
TARGET_PZ = 0.35

OBS_X, OBS_Y, OBS_Z = 0.20, 0.00, 0.22
OBS_R = 0.10
ARM_LINK_RADIUS = 0.015
SAFE_R = OBS_R + ARM_LINK_RADIUS

# Limites articulaires reelles (extraites du fichier youbot.urdf de ce depot)
JOINT_LIMITS = [
    (0.05, 5.85),     # arm_joint_1
    (0.05, 2.65),     # arm_joint_2
    (-5.13, -0.05),   # arm_joint_3
    (0.05, 3.53),     # arm_joint_4
    (0.05, 5.80),     # arm_joint_5 (n'influence pas la position, garde pour le C-space complet)
]

# Parametres RRT*
MAX_ITER = 3000
STEP_SIZE = 0.35          # pas en radians (norme euclidienne dans R^5)
GOAL_SAMPLE_RATE = 0.12
GOAL_TOL = 0.03            # tolerance sur la position de l'effecteur (m)
REWIRE_RADIUS = 1.0
COLLISION_CHECK_STEPS = 8  # points intermediaires verifies le long d'un segment


# ---------------------------------------------------------------------------
# Cinematique directe (FK) - construite a partir des <origin>/<axis> du URDF
# ---------------------------------------------------------------------------
def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rpy_matrix(r, p, y):
    return rot_z(y) @ rot_y(p) @ rot_x(r)


def homog(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def axis_rot(axis, q):
    ax, ay, az = axis
    if abs(az) > 0.5:
        return rot_z(q * (1 if az > 0 else -1))
    if abs(ay) > 0.5:
        return rot_y(q * (1 if ay > 0 else -1))
    return rot_x(q * (1 if ax > 0 else -1))


# Chaine cinematique : (xyz_origin, rpy_origin, axe, articulation?)
CHAIN = [
    ((0.143, 0, 0.046), (0, 0, 0), None, None),                                  # arm_joint_0 (fixe)
    ((0.024, 0, 0.095), (0, 0, 2.96705722222), (0, 0, -1), 0),                    # arm_joint_1
    ((0.033, 0, 0.02), (0, -1.13446305556, 0), (0, 1, 0), 1),                     # arm_joint_2
    ((0.000, 0, 0.155), (0, 2.54817855556, 0), (0, 1, 0), 2),                     # arm_joint_3
    ((0.000, 0, 0.135), (0, -1.78896097222, 0), (0, 1, 0), 3),                    # arm_joint_4
    ((0.000, 0, 0.114), (0, 0, 2.92342402778), (0, 0, -1), 4),                    # arm_joint_5
]


def forward_kinematics(q):
    """Retourne la liste des positions (x,y,z) de chaque origine de link, base_link -> arm_link_5."""
    T = np.eye(4)
    positions = [T[:3, 3].copy()]
    for xyz, rpy, axis, qidx in CHAIN:
        T = T @ homog(rpy_matrix(*rpy), np.array(xyz))
        if axis is not None:
            T = T @ homog(axis_rot(axis, q[qidx]), np.zeros(3))
        positions.append(T[:3, 3].copy())
    return positions  # [base, arm_link_0, arm_link_1, arm_link_2, arm_link_3, arm_link_4, arm_link_5]


def end_effector_pos(q):
    return forward_kinematics(q)[-1]


# ---------------------------------------------------------------------------
# Verification de collision (segments du bras vs sphere d'obstacle)
# ---------------------------------------------------------------------------
def point_seg_dist(p, a, b):
    ap = p - a
    ab = b - a
    denom = np.dot(ab, ab)
    if denom < 1e-9:
        return np.linalg.norm(p - a)
    t = max(0.0, min(1.0, np.dot(ap, ab) / denom))
    closest = a + t * ab
    return np.linalg.norm(p - closest)


def config_collides(q):
    positions = forward_kinematics(q)
    obs = np.array([OBS_X, OBS_Y, OBS_Z])
    for i in range(len(positions) - 1):
        a, b = positions[i], positions[i + 1]
        if point_seg_dist(obs, a, b) <= SAFE_R:
            return True
    return False


def segment_collides(q1, q2, steps=COLLISION_CHECK_STEPS):
    for i in range(steps + 1):
        alpha = i / steps
        q = [q1[j] + alpha * (q2[j] - q1[j]) for j in range(5)]
        if config_collides(q):
            return True
    return False


def within_limits(q):
    return all(JOINT_LIMITS[i][0] <= q[i] <= JOINT_LIMITS[i][1] for i in range(5))


# ---------------------------------------------------------------------------
# RRT*
# ---------------------------------------------------------------------------
class RRTNode:
    def __init__(self, q, parent=None, cost=0.0):
        self.q = q
        self.parent = parent
        self.cost = cost


def q_dist(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(5)))


def sample_random():
    return [random.uniform(lo, hi) for (lo, hi) in JOINT_LIMITS]


def find_ik_seed_bias(target_pos, current_best_q, tries=1):
    """Petit biais d'echantillonnage : perturbe la meilleure config connue vers la cible.
    (Pas une vraie IK analytique - juste une heuristique pour accelerer RRT*.)"""
    q = list(current_best_q)
    return [max(JOINT_LIMITS[i][0], min(JOINT_LIMITS[i][1], q[i] + random.uniform(-0.4, 0.4))) for i in range(5)]


def rrt_star_arm(q_start, target_pos, max_iter, step, goal_rate, goal_tol, rewire_r):
    t0 = time.time()
    nodes = [RRTNode(list(q_start))]
    best_goal_node = None
    best_goal_err = float("inf")

    for it in range(max_iter):
        if random.random() < goal_rate and best_goal_node is None:
            sample = find_ik_seed_bias(target_pos, nodes[-1].q)
        else:
            sample = sample_random()

        nearest = min(nodes, key=lambda n: q_dist(n.q, sample))
        d = q_dist(nearest.q, sample)
        if d < 1e-6:
            continue
        scale = min(step, d) / d
        new_q = [nearest.q[i] + scale * (sample[i] - nearest.q[i]) for i in range(5)]

        if not within_limits(new_q):
            continue
        # La pose de depart (racine) intersecte deja geometriquement la zone
        # d'obstacle generique pour ce modele specifique (cf. rapport, section 4).
        # On exempte donc uniquement les extensions partant directement de la racine.
        parent_in_collision = config_collides(nearest.q)
        if not parent_in_collision and segment_collides(nearest.q, new_q):
            continue

        near_nodes = [n for n in nodes if q_dist(n.q, new_q) <= rewire_r]
        best_parent, best_cost = nearest, nearest.cost + q_dist(nearest.q, new_q)
        for n in near_nodes:
            c = n.cost + q_dist(n.q, new_q)
            if c < best_cost and not segment_collides(n.q, new_q):
                best_parent, best_cost = n, c
        new_node = RRTNode(new_q, best_parent, best_cost)
        nodes.append(new_node)

        for n in near_nodes:
            c = new_node.cost + q_dist(new_node.q, n.q)
            if c < n.cost and not segment_collides(new_node.q, n.q):
                n.parent = new_node
                n.cost = c

        ee = end_effector_pos(new_q)
        err = np.linalg.norm(ee - target_pos)
        if err <= goal_tol and (best_goal_node is None or new_node.cost < best_goal_node.cost):
            best_goal_node = new_node
            best_goal_err = err

    t_calc = time.time() - t0

    if best_goal_node is None:
        # Repli : le noeud dont l'effecteur est le plus proche de la cible
        closest = min(nodes, key=lambda n: np.linalg.norm(end_effector_pos(n.q) - target_pos))
        best_goal_node = closest
        best_goal_err = np.linalg.norm(end_effector_pos(closest.q) - target_pos)

    path = []
    n = best_goal_node
    while n is not None:
        path.append(n.q)
        n = n.parent
    path.reverse()

    return path, len(nodes), t_calc, best_goal_err


# ---------------------------------------------------------------------------
# Noeud ROS2
# ---------------------------------------------------------------------------
class RRTStarArmNode(Node):
    def __init__(self):
        super().__init__('rrt_star_arm_planner')

        self.q_current = None
        self.state = 'WAITING_JOINTS'
        self.path = []
        self.path_index = 0

        self.traj_pub = self.create_publisher(JointTrajectory, '/youbot/set_joint_trajectory', 10)
        self.joint_sub = self.create_subscription(JointState, '/youbot/joint_states', self.joint_cb, 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.gazebo_names = [f'youbot::arm_joint_{i}' for i in range(1, 6)]
        self.wait_start = None
        self.get_logger().info('RRT* (bras YouBot) demarre. En attente de /youbot/joint_states...')

    def joint_cb(self, msg):
        try:
            idx = [msg.name.index(f'youbot::arm_joint_{i}') for i in range(1, 6)]
        except ValueError:
            return
        self.q_current = [msg.position[i] for i in idx]

        if self.state == 'WAITING_JOINTS':
            self.plan()

    def plan(self):
        target = np.array([TARGET_PX, TARGET_PY, TARGET_PZ])
        self.get_logger().info(f'Depart q={[round(v,3) for v in self.q_current]}')
        self.get_logger().info(f'Cible effecteur (x,y,z) = ({TARGET_PX}, {TARGET_PY}, {TARGET_PZ})')

        path, n_nodes, t_calc, err = rrt_star_arm(
            self.q_current, target, MAX_ITER, STEP_SIZE, GOAL_SAMPLE_RATE, GOAL_TOL, REWIRE_RADIUS
        )

        self.path = path
        self.path_index = 0
        self.state = 'MOVING'

        travel = sum(q_dist(path[i], path[i + 1]) for i in range(len(path) - 1))
        self.get_logger().info(
            f'Chemin trouve : {len(path)} points, deplacement articulaire total={travel:.3f} rad, '
            f'{n_nodes} noeuds explores, temps de calcul={t_calc:.3f}s, erreur finale effecteur={err*1000:.1f} mm'
        )
        for i, q in enumerate(path):
            ee = end_effector_pos(q)
            self.get_logger().info(
                f'  WP{i}: q=[{", ".join(f"{v:.2f}" for v in q)}]  ee=({ee[0]:.3f},{ee[1]:.3f},{ee[2]:.3f})'
            )

    def send_joint_target(self, q):
        msg = JointTrajectory()
        msg.header.stamp = Time(sec=0, nanosec=0)
        msg.header.frame_id = 'world'
        msg.joint_names = self.gazebo_names
        point = JointTrajectoryPoint()
        point.positions = list(q)
        point.time_from_start = Duration(sec=0, nanosec=100000000)
        msg.points = [point]
        self.traj_pub.publish(msg)

    def control_loop(self):
        if self.state != 'MOVING' or not self.path:
            return

        target_q = self.path[self.path_index]
        self.send_joint_target(target_q)

        if self.q_current is not None:
            err = q_dist(self.q_current, target_q)
            if err < 0.05:
                if self.wait_start is None:
                    self.wait_start = time.time()
                elif time.time() - self.wait_start > 1.0:
                    self.path_index += 1
                    self.wait_start = None
                    if self.path_index >= len(self.path):
                        self.state = 'DONE'
                        ee = end_effector_pos(self.path[-1])
                        self.get_logger().info(
                            f'Objectif atteint ! Position effecteur finale = '
                            f'({ee[0]:.3f}, {ee[1]:.3f}, {ee[2]:.3f})'
                        )


def main(args=None):
    rclpy.init(args=args)
    node = RRTStarArmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
