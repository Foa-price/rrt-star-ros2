"""
Noeud ROS2 : planification de trajectoire par RRT* pour robot differentiel.
Variant 5 - Etudiant ID 19
"""
import math
import random
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# ---------------------------------------------------------------------------
# Parametres individuels (STUDENT_ID = 19)
# ---------------------------------------------------------------------------
TARGET_X = 3.00
TARGET_Y = 2.25
OBS_X = 1.20
OBS_Y = 0.85
OBS_R = 0.35

# Marge de securite ajoutee au rayon de l'obstacle pour le robot (rayon robot ~0.15m)
ROBOT_RADIUS = 0.15
SAFE_R = OBS_R + ROBOT_RADIUS

# Bornes de l'espace de recherche (m)
X_MIN, X_MAX = -0.5, 4.0
Y_MIN, Y_MAX = -0.5, 3.0

# Parametres RRT*
MAX_ITER = 800
STEP_SIZE = 0.25
GOAL_SAMPLE_RATE = 0.10     # 10% de chances de tirer directement le but
GOAL_RADIUS = 0.15
REWIRE_RADIUS = 0.6


class RRTStarNode:
    """Un noeud de l'arbre RRT*."""
    def __init__(self, x, y, parent=None, cost=0.0):
        self.x = x
        self.y = y
        self.parent = parent
        self.cost = cost


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def collides(p1, p2, ox, oy, r):
    """Verifie si le segment p1->p2 croise le cercle obstacle (ox, oy, r)."""
    x1, y1 = p1
    x2, y2 = p2
    dx, dy = x2 - x1, y2 - y1
    seg_len2 = dx * dx + dy * dy
    if seg_len2 == 0:
        return dist(p1, (ox, oy)) <= r
    t = max(0.0, min(1.0, ((ox - x1) * dx + (oy - y1) * dy) / seg_len2))
    closest = (x1 + t * dx, y1 + t * dy)
    return dist(closest, (ox, oy)) <= r


def rrt_star(start, goal, obs, bounds, max_iter, step, goal_rate, goal_r, rewire_r):
    """Implementation de RRT*. Retourne (chemin, nb_noeuds, temps_calcul)."""
    t0 = time.time()
    ox, oy, orr = obs
    xmin, xmax, ymin, ymax = bounds

    nodes = [RRTStarNode(start[0], start[1])]
    best_goal_node = None

    for _ in range(max_iter):
        # 1. Echantillonnage (avec biais vers le but)
        if random.random() < goal_rate:
            sample = goal
        else:
            sample = (random.uniform(xmin, xmax), random.uniform(ymin, ymax))

        # 2. Noeud le plus proche
        nearest = min(nodes, key=lambda n: dist((n.x, n.y), sample))

        # 3. Direction + pas fixe
        theta = math.atan2(sample[1] - nearest.y, sample[0] - nearest.x)
        new_x = nearest.x + step * math.cos(theta)
        new_y = nearest.y + step * math.sin(theta)

        # 4. Verification collision + bornes
        if not (xmin <= new_x <= xmax and ymin <= new_y <= ymax):
            continue
        if collides((nearest.x, nearest.y), (new_x, new_y), ox, oy, orr):
            continue

        # 5. Choix du meilleur parent parmi les voisins proches
        near_nodes = [n for n in nodes if dist((n.x, n.y), (new_x, new_y)) <= rewire_r]
        best_parent = nearest
        best_cost = nearest.cost + dist((nearest.x, nearest.y), (new_x, new_y))
        for n in near_nodes:
            c = n.cost + dist((n.x, n.y), (new_x, new_y))
            if c < best_cost and not collides((n.x, n.y), (new_x, new_y), ox, oy, orr):
                best_parent, best_cost = n, c

        new_node = RRTStarNode(new_x, new_y, best_parent, best_cost)
        nodes.append(new_node)

        # 6. Rewiring : les voisins passent-ils par new_node pour moins cher ?
        for n in near_nodes:
            c = new_node.cost + dist((new_node.x, new_node.y), (n.x, n.y))
            if c < n.cost and not collides((new_node.x, new_node.y), (n.x, n.y), ox, oy, orr):
                n.parent = new_node
                n.cost = c

        # 7. But atteint ?
        if dist((new_x, new_y), goal) <= goal_r:
            if best_goal_node is None or new_node.cost < best_goal_node.cost:
                best_goal_node = new_node

    t_calc = time.time() - t0

    if best_goal_node is None:
        return None, len(nodes), t_calc

    # Reconstruction du chemin par remontee des parents
    path = []
    n = best_goal_node
    while n is not None:
        path.append((n.x, n.y))
        n = n.parent
    path.reverse()
    path.append(goal)
    return path, len(nodes), t_calc


def yaw_from_quaternion(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class RRTStarPlannerNode(Node):
    def __init__(self):
        super().__init__('rrt_star_planner')

        self.pose = None  # (x, y, yaw)
        self.path = []
        self.path_index = 0
        self.state = 'WAITING_ODOM'

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('RRT* planner node demarre. En attente de /odom...')

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.pose = (x, y, yaw)

        if self.state == 'WAITING_ODOM':
            self.plan()

    def plan(self):
        start = (self.pose[0], self.pose[1])
        goal = (TARGET_X, TARGET_Y)
        obs = (OBS_X, OBS_Y, SAFE_R)
        bounds = (X_MIN, X_MAX, Y_MIN, Y_MAX)

        self.get_logger().info(f'Planification RRT* : depart={start}, but={goal}')
        path, n_nodes, t_calc = rrt_star(
            start, goal, obs, bounds,
            MAX_ITER, STEP_SIZE, GOAL_SAMPLE_RATE, GOAL_RADIUS, REWIRE_RADIUS
        )

        if path is None:
            self.get_logger().error(
                f'Aucun chemin trouve apres {MAX_ITER} iterations ({n_nodes} noeuds, {t_calc:.3f}s).'
            )
            self.state = 'FAILED'
            return

        self.path = path
        self.path_index = 0
        self.state = 'MOVING'

        path_len = sum(dist(path[i], path[i + 1]) for i in range(len(path) - 1))
        self.get_logger().info(
            f'Chemin trouve : {len(path)} points, longueur={path_len:.2f}m, '
            f'{n_nodes} noeuds explores, temps de calcul={t_calc:.3f}s'
        )
        for i, p in enumerate(path):
            self.get_logger().info(f'  Waypoint {i}: ({p[0]:.2f}, {p[1]:.2f})')

    def control_loop(self):
        if self.state != 'MOVING' or self.pose is None:
            return

        if self.path_index >= len(self.path):
            self.stop_robot()
            self.state = 'DONE'
            self.get_logger().info('Objectif atteint ! Robot arrete.')
            return

        target = self.path[self.path_index]
        x, y, yaw = self.pose
        dx, dy = target[0] - x, target[1] - y
        distance = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)
        angle_error = normalize_angle(target_angle - yaw)

        cmd = Twist()

        if distance < 0.12:
            self.path_index += 1
            return

        if abs(angle_error) > 0.35:
            # Rotation sur place d'abord
            cmd.angular.z = max(-0.8, min(0.8, 1.5 * angle_error))
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = max(0.0, min(0.35, 0.6 * distance))
            cmd.angular.z = max(-0.6, min(0.6, 1.2 * angle_error))

        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = RRTStarPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
