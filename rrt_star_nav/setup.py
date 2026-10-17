from setuptools import find_packages, setup

package_name = 'rrt_star_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='foadiengmichel1@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
           'rrt_star_planner = rrt_star_nav.rrt_star_planner:main',
           'rrt_star_arm_planner = rrt_star_nav.rrt_star_arm_planner:main',
        ],
    },
)
