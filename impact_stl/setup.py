from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'impact_stl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob(os.path.join('impact_stl/config', '*.rviz'))),     # rviz configs
        (os.path.join('share', package_name), glob(os.path.join('impact_stl/config', '*.xml'))),      # plotjuggler configs


        (os.path.join('share', package_name), glob(os.path.join('impact_stl/launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name), glob(os.path.join('impact_stl/launch/platforms', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name), glob(os.path.join('impact_stl/launch/atmos_paper', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name), glob(os.path.join('impact_stl/launch/atmos_paper_multi', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name), glob(os.path.join('impact_stl/launch/simple_push', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name), glob(os.path.join('impact_stl/launch/throw_and_catch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name), glob(os.path.join('impact_stl/launch/obstacle_avoidance', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name), glob(os.path.join('impact_stl/launch/double_throw', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name), glob(os.path.join('impact_stl/launch/multi_agent_tracking', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name), glob(os.path.join('impact_stl/launch/cbf_test', '*launch.[pxy][yma]*'))),

        (os.path.join('share', package_name), glob(os.path.join('impact_stl/launch/lab_test_1', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name), glob(os.path.join('impact_stl/launch/lab_test_2', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name), glob(os.path.join('impact_stl/launch/throw_and_catch_exp', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name), glob(os.path.join('impact_stl/launch/complex_stl', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name), glob(os.path.join('impact_stl/launch/pingpong_stl', '*launch.[pxy][yma]*'))),

        # (os.path.join('share', package_name), glob(os.path.join('impact_stl/planners/plans/throw_and_catch', '*.csv'))),
        # (os.path.join('share', package_name), glob(os.path.join('impact_stl/planners/plans/simple_push', '*.csv'))),
        # (os.path.join('share', package_name), glob(os.path.join('impact_stl/planners/plans/atmos_paper', '*.csv'))),
        (os.path.join('share', package_name), glob(os.path.join('impact_stl/planners/plans/atmos_paper_multi', '*.csv'))),
        # (os.path.join('share', package_name), glob(os.path.join('impact_stl/planners/plans/obstacle_avoidance', '*.csv'))),
        # (os.path.join('share', package_name), glob(os.path.join('impact_stl/planners/plans/obstacle_avoidance/worstcase', '*.csv'))),
        # (os.path.join('share', package_name), glob(os.path.join('impact_stl/planners/plans/double_throw', '*.csv'))),
        # (os.path.join('share', package_name), glob(os.path.join('impact_stl/planners/plans/multi_agent_tracking', '*.csv'))),
        # (os.path.join('share', package_name), glob(os.path.join('impact_stl/planners/plans/cbf_test', '*.csv'))),

        # (os.path.join('share', package_name), glob(os.path.join('impact_stl/planners/plans/lab_test_1', '*.csv'))),
        # (os.path.join('share', package_name), glob(os.path.join('impact_stl/planners/plans/lab_test_2', '*.csv'))),
        # (os.path.join('share', package_name), glob(os.path.join('impact_stl/planners/plans/lab_test_2/more_space', '*.csv'))),
        # (os.path.join('share', package_name), glob(os.path.join('impact_stl/planners/plans/throw_and_catch_exp/best_case', '*.csv'))),
        # (os.path.join('share', package_name), glob(os.path.join('impact_stl/planners/plans/throw_and_catch_exp/worst_case', '*.csv'))),
        # (os.path.join('share', package_name), glob(os.path.join('impact_stl/planners/plans/complex_stl/best_case', '*.csv'))),
        # (os.path.join('share', package_name), glob(os.path.join('impact_stl/planners/plans/complex_stl/worst_case', '*.csv'))),
        # (os.path.join('share', package_name), glob(os.path.join('impact_stl/planners/plans/pingpong_stl', '*.csv'))),

        # 1 to 1 comparisons
        # (os.path.join('share', package_name), glob(os.path.join('impact_stl/planners/plans/throw_and_catch/bestcase', '*.csv'))),     # config files
        # (os.path.join('share', package_name), glob(os.path.join('impact_stl/planners/plans/throw_and_catch/worstcase', '*.csv'))), 
        # (os.path.join('share', package_name), glob(os.path.join('impact_stl/planners/plans/obstacle_avoidance/bestcase', '*.csv'))),
        # (os.path.join('share', package_name), glob(os.path.join('impact_stl/planners/plans/obstacle_avoidance/worstcase', '*.csv'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jorisv',
    maintainer_email='jorisv@kth.se',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                # planning and missions
                'main_planner = impact_stl.planners.main_planner:main',
                'replanner = impact_stl.planners.replanner:main',
                'scenario = impact_stl.scenario:main',
                'reset = impact_stl.reset:main',

                # controllers
                'ff_rate_mpc = impact_stl.ff_rate_mpc:main',
                'ff_rate_mpc_impact = impact_stl.ff_rate_mpc_impact:main',
                'ff_wrench_mpc_impact = impact_stl.ff_wrench_mpc_impact:main',
                'ff_rate_mpc_velocity_keeping = impact_stl.ff_rate_mpc_velocity_keeping:main',
                'ff_rate_qp_velocity_keeping = impact_stl.ff_rate_qp_velocity_keeping:main',

                # helpers
                'impact_detector = impact_stl.helpers.impact_detector:main',
                'odom_to_vehicle_local_position = impact_stl.helpers.odom_to_vehicle_local_position:main',
                'odom_to_vehicle_angular_velocity = impact_stl.helpers.odom_to_vehicle_angular_velocity:main',
                'odom_to_vehicle_attitude = impact_stl.helpers.odom_to_vehicle_attitude:main',

                # tests
                'test_replanner = impact_stl.planners.test_replanner:main',
                'test_rate_mpc = impact_stl.controllers.test_rate_mpc:main',
                'test_impact_mpc = impact_stl.controllers.test_impact_mpc:main',
        ],
    },
)
