from setuptools import find_packages, setup

package_name = 'rcprg_smach'

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
    maintainer='adam',
    maintainer_email='adam.krawczyk@robotec.ai',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={'console_scripts': [ 'bring_goods_tasker = rcprg_smach.rcprg_smach.nodes.bring_goods_tasker:main',
                                      'bring_goods = rcprg_smach.rcprg_smach.nodes.bring_goods:main', 'bring_jar_tasker = rcprg_smach.rcprg_smach.nodes.bring_jar_tasker:main', 'bring_jar = rcprg_smach.rcprg_smach.nodes.bring_jar:main', 'call = rcprg_smach.rcprg_smach.nodes.call:main', 'guide_human_tasker = rcprg_smach.rcprg_smach.nodes.guide_human_tasker:main', 'human_fell_approach_tasker = rcprg_smach.rcprg_smach.nodes.human_fell_approach_tasker:main', 'human_fell_approach = rcprg_smach.rcprg_smach.nodes.human_fell_approach:main', 'idle_tasker = rcprg_smach.rcprg_smach.nodes.idle_tasker:main', 'idle = rcprg_smach.rcprg_smach.nodes.idle:main', 'move_to_tasker = rcprg_smach.rcprg_smach.nodes.move_to_tasker.main', 'move_to = rcprg_smach.rcprg_smach.nodes.idle:main', 'stop = rcprg_smach.rcprg_smach.nodes.stop:main', 'task_harmonizer = rcprg_smach.rcprg_smach.nodes.task_harmonizer:main', 'test_bring_goods = rcprg_smach.rcprg_smach.nodes.test_bring_goods:main', 'test_move_to = rcprg_smach.rcprg_smach.nodes.test_move_to:main', 'test_wander = rcprg_smach.rcprg_smach.nodes.test_wander:main', 'wander = rcprg_smach.rcprg_smach.nodes.wander:main', 'nav2_statuses = rcprg_smach.nodes.nav2_statuses:main'
                                      ]},

)
