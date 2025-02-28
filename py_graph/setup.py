from setuptools import setup

package_name = 'py_graph'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lm',
    maintainer_email='94664592+liumingzs@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'graph = py_graph.graph:main',
            'image_publish = py_graph.image_publish:main',  # 添加这一行
            'route_plan = py_graph.route_plan:main',
            'second_image_save = py_graph.second_image_save:main',
            'pose_subscriber = py_graph.pose_subscriber:main',
            'record_virtual_tracking = py_graph.record_virtual_tracking:main',
            'velocity_publisher = py_graph.velocity_publisher:main',
            'image_to_video = py_graph.image_to_video:main',
        ],
    },
)
