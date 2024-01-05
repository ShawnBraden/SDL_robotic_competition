from setuptools import find_packages, setup

package_name = 'RRT_Tree_node'

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
    maintainer='shawnjones',
    maintainer_email='A02248994@aggie.usu.ed',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'start_tree = RRT_Tree_node.rrt_tree:main',
            'test_tree = RRT_Tree_node.rrt_tree_test:main'
        ],
    },
)
