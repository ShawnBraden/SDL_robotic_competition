from setuptools import find_packages, setup

package_name = 'state_space_algo_pub'

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
    maintainer_email='sbradenjones16@gmail.com',
    description='This package handles the simulations of the poss of the robot',
    license='Student',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_sim = state_space_algo_pub.state_locations:main'
        ],
    },
)
