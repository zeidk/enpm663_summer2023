from setuptools import setup

package_name = 'first_package_py'

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
    maintainer='zeid',
    maintainer_email='zeidk@umd.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_node_exe = first_package_py.simple_node:main',
            'minimal_publisher_exe = first_package_py.minimal_publisher:main',
            'exercise1_exe = first_package_py.exercise1:main',
            'exercise2_exe = first_package_py.exercise2:main',
            'exercise3_exe = first_package_py.exercise3:main',
            'advanced_publisher_exe = first_package_py.advanced_publisher:main',
            'subscriber_exe = first_package_py.subscriber:main'
        ],
    },
)
