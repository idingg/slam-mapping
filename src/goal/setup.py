from setuptools import find_packages, setup

package_name = 'goal'

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
    maintainer='ynu',
    maintainer_email='ynu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data = goal.get_mapdata_t10:main',
            'calculate = goal.calculate_t13:main',
            'go = goal.action_t6:main',
            'total = goal.total_t3:main'
        ],
    },
)
