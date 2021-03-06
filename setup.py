from setuptools import setup

package_name = 'auto_nav'

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
    maintainer='nus',
    maintainer_email='nus@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'r2mover = auto_nav.r2mover:main',
            'r2moverotate = auto_nav.r2moverotate:main',
            'r2scanner = auto_nav.r2scanner:main',
            'r2occupancy = auto_nav.r2occupancy:main',
            'r2occupancy2 = auto_nav.r2occupancy2:main',
            'r2auto_nav = auto_nav.r2auto_nav:main',
            'r2auto_nav_old = auto_nav.r2auto_nav_old:main',
            'r3auto_nav = auto_nav.r3auto_nav:main',
            'r4auto_nav = auto_nav.r4auto_nav:main',
            'r5auto_nav = auto_nav.r5auto_nav:main',
            'r6auto_nav = auto_nav.r6auto_nav:main',
            'r7auto_nav = auto_nav.r7auto_nav:main',
            'r8auto_nav = auto_nav.r8auto_nav:main',
            'nav = auto_nav.nav:main',
            'fly = auto_nav.fly:main',
            'bfs = auto_nav.BFS:main',
            'bfs2 = auto_nav.BFS2:main',
            'nav10 = auto_nav.nav_10:main',
            'r9auto_nav = auto_nav.r9auto_nav:main'
        ],
    },
)
