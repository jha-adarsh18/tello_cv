from setuptools import setup

package_name = 'cv1'

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
    maintainer=['rahul', 'Adarsh']
    maintainer_email=['rahulk6661@gmail.com', 'jhaadarsh350@gmail.com']
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "tellocmd=cv1.telloyaw:main",
            "tellotakeoff=cv1.takeoff:main",
            "telloland=cv1.land:main",
            "telloobject=cv1.object:main",
            "climb=cv1.telloclimb:main",
            "tonce=cv1.telloyawonce:main",
            "fall=cv1.tellodown:main",
            "rover=cv1.rover_tracking:main",
        ],
    },
)
