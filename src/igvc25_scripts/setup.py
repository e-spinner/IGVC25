from setuptools import setup

package_name = 'igvc25_scripts'  

setup(
    name='igvc25_scripts',  
    version='0.0.1',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/igvc25_scripts']),
        ('share/igvc25_scripts', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Elijah Spinner',
    maintainer_email='e-spinner@onu.edu',
    description='Additional Nodes for ONU IGVC25 Capstone',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'republish_scan = igvc25_scripts.republish_scan:main',
        ],
    },
)
