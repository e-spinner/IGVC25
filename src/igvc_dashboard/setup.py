from setuptools import find_packages, setup

package_name = 'igvc_dashboard'

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
    maintainer='Elijah Spinner',
    maintainer_email='e-spinner@onu.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'dashboard = igvc_dashboard.dashboard:main',
        ],
    },
)
