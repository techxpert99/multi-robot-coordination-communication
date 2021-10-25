from setuptools import setup

package_name = 'controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ritik Jain',
    maintainer_email='rjain2@cs.iitr.ac.in',
    description='Controller for Robots',
    license='GNU Public License (GPL)',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = controller.main:entry_point'
        ],
    },
)
