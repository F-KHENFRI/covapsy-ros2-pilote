from glob import glob

from setuptools import setup

package_name = 'covapsy_student_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='covapsy',
    maintainer_email='covapsy@example.com',
    description="Lancement côté équipe : pilote local ou distant, chrono, duel",
    license='Apache-2.0',
)
