from glob import glob

from setuptools import find_packages, setup

package_name = 'covapsy_simple_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # circuits : installés comme données ROS 2 (share/), pas comme sous-paquet Python
        ('share/' + package_name + '/circuits',
            glob(package_name + '/circuits/*.yaml') + glob(package_name + '/circuits/*.npz')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='covapsy',
    maintainer_email='covapsy@example.com',
    description='Simulateur Python 2D léger, mêmes interfaces ROS 2 que la voiture Webots',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'simulateur = covapsy_simple_sim.noeud_simulateur:main',
            'chronometre = covapsy_simple_sim.chronometre:main',
        ],
    },
)
