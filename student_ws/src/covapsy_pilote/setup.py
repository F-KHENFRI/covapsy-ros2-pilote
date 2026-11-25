from glob import glob

from setuptools import find_packages, setup

package_name = 'covapsy_pilote'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='equipe',
    maintainer_email='equipe@example.com',
    description="Nœud ROS 2 du pilote de l'équipe CoVAPSy",
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pilote = covapsy_pilote.pilote:main',                 # VOTRE pilote
            'adversaire = covapsy_pilote.pilote_base:main_adversaire',   # pilote de base figé
        ],
    },
)
