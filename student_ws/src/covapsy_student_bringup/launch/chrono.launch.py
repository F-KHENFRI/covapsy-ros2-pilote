"""
Chrono : votre nœud pilote, seul, fait N tours en départ arrêté ; le nœud chronometre
mesure le temps puis arrête tout. Utilisé par « covapsy chrono » (sur 3 circuits).

  ros2 launch covapsy_student_bringup chrono.launch.py circuit:=epingle
  ros2 launch covapsy_student_bringup chrono.launch.py circuit:=ovale tours:=1 affichage:=true

Arguments : circuit, tours (3), duree_max (180 s simulées), facteur_temps_reel (4 : la
simulation va 4 fois plus vite que le temps réel), affichage, resultat (fichier JSON),
parametres (YAML du pilote).
"""
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def construire(context):
    get = lambda k: LaunchConfiguration(k).perform(context)   # noqa: E731
    infra = os.path.join(FindPackageShare('covapsy_autonomy').find('covapsy_autonomy'),
                         'config', 'infra.yaml')
    parametres = get('parametres') or os.path.join(
        FindPackageShare('covapsy_pilote').find('covapsy_pilote'), 'config', 'pilote.yaml')
    sim_time = {'use_sim_time': True}
    chrono = Node(package='covapsy_simple_sim', executable='chronometre', output='screen',
                  parameters=[{'circuit': get('circuit'), 'voitures': ['car1'],
                               'noms': ['pilote'], 'tours': int(get('tours')),
                               'duree_max': float(get('duree_max')),
                               'resultat': get('resultat')}])
    return [
        Node(package='covapsy_simple_sim', executable='simulateur',
             parameters=[{'circuit': get('circuit'), 'voitures': ['car1'],
                          'sans_namespace': True, 'affichage': get('affichage') == 'true',
                          'facteur_temps_reel': float(get('facteur_temps_reel'))}]),
        Node(package='covapsy_autonomy', executable='scan_processor',
             parameters=[infra, sim_time]),
        Node(package='covapsy_autonomy', executable='race_manager',
             parameters=[infra, sim_time, {'autostart_delay': 0.5}]),
        Node(package='covapsy_autonomy', executable='safety_watchdog',
             parameters=[infra, sim_time]),
        Node(package='covapsy_pilote', executable='pilote', parameters=[parametres, sim_time]),
        chrono,
        RegisterEventHandler(OnProcessExit(target_action=chrono,
                                           on_exit=[EmitEvent(event=Shutdown())])),
    ]


def generate_launch_description():
    defauts = {'circuit': 'ovale', 'tours': '3', 'duree_max': '180.0',
               'facteur_temps_reel': '4.0', 'affichage': 'false', 'resultat': '',
               'parametres': ''}
    return LaunchDescription([*[DeclareLaunchArgument(k, default_value=v)
                                for k, v in defauts.items()],
                              OpaqueFunction(function=construire)])
