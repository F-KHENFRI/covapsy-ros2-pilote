"""
Duel : VOTRE pilote (car1, en pôle) contre 3 adversaires (pilote de base figé).

  ros2 launch covapsy_student_bringup duel.launch.py                       # niveau lent
  ros2 launch covapsy_student_bringup duel.launch.py niveau:=rapide circuit:=epingle

4 voitures dans UN simulateur : chaque voiture a son NAMESPACE (/car1, /car2…) avec sa
propre infrastructure et son pilote. Exemple : /car2/scan -> /car2/adversaire.
Essayez pendant un duel :  ros2 node list   ros2 topic list   rqt_graph

Arguments : circuit, niveau (lent | moyen | rapide), tours, facteur_temps_reel,
affichage (true), resultat (JSON), parametres (YAML de votre pilote).
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

# Réglages des adversaires (paramètres ROS 2 du nœud « adversaire »)
NIVEAUX = {                       # adversaire = pilote de base, vitesse constante
    'lent': {'vitesse': 1.0},
    'moyen': {'vitesse': 1.5},
    'rapide': {'vitesse': 1.8},
}
VOITURES = ['car1', 'car2', 'car3', 'car4']


def construire(context):
    get = lambda k: LaunchConfiguration(k).perform(context)   # noqa: E731
    niveau = get('niveau')
    if niveau not in NIVEAUX:
        raise RuntimeError(f'niveau « {niveau} » inconnu : {", ".join(NIVEAUX)}')
    infra = os.path.join(FindPackageShare('covapsy_autonomy').find('covapsy_autonomy'),
                         'config', 'infra.yaml')
    parametres = get('parametres') or os.path.join(
        FindPackageShare('covapsy_pilote').find('covapsy_pilote'), 'config', 'pilote.yaml')
    sim_time = {'use_sim_time': True}
    actions = [Node(package='covapsy_simple_sim', executable='simulateur',
                    parameters=[{'circuit': get('circuit'), 'voitures': VOITURES,
                                 'affichage': get('affichage') == 'true',
                                 'facteur_temps_reel': float(get('facteur_temps_reel'))}])]
    for v in VOITURES:
        actions += [
            Node(package='covapsy_autonomy', executable='scan_processor', namespace=v,
                 parameters=[infra, sim_time]),
            Node(package='covapsy_autonomy', executable='race_manager', namespace=v,
                 parameters=[infra, sim_time, {'autostart_delay': 0.5}]),
            Node(package='covapsy_autonomy', executable='safety_watchdog', namespace=v,
                 parameters=[infra, sim_time]),
        ]
        if v == 'car1':
            actions.append(Node(package='covapsy_pilote', executable='pilote', namespace=v,
                                parameters=[parametres, sim_time]))
        else:
            actions.append(Node(package='covapsy_pilote', executable='adversaire',
                                namespace=v, parameters=[sim_time, NIVEAUX[niveau]]))
    chrono = Node(package='covapsy_simple_sim', executable='chronometre', output='screen',
                  parameters=[{'circuit': get('circuit'), 'voitures': VOITURES,
                               'noms': ['VOUS'] + [f'{niveau}_{i}' for i in (2, 3, 4)],
                               'tours': int(get('tours')), 'duree_max': 180.0,
                               'resultat': get('resultat')}])
    return actions + [chrono, RegisterEventHandler(OnProcessExit(
        target_action=chrono, on_exit=[EmitEvent(event=Shutdown())]))]


def generate_launch_description():
    defauts = {'circuit': 'piste_officielle', 'niveau': 'lent', 'tours': '3',
               'facteur_temps_reel': '1.0', 'affichage': 'true', 'resultat': '',
               'parametres': ''}
    return LaunchDescription([*[DeclareLaunchArgument(k, default_value=v)
                                for k, v in defauts.items()],
                              OpaqueFunction(function=construire)])
