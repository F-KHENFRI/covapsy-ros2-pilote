"""
Lancer le pilote de l'équipe.

  ros2 launch covapsy_student_bringup pilote.launch.py                     # selon equipe.yaml
  ros2 launch covapsy_student_bringup pilote.launch.py mode:=local circuit:=epingle
  ros2 launch covapsy_student_bringup pilote.launch.py mode:=distant

mode:=local    simulateur Python + infrastructure + VOTRE nœud pilote, sur ce PC :

     simulateur ──scan, ultrasons──▶ scan_processor ──▶ race_manager ──race/state──┐
         ▲                                                                    ▼
         └──drive── safety_watchdog ◀──drive_raw── pilote (VOUS) ◀──scan─────┘

mode:=distant  VOTRE nœud pilote seul : simulateur et infrastructure
               tournent sur le PC de l'enseignant (ROS_DOMAIN_ID = domaine de votre
               équipe, voir equipe.yaml)

Le nœud pilote ne voit aucune différence : mêmes topics, mêmes messages.
Arguments : mode, circuit, obstacles, affichage, facteur_temps_reel, depart_auto (s),
parametres (fichier YAML du pilote), lidar_bruit, couleur_droite (vert | rouge : couleur du
mur de droite dans le sens de course ; vide = celle du circuit, vert par défaut).
"""
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


def lire_equipe():
    for c in (os.environ.get('COVAPSY_EQUIPE', ''),
              os.path.join(os.environ.get('COVAPSY_STUDENT_WS', ''), 'equipe.yaml'),
              os.path.join(os.getcwd(), 'equipe.yaml')):
        if c and os.path.isfile(c):
            with open(c) as f:
                return c, yaml.safe_load(f) or {}
    return None, {}


def infrastructure(depart_auto, namespace=''):
    """Nœuds « voiture » : perception, état de course, sécurité."""
    infra = os.path.join(FindPackageShare('covapsy_autonomy').find('covapsy_autonomy'),
                         'config', 'infra.yaml')
    return [
        Node(package='covapsy_autonomy', executable='scan_processor', namespace=namespace,
             parameters=[infra, {'use_sim_time': True}]),
        Node(package='covapsy_autonomy', executable='race_manager', namespace=namespace,
             parameters=[infra, {'use_sim_time': True, 'autostart_delay': depart_auto}]),
        Node(package='covapsy_autonomy', executable='safety_watchdog', namespace=namespace,
             parameters=[infra, {'use_sim_time': True}]),
    ]


def construire(context):
    get = lambda k: LaunchConfiguration(k).perform(context)   # noqa: E731
    mode = get('mode')
    parametres = get('parametres') or os.path.join(
        FindPackageShare('covapsy_pilote').find('covapsy_pilote'), 'config', 'pilote.yaml')
    pilote = Node(package='covapsy_pilote', executable='pilote', output='screen',
                  parameters=[parametres, {'use_sim_time': True}])
    actions = [LogInfo(msg=f"mode {mode}, ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')}"
                           f', paramètres {parametres}')]
    if mode == 'local':
        sim = Node(package='covapsy_simple_sim', executable='simulateur', output='screen',
                   parameters=[{'circuit': get('circuit'),
                                'obstacles': get('obstacles') == 'true',
                                'voitures': ['car1'], 'sans_namespace': True,
                                'affichage': get('affichage') == 'true',
                                'facteur_temps_reel': float(get('facteur_temps_reel')),
                                'lidar_bruit': float(get('lidar_bruit')),
                                'couleur_droite': get('couleur_droite')}])
        actions += [sim, *infrastructure(float(get('depart_auto'))), pilote,
                    RegisterEventHandler(OnProcessExit(          # fenêtre fermée -> fin
                        target_action=sim, on_exit=[EmitEvent(event=Shutdown())]))]
    elif mode == 'distant':
        attendu = get('domaine')
        if attendu and attendu != os.environ.get('ROS_DOMAIN_ID', '0'):
            actions.append(LogInfo(msg=f'ATTENTION : ROS_DOMAIN_ID différent du domaine '
                                       f'{attendu} de equipe.yaml (ouvrez un nouveau terminal)'))
        actions.append(pilote)
    else:
        raise RuntimeError(f"mode « {mode} » inconnu : 'local' ou 'distant'")
    return actions


def generate_launch_description():
    chemin, eq = lire_equipe()
    defauts = {'mode': str(eq.get('mode', 'local')), 'circuit': str(eq.get('circuit', 'ovale')),
               'obstacles': 'true', 'affichage': 'true', 'facteur_temps_reel': '1.0',
               'depart_auto': '2.0', 'parametres': '', 'lidar_bruit': '0.005',
               'domaine': str(eq.get('domaine', '')), 'couleur_droite': ''}
    return LaunchDescription([
        LogInfo(msg=f'configuration équipe : {chemin or "aucun equipe.yaml (défauts)"}'),
        *[DeclareLaunchArgument(k, default_value=v) for k, v in defauts.items()],
        OpaqueFunction(function=construire),
    ])
