"""
DÉFI VITESSE (version ROS 2) : chrono officiel, duel, historique.

    covapsy chrono [--note "texte"] [circuit ...]
    covapsy duel [circuit] [lent|moyen|rapide]
    covapsy historique

Chaque mesure lance un vrai système ROS 2 (ros2 launch covapsy_student_bringup
chrono.launch.py / duel.launch.py) : simulateur, infrastructure, VOTRE nœud pilote et le
nœud chronometre. Le SCORE est la somme des temps « 3 tours, départ arrêté » sur les
circuits du chrono (abandon = 180 s). Plus petit = meilleur.
Historique : student_ws/chronos.csv. Les mesures tournent dans un domaine ROS 2 à part
(99 pour le chrono, 98 pour le duel), confiné au PC : elles ne gênent pas un
« covapsy sim » ouvert, et ne sont pas gênées par lui.
"""
import argparse
import csv
import datetime
import json
import os
import subprocess
import tempfile

CIRCUITS_CHRONO = ['piste_officielle_obstacles', 'epingle', 'ovale_obstacles']
DUREE_MAX = 180.0
BRUIT = 0.5       # s : deux chronos d'un même pilote diffèrent de ~0,2 s (timing ROS 2)
# Domaines ROS 2 réservés aux mesures, confinés au PC (LOCALHOST) : un « covapsy sim » ouvert
# dans le domaine de l'équipe, ou un autre PC, ne peut pas fausser le chrono.
DOMAINE = {'chrono.launch.py': '99', 'duel.launch.py': '98'}


def ws():
    return os.environ.get('COVAPSY_STUDENT_WS') or os.getcwd()


def fichier_historique():
    return os.path.join(ws(), 'chronos.csv')


def lancer(launch, resultat, **args):
    """ros2 launch ... ; renvoie le JSON du chronometre (None si échec)."""
    env = dict(os.environ, ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
               ROS_DOMAIN_ID=DOMAINE[launch])                           # mesure isolée
    cmd = ['ros2', 'launch', 'covapsy_student_bringup', launch, f'resultat:={resultat}'] + \
        [f'{k}:={v}' for k, v in args.items()]
    journal = resultat + '.log'
    with open(journal, 'w') as f:
        subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT, env=env, timeout=900)
    if not os.path.isfile(resultat):
        print(f'  ÉCHEC du lancement : voir {journal}')
        with open(journal) as f:
            lignes = [l for l in f if 'Traceback' in l or 'Error' in l or 'ERROR' in l]
        print(''.join(lignes[:5]))
        return None
    with open(resultat) as f:
        return json.load(f)


def fmt(t):
    return '      -' if t is None else f'{t:7.2f}'


def chrono(circuits, note=''):
    print(f'Chrono officiel (ROS 2) : 3 tours par circuit, abandon au-delà de '
          f'{DUREE_MAX:.0f} s simulées. ~20 s par circuit.\n')
    print(f'{"circuit":28s} {"total":>7s} {"meilleur":>8s} {"chocs":>5s}  remarque')
    tmp = tempfile.mkdtemp(prefix='covapsy_chrono_')
    lignes, score = [], 0.0
    for c in circuits:
        r = lancer('chrono.launch.py', os.path.join(tmp, f'{c}.json'), circuit=c)
        if r is None:
            score += DUREE_MAX
            lignes.append({'circuit': c, 'total': f'{DUREE_MAX:.2f}', 'meilleur': '',
                           'chocs': '', 'fini': 0})
            continue
        x = r['classement'][0]
        total = x['total'] if x['total'] is not None else DUREE_MAX
        score += total
        remarque = '' if x['total'] is not None else f"ABANDON ({x['tours']}/3 tours)"
        print(f"{c:28s} {fmt(x['total'])} {fmt(x['meilleur']):>8s} {x['chocs']:5d}  {remarque}")
        lignes.append({'circuit': c, 'total': f'{total:.2f}',
                       'meilleur': f"{x['meilleur']:.2f}" if x['meilleur'] else '',
                       'chocs': x['chocs'], 'fini': int(x['total'] is not None)})
    print(f'\nSCORE : {score:.2f} s   (somme des temps ; plus petit = meilleur)')
    comparer(score, circuits)
    enregistrer(lignes, score, note)


def lire_historique():
    f = fichier_historique()
    if not os.path.isfile(f):
        return []
    with open(f, newline='') as fh:
        return list(csv.DictReader(fh, delimiter=';'))


def comparer(score, circuits):
    cle = '+'.join(circuits)
    prec = [float(h['score']) for h in lire_historique()
            if h.get('circuit') == 'SCORE' and h.get('circuits') == cle]
    if not prec:
        print('Premier chrono enregistré : il servira de référence.')
        return
    if abs(score - prec[-1]) < BRUIT:
        verdict = f'   (écart < {BRUIT} s : dans le bruit de mesure, pas significatif)'
    else:
        verdict = '   NOUVEAU RECORD !' if score < min(prec) - BRUIT else ''
    print(f'Précédent : {prec[-1]:.2f} s ({score - prec[-1]:+.2f} s)   record : '
          f'{min(prec):.2f} s ({score - min(prec):+.2f} s){verdict}')


def enregistrer(lignes, score, note):
    f = fichier_historique()
    champs = ['date', 'note', 'circuits', 'circuit', 'total', 'meilleur', 'chocs', 'fini',
              'score']
    nouveau = not os.path.isfile(f)
    base = {'date': datetime.datetime.now().strftime('%Y-%m-%d %H:%M'), 'note': note,
            'circuits': '+'.join(l['circuit'] for l in lignes)}
    with open(f, 'a', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=champs, delimiter=';')
        if nouveau:
            w.writeheader()
        for l in lignes:
            w.writerow({**base, **l, 'score': ''})
        w.writerow({**base, 'circuit': 'SCORE', 'score': f'{score:.2f}'})
    print(f'(enregistré dans {f})')


def historique():
    h = [x for x in lire_historique() if x.get('circuit') == 'SCORE']
    if not h:
        print('Aucun chrono enregistré : lancez « covapsy chrono ».')
        return
    record = min(float(x['score']) for x in h)
    print(f'{"date":17s} {"score":>8s}  note')
    for x in h:
        print(f'{x["date"]:17s} {float(x["score"]):8.2f}  {x["note"]}'
              + ('  <- record' if float(x['score']) == record else ''))


def duel(circuit, niveau, affichage):
    print(f'Duel sur « {circuit} » : votre pilote (car1) contre 3 adversaires « {niveau} ». '
          'Fenêtre : fermez-la pour abandonner.\n')
    res = os.path.join(tempfile.mkdtemp(prefix='covapsy_duel_'), 'duel.json')
    r = lancer('duel.launch.py', res, circuit=circuit, niveau=niveau,
               affichage='true' if affichage else 'false')
    if r is None:
        return
    print(f'{"#":>2} {"pilote":16s} {"tours":>5s} {"total":>7s} {"meilleur":>8s} {"chocs":>5s}')
    for x in r['classement']:
        print(f"{x['position']:2d} {x['nom']:16s} {x['tours']:5d} {fmt(x['total'])} "
              f"{fmt(x['meilleur']):>8s} {x['chocs']:5d}")
    rang = next(x['position'] for x in r['classement'] if x['voiture'] == 'car1')
    if rang == 1:
        print('\nVICTOIRE !')
        suivant = {'lent': 'moyen', 'moyen': 'rapide'}.get(niveau)
        if suivant:
            print(f'Niveau supérieur : covapsy duel {circuit} {suivant}')
    else:
        print(f'\nVous finissez {rang}e. Pour gagner : vitesse en virage, dépassement.')


def main(argv=None):
    ap = argparse.ArgumentParser(prog='covapsy', description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sp = ap.add_subparsers(dest='cmd', required=True)
    c = sp.add_parser('chrono')
    c.add_argument('circuits', nargs='*', default=CIRCUITS_CHRONO)
    c.add_argument('--note', default='')
    d = sp.add_parser('duel')
    d.add_argument('circuit', nargs='?', default='piste_officielle')
    d.add_argument('niveau', nargs='?', default='lent', choices=['lent', 'moyen', 'rapide'])
    d.add_argument('--sans-fenetre', action='store_true')
    sp.add_parser('historique')
    a = ap.parse_args(argv)
    if a.cmd == 'chrono':
        chrono(a.circuits, a.note)
    elif a.cmd == 'duel':
        duel(a.circuit, a.niveau, not a.sans_fenetre)
    else:
        historique()


if __name__ == '__main__':
    main()
