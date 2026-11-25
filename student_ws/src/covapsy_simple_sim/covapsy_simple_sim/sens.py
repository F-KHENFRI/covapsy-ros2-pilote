"""
Sens de circulation d'après les couleurs des bordures (service « perception/sens_circuit »).

Module pur (sans ROS), commun au simulateur Python (caméra simulée) et à
wall_color_detector sous Webots (vraie image) : les deux rendent le même service.

Entrée : à chaque image, les proportions de bordure verte / rouge vues dans les moitiés
gauche et droite de l'image (0..1). Elles sont moyennées sur ``tau`` secondes : une image
isolée (mur vu de face dans un virage serré) est peu informative.
Requête : couleur attendue à droite et à gauche DANS LE BON SENS (« vert » / « rouge »).
Score : (attendue à droite + autre à gauche) - (l'inverse), dans [-2, 2].
"""
CHAMPS = ('gauche_vert', 'gauche_rouge', 'droite_vert', 'droite_rouge')
NOMS = {'vert': 'vert', 'green': 'vert', 'rouge': 'rouge', 'red': 'rouge'}


class EstimateurSens:

    def __init__(self, tau=1.0, seuil=0.08, age_max=1.0):
        self.tau, self.seuil, self.age_max = tau, seuil, age_max
        self.moyenne = None                  # {champ: proportion moyennée}
        self.t = None                        # instant de la dernière image (s)

    def reinitialiser(self):
        self.moyenne = self.t = None

    def maj(self, proportions, t):
        """Nouvelle image : ``proportions`` = {gauche_vert, gauche_rouge, droite_vert,
        droite_rouge}, ``t`` en secondes (temps simulé)."""
        valeurs = {c: float(proportions[c]) for c in CHAMPS}
        if self.moyenne is None or self.t is None or t < self.t:
            self.moyenne = valeurs
        else:
            alpha = 1.0 if self.tau <= 0 else min(1.0, (t - self.t) / self.tau)
            for c in CHAMPS:
                self.moyenne[c] += alpha * (valeurs[c] - self.moyenne[c])
        self.t = t

    def repondre(self, couleur_droite, couleur_gauche, t):
        """(success, sens, score, message) ; sens : 1 bon sens, -1 contresens, 0 indécis."""
        droite = NOMS.get(str(couleur_droite).strip().lower())
        gauche = NOMS.get(str(couleur_gauche).strip().lower())
        if droite is None or gauche is None or droite == gauche:
            return (False, 0, 0.0, 'couleurs invalides : donnez « vert » et « rouge », une de '
                                   'chaque côté (ex. couleur_droite: vert, couleur_gauche: rouge)')
        if self.moyenne is None or t - self.t > self.age_max:
            return False, 0, 0.0, 'aucune image récente de la caméra'
        m = self.moyenne
        score = (m[f'droite_{droite}'] + m[f'gauche_{gauche}']
                 - m[f'droite_{gauche}'] - m[f'gauche_{droite}'])
        if score > self.seuil:
            sens, texte = 1, 'bon sens'
        elif score < -self.seuil:
            sens, texte = -1, 'CONTRESENS'
        else:
            sens, texte = 0, 'indécis : bordures peu visibles'
        texte += (f' (vu à gauche : vert {m["gauche_vert"]:.2f} rouge {m["gauche_rouge"]:.2f} ; '
                  f'à droite : vert {m["droite_vert"]:.2f} rouge {m["droite_rouge"]:.2f})')
        return True, sens, float(score), texte
