# =====================================================================================
#  Image CoVAPSy : ROS 2 Jazzy, Webots, simulateur Python, outils du projet.
#  Base : docker.io/estadrone2020/covapsy-jazzy:2026.1 (image utilisée par tout le projet).
#
#  Construite automatiquement par le Dev Container, ou à la main depuis la racine :
#      docker build -t covapsy .devcontainer/
#
#  Proxy d'école : déposez ses certificats .crt dans .devcontainer/certificats/ (facultatif).
# =====================================================================================
FROM docker.io/estadrone2020/covapsy-jazzy:2026.1

# L'image de base tourne en tant que « ubuntu » : root le temps d'installer les certificats
USER root
RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates \
    && rm -rf /var/lib/apt/lists/*
COPY certificats/ /usr/local/share/ca-certificates/covapsy/
RUN update-ca-certificates

# Revenir à l'utilisateur par défaut de l'image (uid 1000)
USER ubuntu
CMD ["bash"]
