#!/bin/bash
# Installation de l'environnement de l'équipe (lancé automatiquement par le Dev Container,
# ou à la main une fois : bash student_ws/installer.sh). Idempotent.
set -e
ICI="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/jazzy/setup.bash
cd "$ICI" && colcon build --symlink-install
LIGNE="source $ICI/setup_equipe.bash"
grep -qxF "$LIGNE" ~/.bashrc 2>/dev/null || echo "$LIGNE" >> ~/.bashrc
echo "Installation terminée. Ouvrez un nouveau terminal puis tapez : covapsy aide"
