#!/bin/bash
# =====================================================================================
#  Lance le conteneur CoVAPSy SANS VS Code (Podman ou Docker).
#
#    ./conteneur.sh                 terminal interactif dans le conteneur
#    ./conteneur.sh covapsy test    exécute une commande puis quitte
#
#  1re fois : construit l'image (.devcontainer/Dockerfile ou docker/Dockerfile, base
#  covapsy-jazzy) puis compile le
#  workspace (quelques minutes). Le dossier du dépôt est partagé avec le conteneur : vos
#  fichiers restent sur votre PC. Réseau de l'hôte (--network host) pour le mode distant.
#  Variables : MOTEUR=podman|docker, IMAGE=nom de l'image (défaut covapsy-projet:local).
# =====================================================================================
set -e
ICI="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
NOM="$(basename "$ICI")"
DEST="/workspaces/$NOM"
MOTEUR=${MOTEUR:-$(command -v podman > /dev/null && echo podman || echo docker)}
IMAGE=${IMAGE:-covapsy-projet:local}

if ! $MOTEUR image inspect "$IMAGE" > /dev/null 2>&1; then
    echo "[conteneur] construction de l'image $IMAGE (une seule fois)..."
    if [ -f "$ICI/.devcontainer/Dockerfile" ]; then      # dépôt équipe : Dockerfile du Dev Container
        $MOTEUR build -t "$IMAGE" "$ICI/.devcontainer"
    else
        $MOTEUR build -f "$ICI/docker/Dockerfile" -t "$IMAGE" "$ICI"
    fi
fi

OPTS=(--rm --network host -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw
      -v "$ICI:$DEST" -w "$DEST")
[ "$MOTEUR" = podman ] && OPTS+=(--userns=keep-id)
[ -t 0 ] && OPTS+=(-it)
command -v xhost > /dev/null && xhost +local: > /dev/null 2>&1 || true   # fenêtres graphiques

if [ -d "$ICI/teacher_ws" ]; then          # dépôt ENSEIGNANT
    COMPILER="[ -f teacher_ws/install/setup.bash ] || bash scripts/installer_enseignant.sh"
    ENV="export COVAPSY_RACINE=$DEST; source $DEST/scripts/enseignant.bash; cd $DEST/teacher_ws"
else                                       # dépôt ÉQUIPE
    COMPILER="[ -f student_ws/install/setup.bash ] || bash student_ws/installer.sh"
    ENV="source $DEST/student_ws/setup_equipe.bash; cd $DEST/student_ws"
fi

if [ $# -gt 0 ]; then                      # commande unique
    exec $MOTEUR run "${OPTS[@]}" "$IMAGE" bash -c "$COMPILER && $ENV && $*"
fi
# terminal interactif : même environnement que dans VS Code
exec $MOTEUR run "${OPTS[@]}" "$IMAGE" bash -c "$COMPILER && \
    echo 'source ~/.bashrc; $ENV' > /tmp/covapsy_rc && exec bash --rcfile /tmp/covapsy_rc"
