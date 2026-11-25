# Environnement de l'équipe : à « sourcer » (fait automatiquement par les terminaux VS Code).
#     source student_ws/setup_equipe.bash
# Charge ROS 2, le workspace compilé et fixe ROS_DOMAIN_ID d'après equipe.yaml.
_ici="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export COVAPSY_STUDENT_WS="$_ici"
source /opt/ros/jazzy/setup.bash
[ -f "$_ici/install/setup.bash" ] && source "$_ici/install/setup.bash"
_dom=$(python3 -c "import yaml;print(yaml.safe_load(open('$_ici/equipe.yaml')).get('domaine',''))" 2>/dev/null)
if [ -n "$_dom" ]; then export ROS_DOMAIN_ID="$_dom"; fi
export PATH="$_ici:$PATH"
echo "CoVAPSy équipe : ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}  (commande d'aide : covapsy aide)"
unset _ici _dom
