# Les 4 télémètres à ultrasons de la voiture dans UN seul message.
# Publié sur « ultrasons » directement par le simulateur (Python ou driver Webots), à
# 20 Hz. Les mêmes mesures existent aussi capteur par capteur sur us/front, us/rear,
# us/left et us/right (sensor_msgs/Range).
# Distances en mètres ; inf = rien de détecté dans la portée (portee_max).
std_msgs/Header header
float32 avant
float32 arriere
float32 gauche
float32 droite
float32 portee_min     # m, en dessous la mesure n'est pas fiable
float32 portee_max     # m (2 m sur la TT02)
