uint8 WAITING=0        # sur la grille, attend race/start
uint8 RACING=1         # conduite autonome
uint8 REVERSING=2      # manœuvre de dégagement
uint8 STOPPED=3        # arrêt demandé par race/stop

std_msgs/Header header
uint8 state
string reason          # cause de la dernière transition, pour la télémétrie
