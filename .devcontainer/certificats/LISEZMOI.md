# Certificats du proxy (facultatif)

Si votre réseau passe par un proxy qui intercepte le HTTPS (réseau d'école), copiez ici
ses certificats racine au format `.crt`, puis reconstruisez le conteneur
(VS Code : « Dev Containers: Rebuild Container »). Sans proxy, laissez ce dossier tel quel.

Ne versionnez pas de certificat privé : `*.crt` est ignoré par git.
