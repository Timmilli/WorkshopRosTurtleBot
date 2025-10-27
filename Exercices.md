# Exercices
Merci de réaliser les exercices dans l’ordre proposé. À la fin de la journée, veuillez m’envoyer votre dossier `src` compressé (format zip) à l’adresse <lucas.joseph@inria.fr>. N’oubliez pas d’ajouter un fichier README.md expliquant comment exécuter vos codes. Pour faciliter l’organisation, nommez vos fichiers en fonction des exercices correspondants.

## 1. Hello Publisher/Subscriber

1. Écrire un nœud publisher qui envoie deux informations :
    - Le nom de l’ordinateur (string)
    - Le domain_id (int)
2. Écrire un nœud subscriber qui les reçoit et les affiche.

## 2. Publication avec un timer

Modifier le publisher pour qu’il envoie les données à 1 Hz grâce à un timer ROS2.

## 3. Ajouter un compteur

Ajouter un champ compteur dans le message, qui s’incrémente à chaque publication.

## 4. Définir un message personnalisé

1. Créer ComputerInfo.msg contenant :
    string computer_name
    int32 domain_id
    int32 counter
2. Utiliser ce message dans le publisher et le subscriber.

## 5. Plusieurs subscribers

1. Un subscriber affiche le message tel quel.
2. Un autre le transforme (par ex. : mettre le nom de l’ordinateur en majuscules, ajouter +1 au domain_id).

## 6. Paramètres dans un node

1. Ajouter deux paramètres :
    - correct_computer_name (string)
    - correct_domain_id (int)
2. Afficher leur valeur au lancement

## 7. Service client/serveur

- Client demande : “Est-ce le bon ordinateur et le bon domain_id ?”
- Serveur répond : True/False.

## 8. Launch file simple

Écrire un launch file qui démarre le publisher et le subscriber.

## 9. Launch file avec paramètres

Passer correct_computer_name et correct_domain_id en paramètres via le launch file.