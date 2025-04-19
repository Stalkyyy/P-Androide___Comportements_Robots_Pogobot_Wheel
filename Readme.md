Projet Androide 2024-25 : Comportements pour le robot Pogobot-Wheel 
=======

Table of contents
-----------------

- [Projet Androide 2024-25](#projet-androide-2024-25--comportements-pour-le-robot-pogobot-wheel)
  - [Table of contents](#table-of-contents)
- [Présentation](#présentation)
- [Dépendences](#dépendances)
- [Compilation](#compilation)



Présentation
============

L'ISIR vient de terminer la construction de 20 robots Pogobot à roues, des robots de 6 cm de diamètre, capable d'echanger des informations avec leurs voisins immédiats, et de se déplacer à une vitesse d'environ 20cm/sec. L'objectif de ce projet est de programmer ces robots pour réaliser plusieurs tâches typique de la robotique autonome et collective. Pour chaque tache suivante, entre 10 et 20 robots sont placés dans une arène :

- **déplacement au hasard** : Chaque robot se déplace en ligne droite lorsqu'il n'y a pas d'obstacle (robot ou mur) et change de direction dans le cas contraire.
- **suivi de leader** : les robots sont placés dans une arène en file indienne. Un robot suit un comportement de déplacement au hasard, les autres robots suivent leur prédécesseur.
- **déplacement en formation** : chaque robot se déplace en ligne droite en l'absence de voisin et évite les murs si besoin. En présence d'un voisin, un robot modifie sa direction pour s'aligner sur celle de son (ses) voisin(s).
- **alignement face au mur** : les robots se rapprochent des murs et s'arrêtent. L'état final est d'obtenir un rangement en ligne des robots face au mur.
- **phototaxie** : les robots se déplacent vers la source lumineuse.
- **dispersion** : les pogobots s'écartent les uns des autres pour couvrir au mieux l'environnement.

Afin de réaliser ces comportements, le groupe (2 à 4 personnes) devra prendre en main le Pogobot, programmable en langage C avec une API dédiée (cf. https://pogobot.github.io/), et adapter le protocole de communication existant pour estimer la distance entre robots (en particulier pour les comportements de suivi, formation, alignement et dispersion). Les étudiant·es auront accès à l'arène multi-robots de l'ISIR et l'ensemble du développement et des démos devra être réalisé sur robots réels. Un serveur Mattermost permettra de communiquer avec l'équipe de l'ISIR travaillant avec le Pogobot.

**Encadrant** : Nicolas Bredeche

**Etudiants affectés** :
- Safia EL KHOUMSI
- Mélanie DELLUC
- Adel BEN SALAH
- Enzo PINHO FERNANDES



Dépendances
===========

Vous devez créer un répertoire `dependencies` à la racine du projet, et y aller.

    mkdir -p dependencies
    cd dependencies

A l'intérieur, on y placera les projets [pogobot-sdk](https://github.com/nekonaute/pogobot-sdk.git) et [pogosim](https://github.com/Adacoma/pogosim.git). Référez-vous à leur page github pour leur installation ainsi que leur dépendances.

Vous pouvez vous référez au readme de [pogosim](https://github.com/Adacoma/pogosim.git) pour voir comment compiler chaque comportement, que ce soit la partie simulation ou physique.

Si vous ne souhaitez pas les placer dans le répertoire `dependencies`, n'oubliez pas de modifier les variables suivantes des Makefiles de chaque comportement de robot.

    export POGO_SDK=/ABSOLUTE/PATH/TO/pogobot-sdk
    export POGOSIM_INCLUDE_DIR=/ABSOLUTE/PATH/TO/pogosim/src
    export POGOUTILS_INCLUDE_DIR=/ABSOLUTE/PATH/TO/pogo-utils/src



Compilation
===========

Pour compiler un des comportements des robots, allez dans leur répertoire et tapez une de ces trois commandes selon votre choix.

    cd dispersion   # Exemple de comportement robots

    make clean sim  # Pour compiler la simulation
    # ou bien...
    make clean bin  # Pour compiler le binaire pour les vrais pogobots.
    # ou bien...
    make clean all  # Pour compiler la simulation et le binaire des pogobots.