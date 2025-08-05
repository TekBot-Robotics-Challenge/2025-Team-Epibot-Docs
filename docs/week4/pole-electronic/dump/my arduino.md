
---

Ce circuit est une reproduction fonctionnelle d’une carte Arduino Uno, conçue autour du microcontrôleur **ATmega328P-P**, avec une gestion complète de l’alimentation et des interfaces nécessaires à son utilisation autonome et à la programmation.

### 1. Microcontrôleur et horloge

Le microcontrôleur **ATmega328P** constitue le cœur de cette carte. Il est cadencé par un **oscillateur à quartz de 16 MHz**, connecté aux broches XTAL1 et XTAL2, avec deux **condensateurs céramiques de 22 pF** assurant la stabilité du signal d’horloge. Ce montage garantit un fonctionnement fiable conforme aux spécifications de l’ATmega328P.

### 2. Circuit de réinitialisation

Un **bouton poussoir** relié à la broche **RESET** permet de redémarrer manuellement le microcontrôleur. Une **résistance de pull-up de 10 kΩ** maintient la broche RESET à l’état logique haut pour éviter les redémarrages intempestifs. Lors de l’appui sur le bouton, la broche est reliée à la masse, entraînant une réinitialisation contrôlée du système.

### 3. Alimentation et distribution des tensions

La carte est alimentée via un **bornier à 2 broches**, permettant la connexion d’une source externe de **9V**. Une **diode 1N4007** protège contre les inversions de polarité en entrée.
Un **régulateur linéaire LM7809** fournit une tension **stabilisée de 9V**, utilisée directement par certains modules et disponible en sortie via un **bornier dédié**.

Pour obtenir la **tension 5V** nécessaire au fonctionnement du microcontrôleur et de ses périphériques, le circuit utilise une **diode Zener 1N4733A de 5,1V** montée de manière à stabiliser la tension sur la ligne 5V. Ce choix permet de simplifier le circuit en éliminant l’utilisation d’un régulateur 7805, tout en maintenant une tension proche de 5V suffisante pour alimenter le cœur du système et les composants compatibles.

Des **condensateurs électrolytiques de 220 µF** sont placés en entrée et en sortie du LM7809, ainsi qu’en parallèle avec la diode Zener, pour lisser la tension et filtrer les parasites.

Deux **LEDs d’indication** permettent de visualiser l’état de l’alimentation :

* Une **LED verte 3V** en série avec une **résistance de 220 Ω** indique la présence de la tension 5V.
* Une **LED rouge 3V** en série avec une **résistance de 330 Ω** signale la présence de la tension 9V.

### 4. Programmation via USB (CH340G)

Le circuit intègre un convertisseur USB–série **CH340G**, permettant la programmation de l’ATmega328P via une connexion USB standard.

* Les broches **TX** et **RX** du CH340G sont reliées respectivement aux broches **RX (PD0)** et **TX (PD1)** du microcontrôleur.
* L’alimentation 5V est partagée entre le CH340G et le reste du circuit.
* Un **connecteur femelle 6 broches** est présent pour faciliter le branchement à un câble USB–TTL ou pour la reprogrammation directe.

### 5. Connectique d’extension

Deux **connecteurs femelles de 14 broches** exposent l’ensemble des **entrées/sorties numériques D0 à D13** ainsi que les **entrées analogiques A0 à A5**, conformément à l’architecture de l’Arduino Uno.
Un **connecteur femelle 6 broches supplémentaires** fournit des points d’accès pour la programmation ou la communication série.

### 6. Sorties d’alimentation disponibles

La carte propose deux sorties de tension accessibles via des borniers :

* Une **sortie 9V**, directement en aval du LM7809, pour alimenter des modules nécessitant une tension plus élevée.
* Une **sortie 5V**, régulée par la diode Zener 5,1V, destinée à l’alimentation des modules sensibles ou logiques compatibles 5V.

### 7. Fonction d’ensemble

Ce circuit constitue une plateforme complète, inspirée de l’Arduino Uno, permettant l’exécution de programmes embarqués et l’interfaçage avec différents modules externes. Grâce à l’intégration de composants discrets pour la régulation de tension et l’alimentation visuelle, cette carte reste simple à produire tout en étant pleinement fonctionnelle. Elle est compatible avec des modules tels que le **laser KY-008**, la **photoresistance réglable**, le **capteur de couleur TCS34725**, et le **driver moteur A4988**, tous conçus sur des PCB dédiés et alimentés par les sorties 5V ou 9V de la carte.

---
