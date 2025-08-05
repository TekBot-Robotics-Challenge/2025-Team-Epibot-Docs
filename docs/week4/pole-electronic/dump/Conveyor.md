
---

## 🔹 1. **PCB du module laser KY-008**

Ce circuit est conçu pour accueillir et piloter le **module laser KY-008**.

* Le **module laser** est directement inséré dans un **connecteur femelle à 3 broches**, prévu pour l’alimenter via VCC, GND et signal.
* Une **LED verte 3V** en série avec une **résistance de 220 ohms** est intégrée sur la carte comme témoin de mise sous tension du module.
* Un **bornier à 3 broches** est présent pour l’alimentation externe ou la commande depuis une carte maître (par exemple une Arduino). Ce bornier facilite le raccordement du VCC, GND et du signal d’activation.

Ce PCB sert donc d’**interface entre le laser KY-008 et le reste du système**, tout en offrant une indication visuelle de son état.

---

## 🔹 2. **PCB du module à photoresistance**

Ce PCB permet de mesurer la lumière ambiante et d’ajuster le seuil de sensibilité.

* Une **photoresistance (R4)** est insérée dans un support (pads directs).
* Un **potentiomètre de 10 kΩ** permet de régler la sensibilité du capteur (RV1).
* Deux **résistances** : 10 kΩ (R5) pour former un pont diviseur avec la photoresistance, et 220 ohms (R6) pour la LED.
* Une **LED verte 3V** (D1) sert de témoin lumineux indiquant l’activation du circuit.
* Deux **borniers à 2 broches** permettent de connecter le circuit à l’alimentation (VCC/GND) et de récupérer la tension de sortie analogique.

Les composants sont montés en direct ou via headers si besoin. Ce PCB permet un **réglage manuel du niveau de détection lumineuse** et une **lecture analogique stable** à partir de la tension issue du pont diviseur.

---

## 🔹 3. **PCB du capteur de couleur TCS34725**

Ce PCB est une carte d’accueil pour le capteur de couleur **TCS34725**, qui fonctionne en I2C.

* Le capteur est monté via un **header femelle à 7 broches** (J3), connecté aux lignes SDA, SCL, VCC, GND, INT, LED, NC.
* Une **LED verte 3V**, montée avec une **résistance de 220 ohms**, fournit un **éclairage d’appoint constant** afin d’assurer des mesures stables.
* Deux **borniers à 2 broches** sont disponibles :

  * L’un pour l’alimentation du module (VCC/GND),
  * L’autre pour exposer les lignes I2C ou récupérer des signaux.

Ce PCB facilite **l’intégration du capteur TCS34725**, son alimentation, et sa communication I2C avec un microcontrôleur externe, tout en assurant une visibilité correcte de la surface à analyser grâce à la LED intégrée.

---

## 🔹 4. **PCB du driver moteur pas-à-pas A4988**

Ce PCB est destiné à accueillir un **driver A4988** (module Pololu) pour contrôler un **moteur pas-à-pas NEMA 17**. Il est conçu pour faciliter les connexions d’alimentation, de commande et du moteur.

### 🧩 Composants présents :

* **1 LED verte 3V** (D4) avec une **résistance de 220 ohms** (R2) servant de **témoin de présence d'alimentation logique**.

* **3 borniers à 2 broches** :

  * **Bornier 1 (VDD/GND)** : pour **l’alimentation logique** du A4988 (5V).
  * **Bornier 2 (VMOT/GND)** : pour **l’alimentation du moteur** (ex. 9V ou 12V).
  * **Bornier 3 (SIGNAL)** : pour une éventuelle **réception de signal de contrôle ou d’état** (ex. RESET/SLEEP), ou pour usage personnalisé selon le câblage du projet.
* **1 header pin mâle à 4 broches** :

  * Ce connecteur permet la **connexion directe au moteur NEMA 17**, avec les broches **1A, 1B, 2A, 2B**.

### 🧰 Fonctionnement :

* Le **driver A4988** est monté via un **connecteur femelle double rangée** (non représenté ici mais généralement 2x8 broches).
* L’alimentation **logique** (5V) et **moteur** (9V ou 12V) sont séparées pour garantir un fonctionnement stable.
* Les signaux de commande (STEP, DIR) sont envoyés par un bornier a deux bornes**.
* Le moteur NEMA est directement connecté au **header pin mâle 4 broches**, pour un câblage simple et clair.

---

