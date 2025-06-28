D

le defis de la 3eme semaine consiste a realiser un
afficheur 7 segement a base de m7 servomoteurs et les
piloter dans la bonne configuration pour representer 
es chiffres de 0 a 9.
pour ce faire nous avons d'abord fait la collecte des
elements essentiels du circuit.
le circuit etant composee du module PCA9685, de 7 servomoteur
moteurs, d'une diode, de 3 resistance, d'une atmega 328p de 2
led rouge et verte, de la LM 7809, d'un crystal
quartz, de deux condensateur de 22uf eb ceramic et
d'un condensateur de 16v. Les sorties de la source de tension sont extensibles a 
d'autres equipements electroniques pour leur propre alimentation.

pour la realisation du projet, nous avons realiser le schema
kicad du projet, dans ce schema, nous avons 2 grande partie,
la realisation d'une mini copie de la carte arduino elle meme et ensuite 
la realisation du brancement du module PCA9685 aux 7 servo
moteurs.

Dans le circuit de la mini carte arduino nous avons l'utilisation de la atmega pour l'execution
et compilation du code. ce composant est alimente a base d'une
d'une source de tension de 5v obtenue par le biais du dispositif
de reglage de tension incorpore dans sur la reproduction de la
carte. ce dispositif est composee de la lm 7809 qui recoit une tension de
14,8v a ses bornes et permet d'obtenir de 9V. cde 9v est
travailler au niveau de la diode zener pour obtenir une 
tension de 5v. ainsi ce dispositif permet d'avoir deux sortie de tension
celle de 5v et celle de 9v. des voyant lumineux sont presentes pour la 
reconnaissances des differentes sources de tension. verte pour le 5v et rouge pour le 9v.
maintenant la atmega 328p alimentee par cette tension de 5v est relie au Quartz crystal qui va lui permettre
de candenser l'execution des commandes avec l'aides dees condensateurs de 22uf
a ce montage est present un bouton poussoir permentant de reset la atmega pour restart son programme internes.
ce premier circuit offres l'acces a toutes les broches de la atmega grace au headers pin comme si nous utilisons la vrai carte arduino

A la notre mini reproduction de la carte arduino nous avons la pca 9685 qui est reliee a la masse du circuit, sa SCL reliee a la sortie A5, sa sDA reliee a la A4,
sa vcc reliee a la source de 5v , sa v++ reliee a la source 9v.
les differents servo moteurs sont tous reliee sur la Pca 9685 grace au pin d'alimentation v++, gnd et la pin de controle de l'information.