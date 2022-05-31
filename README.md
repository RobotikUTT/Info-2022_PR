# Info-2022_PR

Code Arduino pour Petit Robot de la Coupe de France De Robotique 2022.
Les codes du côté jaune et violet n'ont pas été assemblé pour la Mega.

## Installation

Cloner le Git avec ses sous-modules (automatique avec GitHub Desktop) :
```sh
git clone --recurse-submodules <git>
```
Ajouter des sous-modules dans le git qui ont été rajoutés sur github:
```sh
git pull --recurse-submodules <git>
```

(<git>= lien du repo en *https://* ou *git@*)

___

## Répertoires

Le Git est constitué des répertoires suivants:
___
### Code_Mega/
Contient le code pour l'Arduino Mega chargée du déplacement et du système d'action. Elle gère également les actionneurs.

___
### Code_Nano/
Contient le code pour l'Arduino Nano chargée de scruter en continu les retours des sonars et de communiquer le résultat à la Mega.
