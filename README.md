# La voiture des enfers

Dans les grandes lignes, la voiture va chercher à se connecter à un réseau WiFi existant.

Une fois connectée, elle va prendre ses ordres en WebSocket et renvoyer sa télémétrie en MQTT.

Tout le code est entièrement OpenSource, il est donc possible (et même conseillé) d'aller le modifier pour pouvoir l'adapter à vos besoins.

## Quick start

Ce projet est fait pour fonctionner avec [PlatformIO](https://platformio.org/). C'est un plugin VS Code qui va vour permettre de remplacer le traditionnel Arduino IDE au profit d'un framework embarquant une structure plus classique de projet C++ avec un fichier de configuration qui emporte toutes les informations et dépendances de votre projet.

Pour uploader le code sur votre voiture, il vous suffit de brancher votre ESP32 en USB et de cliquer sur le bouton "upload".

## Adapter quelques éléments

Il faudra au moins adapter quelques éléments du code pour rentrer les crédentials de votre WiFi, l'ip que vous allez donner à la voiture, la gateway et le netmask de votre réseau.
Dans `src/main.cpp` vous allez modifier les lignes suivantes:

```cpp
// ...

char *ssid_wifi = "OnePlus 7 Pro";    // Le nom du réseau WiFi
char *password_wifi = "33500a9695df"; // Le password du WiFi

const char *mqtt_server = "192.168.145.40"; // L'IP de votre broker MQTT
const int mqtt_interval_ms = 5000;          // L'interval en ms entre deux envois de données

IPAddress localIP(192, 168, 145, 49); // l'IP que vous voulez donner à votre voiture

IPAddress localGateway(192, 168, 145, 55); // L'IP de la gateway de votre réseau
IPAddress localSubnet(255, 255, 255, 0);   // Le masque de sous réseau

// ...
AsyncWebSocket ws("/ws"); // Changez le nom de ce point d'accès pour "sécuriser" l'accès à votre voiture
```

## Communiquer avec la voiture

Les ordre envoyés à la voiture se font via WebSocket à l'addresse suivante : `ws://<CAR_IP_ADDRESS>/ws`

L'intégralité des ordres seront des JSON avec deux clés:

-   cmd: l'id du type d'ordre que vous voulez passer
-   data: la donnée associée à la commande

Par exemple, si vous voulez changer la vitesse des roues

```json
{
	"cmd": 1,
	"data": [2000, 2000, 2000, 2000]
}
```

Ici, data est un array où chaque valeur correspond à la vitesse d'une roue

On pourrait aussi avoir l'exemple suivant pour assigner le "visage" 3 sur la voiture:

```json
{
	"cmd": 2,
	"data": 3
}
```

## Liste des commandes

| CMD NAME                   | CMD ID | DATA TYPE | MISC                                                                                                                                                                                                                                                                      |
| -------------------------- | ------ | --------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Roues                      | 1      | Array[4]  | Les valeurs doivent être comprises entre -4095 et 4095                                                                                                                                                                                                                    |
| Visage                     | 2      | int       | Chaque valeur correspond à un visage différent, passé 7 les visages sont statiques et aléatoires                                                                                                                                                                          |
| Angle de la tête           | 3      | Array[2]  | Valeurs comprises entre 0 et 180, la première correspond à l'angle vertical, la seconde à l'angle horizontal                                                                                                                                                              |
| Animation des LED          | 4      | int       | Chaque valeur correspond à une animation. Les valeurs sont entre 0 et 5                                                                                                                                                                                                   |
| Couleur principale des LED | 5      | Array[4]  | La première valeur est la représentation binaire d'un bitmask identifiant la LED, les 3 valeurs suivantes sont des valeurs RGB entre 0 et 255                                                                                                                             |
| Couleur secondaire des LED | 6      | Array[4]  | La première valeur est la représentation binaire d'un bitmask identifiant la LED, les 3 valeurs suivantes sont des valeurs RGB entre 0 et 255. Si la première valeur est 0, le changement affectera toutes les LED. Ce réglage n'est visible que sur certaines animations |
| Buzzer Alarm               | 7      | int       | Off: 0, On: 1. Déclenche le buzzer d'alarme                                                                                                                                                                                                                               |
| Buzzer Variable            | 8      | Array[2]  | La première valeur est un ON/OFF (1/0), la seconde est la fréquence du son (entre 0 et 10000Hz)                                                                                                                                                                           |
| Video Activation           | 9      | int       | Off: 0, On: 1. Active ou désactive le flux video                                                                                                                                                                                                                          |

### Exemples

#### Allumer une LED en rouge

On doit d'abbord lancer une animation de LED, 1 est un allumage statique continu

```json
{
	"cmd": 4,
	"data": 1
}
```

Je vais ensuite allumer les LED 1, 2, 3 et 4 en rouge

```json
{
	"cmd": 5,
	"data": [1, 255, 0, 0]
}
```

```json
{
	"cmd": 5,
	"data": [2, 255, 0, 0]
}
```

```json
{
	"cmd": 5,
	"data": [4, 255, 0, 0]
}
```

```json
{
	"cmd": 5,
	"data": [8, 255, 0, 0]
}
```
