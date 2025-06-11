```markdown
# IoT Project: Lijnvolgende Robot met Obstakelontwijking 🤖

Dit project omvat de ontwikkeling van een autonome lijnvolgende robot. De robot is gebouwd op een ESP32-microcontroller, kan zelfstandig obstakels ontwijken en communiceert zijn status en sensordata in real-time via MQTT. Een Node-RED dashboard visualiseert deze data voor eenvoudige monitoring.

## Inhoudsopgave
1.  [GitHub Repository](#1-github-repository)
2.  [Gebruikte Hardware](#2-gebruikte-hardware-)
3.  [Hardware-opstelling](#3-hardware-opstelling-)
4.  [Code Overzicht](#4-code-overzicht-)
5.  [MQTT-communicatie](#5-mqtt-instellingen-)
6.  [Web Dashboard](#6-web-dashboard-)
7.  [Installatie en Testen](#7-installatie-instructies-)
8.  [Zelf Nabouwen](#8-namaak-instructies-)

---

### 1. GitHub Repository
De volledige broncode en Gerber-files zijn te vinden op GitHub:
[https://github.com/stijnmaurissens/IoT-Project-Linefollow-robot](https://github.com/stijnmaurissens/IoT-Project-Linefollow-robot)

---

### 2. Gebruikte Hardware ⚙️

Een overzicht van de essentiële componenten die nodig zijn voor dit project:

* **ESP32 Dev Board**: Het brein van de robot. Deze microcontroller bestuurt alle componenten, verwerkt de logica en regelt de Wi-Fi-verbinding en MQTT-communicatie.
* **Raspberry Pi 5**: Host voor de backend-software: MQTT-broker, InfluxDB database en Node-RED.
* **3x Infrarood (IR) Sensoren**: Worden gebruikt voor het detecteren en volgen van een zwarte lijn.
* **3x Ultrasone Sensoren (HC-SR04)**: Voor het detecteren van obstakels.
* **2x DC Motoren**: Drijven de wielen van de robot aan.
* **1x Drukknop**: Maakt het mogelijk om de robot handmatig te pauzeren en te hervatten.
* **9V Batterij**: Voorziet de ESP32 en de motoren van stroom.
* **Zelf ontworpen PCB**: Een custom printplaat om alle componenten netjes en betrouwbaar te verbinden.

---

### 3. Hardware-opstelling 🔌

Voor de precieze verbindingen tussen de ESP32, sensoren, motoren en andere componenten, zie het schema in de projectbestanden.

*[Voeg hier een afbeelding van het Fritzing- of bedradingsschema in]*

---

### 4. Code Overzicht 👨‍💻

De code is geschreven in C++ voor het Arduino-framework op de ESP32.

#### 4.1. Belangrijkste Functies (Lijnvolging)

Het lijnvolgsysteem is de kernfunctionaliteit. Als alleen de middelste sensor de lijn ziet, rijdt de robot rechtdoor. De lijn detecteren doet hij met de volgende code:

if (irSensorWaarden[0] == LOW && irSensorWaarden[1] == HIGH && irSensorWaarden[2] == LOW) {
    rijVooruit(110);
    Serial.println("vooruit");
}

Waarbij onze functie `rijVooruit` gebruikt wordt met de parameter van 110. Deze parameter wordt gebruikt in de volgende code om de snelheid te bepalen, waarbij beide motoren vooruit rijden.

void rijVooruit(int snelheid) {
    digitalWrite(motorRechts_pin1, LOW);
    digitalWrite(motorRechts_pin2, HIGH); 
    analogWrite(motorRechts_enable, snelheid);
    
    digitalWrite(motorLinks_pin1, HIGH);
    digitalWrite(motorLinks_pin2, LOW); 
    analogWrite(motorLinks_enable, snelheid);
}

Om de robot te laten draaien, passen we dezelfde logica toe als voor het vooruit rijden, maar keren we de draairichting van één van de wielen om.

#### 4.2. Obstakelontwijking

De robot kan zelfstandig obstakels ontwijken. Ziet hij een obstakel, dan start hij een manoeuvre om eromheen te rijden en zoekt daarna de lijn weer op.

##### **Bekend Probleem & Mogelijke Verbetering** ⚠️
* **Het probleem**: De robot draait op basis van **tijd**, niet op basis van een gemeten hoek. Dit is onnauwkeurig en sterk afhankelijk van de batterijspanning en de ondergrond.
* **De oplossing**: Implementeer een **gyroscoop/accelerometer (bv. MPU-6050)**. Hiermee kan de robot zijn rotatie nauwkeurig meten en draaien tot een exacte hoek van 90 graden is bereikt. Dit zou de betrouwbaarheid aanzienlijk verbeteren.

#### 4.3. Dual-Core Verwerking (ESP32)

De ESP32 heeft twee processorkernen, wat ons toelaat om taken parallel uit te voeren voor betere prestaties.

* **Core 0**: Draait een toegewijde taak voor netwerkcommunicatie.
    * Onderhoudt de MQTT-verbinding.
    * Probeert automatisch opnieuw te verbinden als de connectie wegvalt.
    * Verwerkt de input van de fysieke pauzeknop.

* **Core 1**: Draait de standaard `setup()` en `loop()` functies.
    * Voert de hoofdlogica van de robot uit: lijnvolging en het aanroepen van de obstakelontwijking.
    * Reageert op de pauzestatus door de motoren te stoppen.

---

### 5. MQTT-instellingen 📡

De robot communiceert via de volgende MQTT-topics:

* **esp32/status**
    * **Richting**: Robot → Dashboard
    * **Doel**: Stuurt menselijk leesbare statusupdates, zoals "Hervat" of "Gepauzeerd".

* **esp32/distances_state**
    * **Richting**: Robot → Dashboard
    * **Doel**: Stuurt een JSON-object met alle sensordata en de machinestatus.
    * **Voorbeeldbericht**: `{"front":15, "left":30, "right":32, "voltage":7.8, "paused":false}`

* **esp32/batterij**
    * **Richting**: Robot → Dashboard
    * **Doel**: Stuurt enkel het batterijpercentage als een getal, voor simpele monitoring.

* **esp32/command**
    * **Richting**: Dashboard → Robot
    * **Doel**: Ontvangt commando's, zoals "toggle_pause", om de robot aan te sturen.

---

### 6. Web Dashboard 📊

Een **Node-RED** dashboard wordt gebruikt om de sensordata die via MQTT binnenkomt te visualiseren. Dit geeft een real-time overzicht van de status en metingen van de robot.

---

### 7. Installatie-instructies 🚀

1.  Zet de robot aan door de batterij aan te sluiten.
2.  Zorg ervoor dat de Wi-Fi en MQTT-gegevens (broker IP, username, wachtwoord) correct zijn ingevuld in de code.
3.  Activeer de MQTT-broker op de Raspberry Pi.
4.  Data zou nu moeten binnenkomen. Controleer dit via het Node-RED dashboard of een MQTT-client.
5.  Voor onderhoud kan de batterij eenvoudig losgekoppeld en vervangen worden.

---

### 8. Namaak Instructies 🛠️

Indien u deze robot wilt namaken:

1.  De **Gerber-files** voor de PCB zijn te vinden in de GitHub repository.
2.  Soldeer de componenten op de printplaat en verbind alles zoals op het schema staat.
3.  De volledige **Arduino-code** voor de ESP32 is ook te vinden op de GitHub-pagina.

```
