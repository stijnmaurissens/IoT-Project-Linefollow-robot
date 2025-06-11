 
Inhoud
1.	GITHUB LINK	3
2.	BESCHRIJVING VAN DE GEBRUIKTE HARDWARE.	3
3.	SCHEMA’S HARDWARE-OPSTELLING.	4
4.	CODE OVERZICHT EN BELANGRIJKSTE FUNCTIES	5
4.1.	Belangrijkste functies	5
4.2.	Obstakelontwijking	5
4.3.	Dual-Core Verwerking (ESP32)	5
5.	MQTT-INSTELLINGEN.	6
5.1.	esp32/status	6
5.2.	esp32/distances_state	6
5.3.	esp32/batterij	6
5.4.	esp32/command	6
6.	WEB DASHBOARD:	7
7.	INSTALLATIE-INSTRUCTIES VOOR HET TESTEN VAN DE ROBOT EN MQTT-COMMUNICATIE.	8
8.	NAMAAK INSTRUCTIES:	8

 
1.	GitHub link
https://github.com/stijnmaurissens/IoT-Project-Linefollow-robot 

2.	Beschrijving van de gebruikte hardware. 
Een overzicht van de essentiële componenten.

•	ESP32 Dev Board: Deze microcontroller bestuurt alle sensoren en actuatoren, verwerkt de logica voor lijnvolging en obstakelontwijking, en handelt de Wi-Fi-verbinding en MQTT-communicatie af. Het maakt gebruik van de dual-core.

•	Raspberry Pi 5: hier staat de volgende software op : MQTT,InfluxDB database en Node Red (voor het visualiseren van de data in dashboards).

•	3x Infrarood (IR) Sensoren: Gebruikt voor lijnvolging.

•	3x Ultrasone Sensor (HC-SR04): Gebruikt voor het detecteren van obstakels.

•	2x DC Motoren: voor de robot vooruit te laten gaan

•	1x Drukknop (Button): hiermee kan de robot gepauzeerd of worden hervat.

•	9V Batterij: Levert stroom aan de ESP32 en de motoren.

•	Onze zelf ontworpen PCB

3.	Schema’s hardware-opstelling.

 
4.	Code overzicht en belangrijkste functies 
4.1.	Belangrijkste functies
Het lijnvolging systeem.
Als naar de lijn kijken en enkel de midelste sensor ziet de lijn dan rijd de robot vooruit. De lijn detecteren doet hij met de volgende code:

if (irSensorWaarden[0] == LOW && irSensorWaarden[1] == HIGH && irSensorWaarden[2] == LOW) {
            rijVooruit(110);
            Serial.println("vooruit");

Waarbij onze functie rijVooruit gebruikt word met de parameter van 110. Deze parameter word gebruikt in de volgende code om de snelheid te bepalen. Waarbij beide motoren vooruit rijden.

void rijVooruit(int snelheid) {
    digitalWrite(motorRechts_pin1, LOW); digitalWrite(motorRechts_pin2, HIGH); analogWrite(motorRechts_enable, snelheid);
    digitalWrite(motorLinks_pin1, HIGH); digitalWrite(motorLinks_pin2, LOW); analogWrite(motorLinks_enable, snelheid);

Om de robot te laten draaien, passen we dezelfde logica toe als voor het vooruit rijden, maar keren we de draairichting van één van de wielen om.

 
4.2.	Obstakelontwijking
De robot kan obstakels detecteren en een poging manoeuvre uitvoeren om deze proberen te ontwijken.
Maar deze werkt helaas nog niet zoals gewenst

4.3.	Dual-Core Verwerking (ESP32)
De ESP32 heeft een dual-core processor, wat parallelle taakuitvoering mogelijk .
Core 0 (Core0Task):
o	Deze core in een oneindige lus, toegewezen met xTaskCreatePinnedToCore(..., 0).
o	Verantwoordelijkheden:
	MQTT Client Loop: client.loop() onderhoudt de verbinding en verwerkt inkomende/uitgaande MQTT-berichten.
	MQTT Herverbinding: if (!client.connected()) reconect(); zorgt dat de verbinding met de broker hersteld wordt indien verloren.
	Knopafhandeling: Leest de status van buttonPin om isPaused te wisselen.
	Automatisch Hervatten: Als isPaused is en 30 seconden (pauseDuration) zijn verstreken, wordt isPaused op false gezet.
Core 1:
o	Voert de setup() functie eenmalig uit.
o	Voert de loop() functie continu uit.
o	Verantwoordelijkheden:
	Hoofdlogica Robot: Lijnvolging en aanroepen van ontwijkObstakel() wanneer nodig.
	Reageert op de isPaused door te stoppen.
5.	MQTT-instellingen.
5.1.	esp32/status
Richting: Robot → Dashboard
Doel: Dit topic wordt gebruikt voor algemene, menselijk leesbare statusupdates. Denk aan berichten zoals "Hervat", "Gepauzeerd (Knop)", of de aftelberichten zoals "Pauze. Hervat over 15 seconden...". Het is bedoeld om direct op het dashboard te tonen wat de robot aan het doen is.

5.2.	esp32/distances_state
Richting: Robot → Dashboard
Doel: Dit is het belangrijkste datakanaal. De robot stuurt hierover een compleet JSON-object met alle relevante sensordata en de huidige status. Het dashboard gebruikt deze gestructureerde data om de visuele elementen, zoals de afstandsmeters en statusiconen, bij te werken.
Voorbeeldbericht: {"front":15, "left":30, "right":32, "voltage":7.8, "paused":false}

5.3.	esp32/batterij
Richting: Robot → Dashboard
Doel: Hoewel de batterijstatus ook in het distances_state topic zit, wordt op dit aparte topic enkel het batterijpercentage (als een getal) verstuurd. Dit maakt het voor andere systemen of simpele monitoring-scripts makkelijker om enkel de batterijstatus te volgen, zonder een heel JSON-object te moeten parsen.

5.4.	esp32/command
Richting: Dashboard → Robot
Doel: Dit is het commandokanaal om de robot aan te sturen. Wanneer een gebruiker op de pauzeknop in de app of het dashboard klikt, wordt het bericht "toggle_pause" naar dit topic gestuurd. De robot luistert continu naar dit topic en zal zijn pauzestatus omschakelen zodra hij dit bericht ontvangt.

 
6.	Web dashboard:
Node Red krijgt de sensor data binnen via de MQTT. 
  
7.	Installatie-instructies voor het testen van de robot en MQTT-communicatie.
Zet de robot aan (door de batterij aan te sluiten) met de juiste gegevens van de MQTT-broker (internet naam en wachtwoord, de server IP, de MQTT username en wachtwoord). Activeer met de zelfde gegevens MQTT op de Raspberry pi en de data zou moeten binnen komen. Men kan de data dubbel checken door een ssh connectie te maken met de PI en naar de Serial monitor te kijken.
Voor onderhoud kan u de batterij gewoon uit trekken en vervangen met een nieuwe.

8.	Namaak instructies:
Indien u deze robot wilt namaken kan u de Gerber files vinden op onze GitHub (zie document maarten_stijn.zip)
Eens u de print heeft kan u de deze vullen en verbinden zoals op het schema staat (zie immage.png)
De code dat wij gebruiken is ook te vinden op onze GitHub pagina. 

