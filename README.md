# IoT-Project-Linefollow-robot
school opdracht: robot auto met lijnvolging en obstakel ontwijking 


 
Inhoud
1.	BESCHRIJVING VAN DE GEBRUIKTE HARDWARE.	
2.	SCHEMA’S OF DIAGRAMMEN VAN DE HARDWARE-OPSTELLING EN BEKABELING.	
3.	BESCHRIJVING VAN DE BASISFUNCTIONALITEIT.	
3.1.	Lijnvolging	
3.2. Obstakelontwijking	
3.2.	Dual-Core Verwerking (ESP32)	
4.	MQTT-INSTELLINGEN.	
4.1.	Topics:	
4.2.	Webdashboard:	
5.	INSTALLATIE-INSTRUCTIES VOOR HET TESTEN VAN DE ROBOT EN MQTT-COMMUNICATIE.	

 
1.	Beschrijving van de gebruikte hardware. 
Een overzicht van de essentiële componenten.

•	ESP32 Dev Board: Deze microcontroller bestuurt alle sensoren en actuatoren, verwerkt de logica voor lijnvolging en obstakelontwijking, en handelt de Wi-Fi-verbinding en MQTT-communicatie af. Het maakt gebruik van de dual-core.

•	Raspberry Pi 5: hier staat de volgende software op : MQTT,InfluxDB database en Grafana (voor het visualiseren van de data in dashboards).

•	3x Infrarood (IR) Sensoren: Gebruikt voor lijnvolging.

•	1x Ultrasone Sensor (HC-SR04): Gebruikt voor het detecteren van obstakels.

•	L298N Motor Driver Module: Stuurt de twee DC-motoren aan

•	2x DC Motoren.

•	1x Drukknop (Button): hiermee kan de robot gepauzeerd of worden hervat.

•	9V Batterij: Levert stroom aan de ESP32 en de motoren.

•	•  Spanningsdeler :voor Batterijmonitorin. 

2.	Schema’s of diagrammen van de hardware-opstelling en bekabeling.
 

 
3.	Beschrijving van de basisfunctionaliteit.
3.1.	Lijnvolging
De robot  volgt een zwarte lijn op een “witte” ondergrond te volgen. Dit wordt gedaan door drie naar beneden gerichte infraroodsensoren.
•	Sensoruitlezing: De statussen van de drie IR-sensoren (sensorValues[0], sensorValues[1], sensorValues[2]) worden continu gelezen. Een LOW waarde betekent dat wit wordt gedetecteerd, en HIGH dat zwart wordt gedetecteerd

•	Basislogica
1.	Rechtdoor: Als de middelste sensor de lijn ziet (HIGH) en de buitenste sensoren niet (LOW):
	sensorValues[0] == LOW && sensorValues[1] == HIGH && sensorValues[2] == LOW
	Actie: forward(100); (rijdt vooruit met snelheid 100).
2.	Correctie naar Links: Als de linkersensor de lijn ziet (HIGH) en de rechtersensor niet (LOW):
	sensorValues[0] == HIGH && sensorValues[2] == LOW
	Actie: turn_left();
3.	Correctie naar Rechts: Als de rechtersensor de lijn ziet (HIGH) en de linkersensor niet (LOW):
	sensorValues[0] == LOW && sensorValues[2] == HIGH
	Actie: turn_right();
4.	Alle Sensoren Zien Lijn: Als alle drie de sensoren de lijn detecteren (HIGH):
	sensorValues[0] == HIGH && sensorValues[1] == HIGH && sensorValues[2] == HIGH
	Actie: stop(); delay(3000); forward(100); delay(300); (stopt 3 seconden, rijdt dan kort vooruit).
5.	Geen Lijn Gedetecteerd / Andere Scenario's:
	Actie: stop();
•	Snelheidscompensatie: De functie compenseerSnelheid(int basisSnelheid) past de PWM-waarde voor de motoren aan op basis van de actuele batterijspanning (g_huidigeSpanning). Dit helpt om een meer consistente rijsnelheid te behouden naarmate de batterij leegloopt.


3.2. Obstakelontwijking
De robot kan obstakels detecteren en een manoeuvre uitvoeren om deze proberen te ontwijken.

3.2.	Dual-Core Verwerking (ESP32)
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


4.	MQTT-instellingen.
4.1.	Topics:

De ESP32 publiceert data naar de volgende MQTT-topics
•	esp32/batterij
o	Beschrijving: Publiceert het batterijpercentage.
o	code: dtostrf(batterij, 4, 1, batStr); client.publish("esp32/batterij", batStr);
o	Dashboard: deze word als een guage gevisualiseerd op grafana
•	esp32/ultrasonep
o	Beschrijving: Publiceert de gemeten afstand door de ultrasone sensor.
o	Voorbeeld uit code: dtostrf(distance1, 4, 1, ultStr); client.publish("esp32/ultrasone", ultStr);
o	Dashboard: word als een lijn weergeven (hoe dichter bij een object hoe kleiner en roder de lijn word)
•	esp32/status
o	Beschrijving: Publiceert statusberichten over de werking van de robot.
o	Mogelijke berichten:
	"Gepauzeerd"
	"Robot hervat"
	"Auto-hervat na 30s"
	"Start ontwijking"
	"Ontwijking voltooid"
4.2.	Webdashboard:
Grafana krijgt de sensor data binnen via de Raspberry pi en influxdb. Grafana ververst automatisch elke 5seconden, maar indien er nog accuratere data gewenst word kan dit ook handmatig ververst worden. 
5.	Installatie-instructies voor het testen van de robot en MQTT-communicatie.
Zet de robot aan met de juiste gegevens van de MQTT-broker (internet naam en wachtwoord, de server IP, de MQTT username en wachtwoord). Activeer met de zelfde gegevens MQTT op de Raspberry pi en de data zou moeten binnen komen. Men kan de data dubbel checken door een ssh connectie te maken met de PI en naar de Serial monitor te kijken. 

