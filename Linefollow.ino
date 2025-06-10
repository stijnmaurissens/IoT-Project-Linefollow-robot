// ===================================================================================
//
//   FINALE CODE - V3.3
//   Project: Autonome Lijnvolgende Robot met Obstakeldetectie
//
//   BESCHRIJVING:
//   Deze code is voor een ESP32-robot die een zwarte lijn volgt en obstakels ontwijkt.
//   De robot gebruikt twee cores:
//   - Core 1 (hoofd loop): Regelt het rijgedrag (lijn volgen, ontwijken).
//   - Core 0 (achtergrondtaak): Regelt WiFi, MQTT, status-LEDs en de pauzeknop.
//
// ===================================================================================

#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>

// LED Pinnen
const int LED_PIN_ROOD = 33;
const int LED_PIN_GEEL = 32;
const int LED_PIN_GROEN = 12;

// Infrarood Sensoren (Links, Midden, Rechts)
const int irSensorPins[] = { 14, 27, 26 };
int irSensorWaarden[3];

// Ultrasone Afstandssensoren
#define trigPinVoor 5
#define echoPinVoor 25
#define trigPinLinks 4
#define echoPinLinks 16
#define trigPinRechts 15
#define echoPinRechts 2

// Motor Pinnen
int motorLinks_pin1 = 17;
int motorLinks_pin2 = 18;
int motorLinks_enable = 21;
int motorRechts_pin1 = 19;
int motorRechts_pin2 = 23;
int motorRechts_enable = 22;

// Instellingen voor de Ontwijkingsmanoeuvre
int draaiSnelheid_manoeuvre = 150;        // Snelheid voor draaien (0-255)
int draaiTijd90graden_ms = 780;           // Tijd nodig voor een 90 graden draai (kalibreren!)
int rijSnelheid_manoeuvre_ontwijken = 150;
const int OBSTAKEL_DREMPEL_CM_VOOR = 20;  // Afstand om ontwijking te starten
const int ZIJDELINGSE_RUIMTE_NODIG_CM = 40; // Ruimte nodig aan de zijkant om te weten dat het obstakel voorbij is
const int ZOEKSNELHEID_LIJN = 80;         // Snelheid bij het zoeken naar de lijn
const int MAX_ZOEK_LIJN_TIJD_MS = 4000;   // Maximale tijd om de lijn te zoeken
const int GEWENSTE_AFSTAND_OBSTAKEL_CM = 8; // Afstand om van de 'muur' te blijven
const float KP_AFSTANDSREGELING = 8.0;    // Stuurconstante voor het corrigeren van de afstand tot de muur
const int START_WACHTTIJD_NA_DRAAI_MS = 200;

// Batterijmeting & Dataverzending
const int adcPin = 34;      // Pin voor meten batterijspanning
const float Vref = 3.3;     // Referentiespanning ESP32
const float R1 = 7000.0, R2 = 2700.0; // Weerstanden van de spanningsdeler
const float adcMax = 4095.0; // 12-bit ADC resolutie
volatile float g_huidigeSpanning = 0;
unsigned long previousDataMillis = 0;
const long dataSendtime = 2000; // Data elke 2 seconden versturen

// Systeemvariabelen
volatile bool isGepauzeerd = false;          // Houdt de pauzestatus bij (gedeeld tussen cores)
volatile unsigned long pauseStartTime = 0;   // Onthoudt wanneer de pauze begon
volatile bool needsPostLineAction = false;   // Speciale actie nodig na hervatten? (bv. na volle lijn)
bool wasGepauzeerd = false;                  // Vorige status om wijziging te detecteren
volatile long afstandVoor = 0;               // Afstand voorste sensor (gedeeld tussen cores)
int startKnopPin = 13;                       // Fysieke pauzeknop
bool laatsteKnopStatus = HIGH;

// MQTT Configuratie
const char* ssid = "S23";
const char* password = "StijnTestRobot";
const char* mqttServer = "192.168.110.13";
const int mqttPort = 1883;
const char* mqttUser = "stijn1";
const char* mqttPassword = "maanroos";
const char* clientID = "R2-CLASSROOM";
const char* commandTopic = "esp32/command"; // Topic om commando's te ontvangen
WiFiClient espClient;
PubSubClient client(espClient);

// Functiedeclaraties
void stopMotoren();
void verstuurMqttData();
long meetAfstandGefilterd();
long meetAfstand(int trigPin, int echoPin);
void uitvoerenOntwijkManoeuvre(long actueleAfstandLinks, long actueleAfstandRechts);
void rijVooruit(int snelheid);
void rijVooruitGecorrigeerd(int snelheidLinks, int snelheidRechts);
void bochtLinks();
void bochtRechts();
void pivotLinks(int snelheid);
void pivotRechts(int snelheid);
void verbindOpnieuwMetMqtt();
void Core0Task(void* pvParameters);
void callback(char* topic, byte* payload, unsigned int length);
void nonBlockingDelay(unsigned long duration_ms);
void updateLedStatus();

// ===================================================================================
//   SETUP - Wordt eenmalig uitgevoerd bij opstarten
// ===================================================================================
void setup() {
    Serial.begin(115200);
    Serial.println("Robot opstarten (V3.3)...");

    // Pinnen instellen als OUTPUT of INPUT
    pinMode(LED_PIN_ROOD, OUTPUT); digitalWrite(LED_PIN_ROOD, LOW);
    pinMode(LED_PIN_GEEL, OUTPUT); digitalWrite(LED_PIN_GEEL, LOW);
    pinMode(LED_PIN_GROEN, OUTPUT); digitalWrite(LED_PIN_GROEN, LOW);

    pinMode(motorRechts_pin1, OUTPUT); pinMode(motorRechts_pin2, OUTPUT); pinMode(motorRechts_enable, OUTPUT);
    pinMode(motorLinks_pin1, OUTPUT); pinMode(motorLinks_pin2, OUTPUT); pinMode(motorLinks_enable, OUTPUT);

    pinMode(trigPinVoor, OUTPUT); pinMode(echoPinVoor, INPUT);
    pinMode(trigPinLinks, OUTPUT); pinMode(echoPinLinks, INPUT);
    pinMode(trigPinRechts, OUTPUT); pinMode(echoPinRechts, INPUT);

    pinMode(startKnopPin, INPUT);

    for (int i = 0; i < 3; i++) {
        pinMode(irSensorPins[i], INPUT);
    }

    // Start WiFi en MQTT
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    WiFi.setAutoReconnect(true);

    client.setServer(mqttServer, mqttPort);
    client.setCallback(callback); // Functie die wordt aangeroepen bij een MQTT-bericht
    delay(100);

    stopMotoren();

    // Start de achtergrondtaak op de andere core (Core 0)
    xTaskCreatePinnedToCore(Core0Task, "Achtergrond_Taken", 10000, NULL, 1, NULL, 0);

    Serial.println("Setup voltooid. Robot is klaar.");
    if (client.connected()) {
        client.publish("esp32/status", "Setup voltooid. Robot is klaar.");
    }
}

// ===================================================================================
//   HOOFDPROGRAMMA (CORE 1) - Hoofdtaak voor rijgedrag
// ===================================================================================
void loop() {
    if (isGepauzeerd && pauseStartTime != 0 && (millis() - pauseStartTime > 30000)) {
        Serial.println("30s wachttijd voorbij, robot hervat automatisch.");
        isGepauzeerd = false;
    }

    if (wasGepauzeerd && !isGepauzeerd) {
        pauseStartTime = 0;
        if (client.connected()) { client.publish("esp32/status", "Hervat"); }

        // Voer een klein duwtje vooruit uit als we hervatten na een stop bij een volle lijn
        if (needsPostLineAction) {
            rijVooruit(120);
            nonBlockingDelay(300);
            needsPostLineAction = false; // Reset de vlag, actie is uitgevoerd
        }
    }
    wasGepauzeerd = isGepauzeerd; // Onthoud de status voor de volgende keer

    // Als de robot gepauzeerd is, doe verder niets.
    if (isGepauzeerd) {
        stopMotoren();
        delay(100); // Kleine delay om de core niet te overbelasten
        return;
    }
    // EINDE PAUZEBEHEER

    // Meet de afstand met de voorste sensor
    afstandVoor = meetAfstandGefilterd();

    // Als een obstakel wordt gedetecteerd, start de ontwijkingsmanoeuvre
    if (afstandVoor < OBSTAKEL_DREMPEL_CM_VOOR && afstandVoor > 0) {
        Serial.println("[C1] Object gedetecteerd! Start ontwijking.");
        if(client.connected()) { client.publish("esp32/status", "Start ontwijking"); }
        long initAfstandLinks = meetAfstand(trigPinLinks, echoPinLinks);
        long initAfstandRechts = meetAfstand(trigPinRechts, echoPinRechts);
        uitvoerenOntwijkManoeuvre(initAfstandLinks, initAfstandRechts);
        Serial.println("[C1] Ontwijking voltooid, hervat lijnvolgen.");
    } else {
        // Geen obstakel, dus volg de lijn
        for (int i = 0; i < 3; i++) {
            irSensorWaarden[i] = digitalRead(irSensorPins[i]);
        }
        
        // LOW = ziet zwart, HIGH = ziet wit
        if (irSensorWaarden[0] == LOW && irSensorWaarden[1] == HIGH && irSensorWaarden[2] == LOW) {
            rijVooruit(110); // Midden op de lijn, rij rechtdoor
        }
        else if (irSensorWaarden[0] == HIGH && irSensorWaarden[1] == LOW && irSensorWaarden[2] == LOW) {
            bochtLinks(); // Lijn is links, stuur naar links
        }
        else if (irSensorWaarden[0] == LOW && irSensorWaarden[1] == LOW && irSensorWaarden[2] == HIGH) {
            bochtRechts(); // Lijn is rechts, stuur naar rechts
        }
        else if (irSensorWaarden[0] == HIGH && irSensorWaarden[1] == HIGH && irSensorWaarden[2] == LOW) {
            bochtLinks(); // Scherpe bocht naar links
        }
        else if (irSensorWaarden[0] == LOW && irSensorWaarden[1] == HIGH && irSensorWaarden[2] == HIGH) {
            bochtRechts(); // Scherpe bocht naar rechts
        }
        else if (irSensorWaarden[0] == HIGH && irSensorWaarden[1] == HIGH && irSensorWaarden[2] == HIGH) {
            // Alle sensoren zien wit, dit betekent een kruispunt of het einde
            // Pauzeer de robot voor 30 seconden
            if (!wasGepauzeerd) {
                stopMotoren();
                isGepauzeerd = true;
                pauseStartTime = millis();
                needsPostLineAction = true; // Speciale actie nodig na deze pauze
                if(client.connected()) client.publish("esp32/status", "Volle lijn: pauze (30s)");
                Serial.println("stop volle lijn");
            }
        }
        else {
            // Geen lijn gevonden, stop voor de veiligheid
            stopMotoren();
        }
    }
    delay(50); // Korte vertraging in de hoofdloop
}

// ===================================================================================
//   ONTWIJKINGSMANOEUVRE
// ===================================================================================
void uitvoerenOntwijkManoeuvre(long actueleAfstandLinks, long actueleAfstandRechts) {
    const int PAUZE_TUSSEN_STAPPEN_MS = 500;
    long maxRijLangsObstakelTijd_ms = 6000;

    stopMotoren();
    Serial.println("Manoeuvre: Start...");
    nonBlockingDelay(200); if(isGepauzeerd) return; // Stop direct als een pauze wordt geactiveerd

    // Bepaal de beste draairichting: kies de kant met de meeste ruimte.
    bool gaNaarLinks = (actueleAfstandLinks > actueleAfstandRechts);
    if (actueleAfstandLinks < 20 && actueleAfstandRechts < 20 ) {
        gaNaarLinks = true; // Als beide kanten geblokkeerd zijn, kies standaard links.
    }

    // STAP 1: Draai 90 graden weg van het object.
    Serial.println("STAP 1: Draai 90 graden weg.");
    if (gaNaarLinks) { pivotLinks(draaiSnelheid_manoeuvre); }
    else { pivotRechts(draaiSnelheid_manoeuvre); }
    nonBlockingDelay(draaiTijd90graden_ms); if(isGepauzeerd) { stopMotoren(); return; }
    stopMotoren();
    nonBlockingDelay(PAUZE_TUSSEN_STAPPEN_MS); if(isGepauzeerd) return;

    // STAP 1.1: Rijd een klein stukje vooruit om naast het object te komen.
    Serial.println("STAP 1.1: Positioneer naast object.");
    rijVooruit(rijSnelheid_manoeuvre_ontwijken);
    nonBlockingDelay(400); if(isGepauzeerd) { stopMotoren(); return; }
    stopMotoren();
    nonBlockingDelay(PAUZE_TUSSEN_STAPPEN_MS); if(isGepauzeerd) return;

    // STAP 2: Rijd langs het object en houd een vaste afstand.
    Serial.println("STAP 2: Volg zijkant van object...");
    unsigned long startTijdStap2 = millis();
    bool objectZijkantGepasseerd = false;

    while (millis() - startTijdStap2 < maxRijLangsObstakelTijd_ms) {
        if (isGepauzeerd) { stopMotoren(); return; }
        client.loop(); // Houd MQTT levend

        // Veiligheidscheck: als er voor de robot iets nieuws opduikt, stop.
        if (meetAfstandGefilterd() < OBSTAKEL_DREMPEL_CM_VOOR) {
            stopMotoren();
            isGepauzeerd = true; // Ga in een veilige pauzestand
            return;
        }

        // Bepaal welke zijsensor we moeten gebruiken
        int zijSensorTrig, zijSensorEcho;
        if (gaNaarLinks) { zijSensorTrig = trigPinRechts; zijSensorEcho = echoPinRechts; }
        else { zijSensorTrig = trigPinLinks; zijSensorEcho = echoPinLinks; }
        long lokaleAfstandZijkant = meetAfstand(zijSensorTrig, zijSensorEcho);

        // Controleer of de robot het object gepasseerd is (grote open ruimte aan de zijkant)
        if (millis() - startTijdStap2 > START_WACHTTIJD_NA_DRAAI_MS && lokaleAfstandZijkant > ZIJDELINGSE_RUIMTE_NODIG_CM) {
            objectZijkantGepasseerd = true;
        }

        if (objectZijkantGepasseerd) {
            rijVooruit(rijSnelheid_manoeuvre_ontwijken);
            nonBlockingDelay(300); // Rijd nog een klein stukje recht vooruit
            stopMotoren();
            break; // Verlaat de 'while' loop
        }

        // P-controller: pas de snelheid van de wielen aan om de gewenste afstand te behouden
        int fout = GEWENSTE_AFSTAND_OBSTAKEL_CM - lokaleAfstandZijkant;
        int correctie = constrain((int)(KP_AFSTANDSREGELING * (float)fout), -50, 50);
        int basisSnelheid = rijSnelheid_manoeuvre_ontwijken;
        int snelheidLinks, snelheidRechts;

        if (gaNaarLinks) { // Als we naar links zijn gedraaid, is het object rechts
            snelheidLinks = basisSnelheid + correctie; // Stuur van object af
            snelheidRechts = basisSnelheid - correctie; // Stuur naar object toe
        } else {
            snelheidLinks = basisSnelheid - correctie;
            snelheidRechts = basisSnelheid + correctie;
        }

        rijVooruitGecorrigeerd(snelheidLinks, snelheidRechts);
        delay(50);
    }

    if (isGepauzeerd) return;
    if (!objectZijkantGepasseerd) { // Als de tijd om is maar het object niet voorbij is, stop.
        stopMotoren();
        isGepauzeerd = true;
        return;
    }
    nonBlockingDelay(PAUZE_TUSSEN_STAPPEN_MS); if(isGepauzeerd) return;

    // STAP 3: Draai 90 graden terug om weer parallel met de originele lijn te staan.
    Serial.println("STAP 3: Draai terug (parallel).");
    if (gaNaarLinks) { pivotRechts(draaiSnelheid_manoeuvre); } else { pivotLinks(draaiSnelheid_manoeuvre); }
    nonBlockingDelay(draaiTijd90graden_ms); if(isGepauzeerd) { stopMotoren(); return; }
    stopMotoren();
    nonBlockingDelay(PAUZE_TUSSEN_STAPPEN_MS); if(isGepauzeerd) return;

    // STAP 4: Rijd een stukje recht vooruit om de "schaduw" van het obstakel te verlaten.
    Serial.println("STAP 4: Rijd voorbij obstakelzone.");
    rijVooruit(rijSnelheid_manoeuvre_ontwijken);
    nonBlockingDelay(1000); if(isGepauzeerd) { stopMotoren(); return; }
    stopMotoren();
    nonBlockingDelay(PAUZE_TUSSEN_STAPPEN_MS); if(isGepauzeerd) return;

    // STAP 5: Draai 90 graden terug richting de lijn.
    Serial.println("STAP 5: Draai richting de lijn.");
    if (gaNaarLinks) { pivotRechts(draaiSnelheid_manoeuvre); } else { pivotLinks(draaiSnelheid_manoeuvre); }
    nonBlockingDelay(draaiTijd90graden_ms); if(isGepauzeerd) { stopMotoren(); return; }
    stopMotoren();
    nonBlockingDelay(PAUZE_TUSSEN_STAPPEN_MS); if(isGepauzeerd) return;

    // STAP 6: Rijd langzaam vooruit totdat de lijn weer wordt gedetecteerd.
    Serial.println("STAP 6: Zoek de lijn...");
    rijVooruit(ZOEKSNELHEID_LIJN);
    unsigned long startZoekTijd = millis();
    bool lijnGevonden = false;

    while (millis() - startZoekTijd < MAX_ZOEK_LIJN_TIJD_MS) {
        if (isGepauzeerd) { stopMotoren(); return; }
        client.loop();

        if (digitalRead(irSensorPins[0]) == HIGH || digitalRead(irSensorPins[1]) == HIGH || digitalRead(irSensorPins[2]) == HIGH) {
            lijnGevonden = true;
            stopMotoren();
            break;
        }
        delay(20);
    }

    if (!lijnGevonden) { // Als de lijn niet is gevonden binnen de tijd, stop.
        stopMotoren();
        isGepauzeerd = true;
    }
}


// ===================================================================================
//   HULPFUNCTIES
// ===================================================================================


// Een 'delay' die niet blokkeert. MQTT-berichten en pauze-checks blijven werken.
// duration_ms De duur van de wachttijd in milliseconden.
void nonBlockingDelay(unsigned long duration_ms) {
    unsigned long startMillis = millis();
    while (millis() - startMillis < duration_ms) {
        if (client.connected()) {
            client.loop(); // Blijf MQTT-berichten verwerken
        }
        if (isGepauzeerd) { // Als de robot gepauzeerd wordt, stop de actie direct
            return;
        }
        delay(5);
    }
}


// Stuur beide motoren vooruit met dezelfde snelheid.
void rijVooruit(int snelheid) {
    digitalWrite(motorRechts_pin1, LOW); digitalWrite(motorRechts_pin2, HIGH); analogWrite(motorRechts_enable, snelheid);
    digitalWrite(motorLinks_pin1, HIGH); digitalWrite(motorLinks_pin2, LOW); analogWrite(motorLinks_enable, snelheid);
}


// Stuur beide motoren vooruit met verschillende snelheden voor correcties.
void rijVooruitGecorrigeerd(int snelheidLinks, int snelheidRechts) {
    digitalWrite(motorRechts_pin1, LOW); digitalWrite(motorRechts_pin2, HIGH); analogWrite(motorRechts_enable, constrain(snelheidRechts, 0, 255));
    digitalWrite(motorLinks_pin1, HIGH); digitalWrite(motorLinks_pin2, LOW); analogWrite(motorLinks_enable, constrain(snelheidLinks, 0, 255));
}


// Zet beide motoren volledig uit.
void stopMotoren() {
    digitalWrite(motorLinks_pin1, LOW); digitalWrite(motorLinks_pin2, LOW); analogWrite(motorLinks_enable, 0);
    digitalWrite(motorRechts_pin1, LOW); digitalWrite(motorRechts_pin2, LOW); analogWrite(motorRechts_enable, 0);
}


// Bocht naar links
void bochtLinks() {
    digitalWrite(motorRechts_pin1, LOW); digitalWrite(motorRechts_pin2, HIGH); analogWrite(motorRechts_enable, 110);
    digitalWrite(motorLinks_pin1, LOW); digitalWrite(motorLinks_pin2, HIGH); analogWrite(motorLinks_enable, 50);
}

// Bocht naar rechts.
void bochtRechts() {
    digitalWrite(motorLinks_pin1, HIGH); digitalWrite(motorLinks_pin2, LOW); analogWrite(motorLinks_enable, 120);
    digitalWrite(motorRechts_pin1, HIGH); digitalWrite(motorRechts_pin2, LOW); analogWrite(motorRechts_enable, 50);
}


// Draai op de plaats naar links (pivot turn).
void pivotLinks(int snelheid) {
    digitalWrite(motorLinks_pin1, LOW); digitalWrite(motorLinks_pin2, HIGH); analogWrite(motorLinks_enable, snelheid);
    digitalWrite(motorRechts_pin1, LOW); digitalWrite(motorRechts_pin2, HIGH); analogWrite(motorRechts_enable, snelheid);
}


// Draai op de plaats naar rechts (pivot turn).
void pivotRechts(int snelheid) {
    digitalWrite(motorLinks_pin1, HIGH); digitalWrite(motorLinks_pin2, LOW); analogWrite(motorLinks_enable, snelheid);
    digitalWrite(motorRechts_pin1, HIGH); digitalWrite(motorRechts_pin2, LOW); analogWrite(motorRechts_enable, snelheid);
}

// Meet de afstand, maar neemt de mediaan van 3 metingen om uitschieters te negeren.
long meetAfstandGefilterd() {
    const int numReadings = 3;
    long readings[numReadings];
    for (int i = 0; i < numReadings; i++) {
        readings[i] = meetAfstand(trigPinVoor, echoPinVoor);
        delay(5);
    }
    // Sorteer de metingen (simpele bubble sort)
    for (int i = 0; i < numReadings - 1; i++) {
        for (int j = 0; j < numReadings - i - 1; j++) {
            if (readings[j] > readings[j + 1]) {
                long temp = readings[j];
                readings[j] = readings[j + 1];
                readings[j + 1] = temp;
            }
        }
    }
    return readings[numReadings / 2]; // Geef de middelste waarde (mediaan) terug
}


// Voert een enkele afstandsmeting uit met een ultrasone sensor.
// De afstand in cm, of 999 als de meting mislukt.
long meetAfstand(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW); delayMicroseconds(2);
    digitalWrite(trigPin, HIGH); delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH, 25000); // Max 25ms wachten op echo
    if (duration > 0) {
        return duration / 58.2; // Omrekenen van tijd (microsec) naar afstand (cm)
    } else {
        return 999; // Foutwaarde
    }
}

// ===================================================================================
//   CORE 0 - ACHTERGRONDTAKEN
// ===================================================================================
// Deze functie draait continu op de achtergrond op Core 0.

void Core0Task(void* pvParameters) {
    Serial.print("[C0] Achtergrondtaak gestart op core: "); Serial.println(xPortGetCoreID());

    for (;;) { // Een oneindige lus, net als `void loop()`
        // Beheer WiFi en MQTT verbinding
        if (WiFi.status() == WL_CONNECTED) {
            if (!client.connected()) {
                verbindOpnieuwMetMqtt();
            }
            if (client.connected()) {
                client.loop(); // Verwerk binnenkomende MQTT-berichten
            }
        } else {
            delay(1000); // Wacht even als er geen WiFi is
        }

        // Werk de status van de LEDs bij
        updateLedStatus();

        // Controleer of de fysieke knop is ingedrukt
        bool currentButtonState = digitalRead(startKnopPin);
        if (laatsteKnopStatus == HIGH && currentButtonState == LOW) { // Knop is net ingedrukt
            delay(50); // Debounce: wacht even om valse signalen te negeren
            currentButtonState = digitalRead(startKnopPin);
            if (currentButtonState == LOW) { // Knop is nog steeds ingedrukt
                // Wissel de pauzestatus
                if (isGepauzeerd) {
                    isGepauzeerd = false; // Hervat de robot
                } else {
                    isGepauzeerd = true; // Pauzeer de robot
                    pauseStartTime = millis();
                    needsPostLineAction = false; // Een handmatige pauze heeft geen speciale actie nodig
                    stopMotoren();
                }
            }
        }
        laatsteKnopStatus = currentButtonState;

        // Verstuur elke 'dataSendtime' milliseconden een update via MQTT
        if (millis() - previousDataMillis >= dataSendtime) {
            previousDataMillis = millis();

            // Stuur een aftelbericht naar MQTT als de robot gepauzeerd is
            if (isGepauzeerd && pauseStartTime != 0) {
                unsigned long elapsedTime = millis() - pauseStartTime;
                if (elapsedTime < 30000) {
                    int secondsRemaining = (30000 - elapsedTime + 999) / 1000;
                    char statusMsg[50];
                    sprintf(statusMsg, "Pauze. Hervat over %d seconden...", secondsRemaining);
                    Serial.println(statusMsg);
                    if (client.connected()) {
                        client.publish("esp32/status", statusMsg);
                    }
                }
            }

            // Stuur de reguliere sensor- en statusdata
            if (client.connected()) {
                verstuurMqttData();
            }
        }
        
        delay(20); // Kleine delay om de core niet 100% te belasten
    }
}


// Stuurt een pakket met data (batterij, afstanden, status) naar de MQTT-server.

void verstuurMqttData() {
    if (!client.connected()) { return; }

    // Bereken batterijspanning en percentage
    int rawADC = analogRead(adcPin);
    g_huidigeSpanning = (rawADC / adcMax) * Vref * (R1 + R2) / R2;
    float batterijPercentage = constrain(map(g_huidigeSpanning * 100, 600, 840, 0, 100), 0, 100);

    Serial.printf("[C0] Batterij: %.1f%%, Spanning: %.2fV\n", batterijPercentage, g_huidigeSpanning);

    // Stuur batterijpercentage
    char batStr[10];
    dtostrf(batterijPercentage, 4, 1, batStr);
    client.publish("esp32/batterij", batStr);

    // Meet en stuur afstanden
    long currentLeft = meetAfstand(trigPinLinks, echoPinLinks);
    long currentRight = meetAfstand(trigPinRechts, echoPinRechts);
    char distancePayload[50];
    sprintf(distancePayload, "%ld %ld %ld", afstandVoor, currentLeft, currentRight);
    client.publish("esp32/distances", distancePayload);

    // Stuur een compleet statusbericht in JSON-formaat
    char jsonBuffer[200];
    sprintf(jsonBuffer, "{\"front\": %ld, \"left\": %ld, \"right\": %ld, \"voltage\": %.2f, \"paused\": %s}",
            afstandVoor, currentLeft, currentRight, g_huidigeSpanning, isGepauzeerd ? "true" : "false");
    client.publish("esp32/distances_state", jsonBuffer);
}


// Regelt de status-LEDs op basis van de batterijstatus en pauzestand.
void updateLedStatus() {
    if (isGepauzeerd) {
        // Laat de gele LED knipperen in pauzestand
        digitalWrite(LED_PIN_GROEN, LOW);
        digitalWrite(LED_PIN_ROOD, LOW);
        static unsigned long lastBlinkMillis = 0;
        if (millis() - lastBlinkMillis > 500) {
            digitalWrite(LED_PIN_GEEL, !digitalRead(LED_PIN_GEEL));
            lastBlinkMillis = millis();
        }
    } else {
        // Toon batterijstatus met de LEDs
        int rawADC = analogRead(adcPin);
        float spanning = (rawADC / adcMax) * Vref * (R1 + R2) / R2;
        float batterijPercentage = constrain(map(spanning * 100, 600, 840, 0, 100), 0, 100);

       if (batterijPercentage > 30){
            // Groen: batterij OK
            digitalWrite(LED_PIN_GROEN, HIGH);
            digitalWrite(LED_PIN_GEEL, LOW);
            digitalWrite(LED_PIN_ROOD, LOW);
        } else if (batterijPercentage <10){
            // Rood: batterij is kritiek, stop de motoren
            digitalWrite(LED_PIN_GROEN, LOW);
            digitalWrite(LED_PIN_GEEL, LOW);
            digitalWrite(LED_PIN_ROOD, HIGH);
            stopMotoren();
            client.publish("esp32/status", "Batterij leeg!!");
        } else {
            // Geel: batterij is bijna leeg
            digitalWrite(LED_PIN_GROEN, LOW);
            digitalWrite(LED_PIN_ROOD, LOW);
            digitalWrite(LED_PIN_GEEL, HIGH);
            client.publish("esp32/status", "Batterij bijna leeg!");
        }
    }
}

// Probeert opnieuw te verbinden met de MQTT-server als de verbinding wegvalt.
void verbindOpnieuwMetMqtt() {
    if (WiFi.status() != WL_CONNECTED) { return; }
    int retries = 0;
    while (!client.connected() && retries < 5) {
        retries++;
        if (client.connect(clientID, mqttUser, mqttPassword)) {
            client.subscribe(commandTopic); // Abonneer opnieuw op het commando-topic
            client.publish("esp32/status", "MQTT Verbonden");
        } else {
            delay(2000); // Wacht 2 seconden voor de volgende poging
        }
    }
}

// Wordt uitgevoerd wanneer een bericht binnenkomt op een geabonneerd MQTT-topic.
void callback(char* topic, byte* payload, unsigned int length) {
    // Converteer de payload naar een leesbare string
    char message[length + 1];
    memcpy(message, payload, length);
    message[length] = '\0';
    String messageStr = String(message);
    Serial.print("Commando ontvangen via MQTT: "); Serial.println(messageStr);

    // Handel het commando af
    if (messageStr == "toggle_pause") {
        if (isGepauzeerd) {
            isGepauzeerd = false; // Hervat
        } else {
            isGepauzeerd = true; // Pauzeer
            pauseStartTime = millis();
            needsPostLineAction = false; // Dit is een normale pauze, geen speciale actie nodig
            stopMotoren();
            if (client.connected()) client.publish("esp32/status", "Gepauzeerd (MQTT, 30s)");
        }
    }
}
