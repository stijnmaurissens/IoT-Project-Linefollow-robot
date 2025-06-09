// ===================================================================================
//
//        FINALE CODE - V3.3 (AANGEPAST IP & BATTERIJ PRINT)
//
// ===================================================================================

#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>

// --- LED Pinnen
const int LED_PIN_ROOD = 33;
const int LED_PIN_GEEL = 32;
const int LED_PIN_GROEN = 12;

// --- Infrarood Sensoren
const int irSensorPins[] = { 14, 27, 26 };
int irSensorWaarden[3];

// --- Ultrasone Afstandssensoren
#define trigPinVoor 5
#define echoPinVoor 25
#define trigPinLinks 4
#define echoPinLinks 16
#define trigPinRechts 15
#define echoPinRechts 2

// --- Motor Pinnen
int motorLinks_pin1 = 17;
int motorLinks_pin2 = 18;
int motorLinks_enable = 21;
int motorRechts_pin1 = 19;
int motorRechts_pin2 = 23;
int motorRechts_enable = 22;

// --- Instellingen Objectontwijking
int draaiSnelheid_manoeuvre = 150;
int draaiTijd90graden_ms = 780;
int rijSnelheid_manoeuvre_ontwijken = 150;
const int OBSTAKEL_DREMPEL_CM_VOOR = 20;
const int ZIJDELINGSE_RUIMTE_NODIG_CM = 40;
const int ZOEKSNELHEID_LIJN = 80;
const int MAX_ZOEK_LIJN_TIJD_MS = 4000;
const int GEWENSTE_AFSTAND_OBSTAKEL_CM = 8;
const float KP_AFSTANDSREGELING = 8.0;
const int START_WACHTTIJD_NA_DRAAI_MS = 200;

// --- Batterij & Timing
const int adcPin = 34;
const float Vref = 3.3;
const float R1 = 7000.0;
const float R2 = 2700.0;
const float adcMax = 4095.0;
volatile float g_huidigeSpanning = 0;
unsigned long previousDataMillis = 0;
const long dataSendtime = 2000;

// --- Systeem Variabelen
volatile bool isGepauzeerd = false;
volatile unsigned long pauseStartTime = 0;      // Houdt bij wanneer een pauze startte
volatile bool needsPostLineAction = false;     // Onthoudt of een actie nodig is na lijn-pauze
bool wasGepauzeerd = false;                     // Houdt vorige pauzestatus bij
volatile long afstandVoor = 0;
volatile long afstandLinks = 0;
volatile long afstandRechts = 0;
int startKnopPin = 13;
bool laatsteKnopStatus = HIGH;


// --- MQTT Configuratie
const char* ssid = "S23";
const char* password = "StijnTestRobot";
const char* mqttServer = "192.168.110.13"; // *** AANPASSING ***
const int mqttPort = 1883;
const char* mqttUser = "stijn1";
const char* mqttPassword = "maanroos";
const char* clientID = "R2-CLASSROOM";
const char* commandTopic = "esp32/command";
WiFiClient espClient;
PubSubClient client(espClient);

// --- Functie Declaraties
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
//                                     SETUP
// ===================================================================================
void setup() {
    Serial.begin(115200);
    Serial.println("Robot opstarten (V3.3)...");
    
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
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    WiFi.setAutoReconnect(true);
    
    client.setServer(mqttServer, mqttPort);
    client.setCallback(callback);
    delay(100);
    
    stopMotoren();
    
    xTaskCreatePinnedToCore(Core0Task, "Achtergrond_Taken", 10000, NULL, 1, NULL, 0);
    
    Serial.println("Setup voltooid. Robot is klaar.");
    if (client.connected()) {
        client.publish("esp32/status", "Setup voltooid. Robot is klaar.");
    }
}

// ===================================================================================
//                                  HOOFD LOOP (CORE 1)
// ===================================================================================
void loop() {
    // --- CENTRAAL PAUZEBEHEER ---
    // 1. Controleer of de automatische pauzetijd (30s) voorbij is.
    if (isGepauzeerd && pauseStartTime != 0 && (millis() - pauseStartTime > 30000)) {
        Serial.println("30s wachttijd voorbij, robot hervat automatisch.");
        isGepauzeerd = false; // Hervat de robot
    }

    // 2. Detecteer de overgang van gepauzeerd naar actief en voer acties uit.
    if (wasGepauzeerd && !isGepauzeerd) {
        pauseStartTime = 0; // Reset de pauzetimer
        if (client.connected()) { client.publish("esp32/status", "Hervat"); }

        // Voer speciale actie uit als we hervatten na een volle lijn
        if (needsPostLineAction) {
            rijVooruit(120);
            nonBlockingDelay(300);
            needsPostLineAction = false; // Reset de vlag, actie is uitgevoerd
        }
    }
    wasGepauzeerd = isGepauzeerd; // Onthoud de huidige status voor de volgende cyclus

    // 3. Als de robot gepauzeerd is, stop alle acties.
    if (isGepauzeerd) {
        stopMotoren();
        delay(100);
        return;
    }
    // --- EINDE PAUZEBEHEER ---


    afstandVoor = meetAfstandGefilterd();
    
    if (afstandVoor < OBSTAKEL_DREMPEL_CM_VOOR && afstandVoor > 0) {
        Serial.println("[C1] Object gedetecteerd! Start ontwijking.");
        if(client.connected()){
            client.publish("esp32/status", "Start ontwijking");
        }
        long initAfstandLinks = meetAfstand(trigPinLinks, echoPinLinks);
        long initAfstandRechts = meetAfstand(trigPinRechts, echoPinRechts);
        uitvoerenOntwijkManoeuvre(initAfstandLinks, initAfstandRechts);
        Serial.println("[C1] Ontwijking voltooid of onderbroken, hervat lijnvolgen.");
    } else {
        // Lijnvolg logica
        for (int i = 0; i < 3; i++) {
            irSensorWaarden[i] = digitalRead(irSensorPins[i]);
        }
        if (irSensorWaarden[0] == LOW && irSensorWaarden[1] == HIGH && irSensorWaarden[2] == LOW) {
            rijVooruit(110);
            Serial.println("vooruit");
        }
        else if (irSensorWaarden[0] == HIGH && irSensorWaarden[1] == LOW && irSensorWaarden[2] == LOW) {
            bochtLinks();
            Serial.println("links");
        }
        else if (irSensorWaarden[0] == LOW && irSensorWaarden[1] == LOW && irSensorWaarden[2] == HIGH) {
            bochtRechts();
            Serial.println("rechts");
        }
        else if (irSensorWaarden[0] == HIGH && irSensorWaarden[1] == HIGH && irSensorWaarden[2] == LOW) {
            bochtLinks();
            Serial.println("links?");
        }
        else if (irSensorWaarden[0] == LOW && irSensorWaarden[1] == HIGH && irSensorWaarden[2] == HIGH) {
            bochtRechts();
            Serial.println("rechts ?");
        }
        else if (irSensorWaarden[0] == HIGH && irSensorWaarden[1] == HIGH && irSensorWaarden[2] == HIGH) {
            // Start alleen een pauze als de robot nog niet gepauzeerd is.
            if (!wasGepauzeerd) {
                 stopMotoren();
                 isGepauzeerd = true;
                 pauseStartTime = millis();
                 needsPostLineAction = true; // Deze pauze vereist een speciale actie bij hervatten.
                 if(client.connected()) client.publish("esp32/status", "Volle lijn: pauze (30s)");
                 Serial.println("stop volle lijn");
            }
        }
        else {
            stopMotoren();
            Serial.println("stop");
        }
    }
    delay(50);
}

// ===================================================================================
//                             ONTWIJKINGSMANOEUVRE
// ===================================================================================
void uitvoerenOntwijkManoeuvre(long actueleAfstandLinks, long actueleAfstandRechts) {
    const int PAUZE_TUSSEN_STAPPEN_MS = 500;
    long maxRijLangsObstakelTijd_ms = 6000;
    
    stopMotoren();
    Serial.println("Manoeuvre: Start...");
    nonBlockingDelay(200); if(isGepauzeerd) return;

    bool gaNaarLinks = (actueleAfstandLinks > actueleAfstandRechts);
    if (actueleAfstandLinks < 20 && actueleAfstandRechts < 20 ) {
        gaNaarLinks = true;
    }

    Serial.println("STAP 1: Draai 90 graden weg.");
    if(client.connected()) client.publish("esp32/status",  "STAP 1: Draai 90 graden weg.");
    if (gaNaarLinks) { pivotLinks(draaiSnelheid_manoeuvre); } 
    else { pivotRechts(draaiSnelheid_manoeuvre); }
    nonBlockingDelay(draaiTijd90graden_ms); if(isGepauzeerd) { stopMotoren(); return; }
    
    stopMotoren();
    nonBlockingDelay(PAUZE_TUSSEN_STAPPEN_MS); if(isGepauzeerd) return;

    Serial.println("STAP 1.1: Positioneer naast object.");
    rijVooruit(rijSnelheid_manoeuvre_ontwijken);
    nonBlockingDelay(400); if(isGepauzeerd) { stopMotoren(); return; }
    
    stopMotoren();
    nonBlockingDelay(PAUZE_TUSSEN_STAPPEN_MS); if(isGepauzeerd) return;
    
    Serial.println("STAP 2: Volg zijkant van object...");
    unsigned long startTijdStap2 = millis();
    bool objectZijkantGepasseerd = false;
    
    while (millis() - startTijdStap2 < maxRijLangsObstakelTijd_ms) {
        if (isGepauzeerd) { stopMotoren(); return; }
        client.loop();
        
        if (meetAfstandGefilterd() < OBSTAKEL_DREMPEL_CM_VOOR) { 
            stopMotoren(); 
            isGepauzeerd = true;
            return; 
        }

        int zijSensorTrig, zijSensorEcho;
        if (gaNaarLinks) { zijSensorTrig = trigPinRechts; zijSensorEcho = echoPinRechts; }
        else { zijSensorTrig = trigPinLinks; zijSensorEcho = echoPinLinks; }
        long lokaleAfstandZijkant = meetAfstand(zijSensorTrig, zijSensorEcho);
        
        if (millis() - startTijdStap2 > START_WACHTTIJD_NA_DRAAI_MS && lokaleAfstandZijkant > ZIJDELINGSE_RUIMTE_NODIG_CM) {
            objectZijkantGepasseerd = true;
        }
        
        if (objectZijkantGepasseerd) {
            rijVooruit(rijSnelheid_manoeuvre_ontwijken);
            nonBlockingDelay(300); if(isGepauzeerd) { stopMotoren(); return; }
            stopMotoren();
            break;
        }

        int fout = GEWENSTE_AFSTAND_OBSTAKEL_CM - lokaleAfstandZijkant;
        int correctie = constrain((int)(KP_AFSTANDSREGELING * (float)fout), -50, 50);
        int basisSnelheid = rijSnelheid_manoeuvre_ontwijken;
        int snelheidLinks, snelheidRechts;

        if (gaNaarLinks) { snelheidLinks = basisSnelheid + correctie; snelheidRechts = basisSnelheid - correctie; } 
        else { snelheidLinks = basisSnelheid - correctie; snelheidRechts = basisSnelheid + correctie; }
        
        rijVooruitGecorrigeerd(snelheidLinks, snelheidRechts);
        delay(50);
    }

    if (isGepauzeerd) return;
    if (!objectZijkantGepasseerd) { 
        stopMotoren(); 
        isGepauzeerd = true;
        return; 
    }
    nonBlockingDelay(PAUZE_TUSSEN_STAPPEN_MS); if(isGepauzeerd) return;
    
    Serial.println("STAP 3: Draai terug (parallel).");
    if (gaNaarLinks) { pivotRechts(draaiSnelheid_manoeuvre); } else { pivotLinks(draaiSnelheid_manoeuvre); }
    nonBlockingDelay(draaiTijd90graden_ms); if(isGepauzeerd) { stopMotoren(); return; }

    stopMotoren();
    nonBlockingDelay(PAUZE_TUSSEN_STAPPEN_MS); if(isGepauzeerd) return;

    Serial.println("STAP 4: Rijd voorbij obstakelzone.");
    rijVooruit(rijSnelheid_manoeuvre_ontwijken);
    nonBlockingDelay(1000); if(isGepauzeerd) { stopMotoren(); return; }

    stopMotoren();
    nonBlockingDelay(PAUZE_TUSSEN_STAPPEN_MS); if(isGepauzeerd) return;

    Serial.println("STAP 5: Draai richting de lijn.");
    if (gaNaarLinks) { pivotRechts(draaiSnelheid_manoeuvre); } else { pivotLinks(draaiSnelheid_manoeuvre); }
    nonBlockingDelay(draaiTijd90graden_ms); if(isGepauzeerd) { stopMotoren(); return; }

    stopMotoren();
    nonBlockingDelay(PAUZE_TUSSEN_STAPPEN_MS); if(isGepauzeerd) return;

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

    if (!lijnGevonden) {
        stopMotoren();
        isGepauzeerd = true;
    }
}

// ===================================================================================
//                                  HULPFUNCTIES
// ===================================================================================

void nonBlockingDelay(unsigned long duration_ms) {
    unsigned long startMillis = millis();
    while (millis() - startMillis < duration_ms) {
        if (client.connected()) {
            client.loop(); 
        }
        if (isGepauzeerd) {
            return;
        }
        delay(5); 
    }
}

void rijVooruit(int snelheid) {
    digitalWrite(motorRechts_pin1, LOW); digitalWrite(motorRechts_pin2, HIGH); analogWrite(motorRechts_enable, snelheid);
    digitalWrite(motorLinks_pin1, HIGH); digitalWrite(motorLinks_pin2, LOW); analogWrite(motorLinks_enable, snelheid);
}

void rijVooruitGecorrigeerd(int snelheidLinks, int snelheidRechts) {
    digitalWrite(motorRechts_pin1, LOW); digitalWrite(motorRechts_pin2, HIGH); analogWrite(motorRechts_enable, constrain(snelheidRechts, 0, 255));
    digitalWrite(motorLinks_pin1, HIGH); digitalWrite(motorLinks_pin2, LOW); analogWrite(motorLinks_enable, constrain(snelheidLinks, 0, 255));
}

void stopMotoren() {
    digitalWrite(motorLinks_pin1, LOW); digitalWrite(motorLinks_pin2, LOW); analogWrite(motorLinks_enable, 0);
    digitalWrite(motorRechts_pin1, LOW); digitalWrite(motorRechts_pin2, LOW); analogWrite(motorRechts_enable, 0);
}

void bochtLinks() {
    digitalWrite(motorRechts_pin1, LOW); digitalWrite(motorRechts_pin2, HIGH); analogWrite(motorRechts_enable, 110);
    digitalWrite(motorLinks_pin1, LOW); digitalWrite(motorLinks_pin2, HIGH); analogWrite(motorLinks_enable, 50);
}

void bochtRechts() {
    digitalWrite(motorLinks_pin1, HIGH); digitalWrite(motorLinks_pin2, LOW); analogWrite(motorLinks_enable, 120); 
    digitalWrite(motorRechts_pin1, HIGH); digitalWrite(motorRechts_pin2, LOW); analogWrite(motorRechts_enable, 50);
}

void pivotLinks(int snelheid) {
    digitalWrite(motorLinks_pin1, LOW); digitalWrite(motorLinks_pin2, HIGH); analogWrite(motorLinks_enable, snelheid);
    digitalWrite(motorRechts_pin1, LOW); digitalWrite(motorRechts_pin2, HIGH); analogWrite(motorRechts_enable, snelheid);
}

void pivotRechts(int snelheid) {
    digitalWrite(motorLinks_pin1, HIGH); digitalWrite(motorLinks_pin2, LOW); analogWrite(motorLinks_enable, snelheid);
    digitalWrite(motorRechts_pin1, HIGH); digitalWrite(motorRechts_pin2, LOW); analogWrite(motorRechts_enable, snelheid);
}

long meetAfstandGefilterd() {
    const int numReadings = 3;
    long readings[numReadings];
    for (int i = 0; i < numReadings; i++) {
        readings[i] = meetAfstand(trigPinVoor, echoPinVoor);
        delay(5);
    }
    for (int i = 0; i < numReadings - 1; i++) {
        for (int j = 0; j < numReadings - i - 1; j++) {
            if (readings[j] > readings[j + 1]) {
                long temp = readings[j];
                readings[j] = readings[j + 1];
                readings[j + 1] = temp;
            }
        }
    }
    return readings[numReadings / 2];
}

long meetAfstand(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW); delayMicroseconds(2);
    digitalWrite(trigPin, HIGH); delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH, 25000);
    if (duration > 0) {
        return duration / 58.2;
    } else {
        return 999;
    }
}

// ===================================================================================
//                         CORE 0 - ACHTERGRONDTAKEN
// ===================================================================================

void verstuurMqttData() {
    if (!client.connected()) { return; }
    
    int rawADC = analogRead(adcPin);
    g_huidigeSpanning = (rawADC / adcMax) * Vref * (R1 + R2) / R2;
    float batterijPercentage = constrain(map(g_huidigeSpanning * 100, 600, 840, 0, 100), 0, 100);
    
    // *** AANPASSING ***: Print batterijpercentage naar Serial Monitor
    Serial.printf("[C0] Batterij: %.1f%%, Spanning: %.2fV\n", batterijPercentage, g_huidigeSpanning);
    
    char batStr[10];
    dtostrf(batterijPercentage, 4, 1, batStr);
    client.publish("esp32/batterij", batStr);
    
    long currentLeft = meetAfstand(trigPinLinks, echoPinLinks);
    long currentRight = meetAfstand(trigPinRechts, echoPinRechts);
    
    char jsonBuffer[200];
    sprintf(jsonBuffer, "{\"front\": %ld, \"left\": %ld, \"right\": %ld, \"voltage\": %.2f, \"paused\": %s}",
            afstandVoor, currentLeft, currentRight, g_huidigeSpanning, isGepauzeerd ? "true" : "false");
    client.publish("esp32/distances_state", jsonBuffer);
    
    char distancePayload[50];
    sprintf(distancePayload, "%ld %ld %ld", afstandVoor, currentLeft, currentRight);
    client.publish("esp32/distances", distancePayload);
}

void updateLedStatus() {
    if (isGepauzeerd) {
        digitalWrite(LED_PIN_GROEN, LOW);
        digitalWrite(LED_PIN_ROOD, LOW);
        static unsigned long lastBlinkMillis = 0;
        if (millis() - lastBlinkMillis > 500) {
            digitalWrite(LED_PIN_GEEL, !digitalRead(LED_PIN_GEEL));
            lastBlinkMillis = millis();
        }
    } else {
        
        int rawADC = analogRead(adcPin);
        float spanning = (rawADC / adcMax) * Vref * (R1 + R2) / R2;
        float batterijPercentage = constrain(map(spanning * 100, 600, 840, 0, 100), 0, 100);

       if (batterijPercentage > 30){
            digitalWrite(LED_PIN_GROEN, HIGH);
            digitalWrite(LED_PIN_GEEL, LOW);
            digitalWrite(LED_PIN_ROOD, LOW);
        } else if (batterijPercentage <10){
            digitalWrite(LED_PIN_GROEN, LOW);
            digitalWrite(LED_PIN_GEEL, LOW);
            digitalWrite(LED_PIN_ROOD, HIGH);
            stopMotoren();
            client.publish("esp32/status", "Batterij leeg!!");
        } else {
            digitalWrite(LED_PIN_GROEN, LOW);
            digitalWrite(LED_PIN_ROOD, LOW);
            digitalWrite(LED_PIN_GEEL, HIGH);
            client.publish("esp32/status", "Batterij bijna leeg!");
        }
    }
}

void verbindOpnieuwMetMqtt() {
    if (WiFi.status() != WL_CONNECTED) { return; }
    int retries = 0;
    while (!client.connected() && retries < 5) {
        retries++;
        if (client.connect(clientID, mqttUser, mqttPassword)) {
            client.subscribe(commandTopic);
            client.publish("esp32/status", "MQTT Verbonden");
        } else {
            delay(2000);
        }
    }
}

void callback(char* topic, byte* payload, unsigned int length) {
    char message[length + 1];
    memcpy(message, payload, length);
    message[length] = '\0';
    String messageStr = String(message);
    Serial.print("Commando ontvangen via MQTT: "); Serial.println(messageStr);
    
    if (messageStr == "toggle_pause") {
        if (isGepauzeerd) { // Als al gepauzeerd, hervat direct
            isGepauzeerd = false;
        } else { // Als actief, start een 30s pauze
            isGepauzeerd = true;
            pauseStartTime = millis();
            needsPostLineAction = false; // Dit is een normale pauze van de gebruiker
            stopMotoren();
            if (client.connected()) client.publish("esp32/status", "Gepauzeerd (MQTT, 30s)");
        }
    }
}

void Core0Task(void* pvParameters) {
    Serial.print("[C0] Achtergrondtaak gestart op core: "); Serial.println(xPortGetCoreID());
    
    for (;;) {
        if (WiFi.status() == WL_CONNECTED) {
            if (!client.connected()) {
                verbindOpnieuwMetMqtt();
            }
            if (client.connected()) {
                client.loop();
            }
        } else {
            delay(1000);
        }

        updateLedStatus();

        bool currentButtonState = digitalRead(startKnopPin);
        if (laatsteKnopStatus == HIGH && currentButtonState == LOW) {
            delay(50);
            currentButtonState = digitalRead(startKnopPin);
            if (currentButtonState == LOW) {
                if (isGepauzeerd) { // Als al gepauzeerd, hervat direct
                    isGepauzeerd = false;
                } else { // Als actief, start een 30s pauze
                    isGepauzeerd = true;
                    pauseStartTime = millis();
                    needsPostLineAction = false; // Dit is een normale pauze van de gebruiker
                    stopMotoren();
                }
            }
        }
        laatsteKnopStatus = currentButtonState;

        if (millis() - previousDataMillis >= dataSendtime) {
            previousDataMillis = millis();

            // --- AANPASSING: Stuur aftelbericht tijdens pauze ---
            if (isGepauzeerd && pauseStartTime != 0) {
                unsigned long elapsedTime = millis() - pauseStartTime;
                if (elapsedTime < 30000) {
                    // Bereken resterende seconden (naar boven afgerond)
                    int secondsRemaining = (30000 - elapsedTime + 999) / 1000;
                    char statusMsg[50];
                    // Formatteer het bericht met de resterende tijd
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
        
        delay(20);
    }
}
