#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>

// --- Globale variabelen ---
const int sensorPins[3] = {27, 26, 25};
int sensorValues[3];
unsigned long previousDataMillis = 0;
const long dataSendtime = 1000;
int buttonPin = 21;

// --- Gedeelde Variabelen tussen Cores (volatile is BELANGRIJK) ---
volatile bool isPaused = false;
volatile float g_huidigeSpanning = 0.0;
volatile long distance1 = 0; // Afstand wordt nu ook gedeeld

// --- Variabelen voor Core 0 ---
bool lastButtonState = HIGH;
int afstObj = 20;

#define trigPin1 15
#define echoPin1 2

// MQTT
const char* ssid = "";
const char* password = "";
const char* mqttServer = "";
const int mqttPort = 1883;
const char* mqttUser = "";
const char* mqttPassword = "";
const char* clientID = "R2-CLASSROOM";
WiFiClient espClient;
PubSubClient client(espClient);

// Motor pinnen
int motor2Pin1 = 5, motor2Pin2 = 18, enable2Pin = 19; // Links
int motor1Pin1 = 16, motor1Pin2 = 4, enable1Pin = 17; // Rechts

// Batterij
const int adcPin = 34;
const float Vref = 3.3, adcMax = 4095.0;
const float R1 = 7000.0, R2 = 2700.0;

// --- Functie declaraties ---
void stop();
void verstuurMqttData();
long getFilteredDistance();
void ontwijkObstakelSlim();
void forward(int speed);
void turn_left();
void turn_right();
void pivot_right(int speed);
void pivot_left(int speed);
void reconect();


// ===================================================================================
// --- TAAK VOOR CORE 0: COMMUNICATIE, SENSOREN & ACHTERGRONDTAKEN ---
// ===================================================================================
void Core0Task(void * pvParameters) {
  Serial.print("[C0] Taak voor Core 0 draait op core: ");
  Serial.println(xPortGetCoreID());

  unsigned long pauseStartTime = 0;
  const unsigned long pauseDuration = 30000;

  for (;;) { // Oneindige lus voor de taken van Core 0
    if (!client.connected()) reconect();
    client.loop();

    bool currentButtonState = digitalRead(buttonPin);
    if (lastButtonState == HIGH && currentButtonState == LOW) {
      delay(20);
      isPaused = !isPaused;
      if (isPaused) {
        pauseStartTime = millis();
        Serial.println("[C0] Gepauzeerd");
        client.publish("esp32/status", "Gepauzeerd");
        stop();
      } else {
        Serial.println("[C0] Robot hervat");
        client.publish("esp32/status", "Robot hervat");
      }
    }
    lastButtonState = currentButtonState;

    if (isPaused && (millis() - pauseStartTime >= pauseDuration)) {
      isPaused = false;
      Serial.println("[C0] 30s voorbij, auto-hervat");
      client.publish("esp32/status", "Auto-hervat na 30s");
    }

    if (millis() - previousDataMillis >= dataSendtime) {
      previousDataMillis = millis();
      verstuurMqttData();
    }
    delay(20);
  }
}

// ===================================================================================
// --- Setup Functie (draait op Core 1) ---
// ===================================================================================
void setup() {
  Serial.begin(115200);
  pinMode(motor1Pin1, OUTPUT); pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT); pinMode(motor2Pin2, OUTPUT);
  pinMode(trigPin1, OUTPUT);   pinMode(echoPin1, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  for (int i = 0; i < 3; i++) { pinMode(sensorPins[i], INPUT); }

  digitalWrite(trigPin1, LOW);
  WiFi.begin(ssid, password);
  analogReadResolution(12);

  Serial.print("Verbinden met WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(250); Serial.print(".");
  }
  Serial.println("\nVerbonden met WiFi");
  client.setServer(mqttServer, mqttPort);
  delay(500);

  xTaskCreatePinnedToCore(Core0Task, "Core0_Task", 10000, NULL, 1, NULL, 0);
}

// ===================================================================================
// --- Hoofd Loop (draait op Core 1): LIJNVOLGEN MET SLIMME OBSTAKELONTWIJKING ---
// ===================================================================================
void loop() {
  if (isPaused) {
    stop();
    delay(10);
    return;
  }
  
  // STAP 1: Meet EERST de afstand op een betrouwbare manier
  distance1 = getFilteredDistance();

  // STAP 2: Reageer als er een obstakel dichtbij is
  if (distance1 < afstObj && distance1 > 0) {
    ontwijkObstakelSlim(); // Roep de nieuwe, slimme functie aan
  } 
  // STAP 3: Als er geen obstakel is, volg dan de lijn
  else {
    for (int i = 0; i < 3; i++) {
      sensorValues[i] = digitalRead(sensorPins[i]);
    }

    if (sensorValues[0] == LOW && sensorValues[1] == HIGH && sensorValues[2] == LOW) {
      forward(100);
    } else if (sensorValues[0] == HIGH && sensorValues[2] == LOW) {
      turn_left();
    } else if (sensorValues[0] == LOW && sensorValues[2] == HIGH) {
      turn_right();
    } else if (sensorValues[0] == HIGH && sensorValues[1] == HIGH && sensorValues[2] == HIGH){
      stop(); delay(3000); forward(100); delay(300);
    } else {
      stop();
    }
  }
}

// ===================================================================================
// --- Hulpfuncties ---
// ===================================================================================

int compenseerSnelheid(int basisSnelheid) {
  const float referentieSpanning = 7.5;
  if (g_huidigeSpanning < 5.0) return basisSnelheid;
  float compensatieFactor = referentieSpanning / g_huidigeSpanning;
  int aangepasteSnelheid = basisSnelheid * compensatieFactor;
  return constrain(aangepasteSnelheid, 0, 255);
}

void forward(int speed) {
  digitalWrite(motor1Pin2, LOW); digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor2Pin2, HIGH); digitalWrite(motor2Pin1, LOW);
  analogWrite(enable1Pin, compenseerSnelheid(speed));
  analogWrite(enable2Pin, compenseerSnelheid(speed + 10));
}

void stop() {
  analogWrite(enable1Pin, 0);
  analogWrite(enable2Pin, 0);
}

void turn_left() {
  digitalWrite(motor1Pin1, LOW); digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, LOW); digitalWrite(motor2Pin2, HIGH);
  analogWrite(enable1Pin, 0);
  analogWrite(enable2Pin, compenseerSnelheid(90));
}

void turn_right() {
  digitalWrite(motor1Pin1, HIGH); digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH); digitalWrite(motor2Pin2, LOW);
  analogWrite(enable1Pin, compenseerSnelheid(120));
  analogWrite(enable2Pin, compenseerSnelheid(0));
}

// --- Nieuwe, preciezere draaifuncties ---
void pivot_right(int speed) {
  digitalWrite(motor1Pin2, HIGH); digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH); digitalWrite(motor2Pin1, LOW);
  int compensatedSpeed = compenseerSnelheid(speed);
  analogWrite(enable1Pin, compensatedSpeed);
  analogWrite(enable2Pin, compensatedSpeed + 10);
}

void pivot_left(int speed) {
  digitalWrite(motor1Pin2, LOW); digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW); digitalWrite(motor2Pin1, HIGH);
  int compensatedSpeed = compenseerSnelheid(speed);
  analogWrite(enable1Pin, compensatedSpeed);
  analogWrite(enable2Pin, compensatedSpeed + 10);
}


/**
 * @brief Meet de afstand met de ultrasone sensor en filtert het resultaat.
 */
long getFilteredDistance() {
  const int numReadings = 5;
  long readings[numReadings];
  
  for (int i = 0; i < numReadings; i++) {
    digitalWrite(trigPin1, LOW); delayMicroseconds(4);
    digitalWrite(trigPin1, HIGH); delayMicroseconds(10);
    digitalWrite(trigPin1, LOW);
    long duration = pulseIn(echoPin1, HIGH, 3000); 
    if (duration > 0) {
      readings[i] = duration / 58.2;
    } else {
      readings[i] = 999;
    }
    delay(10);
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

/**
 * @brief Voert een dynamische ontwijkingsmanoeuvre uit.
 * VERSIE 3: Met een robuustere methode om de lijn terug te vinden.
 */
void ontwijkObstakelSlim() {
  Serial.println("[C1] Start slimme ontwijking...");
  client.publish("esp32/status", "Start slimme ontwijking");

  int driveSpeed = 100;
  int turnSpeed = 120;
  // ! BELANGRIJK: BLIJF DEZE WAARDE KALIBREREN VOOR JOUW ROBOT !
  // Dit is de tijd in milliseconden die nodig is om 90 graden te draaien.
  int turnDuration90deg = 900;

  // 1. Neem afstand
  stop();
  delay(200);

  // 2. Draai 90 graden naar rechts
  Serial.println("  Draai 90 graden rechts");
  pivot_right(turnSpeed);
  delay(turnDuration90deg);
  stop();
  delay(200);

  // 3. Rijd langs het obstakel tot de weg weer vrij is
  Serial.println("  Rijd langs obstakel...");
  bool obstacleCleared = false;
  int checks = 0;

  forward(driveSpeed);
  delay(800); 

  while (!obstacleCleared && checks < 5) {
    stop();
    delay(200);

    Serial.println("    Kijk of de weg vrij is...");
    pivot_left(turnSpeed);
    delay(turnDuration90deg);
    stop();
    delay(300);

    long distanceAhead = getFilteredDistance();
    Serial.print("    Afstand vooruit: "); Serial.println(distanceAhead);

    if (distanceAhead > afstObj + 20) {
      obstacleCleared = true;
      Serial.println("  Weg is vrij! Robot staat nu weer in de oorspronkelijke rijrichting.");
    } else {
      Serial.println("  Nog niet voorbij, rijd verder...");
      pivot_right(turnSpeed);
      delay(turnDuration90deg);
      stop();
      delay(200);
      forward(driveSpeed);
      delay(500);
    }
    checks++;
  }

  // 4. Keer terug en zoek de lijn
  if (obstacleCleared) {
    Serial.println("  Rond manoeuvre af, rijd stukje rechtdoor");
    forward(driveSpeed);
    delay(700);
    stop();
    delay(200);

    Serial.println("  Draai links, richting de lijn");
    pivot_left(turnSpeed);
    delay(turnDuration90deg);
    stop();
    delay(200);
    
    Serial.println("  Zoek de lijn door vooruit te rijden...");
    forward(driveSpeed - 20);
    unsigned long searchStartTime = millis();
    bool lineFound = false;

    while (millis() - searchStartTime < 4000 && !lineFound) {
      if (digitalRead(sensorPins[1]) == HIGH) { // Check de MIDDELSTE sensor
        lineFound = true;
        Serial.println("  Middelste sensor heeft de lijn gevonden!");
        forward(driveSpeed - 20);
        delay(50);
      }
    }
    
    if (!lineFound) {
      Serial.println("  Lijn niet gevonden na actieve zoekactie, stop voor veiligheid.");
    }

  } else {
    Serial.println("  Kon obstakel niet passeren, stop voor veiligheid.");
  }
  
  stop();
  delay(500);
  Serial.println("[C1] Ontwijking voltooid.");
  client.publish("esp32/status", "Ontwijking voltooid");
}


void verstuurMqttData() {
  int rawADC = analogRead(adcPin);
  float voltageAtADC = (rawADC / adcMax) * Vref;
  g_huidigeSpanning = voltageAtADC * (R1 + R2) / R2;
  
  float batterij = map(g_huidigeSpanning * 100, 600, 840, 0, 100);
  batterij = constrain(batterij, 0, 100);
  
  Serial.print("[C0] Spanning: "); Serial.print(g_huidigeSpanning, 2);
  Serial.print("V, Afstand: "); Serial.println(distance1);
  
  char batStr[6];
  dtostrf(batterij, 4, 1, batStr);
  client.publish("esp32/batterij", batStr);

  char ultStr[6];
  if (distance1 >= 999) {
    strcpy(ultStr, "---");
  } else {
    dtostrf(distance1, 4, 1, ultStr);
  }
  client.publish("esp32/ultrasone", ultStr);
}

void reconect() {
  while (!client.connected()) {
    Serial.println("[C0] Verbinden met MQTT...");
    if (client.connect(clientID, mqttUser, mqttPassword)) {
      Serial.println("[C0] Verbonden");
    } else {
      Serial.print("[C0] Mislukt, rc=");
      Serial.print(client.state());
      Serial.println(" Probeer opnieuw in 2s");
      delay(2000);
    }
  }
}
