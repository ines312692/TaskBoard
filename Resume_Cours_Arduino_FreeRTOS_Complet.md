# RÉSUMÉ COMPLET : Arduino, FreeRTOS, Communication et Systèmes Embarqués

## TABLE DES MATIÈRES

1. [Programmation sous Arduino](#1-programmation-sous-arduino)
2. [Interruptions et Timers Logiciels](#2-interruptions-et-timers-logiciels)
3. [Programmation avec FreeRTOS - Partie I](#3-programmation-avec-freertos---partie-i)
4. [Programmation avec FreeRTOS - Partie II](#4-programmation-avec-freertos---partie-ii)
5. [Communication MQTT sur ESP32](#5-communication-mqtt-sur-esp32)
6. [Exercices Corrigés](#6-exercices-corriges)
7. [Exemple de Devoir Surveillé (DS)](#7-exemple-de-devoir-surveille)

---

## 1. PROGRAMMATION SOUS ARDUINO

### 1.1 Structure d'un programme Arduino

Un programme Arduino (appelé **sketch**) se compose de deux fonctions principales :

```cpp
void setup() {
    // Code d'initialisation exécuté une seule fois
    // Configuration des broches, communication série, etc.
}

void loop() {
    // Code exécuté en boucle indéfiniment
    // Logique principale du programme
}
```

### 1.2 Entrées/Sorties Numériques

#### Fonctions principales :

**pinMode(broche, mode)** : Configure une broche
- `broche` : numéro de la broche (0-13 pour Arduino Uno)
- `mode` : INPUT (entrée) ou OUTPUT (sortie)

```cpp
pinMode(13, OUTPUT);  // Configure la broche 13 en sortie
pinMode(2, INPUT);    // Configure la broche 2 en entrée
```

**digitalWrite(broche, valeur)** : Écrit un niveau logique
- `valeur` : HIGH (5V) ou LOW (0V)

```cpp
digitalWrite(13, HIGH);  // Allume une LED
digitalWrite(13, LOW);   // Éteint une LED
```

**int digitalRead(broche)** : Lit l'état d'une broche
- Retourne HIGH ou LOW

```cpp
int etat = digitalRead(2);  // Lit l'état du bouton sur la broche 2
```

### 1.3 Entrées/Sorties Analogiques

**int analogRead(broche_analog)** : Lit une tension analogique
- Convertit 0-5V en valeur 0-1023 (résolution 10 bits)
- Broches A0-A5 sur Arduino Uno

```cpp
int valeur = analogRead(A0);  // Lit la valeur du potentiomètre
```

**analogWrite(broche, valeur)** : Génère un signal PWM
- `broche` : 3, 5, 6, 9, 10, ou 11 (broches PWM)
- `valeur` : 0-255 (0% à 100% du cycle de travail)

```cpp
analogWrite(9, 128);  // 50% du cycle de travail (127.5/255)
```

#### Le signal PWM (Pulse Width Modulation)

Le PWM permet de simuler une tension analogique variable en faisant varier le rapport cyclique d'un signal numérique :
- 0 = toujours BAS (0V)
- 128 = 50% HAUT, 50% BAS (≈2.5V en moyenne)
- 255 = toujours HAUT (5V)

### 1.4 Fonctions de temps

**delay(ms)** : Pause l'exécution
- `ms` : durée en millisecondes

```cpp
delay(1000);  // Attendre 1 seconde
```

**unsigned long millis()** : Temps écoulé depuis le démarrage
- Retourne le nombre de millisecondes

```cpp
unsigned long temps = millis();
```

### 1.5 Communication Série

La librairie **Serial** permet la communication avec l'ordinateur via USB :

**Serial.begin(vitesse)** : Initialise la communication
```cpp
Serial.begin(9600);  // 9600 bauds
```

**int Serial.available()** : Nombre d'octets disponibles en réception

**int Serial.read()** : Lit un octet reçu

**Serial.print(data)** : Envoie des données
**Serial.println(data)** : Envoie des données avec retour à la ligne

```cpp
Serial.print("Valeur: ");
Serial.println(123);  // Affiche "Valeur: 123"
```

**Serial.print(val, format)** : Format d'affichage
- DEC : décimal
- HEX : hexadécimal
- BIN : binaire

```cpp
Serial.print(255, HEX);  // Affiche "FF"
```

### 1.6 Variables et constantes

#### Niveaux logiques :
- **HIGH** = 1 (5V)
- **LOW** = 0 (0V)
- **TRUE** = 1
- **FALSE** = 0

#### Types de données courants :
```cpp
int temperature = 25;        // Entier (-32768 à 32767)
float tension = 3.3;         // Nombre flottant
bool etatLED = true;         // Booléen
char caractere = 'A';        // Caractère
unsigned long temps = 0;     // Entier non signé (0 à 4294967295)
```

---

## 2. INTERRUPTIONS ET TIMERS LOGICIELS

### 2.1 Concept d'interruption

Une **interruption** est un événement qui interrompt temporairement l'exécution normale du programme pour exécuter une routine d'interruption (ISR - Interrupt Service Routine).

**Avantages** :
- Réponse rapide aux événements externes
- Pas de polling constant nécessaire
- Économie d'énergie

### 2.2 Interruptions sous Arduino

#### Fonction attachInterrupt()

```cpp
attachInterrupt(digitalPinToInterrupt(pin), ISR, mode);
```

**Paramètres** :
- `pin` : numéro de la broche (2 et 3 sur Arduino Uno)
- `ISR` : fonction à exécuter (sans paramètre et sans retour)
- `mode` : type de déclenchement
  - **RISING** : front montant (LOW → HIGH)
  - **FALLING** : front descendant (HIGH → LOW)
  - **CHANGE** : tout changement d'état

**digitalPinToInterrupt(pin)** : Convertit le numéro de broche en numéro d'interruption

#### Exemple d'interruption

```cpp
volatile bool etat = false;  // volatile pour les variables partagées avec ISR

void setup() {
    pinMode(2, INPUT_PULLUP);
    pinMode(13, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(2), changementEtat, FALLING);
}

void loop() {
    if (etat) {
        digitalWrite(13, HIGH);
    } else {
        digitalWrite(13, LOW);
    }
}

void changementEtat() {
    etat = !etat;  // Inverse l'état
}
```

#### Règles importantes pour les ISR :
1. Doivent être courtes et rapides
2. Pas de delay()
3. Pas de Serial.print()
4. Variables partagées : utiliser `volatile`
5. Retour `void` et pas de paramètres

### 2.3 Interruptions avec FreeRTOS

Les ISR avec FreeRTOS utilisent des fonctions spéciales suffixées par **FromISR** :

#### Fonctions FromISR principales :

**xQueueSendFromISR()** : Envoyer dans une file depuis une ISR
```cpp
void IRAM_ATTR buttonISR() {
    int valeur = 1;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(queue, &valeur, &xHigherPriorityTaskWoken);
    
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();  // Force un changement de contexte
    }
}
```

**xSemaphoreGiveFromISR()** : Donner un sémaphore depuis une ISR
```cpp
void IRAM_ATTR timerISR() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(semaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
```

### 2.4 Timers Logiciels avec FreeRTOS

Les **timers logiciels** permettent d'exécuter des fonctions de callback à intervalles réguliers sans utiliser le hardware.

#### Types de timers :

1. **One-Shot** : S'exécute une seule fois
2. **Auto-Reload** : S'exécute périodiquement

#### Création d'un timer

```cpp
TimerHandle_t xTimer;

void vTimerCallback(TimerHandle_t xTimer) {
    // Code à exécuter périodiquement
    Serial.println("Timer exécuté!");
}

void setup() {
    // Créer un timer auto-reload de 1000ms
    xTimer = xTimerCreate(
        "MonTimer",              // Nom du timer
        pdMS_TO_TICKS(1000),    // Période en ticks
        pdTRUE,                 // Auto-reload (pdTRUE) ou One-shot (pdFALSE)
        (void *)0,              // ID du timer
        vTimerCallback          // Fonction callback
    );
    
    // Démarrer le timer
    if (xTimer != NULL) {
        xTimerStart(xTimer, 0);
    }
}
```

#### Fonctions de gestion des timers :

**xTimerCreate()** : Créer un timer
**xTimerStart()** : Démarrer un timer dormant
**xTimerStop()** : Arrêter un timer en cours
**xTimerReset()** : Redémarrer un timer
**xTimerDelete()** : Supprimer un timer
**xTimerChangePeriod()** : Modifier la période d'un timer

#### Exemple de timer périodique

```cpp
#include <Arduino_FreeRTOS.h>
#include <timers.h>

TimerHandle_t ledTimer;
int ledState = LOW;

void ledToggleCallback(TimerHandle_t xTimer) {
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    
    ledTimer = xTimerCreate(
        "LEDTimer",
        pdMS_TO_TICKS(500),  // 500ms
        pdTRUE,              // Auto-reload
        (void *)0,
        ledToggleCallback
    );
    
    xTimerStart(ledTimer, 0);
}

void loop() {
    // Le timer s'exécute automatiquement
}
```

---

## 3. PROGRAMMATION AVEC FreeRTOS - PARTIE I

### 3.1 Introduction à FreeRTOS

**FreeRTOS** (Free Real-Time Operating System) est un système d'exploitation temps réel open-source pour microcontrôleurs.

**Avantages** :
- Multitâche préemptif
- Gestion des priorités
- Communication inter-tâches
- Synchronisation
- Gestion optimisée des ressources

### 3.2 Création de tâches

#### Structure d'une tâche

```cpp
void TaskName(void *pvParameters) {
    // Initialisation de la tâche
    
    for (;;) {  // Boucle infinie
        // Code de la tâche
        
        vTaskDelay(pdMS_TO_TICKS(100));  // Délai de 100ms
    }
    
    // Ne devrait jamais arriver ici
    vTaskDelete(NULL);
}
```

#### Fonction xTaskCreate()

```cpp
BaseType_t xTaskCreate(
    TaskFunction_t pvTaskCode,    // Pointeur vers la fonction de la tâche
    const char * const pcName,    // Nom de la tâche (pour debug)
    uint16_t usStackDepth,        // Taille de la pile (en mots)
    void *pvParameters,           // Paramètre passé à la tâche
    UBaseType_t uxPriority,       // Priorité (0 = plus basse)
    TaskHandle_t *pxCreatedTask   // Handle de la tâche (peut être NULL)
);
```

#### Exemple de création de tâches

```cpp
#include <Arduino_FreeRTOS.h>

void TaskBlink(void *pvParameters);
void TaskSerial(void *pvParameters);

void setup() {
    Serial.begin(9600);
    
    // Créer deux tâches
    xTaskCreate(
        TaskBlink,
        "Blink",
        128,      // Taille de pile
        NULL,
        1,        // Priorité
        NULL
    );
    
    xTaskCreate(
        TaskSerial,
        "Serial",
        128,
        NULL,
        2,        // Priorité plus haute
        NULL
    );
    
    // Le scheduler démarre automatiquement
}

void loop() {
    // Vide - le scheduler gère tout
}

void TaskBlink(void *pvParameters) {
    pinMode(LED_BUILTIN, OUTPUT);
    
    for (;;) {
        digitalWrite(LED_BUILTIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(500));
        digitalWrite(LED_BUILTIN, LOW);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void TaskSerial(void *pvParameters) {
    for (;;) {
        Serial.println("Tâche Serial");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

### 3.3 Gestion des priorités

- Les priorités vont de **0** (la plus basse) à **configMAX_PRIORITIES - 1**
- Sur Arduino : généralement 0 à 3
- Tâche de plus haute priorité s'exécute en premier
- Si même priorité : ordonnancement à tour de rôle (round-robin)

#### Modification de priorité

```cpp
vTaskPrioritySet(TaskHandle_t xTask, UBaseType_t uxNewPriority);
UBaseType_t uxTaskPriorityGet(TaskHandle_t xTask);
```

### 3.4 États des tâches

Une tâche peut être dans 4 états :

1. **Running** : En cours d'exécution
2. **Ready** : Prête à s'exécuter
3. **Blocked** : En attente d'un événement
4. **Suspended** : Suspendue

```cpp
vTaskSuspend(TaskHandle_t xTask);  // Suspendre
vTaskResume(TaskHandle_t xTask);   // Reprendre
```

### 3.5 Délais et temporisation

#### vTaskDelay()

```cpp
vTaskDelay(TickType_t xTicksToDelay);
```

Bloque la tâche pendant un nombre de ticks défini.

```cpp
vTaskDelay(pdMS_TO_TICKS(1000));  // 1 seconde
```

#### vTaskDelayUntil()

Délai absolu pour exécutions périodiques précises :

```cpp
void TaskPeriodique(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100);
    
    for (;;) {
        // Code exécuté exactement toutes les 100ms
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
```

---

## 4. PROGRAMMATION AVEC FreeRTOS - PARTIE II

### 4.1 Files d'attente (Queues)

Les **queues** permettent la communication entre tâches de manière sûre (thread-safe).

#### Création d'une queue

```cpp
QueueHandle_t xQueue;

xQueue = xQueueCreate(
    10,              // Nombre d'éléments
    sizeof(int)      // Taille de chaque élément
);
```

#### Envoi de données

```cpp
int valeur = 42;
BaseType_t status = xQueueSend(
    xQueue,                      // Handle de la queue
    &valeur,                     // Pointeur vers les données
    pdMS_TO_TICKS(100)          // Timeout (0 pour non-bloquant)
);

if (status == pdPASS) {
    // Envoi réussi
}
```

#### Réception de données

```cpp
int valeurRecue;
BaseType_t status = xQueueReceive(
    xQueue,
    &valeurRecue,
    pdMS_TO_TICKS(100)
);

if (status == pdPASS) {
    // Réception réussie
    Serial.println(valeurRecue);
}
```

#### Exemple complet avec queue

```cpp
#include <Arduino_FreeRTOS.h>
#include <queue.h>

QueueHandle_t xQueue;

void TaskProducer(void *pvParameters) {
    int counter = 0;
    
    for (;;) {
        xQueueSend(xQueue, &counter, portMAX_DELAY);
        counter++;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void TaskConsumer(void *pvParameters) {
    int receivedValue;
    
    for (;;) {
        if (xQueueReceive(xQueue, &receivedValue, portMAX_DELAY) == pdPASS) {
            Serial.print("Reçu: ");
            Serial.println(receivedValue);
        }
    }
}

void setup() {
    Serial.begin(9600);
    
    xQueue = xQueueCreate(10, sizeof(int));
    
    if (xQueue != NULL) {
        xTaskCreate(TaskProducer, "Producer", 128, NULL, 1, NULL);
        xTaskCreate(TaskConsumer, "Consumer", 128, NULL, 1, NULL);
    }
}

void loop() {}
```

### 4.2 Sémaphores

Les **sémaphores** sont utilisés pour la synchronisation et l'exclusion mutuelle.

#### Types de sémaphores :

1. **Sémaphore binaire** : 0 ou 1 (synchronisation)
2. **Sémaphore compteur** : 0 à N (ressources multiples)
3. **Mutex** : exclusion mutuelle (un seul accédant)

#### Sémaphore binaire

```cpp
#include <semphr.h>

SemaphoreHandle_t xSemaphore;

void setup() {
    xSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(xSemaphore);  // Initialiser à disponible
}

void TaskExample(void *pvParameters) {
    for (;;) {
        if (xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
            // Section critique
            
            xSemaphoreGive(xSemaphore);  // Libérer
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

#### Mutex

```cpp
SemaphoreHandle_t xMutex;

void setup() {
    xMutex = xSemaphoreCreateMutex();
}

void TaskA(void *pvParameters) {
    for (;;) {
        if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
            // Accès exclusif à la ressource partagée
            Serial.println("Task A");
            
            xSemaphoreGive(xMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

#### Sémaphore compteur

```cpp
SemaphoreHandle_t xCountingSemaphore;

void setup() {
    xCountingSemaphore = xSemaphoreCreateCounting(
        5,  // Valeur maximale
        0   // Valeur initiale
    );
}
```

### 4.3 Synchronisation entre ISR et tâches

#### Exemple avec sémaphore binaire

```cpp
SemaphoreHandle_t xBinarySemaphore;

void IRAM_ATTR buttonISR() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xBinarySemaphore, &xHigherPriorityTaskWoken);
    
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

void TaskHandler(void *pvParameters) {
    for (;;) {
        // Attend le sémaphore donné par l'ISR
        if (xSemaphoreTake(xBinarySemaphore, portMAX_DELAY) == pdTRUE) {
            Serial.println("Bouton pressé!");
        }
    }
}

void setup() {
    Serial.begin(9600);
    xBinarySemaphore = xSemaphoreCreateBinary();
    
    pinMode(2, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(2), buttonISR, FALLING);
    
    xTaskCreate(TaskHandler, "Handler", 128, NULL, 1, NULL);
}

void loop() {}
```

### 4.4 Groupes d'événements (Event Groups)

Les **Event Groups** permettent de synchroniser plusieurs événements.

```cpp
#include <event_groups.h>

EventGroupHandle_t xEventGroup;

#define BIT_0 (1 << 0)
#define BIT_1 (1 << 1)
#define BIT_2 (1 << 2)

void setup() {
    xEventGroup = xEventGroupCreate();
}

// Définir des bits
void TaskSetter(void *pvParameters) {
    for (;;) {
        xEventGroupSetBits(xEventGroup, BIT_0 | BIT_1);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Attendre des bits
void TaskWaiter(void *pvParameters) {
    for (;;) {
        EventBits_t uxBits = xEventGroupWaitBits(
            xEventGroup,
            BIT_0 | BIT_1,  // Bits à attendre
            pdTRUE,         // Clear on exit
            pdTRUE,         // Attendre tous les bits (AND)
            portMAX_DELAY
        );
        
        Serial.println("Événements reçus!");
    }
}
```

---

## 5. COMMUNICATION MQTT SUR ESP32

### 5.1 Introduction à MQTT

**MQTT** (Message Queuing Telemetry Transport) est un protocole de messagerie léger pour l'IoT.

**Architecture** :
- **Broker** : Serveur central (ex: Mosquitto, HiveMQ)
- **Publisher** : Publie des messages sur des topics
- **Subscriber** : S'abonne à des topics

**Qualité de Service (QoS)** :
- **QoS 0** : Au plus une fois (fire and forget)
- **QoS 1** : Au moins une fois (avec accusé)
- **QoS 2** : Exactement une fois (double accusé)

### 5.2 Configuration WiFi sur ESP32

```cpp
#include <WiFi.h>

const char* ssid = "VotreSSID";
const char* password = "VotreMotDePasse";

void setup() {
    Serial.begin(115200);
    
    WiFi.begin(ssid, password);
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    
    Serial.println("");
    Serial.println("WiFi connecté");
    Serial.print("Adresse IP: ");
    Serial.println(WiFi.localIP());
}
```

### 5.3 Client MQTT avec PubSubClient

```cpp
#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "VotreSSID";
const char* password = "VotreMotDePasse";
const char* mqtt_server = "broker.hivemq.com";

WiFiClient espClient;
PubSubClient client(espClient);

void callback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message reçu [");
    Serial.print(topic);
    Serial.print("]: ");
    
    for (int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();
}

void reconnect() {
    while (!client.connected()) {
        Serial.print("Connexion MQTT...");
        
        if (client.connect("ESP32Client")) {
            Serial.println("connecté");
            client.subscribe("esp32/test");
        } else {
            Serial.print("échec, rc=");
            Serial.print(client.state());
            Serial.println(" réessai dans 5s");
            delay(5000);
        }
    }
}

void setup() {
    Serial.begin(115200);
    
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("WiFi connecté");
    
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
}

void loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();
    
    // Publier un message toutes les 5 secondes
    static unsigned long lastMsg = 0;
    unsigned long now = millis();
    
    if (now - lastMsg > 5000) {
        lastMsg = now;
        
        String msg = "Température: " + String(random(20, 30));
        client.publish("esp32/temperature", msg.c_str());
    }
}
```

### 5.4 Topics MQTT

**Format** : `maison/salon/temperature`

**Wildcards** :
- `+` : Un niveau (`maison/+/temperature`)
- `#` : Plusieurs niveaux (`maison/#`)

```cpp
// S'abonner à plusieurs topics
client.subscribe("maison/+/temperature");
client.subscribe("capteurs/#");
```

### 5.5 Publication de données JSON

```cpp
#include <ArduinoJson.h>

void publishSensorData() {
    StaticJsonDocument<200> doc;
    
    doc["temperature"] = 25.5;
    doc["humidity"] = 60;
    doc["timestamp"] = millis();
    
    char buffer[200];
    serializeJson(doc, buffer);
    
    client.publish("esp32/sensors", buffer);
}
```

---

## 6. EXERCICES CORRIGÉS

### Exercice 1 : Feu tricolore simple

**Énoncé** : Créer un feu tricolore avec les séquences suivantes :
- Rouge : 5 secondes
- Vert : 5 secondes
- Orange : 2 secondes

**Solution** :

```cpp
// Définition des broches
#define LED_ROUGE 10
#define LED_ORANGE 11
#define LED_VERTE 12

void setup() {
    pinMode(LED_ROUGE, OUTPUT);
    pinMode(LED_ORANGE, OUTPUT);
    pinMode(LED_VERTE, OUTPUT);
}

void loop() {
    // Feu rouge
    digitalWrite(LED_ROUGE, HIGH);
    digitalWrite(LED_ORANGE, LOW);
    digitalWrite(LED_VERTE, LOW);
    delay(5000);
    
    // Feu vert
    digitalWrite(LED_ROUGE, LOW);
    digitalWrite(LED_ORANGE, LOW);
    digitalWrite(LED_VERTE, HIGH);
    delay(5000);
    
    // Feu orange
    digitalWrite(LED_ROUGE, LOW);
    digitalWrite(LED_ORANGE, HIGH);
    digitalWrite(LED_VERTE, LOW);
    delay(2000);
}
```

---

### Exercice 2 : Lecture de potentiomètre et contrôle LED

**Énoncé** : Lire la valeur d'un potentiomètre sur A0 et contrôler la luminosité d'une LED sur la broche 9 (PWM).

**Solution** :

```cpp
const int potPin = A0;
const int ledPin = 9;

void setup() {
    pinMode(ledPin, OUTPUT);
    Serial.begin(9600);
}

void loop() {
    int valeurPot = analogRead(potPin);      // 0-1023
    int luminosite = map(valeurPot, 0, 1023, 0, 255);  // Conversion 0-255
    
    analogWrite(ledPin, luminosite);
    
    Serial.print("Potentiomètre: ");
    Serial.print(valeurPot);
    Serial.print(" | Luminosité LED: ");
    Serial.println(luminosite);
    
    delay(100);
}
```

---

### Exercice 3 : Compteur avec bouton et interruption

**Énoncé** : Créer un compteur qui s'incrémente à chaque pression sur un bouton (broche 2) et affiche le résultat sur le moniteur série. Utiliser une interruption.

**Solution** :

```cpp
const int buttonPin = 2;
volatile int compteur = 0;

void setup() {
    Serial.begin(9600);
    pinMode(buttonPin, INPUT_PULLUP);
    
    attachInterrupt(digitalPinToInterrupt(buttonPin), incrementer, FALLING);
    
    Serial.println("Compteur initialisé");
}

void loop() {
    static int dernierCompteur = -1;
    
    if (compteur != dernierCompteur) {
        Serial.print("Compteur: ");
        Serial.println(compteur);
        dernierCompteur = compteur;
    }
    
    delay(100);
}

void incrementer() {
    static unsigned long dernierDebounce = 0;
    unsigned long temps = millis();
    
    // Anti-rebond (debouncing)
    if (temps - dernierDebounce > 200) {
        compteur++;
        dernierDebounce = temps;
    }
}
```

---

### Exercice 4 : Multitâche avec FreeRTOS - Blink et Serial

**Énoncé** : Créer deux tâches FreeRTOS :
1. Une tâche qui fait clignoter une LED toutes les 500ms
2. Une tâche qui affiche un message série toutes les secondes

**Solution** :

```cpp
#include <Arduino_FreeRTOS.h>

void TaskBlink(void *pvParameters);
void TaskSerial(void *pvParameters);

void setup() {
    Serial.begin(9600);
    
    xTaskCreate(TaskBlink, "Blink", 128, NULL, 1, NULL);
    xTaskCreate(TaskSerial, "Serial", 128, NULL, 1, NULL);
}

void loop() {
    // Vide
}

void TaskBlink(void *pvParameters) {
    pinMode(LED_BUILTIN, OUTPUT);
    
    for (;;) {
        digitalWrite(LED_BUILTIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(500));
        digitalWrite(LED_BUILTIN, LOW);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void TaskSerial(void *pvParameters) {
    int compteur = 0;
    
    for (;;) {
        Serial.print("Message #");
        Serial.println(compteur++);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

---

### Exercice 5 : Communication entre tâches avec Queue

**Énoncé** : Créer deux tâches :
- Tâche 1 : Lit la température d'un capteur (simulé) et l'envoie dans une queue
- Tâche 2 : Reçoit la température et l'affiche sur le moniteur série

**Solution** :

```cpp
#include <Arduino_FreeRTOS.h>
#include <queue.h>

QueueHandle_t temperatureQueue;

void TaskReadSensor(void *pvParameters);
void TaskDisplayTemp(void *pvParameters);

void setup() {
    Serial.begin(9600);
    
    // Créer une queue de 10 éléments de type float
    temperatureQueue = xQueueCreate(10, sizeof(float));
    
    if (temperatureQueue != NULL) {
        xTaskCreate(TaskReadSensor, "ReadSensor", 128, NULL, 2, NULL);
        xTaskCreate(TaskDisplayTemp, "DisplayTemp", 128, NULL, 1, NULL);
    } else {
        Serial.println("Erreur création queue");
    }
}

void loop() {}

void TaskReadSensor(void *pvParameters) {
    for (;;) {
        // Simuler une lecture de capteur
        float temperature = 20.0 + (random(0, 100) / 10.0);
        
        // Envoyer dans la queue
        if (xQueueSend(temperatureQueue, &temperature, pdMS_TO_TICKS(100)) == pdPASS) {
            Serial.print("Température lue: ");
            Serial.println(temperature);
        }
        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void TaskDisplayTemp(void *pvParameters) {
    float receivedTemp;
    
    for (;;) {
        // Recevoir de la queue
        if (xQueueReceive(temperatureQueue, &receivedTemp, portMAX_DELAY) == pdPASS) {
            Serial.print(">>> Affichage température: ");
            Serial.print(receivedTemp);
            Serial.println(" °C");
        }
    }
}
```

---

### Exercice 6 : Synchronisation avec sémaphore

**Énoncé** : Créer un système où un bouton déclenche une interruption qui donne un sémaphore. Une tâche attend ce sémaphore pour allumer une LED pendant 2 secondes.

**Solution** :

```cpp
#include <Arduino_FreeRTOS.h>
#include <semphr.h>

const int buttonPin = 2;
const int ledPin = 13;

SemaphoreHandle_t xBinarySemaphore;

void IRAM_ATTR buttonISR() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xBinarySemaphore, &xHigherPriorityTaskWoken);
    
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

void TaskLED(void *pvParameters) {
    pinMode(ledPin, OUTPUT);
    
    for (;;) {
        // Attendre le sémaphore
        if (xSemaphoreTake(xBinarySemaphore, portMAX_DELAY) == pdTRUE) {
            Serial.println("LED allumée");
            digitalWrite(ledPin, HIGH);
            vTaskDelay(pdMS_TO_TICKS(2000));
            digitalWrite(ledPin, LOW);
            Serial.println("LED éteinte");
        }
    }
}

void setup() {
    Serial.begin(9600);
    
    pinMode(buttonPin, INPUT_PULLUP);
    
    // Créer le sémaphore binaire
    xBinarySemaphore = xSemaphoreCreateBinary();
    
    if (xBinarySemaphore != NULL) {
        // Attacher l'interruption
        attachInterrupt(digitalPinToInterrupt(buttonPin), buttonISR, FALLING);
        
        // Créer la tâche
        xTaskCreate(TaskLED, "LED_Task", 128, NULL, 1, NULL);
    }
}

void loop() {}
```

---

### Exercice 7 : Timer logiciel périodique

**Énoncé** : Créer un timer logiciel qui inverse l'état d'une LED toutes les 300ms.

**Solution** :

```cpp
#include <Arduino_FreeRTOS.h>
#include <timers.h>

const int ledPin = 13;
int ledState = LOW;

TimerHandle_t ledTimer;

void ledTimerCallback(TimerHandle_t xTimer) {
    ledState = !ledState;
    digitalWrite(ledPin, ledState);
}

void setup() {
    pinMode(ledPin, OUTPUT);
    
    // Créer un timer auto-reload de 300ms
    ledTimer = xTimerCreate(
        "LEDTimer",
        pdMS_TO_TICKS(300),
        pdTRUE,              // Auto-reload
        (void *)0,
        ledTimerCallback
    );
    
    if (ledTimer != NULL) {
        xTimerStart(ledTimer, 0);
    }
}

void loop() {}
```

---

### Exercice 8 : Client MQTT - Capteur de température

**Énoncé** : Créer un client MQTT sur ESP32 qui :
- Publie une température aléatoire toutes les 10 secondes
- S'abonne au topic "esp32/led" pour contrôler une LED

**Solution** :

```cpp
#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "VotreSSID";
const char* password = "VotreMotDePasse";
const char* mqtt_server = "broker.hivemq.com";

const int ledPin = 2;

WiFiClient espClient;
PubSubClient client(espClient);

void callback(char* topic, byte* payload, unsigned int length) {
    String message;
    for (int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    
    Serial.print("Message reçu sur ");
    Serial.print(topic);
    Serial.print(": ");
    Serial.println(message);
    
    if (String(topic) == "esp32/led") {
        if (message == "ON") {
            digitalWrite(ledPin, HIGH);
            Serial.println("LED allumée");
        } else if (message == "OFF") {
            digitalWrite(ledPin, LOW);
            Serial.println("LED éteinte");
        }
    }
}

void reconnect() {
    while (!client.connected()) {
        Serial.print("Connexion MQTT...");
        
        if (client.connect("ESP32TempSensor")) {
            Serial.println("connecté");
            client.subscribe("esp32/led");
            client.publish("esp32/status", "ESP32 connecté");
        } else {
            Serial.print("échec, rc=");
            Serial.print(client.state());
            Serial.println(" réessai dans 5s");
            delay(5000);
        }
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(ledPin, OUTPUT);
    
    // Connexion WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connecté");
    
    // Configuration MQTT
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
}

void loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();
    
    // Publier la température toutes les 10 secondes
    static unsigned long lastMsg = 0;
    unsigned long now = millis();
    
    if (now - lastMsg > 10000) {
        lastMsg = now;
        
        float temperature = 20.0 + random(0, 100) / 10.0;
        String msg = String(temperature, 1);
        
        Serial.print("Publication température: ");
        Serial.println(msg);
        
        client.publish("esp32/temperature", msg.c_str());
    }
}
```

---

## 7. EXEMPLE DE DEVOIR SURVEILLÉ (DS)

### DS : Système de Gestion de Parking Intelligent

**Durée : 2 heures**  
**Documents autorisés : Aucun**  
**Matériel : Arduino Uno / ESP32**

---

#### PARTIE 1 : Questions de cours (6 points)

**Question 1** (2 points)  
Expliquez la différence entre `delay()` et `vTaskDelay()` en FreeRTOS. Pourquoi ne doit-on pas utiliser `delay()` dans une tâche FreeRTOS ?

**Réponse attendue** :
- `delay()` : bloque toute l'exécution du microcontrôleur
- `vTaskDelay()` : bloque uniquement la tâche courante, permettant aux autres tâches de s'exécuter
- En FreeRTOS, on utilise `vTaskDelay()` pour libérer le CPU et permettre le multitâche préemptif
- `delay()` empêche le scheduler de fonctionner correctement

---

**Question 2** (2 points)  
Qu'est-ce qu'une interruption ? Donnez deux avantages et deux contraintes des routines d'interruption (ISR).

**Réponse attendue** :

*Définition* : Une interruption est un événement matériel ou logiciel qui interrompt l'exécution normale du programme pour exécuter une routine spéciale (ISR).

*Avantages* :
1. Réponse rapide aux événements externes
2. Économie d'énergie (pas de polling)

*Contraintes* :
1. Doivent être courtes et rapides
2. Ne peuvent pas utiliser certaines fonctions (Serial.print, delay, etc.)

---

**Question 3** (2 points)  
Dans le protocole MQTT, expliquez les trois niveaux de QoS (Quality of Service) et donnez un cas d'usage pour chacun.

**Réponse attendue** :

1. **QoS 0** (Au plus une fois) : Le message est envoyé sans garantie de réception
   - Cas d'usage : données de température non critiques

2. **QoS 1** (Au moins une fois) : Le message est garanti d'arriver, mais peut être dupliqué
   - Cas d'usage : alarmes importantes

3. **QoS 2** (Exactement une fois) : Le message arrive exactement une fois, sans duplication
   - Cas d'usage : transactions financières, commandes critiques

---

#### PARTIE 2 : Exercice Arduino de base (4 points)

**Énoncé** :  
Écrire un programme Arduino qui :
- Lit la valeur d'un capteur de distance ultrason (trigger sur pin 7, echo sur pin 8)
- Allume une LED rouge (pin 10) si la distance est < 10 cm
- Allume une LED verte (pin 11) si la distance est >= 10 cm
- Affiche la distance sur le moniteur série

**Solution attendue** :

```cpp
const int trigPin = 7;
const int echoPin = 8;
const int ledRouge = 10;
const int ledVerte = 11;

void setup() {
    Serial.begin(9600);
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    pinMode(ledRouge, OUTPUT);
    pinMode(ledVerte, OUTPUT);
}

void loop() {
    // Envoi d'une impulsion ultrason
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    // Lecture du temps de retour
    long duration = pulseIn(echoPin, HIGH);
    
    // Calcul de la distance en cm
    float distance = duration * 0.034 / 2;
    
    // Affichage
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    
    // Contrôle des LEDs
    if (distance < 10) {
        digitalWrite(ledRouge, HIGH);
        digitalWrite(ledVerte, LOW);
    } else {
        digitalWrite(ledRouge, LOW);
        digitalWrite(ledVerte, HIGH);
    }
    
    delay(500);
}
```

**Barème** :
- Configuration correcte des pins (1 pt)
- Mesure ultrason correcte (1.5 pts)
- Logique de contrôle des LEDs (1 pt)
- Affichage série (0.5 pt)

---

#### PARTIE 3 : FreeRTOS et communication (10 points)

**Énoncé du système** :

Vous devez créer un système de gestion de parking avec les spécifications suivantes :

**Matériel** :
- 1 capteur de présence (simulé par un bouton sur pin 2)
- 1 LED verte (pin 10) : places disponibles
- 1 LED rouge (pin 11) : parking complet
- Capacité max : 5 places

**Fonctionnalités** :
1. Lorsqu'un véhicule entre (appui sur le bouton), le compteur de places s'incrémente
2. Après 10 secondes, le véhicule sort automatiquement (compteur décrémente)
3. Une tâche affiche l'état du parking toutes les 2 secondes
4. LED verte allumée si places disponibles, LED rouge si complet
5. Utiliser une Queue pour communiquer entre les tâches

**Questions** :

**A)** Dessiner le diagramme d'architecture du système montrant les tâches, la queue, l'ISR et les interactions. (2 points)

**B)** Écrire le code complet en FreeRTOS. (8 points)

**Solution attendue** :

**A) Diagramme (simplifié en texte)** :

```
Bouton (pin 2) → ISR → Queue → Task Parking Manager → {LED verte, LED rouge}
                                         ↓
                                  Task Display (affichage)
                                         ↓
                                    Serial Monitor
                                         
Timer 10s → Callback → Queue → Task Parking Manager
```

**B) Code complet** :

```cpp
#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <semphr.h>
#include <timers.h>

// Définition des broches
const int buttonPin = 2;
const int ledVerte = 10;
const int ledRouge = 11;

// Variables partagées
volatile int placesOccupees = 0;
const int CAPACITE_MAX = 5;

// Handles
QueueHandle_t eventQueue;
TimerHandle_t exitTimer;

// Structure pour les événements
typedef enum {
    ENTREE_VEHICULE,
    SORTIE_VEHICULE
} EventType;

typedef struct {
    EventType type;
} ParkingEvent;

// ISR du bouton
void IRAM_ATTR buttonISR() {
    static unsigned long lastDebounce = 0;
    unsigned long temps = millis();
    
    if (temps - lastDebounce > 200) {
        ParkingEvent event;
        event.type = ENTREE_VEHICULE;
        
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(eventQueue, &event, &xHigherPriorityTaskWoken);
        
        lastDebounce = temps;
        
        if (xHigherPriorityTaskWoken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

// Callback du timer pour sortie automatique
void exitTimerCallback(TimerHandle_t xTimer) {
    ParkingEvent event;
    event.type = SORTIE_VEHICULE;
    xQueueSend(eventQueue, &event, 0);
}

// Tâche de gestion du parking
void TaskParkingManager(void *pvParameters) {
    ParkingEvent event;
    
    for (;;) {
        if (xQueueReceive(eventQueue, &event, portMAX_DELAY) == pdPASS) {
            
            if (event.type == ENTREE_VEHICULE) {
                if (placesOccupees < CAPACITE_MAX) {
                    placesOccupees++;
                    Serial.print("ENTRÉE - Places occupées: ");
                    Serial.print(placesOccupees);
                    Serial.print("/");
                    Serial.println(CAPACITE_MAX);
                    
                    // Démarrer le timer de sortie
                    xTimerStart(exitTimer, 0);
                } else {
                    Serial.println("REFUSÉ - Parking complet!");
                }
            }
            else if (event.type == SORTIE_VEHICULE) {
                if (placesOccupees > 0) {
                    placesOccupees--;
                    Serial.print("SORTIE - Places occupées: ");
                    Serial.print(placesOccupees);
                    Serial.print("/");
                    Serial.println(CAPACITE_MAX);
                }
            }
            
            // Mise à jour des LEDs
            if (placesOccupees < CAPACITE_MAX) {
                digitalWrite(ledVerte, HIGH);
                digitalWrite(ledRouge, LOW);
            } else {
                digitalWrite(ledVerte, LOW);
                digitalWrite(ledRouge, HIGH);
            }
        }
    }
}

// Tâche d'affichage périodique
void TaskDisplay(void *pvParameters) {
    for (;;) {
        Serial.println("----------------------------");
        Serial.print("État du parking: ");
        Serial.print(placesOccupees);
        Serial.print("/");
        Serial.print(CAPACITE_MAX);
        Serial.println(" places occupées");
        
        if (placesOccupees < CAPACITE_MAX) {
            Serial.print("Places disponibles: ");
            Serial.println(CAPACITE_MAX - placesOccupees);
        } else {
            Serial.println("PARKING COMPLET");
        }
        Serial.println("----------------------------");
        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void setup() {
    Serial.begin(9600);
    
    // Configuration des broches
    pinMode(buttonPin, INPUT_PULLUP);
    pinMode(ledVerte, OUTPUT);
    pinMode(ledRouge, OUTPUT);
    
    // État initial : parking vide
    digitalWrite(ledVerte, HIGH);
    digitalWrite(ledRouge, LOW);
    
    // Créer la queue
    eventQueue = xQueueCreate(10, sizeof(ParkingEvent));
    
    // Créer le timer de sortie (10 secondes, auto-reload)
    exitTimer = xTimerCreate(
        "ExitTimer",
        pdMS_TO_TICKS(10000),
        pdFALSE,  // One-shot (pas auto-reload)
        (void *)0,
        exitTimerCallback
    );
    
    if (eventQueue != NULL) {
        // Attacher l'interruption
        attachInterrupt(digitalPinToInterrupt(buttonPin), buttonISR, FALLING);
        
        // Créer les tâches
        xTaskCreate(TaskParkingManager, "ParkingMgr", 256, NULL, 2, NULL);
        xTaskCreate(TaskDisplay, "Display", 128, NULL, 1, NULL);
        
        Serial.println("Système de parking démarré");
        Serial.println("Capacité: 5 places");
    } else {
        Serial.println("ERREUR: Impossible de créer la queue");
    }
}

void loop() {
    // Vide - le scheduler gère tout
}
```

**Barème détaillé Partie 3** :
- Diagramme d'architecture (2 pts)
- Définition correcte des structures et variables (1 pt)
- ISR correcte avec anti-rebond (1 pt)
- Création et utilisation de la Queue (1.5 pts)
- Timer logiciel pour sortie automatique (1.5 pts)
- Tâche de gestion du parking (logique correcte) (1.5 pts)
- Tâche d'affichage (0.5 pt)
- Gestion des LEDs (0.5 pt)
- Code compilable et fonctionnel (bonus 0.5 pt)

---

### BARÈME TOTAL DS :

- Partie 1 (Questions de cours) : 6 points
- Partie 2 (Arduino de base) : 4 points
- Partie 3 (FreeRTOS) : 10 points
- **TOTAL : 20 points**

---

## ANNEXES

### A. Résumé des fonctions FreeRTOS principales

#### Gestion des tâches
```cpp
xTaskCreate()           // Créer une tâche
vTaskDelete()           // Supprimer une tâche
vTaskDelay()            // Délai relatif
vTaskDelayUntil()       // Délai absolu
vTaskSuspend()          // Suspendre une tâche
vTaskResume()           // Reprendre une tâche
vTaskPrioritySet()      // Modifier la priorité
uxTaskPriorityGet()     // Obtenir la priorité
```

#### Queues
```cpp
xQueueCreate()          // Créer une queue
xQueueSend()            // Envoyer (fin de queue)
xQueueSendToFront()     // Envoyer (début de queue)
xQueueReceive()         // Recevoir
xQueuePeek()            // Lire sans retirer
uxQueueMessagesWaiting() // Nombre de messages
```

#### Sémaphores
```cpp
xSemaphoreCreateBinary()    // Sémaphore binaire
xSemaphoreCreateCounting()  // Sémaphore compteur
xSemaphoreCreateMutex()     // Mutex
xSemaphoreTake()            // Prendre
xSemaphoreGive()            // Donner
```

#### Timers logiciels
```cpp
xTimerCreate()          // Créer un timer
xTimerStart()           // Démarrer
xTimerStop()            // Arrêter
xTimerReset()           // Redémarrer
xTimerDelete()          // Supprimer
xTimerChangePeriod()    // Changer la période
```

#### Fonctions FromISR
```cpp
xQueueSendFromISR()
xQueueReceiveFromISR()
xSemaphoreGiveFromISR()
xSemaphoreTakeFromISR()
portYIELD_FROM_ISR()
```

### B. Broches Arduino Uno

**Broches numériques** : 0-13
- 0-1 : RX/TX (Serial)
- 2-3 : Interruptions externes
- 3, 5, 6, 9, 10, 11 : PWM

**Broches analogiques** : A0-A5
- Lecture ADC 10 bits (0-1023)

**Alimentation** :
- VIN : 7-12V
- 5V : Sortie 5V régulée
- 3.3V : Sortie 3.3V
- GND : Masse

### C. Broches ESP32 DevKit

**GPIO utilisables** : 0, 2, 4, 5, 12-19, 21-23, 25-27, 32-39

**Broches spéciales** :
- GPIO 34-39 : Input only
- GPIO 0 : Boot mode
- GPIO 2 : LED intégrée

**Communication** :
- UART : TX (GPIO1), RX (GPIO3)
- I2C : SDA (GPIO21), SCL (GPIO22)
- SPI : MOSI (GPIO23), MISO (GPIO19), SCK (GPIO18), SS (GPIO5)

**ADC** : 12 bits (0-4095)
**PWM** : 16 canaux

### D. Codes d'erreur MQTT

```
-4 : MQTT_CONNECTION_TIMEOUT
-3 : MQTT_CONNECTION_LOST
-2 : MQTT_CONNECT_FAILED
-1 : MQTT_DISCONNECTED
 0 : MQTT_CONNECTED
 1 : MQTT_CONNECT_BAD_PROTOCOL
 2 : MQTT_CONNECT_BAD_CLIENT_ID
 3 : MQTT_CONNECT_UNAVAILABLE
 4 : MQTT_CONNECT_BAD_CREDENTIALS
 5 : MQTT_CONNECT_UNAUTHORIZED
```

---

## CONCLUSION

Ce résumé complet couvre l'ensemble des concepts abordés dans le cours :

1. **Arduino de base** : I/O numériques/analogiques, PWM, communication série
2. **Interruptions et timers** : ISR, debouncing, timers logiciels FreeRTOS
3. **FreeRTOS** : Tâches, priorités, queues, sémaphores, synchronisation
4. **Communication MQTT** : WiFi, pub/sub, QoS, ESP32
5. **Exercices pratiques** : 8 exercices corrigés progressifs
6. **DS complet** : Exemple de devoir avec système parking intelligent

Les concepts sont illustrés par des exemples de code complets et fonctionnels, avec des explications détaillées. Les exercices progressent du simple au complexe, permettant une bonne assimilation.

---

**Bon courage pour vos études !** 🚀
