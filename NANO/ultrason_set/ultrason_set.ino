#include <Arduino.h>
#include <SoftwareSerial.h>

/*
TODO:Check if the code work
It was in a created repo, does it works ? It must be saved
*/



/* Constante pour le timeout ; 25ms = ~1.7m à 340m/s */
#define MEASURE_TIMEOUT 5000UL
#define SOUND_SPEED 0.34  /* In mm per microsecond */
#define NB_CAPTORS 4
#define COMMUNICATION_PIN A0

const uint8_t trigger_pins[NB_CAPTORS]{2, 5, 8, 11};
const uint8_t echo_pins[NB_CAPTORS]{3, 6, 9, 12};
uint8_t message{0};
SoftwareSerial write_serial(2, 3);


inline double get_distance(int captor_index);


void setup() {
    write_serial.begin(9600);
    Serial.begin(9600);
    Serial.println("[DEBUG] Initialization");

    /* initialize captors */
    for (int i = 0; i < NB_CAPTORS; ++i) {
        pinMode(trigger_pins[i], OUTPUT);
        digitalWrite(trigger_pins[i], LOW);
        pinMode(echo_pins[i], INPUT);
    }
}


void loop() {
  
    /*for (int i = 4; i < NB_CAPTORS + 4; ++i) {
        const double distance = get_distance(i);
        if (distance < 300){
            message |= 0b1 << i;
        }
        else{
            message &= ~(0b1 << i);
        write_serial.write(message);
        }
        //Serial.println(message); // tj = 240
        Serial.println(distance);
    } */
      digitalWrite(TRIGGER_PIN, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIGGER_PIN, LOW);
      
      /* 2. Mesure le temps entre l'envoi de l'impulsion ultrasonique et son écho (si il existe) */
      long measure = pulseIn(ECHO_PIN, HIGH, MEASURE_TIMEOUT);
       
      /* 3. Calcul la distance à partir du temps mesuré */
      float distance_mm = measure / 2.0 * SOUND_SPEED;
       
      /* Affiche les résultats en mm, cm et m */
      Serial.print(F("Distance: "));
      Serial.print(distance_mm);
      Serial.print(F("mm ("));
      Serial.print(distance_mm / 10.0, 2);
      Serial.print(F("cm, "));
}

/**
 * Retourne la distance qui sépare le capteur du plus proche obstacle
 * @param captor_index
 * @return
 */
inline double get_distance(const int captor_index) {
    digitalWrite(trigger_pins[captor_index], HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger_pins[captor_index], LOW);

    /* Mesure le temps entre l'envoi de l'impulsion ultrasonique et son écho (si il existe) */
    unsigned long measure = pulseIn(echo_pins[captor_index], HIGH, MEASURE_TIMEOUT);
    /* Calcul la distance à partir du temps mesuré */
    return (((double) measure) * SOUND_SPEED) / 2.0;
}
