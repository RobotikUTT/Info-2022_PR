#include <Arduino.h>
#include <SoftwareSerial.h>

/* Constante pour le timeout ; 25ms = ~1.7m à 340m/s */
#define MEASURE_TIMEOUT 5000UL
#define SOUND_SPEED 0.34  /* In mm per microsecond */
#define NB_CAPTORS 4

// Captor 1 : rear left, Captor 2 : rear right, Captor 3 : front right, Captor 4 : front left
const uint8_t trigger_pins[NB_CAPTORS]{5, 7, 9, 11};
const uint8_t echo_pins[NB_CAPTORS]{6, 8, 10, 12};
uint8_t message = 0;

inline double get_distance(int captor_index);
SoftwareSerial mega_serial(3, 4);

void setup() {
  mega_serial.begin(9600);
  Serial.begin(9600);
  // initialize captors
  for (int i = 0; i < NB_CAPTORS; ++i) {
    pinMode(trigger_pins[i], OUTPUT);
    digitalWrite(trigger_pins[i], LOW);
    pinMode(echo_pins[i], INPUT);
  }
}

void loop() {
  for (int i = 0; i < NB_CAPTORS; ++i) {
    const double distance = get_distance(i);
    if (distance != 0 && distance < 100) { // we take distance < 120
        bitSet(message, i); // set the bit n°i to 1
    }
    else{
        bitClear(message, i); // set the bit n°i to 0
    }
    mega_serial.write(message);
    // delay(100); // To debug
  }
  // Serial.println(message, BIN); // To debug
}

inline double get_distance(const int captor_index) { // to get distance from a captor
  digitalWrite(trigger_pins[captor_index], HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger_pins[captor_index], LOW);

  /* Mesure le temps entre l'envoi de l'impulsion ultrasonique et son écho (si il existe) */
  unsigned long measure = pulseIn(echo_pins[captor_index], HIGH, MEASURE_TIMEOUT);
  /* Calcul la distance à partir du temps mesuré */
  return (((double) measure) * SOUND_SPEED) / 2.0;
}
