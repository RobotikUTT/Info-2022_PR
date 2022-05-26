#include <Arduino.h>
#include <Encoder.h>
#include <SoftwareSerial.h> // Communication nano-mega
#include <LiquidCrystal_I2C.h> // écran LCD

// suiveur de ligne: capteurs CNY70
// "Petit Robot", Robotik UTT, Coupe de France 2022

//########### Pin configuration
#define pin_eye_R A0
#define pin_eye_L A1

//########### Motor pins

// a pour reculer, b pour avancer
#define pin_en_L 2 // enable
// PIN ENABLE mais c'est du PWM

#define pin_a_L 5 // a=1 et b=0  AVANCER et b=1 et a=0 pour RECULER
#define pin_b_L 6
#define pin_a_R 3
#define pin_b_R 4
#define pin_en_R 7

#define tirette_pin 8

#define team_pin 9
int team_side = 0; // 0 paar défaut, 0 coté jaune, 1 coté violet

SoftwareSerial nano(11, 10); // line série nano et méga

LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
// si l'adresse n'est pas la bonne, utiliser le script i2c pour récupérer l'adresse
// Cablage GND, 5V, SDA, SCL
// LCD 20 mega pour SDA, 21 pour SCL

void setup() {
    Serial.begin(9600);

    // ############# LCD  
    lcd.init();
    Serial.println("[DEBUG] Intitialisation LCD");                      
    // initialize the lcd 
    // Print a message to the LCD.
    lcd.backlight();
    lcd.setCursor(3,0); // x, y, x entre 0 et 15, y = 0 ou 1
    lcd.print("Robotik'UTT");
    

    // Initialisation motor pin ?
    for (int i = 2; i <= 7; i++)
        pinMode(i, OUTPUT);
    pinMode(pin_eye_L, INPUT);
    pinMode(pin_eye_R, INPUT);
    pinMode(tirette_pin, INPUT_PULLUP);

    // Interrupteur choix équipe
    pinMode(team_pin, INPUT_PULLUP);

    // Choix du coté
    lcd.setCursor(0,1);
    lcd.print("[DBG] choix cote"); // DBG pour debug
    Serial.println("[DBG] choix cote");
    delay(7000);
    if(digitalRead(team_pin) == true){
      team_side = 1;
      Serial.println("[DEBUG] Yellow");
      lcd.setCursor(0,1);
      lcd.print("[DBG] Yellow    "); 
      } // sinon par défaut à 0
    else{
      Serial.println("[DEBUG] Purple");
      lcd.setCursor(0,1);
      lcd.print("[DBG] Purple    ");
      }

    

    Serial.println("[DEBUG] Attente tirette");
    Serial.print("Valeur tirette (0 appuyé | 1 relaché) :  ");
    lcd.setCursor(0,1);
    lcd.print(" READY TO START ");
    Serial.println(digitalRead(tirette_pin));
    while (!digitalRead(tirette_pin)) {}
    Serial.println("[DEBUG] START");
    delay(5);

    // par défaut on avance
    digitalWrite(pin_a_L, LOW);
    digitalWrite(pin_a_R, LOW);
    digitalWrite(pin_b_L, HIGH); 
    digitalWrite(pin_b_R, HIGH);

    nano.begin(9600);
    nano.listen();
    /* DEBUG Reception Mega/Nano
    while (true) {
        int val = -1;
        while (nano.available() > 0){
            val &= nano.read();
        }
        Serial.print(!!(val & 1));
        Serial.print(", ");
        Serial.print(!!(val & 2));
        Serial.print(", ");
        Serial.print(!!(val & 4));
        Serial.print(", ");
        Serial.print(!!(val & 8));
        Serial.println(";");
        delay(100);
    }*/

    findLine(); // procédure robot va chercher la ligne (1 fois)
}

void loop() {
    int sensor_R = analogRead(pin_eye_R);
    int sensor_L = analogRead(pin_eye_L);

    Serial.print(sensor_L);
    Serial.print(", ");
    Serial.print(sensor_R);
    Serial.print(", ");

    int speed_R = sensorRMotorTF(sensor_L, sensor_R);
    int speed_L = sensorLMotorTF(sensor_L, sensor_R);

    Serial.print(speed_L);
    Serial.print(", ");
    Serial.println(speed_R);

    if (speed_R < 0) {
        speed_R = -speed_R;
        digitalWrite(pin_a_R, HIGH);
        digitalWrite(pin_b_R, LOW);
    } else {
        digitalWrite(pin_a_R, LOW);
        digitalWrite(pin_b_R, HIGH);
    }

    if (speed_L < 0) {
        speed_L = -speed_L;
        digitalWrite(pin_a_L, HIGH);
        digitalWrite(pin_b_L, LOW);
    } else {
        digitalWrite(pin_a_L, LOW);
        digitalWrite(pin_b_L, HIGH);
    }

    if (isBlocked()) { // bloqué -> on l'arrete
      // transformer la boucle en while quand on entre dedans
        analogWrite(pin_en_R, 0);
        analogWrite(pin_en_L, 0);
        Serial.println("[DEBUG] isBlocked");
        delay(3000);
        if (isBlocked()) {
            pullOut();
        }
    }


    analogWrite(pin_en_R, speed_L);
    analogWrite(pin_en_L, speed_R);

    
}

bool isBlocked() { // Test pour savoir si les capteurs à ultrasons (limite de la nano actuelle
    bool test = false;
    int sensor = -1;
    while (nano.available() > 0) {
        sensor &= nano.read();
        test = true;
    }
    if (!test)
        sensor = 0;
    return !!(sensor & 0b1100);
}

void pullOut() { // On recule
    digitalWrite(pin_a_L, HIGH);
    digitalWrite(pin_b_L, LOW);
    digitalWrite(pin_a_R, HIGH);
    digitalWrite(pin_b_R, LOW);
    analogWrite(pin_en_R, 100);
    analogWrite(pin_en_L, 100);

    delay(500);

    analogWrite(pin_en_R, 0);
    analogWrite(pin_en_L, 0);

    while(true); // Pour terminer le programme à l'infini
}

void SeRetourner () { // Se retourner sur la ligne, pas sans ligne
    int DemiTour = 0;
    while (DemiTour < 2) { 
        int sensor_R = analogRead(pin_eye_R);
        int sensor_L = analogRead(pin_eye_L);
        
        digitalWrite(pin_a_L, LOW);
        digitalWrite(pin_b_L, HIGH);
        digitalWrite(pin_a_R, HIGH);
        digitalWrite(pin_b_R, LOW);
        analogWrite(pin_en_R, 100);
        analogWrite(pin_en_L, 100);

        if ((DemiTour == 0)&&(sensor_R < 500) ){
            DemiTour++;
        }
        if ((DemiTour == 1)&&(sensor_L < 500) ){
            //realignement
            digitalWrite(pin_a_R, HIGH);
            digitalWrite(pin_b_R, LOW);
            digitalWrite(pin_a_L, LOW);
            digitalWrite(pin_b_L, HIGH);
            analogWrite(pin_en_R, 100);
            analogWrite(pin_en_L, 100);
            delay(200);
            DemiTour++;
        }
    }
}

void findLine() {// DEBUG-FDL
    //int other = -1; // variable pour attendre quel capteur est le second à toucher la ligne

    Serial.println("[DEBUG-FDL] findLine");
    lcd.setCursor(0,1);
    lcd.print("[DBG] FDL START ");
    analogWrite(pin_en_R, 100); // 80 rame, 100 va vite, 255 le max, non linéaire, speed à 0 pour l'arreter  PWM
    analogWrite(pin_en_L, 100);

    int aligned = 0; // =0 si non aligné, 1 si aligné sur bande noire
    // TODO la suite
    while (true) {
        int sensor_R = analogRead(pin_eye_R);
        int sensor_L = analogRead(pin_eye_L);

        /* if (other >= 0) {
            if (analogRead(other) < 500) {
                Serial.println("Sequence terminee");
                digitalWrite(pin_a_R, HIGH);
                digitalWrite(pin_b_R, LOW);
                digitalWrite(pin_a_L, HIGH);
                digitalWrite(pin_b_L, LOW);
                analogWrite(pin_en_R, 120);
                analogWrite(pin_en_L, 100);
                delay(200);
                break;
            }
        } else {
            if (sensor_L < 500) {
                // analogWrite(pin_en_R, 100);
                // analogWrite(pin_en_L, 80);
                other = pin_eye_R;
                Serial.println("[DEBUG-FDL] Trouve ligne gauche");
            }
            if (sensor_R < 500) {
                // analogWrite(pin_en_R, 80);
                // analogWrite(pin_en_L, 80);
                other = pin_eye_L;
                Serial.println("Trouve ligne droite");
                analogWrite(pin_en_L, 50);
            }
                
        }*/
    }
}

int sensorRMotorTF(int sensor_L, int sensor_R) { // fonction de transfert proportionnel: valeurs capteurs -> valeurs moteurs
  // Loop pour le suivi de ligne entree capteurs sortie moteur, A UTILISER
    //return sensor_L * motor_speed / (1024);

    if (sensor_L < 500)
        return -60;
    if (sensor_L > 500)
        return 80;
    //int bonus = (sensor_R < 850) ? 20 : 0;
    //return (sensor_L - 800) / 2 + bonus;
}

int sensorLMotorTF(int sensor_L, int sensor_R) {
    //return sensor_R * motor_speed / (1024);

    if (sensor_R < 500)
        return -60;
    if (sensor_R > 500)
        return 80;
    //int bonus = (sensor_L < 850) ? 20 : 0; // chgmt vitesse moteur exté virage de suivi ligne
    //return (sensor_R - 800) / 2 + bonus;
}

/*
void forwardNmeters(float meters) { // erreur de compil
    int init_left = left_wheel.readAndReset();
    int init_right = right_wheel.readAndReset();
    int ticks = meters * TICKS_PER_METER;

    while (true) {
        int right = right_wheel.read();
        int left = left_wheel.read();

        int diff = right - left;

        analogWrite(PIN_EN_L, MOTOR_SPEED + diff);
        analogWrite(PIN_EN_R, MOTOR_SPEED - diff);

        if (right >= ticks && right >= ticks)
            break;
    }
}
*/
