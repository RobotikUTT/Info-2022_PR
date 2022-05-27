#include <Arduino.h>
#include <Encoder.h>
#include <SoftwareSerial.h> // Communication nano-mega
#include <LiquidCrystal_I2C.h> // écran LCD

// suiveur de ligne: capteurs CNY70
// "Petit Robot", Robotik UTT, Coupe de France 2022

//########### Pin configuration
#define pin_eye_R A2
#define pin_eye_L A3

//########### Motor pins

// a pour reculer, b pour avancer
#define pin_en_L 2 // enable, permet de set la vitesse
#define pin_en_R 7
// PIN ENABLE mais c'est du PWM

#define pin_a_L 5 // a=1 et b=0  AVANCER et b=1 et a=0 pour RECULER
#define pin_b_L 6
#define pin_a_R 3
#define pin_b_R 4


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

    
    delay(5000);
    Serial.println("[DEBUG] Attente tirette");
    Serial.print("Valeur tirette (0 appuyé | 1 relaché) :  ");
    lcd.setCursor(0,1);
    lcd.print(" READY TO START ");
    Serial.println(digitalRead(tirette_pin));
    while (!digitalRead(tirette_pin)) {}
    Serial.println("[DEBUG] START");
    lcd.setCursor(0,1);
    lcd.print("[DBG]  START    ");
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

    // findLine(); // procédure robot va chercher la ligne (1 fois) A DECOMMENTER
}

void loop(){
  // ####### TESTS

  // test_4simple_mov();

  test_left();

  
  }


// ########### Own functions ALL BELOW THIS



// #################### Below movement functions

void Forward(){
  // Permet d'avancer sur courte distance

    // DEBUG
    lcd.setCursor(0,1);
    lcd.print("[DBG] Forward   ");
    Serial.println("[DEBUG-FWD] Forward");

    // JOB
    digitalWrite(pin_a_L, HIGH);
    digitalWrite(pin_b_L, LOW);
    digitalWrite(pin_a_R, HIGH);
    digitalWrite(pin_b_R, LOW);
    analogWrite(pin_en_R, 100);
    analogWrite(pin_en_L, 100);

    delay(5000);

    // Arret moteur
    analogWrite(pin_en_R, 0);
    analogWrite(pin_en_L, 0);
  }

void Backward(){
  // Permet de reculer

    // DEBUG
    lcd.setCursor(0,1);
    lcd.print("[DBG] Backward   ");
    Serial.println("[DEBUG-BWD] Backward");


    // JOB
    digitalWrite(pin_a_L, LOW);
    digitalWrite(pin_b_L, HIGH);
    digitalWrite(pin_a_R, LOW);
    digitalWrite(pin_b_R, HIGH);
    analogWrite(pin_en_R, 100);
    analogWrite(pin_en_L, 100);

    delay(5000);

    // Arret moteur
    analogWrite(pin_en_R, 0);
    analogWrite(pin_en_L, 0);
  }


// Rotations faite en bloquant un moteur

void TurnRight(){
  // Tourne à droite

    // DEBUG
    lcd.setCursor(0,1);
    lcd.print("[DBG] TurnRight ");
    Serial.println("[DEBUG-TR] TurnRight");


    // JOB TODO
    digitalWrite(pin_a_L, HIGH);
    digitalWrite(pin_b_L, LOW);
    digitalWrite(pin_a_R, LOW);
    digitalWrite(pin_b_R, LOW);
    analogWrite(pin_en_L, 100);

    delay(5000);

    // Arret moteur
    analogWrite(pin_en_R, 0); // par sécurité on arrête les 2
    analogWrite(pin_en_L, 0);
  }

void TurnLeft(){
  // Tourne à gauche

    // DEBUG
    lcd.setCursor(0,1);
    lcd.print("[DBG] TurnLeft ");
    Serial.println("[DEBUG-TL] TurnLeft");

    // JOB TODO
    digitalWrite(pin_a_L, LOW);
    digitalWrite(pin_b_L, LOW);
    digitalWrite(pin_a_R, HIGH);
    digitalWrite(pin_b_R, LOW);
    analogWrite(pin_en_R, 100);

    delay(5000);

    // Arret moteur
    analogWrite(pin_en_R, 0); // par sécurité on arrête les 2
    analogWrite(pin_en_L, 0);
  }


// ################ Advanced functions

void findLine() {// DEBUG-FDL
    //int other = -1; // variable pour attendre quel capteur est le second à toucher la ligne

    Serial.println("[DEBUG-FDL] findLine");
    lcd.setCursor(0,1);
    lcd.print("[DBG] FDL START ");
    //analogWrite(pin_en_R, 100); // 80 rame, 100 va vite, 255 le max, non linéaire, speed à 0 pour l'arreter  PWM
    //analogWrite(pin_en_L, 100);

    int aligned = 0; // =0 si non aligné, 1 si aligné sur bande noire
    int trigger = 500; // attention au passage sur Tags Aruco
    
    // intitialisation capteurs
    int sensor_R = analogRead(pin_eye_R);
    int sensor_L = analogRead(pin_eye_L);
    while (sensor_R > trigger and sensor_L > trigger){
        sensor_R = analogRead(pin_eye_R);
        sensor_L = analogRead(pin_eye_L);

        // Faire avancer le robot tout droit
    }
    // TODO faire symétrie coté Violet
    //if (sensor_R < 500){
      // L avance puis suivi ligne
      //}
    //else if (sensor_L < 500){
      // suivi ligne
      //}
}

void LineFollower(){ // need the argument
    // Le but est de suivre la ligne jusqu'à ??? un obstacle

    // TODO Vendredi
}

void test_4simple_mov(){
  // Permet de tester les 4 mouvements simples des moteurs: Forward, Backward, TurnLeft, TurnRight
    Backward();
    delay(5000);
    Forward();
    delay(5000);
    TurnRight();
    delay(5000);
    TurnLeft();
    delay(5000);
  }


// ############## TEST functions only

void test_left(){
  // Permet de reculer

    // DEBUG
    lcd.setCursor(0,1);
    lcd.print("[DBG] TEST      ");
    Serial.println("[DEBUG-TEST] test_turn left");


    // JOB
    digitalWrite(pin_a_L, LOW);
    digitalWrite(pin_b_L, HIGH);
    digitalWrite(pin_a_R, LOW);
    digitalWrite(pin_b_R, LOW);
    analogWrite(pin_en_R, 0);
    analogWrite(pin_en_L, 100);

    delay(5000);

    // Arret moteur
    analogWrite(pin_en_R, 0);
    analogWrite(pin_en_L, 0);
  }
