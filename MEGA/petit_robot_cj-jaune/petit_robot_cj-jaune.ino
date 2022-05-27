#include <Arduino.h>
//#include <Encoder.h>
#include <SoftwareSerial.h> // Communication nano-mega
#include <LiquidCrystal_I2C.h> // écran LCD

/*
TODO: stratégie de débug/ de commentaires
*/


// suiveur de ligne: capteurs CNY70
// "Petit Robot", Robotik UTT, Coupe de France 2022

//########### Pin configuration
#define pin_eye_R A2 // capteur infra ligne
#define pin_eye_L A3 // capteur infra ligne

//########### Motor pins

// a pour reculer, b pour avancer VRAI, a changer reste du code TODO --
#define pin_velo_L 2 // permet de set la vitesse
#define pin_velo_R 7
// PIN ENABLE mais c'est du PWM

#define pin_a_L 5 // a=1 et b=0  AVANCER et b=1 et a=0 pour RECULER
#define pin_b_L 6
#define pin_a_R 3
#define pin_b_R 4


#define tirette_pin 8

#define team_pin 9
int team_side = 0; // 0 par défaut, 0 coté jaune, 1 coté violet
int timer_move = 100;//temps de chaque mouvement unitaite (avancer, reculer ...)
int time_start_action; //moment (millis) de depart de l'action

//numero de ligne franchie par le capteur (de 0=sol, 1=noir, 2=blanc, 3=noir, 4=sol)
int num_ligne_R = 0;
int num_ligne_L = 0;

// intitialisation capteurs
int sensor_R = analogRead(pin_eye_R);
int sensor_L = analogRead(pin_eye_L);

//limite de detation de couleur : moins de triger : noir
int trigger = 500; // attention au passage sur Tags Aruco


int velocityR = 80;//vitesse moyenne
int velocityL= 80;
int reduir_vitesse = 0;//reduire une roue en mode super shlage si on est vilet

int velocity = 80; // a debug puis delete

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
    delay(2000);
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

    
    delay(2000);
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

    // par défaut on avance TODO -- défaut moteurs ne font rien
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
    if (team_side = 1)//violet
    {
      reduir_vitesse = 10;
    }
    else{
      reduir_vitesse =0;
    }
}

void loop(){
  onpasseleshomologations();
  }


// ########### Own functions ALL BELOW THIS

void onpasseleshomologations(){
  // fonction qui a pour unique but de passer les homologations: strat ~86cm FWD puis TurnRight yellow
  // ou TurnLeft Purple puis FWD 120cm
    lcd.setCursor(0,1);
    lcd.print("DBG HOMOLOGATION");
// AVANCER
  timer_move = 2550; // pour 86cm
  velocityL = 74;
  velocityR = 87;
  Forward(); // on doit avancer sur 86cm - la rotation

// TOURNER
  timer_move = 2000; // virage
  velocityL = 80;
  // velocityR = 90;
  TurnRight();

// AVANCER 2 
  timer_move = 3330; // pour 86cm
  velocityL = 75;
  velocityR = 89;
  Forward(); // on doit avancer sur 86cm - la rotation
  while(1);

  // timer_move = à définir
  // TurnRight(); 45° // voir le décalage

  // timer_move = à définir
  //Forward(); // avancer sur 120cm
}



// #################### Below movement functions

void Forward(){
  // Permet d'avancer sur courte distance
    // DEBUG
    lcd.setCursor(0,1);
    lcd.print("[DBG] Forward   ");
    Serial.println("[DEBUG-FWD] Forward");

    // JOB
    digitalWrite(pin_a_L, LOW);
    digitalWrite(pin_b_L, HIGH);
    digitalWrite(pin_a_R, LOW);
    digitalWrite(pin_b_R, HIGH);
    analogWrite(pin_velo_R, velocityR);
    analogWrite(pin_velo_L, velocityL);

    delay(timer_move);

    // Arret moteur
    analogWrite(pin_velo_R, 0);
    analogWrite(pin_velo_L, 0);
  }

void Backward(){
  // Permet de reculer

    // DEBUG
    lcd.setCursor(0,1);
    lcd.print("[DBG] Backward   ");
    Serial.println("[DEBUG-BWD] Backward");


    // JOB
    digitalWrite(pin_a_L, HIGH);
    digitalWrite(pin_b_L, LOW);
    digitalWrite(pin_a_R, HIGH);
    digitalWrite(pin_b_R, LOW);
    analogWrite(pin_velo_R, 100);
    analogWrite(pin_velo_L, 80);

    delay(timer_move);

    // Arret moteur
    analogWrite(pin_velo_R, 0);
    analogWrite(pin_velo_L, 0);
  }


// Rotations faite en bloquant un moteur

void TurnRight(){
  // Tourne à droite

    // DEBUG
    lcd.setCursor(0,1);
    lcd.print("[DBG] TurnRight ");
    Serial.println("[DEBUG-TR] TurnRight");


    // JOB TODO
    digitalWrite(pin_a_L, LOW);
    digitalWrite(pin_b_L, HIGH);
    analogWrite(pin_velo_L, velocityL);

    delay(timer_move);

    // Arret moteur
    analogWrite(pin_velo_R, 0); // par sécurité on arrête les 2
    analogWrite(pin_velo_L, 0);
  }

void TurnLeft(){
  // Tourne à gauche

    // DEBUG
    lcd.setCursor(0,1);
    lcd.print("[DBG] TurnLeft ");
    Serial.println("[DEBUG-TL] TurnLeft");

    // JOB TODO
    digitalWrite(pin_a_R, LOW);
    digitalWrite(pin_b_R, HIGH);
    analogWrite(pin_velo_R, 100-20);

    delay(timer_move);

    // Arret moteur
    analogWrite(pin_velo_R, 0); // par sécurité on arrête les 2
    analogWrite(pin_velo_L, 0);
  }

//tourner sur place vers droite
void SpinRight(){
  // DEBUG
    lcd.setCursor(0,1);
    lcd.print("[DBG] SpinRight   ");
    Serial.println("[DBG] SpinRight");

    // JOB
    digitalWrite(pin_a_L, LOW);
    digitalWrite(pin_b_L, HIGH);
    digitalWrite(pin_a_R, HIGH);
    digitalWrite(pin_b_R, LOW);
    analogWrite(pin_velo_R, velocity);
    analogWrite(pin_velo_L, velocity);

    delay(timer_move);

    // Arret moteur
    analogWrite(pin_velo_R, 0);
    analogWrite(pin_velo_L, 0);
}

void SpinLeft(){
  // DEBUG
    lcd.setCursor(0,1);
    lcd.print("[DBG] SpinLeft    ");
    Serial.println("[DBG] SpinLeft");

    // JOB
    digitalWrite(pin_a_L, HIGH);
    digitalWrite(pin_b_L, LOW);
    digitalWrite(pin_a_R, LOW);
    digitalWrite(pin_b_R, HIGH);
    analogWrite(pin_velo_R, velocity);
    analogWrite(pin_velo_L, velocity);

    delay(timer_move);

    // Arret moteur
    analogWrite(pin_velo_R, 0);
    analogWrite(pin_velo_L, 0);
}

// ################ Advanced functions

void findLine() {// DEBUG-FDL

  //init que aucune ligne dépassée
    num_ligne_L = 0;
    num_ligne_R = 0;

    Serial.println("[DEBUG-FDL] findLine");
    lcd.setCursor(0,1);
    lcd.print("[DBG] FDL START ");
    //analogWrite(pin_velo_R, 100); // 80 rame, 100 va vite, 255 le max, non linéaire, speed à 0 pour l'arreter  PWM
    //analogWrite(pin_velo_L, 100);
    
    // intitialisation capteurs
    sensor_R = analogRead(pin_eye_R);
    sensor_L = analogRead(pin_eye_L);

    if (team_side == 0) //JAUNE
    {
      Serial.println("[DBG-FDL] search 1");
      lcd.setCursor(0,1);
      lcd.print("[DBG-FDL] search1");
      while(num_ligne_R != 1){//tant que R touche pas noir, avancer
          Forward();
          count_line();
      }
      Serial.println("[DBG-FDL] search 4");
      lcd.setCursor(0,1);
      lcd.print("[DBG-FDL] search4");
      //cas dépassement noir par R à gérer TODO --
      while(num_ligne_L != 4){//tant que L pas à l'exterieur, tourner
          TurnRight();
          count_line();
      }
      Serial.println("[DBG] search enc1");
      lcd.setCursor(0,1);
      lcd.print("[DBG] search enc1 ");
      while(sensor_R < trigger and sensor_L < trigger){//tant que pas l'autre aussi sur blanc, tourne sur sois
          SpinRight();
      }

    }
    else{//VIOLET
      while(num_ligne_L != 1){//tant que L touche pas noir, avancer
          Forward();
          count_line();
      }
      //cas dépassement noir par L à gérer TODO --
      while(num_ligne_R != 4){//tant que R pas à l'exterieur, tourner
          TurnLeft();
          count_line();
      }
      while(sensor_R < trigger and sensor_L < trigger){//tant que pas l'autre aussi sur blanc, tourne sur sois
          SpinLeft();
      }
    }
}

void LineFollower(){ // need the argument
    // Le but est de suivre la ligne jusqu'à ??? un obstacle
  if (sensor_L <= trigger){
    TurnLeft();
  }else if (sensor_R <= trigger){
    TurnRight();
  }else{
    Forward();
  }
}

void test_4simple_mov(){
  // Permet de tester les 4 mouvements simples des moteurs: Forward, Backward, TurnLeft, TurnRight
  while(1){
    while(sensor_R>500){
      TurnLeft();
    }
    while(sensor_L>500){
      TurnRight();
    }
  }
}

void count_line(){  // non utilisé actuellement, stratégie L1,2,3,4,5
  //compte le numero de la ligne sur laquelle est le capteur
  sensor_R = analogRead(pin_eye_R);
  sensor_L = analogRead(pin_eye_L);
  //il faut retenir sur quelle ligne a été franchie par le capteur L
  if (sensor_L <= trigger and num_ligne_L == 0)//L noir
  {
    num_ligne_L = 1;
  }
  else if (sensor_L > trigger and num_ligne_L == 1)//L blanc
  {
    num_ligne_L = 2;
  }
  else if (sensor_L <= trigger and num_ligne_L == 2)//L noir
  {
    num_ligne_L = 3;
  }
  else if (sensor_L > trigger and num_ligne_L == 3)//L blanc
  {
    num_ligne_L = 4;
  }

  //il faut retenir sur quelle ligne a été franchie par le capteur R
  if (sensor_R <= trigger and num_ligne_R == 0)//L noir
  {
    num_ligne_R = 1;
  }
  else if (sensor_R > trigger and num_ligne_R == 1)//L blanc
  {
    num_ligne_R = 2;
  }
  else if (sensor_R <= trigger and num_ligne_R == 2)//L noir
  {
    num_ligne_R = 3;
  }
  else if (sensor_R > trigger and num_ligne_R == 3)//L blanc
  {
    num_ligne_R = 4;
  }
}


void cj_yellow(){
    // ####### Suivi de ligne // ancien loop 
    sensor_R = analogRead(pin_eye_R);
    sensor_L = analogRead(pin_eye_L);
    lcd.setCursor(0,1);
    lcd.print("droit          ");
    Forward();
    //lcd.print("L" + sensor_L);
    //Serial.println("R" +sensor_R);
    //Serial.println(sensor_L);
    //while(true){
    //  LineFollower();
    //}
//    test_4simple_mov();
    sensor_R = analogRead(pin_eye_R);
    sensor_L = analogRead(pin_eye_L);
    while(sensor_L<500){// ?
      sensor_R = analogRead(pin_eye_R);
    sensor_L = analogRead(pin_eye_L);
      TurnLeft();
    }
    while(sensor_R<500){
      sensor_R = analogRead(pin_eye_R);
    sensor_L = analogRead(pin_eye_L);
      TurnRight();
    }
    Serial.println(sensor_L);
    Serial.println(sensor_R);
    
    Serial.println("");
    
}
