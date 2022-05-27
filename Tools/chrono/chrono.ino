unsigned long MS;
unsigned long start;
unsigned long stop_;

void setup() {
  Serial.begin(9600);

}

void loop() {
  start=millis(); // start tirette
  MS = millis()-start;
  while(MS<100000){
    MS = millis()-start;
    Serial.println(MS);
    }
  stop_ =millis();
  while(1); // STOP

}
