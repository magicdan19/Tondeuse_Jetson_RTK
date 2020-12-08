/*utilitaire pour mesurer le voltage et l'ampérage de la tondeuse
 * recoit byte=1 pour demander voltage
 * recoit byte=2 pour demander ampérage
 */

#include "ACS712.h"
#include <Wire.h>

// We have 30 amps version sensor connected to A2 pin of arduino
// Replace with your version if necessary
ACS712 sensor(ACS712_30A, A2);

// pin pour voltage sensor A0
const byte voltageSensor = A0;
float vOUT = 0.0;
double voltage = 0.0;
float R1 = 30000.0;
float R2 = 7500.0;
int value = 0; 
double amperage = 0;
byte Valeur_Recue = 0;

void setup() {
  pinMode(voltageSensor, INPUT);
  //Serial.begin(115200);
  //init comm i2c
  Wire.begin(0x19);                // join i2c bus with address #0x19
  Wire.onRequest(sendData); // register event
  Wire.onReceive(receiveEvent); // register event

  int zero = sensor.calibrate();
  //Serial.println("Done!");
  //Serial.println("Zero point for this sensor = " + zero);
}

void loop() {
  // lire le voltage
  value = analogRead(voltageSensor);
  //calcul du voltage en fonction des résistances
  vOUT = (value * 5.0) / 1023.0;
  voltage = vOUT / (R2/(R1+R2));
 
  // Read current from sensor
  amperage = sensor.getCurrentDC();
  
  // Wait a second before the new measurement
  delay(500);
}

void sendData(){

   // Buffer to hold temp data, 4 byte for each float
  byte buf1[8];
  byte buf2[4];
   
  byte* y = (byte*) &voltage;
  // 4 bytes for each float
  for(int i = 0; i < 4; i++){
    buf1[i] = y[i];
  }
  Wire.write(buf1, 4);
  
  byte* z = (byte*) &amperage;
  // 4 bytes for each float
  for(int i = 0; i < 4; i++){
    buf2[i] = z[i];
  }
}

void receiveEvent(int howMany){

  
}
