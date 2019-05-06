//Board : DO IT ESP32 DEVKIT V1

//Library for HR Sensor
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

//-------------------------------------------------------------------------- heart beat sensor

MAX30105 particleSensor;

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;

//--------------------------------------------------------------


//LEDIndicator <---- don't remeber what is this for
int LED = 2;

//-----------------------------------------------------------------------------------------------------------------------------Motor Driver (H-Bridge)   - Pin for H Brdige
int relay2 = 19;
int relay3 = 32;

//-----------------------------------------------------------------------------------------------------------------------------offset for input from serial monitor from arduino IDE
int offset_serial = 0; // <---- pilih offset 0 untuk applikasi dan 40 untuk input dari serial arduino
//int offset_serial = 40; //<---- input di serial arduino :   "(" untuk level 0, dan "O" (o besar) untuk level 4080 
 
//-----------------------------------------------------------------------------------------------------------------------------Motor Position & Variable
int potread = 0;
int Potentio = 34;  //<---- pin input from potentio (MPS)
int bottompos = 0;  //<---- 1 if motor reach bottom position
int toppos = 0;     //<---- 1 if motor reach top position
 
//-----------------------------------------------------------------------------------------------------------------------------Button Input
int BTNUP = 15;  //<--- pin button up
int BTNDOWN = 4; //<--- pin button down
int BTNMID = 5; //<--- pin button mid
int BTNLEFT = 13; //<--- pin button left
int BTNRIGHT = 12; //<--- pin button right
int intervalBTN = 500; //<--- interval for avoid rapid pressing button
unsigned long buttonMillis = 0; //<--- for checking if there is any input (for checking stand by mode)
int MIDHOLD = 0;
unsigned long midholdMillis = 0; //<--- timer check if middle button is hold by user
int TOGGLEMOTOR = 18; // pull LOW for activate 'motor adjustment manual control' using jumper on white PCB

//-----------------------------------------------------------------------------------------------------------------------------Bike Speed Sensor & Variable
int analog_value = 0;
int ANALOG_PIN_0 = 33;    //<---- bike speed input pin, can be digital read HIGH or LOW 
unsigned long currentMillis = 0; // <--- timer to check from previous read
unsigned long prevMillis = 0; // <--- timer to check from previous read
unsigned long timing = 0; // <--- timer to check from previous read
unsigned long timingMillis = 0; // <--- timer to check from previous read
float distance = 0;
float timingfloat = 0;
int active = 1;
float bikespeed;
int interval = 6000;

//speed debug
unsigned long debugSpeed = 0; 

//-----------------------------------------------------------------------------------------------------------------------------Serial Input & motor detection (software based)
byte dataByte = 0;
int level = 1000;
int automove_dec = 0;
int automove_inc = 0;


//-----------------------------------------------------------------------------------------------------------------------------Stand-by Millis
unsigned long standbyMillis = 0;
unsigned long standbyInterval = 70000;
int standbyMode = 0;
 
void setup()
{
  // LED pin
  pinMode (LED, OUTPUT);
  // sensor pin
  pinMode(ANALOG_PIN_0, INPUT);
  pinMode(Potentio, INPUT);
  // input pin
  pinMode (BTNLEFT, INPUT_PULLUP);
  pinMode (BTNRIGHT, INPUT_PULLUP);
  pinMode (BTNUP, INPUT_PULLUP);
  pinMode (BTNDOWN, INPUT_PULLUP);
  pinMode (BTNMID, INPUT_PULLUP);
  pinMode (TOGGLEMOTOR, INPUT_PULLUP);
  // relay pin for H Bridge
  pinMode(relay2, OUTPUT);
  pinMode(relay3, OUTPUT);
  digitalWrite(relay2, LOW);
  digitalWrite(relay3, LOW);
 
  Serial.begin(115200);
  delay(1000); // give me time to bring up serial monitor
  Serial.println("ESP32 Ready");
  
  // Initialize sensor (HR sensor)
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
 
}
 
void loop()
{
  currentMillis = millis();
  analog_value = analogRead(ANALOG_PIN_0);
  potread = analogRead(Potentio);
  //Serial.print(level);Serial.print(" - ");
  //Serial.println(potread);
  //Serial.println(analog_value);
  //Serial.println(digitalRead(TOGGLEMOTOR));
  
  //-----------------------------------------------------------------------------------------------------------------------------speed detect reset
  if (analog_value >= 1023 && active == 0){
    active = 1;
    }
   
  //-----------------------------------------------------------------------------------------------------------------------------speed detect
  if (analog_value < 1023 && active == 1){
    //Serial.print ("detect : ");
    timing = currentMillis - timingMillis;
   
    timingMillis = millis();
    standbyMillis = millis();
    
    active = 0;
    timingfloat = timing;
    bikespeed = 70000/timingfloat*0.036; //<----- dalam KM/H,  70000 itu keliling ban 70cm * 1000 / 3600 konversi m/s ke km/h
    distance = distance+0.0007; //<----- tambah jarak 70cm per satu rotasi
    
    if (bikespeed <= 100.00){   //<----- max speed 100km/h (lebih dari itu dianggap 100
    Serial.print ("SPD");
    Serial.println (bikespeed);    
    /*
    Serial.print (timing);
    Serial.print (" distance : ");
    Serial.print (distance);
    Serial.print (" KM - ");
    Serial.print(potread);
    Serial.print(toppos);
    Serial.print(bottompos);
    Serial.print (" - ");
    Serial.println(level);*/
    }
    
    if (standbyMode == 1){ //<----- wake up si game kalau deteksi ban muter
    Serial.println("BTNUP");
    Serial.println ("BIKE WAKE-UP");
    standbyMode = 0;  
    
    }
    } //-----------------------------------------------------------------------------------------------------------------------------end of speed detect

   //fake speed
   //if (currentMillis - debugSpeed >= 2000 ){Serial.println ("SPD100.00"); debugSpeed = millis();}
 
   //----------------------------------------------------------------------------------------------------------------------------- button input
   if (digitalRead(BTNUP) == LOW && currentMillis - buttonMillis >= intervalBTN){
   Serial.println("BTNUP");
   buttonMillis = millis();
   standbyMillis = millis();

   if (digitalRead (TOGGLEMOTOR) == LOW){increase_weight(); Serial.println("increase_weight");} //<-------------- adjust motor position
    
   }
   if (digitalRead(BTNDOWN) == LOW && currentMillis - buttonMillis >= intervalBTN){
   Serial.println("BTNDOWN");
   buttonMillis = millis();
   standbyMillis = millis();
   
   if (digitalRead (TOGGLEMOTOR) == LOW){decrease_weight(); Serial.println("decrease_weight");} //<-------------- adjust motor position

   }
   
   if (digitalRead(BTNMID) == LOW && currentMillis - buttonMillis >= intervalBTN){
   Serial.println("BTNMID");
   buttonMillis = millis();
   standbyMillis = millis();
   if ( currentMillis - midholdMillis <= (intervalBTN + 30)){
    MIDHOLD = MIDHOLD+1;
    midholdMillis = millis ();
    Serial.println (MIDHOLD);
    }else{
    MIDHOLD = 0;
    midholdMillis = millis ();
    Serial.println (MIDHOLD);
   }

   if (MIDHOLD >= 3){
   Serial.println("MIDHOLD");
   MIDHOLD = 0;
   }
   
   if (digitalRead (TOGGLEMOTOR) == LOW){stop_weight(); Serial.println("stop_weight");} //<-------------- stop any manual adjustment
   
   }
   
   if (digitalRead(BTNLEFT) == LOW && currentMillis - buttonMillis >= intervalBTN){
   Serial.println("BTNLEFT");
   buttonMillis = millis();
   standbyMillis = millis();
   //stop_weight();
   }
   if (digitalRead(BTNRIGHT) == LOW && currentMillis - buttonMillis >= intervalBTN){
   Serial.println("BTNRIGHT");
   buttonMillis = millis();
   standbyMillis = millis();
   //stop_weight();
   }

 //------------------------------------------------------------------------------------------------------------------------------- auto move weight stop function
   if (potread <= level && automove_dec == 1){
    stop_weight();
    automove_dec = 0;
   }
 
    if (potread >= level && automove_inc == 1){
    stop_weight();
    automove_inc = 0;
   }
 //-------------------------------------------------------------------------------------------------------------------------------- end of auto move weight stop function

 
   //------------------------------------------------------------------------------------------------------------------------------ Serial Input
   if (Serial.available() > 0){
    dataByte = Serial.read(); //<---- baca dari serial (dari apps atau serial monitor arduino IDE)
    dataByte = dataByte - offset_serial;
    Serial.print("data diterima: ");
    Serial.println(dataByte);
    
    if(dataByte >= 0 && dataByte < 41){ //<---- data yang masuk hanya bisa byte dari 0 sampai 40
    if (digitalRead(LED) == HIGH){digitalWrite (LED, LOW);}else{digitalWrite (LED, HIGH);}
    }
 
    if (dataByte >= 0 && dataByte <= 20){level = (dataByte)*65;}
    if (dataByte > 20 && dataByte <= 30){level = ((dataByte)*110)-900;}
    if (dataByte > 30 && dataByte < 41){level = ((dataByte)*160)-2400;}

    //Serial.print(potread);Serial.print(" - ");
    //Serial.println (level);
    
    if (level >= 0 && level <= 4096){
      if (level == 4096){level = 4080;}
      if (potread > level){
       decrease_weight();
       automove_dec = 1;
      }
      if (potread < level){
       increase_weight();
       automove_inc = 1;
      }
    }
   } //------------------------------ end SERIAL INPUT

//--------------------------------------------------------------------------------------------------------------------- Stand By function

if (currentMillis - standbyMillis > standbyInterval && currentMillis - standbyMillis < 4294967295){
  if (standbyMode == 0){
  Serial.println ("BIKE STAND-BY");
  standbyMode = 1;
  level = 1000;
  if (potread > level){
    decrease_weight();
    automove_dec = 1;
  }
  if (potread < level){
    increase_weight();
    automove_inc = 1;
  }
}
}

//--------------------------------------------------------------------------------------------------------------------- Heart Beat

  long irValue = particleSensor.getIR();


   if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

    Serial.print ("HB");
    Serial.println (beatAvg);

    if (irValue < 50000){
    beatAvg=0;
    Serial.println(" No finger?");

}
 
}//-----------------------------------------------------------------------------------------------------------------------void loop close
 
void increase_weight(){
  digitalWrite(relay2, HIGH);
  digitalWrite(relay3, LOW);
}
 
void decrease_weight(){
  digitalWrite(relay2, LOW);
  digitalWrite(relay3, HIGH);
}
 
void stop_weight(){
  digitalWrite(relay2, LOW);
  digitalWrite(relay3, LOW);
}
