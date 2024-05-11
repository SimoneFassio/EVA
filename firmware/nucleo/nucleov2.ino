#include <ArduinoJson.h>
#include <Arduino.h>
#include <Servo.h>

//l'usb è lento, il print è ancora più lento, se riepmi il buffer perdi i caratteri successivi

// Macro used to define contants used in the code
#define NUM_SERVO 8

// #define MAX_Z 1750
#define SERVO_OFF 1550 // Value to write in order to stop the servo
#define MAX_INPUT_LENGTH 200
#define HAND_PWM_PERIOD 40000
#define WRIST_PWM_PERIOD 1000
#define SPEED 300

//hand pins
#define PIN_PWM_HAND A6 //actuator
#define PIN_EN_HAND  //actuator //NON C'È ENABLE PER L'HAND
#define PIN_DIR_HAND D4 //actuator

//wrist pins
#define PIN_PWM_WRIST A1
#define PIN_EN_WRIST D11
#define PIN_DIR_WRIST D12

int shift = 50; //shift for the optoisolator (add 50 to the PWM value)
int torque_on = 0;


// motors position definition
typedef enum
{
  FDX, 
  RSX, 
  RDX, 
  UPRSX, 
  FSX, 
  UPFDX, 
  UPFSX, 
  UPRDX, 
  HAND, 
  WRIST 
} motors_position_mapping;
/*
Motors:

  - RDX -> Rear Right
  - FSX -> Forward Left
  - FDX -> Forward Right
  - UPRSX -> Up Rear Left
  - RSX -> Rear Left
  - UPFDX -> Up Rear Right
  - UPFSX -> Up Forward Left
  - UPRDX -> Up Rear Right

See the ROV picture for a proper understanding of the motors mapping

*/
// servo related variables
//unsigned

// servo related variables
Servo servo[NUM_SERVO];
unsigned char servoPin[NUM_SERVO] = {D6,D10,A2,D3,D9,A3,A5,A0};
//unsigned char servoPin[NUM_SERVO] = {A0,D1,D2,D3,D4,D6,D7,D8};
//unsigned char servoPin = {D6};
// json parsing related variables
DeserializationError error;

void setup()
{
  Serial.begin(115200);

  int i;
  for (i = 0; i < 8; i++)
  {
    servo[i].attach(servoPin[i]);
    servo[i].writeMicroseconds(SERVO_OFF);
  }
  pinMode(A6, OUTPUT);
  pinMode(PIN_PWM_WRIST, OUTPUT);
  pinMode(PIN_EN_WRIST, OUTPUT);
  pinMode(D4, OUTPUT);
  pinMode(PIN_DIR_WRIST, OUTPUT);

  analogWrite(A6, 155);
  digitalWrite(D4, HIGH);

  digitalWrite(PIN_PWM_WRIST, HIGH);
  digitalWrite(PIN_EN_WRIST, HIGH); //active-low
  digitalWrite(PIN_DIR_WRIST, HIGH);
}



void loop() {
  if(Serial.available() != 0 ){
    static StaticJsonDocument<256> commandsIn;
  
    error = deserializeJson(commandsIn, Serial);
     if (error) {
       Serial.print(F("deserializeJson() failed: "));
       Serial.println(error.f_str());
       return;
     } /*else  {
       Serial.print(F("Recevied valid json document with "));
       Serial.print(commandsIn.size());
       Serial.println(F(" elements."));
       Serial.println(F("Pretty printed back at you:"));
       serializeJsonPretty(commandsIn, Serial);
       Serial.println();
       
     }*/
    switch ((unsigned char) commandsIn["TYPE"]) {
      case 'A': //per i motori
        servo[FDX].writeMicroseconds((int)commandsIn["FDX"]+shift);
        servo[FSX].writeMicroseconds((int)commandsIn["FSX"]+shift);
        servo[RDX].writeMicroseconds((int)commandsIn["RDX"]+shift);
        servo[RSX].writeMicroseconds((int)commandsIn["RSX"]+shift);
        servo[UPFDX].writeMicroseconds((int)commandsIn["UPFDX"]+shift);
        servo[UPRSX].writeMicroseconds((int)commandsIn["UPRSX"]+shift);
        servo[UPRDX].writeMicroseconds((int)commandsIn["UPRDX"]+shift);
        servo[UPFSX].writeMicroseconds((int)commandsIn["UPFSX"]+shift);
        break;
    
      case 'B': //per il braccio
        switch ((int) commandsIn["COMMAND"]) {
          case 0: //Rot claw ccw
            Serial.println("0");
            digitalWrite(PIN_DIR_WRIST, HIGH);
            digitalWrite(PIN_EN_WRIST, LOW);
            analogWriteFrequency(SPEED);
            analogWrite(PIN_PWM_WRIST,127);
            break;
          case 1: //Rot claw cw
            Serial.println("1");
            digitalWrite(PIN_EN_WRIST, LOW);
            digitalWrite(PIN_DIR_WRIST, LOW);
            analogWriteFrequency(SPEED);
            analogWrite(PIN_PWM_WRIST,127);
            
            break;
          case 2: //Stop wrist
          Serial.println("2");
            if(torque_on==1)
              analogWrite(PIN_PWM_WRIST,0);   //STOP WITH TORQUE
            else
              digitalWrite(PIN_EN_WRIST, HIGH); //NO TORQUE
            analogWriteFrequency(1000);
            break;
          case 3: //Close claw
            digitalWrite(PIN_DIR_HAND, LOW);
            analogWrite(PIN_PWM_HAND, 127);
            Serial.println("3");
            break; 
          case 4: //Open claw
            Serial.println("4");
            digitalWrite(PIN_DIR_HAND, HIGH);
            analogWrite(PIN_PWM_HAND, 127);
            break;
          case 5: //Stop claw
            Serial.println("5");
            analogWrite(PIN_PWM_HAND, 0);
            break;
          case 9: //TORQUE ON
            torque_on = 1;
            Serial.println("9");
            digitalWrite(PIN_EN_WRIST, LOW); 
            analogWrite(PIN_PWM_WRIST,0);
            break;
          case 7: //TORQUE OFF
            torque_on = 0;
            Serial.println("7");
            digitalWrite(PIN_EN_WRIST, HIGH); 
            break;
          default:
            break;
        }
        break;
      default:
        break;
    }
  }
}
