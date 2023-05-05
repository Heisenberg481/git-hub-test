// #Define

#define servoPin 2       // Servo pin
#define leftMotorFrw 3   // Forward direction l293d left motor
#define leftMotorRvs 4   // Reverse direction l293d left motor
#define rightMotorFrw 7  // Forward direction l293d right motor
#define rightMotorRvs 8  // Reverse direction l293d left motor
#define leftMotorPWM 5   // Speed left motor (PWM)
#define rightMotorPWM 6  // Speed right motor (PWM)
#define batCharge A0     // Batery voltage input

// #Define
//-----------------------------------------------------------------------------------------------------------------
// #Include

#include <stdint.h>
#include <SPI.h>       // SPI library
#include "nRF24L01.h"  // NRF24 library
#include "RF24.h"      // NRF24 library
#include <Servo.h>     // Servo library

// #Include
//-----------------------------------------------------------------------------------------------------------------
// Module settings

RF24 radio(9, 10);
byte pipeNo;                                                                   // Setup of NRF24 on 9 and 10 pin
byte address[][6] = { "1Node", "2Node", "3Node", "4Node", "5Node", "6Node" };  //Possible number of pipes

// Module settings
//-----------------------------------------------------------------------------------------------------------------
// Structures

struct ReceiveStruct {  // Receive structure
  int leftMotor;
  int rightMotor;
  bool servo;
};

struct TransStruct {  // Transmission structure
  float BatCharge;
};

// Structures
//-----------------------------------------------------------------------------------------------------------------
// Gloabal variables

Servo servo;  //Creating structure
TransStruct TransStruct;
ReceiveStruct ReceiveStruct;

//  Gloabal variables
//-----------------------------------------------------------------------------------------------------------------

void setup() {

  servo.attach(servoPin);          // Setup servo pin
  pinMode(leftMotorFrw, OUTPUT);   // Setup left motor forward pin as output
  pinMode(leftMotorRvs, OUTPUT);   // Setup left motor reverse pin as output
  pinMode(rightMotorFrw, OUTPUT);  // Setup right motor forward pin as output
  pinMode(rightMotorRvs, OUTPUT);  // Setup right motor reverse pin as output
  pinMode(batCharge, INPUT);       // Setup battery voltage pin as input

  Serial.begin(9600);  // Setup serial port
  radioSetup();
}

void loop() {

  while (radio.available(&pipeNo)) {                    // listen to the broadcast from all pipes
    radio.read(&ReceiveStruct, sizeof(ReceiveStruct));  // read the incoming signal

    TransStruct.BatCharge = 750;
    radio.writeAckPayload(pipeNo, &TransStruct, sizeof(TransStruct));

    Serial.print("Recieved: ");
    Serial.println(ReceiveStruct.leftMotor);
    Serial.println(ReceiveStruct.rightMotor);
    Serial.println(ReceiveStruct.servo);
  }

  Motor(ReceiveStruct.leftMotor, leftMotorPWM, leftMotorFrw, leftMotorRvs);
  Motor(ReceiveStruct.rightMotor, rightMotorPWM, rightMotorFrw, rightMotorRvs);

  servoCtrl(ReceiveStruct.servo, 0, 180);
}

void Motor(int Speed, int pwmPin, int FrwPin, int RvsPin) {
  if (Speed > 5) {
    analogWrite(pwmPin, Speed);
    digitalWrite(FrwPin, HIGH);
    digitalWrite(RvsPin, LOW);

  } else if (Speed < -5) {
    analogWrite(pwmPin, (Speed * -1));
    digitalWrite(FrwPin, LOW);
    digitalWrite(RvsPin, HIGH);

  } else {
    digitalWrite(FrwPin, LOW);
    digitalWrite(RvsPin, LOW);
  }
}

void servoCtrl(bool button, int stopAngle, int pressAngle) {
  if (button == 1) {
    servo.write(pressAngle);
  } else {
    servo.write(stopAngle);
  }
}

void radioSetup() {
  radio.begin();                         // Activate module
  radio.setAutoAck(1);                   // Acknowledgment mode, 1 ON 0 OFF
  radio.setRetries(0, 15);               // Time of retries, number of retries
  radio.enableAckPayload();              // Allow sending data in response to an incoming signal
  radio.enableDynamicPayloads();         // Size of package
  radio.openReadingPipe(1, address[0]);  // Data channel
  radio.setChannel(0x60);                // Channel with not noises
  radio.setPALevel(RF24_PA_LOW);         // Level of power tranciever RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
  radio.setDataRate(RF24_250KBPS);       // Speed ​​of exchange RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
  radio.powerUp();                       // Start work
  radio.startListening();                // Listen, receive
}
