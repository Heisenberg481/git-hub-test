// #Define

#define PIN_VRX A0   // Joystick right motor
#define PIN_VRY A1   // Joystick left motor
#define SERVO_BUT 8  // Servo button

#define T_10ms 10  // Transmition delay
#define T_100ms 100
#define T_1000ms 1000

#define analogMax 1024  // Maximum value of analog input
#define analogHalf 512  // Half value of maximum analog input
#define pwmMax 256      // Maximum value of PWM

// #Define
//-----------------------------------------------------------------------------------------------------------------
// #Include

#include <stdint.h>
#include <SPI.h>                // SPI library
#include "RF24.h"               // NRF24 library
#include "nRF24L01.h"           // NRF24 library
#include <LiquidCrystal_I2C.h>  // LCD1602 library

// #Include
//-----------------------------------------------------------------------------------------------------------------
// Module settings

RF24 radio(9, 10);                                                             // Setup of NRF24 on 9 and 10 pin
byte address[][6] = { "1Node", "2Node", "3Node", "4Node", "5Node", "6Node" };  //Possible number of pipes
LiquidCrystal_I2C lcd(0x27, 16, 2);                                            // LCD1602 setup

// Module settings
//-----------------------------------------------------------------------------------------------------------------
// Structures

struct ReceiveStruct {  // Receive structure
  float BatCharge;
};

struct TransStruct {  // Transmission structure
  int lefMotor;
  int rightMotor;
  bool servo;
};

// Structures
//-----------------------------------------------------------------------------------------------------------------
// Gloabal variables

TransStruct TransStruct;  //Creating structure
ReceiveStruct ReceiveStruct;

uint32_t timer = 0;       // Send timer
uint32_t RSSI_timer = 0;  // Signal quality calculation timer
uint32_t LCD_timer = 0;   // Update display timer

int trnsmtd_pack = 1, failed_pack = 0;
float batteryCharge;
uint8_t rssi;

//  Gloabal variables
//-----------------------------------------------------------------------------------------------------------------

void setup() {

  pinMode(SERVO_BUT, INPUT);  // Setup servo button as input
  Serial.begin(9600);         // Setup serial port
  radioSetup();
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Battery: ");
  lcd.setCursor(13, 0);
  lcd.print("%");
  lcd.setCursor(0, 1);
  lcd.print("Signal: ");
  lcd.setCursor(13, 1);
  lcd.print("%");
}


void loop() {
  int leftSpeed = analogRead(PIN_VRX);
  int rightSpeed = analogRead(PIN_VRY);

  TransStruct.lefMotor = scalled1(leftSpeed);
  TransStruct.rightMotor = scalled1(rightSpeed);

  TransStruct.servo = digitalRead(SERVO_BUT);

  batteryCharge = scalled(ReceiveStruct.BatCharge, 860, 614, 100, 0);


  if (millis() - timer >= 25) {
    timer = millis();
    int rslt = radio.write(&TransStruct, sizeof(TransStruct));
    Serial.println(rslt);
    if (radio.available()) {
      radio.write(&TransStruct, sizeof(TransStruct));
      radio.read(&ReceiveStruct, sizeof(ReceiveStruct));
      trnsmtd_pack++;
    } else {
      failed_pack++;
    }
    /*    if (radio.write(&TransStruct, sizeof(TransStruct))) {
      trnsmtd_pack++;
      if (!radio.available()) {
      } else {
        while (radio.available()) {
          radio.read(&ReceiveStruct, sizeof(ReceiveStruct));
        }
      }
    } 
    else {
      failed_pack++;
    }
*/
  }

  if (millis() - RSSI_timer > T_1000ms) {
    RSSI_timer = millis();
    rssi = (1 - ((float)failed_pack / trnsmtd_pack)) * 100;
    Serial.println(failed_pack);
    Serial.println(trnsmtd_pack);
    Serial.println(batteryCharge);
    Serial.println("");

    lcd.setCursor(10, 0);
    lcd.print(failed_pack);
    lcd.setCursor(10, 1);
    lcd.print(failed_pack);
    failed_pack = 0;
    trnsmtd_pack = 0;
  }

  /*  if (millis() - LCD_timer > 1000) {
    LCD_timer = millis();
  }
  */
}

float scalled(int input, int maxIn, int minIn, float maxOut, float minOut) {
  float val;
  if (input >= maxIn) {
    return val = maxOut;
  } else if (input <= minIn) {
    return val = minOut;
  } else {
    val = (input - minIn) * (maxOut - minOut) / (maxIn - minIn) + minOut;
    return val;
  }
}

void radioSetup() {
  radio.begin();                      // Activate module
  radio.setAutoAck(1);                // Acknowledgment mode, 1 ON 0 OFF
  radio.setRetries(0, 15);            // Time of retries, number of retries
  radio.enableAckPayload();           // Allow sending data in response to an incoming signal
  radio.enableDynamicPayloads();            // Size of package
  radio.openWritingPipe(address[0]);  // Data channel
  radio.setChannel(0x60);             // Channel with not noises
  radio.setPALevel(RF24_PA_MIN);      // Level of power tranciever RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
  radio.setDataRate(RF24_250KBPS);    // Speed ​​of exchange RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
  radio.powerUp();                    // Start work
  radio.stopListening();              // Not listen, tranceive
}

int scalled1(int input) {
  int val;
  input = input - analogHalf;
  val = (((input * pwmMax) / analogMax) * 2) + 1;
  return val;
}