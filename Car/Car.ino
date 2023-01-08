#include "RF24.h"
#include "printf.h"
#include <Adafruit_MotorShield.h>

// Motor initialization
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *leftMotor = AFMS.getMotor(3);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(4);

int left_wheel_speed = 40;
int right_wheel_speed = 40;
float radio_detection_timer = millis();
unsigned long radio_detection_interval = 4000;


bool writing_left = true;

RF24 radio(9, 10); //CE, CSN
/**********************************************************/
 
byte address[6] = "1Node"; //Any 5 character string for address, this is the address we'll tx and listen to on rx side
 

/**********************************************************/
//Function to configure the radio
void configureRadio() {
  radio.begin(); //starts radio
  radio.setPALevel(RF24_PA_HIGH); //Power level of radio (Can also be RF24_PA_MED, RF24_PA_HIGH, RF24_PA_MAX)
  radio.openReadingPipe(0, address); //Recieves on pipe 0 of this address (can have multiple pipes such that one rx node can receive from multiple tx nodes on same address.
  radio.startListening(); //Sets radio in rx mode
  radio.setAutoAck(0,false);
}
 
 
/**********************************************************/

const int RADIO_INTERVAL = 10;
float radio_timer = millis();

unsigned long MOTOR_INTERVAL = 10;
float motor_timer = millis();

void setup() {
  Serial.begin(115200);
  printf_begin();
  configureRadio();
  
  if (!AFMS.begin()) {
    Serial.println("Could not find Motor Shield");
    
  }
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
}


void handle_motors() {
  leftMotor->setSpeed(left_wheel_speed);
  rightMotor->setSpeed(right_wheel_speed);
}

void handle_radio() {
  if (millis() - radio_detection_timer > radio_detection_interval) {
    radio_detection_timer = millis();
    Serial.println("Configuring radio");
    configureRadio();
  }
  if (radio.available()) { //returns bool value, so only execute if radio indicates it has data ready to output
    Serial.println("Radio receiving");
    uint32_t failTimer = millis();
    char text[32] = ""; //empty 32 char string to store received message in (good practice to make sure the payload are both same length on tx and rx side).
    while (radio.available())
    {
      if (millis() - failTimer > 500) { //Sanity check to make sure radios aren't false reporting always having data to rx (this would indicate error)
        Serial.println("Radio available failure detected");
        break;
      }
      radio.read(&text, sizeof(text));// Get the payload
      Serial.println(text);
      String left_data;
      String right_data;
      
      for (int i=0; i<10; i++) {
        if (text[i] == 32) {
          if (writing_left == false) {
            break;
          }
          writing_left = false;
          continue;
        }
        if (writing_left == true) {
          left_data = left_data + text[i];
        } else {
          right_data = right_data + text[i];
        }
      }
      writing_left = true;

      left_wheel_speed = left_data.toInt();
      right_wheel_speed = right_data.toInt();

      Serial.print("Left wheel speed: ");
      Serial.println(left_wheel_speed);
      Serial.print("Right wheel speed: ");
      Serial.println(right_wheel_speed);
      radio_detection_timer = millis();
    }
  }
}

void loop() {
  if (millis() - motor_timer > MOTOR_INTERVAL) {
    handle_motors();
    motor_timer = millis();
  }

  if (millis() - radio_timer > RADIO_INTERVAL) {
    handle_radio();
    radio_timer = millis();
  }
}
