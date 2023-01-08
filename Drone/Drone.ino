#include <Wire.h>
#include <Servo.h>
#include "RF24.h"
#include "printf.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Initialize ESCs ***************************

byte servoPin1 = 3; // Y-minus
byte servoPin2 = 4; // Z-plus
byte servoPin3 = 5; // Z-minus
byte servoPin4 = 6; // Y-plus

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

byte potentiometerPin = A0; // analog input pin for thrust control

int motor_y_plus_thrust = 0;
int motor_y_minus_thrust = 0;
int motor_z_plus_thrust = 0;
int motor_z_minus_thrust = 0;

float base_thrust = 0;
float pitch_angle = 0;
float roll_angle = 0;

// Glove sensor data *************************

String base_thrust_control = "";
String pitch_control = "";
String roll_control = "";

// Initialize radio **************************

RF24 radio(9, 10); //CE, CSN

byte address[6] = "1Node"; //Any 5 character string for address, this is the address we'll tx and listen to on rx side

// Timers ************************************

const float CYCLE_DELAY_MS = 100;
float cycle_timer = millis();

const float DEBUG_DELAY_MS = 500;
float debug_timer = millis();

const float RADIO_INTERVAL = 10;
float radio_timer = millis();

unsigned long radio_detection_interval = 4000;
float radio_detection_timer = millis();

// IMU data arrays ***************************

float gyro_raw[3] = {0, 0, 0};
float gyro_offset[3] = {0, 0, 0};
float gyro_flat_offset[3] = {0, 0, 0};
float gyro_real[3] = {0, 0, 0};
float gyro_real_last[3] = {0, 0, 0};

// PID variables / constants ******************

float K_P_yaw = 2.0;
float K_P = 0.6;
float K_I = 0.0;
float K_D_yaw = 0.0;
float K_D = 0.0;

float angle_x = 0;
float angle_y_plus = 0;
float angle_y_minus = 0;
float angle_z_plus = 0;
float angle_z_minus = 0;

float integral_x_plus = 0;
float integral_x_minus = 0;
float integral_y_plus = 0;
float integral_y_minus = 0;
float integral_z_plus = 0;
float integral_z_minus = 0;

float diff_x = 0;
float diff_y_plus = 0;
float diff_y_minus = 0;
float diff_z_plus = 0;
float diff_z_minus = 0;

// ********************************************

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

//Function to configure the radio
void configureRadio() {
  radio.begin(); //starts radio
  radio.setPALevel(RF24_PA_HIGH); //Power level of radio (Can also be RF24_PA_MED, RF24_PA_HIGH, RF24_PA_MAX)
  radio.openReadingPipe(0, address); //Recieves on pipe 0 of this address (can have multiple pipes such that one rx node can receive from multiple tx nodes on same address.
  radio.startListening(); //Sets radio in rx mode
  radio.setAutoAck(0,false);
}

void setup() {
  
  Serial.begin(115200);

  // Radio setup
  printf_begin();
  configureRadio();
  
  // ESC calibration 
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  servo3.attach(servoPin3);
  servo4.attach(servoPin4);
  servo1.writeMicroseconds(1500); // 1500 is stop signal
  servo2.writeMicroseconds(1500);
  servo3.writeMicroseconds(1500);
  servo4.writeMicroseconds(1500);
  
  delay(7000); // Give time to ESC to recognize stop signal
  
  // Check if 9-axis IMU is connected properly
  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  // Using external crystal yields better accuracy
  bno.setExtCrystalUse(true);

  delay(1000);
  // Begin flat offset calculation
  calculate_flat_offset();
  Serial.print("Offset X: ");
  Serial.print(gyro_flat_offset[0]);
  Serial.print(" Offset Y: ");
  Serial.print(gyro_flat_offset[1]);
  Serial.print(" Offset Z: ");
  Serial.println(gyro_flat_offset[2]);
  
  delay(2000);
}

void loop() {

  if (millis() - cycle_timer > CYCLE_DELAY_MS) {
    read_imu_data();
    read_glove_data();
    calculate_offset();
    calculate_real_orientation();
    PID_controller();
    drive_motors();
    cycle_timer = millis();
  }

  if (millis() - debug_timer > DEBUG_DELAY_MS) {
    Serial.print("Raw X: ");
    Serial.print(gyro_raw[0]);
    Serial.print(" Raw Y: ");
    Serial.print(gyro_raw[1]);
    Serial.print(" Raw Z: ");
    Serial.println(gyro_raw[2]);

    Serial.print("Real X: ");
    Serial.print(gyro_real[0]);
    Serial.print(" Real Y: ");
    Serial.print(gyro_real[1]);
    Serial.print(" Real Z: ");
    Serial.println(gyro_real[2]);
    debug_timer = millis();
  }
  
}

void read_imu_data() {
  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);

  gyro_raw[0] = event.orientation.x;
  gyro_raw[1] = event.orientation.y;
  gyro_raw[2] = event.orientation.z;

  /* Also send calibration data for each sensor. */
 uint8_t sys, gyro, accel, mag = 0;
 bno.getCalibration(&sys, &gyro, &accel, &mag);

}

void calculate_flat_offset() {
  // Finds the flat orientation of the drone
  int positive_count = 0;
  int negative_count = 0;
  int samples = 1000;
  for (int i = 0; i < samples; i++) {
    read_imu_data();
    gyro_flat_offset[0] += gyro_raw[0];
    gyro_flat_offset[1] += gyro_raw[1];
    if (gyro_raw[2] < 0) {
      gyro_raw[2] = 180 + gyro_raw[2];
    } else {
      gyro_raw[2] = -180 + gyro_raw[2];
    }
    gyro_flat_offset[2] += gyro_raw[2];
  }


  gyro_flat_offset[0] /= samples;
  gyro_flat_offset[1] /= samples;
  gyro_flat_offset[2] /= samples;
  gyro_real_last[0] = gyro_real[0];
  gyro_real_last[1] = gyro_real[1];
  gyro_real_last[2] = gyro_real[2];
}

void read_glove_data() {
  if (millis() - radio_detection_timer > radio_detection_interval) {
    Serial.println("Configuring radio");
    configureRadio();
    radio_detection_timer = millis();
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
      
      bool writing_pitch = true;
      bool writing_roll = true;
      bool writing_base_thrust = true;

      pitch_control = "";
      roll_control = "";
      base_thrust_control = "";
      
      for (int i=0; i<20; i++) {
        if (text[i] == 32) {
          if (writing_pitch == true) {
            writing_pitch = false;
            continue;
          }
          if (writing_roll == true) {
            writing_roll = false;
            continue;
          }
          if (writing_base_thrust == true) {
            break;
          }
        }
        if (writing_pitch == true) {
          pitch_control = pitch_control + text[i];
        } else if (writing_roll == true) {
          roll_control = roll_control + text[i];
        } else {
          base_thrust_control = base_thrust_control + text[i];
        }
      }

      pitch_angle = pitch_control.toInt();
      roll_angle = roll_control.toInt();
      base_thrust = base_thrust_control.toInt();

      Serial.print("Pitch Angle: ");
      Serial.println(pitch_angle);
      Serial.print("Roll Angle: ");
      Serial.println(roll_angle);
      Serial.print("Base Thrust: ");
      Serial.println(base_thrust);
      radio_detection_timer = millis();
    }
  }
}

void calculate_offset() {
  
  gyro_offset[0] = gyro_flat_offset[0];
  gyro_offset[1] = gyro_flat_offset[1] + roll_angle;
  gyro_offset[2] = gyro_flat_offset[2] - pitch_angle;

}

void calculate_real_orientation() {
  gyro_real[0] = gyro_raw[0] - gyro_offset[0];
  gyro_real[1] = gyro_raw[1] - gyro_offset[1];
  if (gyro_raw[2] < 0) {
    gyro_raw[2] = 180 + gyro_raw[2];
  } else {
    gyro_raw[2] = -180 + gyro_raw[2];
  }
  gyro_real[2] = gyro_raw[2] - gyro_offset[2]; 
}

void PID_controller() {
  // Yaw
  angle_x = gyro_real[0];

  // Roll
  angle_y_plus = -gyro_real[1];
  angle_y_minus = gyro_real[1];

  // Pitch
  angle_z_plus = -gyro_real[2];
  angle_z_minus = gyro_real[2];

  // Yaw
  diff_x = gyro_real[0] - gyro_real_last[0];

  // Roll
  diff_y_plus = -(gyro_real[1] - gyro_real_last[1]);
  diff_y_minus = gyro_real[1] - gyro_real_last[1];

  // Pitch
  diff_z_plus = -(gyro_real[2] - gyro_real_last[2]);
  diff_z_minus = gyro_real[2] - gyro_real_last[2];


  int potVal = analogRead(potentiometerPin); // read input from potentiometer.
  
  int pwmVal = map(potVal,0, 1023, 1000, 1900); // maps potentiometer values to PWM value.
  
  motor_y_plus_thrust = pwmVal - K_P*angle_y_plus + K_D*diff_y_plus - K_P_yaw*angle_x - K_D_yaw*diff_x;
  motor_y_minus_thrust = pwmVal - K_P*angle_y_minus + K_D*diff_y_minus - K_P_yaw*angle_x - K_D_yaw*diff_x;
  motor_z_plus_thrust = pwmVal - K_P*angle_z_plus + K_D*diff_z_plus + K_P_yaw*angle_x + K_D_yaw*diff_x;
  motor_z_minus_thrust = pwmVal - K_P*angle_z_minus  + K_D*diff_z_minus + K_P_yaw*angle_x + K_D_yaw*diff_x;

  Serial.println("-----------------");
  Serial.println(motor_y_plus_thrust);
  Serial.println(motor_y_minus_thrust);
  Serial.println(motor_z_plus_thrust);
  Serial.println(motor_z_minus_thrust);
  Serial.println("-----------------");
  gyro_real_last[0] = gyro_real[0];
  gyro_real_last[1] = gyro_real[1];
  gyro_real_last[2] = gyro_real[2];
  
}

void drive_motors() {

  servo1.writeMicroseconds(motor_y_minus_thrust); // Send signal to ESC.
  servo2.writeMicroseconds(motor_z_plus_thrust); // Send signal to ESC.
  servo3.writeMicroseconds(motor_z_minus_thrust); // Send signal to ESC.
  servo4.writeMicroseconds(motor_y_plus_thrust); // Send signal to ESC.
}

// Send motor signal to drone
