/*
  Obstacle Avoiding Vacuum Cleaner v1.0

  Features:
    Over-Current Mitigation
    Software-Activated Remote Control
    Low-Battery Sensor
    Buzzer
    LCD

  Author:
    Erwin Bernard B. Talento

*/
// =========================================================== LIBRARIES:
#include <Wire.h>
#include <Servo.h>
#include <Ultrasonic.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_MotorShield.h>
// =========================================================== TESTING SWITCHES:
const boolean avoid_overcurrent = true; // for Powerbank or any power source that shut downs due to over-current
const boolean skip_lcd = false;
const boolean skip_dcwheel = false;
const boolean skip_servo = false;
const boolean skip_vsensor = false;
const boolean skip_usensor = false;
const boolean skip_buzzer = false;
const boolean skip_vacuum = false;
const boolean skip_lowbatt = false;
// =========================================================== PRESET VALUES:
// Analog = A0,A1,A2,A3,A4,A5
// Adafruit Motor Shield = 1,2,3,4
// Digital Non-PWM = 2,4,7,8,12,13
// Digital PWM = 3,5,6,9,10,11
const uint8_t Pin_Remote = 2;  // Digital Non-PWM
const uint8_t Pin_Relay = 4;  // Digital Non-PWM
const uint8_t Pin_Buzzer1 = 5;  // Digital PWM
const uint8_t Pin_Servo1 = 10;  // Digital PWM
const uint8_t Pin_LeftWheel = 1;  // Adafruit Motor Shield
const uint8_t Pin_RightWheel = 2; // Adafruit Motor Shield
const uint8_t Pin_USensor1_Trigger = 7;  // Digital Non-PWM
const uint8_t Pin_USensor1_Echo = 8;  // Digital Non-PWM
const uint8_t Pin_VSensor1 = A2; // Analog
const uint8_t Pin_Vacuum_AEn = 3;  // Digital PWM
const uint8_t Pin_Vacuum_AIn1 = 6;  // Digital PWM
const uint8_t Pin_Vacuum_AIn2 = 9;  // Digital PWM
const uint8_t MaxSpeed_Vacuum = 180;  // 180
const uint8_t MinSpeed_Vacuum = 50;  // 50
const uint16_t Vacuum_Delay = 2;
// Set Voltage Sensor Credentials:
const float Resistor1 = 30000.0; // 30K Ohm Resistor
const float Resistor2 = 7500.0; // 30K Ohm Resistor
const float LowBatt_Voltage = 4.5;
// Set the speed to start, from 0 (off) to 255 (max speed)
const uint8_t MaxSpeed_AllWheels = 250; // for Flat Glass Surface = 200 & Rough Surface = 250
const uint8_t MinSpeed_AllWheels = 200;
const uint16_t Moving_Delay = 1;
const uint16_t Turning_Delay = 700;
const uint16_t Backward_Delay = 700;
const uint8_t LeftWheel_Offset = 0;
const uint8_t RightWheel_Offset = 0;
const uint8_t Colliding_Distance = 24;
const uint8_t Turn_Distance = Colliding_Distance+10;
const uint8_t Servo_Offset = 12;
// Default frequency is 1.6KHz. Higher frequencies will produce less audible hum in operation, but may result in lower torque with some motors
const int16_t Wheel_Frequency = 1000; // 1KHz
// =========================================================== DECLARATION:
Servo Servo1;   // Servo Motor for Ultrasonic Sensor
LiquidCrystal_I2C LCD(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
Ultrasonic USensor1(Pin_USensor1_Trigger,Pin_USensor1_Echo);    // set Ultrasonic Sensor Pin
Adafruit_MotorShield AFMS = Adafruit_MotorShield();   // Create the motor shield object with the default I2C address
Adafruit_DCMotor *LeftWheel = AFMS.getMotor(Pin_LeftWheel);   // Set Left Wheel Port
Adafruit_DCMotor *RightWheel = AFMS.getMotor(Pin_RightWheel);   // Set Right Wheel Port
// =========================================================== GLOBAL VARIABLES:
boolean is_Initialized = false;
boolean is_turnedOn = false;
float current_Voltage = 0.0;
int16_t current_Distance = 0;
int16_t degree_Pan = 0; 
int16_t max_Angle = 0;
uint8_t Speed_Starting = 0;
// =========================================================== SETUP
void setup () {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Vacuum Cleaner");
  // ==========================
  // Configure Pins to behave as Input or Output
  // ==========================
  pinMode(Pin_Remote, INPUT);
  pinMode(Pin_Relay, OUTPUT);
  pinMode(Pin_Buzzer1,OUTPUT);
  pinMode(Pin_VSensor1, INPUT);
  pinMode(Pin_Vacuum_AEn, OUTPUT);
  pinMode(Pin_Vacuum_AIn1, OUTPUT);
  pinMode(Pin_Vacuum_AIn2, OUTPUT);
  analogWrite(Pin_Vacuum_AEn, 0);
  digitalWrite(Pin_Vacuum_AIn1, LOW);
  digitalWrite(Pin_Vacuum_AIn2, LOW);
  LCD.init();
  LCD.noBacklight();
  Initialize(); // this will Initialized because both is_Initialized & is_turnedOn are False
  // ==========================
  // End of Setup
  // ==========================
}
// =========================================================== LOOP
void loop () {
  Initialize(); // this will Initialized depending if it was initialized before
  if (is_Initialized == true) {
    // ==========================
    // Already Initialized
    // ==========================
    if (is_turnedOn == false) {
      // ==========================
      // Turned-Off
      // ==========================
      set_Buzzer(600,2000); //beep the buzzer for 2 second at 700hertz
      Servo1.detach();
      move_Stop();  // Stop the Wheels
      analogWrite(Pin_Vacuum_AEn, 0);
      digitalWrite(Pin_Vacuum_AIn1, LOW);
      digitalWrite(Pin_Vacuum_AIn2, LOW);
      display_LCD("Turning Off...");
      delay(2000);
      LCD.clear();  // Clear the screen
      LCD.noBacklight();  // turn off
      is_Initialized = false;
    } else {
      // ==========================
      // Turned-On
      // ==========================
      Serial.println("Device Turned ON");
      display_LCD("Project AVAC 1.0");
      // ==========================
      // Avoid Over-Current
      // ==========================
      if (avoid_overcurrent == true) {
        move_Stop();  // Stop the Wheels to Reset it back to speed 0
      }
      // ==========================
      // Run the Vacuum
      // ==========================
      if (skip_vacuum == false) {
        for (Speed_Starting = MinSpeed_Vacuum; Speed_Starting < MaxSpeed_Vacuum; Speed_Starting +=2) {
          analogWrite(Pin_Vacuum_AEn, Speed_Starting);
          delay(Vacuum_Delay);
        }
      }
      // ==========================
      // Check if Low Battery:
      // ==========================
      current_Voltage = get_Voltage();
      if (current_Voltage != 0.00) {
        if ((current_Voltage < LowBatt_Voltage) && (skip_lowbatt == false)) {
          // ==========================
          // Sound the Alarm if Battery is Low
          // ==========================
          set_Buzzer(800,2000);  // generate 800Hz tone every 2 second
          LCD.noBacklight();  // turn off
          delay(2000);
          LCD.backlight();  // turn on again
        }
        Serial.println("Voltage: " + String(current_Voltage,2));
      }
      // ==========================
      // Run the Servo
      // ==========================
      if (skip_servo == false) {
        Servo1.write(144+Servo_Offset);
        delay(120); 
      }
      // ==========================
      // Run the Wheels
      // ==========================
      if (skip_dcwheel == false) {
        int16_t current_Left = 0;
        int16_t current_Front = 0;
        int16_t current_Right = 0;
        int16_t current_Distance = 0;
        // ==========================
        // Run Forward
        // ==========================
        LeftWheel->run(FORWARD);
        RightWheel->run(FORWARD);
        // ==========================
        // Avoid Over-Current
        // ==========================
        if (avoid_overcurrent == true) {
          move_Increment(); // Incremental Speed from Minimum Speed to Maximum Speed
        } else {
          LeftWheel->setSpeed(MaxSpeed_AllWheels);
          RightWheel->setSpeed(MaxSpeed_AllWheels);
        }
        for (degree_Pan = 144; degree_Pan >= 36; degree_Pan-=18) {
          if (skip_servo == false) {
            Servo1.write(degree_Pan+Servo_Offset);
            delay(90);
          }
          // ==========================
          // Get Current Distance
          // ==========================
          if (skip_usensor == false) {
            delay(70);
            current_Distance=USensor1.Ranging(CM);  //get the current distance;
          } else {
            current_Distance = 0;
          }
          if (current_Distance < Colliding_Distance) {
            // ==========================
            // Avoid Over-Current
            // ==========================
            if (avoid_overcurrent == true) {
              analogWrite(Pin_Vacuum_AEn, 0);   // Disable the Vacuum
              move_Stop();  // Stop the Wheels to Reset it back to speed 0
            }
            display_LCD("Obstacle Found!");
            // ==========================
            // Run Backward
            // ==========================
            LeftWheel->run(BACKWARD);
            RightWheel->run(BACKWARD);
            // ==========================
            // Avoid Over-Current
            // ==========================
            if (avoid_overcurrent == true) {
              move_Increment(); // Incremental Speed from Minimum Speed to Maximum Speed
            }
            delay(Backward_Delay);
            // ==========================
            // Avoid Over-Current
            // ==========================
            if (avoid_overcurrent == true) {
              move_Stop();  // Stop the Wheels to Reset it back to speed 0
            }
            // ==========================
            // Turn Left or Right
            // ==========================
            if (max_Angle < 90) {
              // ==========================
              // Turn Right
              // ==========================
              display_LCD("Turning Right");
              set_Buzzer(500,100);
              LeftWheel->run(FORWARD);
              RightWheel->run(BACKWARD);
            } else {  // >90
              // ==========================
              // Turn Left
              // ==========================
              display_LCD("Turning Left");
              set_Buzzer(500,100);
              LeftWheel->run(BACKWARD);
              RightWheel->run(FORWARD);
            }
            // ==========================
            // Avoid Over-Current
            // ==========================
            if (avoid_overcurrent == true) {
              move_Increment(); // Incremental Speed from Minimum Speed to Maximum Speed
            }
            delay(Turning_Delay);
            // ==========================
            // Move Forward Again??????
            // ==========================
            move_Stop();  // Stop the Wheels to Reset it back to speed 0
            LeftWheel->run(FORWARD);
            RightWheel->run(FORWARD);
            // ==========================
            // Avoid Over-Current
            // ==========================
            if (avoid_overcurrent == true) {
              move_Increment(); // Incremental Speed from Minimum Speed to Maximum Speed
            }
          } else if (current_Distance < Turn_Distance) {
            // ==========================
            // Avoid Over-Current
            // ==========================
            if (avoid_overcurrent == true) {
              analogWrite(Pin_Vacuum_AEn, 0);   // Disable the Vacuum
              move_Stop();  // Stop the Wheels to Reset it back to speed 0
            }
            // Set Path if Left or Right
            if (degree_Pan < 90) {
              // ==========================
              // Look Left
              // ==========================
              display_LCD("Leaning Left");
              set_Buzzer(500,100);
              LeftWheel->run(BACKWARD);
              // ==========================
              // Avoid Over-Current
              // ==========================
              if (avoid_overcurrent == true) {
                move_Increment(); // Incremental Speed from Minimum Speed to Maximum Speed
              }
              delay(Turning_Delay);
              // ==========================
              // Avoid Over-Current
              // ==========================
              if (avoid_overcurrent == true) {
                move_Stop();  // Stop the Wheels to Reset it back to speed 0
              }
              LeftWheel->run(FORWARD);
            } else {  // >90
              // ==========================
              // Look Right
              // ==========================
              display_LCD("Leaning Right");
              set_Buzzer(500,100);
              RightWheel->run(BACKWARD);
              // ==========================
              // Avoid Over-Current
              // ==========================
              if (avoid_overcurrent == true) {
                move_Increment(); // Incremental Speed from Minimum Speed to Maximum Speed
              }
              delay(Turning_Delay);
              // ==========================
              // Avoid Over-Current
              // ==========================
              if (avoid_overcurrent == true) {
                move_Stop();  // Stop the Wheels to Reset it back to speed 0
              }
              RightWheel->run(FORWARD);
            }
            // ==========================
            // Avoid Over-Current
            // ==========================
            if (avoid_overcurrent == true) {
              move_Increment(); // Incremental Speed from Minimum Speed to Maximum Speed
            }
          }
          if (current_Distance > current_Distance) {
            max_Angle = degree_Pan;
          }
          if (degree_Pan > 90 && current_Distance > current_Left) {
            current_Left = current_Distance;
          }
          if (degree_Pan == 90 && current_Distance > current_Front) {
            current_Front = current_Distance;
          }
          if (degree_Pan < 90 && current_Distance > current_Right) {
            current_Right = current_Distance;
          }
        }
      }
    }
  }
  // ==========================
  // End of Loop
  // ==========================
}
// =========================================================== INITIALIZE
void Initialize () {
  // ==========================
  // Check if Remote Control is turned on
  // ==========================
  if (digitalRead(Pin_Remote) == HIGH) {
    // ==========================
    // Remote Turned-On
    // ==========================
    is_turnedOn = true;
    Serial.println("Remote Turned ON");
    digitalWrite(Pin_Relay, LOW);  // Connect Low-Trigger Relay
    if (is_Initialized == true) {
      // ==========================
      // Initialized Already
      // ==========================
      Serial.println("No Required Initialization");
      return; // already initialized
    } else {
      // ==========================
      // Start of Initialization
      // ==========================
      set_Buzzer(700,1500);   // beep the buzzer for 2 second at 700hertz
      if (skip_lcd == false) {
        // ==========================
        // Initialize the LCD
        // ==========================
        LCD.init();
        LCD.clear();  // Clear the screen
        LCD.begin(16,2);  // Define 16 columns and 2 rows of LCD display
        LCD.backlight();  // Power ON the back light;
        LCD.setCursor(0,0); // Defining position to write from first column, first row
        display_LCD("Initializing...");
        delay(2000);
      }
      // ==========================
      // Initialize Vacuum
      // ==========================
      if (skip_vacuum == false) {
        digitalWrite(Pin_Vacuum_AIn1, LOW);
        digitalWrite(Pin_Vacuum_AIn2, HIGH);
      }
      // ==========================
      // Initialize Brushed DC Motors
      // ==========================
      if (skip_dcwheel == false) {
        AFMS.begin(Wheel_Frequency);
        LeftWheel->setSpeed(MaxSpeed_AllWheels);
        RightWheel->setSpeed(MaxSpeed_AllWheels);
        move_Stop();  // Stop the Wheels
      }
      // ==========================
      // Initialize Servo Motor
      // ==========================
      if (skip_servo == false) {
        Servo1.attach(Pin_Servo1);  // Attach a servo to preset pin
        Servo1.write(90+Servo_Offset); // Make servo go to 90 degrees 
        delay(1000);
      }
    }
    // ==========================
    // End of Initialization
    // ==========================
    is_Initialized = true;
  } else {
    // ==========================
    // Remote Turned-Off
    // ==========================
    is_turnedOn = false;
    Serial.println("Remote Turned Off");
    digitalWrite(Pin_Relay, HIGH);   // Disconnect Low-Trigger Relay
    return; // there's no need to process bottom procedures
  }
}
// =========================================================== MOVE_INCREMENT
void move_Increment () {
  if (skip_dcwheel == false) {
    for (Speed_Starting = MinSpeed_AllWheels; Speed_Starting < MaxSpeed_AllWheels; Speed_Starting +=2) {
      LeftWheel->setSpeed(Speed_Starting+LeftWheel_Offset);
      RightWheel->setSpeed(Speed_Starting+RightWheel_Offset);
      delay(Moving_Delay);
    }
  }
}
// =========================================================== MOVE_STOP
void move_Stop () {
  if (skip_dcwheel == false) {
    // Stop the motor.  This removes power from the motor and is equivalent to setSpeed(0)
    LeftWheel->run(RELEASE);
    RightWheel->run(RELEASE);
  }
}
// =========================================================== SET_BUZZER
void set_Buzzer (uint16_t frequency, uint16_t duration) {
  if (skip_buzzer == false) {
    tone(Pin_Buzzer1,frequency,duration);
  }
}
// =========================================================== GET_VOLTAGE
float get_Voltage () {
  float Voltage_Out = 0.0;
  float Voltage_In = 0.0;
  if (skip_vsensor == false) {
    Voltage_Out = (analogRead(Pin_VSensor1) * 5.0) / 1024.0;
    Voltage_In = Voltage_Out / (Resistor2/(Resistor1+Resistor2));
  }
  return Voltage_In;  // Actual Voltage
}
// =========================================================== DISPLAY_LCD
void display_LCD (String LCD_Msg) {
  if (skip_lcd == false) {
    String VoltLabel = "Voltage";
    LCD.clear();  // Clear the screen
    LCD.setCursor(0,0); // 1st column, 1st row
    LCD.print(LCD_Msg);
    LCD.setCursor(0,1); // 1st column, 2nd row
    current_Voltage = get_Voltage();
    if (current_Voltage < LowBatt_Voltage && (skip_lowbatt == false)) {
      VoltLabel = "LowBatt";
    }
    LCD.print(VoltLabel + ": " + String(current_Voltage,2) + "v"); // display Battery Voltage
  }
  Serial.println(LCD_Msg);
}
// =========================================================== END-OF-FILE
