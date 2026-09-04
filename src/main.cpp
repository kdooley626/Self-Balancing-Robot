#include <Arduino.h>
#include <Wire.h>
#include "IMU.h"
#include "Filter.h"
#include "LQR.h"
#include "Motors.h"
#include "TestConfig.h"

Motor motor1 = {25, 27, 26, 155, 150};
Motor motor2 = {13, 32, 33, 150, 155};

struct motorCommand {
  String MotorState;
  int MotorSpeed;
};

bool timeTrack(unsigned long dt, unsigned long t0);
float rad2deg(float rad);
void runImuOnly();
void runMotorsOnly();
motorCommand parseMotorString(String motorInput);
void runLQROnly();
void runFullBalance();


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);
  //Initialize IMU sensor
  ImuBegin();
  // get starting values
  ImuSample s0 = ImuRead();
  filterBegin(s0.accel_theta);
  if (ACTIVE_MODE == motors_only || ACTIVE_MODE == full_balance) {
    setPWM(motor1);
    setPWM(motor2);
  }
  
}

void loop() {
  switch (ACTIVE_MODE) {
    case IMU_only: runImuOnly(); break;
    case motors_only: runMotorsOnly(); break;
    case LQR_only: runLQROnly(); break;
    case full_balance: runFullBalance(); break;
  }

  
}
void runFullBalance() {
  static unsigned long t0 = micros();
  static unsigned long t0_print = micros();
  unsigned long interval = 10000; //100Hz frequency
  unsigned long print_interval = 20000; //50 Hz telemetry frequency
  if (timeTrack(interval, t0)) {
    ImuSample s = ImuRead(); //Get pitch and pitch rate data
    float dt_sec = interval/1000000.0;
    FiltData s_Filt = filterUpdate(s.accel_theta, s.theta_dot, dt_sec); //Filter data to get filtered pitch and pitch rate
    int cmd = LQR_output(s_Filt.theta, s_Filt.theta_dot); //Convert pitch and pitch rate into a motor command
    runBothMotors(motor1, motor2, cmd);
    float FiltTheta_deg = rad2deg(s_Filt.theta);
    float GyroDrift_deg = rad2deg(s_Filt.gyro_drift_theta);
    float theta_dot_deg = rad2deg(s_Filt.theta_dot);
    
    t0 = s.time;
    if (timeTrack(print_interval, t0_print)) {
      Serial.print(">Filt_theta:"); Serial.println(FiltTheta_deg); Serial.print(">GyroDrift:"); Serial.println(GyroDrift_deg); Serial.print(">MotorCommand:"); Serial.println(cmd); Serial.print(">Theta_dot:"); Serial.println(theta_dot_deg);
      t0_print = s.time;
    }
  }



}

void runMotorsOnly() {
  static bool promptPrinted = false;
  float runtime = 5; //Run time for motors in seconds
  String command;
  if (!promptPrinted) {
    Serial.println("Type one of the following commands: Motor1 <int speed>, Motor2 <int speed>, Both <int speed>");
    promptPrinted = true;
  }
 
  if (Serial.available() > 0) {
    unsigned long t0 = micros();
    command = Serial.readStringUntil('\n');
    command.trim();
    motorCommand input = parseMotorString(command);
    if (input.MotorState == "Motor1") {
      run1Motor(motor1, input.MotorSpeed);
      
      delay(1000*runtime);
      stopMotor(motor1);
      
      
    } else if (input.MotorState == "Motor2") {
      run1Motor(motor2, input.MotorSpeed);
      delay(1000*runtime);
      stopMotor(motor2);
      
    } else if (input.MotorState == "Both") {
      runBothMotors(motor1, motor2, input.MotorSpeed);
      delay(1000*runtime);
      stopBothMotors(motor1, motor2);
      

    } else {
      Serial.println("Invalid command");
    }
    promptPrinted = false;
  }

}

void runLQROnly() {
  static unsigned long t0 = micros();
  static unsigned long t0_print = micros();
  unsigned long interval = 10000; //100Hz frequency
  unsigned long print_interval = 20000; //Telemetry 50 Hz frequency
  if (timeTrack(interval, t0)) {
    ImuSample s = ImuRead();
    float dt_sec = interval/1000000.0;
    FiltData s_Filt = filterUpdate(s.accel_theta, s.theta_dot, dt_sec);
    int cmd = LQR_output(s_Filt.theta, s_Filt.theta_dot);
    float FiltTheta_deg = rad2deg(s_Filt.theta);
    float GyroDrift_deg = rad2deg(s_Filt.gyro_drift_theta);
    float theta_dot_deg = rad2deg(s_Filt.theta_dot);
    
    t0 = s.time;
    if (timeTrack(print_interval, t0_print)) {
      Serial.print(">Filt_theta:"); Serial.println(FiltTheta_deg); Serial.print(">GyroDrift:"); Serial.println(GyroDrift_deg); Serial.print(">Theta_dot:"); Serial.println(theta_dot_deg);Serial.print(">MotorCommand:"); Serial.println(cmd);
      t0_print = s.time;
    }
  }
}

void runImuOnly() {
  static unsigned long t0 = micros();
  unsigned long interval = 20000; //50 Hz telemetry frequency
  if (timeTrack(interval, t0))  {
    ImuSample s = ImuRead();
    float accel_theta_deg = rad2deg(s.accel_theta);
    float theta_dot_deg = rad2deg(s.theta_dot);
    Serial.print(">Theta:"); Serial.println(accel_theta_deg); Serial.print(">Theta_dot:"); Serial.println(theta_dot_deg);
    t0 = s.time;
  }
  
}

bool timeTrack(unsigned long interval, unsigned long t0) {
  unsigned long t1 = micros();
  unsigned long delta_t = t1-t0;
  if (delta_t>=interval) {
    return true;
  } else {
    return false;
  }
}

float rad2deg(float rad) {
  float deg = rad*180.0/PI;
  return deg;
}

motorCommand parseMotorString(String motorInput) {
  int spaceIndex = motorInput.indexOf(' ');
  motorCommand CommandOutput;
  if (spaceIndex == -1) {
    Serial.println("Invalid command");
    CommandOutput.MotorState = " ";
    CommandOutput.MotorSpeed = 0;
    return CommandOutput;
  }

  String target = motorInput.substring(0, spaceIndex);
  int speed = motorInput.substring(spaceIndex + 1).toInt();
  
  CommandOutput.MotorState = target;
  CommandOutput.MotorSpeed = speed;
  return CommandOutput;
}