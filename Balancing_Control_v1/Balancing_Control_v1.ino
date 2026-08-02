#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Set up IMU sensor
// Create instance of the MPU6050 library
Adafruit_MPU6050 mpu;
// Accelerometer angle
float offset = 5.83*PI/180.0;
float gyroDrift = 0;
sensors_event_t a, g, temp;
float failure_angle_deg = 30; // Point at which robot is considered to be past point of rebalancing
float failure_angle_rad = failure_angle_deg*PI/180.0;
bool fail_state = false;

// Set up motors
// Motor 1 pins
const int enable1Pin = 25;
const int motor1Pin1 = 26;
const int motor1Pin2 = 27;

// Motor 2 pins
const int enable2Pin = 13;
const int motor2Pin1 = 32;
const int motor2Pin2 = 33;

// Set PWM properties
const int freq = 5000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 200;


// Set up control loop
// Complementary Filter for pitch and roll
float alpha = 0.98; 
const unsigned long time_step = 10000;
// LQR speed constant (to multiply u by because LQR loop gives a motor torque but robot can only control motor speed)
float u_sp = -1500;
int motor_input = 0;

//Declare time variable
unsigned long t0=0;
// float gyro_theta = 0;
// float accel_theta = 0;
float theta0 = 0;
// float theta_filtered = 0;

//Print time variable
unsigned long t_print = 0;
const unsigned long print_step = 50000;

//Declare Gyro struct for integration
struct gyroData {
  float delta_theta;
  unsigned long time;
  float theta_dot;
};

struct FiltData {
  float theta = 0;
  float gyro_theta = 0;
  float accel_theta = 0;
  unsigned long time = 0;
  float theta_dot = 0;
};
// Minimum duty cycle-currently a placeholder
int motor_min = 160;

gyroData gyro_data{};
FiltData current_data{};
void setup() {
  // Set up serial monitor
  Serial.begin(115200);
  delay(1000);
  
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found — check wiring");
    while (1) delay(10);          // halt instead of looping the panic
  }
  Serial.println("MPU6050 ready");
  
   
  // Configure sensor measurement ranges
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  gyroDrift = getAverageGyro();

  delay(100);
  // Set up motors
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enable2Pin, OUTPUT);
  // configure LEDC PWM
  ledcAttach(enable1Pin, freq, resolution);
  ledcAttach(enable2Pin, freq, resolution);
 
  // Time variable initiation
  // Get initial time
  t0=micros();
  t_print = micros();
  // initial theta from "previous time step" because there is no previous time step
  mpu.getEvent(&a, &g, &temp);
  theta0 = get_theta();
  fail_state = false;
  current_data.theta = theta0;
  current_data.accel_theta = theta0;
  current_data.gyro_theta = 0;
  

 

}

void loop() {
  if (!fail_state) {
    unsigned long t1 = micros();
    unsigned long delta_t = t1-t0;
    unsigned long delta_tprint = t1-t_print;
    if (delta_t > time_step) {
      current_data = get_filtered(t0, theta0);
      t0 = current_data.time;
      theta0 = current_data.theta;
      // Get motor input
      motor_input = LQR_output(current_data.theta, current_data.theta_dot);
      // Apply motor input to motors
      runMotors(motor_input);
    }
    


    if (delta_tprint > print_step) {
      float theta_filtered_deg = theta0*180.0/PI;
      float theta_gyro_deg = current_data.gyro_theta*180.0/PI;
      float theta_accel_deg = current_data.accel_theta*180.0/PI;

      Serial.print("Theta_Filt:"); Serial.print(theta_filtered_deg); Serial.print(",Theta_Gyro:"); Serial.print(theta_gyro_deg); Serial.print(",Theta_accel:"); Serial.println(theta_accel_deg);
      t_print = micros();

    }
    
   // Failsafe
    if (fabs(theta0)>failure_angle_rad) {
      motorsOff();
      Serial.println("Robot fell.");
      Serial.print("Fall angle: "); Serial.print(theta0*180.0/PI); Serial.println(" degrees");
      //fail_state=true;
    }

  }
  
}
// Function to get all the needed data values for the controls at a given time instant
// Inputs time and theta from previous time step
// Outputs theta, theta_dot, and time of current time step
FiltData get_filtered(unsigned long t_0, float theta_0) {
  FiltData current;
  mpu.getEvent(&a, &g, &temp);
  gyroData gyro_data = get_deltaTheta(t_0);      
  unsigned long t_current=gyro_data.time;
  float gyro_theta = gyro_data.delta_theta+theta_0;
  float accel_theta = get_theta();
  float theta_filtered = alpha*gyro_theta+(1-alpha)*accel_theta;
  current.time = t_current;
  current.theta = theta_filtered;
  current.theta_dot = gyro_data.theta_dot;
  current.gyro_theta = gyro_theta;
  current.accel_theta = accel_theta;
  return current;

}

// Function to be used in control loop
// Runs motors based on the input speed which will be determined by the control loop
void runMotors(int speed) {
  int speed_mag = abs(speed);
  if (speed_mag >= motor_min) {
    if (speed_mag <= 255) {
      ledcWrite(enable1Pin, speed_mag);
      ledcWrite(enable2Pin, speed_mag);
    } else {
      ledcWrite(enable1Pin, 255);
      ledcWrite(enable2Pin, 255);
    }
    if (speed > 0) {
      digitalWrite(motor1Pin1, HIGH);
      digitalWrite(motor1Pin2, LOW);
      digitalWrite(motor2Pin1, LOW);
      digitalWrite(motor2Pin2, HIGH);
    }
    else {
      digitalWrite(motor1Pin1, LOW);
      digitalWrite(motor1Pin2, HIGH);
      digitalWrite(motor2Pin1, HIGH);
      digitalWrite(motor2Pin2, LOW);
    }
  }
  else {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW);
  }
  
}

int LQR_output(float theta, float theta_dot) {
  int output = (int)round(u_sp * (1.047*theta + .32*theta_dot));
  return output;
}

void motorsOff() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
  ledcWrite(enable1Pin, 0);
  ledcWrite(enable2Pin, 0);

}

// Function to get delta theta by multiplying gyro reading by time step 
gyroData get_deltaTheta(unsigned long t0) {
  gyroData data;
  unsigned long t1 = micros();
  float delta_t =(t1-t0)/1000000.0;
  
  float gyro_y = -1.0*g.gyro.y-gyroDrift;
  float delta_theta = gyro_y*delta_t;
  data.delta_theta = delta_theta;
  data.time = t1;
  data.theta_dot = gyro_y;
  return data;
}

float get_theta() {
  
  float xa = a.acceleration.x;
  float  za = a.acceleration.z;
  float theta = atan2(za, -1.0*xa) + offset;
  return theta;
};

void getAverageReading() {
  float total_xa = 0;
  float total_ya = 0;
  float total_za = 0;
  for (int i = 0; i < 10; i++) {
  
    mpu.getEvent(&a, &g, &temp);
    total_xa = total_xa + a.acceleration.x;
    total_ya = total_ya + a.acceleration.y;
    total_za = total_za + a.acceleration.z;
    Serial.print("Accel X: "); Serial.print(a.acceleration.x);
    Serial.print(", Y: "); Serial.print(a.acceleration.y);
    Serial.print(", Z: "); Serial.print(a.acceleration.z);
    Serial.println(" m/s^2");
    delay(100);
  }
 

  float average_xa = total_xa / 10.0;
  float average_ya = total_ya / 10.0;
  float average_za = total_za / 10.0;
  float theta = atan2(average_za, -1.0*average_xa) + offset;
  float theta_deg = theta*180.0/PI;
  float theta_deg_round = round(theta_deg*100)/100.0;
  Serial.println(theta_deg_round);
  
  Serial.print("Average Accel X: "); Serial.print(average_xa);
  Serial.print(", Y: "); Serial.print(average_ya);
  Serial.print(", Z: "); Serial.print(average_za);
  Serial.println(" m/s^2");
}
float getAverageGyro() {
  float total_gy = 0;
  for (int i = 0; i < 100; i++) {
    mpu.getEvent(&a, &g, &temp);
    float gyro_y = -1.0*g.gyro.y;
    total_gy = total_gy + gyro_y;
    float gyro_y_deg = gyro_y*180.0/PI;
    
    
  }
  float average_gy = total_gy / 100.0;
  return average_gy;

};
void runMotor1() {
  ledcWrite(enable1Pin, dutyCycle);
  // put your main code here, to run repeatedly:
  // Move the DC motor forward at max speed
  Serial.println("Moving motor 1 forward");
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  delay(2000);
  // Stop the DC motor
  Serial.println("Motor 1 stopped");
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  delay(1000);
}

void runMotor2() {
  ledcWrite(enable2Pin, dutyCycle);
  // put your main code here, to run repeatedly:
  // Move the DC motor forward at max speed
  Serial.println("Moving motor 2 forward");
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
  delay(2000);
  // Stop the DC motor
  Serial.println("Motor 2 stopped");
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
  delay(1000);
}





