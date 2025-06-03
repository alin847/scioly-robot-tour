#include <PID_v1.h>
#define PI 3.1415926535897932384626433832795

// ROBOT SPECS + Variables
double wheel_diameter = 0.065;  // in meters
double robot_radius = 0.09325;  // in meter
// 20.25 cm is how long the robot is
// 12 cm is how long the back to wheel is
double rotation_per_tick = 40;
unsigned long x_realtime;
unsigned long theta_realtime;
double north_speed = 0;
double south_speed = 0;
double Ks1 = 30;    // coefficent to overcome static friction 25
double Kv1 = 0;     // coefficient to curise
double Ks2 = 30;    // coefficent to overcome static friction 32
double Kv2 = 0;  // coefficient to curise

// PID SETUP
// PID for displacement
double xdisplacement, xcurrent_displacement, xspeed = 0;                                        // xdisplacement = desired position; xcurrent_displacement = current position; xspeed = control
double Kp1 = 325, Ki1 = 0.01, Kd1 = 0;                                                                                                             
PID PID_xDisplacement(&xcurrent_displacement, &xspeed, &xdisplacement, Kp1, Ki1, Kd1, DIRECT);  // setup PID
// PID for rotation
double angle, current_angle, angular_speed = 0;  // angle = desired angle; current_angle = current angle; angular_speed = control
double Kp3 = 90, Ki3 = 0.015, Kd3 = 0;
PID PID_Rotation(&current_angle, &angular_speed, &angle, Kp3, Ki3, Kd3, DIRECT);  // setup PID

// Path
// example path
// double x[] = {0.5, 1, 1.5, 2};
// double x_time[] = {0, 5000, 10000, 15000};
// 0.5 from 0 to 5000
// 1 from 5000 to 10000
// 1.5 from 10000 to 15000
// 2 from 15000 to infinity
// example path
// double x[] = { .13, 0.63, 1.13, 1.63, 2.13, 2.63, 3.13, 2.63, 3.13, 3.63, 4.13, 3.63, 4.13, 4.63, 5.13, 5.475};  // xdisplacement for PID_xDisplacement
// double x_time[] = { 0, 4000, 8000, 12000, 16000, 20000, 24000, 26000, 30000, 36000, 40000, 42000, 46000, 51000, 56000, 61000};  // time


// Rotation
// example rotation
// double theta[] = {0, PI/2, PI};
// double theta_time[] = {0, 5000, 10000};
// 0 from 0 to 5000
// pi/2 from 5000 to 10000
// pi from 10000 to infinity
// example rotation
// double theta[] = { 0, -PI / 2, 0, -PI/2, 0, PI/2, 0, PI/2, PI, PI/2, 0, PI/2, 0,-PI/2}; // theta for PID_xDisplacement
// double theta_time[] = { 0, 2000, 6000, 10000, 14000, 18000, 22000, 28000, 34000, 38000, 44000, 49000, 54000, 59000 };  // time

// MAKE PATH HERE
double target_time = 16000; // 2 sec min per move
double end_time = 18000;
char command[] = "FUFUFDDF";
const int size = sizeof(command) - 1;
int i = 0;
int k = 0;

double interval = target_time / (size - 1);
double counter = 0;
double x_time[size];
double theta_time[size];

double S = 0.34;     // distance needed to get wheel to middle from start (first step)
double F = 0.5;      // move foward
double R = -0.5;     // move reverse (most likely not gonna be used)
double U = PI / 2;   // move clockwise (up spin vector)
double D = -PI / 2;  // move counter clockwise (down spin vector)
double E = 0.41;    // distance needed to get to center of target (final step)

double x_counter = 0;
double x[size];

double theta_counter = 0;
double theta[size];


// Motor North (yellow)
int pwmNorth = 10;
int North1 = 8;
int North2 = 9;
// Motor South (green)
int pwmSouth = 3;
int South1 = 5;
int South2 = 4;

// put encoders in analog pins
// North Encoder
double north_ticks = 0;
int north_encoderA = 14;
int prev_north_encoderA;
// South Encoder
double south_ticks = 0;
int south_encoderA = 15;
int prev_south_encoderA;

void setup() {
  // put your setup code here, to run once:
  pinMode(pwmNorth, OUTPUT);
  pinMode(North1, OUTPUT);
  pinMode(North2, OUTPUT);
  pinMode(pwmSouth, OUTPUT);
  pinMode(South1, OUTPUT);
  pinMode(South2, OUTPUT);

  PID_xDisplacement.SetMode(AUTOMATIC);  // turn PID on
  PID_Rotation.SetMode(AUTOMATIC);       // turn PID on
  PID_xDisplacement.SetOutputLimits(-255, 255);
  PID_Rotation.SetOutputLimits(-255, 255);

  pinMode(north_encoderA, INPUT);
  // pinMode(north_encoderB, INPUT);
  pinMode(south_encoderA, INPUT);
  // pinMode(south_encoderB, INPUT);

  prev_north_encoderA = digitalRead(north_encoderA);
  prev_south_encoderA = digitalRead(south_encoderA);

  // setting up path
  for (int i = 0; i < size; i++) {
    x_time[i] = counter;
    theta_time[i] = counter;
    counter += interval;
  }

  for (int i = 0; i < size; i++) {
    if (command[i] == 'S') {
      x_counter += S;
      x[i] = x_counter;
    } else if (command[i] == 'F') {
      x_counter += F;
      x[i] = x_counter;
    } else if (command[i] == 'R') {
      x_counter += R;
      x[i] = x_counter;
    } else if (command[i] == 'E') {
      x_counter += E;
      x[i] = x_counter;
    } else {
      x[i] = x_counter;
    }
  }

  for (int i = 0; i < size; i++) {
    if (command[i] == 'U') {
      theta_counter += U;
      theta[i] = theta_counter;
    } else if (command[i] == 'D') {
      theta_counter += D;
      theta[i] = theta_counter;
    } else {
      theta[i] = theta_counter;
    }
  }
}


double encoder(int encoder_pin, double wheel_speed, int &prev_encoder) {
  int current_state = digitalRead(encoder_pin);
  if (current_state != prev_encoder) {
    if (wheel_speed >= 0) {
      prev_encoder = current_state;
      return 1;
    } else {
      prev_encoder = current_state;
      return -1;
    }
  } else {
    prev_encoder = current_state;
    return 0;
  }
}

int sign(int val) {
  if (val > 0) {
    return 1;
  } else if (val < 0) {
    return -1;
  } else {
    return 0;
  }
}

void motor(int pin1, int pin2, int pwm, double speed) {
  if (speed >= 0) {
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
    analogWrite(pwm, speed);
  } else {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
    analogWrite(pwm, -speed);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  // controls the value of xdisplacement given time
  // controls the value of rotation given time
  x_realtime = millis();
  theta_realtime = millis();

  if (x_realtime >= x_time[i]) {
    xdisplacement = x[i];
    if (i < sizeof(x) / sizeof(x[0]) - 1) {
      i++;
    }
  }

  if (theta_realtime >= theta_time[k]) {
    angle = theta[k];
    if (k < sizeof(theta) / sizeof(theta[0]) - 1) {
      k++;
    }
  }

  north_ticks += encoder(north_encoderA, north_speed, prev_north_encoderA);
  south_ticks += encoder(south_encoderA, south_speed, prev_south_encoderA);

  xcurrent_displacement = (((north_ticks + south_ticks) / 2) * PI * wheel_diameter) / rotation_per_tick;          
  current_angle = ((((south_ticks - north_ticks) / 2) * PI * wheel_diameter) / rotation_per_tick) / robot_radius;
  
  // computes PIDs
  PID_xDisplacement.Compute();
  PID_Rotation.Compute();
  // computes motor speeds
  north_speed = xspeed - angular_speed;
  south_speed = xspeed + angular_speed;

  // feed foward
  north_speed += Ks1 * sign(north_speed) + Kv1 * north_speed;
  south_speed += Ks2 * sign(south_speed) + Kv2 * south_speed;

  if (x_realtime >= end_time) {
    north_speed = 0;
    south_speed = 0;
  }

  // performs motor speeds
  motor(North1, North2, pwmNorth, north_speed);
  motor(South1, South2, pwmSouth, south_speed); 
}
