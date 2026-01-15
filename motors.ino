// DRV8871 IN1/IN2 mode pins
const int LM_IN1 = 5;
const int LM_IN2 = 6;

const int RM_IN1 = 9;
const int RM_IN2 = 10;

void initMotors() {
  pinMode(LM_IN1, OUTPUT);
  pinMode(LM_IN2, OUTPUT);
  pinMode(RM_IN1, OUTPUT);
  pinMode(RM_IN2, OUTPUT);

  // Stop motors at boot
  analogWrite(LM_IN1, 0); analogWrite(LM_IN2, 0);
  analogWrite(RM_IN1, 0); analogWrite(RM_IN2, 0);
}

static void setMotorOne(int in1, int in2, int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    analogWrite(in1, speed);
    analogWrite(in2, 0);
  } else if (speed < 0) {
    analogWrite(in1, 0);
    analogWrite(in2, -speed);
  } else {
    analogWrite(in1, 0);
    analogWrite(in2, 0);
  }
}

void setMotors(int leftSpeed, int rightSpeed) {
  setMotorOne(LM_IN1, LM_IN2, leftSpeed);
  setMotorOne(RM_IN1, RM_IN2, rightSpeed);
}