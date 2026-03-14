// ================== motors.ino ==================
const int LM_IN1 = 3;
const int LM_IN2 = 11;
const int RM_IN1 = 9;
const int RM_IN2 = 10;

const int PWM_MAX  = 255;
const int DEAD_PWM = 20;
const int RAMP_STEP = 20;

int g_kickPWM = 170;
int g_kickMs  = 70;

void initMotors() {
  pinMode(LM_IN1, OUTPUT); pinMode(LM_IN2, OUTPUT);
  pinMode(RM_IN1, OUTPUT); pinMode(RM_IN2, OUTPUT);
  analogWrite(LM_IN1, 0); analogWrite(LM_IN2, 0);
  analogWrite(RM_IN1, 0); analogWrite(RM_IN2, 0);
}

void setMotorProfile(int kickPWM, int kickMs) {
  g_kickPWM = kickPWM;
  g_kickMs  = kickMs;
}

static void setMotorOne(int in1, int in2, int speed) {
  speed = constrain(speed, -PWM_MAX, PWM_MAX);
  if (speed > 0) {
    analogWrite(in1, speed); analogWrite(in2, 0);
  } else if (speed < 0) {
    analogWrite(in1, 0); analogWrite(in2, -speed);
  } else {
    analogWrite(in1, 0); analogWrite(in2, 0);
  }
}

void setMotors(int leftSpeed, int rightSpeed) {
  setMotorOne(LM_IN1, LM_IN2, leftSpeed);
  setMotorOne(RM_IN1, RM_IN2, rightSpeed);
}

void setMotorsKick(int leftTarget, int rightTarget) {
  static bool wasStopped = true;
  bool nowStopped = (abs(leftTarget) < DEAD_PWM && abs(rightTarget) < DEAD_PWM);
  if (wasStopped && !nowStopped) {
    int lKick = 0, rKick = 0;
    if (abs(leftTarget)  >= DEAD_PWM) lKick = (leftTarget  > 0) ? g_kickPWM : -g_kickPWM;
    if (abs(rightTarget) >= DEAD_PWM) rKick = (rightTarget > 0) ? g_kickPWM : -g_kickPWM;
    setMotors(lKick, rKick);
    delay(g_kickMs);
  }
  wasStopped = nowStopped;
}

void setMotorsRamp(int leftTarget, int rightTarget) {
  static int curL = 0;
  static int curR = 0;
  leftTarget  = constrain(leftTarget,  -PWM_MAX, PWM_MAX);
  rightTarget = constrain(rightTarget, -PWM_MAX, PWM_MAX);
  if (curL < leftTarget)  curL += RAMP_STEP;
  if (curL > leftTarget)  curL -= RAMP_STEP;
  if (curR < rightTarget) curR += RAMP_STEP;
  if (curR > rightTarget) curR -= RAMP_STEP;
  if (abs(curL - leftTarget)  <= RAMP_STEP) curL = leftTarget;
  if (abs(curR - rightTarget) <= RAMP_STEP) curR = rightTarget;
  setMotors(curL, curR);
}

void setMotorsSmart(int leftTarget, int rightTarget) {
  setMotorsKick(leftTarget, rightTarget);
  setMotorsRamp(leftTarget, rightTarget);
}

void stopMotorsHard() {
  setMotors(0, 0);
}