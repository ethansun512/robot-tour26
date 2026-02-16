// ================== motors.ino ==================
// DRV8871 IN1/IN2 mode pins (PWM-capable on Uno)
const int LM_IN1 = 3;
const int LM_IN2 = 11;

const int RM_IN1 = 9;
const int RM_IN2 = 10;

// --------- Tunables ---------
const int PWM_MAX = 255;

// Kick settings (to overcome stiction)
const int KICK_PWM = 160;      // 140–180 typical
const int KICK_MS  = 70;       // 50–120 ms
const int DEAD_PWM = 10;       // treat below this as "stopped"

// Ramp settings (smooth changes without blocking)
const int RAMP_STEP = 4;       // PWM step per call (2–8 typical)
// ----------------------------

void initMotors() {
  pinMode(LM_IN1, OUTPUT);
  pinMode(LM_IN2, OUTPUT);
  pinMode(RM_IN1, OUTPUT);
  pinMode(RM_IN2, OUTPUT);

  // Stop motors at boot
  analogWrite(LM_IN1, 0); analogWrite(LM_IN2, 0);
  analogWrite(RM_IN1, 0); analogWrite(RM_IN2, 0);
}

// Drive ONE motor with signed speed (-255..255)
static void setMotorOne(int in1, int in2, int speed) {
  speed = constrain(speed, -PWM_MAX, PWM_MAX);

  if (speed > 0) {
    analogWrite(in1, speed);
    analogWrite(in2, 0);
  } else if (speed < 0) {
    analogWrite(in1, 0);
    analogWrite(in2, -speed);
  } else {
    // coast stop (both PWM 0)
    analogWrite(in1, 0);
    analogWrite(in2, 0);
  }
}

// Basic direct motor command
void setMotors(int leftSpeed, int rightSpeed) {
  setMotorOne(LM_IN1, LM_IN2, leftSpeed);
  setMotorOne(RM_IN1, RM_IN2, rightSpeed);
}

// One-time kick when transitioning from stop -> moving
void setMotorsKick(int leftTarget, int rightTarget) {
  static bool wasStopped = true;

  bool nowStopped = (abs(leftTarget) < DEAD_PWM && abs(rightTarget) < DEAD_PWM);

  // Kick only once when we START moving
  if (wasStopped && !nowStopped) {
    int lKick = 0, rKick = 0;

    if (abs(leftTarget) >= DEAD_PWM) {
      lKick = (leftTarget > 0) ? KICK_PWM : -KICK_PWM;
    }
    if (abs(rightTarget) >= DEAD_PWM) {
      rKick = (rightTarget > 0) ? KICK_PWM : -KICK_PWM;
    }

    setMotors(lKick, rKick);
    delay(KICK_MS);
  }

  wasStopped = nowStopped;
}

// Smooth ramp toward target (NO delays, safe in fast loop)
void setMotorsRamp(int leftTarget, int rightTarget) {
  static int curL = 0;
  static int curR = 0;

  leftTarget  = constrain(leftTarget,  -PWM_MAX, PWM_MAX);
  rightTarget = constrain(rightTarget, -PWM_MAX, PWM_MAX);

  // Ramp L
  if (curL < leftTarget) curL += RAMP_STEP;
  if (curL > leftTarget) curL -= RAMP_STEP;

  // Ramp R
  if (curR < rightTarget) curR += RAMP_STEP;
  if (curR > rightTarget) curR -= RAMP_STEP;

  // Snap when close
  if (abs(curL - leftTarget) <= RAMP_STEP) curL = leftTarget;
  if (abs(curR - rightTarget) <= RAMP_STEP) curR = rightTarget;

  setMotors(curL, curR);
}

// RECOMMENDED: call this inside your motion loops
// - kicks once at start
// - ramps smoothly every loop
void setMotorsSmart(int leftTarget, int rightTarget) {
  setMotorsKick(leftTarget, rightTarget);
  setMotorsRamp(leftTarget, rightTarget);
}

void stopMotorsHard() {
  setMotors(0, 0);
  // Force reset by calling setMotorsRamp with 0,0 several times
  for(int i=0; i<10; i++) {
    setMotorsRamp(0, 0);
  }
}