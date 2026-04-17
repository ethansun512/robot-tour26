// ================== motion.ino ==================

static float g_totalDistError = 0.0f;
bool g_hasBottle = false;

float g_posX      = 0.0f;
float g_posY      = 0.0f;
float g_facingDeg = 0.0f;

void resetDistanceError() {
  g_totalDistError = 0.0f;
}

const float LEFT_SIGN  = -1.0f;
const float RIGHT_SIGN = +1.0f;

const int BRAKE_PWM_HARD = 20;
const int BRAKE_MS_HARD  = 20;
const int BRAKE_PWM_HOLD = 5;
const int BRAKE_MS_HOLD  = 30;

struct MotionProfile {
  float targetVelCmS;
  float kp_vel;
  float ki_vel;
  int   maxPWM;
  float leftBias;
  float leftBiasBack;
  float kp_head;
  float kd_head;
  float kp_cross;
  int   turnPWM;
  int   turnPWMMid;
  int   turnPWMLow;
  int   turnPWMFinal;
  float turnExitEarly;
  int   kickPWM;
  int   kickMs;
};

// NOTE: leftBias reduced — was masking the sign issue.
// If robot still drifts right after testing, check gyro sign first.
const MotionProfile PROFILE_DRIVE = {
  20.0f, 4.0f, 2.0f, 200,
  0.0f, 3.0f,          // leftBias = 0 for now; tune AFTER sign is verified
  0.2f, 0.0f, 0.3f,
  42, 40, 38, 38, 4.0f,  // turnExitEarly dropped 12 -> 4
  100, 40
};

const MotionProfile PROFILE_TURN_EMPTY = {
  20.0f, 4.0f, 2.0f, 200, 0.0f, 3.0f, 0.2f, 0.0f, 0.0f,
  50, 42, 35, 30, 2.0f,
  100, 30
};

const MotionProfile PROFILE_TURN_BOTTLE = {
  15.0f, 4.0f, 2.0f, 200, 1.0f, 3.0f, 0.3f, 0.0f, 0.0f,
  75, 65, 55, 45, 6.0f,
  140, 50
};

void setMotors(int leftSpeed, int rightSpeed);
void setMotorsSmart(int leftTarget, int rightTarget);
void setMotorsRamp(int leftTarget, int rightTarget);
void setMotorProfile(int kickPWM, int kickMs);
void resetRamp();
void stopMotorsHard();
void readEncoders(float &leftDeg, float &rightDeg);
void readIMU(float &gyroZ);

float g_gyroZ_bias = 0.0f;

void calibrateGyroZ(unsigned long ms) {
  Serial.println(F("Calibrating gyro Z... keep robot still"));
  const int N = 500;
  float sum = 0.0f;
  for (int i = 0; i < N; i++) {
    float gz = 0.0f;
    readIMU(gz);
    sum += gz;
    delay(ms / N);
  }
  g_gyroZ_bias = sum / N;
  Serial.print(F("GyroZ bias = "));
  Serial.println(g_gyroZ_bias, 6);
}

static float deltaWrappedDeg(float prevDeg, float nowDeg) {
  float d = nowDeg - prevDeg;
  if (d >  180.0f) d -= 360.0f;
  if (d < -180.0f) d += 360.0f;
  return d;
}

static float crossTrackError(float startX, float startY, float intendedDeg) {
  float intendedRad = intendedDeg * PI / 180.0f;
  float dx = g_posX - startX;
  float dy = g_posY - startY;
  return dx * cos(intendedRad) - dy * sin(intendedRad);
}

static void normalizeFacing() {
  while (g_facingDeg <   0.0f) g_facingDeg += 360.0f;
  while (g_facingDeg > 360.0f) g_facingDeg -= 360.0f;
}

static int velocityPI(float err, float &integrator, float kp, float ki,
                      float dt, int maxPWM, int polarity) {
  int pwm;
  if (polarity > 0) {
    pwm = constrain((int)(kp * err + ki * integrator), 0, maxPWM);
    bool satHi = (pwm >= maxPWM && err > 0);
    bool satLo = (pwm <= 0      && err < 0);
    if (!satHi && !satLo) {
      integrator = constrain(integrator + err * dt, -30.0f, 30.0f);
      pwm = constrain((int)(kp * err + ki * integrator), 0, maxPWM);
    }
  } else {
    pwm = constrain(-(int)(kp * err + ki * integrator), -maxPWM, 0);
    bool satHi = (pwm <= -maxPWM && err > 0);
    bool satLo = (pwm >= 0       && err < 0);
    if (!satHi && !satLo) {
      integrator = constrain(integrator + err * dt, -30.0f, 30.0f);
      pwm = constrain(-(int)(kp * err + ki * integrator), -maxPWM, 0);
    }
  }
  return pwm;
}

// ===================== MOVE FORWARD =====================
void moveForwardCM(float targetCM) {
  const MotionProfile &P = PROFILE_DRIVE;

  const float WHEEL_DIAMETER_CM  = 6.9f;
  const float WHEEL_CIRC_CM      = PI * WHEEL_DIAMETER_CM;
  const unsigned long TIMEOUT_MS = 15000;

  const int BASE_PWM = 70;
  const float KP_HEAD = 2.5f;
  const float KD_HEAD = 0.3f;

  setMotors(P.kickPWM, P.kickPWM);
  delay(P.kickMs);

  float lPrev=0, rPrev=0;
  readEncoders(lPrev, rPrev);

  float lTotal=0, rTotal=0;
  float lDist=0, rDist=0;

  float headingDeg  = 0.0f;
  float prevHeadErr = 0.0f;

  unsigned long lastT = millis();
  unsigned long t0    = millis();

  while (true) {
    unsigned long nowT = millis();
    float dt = (nowT - lastT) / 1000.0f;
    if (dt < 0.001f) dt = 0.001f;
    lastT = nowT;

    float gyroZ = 0.0f;
    readIMU(gyroZ);
    gyroZ -= g_gyroZ_bias;
    headingDeg += gyroZ * dt;

    g_facingDeg += gyroZ * dt;
    normalizeFacing();

    float lNow=0, rNow=0;
    readEncoders(lNow, rNow);

    float dL = deltaWrappedDeg(lPrev, lNow);
    float dR = deltaWrappedDeg(rPrev, rNow);
    lPrev = lNow; rPrev = rNow;

    float dLs = LEFT_SIGN  * dL;
    float dRs = RIGHT_SIGN * dR;

    lTotal += dLs;
    rTotal += dRs;

    lDist = (fabs(lTotal) / 360.0f) * WHEEL_CIRC_CM;
    rDist = (fabs(rTotal) / 360.0f) * WHEEL_CIRC_CM;
    float avgDist = 0.5f * (lDist + rDist);

    float dDistSigned = 0.5f * (dLs + dRs) / 360.0f * WHEEL_CIRC_CM;
    float facingRad = g_facingDeg * PI / 180.0f;
    g_posX += dDistSigned * sin(facingRad);
    g_posY += dDistSigned * cos(facingRad);

    if (avgDist >= targetCM) break;
    if (millis() - t0 > TIMEOUT_MS) break;

    float remaining = targetCM - avgDist;
    int basePWM = BASE_PWM;
    if (remaining < 10.0f) basePWM = (int)(BASE_PWM * 0.7f);
    if (remaining < 4.0f)  basePWM = (int)(BASE_PWM * 0.5f);

    float headErr = headingDeg;
    float dHeadErr = (headErr - prevHeadErr) / dt;
    prevHeadErr = headErr;

    float corr = KP_HEAD * headErr + KD_HEAD * dHeadErr;
    corr = constrain(corr, -40.0f, 40.0f);

    int leftPWM  = basePWM + (int)corr;
    int rightPWM = basePWM - (int)corr;

    leftPWM  = constrain(leftPWM,  0, P.maxPWM);
    rightPWM = constrain(rightPWM, 0, P.maxPWM);

    setMotors(leftPWM, rightPWM);
    delay(10);
  }

  float actualDist = 0.5f * (lDist + rDist);
  g_totalDistError += (targetCM - actualDist);

  // Left trim: left motor brakes too hard, reduce its magnitude by 3
  setMotors(-BRAKE_PWM_HARD + 8, -BRAKE_PWM_HARD);
  delay(BRAKE_MS_HARD);
  setMotors(-BRAKE_PWM_HOLD + 3, -BRAKE_PWM_HOLD);
  delay(BRAKE_MS_HOLD);
  setMotors(0, 0);
  resetRamp();
  stopMotorsHard();
}


// ===================== MOVE BACKWARD =====================
void moveBackwardCM(float targetCM) {
  const MotionProfile &P = PROFILE_DRIVE;

  const float WHEEL_DIAMETER_CM  = 6.9f;
  const float WHEEL_CIRC_CM      = PI * WHEEL_DIAMETER_CM;
  const unsigned long TIMEOUT_MS = 15000;

  const int BASE_PWM = 70;
  const float KP_HEAD = 5.5f;
  const float KD_HEAD = 0.3f;

  setMotors(-P.kickPWM, -P.kickPWM);
  delay(P.kickMs);

  float lPrev=0, rPrev=0;
  readEncoders(lPrev, rPrev);

  float lTotal=0, rTotal=0;
  float lDist=0, rDist=0;

  float headingDeg  = 0.0f;
  float prevHeadErr = 0.0f;

  unsigned long lastT = millis();
  unsigned long t0    = millis();

  while (true) {
    unsigned long nowT = millis();
    float dt = (nowT - lastT) / 1000.0f;
    if (dt < 0.001f) dt = 0.001f;
    lastT = nowT;

    float gyroZ = 0.0f;
    readIMU(gyroZ);
    gyroZ -= g_gyroZ_bias;
    headingDeg += gyroZ * dt;

    g_facingDeg += gyroZ * dt;
    normalizeFacing();

    float lNow=0, rNow=0;
    readEncoders(lNow, rNow);

    float dL = deltaWrappedDeg(lPrev, lNow);
    float dR = deltaWrappedDeg(rPrev, rNow);
    lPrev = lNow; rPrev = rNow;

    float dLs = LEFT_SIGN  * dL;
    float dRs = RIGHT_SIGN * dR;

    lTotal += dLs;
    rTotal += dRs;

    lDist = (fabs(lTotal) / 360.0f) * WHEEL_CIRC_CM;
    rDist = (fabs(rTotal) / 360.0f) * WHEEL_CIRC_CM;
    float avgDist = 0.5f * (lDist + rDist);

    float dDistSigned = 0.5f * (dLs + dRs) / 360.0f * WHEEL_CIRC_CM;
    float facingRad = g_facingDeg * PI / 180.0f;
    g_posX += dDistSigned * sin(facingRad);
    g_posY += dDistSigned * cos(facingRad);

    if (avgDist >= targetCM) break;
    if (millis() - t0 > TIMEOUT_MS) break;

    float remaining = targetCM - avgDist;
    int basePWM = BASE_PWM;
    if (remaining < 10.0f) basePWM = (int)(BASE_PWM * 0.7f);
    if (remaining < 4.0f)  basePWM = (int)(BASE_PWM * 0.5f);

    float headErr = headingDeg;
    float dHeadErr = (headErr - prevHeadErr) / dt;
    prevHeadErr = headErr;

    float corr = KP_HEAD * headErr + KD_HEAD * dHeadErr;
    corr = constrain(corr, -40.0f, 40.0f);

    // FLIPPED SIGNS — this fixes the CW U-turn
    int leftPWM  = -basePWM + (int)corr;
    int rightPWM = -basePWM - (int)corr;

    leftPWM  = constrain(leftPWM,  -P.maxPWM, 0);
    rightPWM = constrain(rightPWM, -P.maxPWM, 0);

    setMotors(leftPWM, rightPWM);
    delay(10);
  }

  setMotors(BRAKE_PWM_HARD, BRAKE_PWM_HARD - 10);
  delay(BRAKE_MS_HARD);
  setMotors(BRAKE_PWM_HOLD, BRAKE_PWM_HOLD - 10);
  delay(BRAKE_MS_HOLD + 50);
  setMotors(0, 0);
  resetRamp();
  stopMotorsHard();
}


// ===================== TURNS =====================

static void updateHeadingTick(float &headingDeg, unsigned long &lastT) {
  unsigned long nowT = millis();
  float dt = (nowT - lastT) / 1000.0f;
  if (dt < 0.001f) dt = 0.001f;
  lastT = nowT;

  float gz = 0.0f;
  readIMU(gz);
  gz -= g_gyroZ_bias;
  headingDeg += gz * dt;

  g_facingDeg += gz * dt;
  normalizeFacing();
}

static void turnDeg(float targetDeg, int direction) {
  const MotionProfile &P = g_hasBottle ? PROFILE_TURN_BOTTLE : PROFILE_TURN_EMPTY;

  const unsigned long TIMEOUT_MS = 8000;

  float headingDeg = 0.0f;
  unsigned long lastT = millis();
  unsigned long t0;

  // Kick phase — raw setMotors, no smart wrapper
  unsigned long kickStart = millis();
  while (millis() - kickStart < (unsigned long)P.kickMs) {
    updateHeadingTick(headingDeg, lastT);
    setMotors(-direction * P.kickPWM, direction * P.kickPWM);
    delay(5);
  }

  t0 = millis();
  lastT = millis();

  while (true) {
    updateHeadingTick(headingDeg, lastT);

    if (fabs(headingDeg) >= targetDeg - P.turnExitEarly) break;
    if (millis() - t0 > TIMEOUT_MS) {
      Serial.println(direction > 0 ? F("TL TIMEOUT") : F("TR TIMEOUT"));
      break;
    }

    float remaining = targetDeg - fabs(headingDeg);
    int pwm = P.turnPWM;
    if (remaining < 30) pwm = P.turnPWMMid;
    if (remaining < 15) pwm = P.turnPWMLow;
    if (remaining < 6)  pwm = P.turnPWMFinal;

    setMotors(-direction * pwm, direction * pwm);
    delay(5);
  }

  // Real counter-brake — above DEAD_PWM so motors actually respond
  setMotors(direction * 25, -direction * 25);
  unsigned long brakeStart = millis();
  while (millis() - brakeStart < 40) {
    updateHeadingTick(headingDeg, lastT);
    delay(5);
  }
  setMotors(0, 0);

  unsigned long settleStart = millis();
  while (millis() - settleStart < 300) {
    updateHeadingTick(headingDeg, lastT);
    delay(10);
  }

  resetRamp();
  stopMotorsHard();

  Serial.print(F("Turn done. Target: "));
  Serial.print(targetDeg, 1);
  Serial.print(F("  Got: "));
  Serial.print(fabs(headingDeg), 1);
  Serial.print(F("  Error: "));
  Serial.print(targetDeg - fabs(headingDeg), 1);
  Serial.print(F("  Facing: "));
  Serial.println(g_facingDeg, 1);
}

void turnLeftDeg(float targetDeg)  { turnDeg(targetDeg, +1); }
void turnRightDeg(float targetDeg) { turnDeg(targetDeg, -1); }

void resetRamp() {
  setMotorsSmart(0, 0);
  for (int i = 0; i < 15; i++) {
    setMotorsRamp(0, 0);
  }
}