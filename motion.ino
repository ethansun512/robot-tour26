// ================== motion.ino ==================

static float g_totalDistError = 0.0f;
bool g_hasBottle = false;

void resetDistanceError() {
  g_totalDistError = 0.0f;
}

// ===== Encoder sign constants =====
const float LEFT_SIGN  = -1.0f;
const float RIGHT_SIGN = +1.0f;

// ===== Brake tunables =====
const int BRAKE_PWM_HARD  = 20;
const int BRAKE_MS_HARD   = 20;
const int BRAKE_PWM_HOLD  = 5;
const int BRAKE_MS_HOLD   = 60;

// ===== Motion profiles =====
struct MotionProfile {
  int   basePWM;
  int   maxPWM;
  float   leftBias;
  float kp;
  float ki;
  float kenc;
  int   turnPWM;
  int   turnPWMMid;
  int   turnPWMLow;
  int   turnPWMFinal;
  float turnExitEarly;
  int   kickPWM;
  int   kickMs;
};

const MotionProfile PROFILE_EMPTY = {
  .basePWM       = 90,
  .maxPWM        = 110,
  .leftBias      = -1.5f,
  .kp            = 0.4f,
  .ki            = 0.0f,
  .kenc          = 0.0f,
  .turnPWM       = 42,
  .turnPWMMid    = 40,
  .turnPWMLow    = 38,
  .turnPWMFinal  = 38,
  .turnExitEarly = 12.0f,
  .kickPWM       = 120,
  .kickMs        = 70,
};

const MotionProfile PROFILE_BOTTLE = {
  .basePWM       = 110,
  .maxPWM        = 120,
  .leftBias      = 1,
  .kp            = 0.5f,
  .ki            = 0.0f,
  .kenc          = 0.0f,
  .turnPWM       = 60,
  .turnPWMMid    = 60,
  .turnPWMLow    = 60,
  .turnPWMFinal  = 60,
  .turnExitEarly = 16.0f,
  .kickPWM       = 140,
  .kickMs        = 90,
};

// ===== Prototypes =====
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
  const int N = 200;
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

// ===================== MOVE FORWARD =====================
void moveForwardCM(float targetCM) {
  const MotionProfile &P = g_hasBottle ? PROFILE_BOTTLE : PROFILE_EMPTY;

  const float WHEEL_DIAMETER_CM  = 6.9f;
  const float WHEEL_CIRC_CM      = PI * WHEEL_DIAMETER_CM;
  const unsigned long TIMEOUT_MS = 15000;

  setMotorProfile(P.kickPWM, P.kickMs);

  float compensatedTarget = targetCM; //+ (g_totalDistError * 0.3f);

  float lPrev=0, rPrev=0;
  readEncoders(lPrev, rPrev);

  float lTotal=0, rTotal=0;
  float lDist=0, rDist=0;

  float headingDeg = 0.0f;
  float headingInt = 0.0f;
  unsigned long lastT = millis();
  unsigned long t0    = millis();

  while (true) {
    unsigned long nowT = millis();
    float dt = (nowT - lastT) / 1000.0f;
    if (dt <= 0) dt = 0.001f;
    lastT = nowT;

    float gyroZ = 0.0f;
    readIMU(gyroZ);
    gyroZ -= g_gyroZ_bias;
    headingDeg += gyroZ * dt;

    float lNow=0, rNow=0;
    readEncoders(lNow, rNow);

    float dL = deltaWrappedDeg(lPrev, lNow);
    float dR = deltaWrappedDeg(rPrev, rNow);
    lPrev = lNow; rPrev = rNow;

    lTotal += LEFT_SIGN  * dL;
    rTotal += RIGHT_SIGN * dR;

    lDist = (fabs(lTotal) / 360.0f) * WHEEL_CIRC_CM;
    rDist = (fabs(rTotal) / 360.0f) * WHEEL_CIRC_CM;
    float avgDist = 0.5f * (lDist + rDist);

    if (avgDist >= compensatedTarget) break;
    if (millis() - t0 > TIMEOUT_MS) { Serial.println(F("FWD TIMEOUT")); break; }

    float remaining = compensatedTarget - avgDist;
    int base = P.basePWM;
    if (remaining < 15) base = P.basePWM - 10;
    if (remaining < 8)  base = P.basePWM - 20;
    if (remaining < 3)  base = P.basePWM - 30;

    float distErr = rDist - lDist;
    int encCorr = (int)(P.kenc * distErr);
    encCorr = constrain(encCorr, -10, 10);

    float errDeg = headingDeg;
    if (fabs(errDeg) < 0.7f) errDeg = 0.0f;
    headingInt += errDeg * dt;
    headingInt = constrain(headingInt, -20.0f, 20.0f);

    int gyroCorr  = (int)(P.kp * errDeg + P.ki * headingInt);
    gyroCorr      = constrain(gyroCorr, -25, 25);
    int totalCorr = constrain(gyroCorr + encCorr, -15, 15);

    int leftPWM  = constrain(base + P.leftBias + totalCorr, -P.maxPWM, P.maxPWM);
    int rightPWM = constrain(base - totalCorr,              -P.maxPWM, P.maxPWM);

    setMotorsSmart(leftPWM, rightPWM);
    delay(10);
  }

  Serial.println(F("=== FORWARD COMPLETE ==="));
  Serial.print(F("Target: ")); Serial.print(targetCM, 2);  Serial.println(F(" cm"));
  Serial.print(F("L dist: ")); Serial.print(lDist, 2);     Serial.println(F(" cm"));
  Serial.print(F("R dist: ")); Serial.print(rDist, 2);     Serial.println(F(" cm"));
  Serial.print(F("Avg:    ")); Serial.print(0.5f*(lDist+rDist), 2); Serial.println(F(" cm"));
  Serial.println(F("========================"));

  float actualDist = 0.5f * (lDist + rDist);
  g_totalDistError += (targetCM - actualDist);

  setMotors(-BRAKE_PWM_HARD, -BRAKE_PWM_HARD);  // was -BRAKE_PWM_HARD + 12, symmetric now
  delay(BRAKE_MS_HARD);
  setMotors(-BRAKE_PWM_HOLD, -BRAKE_PWM_HOLD);  // was -BRAKE_PWM_HOLD + 5
  delay(BRAKE_MS_HOLD);
  setMotors(0, 0);
  resetRamp();
  stopMotorsHard();
}


// ===================== MOVE BACKWARD =====================
void moveBackwardCM(float targetCM) {
  const MotionProfile &P = g_hasBottle ? PROFILE_BOTTLE : PROFILE_EMPTY;

  const float WHEEL_DIAMETER_CM  = 6.9f;
  const float WHEEL_CIRC_CM      = PI * WHEEL_DIAMETER_CM;
  const unsigned long TIMEOUT_MS = 15000;

  setMotorProfile(P.kickPWM, P.kickMs);

  float compensatedTarget = targetCM * 1.1f;

  float lPrev=0, rPrev=0;
  readEncoders(lPrev, rPrev);

  float lTotal=0, rTotal=0;
  float lDist=0, rDist=0;

  float headingDeg = 0.0f;
  float headingInt = 0.0f;
  unsigned long lastT = millis();
  unsigned long t0    = millis();

  while (true) {
    unsigned long nowT = millis();
    float dt = (nowT - lastT) / 1000.0f;
    if (dt <= 0) dt = 0.001f;
    lastT = nowT;

    float gyroZ = 0.0f;
    readIMU(gyroZ);
    gyroZ -= g_gyroZ_bias;
    headingDeg += gyroZ * dt;

    float lNow=0, rNow=0;
    readEncoders(lNow, rNow);

    float dL = deltaWrappedDeg(lPrev, lNow);
    float dR = deltaWrappedDeg(rPrev, rNow);
    lPrev = lNow; rPrev = rNow;

    lTotal += LEFT_SIGN  * dL;
    rTotal += RIGHT_SIGN * dR;

    lDist = (fabs(lTotal) / 360.0f) * WHEEL_CIRC_CM;
    rDist = (fabs(rTotal) / 360.0f) * WHEEL_CIRC_CM;
    float avgDist = 0.5f * (lDist + rDist);

    if (avgDist >= compensatedTarget) break;
    if (millis() - t0 > TIMEOUT_MS) { Serial.println(F("BACK TIMEOUT")); break; }

    float remaining = compensatedTarget - avgDist;
    int base = P.basePWM;
    if (remaining < 15) base = P.basePWM - 10;
    if (remaining < 8)  base = P.basePWM - 20;
    if (remaining < 3)  base = P.basePWM - 30;

    float distErr = rDist - lDist;
    int encCorr = (int)(P.kenc * distErr);
    encCorr = constrain(-encCorr, -10, 10);

    float errDeg = headingDeg;
    if (fabs(errDeg) < 0.7f) errDeg = 0.0f;
    headingInt += errDeg * dt;
    headingInt = constrain(headingInt, -20.0f, 20.0f);

    int gyroCorr  = (int)(P.kp * errDeg + P.ki * headingInt);
    gyroCorr      = constrain(gyroCorr, -25, 25);
    int totalCorr = constrain(gyroCorr + encCorr, -15, 15);

    const int BACK_BIAS = 3;  // positive = speed up left going backward
    int leftPWM = constrain(-(base - BACK_BIAS) - totalCorr, -P.maxPWM, P.maxPWM);
    int rightPWM = constrain(-base + totalCorr,                -P.maxPWM, P.maxPWM);

    setMotorsSmart(leftPWM, rightPWM);
    delay(10);
  }

  setMotors(BRAKE_PWM_HARD, BRAKE_PWM_HARD);
  delay(BRAKE_MS_HARD);
  setMotors(BRAKE_PWM_HOLD, BRAKE_PWM_HOLD);
  delay(BRAKE_MS_HOLD);
  setMotors(0, 0);
  resetRamp();
  stopMotorsHard();
}


// ===================== TURNS =====================

static void turnDeg(float targetDeg, int direction) {
  const MotionProfile &P = g_hasBottle ? PROFILE_BOTTLE : PROFILE_EMPTY;

  setMotorProfile(P.kickPWM, P.kickMs);

  delay(200);

  float lStart=0, rStart=0;
  readEncoders(lStart, rStart);

  const unsigned long TIMEOUT_MS = 8000;

  float headingDeg = 0.0f;
  unsigned long lastT = millis();
  unsigned long t0    = millis();

  unsigned long kickStart = millis();
  while (millis() - kickStart < (unsigned long)P.kickMs) {
    unsigned long nowT = millis();
    float dt = (nowT - lastT) / 1000.0f;
    if (dt <= 0) dt = 0.001f;
    lastT = nowT;
    float gz = 0.0f;
    readIMU(gz);
    gz -= g_gyroZ_bias;
    headingDeg += gz * dt;
    setMotors(-direction * P.kickPWM, direction * P.kickPWM);
    delay(5);
  }

  // Settle at turn speed
  setMotors(-direction * P.turnPWM, direction * P.turnPWM);
  delay(50);

  // Update lastT after settle — do NOT reset headingDeg
  t0 = millis();
  lastT = millis();

  while (true) {
    unsigned long nowT = millis();
    float dt = (nowT - lastT) / 1000.0f;
    if (dt <= 0) dt = 0.001f;
    lastT = nowT;

    float gyroZ = 0.0f;
    readIMU(gyroZ);
    gyroZ -= g_gyroZ_bias;
    headingDeg += gyroZ * dt;

    if (fabs(headingDeg) >= targetDeg - P.turnExitEarly) break;
    if (millis() - t0 > TIMEOUT_MS) {
      Serial.println(direction > 0 ? F("TR TIMEOUT") : F("TL TIMEOUT"));
      break;
    }

    float remaining = targetDeg - fabs(headingDeg);
    int pwm = P.turnPWM;
    if (remaining < 20) pwm = P.turnPWMMid;
    if (remaining < 10) pwm = P.turnPWMLow;
    if (remaining < 5)  pwm = P.turnPWMFinal;

    // Direct motor control — no drift correction, no kick re-firing
    setMotors(-direction * pwm, direction * pwm);
    delay(5);
  }

  setMotors(direction * 20, -direction * 20);
  delay(60);
  setMotors(0, 0);
  delay(250);
  resetRamp();
  stopMotorsHard();
}

void turnLeftDeg(float targetDeg)  { turnDeg(targetDeg, +1); }
void turnRightDeg(float targetDeg) { turnDeg(targetDeg, -1); }

void resetRamp() {
  setMotorsSmart(0, 0);
  for (int i = 0; i < 15; i++) {
    setMotorsRamp(0, 0);
  }
}