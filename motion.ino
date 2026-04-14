// ================== motion.ino ==================

static float g_totalDistError = 0.0f;
bool g_hasBottle = false;  // used for turns only

// ===== Coordinate system =====
float g_posX      = 0.0f;  // cm, X = right
float g_posY      = 0.0f;  // cm, Y = forward
float g_facingDeg = 0.0f;  // 0 = forward, 90 = right, increases clockwise

void resetDistanceError() {
  g_totalDistError = 0.0f;
}

const float LEFT_SIGN  = -1.0f;
const float RIGHT_SIGN = +1.0f;

const int BRAKE_PWM_HARD = 20;
const int BRAKE_MS_HARD  = 20;
const int BRAKE_PWM_HOLD = 5;
const int BRAKE_MS_HOLD  = 60;

// ===== Motion profile =====
// targetVelCmS: desired speed — velocity PID adjusts PWM to hit this
//               automatically compensates for bottle weight, no profile split needed
// kp_vel/ki_vel: tune with velocity_pid_tuner.ino
// kp_head/kd_head: heading correction gains
// kp_cross: cross-track correction — how hard to correct lateral drift
//           using the coordinate system. Start at 0.3, increase if still drifting.
struct MotionProfile {
  float targetVelCmS;
  float kp_vel;
  float ki_vel;
  int   maxPWM;
  float leftBias;
  float leftBiasBack;
  float kp_head;
  float kd_head;
  float kp_cross;   // cross-track error gain (coordinate-based correction)
  int   turnPWM;
  int   turnPWMMid;
  int   turnPWMLow;
  int   turnPWMFinal;
  float turnExitEarly;
  int   kickPWM;
  int   kickMs;
};

// Single drive profile — works with or without bottle
const MotionProfile PROFILE_DRIVE = {
  20.0f,   // targetVelCmS
  4.0f,    // kp_vel
  2.0f,    // ki_vel
  200,     // maxPWM
  0.3f,    // leftBias
  3.0f,    // leftBiasBack
  0.2f,    // kp_head
  0.0f,    // kd_head
  0.3f,    // kp_cross — coordinate correction strength
  42,      // turnPWM
  40,      // turnPWMMid
  38,      // turnPWMLow
  38,      // turnPWMFinal
  12.0f,   // turnExitEarly
  120,     // kickPWM
  40       // kickMs
};

// Turns still split — need different PWM floors with/without bottle
const MotionProfile PROFILE_TURN_EMPTY = {
  20.0f, 4.0f, 2.0f, 200, 0.3f, 3.0f, 0.2f, 0.0f, 0.0f,
  42, 40, 38, 38, 12.0f, 120, 40
};

const MotionProfile PROFILE_TURN_BOTTLE = {
  15.0f, 4.0f, 2.0f, 200, 1.0f, 3.0f, 0.3f, 0.0f, 0.0f,
  60, 60, 60, 60, 16.0f, 140, 50
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

// ===== Cross-track error =====
// Computes how far the robot has drifted perpendicular to its intended path.
// startX/startY = position at start of move
// intendedDeg = intended heading at start of move
// Returns signed lateral error in cm (positive = drifted right)
static float crossTrackError(float startX, float startY,
                              float intendedDeg) {
  float intendedRad = intendedDeg * PI / 180.0f;

  // Vector from start to current position
  float dx = g_posX - startX;
  float dy = g_posY - startY;

  // Perpendicular component (cross product with intended direction)
  // intendedDir = (sin(intendedRad), cos(intendedRad))
  // cross = dx * cos - dy * sin  (positive = drifted right)
  float cross = dx * cos(intendedRad) - dy * sin(intendedRad);
  return cross;
}

// ===================== MOVE FORWARD =====================
void moveForwardCM(float targetCM) {
  const MotionProfile &P = PROFILE_DRIVE;

  const float WHEEL_DIAMETER_CM  = 6.9f;
  const float WHEEL_CIRC_CM      = PI * WHEEL_DIAMETER_CM;
  const unsigned long TIMEOUT_MS = 15000;

  setMotorProfile(P.kickPWM, P.kickMs);

  float compensatedTarget = targetCM;

  // Record start position and intended heading for cross-track correction
  float startX       = g_posX;
  float startY       = g_posY;
  float intendedDeg  = g_facingDeg;

  float lPrev=0, rPrev=0;
  readEncoders(lPrev, rPrev);

  float lTotal=0, rTotal=0;
  float lDist=0, rDist=0;

  float headingDeg  = 0.0f;
  float prevHeadErr = 0.0f;
  float velIntL     = 0.0f;
  float velIntR     = 0.0f;

  unsigned long lastT = millis();
  unsigned long t0    = millis();

  while (true) {
    unsigned long nowT = millis();
    float dt = (nowT - lastT) / 1000.0f;
    if (dt < 0.001f) dt = 0.001f;
    lastT = nowT;

    // --- Heading ---
    float gyroZ = 0.0f;
    readIMU(gyroZ);
    gyroZ -= g_gyroZ_bias;
    headingDeg += gyroZ * dt;

    // --- Encoders ---
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

    // --- Velocity measurement ---
    float velL = (fabs(LEFT_SIGN  * dL) / 360.0f * WHEEL_CIRC_CM) / dt;
    float velR = (fabs(RIGHT_SIGN * dR) / 360.0f * WHEEL_CIRC_CM) / dt;

    // --- Update coordinate position ---
    float dDist = 0.5f * (fabs(LEFT_SIGN * dL) + fabs(RIGHT_SIGN * dR))
                  / 360.0f * WHEEL_CIRC_CM;
    float facingRad = g_facingDeg * PI / 180.0f;
    g_posX += dDist * sin(facingRad);
    g_posY += dDist * cos(facingRad);

    // --- Stop condition ---
    if (avgDist >= compensatedTarget) break;
    if (millis() - t0 > TIMEOUT_MS) { Serial.println(F("FWD TIMEOUT")); break; }

    // --- Target velocity ramp down near end ---
    float remaining = compensatedTarget - avgDist;
    float targetVel = P.targetVelCmS;
    if (remaining < 10.0f) targetVel = P.targetVelCmS * 0.6f;
    if (remaining < 4.0f)  targetVel = P.targetVelCmS * 0.3f;

    // --- Heading PD correction ---
    float headErr = headingDeg;
    if (fabs(headErr) < 0.7f) headErr = 0.0f;
    float dHeadErr = (headErr - prevHeadErr) / dt;
    prevHeadErr = headErr;
    float headCorr = P.kp_head * headErr + P.kd_head * dHeadErr;

    // --- Cross-track correction (coordinate-based) ---
    // If robot drifted right (positive cross), steer left (negative correction)
    float crossErr = crossTrackError(startX, startY, intendedDeg);
    float crossCorr = P.kp_cross * crossErr;

    // Combine corrections — both act on heading
    float totalCorr = constrain(headCorr + crossCorr, -20.0f, 20.0f);

    // --- Velocity PI per motor ---
    float targetVelL = targetVel + totalCorr + P.leftBias;
    float targetVelR = targetVel - totalCorr;

    float errL = targetVelL - velL;
    float errR = targetVelR - velR;

    velIntL += errL * dt;
    velIntR += errR * dt;
    velIntL = constrain(velIntL, -30.0f, 30.0f);
    velIntR = constrain(velIntR, -30.0f, 30.0f);

    int leftPWM  = constrain((int)(P.kp_vel * errL + P.ki_vel * velIntL), 0, P.maxPWM);
    int rightPWM = constrain((int)(P.kp_vel * errR + P.ki_vel * velIntR), 0, P.maxPWM);

    setMotorsSmart(leftPWM, rightPWM);
    delay(10);
  }

  Serial.println(F("=== FORWARD COMPLETE ==="));
  Serial.print(F("Target: ")); Serial.print(targetCM, 2); Serial.println(F(" cm"));
  Serial.print(F("Avg:    ")); Serial.print(0.5f*(lDist+rDist), 2); Serial.println(F(" cm"));
  Serial.print(F("Pos X=")); Serial.print(g_posX, 1);
  Serial.print(F(" Y=")); Serial.println(g_posY, 1);
  Serial.println(F("========================"));

  float actualDist = 0.5f * (lDist + rDist);
  g_totalDistError += (targetCM - actualDist);

  setMotors(-BRAKE_PWM_HARD + 3, -BRAKE_PWM_HARD);
  delay(BRAKE_MS_HARD);
  setMotors(-BRAKE_PWM_HOLD + 1, -BRAKE_PWM_HOLD);
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

  setMotorProfile(P.kickPWM, P.kickMs);

  float compensatedTarget = targetCM * 1.1f;

  // Record start position and intended heading for cross-track correction
  float startX      = g_posX;
  float startY      = g_posY;
  float intendedDeg = g_facingDeg;

  float lPrev=0, rPrev=0;
  readEncoders(lPrev, rPrev);

  float lTotal=0, rTotal=0;
  float lDist=0, rDist=0;

  float headingDeg  = 0.0f;
  float prevHeadErr = 0.0f;
  float velIntL     = 0.0f;
  float velIntR     = 0.0f;

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

    float velL = (fabs(LEFT_SIGN  * dL) / 360.0f * WHEEL_CIRC_CM) / dt;
    float velR = (fabs(RIGHT_SIGN * dR) / 360.0f * WHEEL_CIRC_CM) / dt;

    // Update coordinate position (subtract — moving backward)
    float dDist = 0.5f * (fabs(LEFT_SIGN * dL) + fabs(RIGHT_SIGN * dR))
                  / 360.0f * WHEEL_CIRC_CM;
    float facingRad = g_facingDeg * PI / 180.0f;
    g_posX -= dDist * sin(facingRad);
    g_posY -= dDist * cos(facingRad);

    if (avgDist >= compensatedTarget) break;
    if (millis() - t0 > TIMEOUT_MS) { Serial.println(F("BACK TIMEOUT")); break; }

    float remaining = compensatedTarget - avgDist;
    float targetVel = P.targetVelCmS;
    if (remaining < 10.0f) targetVel = P.targetVelCmS * 0.6f;
    if (remaining < 4.0f)  targetVel = P.targetVelCmS * 0.3f;

    float headErr = headingDeg;
    if (fabs(headErr) < 0.7f) headErr = 0.0f;
    float dHeadErr = (headErr - prevHeadErr) / dt;
    prevHeadErr = headErr;
    float headCorr = P.kp_head * headErr + P.kd_head * dHeadErr;

    // Cross-track correction — flip sign for backward direction
    float crossErr  = crossTrackError(startX, startY, intendedDeg);
    float crossCorr = -P.kp_cross * crossErr;

    float totalCorr = constrain(headCorr + crossCorr, -20.0f, 20.0f);

    // For backward: flip corrections and use leftBiasBack
    float targetVelL = targetVel - totalCorr - P.leftBiasBack;
    float targetVelR = targetVel + totalCorr;

    float errL = targetVelL - velL;
    float errR = targetVelR - velR;

    velIntL += errL * dt;
    velIntR += errR * dt;
    velIntL = constrain(velIntL, -30.0f, 30.0f);
    velIntR = constrain(velIntR, -30.0f, 30.0f);

    int leftPWM  = constrain(-(int)(P.kp_vel * errL + P.ki_vel * velIntL), -P.maxPWM, 0);
    int rightPWM = constrain(-(int)(P.kp_vel * errR + P.ki_vel * velIntR), -P.maxPWM, 0);

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
  const MotionProfile &P = g_hasBottle ? PROFILE_TURN_BOTTLE : PROFILE_TURN_EMPTY;

  setMotorProfile(P.kickPWM, P.kickMs);
  delay(200);

  const unsigned long TIMEOUT_MS = 8000;

  float headingDeg = 0.0f;
  unsigned long lastT = millis();
  unsigned long t0    = millis();

  // Kick while tracking heading and facing
  unsigned long kickStart = millis();
  while (millis() - kickStart < (unsigned long)P.kickMs) {
    unsigned long nowT = millis();
    float dt = (nowT - lastT) / 1000.0f;
    if (dt < 0.001f) dt = 0.001f;
    lastT = nowT;

    float gz = 0.0f;
    readIMU(gz);
    gz -= g_gyroZ_bias;
    headingDeg += gz * dt;

    g_facingDeg += direction * gz * dt;
    while (g_facingDeg <   0.0f) g_facingDeg += 360.0f;
    while (g_facingDeg > 360.0f) g_facingDeg -= 360.0f;

    setMotors(-direction * P.kickPWM, direction * P.kickPWM);
    delay(5);
  }

  setMotors(-direction * P.turnPWM, direction * P.turnPWM);
  delay(50);

  t0 = millis();
  lastT = millis();

  while (true) {
    unsigned long nowT = millis();
    float dt = (nowT - lastT) / 1000.0f;
    if (dt < 0.001f) dt = 0.001f;
    lastT = nowT;

    float gyroZ = 0.0f;
    readIMU(gyroZ);
    gyroZ -= g_gyroZ_bias;
    headingDeg += gyroZ * dt;

    g_facingDeg += direction * gyroZ * dt;
    while (g_facingDeg <   0.0f) g_facingDeg += 360.0f;
    while (g_facingDeg > 360.0f) g_facingDeg -= 360.0f;

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

    setMotors(-direction * pwm, direction * pwm);
    delay(5);
  }

  setMotors(direction * 10, -direction * 10);
  delay(40);
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
