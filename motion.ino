// At top of motion.ino:
static float g_totalDistError = 0.0f;  // cumulative error

// ===== Encoder sign constants =====
const float LEFT_SIGN  = -1.0f;   // left encoder counts backwards
const float RIGHT_SIGN = +1.0f;   // right encoder counts forwards

// ===== Prototypes so this tab can call other tabs =====
void setMotors(int leftSpeed, int rightSpeed);
void setMotorsSmart(int leftTarget, int rightTarget);

void readEncoders(float &leftDeg, float &rightDeg);
void readIMU(float &gyroZ);

// ===== Global gyro bias =====
float g_gyroZ_bias = 0.0f;   // deg/s bias when still

void calibrateGyroZ(unsigned long ms) {
  Serial.println("Calibrating gyro Z... keep robot still");

  const int N = 200;
  float sum = 0.0f;

  for (int i = 0; i < N; i++) {
    float gz = 0.0f;
    readIMU(gz);          // should be deg/s
    sum += gz;
    delay(ms / N);
  }

  g_gyroZ_bias = sum / N;

  Serial.print("GyroZ bias = ");
  Serial.println(g_gyroZ_bias, 6);
}

static float deltaWrappedDeg(float prevDeg, float nowDeg) {
  float d = nowDeg - prevDeg;
  if (d > 180.0f)  d -= 360.0f;
  if (d < -180.0f) d += 360.0f;
  return d;
}

static void dbgEvery200ms(float lDist, float rDist, float headingDeg, int leftPWM, int rightPWM) {
  static unsigned long last = 0;
  if (millis() - last < 200) return;
  last = millis();

  Serial.print("lDist="); Serial.print(lDist, 1);
  Serial.print(" rDist="); Serial.print(rDist, 1);
  Serial.print(" head="); Serial.print(headingDeg, 2);
  Serial.print(" Lpwm="); Serial.print(leftPWM);
  Serial.print(" Rpwm="); Serial.println(rightPWM);
}

// ===================== MOVE FORWARD =====================
void moveForwardCM(float targetCM) {
  // ---- Tunables ----
  const float WHEEL_DIAMETER_CM = 6.0f;
  const float WHEEL_CIRC_CM = PI * WHEEL_DIAMETER_CM;

  const int MAX_PWM  = 120;

  const int LEFT_BIAS = 8;          // boost weak left motor (0..20)

  // Correction gains
  const float KP = 1.0f;
  const float KI = 0.4f;
  const float KENC = 30.0f;         // encoder balance gain (20..50)

  const unsigned long TIMEOUT_MS = 15000;
  // ---------------

  // Apply cumulative error compensation
  float compensatedTarget = targetCM + (g_totalDistError * 0.3f);

  // Start encoders
  float lPrev=0, rPrev=0;
  readEncoders(lPrev, rPrev);

  float lTotal=0, rTotal=0;
  float lDist=0, rDist=0;  // Declare outside loop

  // Heading integration
  float headingDeg = 0.0f;
  float headingInt = 0.0f;
  unsigned long lastT = millis();

  unsigned long t0 = millis();

  while (true) {
    // --- time step ---
    unsigned long nowT = millis();
    float dt = (nowT - lastT) / 1000.0f;
    if (dt <= 0) dt = 0.001f;
    lastT = nowT;

    // --- IMU integrate heading ---
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

    // --- stop conditions ---
    if (avgDist >= compensatedTarget) break;
    if (millis() - t0 > TIMEOUT_MS) { Serial.println("FWD TIMEOUT"); break; }

    // --- brake-zone speed ---
    float remaining = compensatedTarget - avgDist;
    int base = 45;
    if (remaining < 15) base = 40;
    if (remaining < 8)  base = 34;
    if (remaining < 3)  base = 28;

    // --- encoder balance correction ---
    float distErr = rDist - lDist;     // if right traveled more, speed up left
    int encCorr = (int)(KENC * distErr);
    encCorr = constrain(encCorr, -25, 25);

    // --- gyro PI correction ---
    float errDeg = headingDeg;
    if (fabs(errDeg) < 0.7f) errDeg = 0.0f;

    headingInt += errDeg * dt;
    headingInt = constrain(headingInt, -20.0f, 20.0f);

    int gyroCorr = (int)(KP * errDeg + KI * headingInt);
    gyroCorr = constrain(gyroCorr, -25, 25);

    int totalCorr = gyroCorr + encCorr;
    totalCorr = constrain(totalCorr, -35, 35);

    // --- apply ---
    int leftPWM  = base + LEFT_BIAS - totalCorr;
    int rightPWM = base + totalCorr;

    leftPWM  = constrain(leftPWM,  -MAX_PWM, MAX_PWM);
    rightPWM = constrain(rightPWM, -MAX_PWM, MAX_PWM);

    setMotorsSmart(leftPWM, rightPWM);
    delay(10);
  }

  // Track actual error for next move
  float actualDist = 0.5f * (lDist + rDist);
  g_totalDistError += (targetCM - actualDist);

  // Aggressive brake
  setMotors(-40, -40);
  delay(30);
  // Then active brake hold
  setMotors(-10, -10);  
  delay(100);
  setMotors(0, 0);
  stopMotorsHard();
}

// ===================== MOVE BACKWARD =====================
void moveBackwardCM(float targetCM) {
  const float WHEEL_DIAMETER_CM = 6.0f;
  const float WHEEL_CIRC_CM = PI * WHEEL_DIAMETER_CM;

  const int MAX_PWM  = 120;

  const int LEFT_BIAS = 8;

  const float KP = 1.0f;
  const float KI = 0.4f;
  const float KENC = 30.0f;

  const unsigned long TIMEOUT_MS = 15000;

  // Apply cumulative error compensation
  float compensatedTarget = targetCM + (g_totalDistError * 0.3f);

  float lPrev=0, rPrev=0;
  readEncoders(lPrev, rPrev);

  float lTotal=0, rTotal=0;
  float lDist=0, rDist=0;  // Declare outside loop

  float headingDeg = 0.0f;
  float headingInt = 0.0f;
  unsigned long lastT = millis();
  unsigned long t0 = millis();

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
    if (millis() - t0 > TIMEOUT_MS) { Serial.println("BACK TIMEOUT"); break; }

    float remaining = compensatedTarget - avgDist;
    int base = 45;
    if (remaining < 15) base = 40;
    if (remaining < 8)  base = 34;
    if (remaining < 3)  base = 28;

    float distErr = rDist - lDist;
    int encCorr = (int)(KENC * distErr);
    encCorr = constrain(encCorr, -25, 25);

    float errDeg = headingDeg;
    if (fabs(errDeg) < 0.7f) errDeg = 0.0f;

    headingInt += errDeg * dt;
    headingInt = constrain(headingInt, -20.0f, 20.0f);

    int gyroCorr = (int)(KP * errDeg + KI * headingInt);
    gyroCorr = constrain(gyroCorr, -25, 25);

    int totalCorr = gyroCorr + encCorr;
    totalCorr = constrain(totalCorr, -35, 35);

    // backward = negative speed
    int leftPWM  = -(base + LEFT_BIAS) + totalCorr;
    int rightPWM = -base - totalCorr;

    leftPWM  = constrain(leftPWM,  -MAX_PWM, MAX_PWM);
    rightPWM = constrain(rightPWM, -MAX_PWM, MAX_PWM);

    setMotorsSmart(leftPWM, rightPWM);
    delay(10);
  }

  // Track actual error for next move
  float actualDist = 0.5f * (lDist + rDist);
  g_totalDistError += (targetCM - actualDist);

  // Brake forward (opposite of backward motion)
  setMotors(40, 40);
  delay(30);
  setMotors(10, 10);  
  delay(100);
  setMotors(0, 0);
  stopMotorsHard();
}

// ===================== TURNS =====================
void turnLeftDeg(float targetDeg) {
  delay(200);  // Let robot settle from previous move
  
  float lStart=0, rStart=0;
  readEncoders(lStart, rStart);

  const unsigned long TIMEOUT_MS = 8000;

  float headingDeg = 0.0f;
  unsigned long lastT = millis();
  unsigned long t0 = millis();

  // Hold position briefly
  for(int i=0; i<50; i++) {
    setMotors(-5, -5);  // Very light brake
    delay(2);
  }
  setMotors(0, 0);
  delay(50);

  while (true) {
    unsigned long nowT = millis();
    float dt = (nowT - lastT) / 1000.0f;
    if (dt <= 0) dt = 0.001f;
    lastT = nowT;

    float gyroZ = 0.0f;
    readIMU(gyroZ);
    gyroZ -= g_gyroZ_bias;
    headingDeg += gyroZ * dt;

    if (fabs(headingDeg) >= targetDeg) break;
    if (millis() - t0 > TIMEOUT_MS) { Serial.println("TL TIMEOUT"); break; }

    float remaining = targetDeg - fabs(headingDeg);

    int pwm = 85;
    if (remaining < 30) pwm = 70;
    if (remaining < 15) pwm = 55;
    if (remaining < 6)  pwm = 40;

    // Check for linear drift
    float lNow=0, rNow=0;
    readEncoders(lNow, rNow);
    float lDrift = deltaWrappedDeg(lStart, lNow) * LEFT_SIGN;
    float rDrift = deltaWrappedDeg(rStart, rNow) * RIGHT_SIGN;
    float linearDrift = 0.5f * (lDrift + rDrift);
    
    if (fabs(linearDrift) > 3.0f) {
      int brakePWM = (int)(linearDrift * 2.0f);
      brakePWM = constrain(brakePWM, -15, 15);
      setMotorsSmart(-pwm + brakePWM, pwm + brakePWM);
    } else {
      setMotorsSmart(-pwm, pwm);
    }

    delay(5);
  }

  setMotors(25, -25);
  delay(40);
  setMotors(0, 0);
  delay(150);
  stopMotorsHard();
}

void turnRightDeg(float targetDeg) {
  delay(200);  // Let robot settle from previous move
  
  float lStart=0, rStart=0;
  readEncoders(lStart, rStart);
  
  const unsigned long TIMEOUT_MS = 8000;

  float headingDeg = 0.0f;
  unsigned long lastT = millis();
  unsigned long t0 = millis();

  // Hold position briefly
  for(int i=0; i<50; i++) {
    setMotors(-5, -5);  // Very light brake
    delay(2);
  }
  setMotors(0, 0);
  delay(50);

  while (true) {
    unsigned long nowT = millis();
    float dt = (nowT - lastT) / 1000.0f;
    if (dt <= 0) dt = 0.001f;
    lastT = nowT;

    float gyroZ = 0.0f;
    readIMU(gyroZ);
    gyroZ -= g_gyroZ_bias;
    headingDeg += gyroZ * dt;

    if (fabs(headingDeg) >= targetDeg) break;
    if (millis() - t0 > TIMEOUT_MS) { Serial.println("TR TIMEOUT"); break; }

    float remaining = targetDeg - fabs(headingDeg);

    int pwm = 85;
    if (remaining < 30) pwm = 70;
    if (remaining < 15) pwm = 55;
    if (remaining < 6)  pwm = 40;

    // Check for linear drift
    float lNow=0, rNow=0;
    readEncoders(lNow, rNow);
    float lDrift = deltaWrappedDeg(lStart, lNow) * LEFT_SIGN;
    float rDrift = deltaWrappedDeg(rStart, rNow) * RIGHT_SIGN;
    float linearDrift = 0.5f * (lDrift + rDrift);
    
    if (fabs(linearDrift) > 3.0f) {
      int brakePWM = (int)(linearDrift * 2.0f);
      brakePWM = constrain(brakePWM, -15, 15);
      setMotorsSmart(pwm + brakePWM, -pwm + brakePWM);
    } else {
      setMotorsSmart(pwm, -pwm);
    }

    delay(5);
  }

  setMotors(-25, 25);
  delay(40);
  setMotors(0, 0);
  delay(150);
  stopMotorsHard();
}