float g_gyroZ_bias = 0.0f;   // deg/s bias when sitting still

void calibrateGyroZ(unsigned long ms = 2000) {
  Serial.println("Calibrating gyro Z... keep robot still");

  const int N = 200;
  float sum = 0.0f;

  for (int i = 0; i < N; i++) {
    float gz = 0.0f;
    readIMU(gz);          // gz should be deg/s
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

void moveForwardCM(float targetCM) {
  // ---- TUNE THESE ----
  const float WHEEL_DIAMETER_CM = 6.0f;
  const float WHEEL_CIRC_CM = 3.14159f * WHEEL_DIAMETER_CM;

  const int BASE_PWM = 85;         // slower base speed
  const int MAX_PWM  = 120;        // cap

  const float LEFT_SIGN  = -1.0f;  // you said left counts backwards
  const float RIGHT_SIGN = +1.0f;

  // Heading-hold gain: start small
  // correctionPWM = KP * headingErrorDeg
  const float KP_HEADING = 2.0f;   // try 1.0 to 4.0 range
  // --------------------

  // Encoder start (wrapped angles)
  float lPrev = 0, rPrev = 0;
  readEncoders(lPrev, rPrev);

  float lTotal = 0, rTotal = 0;

  // Heading estimate (degrees)
  float headingDeg = 0.0f;   // we want to keep this near 0
  unsigned long lastT = millis();

  unsigned long t0 = millis();
  const unsigned long TIMEOUT_MS = 15000;

  while (true) {
    unsigned long nowT = millis();
    float dt = (nowT - lastT) / 1000.0f;  // seconds
    if (dt <= 0) dt = 0.001f;
    lastT = nowT;

    // ----- IMU: integrate gyroZ to get heading change -----
    float gyroZ = 0.0f;     // deg/s (expected)
    readIMU(gyroZ);
    gyroZ -= g_gyroZ_bias;  // remove bias
    headingDeg += gyroZ * dt;

    // ----- Encoders: accumulate distance -----
    float lNow = 0, rNow = 0;
    readEncoders(lNow, rNow);

    float dL = deltaWrappedDeg(lPrev, lNow);
    float dR = deltaWrappedDeg(rPrev, rNow);
    lPrev = lNow;
    rPrev = rNow;

    lTotal += LEFT_SIGN  * dL;
    rTotal += RIGHT_SIGN * dR;

    float lDist = (fabs(lTotal) / 360.0f) * WHEEL_CIRC_CM;
    float rDist = (fabs(rTotal) / 360.0f) * WHEEL_CIRC_CM;
    float avgDist = 0.5f * (lDist + rDist);

    // Stop conditions
    if (avgDist >= targetCM) break;
    if (millis() - t0 > TIMEOUT_MS) { Serial.println("TIMEOUT"); break; }

    // ----- Heading hold control -----
    // If headingDeg is positive, you rotated one direction; correct by differential PWM
    int correction = (int)(KP_HEADING * headingDeg);

    int leftPWM  = BASE_PWM - correction;
    int rightPWM = BASE_PWM + correction;

    leftPWM  = constrain(leftPWM,  -MAX_PWM, MAX_PWM);
    rightPWM = constrain(rightPWM, -MAX_PWM, MAX_PWM);

    setMotors(leftPWM, rightPWM);

    // Debug (optional)
    // Serial.print("dist="); Serial.print(avgDist,1);
    // Serial.print(" heading="); Serial.println(headingDeg,2);

    delay(10);
  }

  setMotors(0, 0);
}

void moveBackwardCM(float targetCM) {

  const float WHEEL_DIAMETER_CM = 6.0f;
  const float WHEEL_CIRC_CM = 3.14159f * WHEEL_DIAMETER_CM;

  const int BASE_PWM = 80;        // slower backward
  const int MAX_PWM  = 120;

  const float LEFT_SIGN  = -1.0f;   // keep same as forward
  const float RIGHT_SIGN = +1.0f;

  const float KP_HEADING = 2.0f;    // heading correction gain
  const unsigned long TIMEOUT_MS = 15000;

  // --- Initial encoder reading ---
  float lPrev = 0, rPrev = 0;
  readEncoders(lPrev, rPrev);

  float lTotal = 0;
  float rTotal = 0;

  float headingDeg = 0.0f;

  unsigned long lastT = millis();
  unsigned long t0 = millis();

  while (true) {

    unsigned long nowT = millis();
    float dt = (nowT - lastT) / 1000.0f;
    if (dt <= 0) dt = 0.001f;
    lastT = nowT;

    // --- Gyro heading integration ---
    float gyroZ = 0.0f;
    readIMU(gyroZ);
    gyroZ -= g_gyroZ_bias;
    headingDeg += gyroZ * dt;

    // --- Encoder unwrap accumulation ---
    float lNow = 0, rNow = 0;
    readEncoders(lNow, rNow);

    float dL = lNow - lPrev;
    float dR = rNow - rPrev;

    if (dL > 180.0f)  dL -= 360.0f;
    if (dL < -180.0f) dL += 360.0f;
    if (dR > 180.0f)  dR -= 360.0f;
    if (dR < -180.0f) dR += 360.0f;

    lPrev = lNow;
    rPrev = rNow;

    lTotal += LEFT_SIGN  * dL;
    rTotal += RIGHT_SIGN * dR;

    float lDist = (fabs(lTotal) / 360.0f) * WHEEL_CIRC_CM;
    float rDist = (fabs(rTotal) / 360.0f) * WHEEL_CIRC_CM;
    float avgDist = 0.5f * (lDist + rDist);

    if (avgDist >= targetCM) break;
    if (millis() - t0 > TIMEOUT_MS) break;

    // --- Heading correction ---
    int correction = (int)(KP_HEADING * headingDeg);

    int leftPWM  = -BASE_PWM - correction;
    int rightPWM = -BASE_PWM + correction;

    leftPWM  = constrain(leftPWM,  -MAX_PWM, MAX_PWM);
    rightPWM = constrain(rightPWM, -MAX_PWM, MAX_PWM);

    setMotors(leftPWM, rightPWM);

    delay(10);
  }

  setMotors(0, 0);
  delay(200);
}

void turnLeftDeg(float targetDeg) {
  const int TURN_PWM = 85;     // start slow
  const float KP = 2.0f;       // heading correction (optional fine control)
  const unsigned long TIMEOUT_MS = 8000;

  float headingDeg = 0.0f;

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

    if (fabs(headingDeg) >= targetDeg) break;
    if (millis() - t0 > TIMEOUT_MS) break;

    // Turn left in place:
    // left backward, right forward
    setMotors(-TURN_PWM, TURN_PWM);

    delay(5);
  }

  setMotors(0, 0);
  delay(200);   // small settle time
}

void turnRightDeg(float targetDeg) {
  const int TURN_PWM = 85;
  const unsigned long TIMEOUT_MS = 8000;

  float headingDeg = 0.0f;

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

    if (fabs(headingDeg) >= targetDeg) break;
    if (millis() - t0 > TIMEOUT_MS) break;

    // Turn right in place:
    // left forward, right backward
    setMotors(TURN_PWM, -TURN_PWM);

    delay(5);
  }

  setMotors(0, 0);
  delay(200);
}