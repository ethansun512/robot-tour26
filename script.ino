// ================== program.ino ==================

float g_targetRunSec = 0.0f;
unsigned long g_runStartMs = 0;

void startRunTimer(float targetSeconds) {
  g_targetRunSec = targetSeconds;
  g_runStartMs   = millis();
}

float elapsedRunSec() {
  return (millis() - g_runStartMs) / 1000.0f;
}

bool isCmdLetter(char c) {
  return (c == 'f' || c == 'b' || c == 'l' || c == 'r' || c == 'p');
}

int countCommands(const char* cmd) {
  int count = 0;
  for (int i = 0; cmd[i] != '\0'; i++) {
    if (isCmdLetter(cmd[i])) count++;
  }
  return count;
}

void runProgramTimed(const char* cmd) {
  int totalCmds = countCommands(cmd);
  if (totalCmds == 0) {
    Serial.println(F("runProgramTimed: no commands found."));
    return;
  }

  float timePerCmd = g_targetRunSec / (float)totalCmds;
  int executed = 0;
  int i = 0;

  while (cmd[i] != '\0') {

    // Skip whitespace
    while (cmd[i] == ' ') i++;
    if (cmd[i] == '\0') break;

    char action = cmd[i];
    i++;

    // Read optional bottle modifier
    if (cmd[i] == 'b' || cmd[i] == 'n') {
      g_hasBottle = (cmd[i] == 'b');
      i++;
      Serial.print(g_hasBottle ? F("[BOTTLE] ") : F("[EMPTY]  "));
    }

    // Read number into buffer
    char numberBuf[8];
    int j = 0;
    while ((isDigit(cmd[i]) || cmd[i] == '.') && j < 7) {
      numberBuf[j++] = cmd[i++];
    }
    numberBuf[j] = '\0';

    if (j == 0) {
      Serial.print(F("Missing number after: "));
      Serial.println(action);
      continue;
    }

    float value = atof(numberBuf);

    // Encoder health check
    if (action != 'p' && !rightEncoderHealthy()) {
      Serial.println(F("ABORT: Right encoder failed."));
      setMotors(0, 0);
      stopMotorsHard();
      return;
    }

    unsigned long cmdStart = millis();

    switch (action) {
      case 'f':
        Serial.print(F("CMD fwd ")); Serial.print(value); Serial.println(F(" cm"));
        moveForwardCM(value);
        break;
      case 'b':
        Serial.print(F("CMD back ")); Serial.print(value); Serial.println(F(" cm"));
        moveBackwardCM(value);
        break;
      case 'l':
        Serial.print(F("CMD left ")); Serial.print(value); Serial.println(F(" deg"));
        turnLeftDeg(value);
        break;
      case 'r':
        Serial.print(F("CMD right ")); Serial.print(value); Serial.println(F(" deg"));
        turnRightDeg(value);
        break;
      case 'p':
        Serial.print(F("CMD pause ")); Serial.print(value); Serial.println(F(" ms"));
        delay((unsigned long)value);
        break;
      default:
        Serial.print(F("Unknown cmd: "));
        Serial.println(action);
        break;
    }

    executed++;


    // Only pad timing between commands, not after the last one
    if (executed < totalCmds) {
      unsigned long cmdElapsed  = millis() - cmdStart;
      unsigned long targetCmdMs = (unsigned long)(timePerCmd * 1000.0f);
      if (cmdElapsed < targetCmdMs) {
        delay(targetCmdMs - cmdElapsed);
      } else {
        Serial.print(F("WARNING: overran by "));
        Serial.print(cmdElapsed - targetCmdMs);
        Serial.println(F(" ms"));
      }
    }
  }
  Serial.print(F("Done. "));
  Serial.print(executed);
  Serial.print(F("/"));
  Serial.print(totalCmds);
  Serial.println(F(" cmds executed."));
}