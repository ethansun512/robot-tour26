float g_targetRunSec = 0.0f;
unsigned long g_runStartMs = 0;

void startRunTimer(float targetSeconds) {
  g_targetRunSec = targetSeconds;
  g_runStartMs = millis();
}

float elapsedRunSec() {
  return (millis() - g_runStartMs) / 1000.0f;
}

bool isCmdLetter(char c) {
  return (c=='f' || c=='b' || c=='l' || c=='r' || c=='p');
}

// Count how many commands exist in the string (for timing)
int countCommands(const String &cmd) {
  int count = 0;
  for (int i = 0; i < cmd.length(); i++) {
    if (isCmdLetter(cmd[i])) count++;
  }
  return count;
}

void runProgramTimed(String cmd) {
  int totalCmds = countCommands(cmd);
  int executed = 0;
  
  float timePerCmd = g_targetRunSec / (float)totalCmds;

  int i = 0;
  while (i < cmd.length()) {

    while (i < cmd.length() && cmd[i] == ' ') i++;
    if (i >= cmd.length()) break;

    char action = cmd[i];
    i++;

    // read number after letter
    String numberStr = "";
    while (i < cmd.length() && (isDigit(cmd[i]) || cmd[i]=='.')) {
      numberStr += cmd[i];
      i++;
    }

    if (numberStr.length() == 0) {
      Serial.print("Missing number after command: ");
      Serial.println(action);
      continue;
    }

    float value = numberStr.toFloat();

    unsigned long cmdStart = millis();

    switch (action) {
      case 'f': moveForwardCM(value); break;
      case 'b': moveBackwardCM(value); break;
      case 'l': turnLeftDeg(value); break;
      case 'r': turnRightDeg(value); break;
      case 'p': delay((unsigned long)value); break; // p500 means pause 500ms
      default:
        Serial.print("Unknown command: ");
        Serial.println(action);
        break;
    }

    // Wait to match target timing
    unsigned long cmdElapsed = millis() - cmdStart;
    unsigned long targetCmdTime = (unsigned long)(timePerCmd * 1000.0f);
    if (cmdElapsed < targetCmdTime) {
      delay(targetCmdTime - cmdElapsed);
    }

    executed++;
  }
}