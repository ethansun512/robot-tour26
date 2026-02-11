

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

  int i = 0;
  while (i < cmd.length()) {

    while (i < cmd.length() && cmd[i] == ' ') i++;
    if (i >= cmd.length()) break;

    char action = cmd[i];
    i++;

    // read number after letter (supports decimals too)
    String numberStr = "";
    while (i < cmd.length() && (isDigit(cmd[i]) || cmd[i]=='.')) {
      numberStr += cmd[i];
      i++;
    }
    float value = numberStr.toFloat();

    // Execute
    switch (action) {
      case 'f': moveForwardCM(value); break;
      case 'b': moveBackwardCM(value); break;
      case 'l': turnLeftDeg(value); break;
      case 'r': turnRightDeg(value); break;
      case 'p': delay((unsigned long)value); break; // p500 means pause 500 ms
      default:
        Serial.print("Unknown command: "); Serial.println(action);
        break;
    }

    executed++;
    int remainingSteps = totalCmds - executed;

    // Smart timing wait (adjusts if something ran slow/fast)
    smartInterStepWait(remainingSteps);
  }
}