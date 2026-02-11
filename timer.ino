float g_targetRunSec = 0.0f;
unsigned long g_runStartMs = 0;

void startRunTimer(float targetSeconds) {
  g_targetRunSec = targetSeconds;
  g_runStartMs = millis();
}

float elapsedRunSec() {
  return (millis() - g_runStartMs) / 1000.0f;
}

// Wait a little between steps so total time approaches target.
// remainingSteps = how many commands are still left AFTER the current one.
void smartInterStepWait(int remainingSteps) {
  if (g_targetRunSec <= 0) return;                 // disabled
  if (remainingSteps <= 0) return;                 // last step, don't wait

  float elapsed = elapsedRunSec();
  float remainingTime = g_targetRunSec - elapsed;

  if (remainingTime <= 0) return;                  // behind schedule: don't wait

  // Spread remaining time evenly over remaining steps
  float waitSec = remainingTime / (float)remainingSteps;

  // Clamp so we never "freeze" too long if something weird happens
  if (waitSec > 1.0f) waitSec = 1.0f;              // max 1 second per step
  if (waitSec < 0.0f) waitSec = 0.0f;

  unsigned long waitMs = (unsigned long)(waitSec * 1000.0f);

  if (waitMs > 0) delay(waitMs);
}