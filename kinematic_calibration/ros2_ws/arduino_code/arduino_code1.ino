#define X_STEP 2
#define X_DIR 5

#define Y_STEP 3
#define Y_DIR 6

#define Z_STEP 4
#define Z_DIR 7

#define A_STEP 12
#define A_DIR 13

#define ENABLE_PIN 8

#define X_ENDSTOP 9
#define Y_ENDSTOP 10
#define Z_ENDSTOP 11
#define A_ENDSTOP A0

const int FAST_DELAY_US = 1000;
const int SECOND_PASS_DELAY = 2000;

// ---------- helpers ----------
bool isPressed(int pin) {
  if (digitalRead(pin) == LOW) {
    delay(3);
    return digitalRead(pin) == LOW;
  }
  return false;
}

void pulseMotor(int stepPin, int usDelay) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(usDelay);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(usDelay);
}

void pulseAll(bool x, bool y, bool z, bool a, int usDelay) {
  if (x) digitalWrite(X_STEP, HIGH);
  if (y) digitalWrite(Y_STEP, HIGH);
  if (z) digitalWrite(Z_STEP, HIGH);
  if (a) digitalWrite(A_STEP, HIGH);

  delayMicroseconds(usDelay);

  if (x) digitalWrite(X_STEP, LOW);
  if (y) digitalWrite(Y_STEP, LOW);
  if (z) digitalWrite(Z_STEP, LOW);
  if (a) digitalWrite(A_STEP, LOW);

  delayMicroseconds(usDelay);
}

bool parseMoveCommand(const String &cmd, int &s1, int &s2, int &s3, int &s4, float &speed) {
  int p1 = cmd.indexOf(',');
  int p2 = cmd.indexOf(',', p1 + 1);
  int p3 = cmd.indexOf(',', p2 + 1);
  int p4 = cmd.indexOf(',', p3 + 1);
  int p5 = cmd.indexOf(',', p4 + 1);

  if (p1 < 0 || p2 < 0 || p3 < 0 || p4 < 0) {
    return false;
  }

  s1 = cmd.substring(p1 + 1, p2).toInt();
  s2 = cmd.substring(p2 + 1, p3).toInt();
  s3 = cmd.substring(p3 + 1, p4).toInt();

  if (p5 < 0) {
    s4 = cmd.substring(p4 + 1).toInt();
    speed = 1.0;
    return true;
  }

  s4 = cmd.substring(p4 + 1, p5).toInt();
  speed = cmd.substring(p5 + 1).toFloat();
  return true;
}

bool parseHomeSmartCommand(const String &cmd, bool &yDirHigh, bool &aDirHigh) {
  int p1 = cmd.indexOf(',');
  int p2 = cmd.indexOf(',', p1 + 1);

  if (p1 < 0 || p2 < 0) {
    return false;
  }

  int yVal = cmd.substring(p1 + 1, p2).toInt();
  int aVal = cmd.substring(p2 + 1).toInt();

  yDirHigh = (yVal != 0);
  aDirHigh = (aVal != 0);
  return true;
}

// ---------- ONE-DIRECTION HOMING (FAST → BACKOFF → SLOW) ----------
void homeOneDirection(bool dirForward) {
  Serial.println("SINGLE HOMING (FAST + SLOW)");

  digitalWrite(X_DIR, dirForward ? LOW : HIGH);
  digitalWrite(Y_DIR, dirForward ? LOW : HIGH);
  digitalWrite(Z_DIR, dirForward ? LOW : HIGH);
  digitalWrite(A_DIR, dirForward ? LOW : HIGH);

  bool xHomed = false, yHomed = false, zHomed = false, aHomed = false;

  while (!(xHomed && yHomed && zHomed && aHomed)) {
    if (!xHomed && isPressed(X_ENDSTOP)) xHomed = true;
    if (!yHomed && isPressed(Y_ENDSTOP)) yHomed = true;
    if (!zHomed && isPressed(Z_ENDSTOP)) zHomed = true;
    if (!aHomed && isPressed(A_ENDSTOP)) aHomed = true;

    pulseAll(!xHomed, !yHomed, !zHomed, !aHomed, FAST_DELAY_US);
  }

  digitalWrite(X_DIR, dirForward ? HIGH : LOW);
  digitalWrite(Y_DIR, dirForward ? HIGH : LOW);
  digitalWrite(Z_DIR, dirForward ? HIGH : LOW);
  digitalWrite(A_DIR, dirForward ? HIGH : LOW);

  unsigned long startTime = millis();
  while (millis() - startTime < 500) {
    pulseAll(true, true, true, true, FAST_DELAY_US);
  }

  delay(200);

  digitalWrite(X_DIR, dirForward ? LOW : HIGH);
  digitalWrite(Y_DIR, dirForward ? LOW : HIGH);
  digitalWrite(Z_DIR, dirForward ? LOW : HIGH);
  digitalWrite(A_DIR, dirForward ? LOW : HIGH);

  xHomed = yHomed = zHomed = aHomed = false;

  while (!(xHomed && yHomed && zHomed && aHomed)) {
    if (!xHomed && isPressed(X_ENDSTOP)) xHomed = true;
    if (!yHomed && isPressed(Y_ENDSTOP)) yHomed = true;
    if (!zHomed && isPressed(Z_ENDSTOP)) zHomed = true;
    if (!aHomed && isPressed(A_ENDSTOP)) aHomed = true;

    pulseAll(!xHomed, !yHomed, !zHomed, !aHomed, SECOND_PASS_DELAY);
  }

  Serial.println("HOMING DONE");
}

void homeDual() {
  Serial.println("DUAL HOMING");

  digitalWrite(X_DIR, LOW);
  digitalWrite(Y_DIR, LOW);
  digitalWrite(Z_DIR, LOW);
  digitalWrite(A_DIR, LOW);

  bool xHomed = false, yHomed = false, zHomed = false, aHomed = false;

  while (!(xHomed && yHomed && zHomed && aHomed)) {
    if (!xHomed && isPressed(X_ENDSTOP)) { xHomed = true; }
    if (!yHomed && isPressed(Y_ENDSTOP)) { yHomed = true; }
    if (!zHomed && isPressed(Z_ENDSTOP)) { zHomed = true; }
    if (!aHomed && isPressed(A_ENDSTOP)) { aHomed = true; }

    pulseAll(!xHomed, !yHomed, !zHomed, !aHomed, FAST_DELAY_US);
  }

  digitalWrite(X_DIR, HIGH);
  digitalWrite(Y_DIR, HIGH);
  digitalWrite(Z_DIR, HIGH);
  digitalWrite(A_DIR, HIGH);

  unsigned long startTime = millis();
  while (millis() - startTime < 1500) {
    pulseAll(true, true, true, true, FAST_DELAY_US);
  }

  delay(200);

  digitalWrite(X_DIR, LOW);
  digitalWrite(Y_DIR, LOW);
  digitalWrite(Z_DIR, LOW);
  digitalWrite(A_DIR, LOW);

  xHomed = yHomed = zHomed = aHomed = false;

  while (!(xHomed && yHomed && zHomed && aHomed)) {
    if (!xHomed && isPressed(X_ENDSTOP)) { xHomed = true; }
    if (!yHomed && isPressed(Y_ENDSTOP)) { yHomed = true; }
    if (!zHomed && isPressed(Z_ENDSTOP)) { zHomed = true; }
    if (!aHomed && isPressed(A_ENDSTOP)) { aHomed = true; }

    pulseAll(!xHomed, !yHomed, !zHomed, !aHomed, SECOND_PASS_DELAY);
  }

  Serial.println("HOMING DONE");
}

void homeLower() {
  Serial.println("LOWER HOMING (Y + A)");

  // First pass: move toward endstops.
  digitalWrite(Y_DIR, HIGH);
  digitalWrite(A_DIR, HIGH);

  bool yHomed = false;
  bool aHomed = false;

  while (!(yHomed && aHomed)) {
    if (!yHomed && isPressed(Y_ENDSTOP)) yHomed = true;
    if (!aHomed && isPressed(A_ENDSTOP)) aHomed = true;

    pulseAll(false, !yHomed, false, !aHomed, FAST_DELAY_US);
  }

  // Backoff.
  digitalWrite(Y_DIR, LOW);
  digitalWrite(A_DIR, LOW);

  unsigned long startTime = millis();
  while (millis() - startTime < 200) {
    pulseAll(false, true, false, true, FAST_DELAY_US);
  }

  delay(200);

  // Slow second pass.
  digitalWrite(Y_DIR, HIGH);
  digitalWrite(A_DIR, HIGH);

  yHomed = false;
  aHomed = false;

  while (!(yHomed && aHomed)) {
    if (!yHomed && isPressed(Y_ENDSTOP)) yHomed = true;
    if (!aHomed && isPressed(A_ENDSTOP)) aHomed = true;

    pulseAll(false, !yHomed, false, !aHomed, SECOND_PASS_DELAY);
  }

  Serial.println("HOMING DONE");
}

void homeLowerSmart(bool yDirHigh, bool aDirHigh) {
  Serial.println("LOWER SMART HOMING (Y + A)");

  // First pass: move toward endstops using per-axis directions.
  digitalWrite(Y_DIR, yDirHigh ? HIGH : LOW);
  digitalWrite(A_DIR, aDirHigh ? HIGH : LOW);

  bool yHomed = false;
  bool aHomed = false;

  while (!(yHomed && aHomed)) {
    if (!yHomed && isPressed(Y_ENDSTOP)) yHomed = true;
    if (!aHomed && isPressed(A_ENDSTOP)) aHomed = true;

    pulseAll(false, !yHomed, false, !aHomed, FAST_DELAY_US);
  }

  // Backoff: reverse each axis individually.
  digitalWrite(Y_DIR, yDirHigh ? LOW : HIGH);
  digitalWrite(A_DIR, aDirHigh ? LOW : HIGH);

  unsigned long startTime = millis();
  while (millis() - startTime < 200) {
    pulseAll(false, true, false, true, FAST_DELAY_US);
  }

  delay(200);

  // Slow second pass: go toward the endstops again.
  digitalWrite(Y_DIR, yDirHigh ? HIGH : LOW);
  digitalWrite(A_DIR, aDirHigh ? HIGH : LOW);

  yHomed = false;
  aHomed = false;

  while (!(yHomed && aHomed)) {
    if (!yHomed && isPressed(Y_ENDSTOP)) yHomed = true;
    if (!aHomed && isPressed(A_ENDSTOP)) aHomed = true;

    pulseAll(false, !yHomed, false, !aHomed, SECOND_PASS_DELAY);
  }

  Serial.println("HOMING DONE");
}

// ---------- MOVE ----------
void moveSteps(int s1, int s2, int s3, int s4, float speed) {
  Serial.println("START MOVE");

  digitalWrite(X_DIR, s1 > 0 ? LOW : HIGH);
  digitalWrite(Y_DIR, s2 > 0 ? LOW : HIGH);
  digitalWrite(Z_DIR, s3 > 0 ? LOW : HIGH);
  digitalWrite(A_DIR, s4 > 0 ? LOW : HIGH);

  int delay_us = 700;  // stable value (same as before)

  int steps[4] = {abs(s1), abs(s2), abs(s3), abs(s4)};
  int maxSteps = max(max(steps[0], steps[1]), max(steps[2], steps[3]));

  if (maxSteps == 0) {
    Serial.println("PROGRESS:100");
    Serial.println("DONE");
    return;
  }

  for (int i = 0; i < maxSteps; i++) {
    bool xActive = (i < steps[0]);
    bool yActive = (i < steps[1]);
    bool zActive = (i < steps[2]);
    bool aActive = (i < steps[3]);

    if (xActive) digitalWrite(X_STEP, HIGH);
    if (yActive) digitalWrite(Y_STEP, HIGH);
    if (zActive) digitalWrite(Z_STEP, HIGH);
    if (aActive) digitalWrite(A_STEP, HIGH);

    delayMicroseconds(delay_us);

    if (xActive) digitalWrite(X_STEP, LOW);
    if (yActive) digitalWrite(Y_STEP, LOW);
    if (zActive) digitalWrite(Z_STEP, LOW);
    if (aActive) digitalWrite(A_STEP, LOW);

    delayMicroseconds(delay_us);

    if (i % (maxSteps / 100 + 1) == 0) {
      long percent = (long)i * 100L / (long)maxSteps;
      Serial.print("PROGRESS:");
      Serial.println(percent);
    }
  }

  Serial.println("PROGRESS:100");
  Serial.println("DONE");
}

void moveLowerSteps(int ySteps, int aSteps, float speed) {
  Serial.println("START LOWER MOVE");

  digitalWrite(Y_DIR, ySteps > 0 ? LOW : HIGH);
  digitalWrite(A_DIR, aSteps > 0 ? LOW : HIGH);

  int delay_us = 700;

  int stepsY = abs(ySteps);
  int stepsA = abs(aSteps);
  int maxSteps = max(stepsY, stepsA);

  if (maxSteps == 0) {
    Serial.println("PROGRESS:100");
    Serial.println("DONE");
    return;
  }

  for (int i = 0; i < maxSteps; i++) {
    bool yActive = (i < stepsY);
    bool aActive = (i < stepsA);

    if (yActive) digitalWrite(Y_STEP, HIGH);
    if (aActive) digitalWrite(A_STEP, HIGH);

    delayMicroseconds(delay_us);

    if (yActive) digitalWrite(Y_STEP, LOW);
    if (aActive) digitalWrite(A_STEP, LOW);

    delayMicroseconds(delay_us);

    if (i % (maxSteps / 100 + 1) == 0) {
      long percent = (long)i * 100L / (long)maxSteps;
      Serial.print("PROGRESS:");
      Serial.println(percent);
    }
  }

  Serial.println("PROGRESS:100");
  Serial.println("DONE");
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);

  pinMode(X_STEP, OUTPUT);
  pinMode(X_DIR, OUTPUT);
  pinMode(Y_STEP, OUTPUT);
  pinMode(Y_DIR, OUTPUT);
  pinMode(Z_STEP, OUTPUT);
  pinMode(Z_DIR, OUTPUT);
  pinMode(A_STEP, OUTPUT);
  pinMode(A_DIR, OUTPUT);

  pinMode(ENABLE_PIN, OUTPUT);

  pinMode(X_ENDSTOP, INPUT_PULLUP);
  pinMode(Y_ENDSTOP, INPUT_PULLUP);
  pinMode(Z_ENDSTOP, INPUT_PULLUP);
  pinMode(A_ENDSTOP, INPUT_PULLUP);

  Serial.print("X: "); Serial.println(digitalRead(X_ENDSTOP));
  Serial.print("Y: "); Serial.println(digitalRead(Y_ENDSTOP));
  Serial.print("Z: "); Serial.println(digitalRead(Z_ENDSTOP));
  Serial.print("A: "); Serial.println(digitalRead(A_ENDSTOP));

  digitalWrite(ENABLE_PIN, LOW);

  Serial.println("### NEW ARDUINO VERSION WITH HOME_LOWER ###");
  Serial.println("READY");
}

// ---------- LOOP ----------
String cmd = "";

void loop() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      cmd.trim();

      Serial.println("RECEIVED COMMAND >>>");
      Serial.println(cmd);

      if (cmd.startsWith("HOME_LOWER_SMART")) {
        Serial.println("CMD: HOME_LOWER_SMART");

        bool yDirHigh, aDirHigh;
        if (!parseHomeSmartCommand(cmd, yDirHigh, aDirHigh)) {
          Serial.println("ERR:HOME_SMART_PARSE");
        } else {
          homeLowerSmart(yDirHigh, aDirHigh);
        }
      }
      else if (cmd.startsWith("HOME_LOWER")) {
        Serial.println("CMD: HOME_LOWER");
        homeLower();
      }
      else if (cmd.startsWith("MOVE")) {
        int s1, s2, s3, s4;
        float speed = 1.0f;

        if (!parseMoveCommand(cmd, s1, s2, s3, s4, speed)) {
          Serial.println("ERR:MOVE_PARSE");
        } else {
          // Lower-base mode uses only Y and A; X/Z are ignored.
          moveLowerSteps(s2, s4, speed);
        }
      }

      cmd = "";
    } else {
      cmd += c;
    }
  }
}
