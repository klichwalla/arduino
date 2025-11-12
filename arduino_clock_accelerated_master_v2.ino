#include <AccelStepper.h>

/* ===================== Cues & Nudge Settings ===================== */
/*
  CH1..CH5 use this cue table.
  CH6..CH8 are reserved for manual set:
    - CH6: enter/exit manual set & commit current 12:00
    - CH7: fine minute-hand nudge
    - CH8: fine hour-hand nudge
*/
enum CueType { CUE_NONE, CUE_TIME, CUE_HOME, CUE_RUN, CUE_TOGGLE_RUN };
struct Cue { CueType type; uint8_t hour; uint8_t minute; bool runAfter; };

Cue cues[9];  // 1..8 used

void setupCues() {
  for (int i = 0; i < 9; i++) cues[i] = {CUE_NONE,0,0,false};
  cues[1] = {CUE_HOME,       0,  0, false}; // CH1: home to midnight
  cues[2] = {CUE_TIME,      10,  0, true }; // CH2: 10:00, then run
  cues[3] = {CUE_TIME,      11, 58, false}; // CH3: 11:58, then pause
  cues[4] = {CUE_TOGGLE_RUN, 0,  0, false}; // CH4: toggle run/pause
  cues[5] = {CUE_TIME,        7,  0, false}; // CH5: 7:00, then pause
}

/* Fine nudge amounts for manual set (CH7 & CH8) */
const float NUDGE_MINUTES = 0.5f;   // each minute nudge button press = 0.5 minute
const float NUDGE_HOURS   = 0.25f;  // each hour nudge button press = 0.25 hour

/* Run-mode speed multiplier
   1.0 = real-time clock (1 hour of dial motion per real hour)
   2.0 = 2x speed  (clock runs twice as fast)
   4.0 = 4x speed  (clock runs four times as fast)

   For your NEMA 17 + TB6600 and light wooden hands, 1.0–4.0 is a sensible range.
   Higher values may cause missed steps if the mechanics are heavily loaded.
*/
const float RUN_SPEED_MULTIPLIER = 2.0f;

/* Movement strategy for cue moves
   MOVE_SHORTEST     = hands may move forward or backward; choose the shorter arc.
   MOVE_FORWARD_ONLY = hands always move forward around the dial to the target time.

   For stage visuals (“time passing forward”), use MOVE_FORWARD_ONLY.
   For shortest mechanical path, use MOVE_SHORTEST.
*/
enum MoveStrategy { MOVE_SHORTEST, MOVE_FORWARD_ONLY };
const MoveStrategy MOVE_STRATEGY = MOVE_FORWARD_ONLY;

/* ===================== Motors ===================== */
#define PIN_MIN_STEP  2
#define PIN_MIN_DIR   3
#define PIN_HR_STEP   9
#define PIN_HR_DIR   10

/* ===================== RF Channels (direct to Arduino) ===================== */
/*
  RF receiver: COM -> GND, NO -> these pins (active LOW).
*/
#define CH1_PIN 4
#define CH2_PIN 5
#define CH3_PIN 8
#define CH4_PIN 11
#define CH5_PIN 12
#define CH6_PIN 13
#define CH7_PIN A0
#define CH8_PIN A1

const int rfPins[9] = {
  -1,
  CH1_PIN, // 1
  CH2_PIN, // 2
  CH3_PIN, // 3
  CH4_PIN, // 4
  CH5_PIN, // 5
  CH6_PIN, // 6
  CH7_PIN, // 7
  CH8_PIN  // 8
};

bool lastPressed[9] = {false};

/* ===================== Motion ===================== */
AccelStepper stepMin(AccelStepper::DRIVER, PIN_MIN_STEP, PIN_MIN_DIR);
AccelStepper stepHr (AccelStepper::DRIVER, PIN_HR_STEP,  PIN_HR_DIR);

// TB6600 set for 1/4 microstep: 200 * 4 = 800
const float MICRO_REV = 800.0f;
const float RED_MIN   = 3.0f;  // 20T:60T
const float RED_HR    = 3.0f;

const long USTEPS_MIN = (long)(MICRO_REV * RED_MIN); // minute shaft µsteps/rev
const long USTEPS_HR  = (long)(MICRO_REV * RED_HR);  // hour shaft µsteps/rev

// "Zero phase" = motor step position corresponding to 12:00 on the dial.
// We calibrate these when leaving manual set mode.
long zeroPhaseMin = 0;
long zeroPhaseHr  = 0;

enum Mode { MODE_CUE, MODE_RUN };
Mode mode = MODE_CUE;
bool running        = false;
bool setMode        = false;
bool autoRunPending = false;
unsigned long lastMs = 0;
float carryMin = 0;
float carryHr  = 0;

/* ---------- Helpers ---------- */
long modWrap(long x, long m) {
  long r = x % m;
  if (r < 0) r += m;
  return r;
}

long shortestPathTarget(long cur, long targetPhase, long stepsPerRev) {
  long curPhase = modWrap(cur, stepsPerRev);
  long delta    = targetPhase - curPhase;

  if (delta >  stepsPerRev / 2) delta -= stepsPerRev;
  if (delta < -stepsPerRev / 2) delta += stepsPerRev;

  return cur + delta;
}

// Forward-only path: always move forward (positive direction) to targetPhase
long forwardOnlyTarget(long cur, long targetPhase, long stepsPerRev) {
  long curPhase = modWrap(cur, stepsPerRev);
  long delta    = targetPhase - curPhase;    // may be negative or > stepsPerRev
  delta         = modWrap(delta, stepsPerRev); // wrap into 0..stepsPerRev-1
  return cur + delta;                        // always non-negative movement
}

/* ---------- Time → Motion ---------- */
void gotoTime(uint8_t h, uint8_t m) {
  // Dial angles
  float minDeg = m * 6.0f;
  float hrDeg  = ((h % 12) + m / 60.0f) * 30.0f;

  // Convert to shaft steps relative to 12:00
  long minAngleSteps = lroundf(minDeg * (USTEPS_MIN / 360.0f));
  long hrAngleSteps  = lroundf(hrDeg  * (USTEPS_HR  / 360.0f));

  // Add zero-phase calibration and wrap into one revolution
  long minPhase = modWrap(zeroPhaseMin + minAngleSteps, USTEPS_MIN);
  long hrPhase  = modWrap(zeroPhaseHr  + hrAngleSteps,  USTEPS_HR);

  // Compute absolute targets based on movement strategy
  long absMinTarget;
  long absHrTarget;

  if (MOVE_STRATEGY == MOVE_SHORTEST) {
    absMinTarget = shortestPathTarget(stepMin.currentPosition(), minPhase, USTEPS_MIN);
    absHrTarget  = shortestPathTarget(stepHr .currentPosition(), hrPhase,  USTEPS_HR);
  } else { // MOVE_FORWARD_ONLY
    absMinTarget = forwardOnlyTarget(stepMin.currentPosition(), minPhase, USTEPS_MIN);
    absHrTarget  = forwardOnlyTarget(stepHr .currentPosition(), hrPhase,  USTEPS_HR);
  }

  stepMin.moveTo(absMinTarget);
  stepHr .moveTo(absHrTarget);
}

/* ---------- Homing (soft) ---------- */
void homeBoth() {
  stepMin.setCurrentPosition(0);
  stepHr .setCurrentPosition(0);
  zeroPhaseMin = 0;
  zeroPhaseHr  = 0;
  gotoTime(12, 0);  // logically 12:00 (until you calibrate with manual set)
}

/* ---------- Cue Handling ---------- */
void handleCue(const Cue& c) {
  switch (c.type) {
    case CUE_TIME:
      mode    = MODE_CUE;
      running = false;
      carryMin = 0;            // reset fractional accumulators when switching modes
      carryHr  = 0;
      gotoTime(c.hour, c.minute);
      autoRunPending = c.runAfter;
      break;

    case CUE_HOME:
      mode    = MODE_CUE;
      running = false;
      autoRunPending = false;
      carryMin = 0;
      carryHr  = 0;
      gotoTime(12, 0);
      break;

    case CUE_RUN:
      mode    = MODE_RUN;
      running = true;
      autoRunPending = false;
      carryMin = 0;
      carryHr  = 0;
      lastMs   = millis();     // ensure first dt in RUN is clean
      break;

    case CUE_TOGGLE_RUN:
      mode    = MODE_RUN;
      running = !running;
      autoRunPending = false;
      if (running) {
        carryMin = 0;
        carryHr  = 0;
        lastMs   = millis();   // reset timing when resuming
      }
      break;

    default:
      break;
  }
}

/* ---------- Manual Set (CH6/CH7/CH8) ---------- */

void handleSetButton(uint8_t ch) {
  if (ch == 6) {
    // Toggle manual set mode.
    // When leaving set mode, commit current hand orientation as 12:00.
    if (!setMode) {
      setMode        = true;
      mode           = MODE_CUE;
      running        = false;
      autoRunPending = false;
    } else {
      // Commit current position as "12:00"
      zeroPhaseMin = modWrap(stepMin.currentPosition(), USTEPS_MIN);
      zeroPhaseHr  = modWrap(stepHr .currentPosition(), USTEPS_HR);
      setMode      = false;
      mode         = MODE_CUE;
      running      = false;
      autoRunPending = false;
    }
    return;
  }

  if (!setMode) return;  // CH7/CH8 only active while in set mode

  // Compute nudge sizes in steps from time units
  long jogMinSteps = lroundf(NUDGE_MINUTES * 6.0f  * (USTEPS_MIN / 360.0f)); // minutes → degrees → steps
  long jogHrSteps  = lroundf(NUDGE_HOURS   * 30.0f * (USTEPS_HR  / 360.0f)); // hours   → degrees → steps

  if (ch == 7) {
    // Minute-hand fine nudge
    stepMin.move(jogMinSteps);
    while (stepMin.distanceToGo()) stepMin.run();
  } else if (ch == 8) {
    // Hour-hand fine nudge
    stepHr.move(jogHrSteps);
    while (stepHr.distanceToGo()) stepHr.run();
  }
}

/* ---------- RF Dispatch ---------- */
void readButtonsAndDispatch() {
  for (int ch = 1; ch <= 8; ch++) {
    int pin = rfPins[ch];
    if (pin < 0) continue;

    bool pressed = (digitalRead(pin) == LOW); // active LOW

    if (pressed && !lastPressed[ch]) {
      // New press
      if (ch >= 6) {
        handleSetButton(ch);
      } else if (!setMode) {
        // Ignore cue presses while in set mode
        handleCue(cues[ch]);
      }
    }

    lastPressed[ch] = pressed;
  }
}

/* ---------- Setup ---------- */
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("[SETUP] Stage clock (no RTC, µstep=1/4, variable nudges, speed multiplier, dir mode)"));

  stepMin.setMaxSpeed(2500);
  stepMin.setAcceleration(1500);
  stepHr .setMaxSpeed(2500);
  stepHr .setAcceleration(1500);

  // RF inputs
  for (int ch = 1; ch <= 8; ch++) {
    int pin = rfPins[ch];
    if (pin >= 0) {
      pinMode(pin, INPUT_PULLUP);  // RF COM->GND, NO->pin
    }
  }

  setupCues();

  // Minute motor quarter-turn test
  long q = (long)(MICRO_REV / 4.0f);
  stepMin.move(q);   while (stepMin.distanceToGo()) stepMin.run();
  delay(200);
  stepMin.move(-q);  while (stepMin.distanceToGo()) stepMin.run();

  // Soft home
  homeBoth();

  mode          = MODE_CUE;
  running       = false;
  lastMs        = millis();
  autoRunPending= false;
  setMode       = false;
}

/* ---------- Loop ---------- */
void loop() {
  // Heartbeat once per second for debug
  static unsigned long lastBeat = 0;
  unsigned long nowMs = millis();
  if (nowMs - lastBeat > 1000) {
    lastBeat = nowMs;
    Serial.print(F("[HB] mode="));
    Serial.print(mode == MODE_CUE ? F("CUE") : F("RUN"));
    Serial.print(F(" running="));
    Serial.print(running ? F("true") : F("false"));
    Serial.print(F(" setMode="));
    Serial.print(setMode ? F("true") : F("false"));
    Serial.print(F(" distMin="));
    Serial.print(stepMin.distanceToGo());
    Serial.print(F(" distHr="));
    Serial.println(stepHr.distanceToGo());
  }

  readButtonsAndDispatch();

  if (mode == MODE_CUE) {
    if (stepMin.distanceToGo() || stepHr.distanceToGo()) {
      stepMin.run();
      stepHr.run();
    } else if (autoRunPending && !setMode) {
      autoRunPending = false;
      mode    = MODE_RUN;
      running = true;
      // start RUN timing cleanly
      carryMin = 0;
      carryHr  = 0;
      lastMs   = millis();
    }
    return;
  }

  if (mode == MODE_RUN && running) {
    unsigned long now = millis();
    unsigned long dt  = now - lastMs;
    lastMs = now;

    // Minute shaft: 1 rev / 60 minutes, scaled by RUN_SPEED_MULTIPLIER
    float uMin = RUN_SPEED_MULTIPLIER * (float)USTEPS_MIN / (60.0f * 60000.0f);
    float iMin = dt * uMin + carryMin;
    long  wMin = (long)iMin;
    carryMin   = iMin - wMin;
    if (wMin) stepMin.move(wMin);

    // Hour shaft: 1 rev / 12 hours = 720 minutes, scaled by RUN_SPEED_MULTIPLIER
    float uHr = RUN_SPEED_MULTIPLIER * (float)USTEPS_HR / (720.0f * 60000.0f);
    float iHr = dt * uHr + carryHr;
    long  wHr = (long)iHr;
    carryHr   = iHr - wHr;
    if (wHr) stepHr.move(wHr);

    stepMin.run();
    stepHr.run();
  }
}
