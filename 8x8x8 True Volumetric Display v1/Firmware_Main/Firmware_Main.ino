#include <Arduino.h>
#include <avr/interrupt.h>
#include <AltSoftSerial.h>
#include "V1FunctionConversion.h"

const byte DATA_PIN = 11;
const byte CLOCK_PIN = 13;
const byte LATCH_PIN = 12;
const byte TOUCH_PIN = 10;
const byte POT_PIN = A0;
AltSoftSerial bluetooth;

volatile byte currentCubeMode = 0;
unsigned int animationIndex = 0;
byte frameCounter = 0;
const unsigned int TOTAL_ANIMATIONS = 38;
const unsigned int BUILTIN_ANIMATIONS = 37;
const unsigned int BLUETOOTH_FUNCTION_ANIMATION = 37;
const unsigned int FRAME_TIME = 200; // user-confirmed on hardware: fine at 200ms, flicker-free
const unsigned long AUTO_MODE_CAROUSEL_TIME = 10000UL;
unsigned long lastFrameTime = 0;
unsigned long animationStart = 0;
bool bluetoothFunctionValid = false;
unsigned long lastBtByteTime = 0;
const unsigned long FUNCTION_RECEPTION_TIMEOUT_MS = 3000UL;

volatile byte globalBrightness = 5;
volatile byte displayBuffer[2][8][8];
volatile byte activeDisplayBuffer = 0;
byte drawDisplayBuffer = 1;
volatile byte brightnessAccumulator[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

void drawAnimationFrame(unsigned int animation, byte frame);
void snakePosition(byte step, byte& sx, byte& sy, byte& sz);

const unsigned long CONFIRMATION_PHASE_MS = 250UL;
const byte CONFIRMATION_NONE = 0;
const byte CONFIRMATION_RECEIVED = 1;
const byte CONFIRMATION_COMPILED = 2;
bool confirmationActive = false;
byte confirmationType = CONFIRMATION_NONE;
byte confirmationPhase = 0;
unsigned long confirmationPhaseStart = 0;
bool pendingCompiledConfirmation = false;

struct ColumnMap {
  byte reg;
  byte bit;
};
const ColumnMap COLUMN_MAP[64] PROGMEM = {
{1,0},{1,1},{1,2},{1,3},{1,4},{1,5},{1,6},{1,7},
 {2,0},{2,1},{2,2},{2,3},{2,4},{2,5},{2,6},{2,7},
 {3,0},{3,1},{3,2},{3,3},{3,4},{3,5},{3,6},{3,7},
 {4,0},{4,1},{4,2},{4,3},{4,4},{4,5},{4,6},{4,7},
 {5,0},{5,1},{5,2},{5,3},{5,4},{5,5},{5,6},{5,7},
 {6,0},{6,1},{6,2},{6,3},{6,4},{6,5},{6,6},{6,7},
 {7,0},{7,1},{7,2},{7,3},{7,4},{7,5},{7,6},{7,7},
 {8,0},{8,1},{8,2},{8,3},{8,4},{8,5},{8,6},{8,7}
 };

inline byte columnIndex(byte x, byte y) {
  return (y * 8) + x;
}

void commitFrame() {
  noInterrupts();
  byte oldDisplay = activeDisplayBuffer;
  activeDisplayBuffer = drawDisplayBuffer;
  drawDisplayBuffer = oldDisplay;
  interrupts();
}

inline void shiftByteFast(byte value) {
  for (int8_t bit = 7; bit >= 0; bit--) {
    if (value & (1 << bit)) PORTB |= _BV(PB3);
    else PORTB &= ~_BV(PB3);
    PORTB |= _BV(PB5);
    PORTB &= ~_BV(PB5);
  }
}
inline void latchFast() {
  PORTB |= _BV(PB4);
  PORTB &= ~_BV(PB4);
}

void refreshDisplay() {
  static byte layer = 0;
  
  brightnessAccumulator[layer] += globalBrightness;
  if (brightnessAccumulator[layer] >= 8) {
    brightnessAccumulator[layer] -= 8;
    
    shiftByteFast(1 << layer);
    for (int8_t r = 7; r >= 0; r--) {
      shiftByteFast(displayBuffer[activeDisplayBuffer][layer][r]);
    }
    latchFast();
  } else {
    shiftByteFast(0x00);
    for (int8_t r = 7; r >= 0; r--) {
      shiftByteFast(0x00);
    }
    latchFast();
  }
  layer = (layer + 1) % 8;
}

ISR(TIMER2_COMPA_vect) {
  refreshDisplay();
}
void startRefreshTimer() {
  noInterrupts();
  TCCR2A = _BV(WGM21);
  TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20);
  // Arduino Uno flicker free OCR2A=3
  OCR2A = 3;
  TIMSK2 |= _BV(OCIE2A);
  interrupts();
}
void stopRefreshTimer() {
  noInterrupts();
  TIMSK2 &= ~_BV(OCIE2A);
  interrupts();
}

//  1-byte ACK/NACK sent back over the HM-10 link so the phone-side UI can
// actually see whether reception/compile succeeded, instead of guessing.
// 'K' = OK, 'X' = failed.
void sendAck(bool ok) {
  bluetooth.write(ok ? 'K' : 'X');
}

void blankCubeAndStop() {
  stopRefreshTimer();
  noInterrupts();
  memset((void*)displayBuffer, 0, sizeof(displayBuffer));
  for (byte i = 0; i < 8; i++) brightnessAccumulator[i] = 0;
  interrupts();
  shiftByteFast(0x00);
  for (byte r = 0; r < 8; r++) shiftByteFast(0x00);
  latchFast();
}

// clears just the draw buffer and commits it, WITHOUT stopping the refresh timer. 
//Used to keep the cube visibly blank while a BLE-transmitted
// formula has been received but not yet compiled/confirmed with 'R'

void clearAnimationFrame() {
  for (byte z = 0; z < 8; z++) {
    for (byte r = 0; r < 8; r++) {
      displayBuffer[drawDisplayBuffer][z][r] = 0;
    }
  }
  commitFrame();
}

void setConfirmationDisplay(bool on) {
  noInterrupts();
  for (byte b = 0; b < 2; b++)
    for (byte layer = 0; layer < 8; layer++)
      for (byte r = 0; r < 8; r++) displayBuffer[b][layer][r] = on ? 0xFF : 0x00;
  interrupts();
}

void cancelConfirmation() {
  if (!confirmationActive) return;
  confirmationActive = false;
  confirmationType = CONFIRMATION_NONE;
  confirmationPhase = 0;
  confirmationPhaseStart = 0;
  pendingCompiledConfirmation = false;
  setConfirmationDisplay(false);
  stopRefreshTimer();
}

void startConfirmation(byte type) {
  confirmationActive = true;
  confirmationType = type;
  confirmationPhase = 0;
  confirmationPhaseStart = millis();
  pendingCompiledConfirmation = false;
  startRefreshTimer();
  setConfirmationDisplay(true);
}

void finishConfirmation() {
  byte finishedType = confirmationType;
  confirmationActive = false;
  confirmationType = CONFIRMATION_NONE;
  confirmationPhase = 0;
  setConfirmationDisplay(false);
  if (finishedType == CONFIRMATION_RECEIVED && pendingCompiledConfirmation) {
    pendingCompiledConfirmation = false;
    startConfirmation(CONFIRMATION_COMPILED);
    return;
  }
  if (finishedType == CONFIRMATION_RECEIVED && !pendingCompiledConfirmation) {
    // formula stored, blink done, now just waiting for 'R'. Stop the
    // refresh ISR completely (display is already blank) instead of letting it keep firing every ~256us while idle - 
    // this maximizes the interrupt bandwidth available to AltSoftSerial right when 'R' needs to arrive.
    // No change to shiftByteFast/refreshDisplay/COLUMN_MAP themselves.
    stopRefreshTimer();
  }
  if (finishedType == CONFIRMATION_COMPILED) {
    currentCubeMode = 1;
    animationIndex = BLUETOOTH_FUNCTION_ANIMATION;
    frameCounter = 0;
    animationStart = millis();
    lastFrameTime = millis();
    drawAnimationFrame(animationIndex, frameCounter);
    sendAck(true); // confirm to the phone that the compiled function is now the active animation
  }
}

void serviceConfirmation(unsigned long now) {
  if (!confirmationActive) return;
  if (now - confirmationPhaseStart < CONFIRMATION_PHASE_MS) return;
  confirmationPhaseStart = now;
  confirmationPhase++;
  if (confirmationPhase == 1) setConfirmationDisplay(false);
  else if (confirmationPhase == 2) setConfirmationDisplay(true);
  else if (confirmationPhase == 3) setConfirmationDisplay(false);
  else finishConfirmation();
}

bool firecrackerVoxel(byte f, byte x, byte y, byte z) {
  if (f < 16) {
    byte lZ = f / 2;
    if ((x == 3 || x == 4) && (y == 3 || y == 4)) {
      if (z == lZ) return true;
      if (f > 1 && z + 1 == lZ) return true;
    }
    return false;
  }
  byte bF = f - 16, d = bF / 3;
  if (d > 3) d = 3;
  if (z != 7) return false;
  int vx = (int)x - 3, vy = (int)y - 3;
  if (vx == 0 && vy == 0) return d == 0;
  if (!(vx == 0 || vy == 0 || abs(vx) == abs(vy))) return false;
  return max(abs(vx), abs(vy)) == (int)d;
}

bool snakeVoxel(byte f, byte x, byte y, byte z) {
  for (byte k = 0; k < 8; k++) {
    byte step = (byte)((f + 50 - k) % 50);
    byte sx, sy, sz;
    snakePosition(step, sx, sy, sz);
    if (x == sx && y == sy && z == sz) return true;
  }
  return false;
}

const byte SNAKE_DIRS[49] PROGMEM = { 0, 5, 1, 1, 5, 1, 2, 4, 2, 0, 0, 2, 5, 2, 5, 1, 4, 1, 1, 5, 3, 3, 0, 0, 3, 1, 3, 4, 4, 2, 4, 0, 0, 2, 4, 3, 4, 4, 2, 5, 5, 3, 3, 1, 2, 2, 0, 3, 5 };
void snakePosition(byte step, byte& sx, byte& sy, byte& sz) {
  int8_t px = 3, py = 3, pz = 3;
  for (byte s = 0; s < step; s++) {
    byte d = pgm_read_byte(&SNAKE_DIRS[s % 49]);
    if (d == 0) px++;
    else if (d == 1) px--;
    else if (d == 2) py++;
    else if (d == 3) py--;
    else if (d == 4) pz++;
    else pz--;
  }
  sx = (byte)px;
  sy = (byte)py;
  sz = (byte)pz;
}

const byte HEART_MASK[8] = { 0x66, 0xFF, 0xFF, 0x7E, 0x3C, 0x18, 0x18, 0x00 };

bool rotatingHeartVoxel(byte f, byte x, byte y, byte z) {
  if (y != 0 && y != 1) return false;
  byte r = (f / 4) % 4, u, v;
  if (r == 0) {
    u = x;
    v = z;
  } else if (r == 1) {
    u = z;
    v = 7 - x;
  } else if (r == 2) {
    u = 7 - x;
    v = 7 - z;
  } else {
    u = 7 - z;
    v = x;
  }
  return (HEART_MASK[v] & (1 << u)) != 0;
}
// ---------------------------------------------------------------------------
// Built-in animations 0-8: simple sweeping planes / diagonals / shells
// ---------------------------------------------------------------------------
bool anim00(byte f, byte x, byte y, byte z) { return z == (f % 8); }
bool anim01(byte f, byte x, byte y, byte z) { return z == (7 - (f % 8)); }
bool anim02(byte f, byte x, byte y, byte z) { return x == (f % 8); }
bool anim03(byte f, byte x, byte y, byte z) { return y == (f % 8); }
bool anim04(byte f, byte x, byte y, byte z) { return x == y && y == z && x == (f % 8); }
bool anim05(byte f, byte x, byte y, byte z) { return x == y && z == (7 - x) && x == (f % 8); }
bool anim06(byte f, byte x, byte y, byte z) { return ((x + y + z + f) & 1) == 0; }
bool anim07(byte f, byte x, byte y, byte z) {
  // true cube center for an 8-wide axis (0..7) is 3.5, not 3.
  // Scaling by 2 keeps everything in integers: |2*idx - 7| 
  // It is the distance-from-center in half-voxel units, symmetric on both sides.
  int d2 = max(abs(2 * (int)x - 7), max(abs(2 * (int)y - 7), abs(2 * (int)z - 7)));
  return d2 == (2 * (f % 4) + 1);
}
bool anim08(byte f, byte x, byte y, byte z) {
  int d2 = max(abs(2 * (int)x - 7), max(abs(2 * (int)y - 7), abs(2 * (int)z - 7)));
  return d2 == (2 * (3 - (f % 4)) + 1);
}

// ---------------------------------------------------------------------------
// Built-in animations 9-23
// ---------------------------------------------------------------------------
bool anim09(byte f, byte x, byte y, byte z) { return (x + y) == (f % 15); }                    // diagonal plane sweep (x+y)
bool anim10(byte f, byte x, byte y, byte z) { return ((int)x - (int)y + 7) == (f % 15); }      // diagonal plane sweep (x-y)
bool anim11(byte f, byte x, byte y, byte z) { return (x + y + z) == (f % 22); }                 // corner-to-corner wavefront
bool anim12(byte f, byte x, byte y, byte z) { return (x + y + z) == (21 - (f % 22)); }          // wavefront, reverse direction
bool anim13(byte f, byte x, byte y, byte z) { return z == (7 - ((f + x * 3 + y * 5) % 8)); }    // rain (staggered falling columns)
bool anim14(byte f, byte x, byte y, byte z) { return z == ((f + x * 2 + y * 7) % 8); }          // rising bubbles
bool anim15(byte f, byte x, byte y, byte z) {                                                   // alternating +/X on mid layer
  if (y != 3) return false;
  if ((f / 8) % 2 == 0) return (x == 3 || z == 3);
  return (x == z || x + z == 7);
}
bool anim16(byte f, byte x, byte y, byte z) {                                                   // expanding rings, horizontal
  if (y != 3) return false;
  int d = max(abs((int)x - 3), abs((int)z - 3));
  return d == (f % 5);
}
bool anim17(byte f, byte x, byte y, byte z) {                                                   // expanding rings, vertical
  if (x != 3) return false;
  int d = max(abs((int)y - 3), abs((int)z - 3));
  return d == (f % 5);
}
bool anim18(byte f, byte x, byte y, byte z) {                                                   // growing diamond (octahedron)
  // v6 FIX: same true-center correction as anim07/08, applied to Manhattan distance.
  int d2 = abs(2 * (int)x - 7) + abs(2 * (int)y - 7) + abs(2 * (int)z - 7); // odd, range 3..21
  return d2 == (2 * (f % 10) + 3);
}
bool anim19(byte f, byte x, byte y, byte z) {                                                   // shrinking diamond
  int d2 = abs(2 * (int)x - 7) + abs(2 * (int)y - 7) + abs(2 * (int)z - 7);
  return d2 == (2 * (9 - (f % 10)) + 3);
}
bool anim20(byte f, byte x, byte y, byte z) {                                                   // sine wave ribbon across X
  if (y != 3) return false;
  uint8_t angle = (uint8_t)((x * 32 + f * 8) & 0xFF);
  int16_t s = V1FunctionConversion::fx_sin(angle);
  int waveZ = 3 + (s * 3) / 255;
  return (int)z == waveZ;
}
bool anim21(byte f, byte x, byte y, byte z) {                                                   // sine wave ribbon across Y
  if (x != 3) return false;
  uint8_t angle = (uint8_t)((y * 32 + f * 8) & 0xFF);
  int16_t s = V1FunctionConversion::fx_sin(angle);
  int waveZ = 3 + (s * 3) / 255;
  return (int)z == waveZ;
}
bool anim22(byte f, byte x, byte y, byte z) {                                                   // bouncing diagonal comet
  byte t = f % 14;
  byte pos = (t <= 7) ? t : (14 - t);
  return x == pos && y == pos && z == pos;
}
bool anim23(byte f, byte x, byte y, byte z) {                                                   // two comets, opposite corners
  byte t = f % 8;
  return (x == t && y == t && z == t) || (x == (7 - t) && y == (7 - t) && z == (7 - t));
}

// ---------------------------------------------------------------------------
// Built-in animations 27-36
// ---------------------------------------------------------------------------
bool anim27(byte f, byte x, byte y, byte z) {                                                   // sparkle field A
  uint32_t h = (uint32_t)x * 73u + (uint32_t)y * 151u + (uint32_t)z * 211u + (uint32_t)f * 37u;
  h ^= (h >> 3);
  h *= 2654435761u;
  h ^= (h >> 15);
  return (h & 0x1F) == 0;
}
bool anim28(byte f, byte x, byte y, byte z) {                                                   // sparkle field B
  uint32_t h = (uint32_t)x * 97u + (uint32_t)y * 193u + (uint32_t)z * 251u + (uint32_t)f * 53u + 12345u;
  h ^= (h >> 5);
  h *= 2246822519u;
  h ^= (h >> 13);
  return (h & 0xF) == 5;
}
bool anim29(byte f, byte x, byte y, byte z) {                                                   // blinking hollow shell
  bool onShell = (x == 0 || x == 7 || y == 0 || y == 7 || z == 0 || z == 7);
  return onShell && (((f / 4) % 2) == 0);
}
bool anim30(byte f, byte x, byte y, byte z) {                                                   // blinking inner core
  bool inCore = (x >= 3 && x <= 4 && y >= 3 && y <= 4 && z >= 3 && z <= 4);
  return inCore && (((f / 4) % 2) == 0);
}
bool anim31(byte f, byte x, byte y, byte z) {                                                   // corner-hopping light
  byte idx = f % 8;
  byte cx = (idx & 1) ? 7 : 0, cy = (idx & 2) ? 7 : 0, cz = (idx & 4) ? 7 : 0;
  return x == cx && y == cy && z == cz;
}
bool anim32(byte f, byte x, byte y, byte z) {                                                   // sweep along X-axis edges
  return ((y == 0 || y == 7) && (z == 0 || z == 7)) && x == (f % 8);
}
bool anim33(byte f, byte x, byte y, byte z) {                                                   // sweep along Y-axis edges
  return ((x == 0 || x == 7) && (z == 0 || z == 7)) && y == (f % 8);
}
bool anim34(byte f, byte x, byte y, byte z) {                                                   // sweep along Z-axis edges
  return ((x == 0 || x == 7) && (y == 0 || y == 7)) && z == (f % 8);
}
bool anim35(byte f, byte x, byte y, byte z) { return (x + z) == (f % 15); }                     // diagonal wall sweep (x+z)
bool anim36(byte f, byte x, byte y, byte z) { return ((int)x - (int)z + 7) == (f % 15); }       // diagonal wall sweep (x-z)

// ---------------------------------------------------------------------------
// Dispatch table: one authoritative list of every built-in animation.
// Indices 24-26 are the pre-existing named functions defined above.
// If a slot is ever left null, animationVoxel() treats it as blank on
// purpose rather than silently - that gap would now be obvious to find here.
// ---------------------------------------------------------------------------
typedef bool (*AnimFn)(byte f, byte x, byte y, byte z);

const AnimFn ANIMATION_TABLE[BUILTIN_ANIMATIONS] = {
  anim00, anim01, anim02, anim03, anim04, anim05, anim06, anim07, anim08,  //  0- 8
  anim09, anim10, anim11, anim12, anim13, anim14, anim15, anim16, anim17,  //  9-17
  anim18, anim19, anim20, anim21, anim22, anim23,                          // 18-23
  firecrackerVoxel, snakeVoxel, rotatingHeartVoxel,                        // 24-26
  anim27, anim28, anim29, anim30, anim31, anim32, anim33, anim34, anim35, anim36  // 27-36
};

bool animationVoxel(byte a, byte f, byte x, byte y, byte z) {
  if (a == BLUETOOTH_FUNCTION_ANIMATION) {
    return V1FunctionConversion::evaluate(x, y, z, f);
  }
  if (a >= BUILTIN_ANIMATIONS) return false;
  return ANIMATION_TABLE[a](f, x, y, z);
}

void drawAnimationFrame(unsigned int animation, byte frame) {
  if (animation >= TOTAL_ANIMATIONS) return;
  for (byte z = 0; z < 8; z++) {
    for (byte y = 0; y < 8; y++) {
      for (byte x = 0; x < 8; x++) {
        bool state = animationVoxel(animation, frame, x, y, z);
        byte column = columnIndex(x, y);
        byte reg = pgm_read_byte(&(COLUMN_MAP[column].reg));
        byte bit = pgm_read_byte(&(COLUMN_MAP[column].bit));
        if (reg >= 1 && reg <= 8 && bit <= 7) {
          if (state) displayBuffer[drawDisplayBuffer][z][reg - 1] |= (1 << bit);
          else       displayBuffer[drawDisplayBuffer][z][reg - 1] &= ~(1 << bit);
        }
      }
    }
  }
  commitFrame();
}

void setMode(byte targetMode, unsigned int targetAnimation) {
  blankCubeAndStop();
  currentCubeMode = targetMode;

  if (targetMode == 0) {
    animationIndex = targetAnimation % BUILTIN_ANIMATIONS;
  } else {
    animationIndex = targetAnimation;
  }

  frameCounter = 0;
  animationStart = millis();
  lastFrameTime = animationStart;
  drawAnimationFrame(animationIndex, frameCounter);
  startRefreshTimer();
}

void setup() {
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(TOUCH_PIN, INPUT);
  PORTB &= ~(_BV(PB3) | _BV(PB4) | _BV(PB5));
  bluetooth.begin(9600);
  startRefreshTimer();
  animationStart = millis();
  lastFrameTime = millis();
}

void loop() {
  unsigned long now = millis();
  int rawPot = analogRead(POT_PIN);
  noInterrupts();
  globalBrightness = map(rawPot, 0, 1023, 2, 8);
  interrupts();
  static bool lastTouchState = false;
  static unsigned long touchDebounceTimer = 0;
  static bool hasTriggeredLongPress = false;
  bool currentTouchState = (digitalRead(TOUCH_PIN) == HIGH);
  if (currentTouchState && !lastTouchState) {
    touchDebounceTimer = now;
    hasTriggeredLongPress = false;
  } else if (currentTouchState && lastTouchState) {
    unsigned long touchDuration = now - touchDebounceTimer;
    if (!hasTriggeredLongPress && touchDuration >= 3000UL) {
      byte targetMode = (currentCubeMode == 0) ? 1 : 0;
      cancelConfirmation();
      // Use a safe animation index when switching modes
      setMode(targetMode, (targetMode == 0) ? (animationIndex % BUILTIN_ANIMATIONS) : animationIndex);
      hasTriggeredLongPress = true;
    }
  } else if (!currentTouchState && lastTouchState) {
    unsigned long touchDuration = now - touchDebounceTimer;
    if (!hasTriggeredLongPress && currentCubeMode == 1 && touchDuration >= 50 && touchDuration < 3000UL) {
      byte nextAnimation = (animationIndex + 1);
      // If we were on BT animation (37), wrap back to 0. If we were on built-in, stay in built-in range.
      if (nextAnimation >= TOTAL_ANIMATIONS) nextAnimation = 0;
      if (nextAnimation == BLUETOOTH_FUNCTION_ANIMATION) nextAnimation = 0; // Skip BT when manually cycling
      
      cancelConfirmation();
      setMode(1, nextAnimation);
    }
  }
  lastTouchState = currentTouchState;
  while (bluetooth.available() > 0) {
    char inChar = (char)bluetooth.read();
    lastBtByteTime = now;
    if (inChar == '@') {
      cancelConfirmation();
      blankCubeAndStop();
      currentCubeMode = 1;
      animationIndex = BLUETOOTH_FUNCTION_ANIMATION;
      frameCounter = 0;
      animationStart = now;
      lastFrameTime = now;
      V1FunctionConversion::startReception();
    } else if (V1FunctionConversion::isFunctionStarted()) {
      if (inChar == 'E') {
        V1FunctionConversion::stopReception();
        if (V1FunctionConversion::isFunctionComplete()) {
          startConfirmation(CONFIRMATION_RECEIVED);
        } else {
          // reception failed (empty or corrupted) - nothing will blink to signal that,
          //  so tell the phone directly instead of going silent.
          sendAck(false);
        }
      } else {
        V1FunctionConversion::receiveCharacter(inChar);
      }
    } else if (inChar == 'A') {
      cancelConfirmation();
      setMode(0, animationIndex % BUILTIN_ANIMATIONS);
    } else if (inChar == 'M') {
      cancelConfirmation();
      setMode(1, animationIndex % BUILTIN_ANIMATIONS);
    } else if (inChar == 'N' && currentCubeMode == 1) {
      cancelConfirmation();
      byte nextAnimation = (animationIndex + 1);
      if (nextAnimation >= TOTAL_ANIMATIONS) nextAnimation = 0;
      if (nextAnimation == BLUETOOTH_FUNCTION_ANIMATION) nextAnimation = 0;
      setMode(1, nextAnimation);
    } else if (inChar == 'C') {
      cancelConfirmation();
      bluetoothFunctionValid = false;
      blankCubeAndStop();
      currentCubeMode = 1;
      animationIndex = BLUETOOTH_FUNCTION_ANIMATION;
      V1FunctionConversion::cancelReception();
    } else if (inChar == 'R') {
      if (confirmationActive && confirmationType == CONFIRMATION_RECEIVED) {
        if (V1FunctionConversion::isFunctionComplete() && V1FunctionConversion::compileFunction()) {
          bluetoothFunctionValid = true;
          pendingCompiledConfirmation = true; // ack sent later, once the COMPILED blink finishes
        } else {
          bluetoothFunctionValid = false;
          sendAck(false); // no further blink is coming for this path, so ack now
        }
      } else if (!confirmationActive) {
        // previously called blankCubeAndStop() here unconditionally,
        // which disables the refresh-timer interrupt. If compileFunction()
        // then failed, nothing ever turned the timer back on and the cube
        // stayed permanently dark with zero feedback. Now we only touch the
        // display (never the timer) up front, and always leave the timer
        // running either way.
        if (V1FunctionConversion::isFunctionComplete() && V1FunctionConversion::compileFunction()) {
          bluetoothFunctionValid = true;
          startConfirmation(CONFIRMATION_COMPILED); // this starts/keeps the timer running; ack sent when it finishes
        } else {
          bluetoothFunctionValid = false;
          clearAnimationFrame(); // stay blank, but WITHOUT stopping the refresh timer here...
          stopRefreshTimer();    // ...then stop it explicitly, so a retry of 'R' also gets max serial bandwidth
          sendAck(false);
        }
      }
    }
  }
  now = millis();
  if (V1FunctionConversion::isFunctionStarted() && (now - lastBtByteTime > FUNCTION_RECEPTION_TIMEOUT_MS)) {
    V1FunctionConversion::cancelReception();
  }
  serviceConfirmation(now);
  if (confirmationActive) return;
  if (currentCubeMode == 0) {
    if (now - animationStart >= AUTO_MODE_CAROUSEL_TIME) {
      animationIndex = (animationIndex + 1) % BUILTIN_ANIMATIONS;
      frameCounter = 0;
      animationStart = now;
      lastFrameTime = now;
    }
    if (now - lastFrameTime >= FRAME_TIME) {
      lastFrameTime = now;
      drawAnimationFrame(animationIndex, frameCounter);
      frameCounter = (frameCounter + 1) % 50;
    }
  } else if (currentCubeMode == 1 && animationIndex == BLUETOOTH_FUNCTION_ANIMATION) {
    //  previously this fell back to drawAnimationFrame(0, ...)
    // whenever a formula had been received but not yet compiled/confirmed
    // with 'R'. That made the cube look like it had spontaneously started
    // playing a built-in animation. Now it just stays blank until a valid
    // compiled function exists.
    if (now - lastFrameTime >= FRAME_TIME) {
      lastFrameTime = now;
      if (bluetoothFunctionValid) {
        drawAnimationFrame(BLUETOOTH_FUNCTION_ANIMATION, frameCounter);
      } else {
        clearAnimationFrame();
      }
      frameCounter = (frameCounter + 1) % 50;
    }
  } else if (currentCubeMode == 1 && animationIndex < BUILTIN_ANIMATIONS) {
    if (now - lastFrameTime >= FRAME_TIME) {
      lastFrameTime = now;
      drawAnimationFrame(animationIndex, frameCounter);
      frameCounter = (frameCounter + 1) % 50;
    }
  }
}
