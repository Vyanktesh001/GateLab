/*
  GateLab trainer firmware - Arduino Uno
  ---------------------------------------
  Talks to logic_trainer.py over USB at 115200 8-N-1.

  Board -> PC  (on every switch change + every 200 ms):
      {"type":"inputs","A":1,"B":0,"C":1,"D":0}
  PC -> Board:
      CMD:<L0>,<L1>,<L2>,<L3>:<16-char LCD line 1>:<16-char LCD line 2>

  Wiring (matches the CONFIG block below):
      DIP switch 1..4 (A,B,C,D) -> D2, D3, D4, D5   other side of each switch -> GND
      LED0..LED3                -> D6, D7, D8, D9   LED long leg (+) to the pin,
                                                    short leg (-) -> 220-330 ohm -> GND
      16x2 I2C LCD (optional)   -> SDA=A4, SCL=A5, VCC=5V, GND=GND  (0x27 or 0x3F, auto-detected)

  Behaviour:
      * At power-up the LEDs light one after another (wiring self-test).
      * With no PC talking to it, the LEDs and LCD simply mirror the switches
        (switch ON -> its LED ON), so the board can be checked on its own.
      * As soon as GateLab sends commands, the PC drives the LEDs and LCD.
        If the PC goes quiet for 6 s the board falls back to mirroring.
      * The LCD is refreshed a few characters per loop, so the board never stops
        reading the switches or the serial port while the (slow) LCD updates.
      * A 2 s watchdog restarts the board automatically if it ever freezes.

  IMPORTANT: close GateLab before uploading - only one program can use the COM port.
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <avr/wdt.h>

// ============================== CONFIG ==============================
const uint8_t SWITCH_PINS[4] = {2, 3, 4, 5};   // A, B, C, D  (DIP switch 1..4)
// true  : switch connects the pin to GND (internal pull-up used) -> switch ON reads as 1
// false : switch connects the pin to 5V with an external pull-down -> switch ON reads as 1
const bool SWITCH_ACTIVE_LOW = true;

const uint8_t LED_PINS[4] = {6, 7, 8, 9};      // LED0 .. LED3
// true  : LED + to the pin, LED - through the resistor to GND (pin HIGH = LED on)
// false : LED + to 5V, LED - through the resistor to the pin   (pin LOW  = LED on)
const bool LED_ACTIVE_HIGH = true;

#define USE_LCD 1                              // 0 if no LCD is fitted
// ====================================================================

const unsigned long DEBOUNCE_MS = 15;
const unsigned long HEARTBEAT_MS = 200;
const unsigned long PC_TIMEOUT_MS = 6000;
const uint8_t LCD_CHARS_PER_LOOP = 2;          // keeps every loop() pass short (~4 ms max)

#if USE_LCD
LiquidCrystal_I2C lcd27(0x27, 16, 2);
LiquidCrystal_I2C lcd3F(0x3F, 16, 2);
LiquidCrystal_I2C *lcd = nullptr;
char lcdShown[2][16];                          // what is on the glass now
char lcdWanted[2][16];                         // what it should show
#endif

uint8_t stableState[4];
uint8_t lastRaw[4];
unsigned long rawChangedAt[4];
unsigned long lastSendMs = 0;
unsigned long lastCmdMs = 0;
bool pcActive = false;

char rxBuf[64];
uint8_t rxLen = 0;

uint8_t readSwitch(uint8_t i) {
  uint8_t v = digitalRead(SWITCH_PINS[i]) == HIGH ? 1 : 0;
  return SWITCH_ACTIVE_LOW ? (uint8_t)(1 - v) : v;
}

void setLed(uint8_t i, bool on) {
  digitalWrite(LED_PINS[i], (on == LED_ACTIVE_HIGH) ? HIGH : LOW);
}

void sendInputs() {
  // only send when the whole frame fits in the TX buffer, so Serial never blocks the loop
  if (Serial.availableForWrite() < 56) {
    return;
  }
  Serial.print(F("{\"type\":\"inputs\",\"A\":"));
  Serial.print(stableState[0]);
  Serial.print(F(",\"B\":"));
  Serial.print(stableState[1]);
  Serial.print(F(",\"C\":"));
  Serial.print(stableState[2]);
  Serial.print(F(",\"D\":"));
  Serial.print(stableState[3]);
  Serial.print(F(",\"u\":"));                  // uptime in 0.1 s: lets the PC notice a restart
  Serial.print(millis() / 100);
  Serial.print(F("}\n"));
  lastSendMs = millis();
}

void setLcdText(const char *line1, const char *line2) {
#if USE_LCD
  memcpy(lcdWanted[0], line1, 16);
  memcpy(lcdWanted[1], line2, 16);
#endif
}

// Writes at most LCD_CHARS_PER_LOOP changed characters, then returns.
void serviceLcd() {
#if USE_LCD
  if (lcd == nullptr) {
    return;
  }
  uint8_t budget = LCD_CHARS_PER_LOOP;
  for (uint8_t row = 0; row < 2 && budget > 0; row++) {
    for (uint8_t k = 0; k < 16 && budget > 0; k++) {
      if (lcdShown[row][k] != lcdWanted[row][k]) {
        lcd->setCursor(k, row);
        lcd->write(lcdWanted[row][k]);
        lcdShown[row][k] = lcdWanted[row][k];
        budget--;
      }
    }
  }
#endif
}

void showMirror() {
  for (uint8_t i = 0; i < 4; i++) {
    setLed(i, stableState[i]);
  }
  char l1[17];
  char l2[17];
  uint8_t value = (stableState[0] << 3) | (stableState[1] << 2) | (stableState[2] << 1) | stableState[3];
  snprintf(l1, sizeof(l1), "ABCD = %d%d%d%d     ", stableState[0], stableState[1], stableState[2], stableState[3]);
  snprintf(l2, sizeof(l2), "m%-2d  (no PC)     ", value);
  setLcdText(l1, l2);
}

// Frame layout (fixed width): "CMD:1,0,1,0:" + 16 chars + ":" + 16 chars  = 45 chars
void handleLine(const char *s, uint8_t n) {
  if (n < 11 || strncmp(s, "CMD:", 4) != 0) {
    return;
  }
  for (uint8_t i = 0; i < 4; i++) {
    char c = s[4 + 2 * i];
    if (c != '0' && c != '1') {
      return;                                  // corrupted frame
    }
  }
  lastCmdMs = millis();
  pcActive = true;
  for (uint8_t i = 0; i < 4; i++) {
    setLed(i, s[4 + 2 * i] == '1');
  }
  if (n >= 45 && s[11] == ':' && s[28] == ':') {
    setLcdText(s + 12, s + 29);
  }
}

void pollSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      if (rxLen > 0 && rxBuf[rxLen - 1] == '\r') {
        rxLen--;
      }
      handleLine(rxBuf, rxLen);
      rxLen = 0;
    } else if (rxLen < sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = c;
    } else {
      rxLen = 0;                               // overlong line: drop it
    }
  }
}

void pollSwitches() {
  unsigned long now = millis();
  bool changed = false;
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t raw = readSwitch(i);
    if (raw != lastRaw[i]) {
      lastRaw[i] = raw;
      rawChangedAt[i] = now;
    }
    if (raw != stableState[i] && now - rawChangedAt[i] >= DEBOUNCE_MS) {
      stableState[i] = raw;
      changed = true;
    }
  }
  if (changed || now - lastSendMs >= HEARTBEAT_MS) {
    sendInputs();
  }
  if (pcActive && now - lastCmdMs > PC_TIMEOUT_MS) {
    pcActive = false;                          // PC went away: back to mirroring
    changed = true;
  }
  if (!pcActive && changed) {
    showMirror();
  }
}

void setup() {
  wdt_disable();
  Serial.begin(115200);
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(SWITCH_PINS[i], SWITCH_ACTIVE_LOW ? INPUT_PULLUP : INPUT);
    pinMode(LED_PINS[i], OUTPUT);
    setLed(i, false);
  }
  for (uint8_t i = 0; i < 4; i++) {
    stableState[i] = lastRaw[i] = readSwitch(i);
    rawChangedAt[i] = millis();
  }
#if USE_LCD
  Wire.begin();
  Wire.setWireTimeout(3000, true);             // never hang if the I2C bus misbehaves
  Wire.beginTransmission(0x27);
  if (Wire.endTransmission() == 0) {
    lcd = &lcd27;
  } else {
    Wire.beginTransmission(0x3F);
    if (Wire.endTransmission() == 0) {
      lcd = &lcd3F;
    }
  }
  if (lcd != nullptr) {
    lcd->init();
    lcd->backlight();
    lcd->clear();
  }
  memset(lcdShown, ' ', sizeof(lcdShown));
  memset(lcdWanted, ' ', sizeof(lcdWanted));
#endif
  showMirror();
  sendInputs();
  wdt_enable(WDTO_2S);                         // from here on, a freeze reboots the board
}

// Wiring self-test without blocking: LED0 -> LED3 over the first 0.6 s after start-up.
// It runs alongside everything else, so the board reports its switches immediately,
// and it stops as soon as the PC takes over the LEDs.
void serviceSelfTest() {
  static bool finished = false;
  if (finished) {
    return;
  }
  unsigned long t = millis();
  if (pcActive || t >= 600) {
    finished = true;
    if (!pcActive) {
      showMirror();
    }
    return;
  }
  for (uint8_t i = 0; i < 4; i++) {
    setLed(i, i == t / 150);
  }
}

void loop() {
  wdt_reset();
  pollSerial();
  pollSwitches();
  serviceSelfTest();
  serviceLcd();
}
