// EchoPath ESP32 Firmware
// Ultrasonic + Buzzer Controller
//
// Reads distance from an HC-SR04 ultrasonic sensor and continuously streams it
// over Serial as "DIST:<value>". Listens for direction commands from the
// Python side (LEFT / RIGHT / CENTER / NONE) and drives a passive piezo buzzer
// with a distinct pattern for each.
//
// ✅ Compatible with BOTH ESP32 Arduino Core 2.x AND Core 3.x.
//    Core 3.x: ledcAttach(pin, freq, bits)  + ledcWriteTone(pin, freq)
//    Core 2.x: ledcSetup(ch, freq, bits)
//              + ledcAttachPin(pin, ch)
//              + ledcWriteTone(ch, freq)
//
// Wiring:
//   HC-SR04 TRIG → GPIO 5
//   HC-SR04 ECHO → GPIO 18
//   Passive Buzzer → GPIO 4  (use a transistor or direct 3.3V drive)

#define TRIG_PIN    5
#define ECHO_PIN    18
#define BUZZER_PIN  4
#define LEDC_CHAN   0       // used only on Core 2.x
#define LEDC_BITS   8

String command = "";

// ----------------------------
// Core version shim
// ----------------------------
// ESP_ARDUINO_VERSION_MAJOR is defined by the ESP32 Arduino core.
// Core 3.x = 3, Core 2.x = 2 (or undefined on very old installs → treat as 2).

inline void buzzerTone(uint32_t freq) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWriteTone(BUZZER_PIN, freq);
#else
  ledcWriteTone(LEDC_CHAN, freq);
#endif
}

inline void buzzerOff() {
  buzzerTone(0);
}

// ----------------------------
// Setup
// ----------------------------
void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Passive buzzer PWM — init differs between core versions
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(BUZZER_PIN, 2000, LEDC_BITS);
#else
  ledcSetup(LEDC_CHAN, 2000, LEDC_BITS);
  ledcAttachPin(BUZZER_PIN, LEDC_CHAN);
#endif

  Serial.println("EchoPath Ready");
}

// ----------------------------
// Ultrasonic
// ----------------------------
int getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);  // 30 ms timeout ~ 510 cm max
  if (duration == 0) return -1;
  return (int)(duration * 0.034 / 2);
}

// ----------------------------
// Buzzer Patterns
// ----------------------------

// LEFT = two short beeps (1500 Hz)
void leftBeep() {
  buzzerTone(1500); delay(200);
  buzzerOff();      delay(150);
  buzzerTone(1500); delay(200);
  buzzerOff();
}

// RIGHT = one long beep (2500 Hz)
void rightBeep() {
  buzzerTone(2500); delay(500);
  buzzerOff();
}

// CENTER = three rapid beeps (2000 Hz) — urgent, straight ahead
void centerBeep() {
  for (int i = 0; i < 3; i++) {
    buzzerTone(2000); delay(120);
    buzzerOff();      delay(80);
  }
}

// NONE = silence
// (handled by buzzerOff())

// ----------------------------
// Main Loop
// ----------------------------
void loop() {
  // ── Stream ultrasonic distance ──────────────────────────────────────────
  int dist = getDistance();
  Serial.print("DIST:");
  Serial.println(dist);

  // ── Receive Python command ──────────────────────────────────────────────
  if (Serial.available()) {
    command = Serial.readStringUntil('\n');
    command.trim();

    // Echo back so Python can confirm receipt
    Serial.print("CMD:");
    Serial.println(command);

    if      (command == "LEFT")   leftBeep();
    else if (command == "RIGHT")  rightBeep();
    else if (command == "CENTER") centerBeep();
    else                          buzzerOff();   // NONE or unknown → silent
  }

  delay(100);  // ~10 Hz distance polling
}
