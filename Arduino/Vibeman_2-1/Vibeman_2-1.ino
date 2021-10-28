// includes

// hardware
#define MOT_PIN 4
#define SW_PIN 3
#define POT_PIN A0
#define LED 13

// timing
#define FREQ 60
#define SW_LONG 600
#define SW_VLONG 2500
#define DEBOUNCE 50

#define PERIOD (1000/FREQ)
#define ONOFF (SW_LONG / PERIOD)

// button enumeration
#define BTN_NONE 0
#define BTN_SHORT 1
#define BTN_LONG 2
#define BTN_VLONG 3
#define BTN_HOLD 4

// variables
bool motState = 0;
uint8_t motMax = 255;
uint8_t motMin = 20;
float motCur = 0;
uint8_t btn = BTN_NONE;

int rampTimeS = 30;

void beep (int f1, int f2, int f3) {
  if (motCur > 245) analogWrite(MOT_PIN, 245);
  else if (motCur < 10) analogWrite(MOT_PIN, 10);

  analogWriteFrequency(MOT_PIN, f1);
  delay(250);
  analogWriteFrequency(MOT_PIN, f2);
  delay(250);
  analogWriteFrequency(MOT_PIN, f3);
  delay(250);
  analogWriteFrequency(MOT_PIN, 440);
  analogWrite(MOT_PIN, motCur);
}

void setup() {
  pinMode(MOT_PIN, OUTPUT);
  pinMode(SW_PIN, INPUT_PULLUP);
  pinMode(POT_PIN, INPUT);
  pinMode(LED, OUTPUT);

  analogReadRes(12);
  analogReadAveraging(32);

  digitalWrite(MOT_PIN, LOW);
  digitalWrite(LED, HIGH);
  delay(3000);

  Serial.begin(115200);
  beep(1047, 1396, 2093);
}

uint8_t check_button() {
  static bool lastBtn = LOW;
  static unsigned long keyDownTime = 0;
  uint8_t btnState = BTN_NONE;
  bool thisBtn = digitalRead(SW_PIN);

  if (thisBtn == LOW && lastBtn == HIGH) keyDownTime = millis();

  if (thisBtn == HIGH && lastBtn == LOW) {
    if ((millis() - keyDownTime) >= SW_VLONG) btnState = BTN_VLONG;
    else if ((millis() - keyDownTime) >= SW_LONG) btnState = BTN_LONG;
    else if ((millis() - keyDownTime) > DEBOUNCE) btnState = BTN_SHORT;
    else btnState = BTN_NONE;
  }else if (thisBtn == LOW) btnState = BTN_HOLD;

  lastBtn = thisBtn;
  return btnState;
}

void updateMotor() {
  static float motIncrement = 0.0;

  motMax = map(analogRead(POT_PIN), 0, 4095, 0, 255);
  motIncrement = ((float)motMax / ((float)FREQ * (float)rampTimeS));
  Serial.print(motMax);
  Serial.print(",");

  if (btn == BTN_SHORT) motCur = -0.5 * (float)rampTimeS * ((float)FREQ * motIncrement);
  else if (motCur < (float)motMax) motCur += motIncrement;

  if (motCur > motMax) motCur = motMax;

  if (!motState) motCur = 0;
  if (motCur > motMin) analogWrite(MOT_PIN, (int)motCur);
  else analogWrite(MOT_PIN, 0);
}

void loop() {
  if (millis() % PERIOD != 0) return;
  
  delay(1);
  btn = check_button();

  if (btn == BTN_LONG) motState = !motState;
  updateMotor();
  Serial.print(btn);
  Serial.print(",");
  Serial.print(motState);
  Serial.print(",");
  Serial.println(motCur);
}
