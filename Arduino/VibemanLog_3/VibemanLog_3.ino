#include "VibemanLog.h"

void setup() {
  delay(250);                   // Delay to allow radio to finish powering up
  Serial.begin(4608000);

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }

  pinMode(BTNPIN, INPUT_PULLUP);
  pinMode(EMGPIN, INPUT);
  pinMode(LEDPIN, OUTPUT);

  analogReadRes(12);        // Changing ADC resolution to 12 bits (4095)
  analogReadAveraging(32);      // Averaging analog input helps prevent jitter

  // Display some sort of logo?
  display.display();

  digitalWrite(LEDPIN, HIGH);
  
  Serial.println((cardInit()) ? "SD card initialized" : "Failed to initialize SD card");

#ifdef HAS_RADIO
  radio.begin();
  radio.setChannel(108);
  radio.openWritingPipe(address[0]);
  radio.openReadingPipe(1, address[1]);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();
  radio.setAutoAck(0, false);
  radio.setAutoAck(1, false);

  Serial.println((radio.isChipConnected()) ? "Radio connected" : "Radio failure");
#endif

  Serial.println("Setup complete.");
  display.clearDisplay();
  // Display UI
  display.display();
}

void loop() {
  // put your main code here, to run repeatedly:
  //readRadio();
  btn = btnCheck();
   //pot = analogRead(POTPIN);
  if (btn == LONG) {
    recording = !recording;
    rec = 0;
  }
  
  updateRec();
#ifdef DEBUG
  Serial.println(analogRead(EMGPIN));
#endif
}

void writeSerial() {
  Serial.println(String(cmdRx));
}

int readRadio() {
  if (!radio.available())
    return 0;
  char tmp[32] = "";
  uint16_t len = radio.getDynamicPayloadSize();
  radio.read(&tmp, len);
  //Serial.println(tmp);
  if (tmp[0] == '!')
    memset(cmdRx, 0, sizeof(cmdRx));

  memcpy(&cmdRx[strlen(cmdRx)], &tmp, len);

  if (cmdRx[strlen(cmdRx) - 1] == '!')
    writeSerial();
    
  return len;
}

void print2digits(int number) {
  if (number >= 0 && number < 10) {
    Serial.write('0');
  }
  Serial.print(number);
}

bool cardInit() {
  // Open card
  if (!sd.begin(SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(50)))) return false;
  return true;
}

uint8_t fileOpen(int index) {
  // If index -1, increment file and write header
  // If index not -1, open file
  // Not currently implemented

  tmElements_t tm;
  char filename1[19] = "0000-00-00_0000_";

  if (RTC.read(tm)) {
    sprintf(filename1, "%4d-%02d-%02d_%02d%02d_",tmYearToCalendar(tm.Year), tm.Month, tm.Day, tm.Hour, tm.Minute);
  }

  uint8_t i = 0;
  sprintf(filename1[16], "%02d.csv",i);
  while (sd.exists(filename1)) {
    sprintf(filename1[16], "%02d.csv",++i);
  }

#ifdef DEBUG
  Serial.print("Recording to ");
  Serial.println(filename1);
#endif

  if (!file.open(filename1, O_RDWR | O_CREAT)) return 0;
  for (uint8_t i = 0; i < 8; i++) {
    file.print(dLabel[i]);
    if (i < 7) file.print(',');
  }
  if (!file.print(F("\r\n"))) Serial.println("File write failed"); else Serial.println("File write successful");

  return 1;
}

void fileClose() {
  file.close();
}

void clearSet() {
  // erase dataSet, set index to 0
  memset (dSet, 0, sizeof(dSet));
  dNum = 0;
}

uint8_t dataSet() {
  // Add data set to buffer
  // Add data to graphSet array, pop if necessary
  // Return free space in buffer
  /*float tmpRow[10] = {millis(), motState, motMax, motMin, motCur, btn, rampTimeS, pressure, avgPressure, sensitivity};
  memcpy(dSet[dNum++], tmpRow, sizeof(tmpRow));*/
  //dSet[dNum++] = tmpRow;

  parseRx();
  
  Serial.println(dNum);
  if (dNum == BUFFER_SIZE) {
    dataWrite();
    clearSet();
  }

  dSet[dNum] = {((float)rec/86400000.0f), motCur + 3000, mode, pressure, avgPressure, btn * 250 + 2000, sensitivity, analogRead(EMGPIN)};
  dNum++;
  return 0;
}

bool dataWrite() {
  for (uint8_t i = 0; i < dNum; i++) {
    file.print((String)dSet[i].ts);
    file.print(',');
    file.print((String)dSet[i].motCur);
    file.print(',');
    file.print((String)dSet[i].mode);
    file.print(',');
    file.print((String)dSet[i].pressure);
    file.print(',');
    file.print((String)dSet[i].avgPressure);
    file.print(',');
    file.print((String)dSet[i].btn);
    file.print(',');
    file.print((String)dSet[i].sensitivity);
    file.print(',');
    file.print((String)dSet[i].emg);
    if (!file.print(F("\r\n"))) {
      Serial.println("SD card write failed.");
      return false;
    }
  }
  return true;
}

Btn btnCheck() {
  static bool lastBtn = HIGH;
  static unsigned long keyDownTime = 0;
  Btn btnState = UP;
  bool thisBtn = digitalRead(BTNPIN);

  if (thisBtn == LOW && lastBtn == HIGH) keyDownTime = millis();

  if (thisBtn == HIGH && lastBtn == LOW) {
    if ((millis() - keyDownTime) >= SW_VLONG) btnState = VLONG;
    else if ((millis() - keyDownTime) >= SW_LONG) btnState = LONG;
    else if ((millis() - keyDownTime) > DEBOUNCE) btnState = SHORT;
    else btnState = DOWN;
  }else if (thisBtn == LOW) btnState = DOWN;

  lastBtn = thisBtn;
  return btnState;
}

float tween(uint16_t curTick, uint16_t lastTick, CURVE curve = LINEAR) {
  float tOut = 0.0f;
  switch (curve) {
    case LINEAR:
      tOut = (float)curTick / (float)lastTick;
      break;

    case QUADRATIC:
      tOut = pow(((float)curTick / (float)lastTick),2);
      break;

    case EASEOUT:
      tOut = 1 - sqrt((1 - pow((float)curTick / (float)lastTick, 2)));
      break;

    case EASEIN:
      tOut = sqrt(1 - pow(((float)curTick / (float)lastTick) - 1,2));
      break;

    case COSINE:
      tOut = 0.5f * cos(PI * ((float)curTick / (float)lastTick) + PI) + 0.5f;
      break;

    case NA:
    default:
      return 0;
      break;
  }

  return tOut;
}

void updateRec() {
  static bool lastRec = false;
  static elapsedMillis recLight = 0;
  bool newData = readRadio();
  if (lastRec == false && recording == true) {      // If we just started recording, open a new file
    fileOpen();
#ifdef DEBUG
    Serial.println("Recording beginning.");
#endif
  }
  else if (lastRec == true && recording == false) { // If we just stopped recording, close the file
    fileClose();
#ifdef DEBUG
    Serial.println("Recording ended.");
#endif
    analogWrite(LEDPIN, 255);
  }
    
  if (recording) {                                  // If we are currently recording, update the buffer
    if (newData)                                    // Only record new data if any available
      dataSet();
      
    if (recLight < 1000)
      analogWrite(LEDPIN, 255 * tween(recLight, 1000));
    else
      analogWrite(LEDPIN, 255 - 255 * tween(recLight - 1000, 1000));

    if (recLight > 2000)
      recLight = 0;
  }

  lastRec = recording;
}

void parseRx() {
  // Do a thing here.
  char tmp[128] = "";
  char cmd[2] = "";
  char dat[16] = "";
  char * pch;
  memcpy(tmp, cmdRx, strlen(cmdRx));   // Copy cmdRx to tmp.
  //Serial.println(tmp);

  pch = strtok(tmp, ",!");
  while (pch != NULL) {
    memset(cmd, 0, sizeof(cmd));
    memset(dat, 0, sizeof(dat));
    memcpy(cmd, pch, 2);
    memcpy(&dat, &pch[2], strlen(pch) - 2);
    pch = strtok(NULL, ",!");

    if (cmd[1] == 'C')  motCur = atof(dat);
    else if (cmd[1] == 'D') mode = atoi(dat);
    else if (cmd[1] == 'S') pressure = atoi(dat);
    else if (cmd[1] == 'A') avgPressure = atoi(dat);
    else if (cmd[1] == 'N') sensitivity = atoi(dat);
  }
#ifdef DEBUG
    Serial.print(motCur);
    Serial.print(' ');
    Serial.print(mode);
    Serial.print(' ');
    Serial.print(pressure);
    Serial.print(' ');
    Serial.print(avgPressure);
    Serial.print(' ');
    Serial.print(sensitivity);
    Serial.print(' ');
    Serial.println(analogRead(EMGPIN));
#endif
}
