#include "VibemanC.h"

// Functions
void setup() {
  delay(250);
  Serial.begin(115200);

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }

  pinMode(BTNPIN, INPUT_PULLUP);
  pinMode(POTPIN, INPUT);
  pinMode(LEDPIN, OUTPUT);

  analogReadRes(12);
  analogReadAveraging(32);

  digitalWrite(LEDPIN, LOW);
  //display.clearDisplay();
  // Display some sort of logo?
  display.display();

  digitalWrite(LEDPIN, HIGH);

  if (cardInit()) {
    Serial.println("SD card initialized");
    if (fileOpen()) {
      Serial.println("File opened");
      fileClose();
    }else Serial.println("Failed to open file");
  }else Serial.println("Failed to initialize SD card");

#ifdef HAS_RADIO
  radio.begin();
  radio.setChannel(108);
  radio.openWritingPipe(address[0]);
  radio.openReadingPipe(1, address[1]);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();

  if (radio.isChipConnected())
    Serial.println("Radio connected");
  else
    Serial.println("Radio connection failure");
#endif

  Serial.println("Setup complete.");
  display.clearDisplay();
  // Display UI
  display.display();
}

void loop() {
  readSerial();
  readRadio();
  delay(1);
}

bool cardInit() {
  // Open card
  if (!sd.begin(SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(50)))) return false;
  return true;
}

uint8_t fileOpen(int index) {
  // If index -1, increment file and write header
  // If index not -1, open file
  bool newfile = false;
  if (!sd.exists("SessionLog.csv"))
    newfile = true;
    
  if (!file.open("SessionLog.csv", O_RDWR | O_CREAT)) return 0;
  if (newfile) {
    for (uint8_t i = 0; i < 10; i++) {
      file.print(dLabel[i]);
      if (i < 9) file.print(',');
    }
    if (!file.print(F("\r\n"))) Serial.println("File write failed"); else Serial.println("File write successful");
  }
  if (newfile)
    return 2;

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
  float tmpRow[10] = {millis(), motState, motMax, motMin, motCur, btn, rampTimeS, pressure, avgPressure, sensitivity};
  memcpy(dSet[dNum++], tmpRow, sizeof(tmpRow));
  //dSet[dNum++] = tmpRow;
  return 0;
}

bool dataWrite() {
  // Write dataSet to SD card
  
  return false;
}

bool loadData() {
  // load data from current SD file to graphSet, working backwards until header or graphSet full

  return false;
}

void drawGraph() {
  // Working from end of graphSet, draw data to graph
}

void writeSerial() {
  Serial.println(String(cmdRx));
}

void writeRadio() {
  radio.stopListening();
  radio.write(&cmdRx, strlen(cmdRx));
}

int readSerial() {
  if (!Serial.available())
    return 0;

  memset(cmdTx, 0, sizeof(cmdTx));
  int tsize = 0;
  while (Serial.available())
    cmdTx[tsize++] = Serial.read();
    
  //writeRadio();
  writeSerial();
  return tsize;
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
    parseRx();
    
  return len;
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
    Serial.println(pch);
    memset(cmd, 0, sizeof(cmd));
    memset(dat, 0, sizeof(dat));
    memcpy(cmd, pch, 2);
    memcpy(&dat, &pch[2], strlen(pch) - 2);
    pch = strtok(NULL, ",!");
    //Serial.println(pch);

    if (cmd == "TS") {
      
    }
  }
}

void parseData(char cmd[] = cmdRx) {
  /*char &cmd[128] = cmdRx
  if (Tx)
    &cmd = cmdTx*/
    
  memset(cmd, 0, sizeof(cmd));
  String tmp = dLabel[0];
  tmp += "00:00:00.00";
  tmp += "," + dLabel[1] + (motCur >= motMin) ? 1 : 0;
  tmp += "," + dLabel[2] + motMax;
  tmp += "," + dLabel[3] + motMin;
  tmp += "," + dLabel[4] + motCur;
  tmp += "," + dLabel[5] + state;
  tmp += "," + dLabel[6] + rampTimeS;
  tmp += "," + dLabel[7] + pressure;
  tmp += "," + dLabel[8] + avgPressure;
  tmp += "," + dLabel[9] + sensitivity;

  tmp.toCharArray(cmd, tmp.length() + 1);
}
