#include "Vibeman_2-3.h"

void setup() {
  delay(250);
  Serial.begin(115200);
  Serial.println("Radio initializing...");

#ifdef HAS_RADIO
  radio.begin();
  radio.setChannel(108);
  radio.openWritingPipe(address[0]);
  radio.openReadingPipe(1, address[1]);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();
  radio.setAutoAck(0, false);
  radio.setAutoAck(1, false);

  if (radio.isChipConnected())
    Serial.println("Radio connected");
  else
    Serial.println("Radio connection failure");
#endif
  
  pinMode(ENC_R,   OUTPUT); //Connected to RGB LED in the encoder
  pinMode(ENC_G, OUTPUT);
  pinMode(ENC_B,  OUTPUT);
  pinMode(ENC_SW,   INPUT); //Pin to read quadrature pulses from encoder

#ifndef HAS_RADIO
  pinMode(SW1PIN,   INPUT_PULLUP); //Set DIP switch pins as inputs
  pinMode(SW2PIN,   INPUT_PULLUP);
  pinMode(SW3PIN,   INPUT_PULLUP);
  pinMode(SW4PIN,   INPUT_PULLUP);
#endif

  /*digitalWrite(SW1PIN, HIGH); //Enable pullup resistors on DIP switch pins.
  digitalWrite(SW2PIN, HIGH); //They are tied to GND when switched on.
  digitalWrite(SW3PIN, HIGH);
  digitalWrite(SW4PIN, HIGH);*/

  pinMode(MOT_PIN,OUTPUT); //Enable "analog" out (PWM)
  
  pinMode(PRES_PIN,INPUT); //default is 10 bit resolution (1024), 0-3.3
  analogReadRes(12); //changing ADC resolution to 12 bits (4095)
  analogReadAveraging(32); //To reduce noise, average 32 samples each read.
  
  raPressure.clear(); //Initialize a running pressure average

  digitalWrite(MOT_PIN, LOW);//Make sure the motor is off
  analogWriteFrequency(MOT_PIN, MOT_FREQ);

  //delay(3000); // 3 second delay for recovery

  //If a pin reads low, the switch is enabled. Here, we read in the DIP settings
  //Right now, only SW1 is used, for enabling higher maximum motor speed.
  /*Serial.println(digitalRead(SW1PIN));
  if(digitalRead(SW1PIN)){
    MOT_MAX = 179; //At the default low position, limit the motor speed
  }
  else{
    MOT_MAX = 255; //When SW1 is flipped high, allow higher motor speeds
  }
  Serial.println(MOT_MAX);*/
  /*SW2 = (digitalRead(SW2PIN) == LOW);
  SW3 = (digitalRead(SW3PIN) == LOW);
  SW4 = (digitalRead(SW4PIN) == LOW);*/

  FastLED.addLeds<PIX_TYPE, PIX_PIN, PIX_ORDER>(pix, PIX_NUM).setCorrection(TypicalLEDStrip);
  // limit power draw to .6A at 5v... Didn't seem to work in my FastLED version though
  //FastLED.setMaxPowerInVoltsAndMilliamps(5,DEFAULT_PLIMIT);
  FastLED.setBrightness(BRIGHTNESS);

  //Recall saved settings from memory
  sensitivity = EEPROM.read(SENSITIVITY_ADDR);
  motMax = min(EEPROM.read(MAX_SPEED_ADDR),motMax); //Obey the MOT_MAX the first power  cycle after chaning it.
  beep(1047,1396,2093); //Power on beep
}

void loop() {
  static Mode state = MANUAL;
  static int sampleTick = 0;
  //Run this section at the update frequency (default 60 Hz)
  if (tickUpdate >= PERIOD) {
    //delay(1);
    
    sampleTick++; //Add pressure samples to the running average slower than 60Hz
    if (sampleTick % RA_TICK_PERIOD == 0) {
      raPressure.addValue(pressure);
      avgPressure = raPressure.getAverage();
    }
    
    pressure = analogRead(PRES_PIN);
    fadeToBlackBy(pix,PIX_NUM,20); //Create a fading light effect. LED buffer is not otherwise cleared
    Btn btnState = btnCheck();
    state = setState(btnState,state); //Set the next state based on this state and button presses
    runUpdate(state);
    FastLED.show(); //Update the physical LEDs to match the buffer in software

    //Alert that the Pressure voltage amplifier is railing, and the trim pot needs to be adjusted
    if(pressure > 4030)beep(2093,2093,2093); //Three high beeps

    //Report pressure and motor data over USB for analysis / other uses. timestamps disabled by default
    //Serial.print(millis()); //Timestamp (ms)
    //Serial.print(",");

#ifndef DEBUG_ONLY
    /*Serial.print(motCur); //Motor speed (0-255)
    Serial.print(",");
    Serial.print(pressure); //(Original ADC value - 12 bits, 0-4095)
    Serial.print(",");
    Serial.println(avgPressure); //Running average of (default last 25 seconds) pressure*/
    parseData(state);
    //Serial.println(cmd);
#endif
    //updateMotor();
    Serial.print(tickUpdate);
    Serial.print(" : ");
    Serial.println(PERIOD);
    tickUpdate = 0;
    writeRadio();
  }
}

void beep(int f1, int f2, int f3) {
  if(motCur > 245) analogWrite(MOT_PIN, 245); //make sure the frequency is audible
  else if(motCur < 10) analogWrite(MOT_PIN, 10);
  analogWriteFrequency(MOT_PIN, f1);
  delay(250);
  analogWriteFrequency(MOT_PIN, f2);
  delay(250);
  analogWriteFrequency(MOT_PIN, f3);
  delay(250);
  analogWriteFrequency(MOT_PIN, MOT_FREQ);
  analogWrite(MOT_PIN, motCur);
}

void showKnobRGB(const CRGB& rgb) {
  analogWrite(ENC_R, rgb.r);
  analogWrite(ENC_G, rgb.g);
  analogWrite(ENC_B, rgb.b);
}

void draw_cursor_3(int pos,CRGB C1, CRGB C2, CRGB C3){
  pos = constrain(pos,0,PIX_NUM*3-1);
  int colorNum = pos/PIX_NUM; //revolution number
  int cursorPos = pos % PIX_NUM; //place on circle, from 0-12
  switch(colorNum){
    case 0:
      pix[cursorPos] = C1;
      break;
    case 1:
      pix[cursorPos] = C2;
      break;
    default:
      pix[cursorPos] = C3;
      break;
  }
}

void draw_cursor(int pos,CRGB C1){
  pos = constrain(pos,0,PIX_NUM-1);
  pix[pos] = C1;
}

void draw_bars_3(int pos,CRGB C1, CRGB C2, CRGB C3){
  pos = constrain(pos,0,PIX_NUM*3-1);
  int colorNum = pos/PIX_NUM; //revolution number
  int barPos = pos % PIX_NUM; //place on circle, from 0-12
  switch(colorNum){
    case 0:
      fill_gradient_RGB(pix,0,C1,barPos,C1);
      //pix[barPos] = C1;
      break;
    case 1:
      fill_gradient_RGB(pix,0,C1,barPos,C2);
      break;
    case 2:
      fill_gradient_RGB(pix,0,C2,barPos,C3);
      break;
  }
}

int encLimitRead(int minVal, int maxVal){
  if(myEnc.read()>maxVal*4)myEnc.write(maxVal*4);
  else if(myEnc.read()<minVal*4) myEnc.write(minVal*4);
  return constrain(myEnc.read()/4,minVal,maxVal);
}

void runManual() {
  //In manual mode, only allow for 13 cursor positions, for adjusting motor speed.
  int knob = encLimitRead(0,12);
  motCur = map(knob, 0, 12, 0., (float)motMax);
  analogWrite(MOT_PIN, motCur);

  //gyrGraphDraw(avgPressure, 0, 4 * 3 * NUM_LEDS);
  int presDraw = map(constrain(pressure - avgPressure, 0, pLimit),0,pLimit,0,PIX_NUM*3);
  draw_bars_3(presDraw, CRGB::Green,CRGB::Yellow,CRGB::Red);
  draw_cursor(knob, CRGB::Red);
}

void runAuto() {
  /*static float motIncrement = 0.0;
  motIncrement = ((float)motMax / ((float)FREQ * (float)rampTimeS));

  int knob = encLimitRead(0,(3*PIX_NUM)-1);
  sensitivity = knob*4; //Save the setting if we leave and return to this state
  //Reverse "Knob" to map it onto a pressure limit, so that it effectively adjusts sensitivity
  pLimit = map(knob, 0, 3 * (PIX_NUM - 1), 600, 1); //set the limit of delta pressure before the vibrator turns off
  //When someone clenches harder than the pressure limit
  if (pressure - avgPressure > pLimit) {
    motCur = -.5*(float)rampTimeS*((float)FREQ*motIncrement);//Stay off for a while (half the ramp up time)
  }
  else if (motCur < (float)motMax) {
    motCur += motIncrement;
  }
  if (motCur > motMin) {
    analogWrite(MOT_PIN, (int) motCur);
  } else {
    analogWrite(MOT_PIN, 0);
  }

  int presDraw = map(constrain(pressure - avgPressure, 0, pLimit),0,pLimit,0,PIX_NUM*3);
  draw_bars_3(presDraw, CRGB::Green,CRGB::Yellow,CRGB::Red);
  draw_cursor_3(knob, CRGB::Blue,CRGB::Cyan,CRGB::Purple);*/
  updateMotor();
  analogWrite(MOT_PIN, motCur);
}

void runRadio() {
  // To do:  This section
}

void runOptSpeed() {
  Serial.println("speed settings");
  int knob = encLimitRead(0,12);
  motCur = map(knob, 0, 12, 0., (float)motMax);
  analogWrite(MOT_PIN, motCur);
  motMax = motCur; //Set the maximum ramp-up speed in automatic mode
  //Little animation to show ramping up on the LEDs
  static int visRamp = 0;
  if(visRamp <= FREQ*PIX_NUM-1) visRamp += 16;
  else visRamp = 0;
  draw_bars_3(map(visRamp,0,(PIX_NUM-1)*FREQ,0,knob),CRGB::Green,CRGB::Green,CRGB::Green);
}

void runOptRamp() {
  Serial.println("rampSpeed");
}

void runOptBeep() {
  Serial.println("Brightness Settings");
}

void runOptPres() {
  int p = map(analogRead(PRES_PIN),0,4095,0,PIX_NUM-1);
  draw_cursor(p,CRGB::White);
}

Btn btnCheck() {
  static bool lastBtn = LOW;
  static unsigned long keyDownTime = 0;
  Btn btnState = UP;
  bool thisBtn = digitalRead(ENC_SW);

  if (thisBtn == HIGH && lastBtn == LOW) keyDownTime = millis();

  if (thisBtn == LOW && lastBtn == HIGH) {
    if ((millis() - keyDownTime) >= SW_VLONG) btnState = VLONG;
    else if ((millis() - keyDownTime) >= SW_LONG) btnState = LONG;
    else if ((millis() - keyDownTime) > DEBOUNCE) btnState = SHORT;
    else btnState = DOWN;
  }else if (thisBtn == HIGH) btnState = DOWN;

  lastBtn = thisBtn;
  return btnState;
}

void runUpdate(Mode state){
  switch (state) {
      case MANUAL:
        showKnobRGB(CRGB::Red);
        runManual();
        break;
      case AUTO:
        showKnobRGB(CRGB::Blue);
        runAuto();
        break;
      case OPT_SPEED:
        showKnobRGB(CRGB::Green);
        runOptSpeed();
        break;
      case OPT_RAMPSPD:
        showKnobRGB(CRGB::Yellow);
        runOptRamp();
        break;
      case OPT_BEEP:
        showKnobRGB(CRGB::Purple);
        runOptBeep();
        break;
      case OPT_PRES:
        showKnobRGB(CRGB::White);
        runOptPres();
        break;
      default:
        runManual();
        break;
    }
}

Mode setState(Btn btnState, Mode state){
  if(btnState == UP || btnState == DOWN){
    return state;
  }
  if(btnState == VLONG){
    //Turn the device off until woken up by the button
    Serial.println("power off");
    fill_gradient_RGB(pix,0,CRGB::Black,PIX_NUM-1,CRGB::Black);//Turn off LEDS
    FastLED.show();
    showKnobRGB(CRGB::Black);
    analogWrite(MOT_PIN, 0);
    beep(2093,1396,1047);
    analogWrite(MOT_PIN, 0); //Turn Motor off
    while(!digitalRead(ENC_SW))delay(1);
    beep(1047,1396,2093);
    return MANUAL ;
  }
  else if(btnState == SHORT){
    switch(state){
      case MANUAL:
        myEnc.write(sensitivity);//Whenever going into auto mode, keep the last sensitivity
        motCur = 0; //Also reset the motor speed to 0
        motTick = 0;
        motStep = 0;
        return AUTO;
      case AUTO:
        myEnc.write(0);//Whenever going into manual mode, set the speed to 0.
        motCur = 0;
        EEPROM.update(SENSITIVITY_ADDR, sensitivity);
        return MANUAL;
      case OPT_SPEED:
        myEnc.write(0);
        EEPROM.update(MAX_SPEED_ADDR, motMax);
        //return OPT_RAMPSPD;
        //return OPT_BEEP;
        motCur = 0;
        analogWrite(MOT_PIN, motCur); //Turn the motor off for the white pressure monitoring mode
        return OPT_PRES; //Skip beep and rampspeed settings for now
      case OPT_RAMPSPD: //Not yet implimented
        //motSpeed = 0;
        //myEnc.write(0);
        return OPT_BEEP;
      case OPT_BEEP:
        myEnc.write(0);
        return OPT_PRES;
      case OPT_PRES:
        myEnc.write(map(motMax,0,255,0,4*(PIX_NUM)));//start at saved value
        return OPT_SPEED;
    }
  }
  else if(btnState == LONG){
    switch (state) {
          case MANUAL:
          case AUTO:
            myEnc.write(map(motMax,0,255,0,4*(PIX_NUM)));//start at saved value
            return OPT_SPEED;
          case OPT_SPEED:
            myEnc.write(0);
            return MANUAL;
          case OPT_RAMPSPD:
          case OPT_BEEP:
            return MANUAL;
          case OPT_PRES:
            myEnc.write(0);
            return MANUAL;
        }
  }
  else return MANUAL;
}

void writeSerial() {
  Serial.println(String(cmd));
}

void writeRadio() {
  /*static uint32_t xmit = 0;
  xmit = ++xmit % 4;
  if (xmit)
    return;*/
    
  char tmp[128] = "";
  radio.stopListening();
  while (strlen(cmd) > 32) {
    memset(tmp, 0, sizeof(tmp));
    memcpy(tmp, cmd, sizeof(cmd));
    Serial.print(cmd);
    Serial.println((radio.writeFast(&cmd, 32, false)) ? 'S' : 'F');
      /*Serial.println("sent");
    else
      Serial.println("fail");*/
      
    memset(cmd, 0, sizeof(cmd));
    memcpy(&cmd, &tmp[32], sizeof(cmd) - sizeof(char) * 32);
    //Serial.println(cmd);
    //delay(1);
  }
  radio.writeFast(&cmd, strlen(cmd), false);
  radio.txStandBy();
  //Serial.println(cmd);
}

int readSerial() {
  if (!Serial.available())
    return 0;

  memset(cmd, 0, sizeof(cmd));
  int tsize = 0;
  while (Serial.available())
    cmd[tsize++] = Serial.read();

  return tsize;
}

int readRadio() {
  if (!radio.available())
    return 0;

  memset(cmd, 0, sizeof(cmd));
  uint16_t len = radio.getDynamicPayloadSize();
  radio.read(&cmd, len);
  return len;
}

void parseCmd() {
  // Do a thing here.
}

void parseData(Mode state) {
  memset(cmd, 0, sizeof(cmd));
  String tmp = "!" + sLabel[0];
  tmp += millis();
  tmp += "," + sLabel[1] + ((motCur >= motMin) ? 1 : 0);
  tmp += "," + sLabel[2] + motMax;
  tmp += "," + sLabel[3] + motMin;
  tmp += "," + sLabel[4] + motCur;
  tmp += "," + sLabel[5] + state;
  tmp += "," + sLabel[6] + rampTimeS;
  tmp += "," + sLabel[7] + pressure;
  tmp += "," + sLabel[8] + avgPressure;
  tmp += "," + sLabel[9] + sensitivity + "!";

  tmp.toCharArray(cmd, tmp.length() + 1);
}

void updateMotor() {
  uint8_t patternLen = sizeof(vWave) / sizeof(vWave[0]);
  uint16_t patternTick = 0;
  uint16_t stepTick = 0;

  if (motTick > vWave[motStep].deltaT * FREQ) {
    motTick = 0;
    motStep = ++motStep % patternLen;
  }

  uint16_t mLen = vWave[motStep].deltaT * FREQ;
  //float tween = 0.5f * cos(PI * (float)(motTick / (vWave[motStep].deltaT * FREQ)) + PI) + 0.5f;

  float mFrom = vWave[motStep].pwr;
  float mDelta = vWave[0].pwr - mFrom;
  if (motStep < patternLen - 1)
    mDelta = vWave[motStep + 1].pwr - mFrom;

  motCur = mDelta * tween(motTick, mLen, vWave[motStep].curve) + mFrom;
  /*if (motCur <= motMin)
    analogWrite(MOT_PIN, 0);
  else if (motCur >= motMax)
    analogWrite(MOT_PIN, motMax);
  else
    analogWrite(MOT_PIN, motCur);*/
  
#ifdef DEBUG
  /*Serial.print(motStep);
  Serial.print('/');
  Serial.print(patternLen);
  Serial.print(" ");
  Serial.print(motTick);
  Serial.print('/');
  Serial.print(mLen);
  Serial.print(" ");

  Serial.print(mFrom);
  Serial.print("-");
  Serial.print(mFrom + mDelta);
  Serial.print(" : ");*/
  //Serial.println(tween(motTick, mLen, vWave[motStep].curve));
  //Serial.print(" :: ");

  Serial.println(motCur);
#endif
  //Serial.println(tween);
  
  motTick++;
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
