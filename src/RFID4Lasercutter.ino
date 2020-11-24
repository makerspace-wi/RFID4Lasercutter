/* DESCRIPTION
  ====================
  started on 01JUN2017 - uploaded on 06.06.2017 by Dieter
  Code for machine and cleaner control over RFID
  reading IDENT from xBee, retrait sending ...POR until time responds

  Commands to Raspi --->
  'MAxx'  - from xBee (=Ident)
  'POR'   - machine power on reset (Ident;por)

  'Ident;on'   - machine reporting ON-Status
  'Ident;off'  - machine reporting OFF-Status
  'card;nn...' - uid_2 from reader

  Commands from Raspi
  'time'   - format time33.33.33 33:33:33
  'onp'    - Machine permanent ON
  'ontxx'  - Machine xxx minutes ON
  'off'    - Machine OFF
  'noreg'  - uid_2 not registered

  'setce'  - set time before ClosE machine
  'setcn'  - set time for longer CleaN on
  'setcl'  - set Current Level for switching on and off
  'dison'  - display on for 60 setCursor
  'r3t...' - display text in row 3 "r3tabcde12345", max 20
  'r4t...' - display text in row 4 "r4tabcde12345", max 20

  Commands to Lasercutter
  'lacu,sd' - LCIdent started?
  'lacu;ok' - LCIdent on?
  'lacu;em' - LCIdent error message?
  'lsdis'   - Laser disable
  'lsena'   - Laser enable
  'ok'      - Handshake return

  Commands from Lasercutter
  'lacu'      - LAserCUtter =LCIdent
  'por'       - machine power on reset (LCIdent;por)
  'err'       - bus error ocurse
  'sdv;00.0'  - Send Display Temperatur Vorlauf °C
  'sdr;00.0'  - Send Display Temperatur Rücklauf °C
  'sdf;00.0'  - Send Display Flowmeter value l/min
  'msl;x'     - message Laser enabled 1 = active (displayed as las.ena / las.dis)
  'mse;x'     - message Emergency stop 1 = active
  'msc;x'     - message Cover positon 1 = open
  'msp;x'     - message power on 1 = switch on and voltage ok

  'er0;Vorlauf? DS18B20' - no sensor avilable
  'er1;Rueckl.? DS18B20' - no sensor avilable

  last change: 18.11.2020 by Michael Muehl
  changed: switching lasercutter on and off with RFID detection
           (new version of controlling laser)
*/
#define Version "7.0.1" // (Test =7.0.x ==> 7.0.2)

#include <Arduino.h>
#include <TaskScheduler.h>
#include <Wire.h>
#include <LCDLED_BreakOUT.h>
#include <utility/Adafruit_MCP23017.h>
#include <SPI.h>
#include <MFRC522.h>

#include <SoftwareSerial.h>

// PIN Assignments
// RFID Control -------
#define RST_PIN      4  // RFID Reset
#define SS_PIN      10  // RFID Select

// Machine Control (ext)
#define currMotor   A0  // [Input] Motor current (Machine) [no used]
#define SSR_Machine A2  // SSR Machine on / off  (Laser)
#define REL_RS232   A3  // Relais RS232 connected on / off

#define BUSError     8  // Bus error

// Softserial Lasercutter
#define RxD 2           // receive data
#define TxD 5           // transmit data

// I2C IOPort definition
byte I2CFound = 0;
byte I2CTransmissionResult = 0;
#define I2CPort   0x20  // I2C Adress MCP23017

// Pin Assignments Display (I2C LCD Port A/LED +Button Port B)
// Switched to LOW
#define FlashLED_A   0  // Flash LEDs oben
#define FlashLED_B   1  // Flash LEDs unten
#define buzzerPin    2  // Buzzer Pin
#define LCPOWUPLed   3  // LED Button POWER
// Switched High - Low - High - Low
#define StopLEDrt    4  // StopLEDrt (LED + Stop-Taster)
#define StopLEDgn    5  // StopLEDgn (LED - Stop-Taster)
// switch to HIGH Value (def .h)
// BUTTON_P1  2         // POWERBUTTON
// BUTTON_P2  1         // StopSwitch
// BACKLIGHT for LCD-Display
#define BACKLIGHToff 0x0
#define BACKLIGHTon  0x1

// DEFINES
#define porTime         5 // wait seconds for sending Ident + POR
#define CLOSE2END      15 // MINUTES before activation is off
#define CLEANON         4 // TASK_SECOND vac on for a time

// CREATE OBJECTS
Scheduler runner;
LCDLED_BreakOUT lcd = LCDLED_BreakOUT();
MFRC522 mfrc522(SS_PIN, RST_PIN);  // Create MFRC522 instance

SoftwareSerial connectLaser(RxD, TxD);

// Callback methods prototypes
void checkXbee();        // Task connect to xBee Server
void BlinkCallback();    // Task to let LED blink - added by D. Haude 08.03.2017
void UnLoCallback();     // Task to Unlock machine
void BuzzerOn();         // added by DieterH on 22.10.2017
void FlashCallback();    // Task to let LED blink - added by D. Haude 08.03.2017
void DispOFF();          // Task to switch display off after time
void MesaDelay();        // Task to send message to controller delayed

void OnTimed(long);
void flash_led(int);

void connectLaserEvent(); // Softserial Lasercutter

// TASKS
Task tM(TASK_SECOND / 2, TASK_FOREVER, &checkXbee);	    // 500ms main task
Task tU(TASK_SECOND / 2, TASK_FOREVER, &UnLoCallback);  // 500ms
Task tB(TASK_SECOND * 5, TASK_FOREVER, &BlinkCallback); // 5000ms added M. Muehl

Task tBU(TASK_SECOND / 10, 6, &BuzzerOn);               // 100ms 6x =600ms added by DieterH on 22.10.2017
Task tBD(1, TASK_ONCE, &FlashCallback);                 // Flash Delay
Task tDF(1, TASK_ONCE, &DispOFF);                       // display off
Task tMD(1, TASK_ONCE, &MesaDelay);                     // send message delayed

// Communication with Lasercutter
Task tCL(TASK_SECOND / 50 + 3, TASK_FOREVER, &connectLaserEvent); // 23ms
Task tV(TASK_SECOND + 4, TASK_FOREVER, &dispVALUES);         // 500ms (1004ms)

// VARIABLES
unsigned long val;
unsigned int timer = 0;
bool onTime = false;
int minutes = 0;
bool toggle = false;
bool flashB;
unsigned long code;
byte atqa[2];
byte atqaLen = sizeof(atqa);
byte mesaDy = 0;  // message number delayed
byte mesaBg = 0;  // message number blinking

uint8_t prevButt = 0;       // prevue button value
bool powMode = LOW;         // Lasercutter switched off / on
// Variables can be set externaly: ---
// --- on timed, time before new activation
unsigned int CLOSE = CLOSE2END; // RAM cell for before activation is off
bool firstCLOSE = false;
// --- for cleaning
unsigned int CLEAN = CLEANON; // RAM cell for Dust vaccu cleaner on
unsigned int CURLEV = 0;      // RAM cell for before activation is off

// Serial with xBee
String inStr = "";      // a string to hold incoming data
String IDENT = "";      // Machine identifier for remote access control
byte plplpl = 0;        // send +++ control AT sequenz
byte getTime = porTime;

// Lasercutter ---------
// sd:
String txtempV = "00.0";     // Temperature Vorlauf (V)
String txtempR = "00.0";     // Temperature Rücklauf (R)
String txtflow = "00.0";     // represents Liter/minute
String txLaser = "LAS.DIS";  // Laser value: Dis-, Enable

String LCIDENT = "LACU";     // Machine identifier for LAserCUtercontroller
String inSfStr = "";         // a string to hold SoFtserial incoming data
bool displayIsON = true;

bool isLaser = false;        // - message Laser enabled 1 = active(displayed as las.ena / las.dis)
bool isTmpSn = false;        // - message Temperature sensor error 1 = active
bool isEmerg = false;        // - message Emergency stop 1 = active
bool isCover = false;        // - message Cover positon 1 = open
bool isPower = false;        // - message power on 1 = switch on and voltage ok

// ======>  SET UP AREA <=====
void setup() {
  //init Serial port
  Serial.begin(57600);  // Serial
  inStr.reserve(40);    // reserve for instr serial input
  IDENT.reserve(5);     // reserve for IDENT serial output
  //init Softserial
  connectLaser.begin(9600);
  inSfStr.reserve(30);  // reserve for insfstr serial input

  // initialize:
  Wire.begin();         // I2C
  lcd.begin(20,4);      // initialize the LCD
  SPI.begin();          // SPI
  mfrc522.PCD_Init();   // Init MFRC522
  mfrc522.PCD_SetAntennaGain(mfrc522.RxGain_max);

  // IO MODES
  pinMode(BUSError, OUTPUT);
  pinMode(SSR_Machine, OUTPUT);
  pinMode(REL_RS232, OUTPUT);

  // Set default values
  digitalWrite(BUSError, HIGH);	// turn the LED ON (init start)
  digitalWrite(SSR_Machine, LOW);
  digitalWrite(REL_RS232, HIGH);

  runner.init();
  runner.addTask(tM);
  runner.addTask(tB);

  runner.addTask(tU);
  runner.addTask(tBU);
  runner.addTask(tBD);
  runner.addTask(tDF);
  runner.addTask(tMD);
  // Softserial
  runner.addTask(tCL);
  // Display values
  runner.addTask(tV);

  // I2C _ Ports definition only for test if I2C is avilable
  Wire.beginTransmission(I2CPort);
  I2CTransmissionResult = Wire.endTransmission();
  if (I2CTransmissionResult == 0) {
    I2CFound++;
  }
  // I2C Bus mit slave vorhanden
  if (I2CFound != 0)
  {
    lcd.clear();
    lcd.print("Wait for controller");
    tCL.enable();  // soft serial enable
    mesaDy = 1;
    tMD.restartDelayed(TASK_SECOND);
  }
  else
  {
    tB.enable();  // enable Task Error blinking
    tB.setInterval(TASK_SECOND);
  }
}

// FUNCTIONS (Tasks) ----------------------------
void connectLaserEvent() {
  if (connectLaser.available()) {
    char inChar = (char)connectLaser.read();
    if (inChar == '\x0d') {
      evalSoftSerialData();
      inSfStr = "";
    } else if (inChar != '\x0a') {
      inSfStr += inChar;
    }
  }
}

void startRFID() {
  flash_led(1);
  but_led(0);
  lcd.clear();
  dispRFID();
  Serial.print("+++");  //Starting the request of IDENT
  tM.enable();          // xBee check
}

void checkXbee() {
  if (IDENT.startsWith("MA") && plplpl == 2) {
    ++plplpl;
    tB.setCallback(retryPOR);
    tB.enable();
    digitalWrite(BUSError, LOW); // turn the LED off (Programm start)
  }
}

void retryPOR() {
  tDF.restartDelayed(TASK_SECOND * 30); // restart display light
  if (getTime < porTime * 5) {
    Serial.println(String(IDENT) + ";POR;V" + String(Version));
    ++getTime;
    tB.setInterval(TASK_SECOND * getTime);
    lcd.setCursor(0, 0); lcd.print(String(IDENT) + " ");
    lcd.setCursor(16, 1); lcd.print((getTime - porTime) * porTime);
  }
  else if (getTime == 255) {
    tM.setCallback(checkRFID);
    tM.enable();
    tB.disable();
    tV.enable();
    displayON();
  }
}

void checkRFID() {      // 500ms Tick
  tCL.disable();        // soft serial enable
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    code = 0;
    firstCLOSE = false;
    for (byte i = 0; i < mfrc522.uid.size; i++) {
      code = ((code + mfrc522.uid.uidByte[i]) * 10);
    }
    if (!digitalRead(SSR_Machine))  { // Check if machine is switched on
      flash_led(4);
      tBD.setCallback(&FlashCallback);
      tBD.restartDelayed(100);
      tDF.restartDelayed(TASK_SECOND * 30);
    }
    Serial.println("card;" + String(code));
    // Display changes
    lcd.setCursor(5, 0); lcd.print("               ");
    lcd.setCursor(0, 0); lcd.print("Card# "); lcd.print(code);
    displayON();
  }
  tCL.enable(); // soft serial enable
}

void UnLoCallback() {   // 500ms Tick
  uint8_t buttons = lcd.readButtons();
  if (timer > 0) {
    if (timer / 120 < CLOSE) { // Close to end time reached
      toggle = !toggle;
      if (toggle)  { // toggle GREEN Button LED
        but_led(1);
        flash_led(1);
      } else  {
        but_led(3);
        flash_led(4);
      }
      if (!firstCLOSE)
      {
        lcd.setCursor(0, 0);
        lcd.print("Place Tag @ Reader");
        lcd.setCursor(0, 1);
        lcd.print("to extend Time ");
        tB.disable();
        tM.enable();
        firstCLOSE = true;
      }
    }
    timer -= 1;
    minutes = timer / 120;
    if (timer % 120 == 0) {
      char tbs[8];
      sprintf(tbs, "% 3d", minutes);
      lcd.setCursor(17, 1); lcd.print(tbs);
    }
  }
  if ((timer == 0 && onTime) || buttons & BUTTON_P2) {   //  time == 0 and timed or Button
      onTime = false;
      shutdown();
  }
  if ( (buttons & BUTTON_P1) && !(prevButt & BUTTON_P1)) {
    if (isEmerg && !onTime)
      digitalWrite(SSR_Machine, !digitalRead(SSR_Machine));
  }
}

void dispVALUES() {
  if (displayIsON) {
    dispValues();
    connectLaser.println("OK");
  }
}

// Send Messages delayed
void MesaDelay()
{
  switch (mesaDy)
  {
  case 1:
    lcd.setCursor(0, 1);
    lcd.print("---> SD - OK?");
    connectLaser.println(String(LCIDENT) + ";SD");
    mesaDy = 0;
    break;

  case 2:
    connectLaser.println(String(LCIDENT) + ";OK");
    mesaDy = 0;
    break;

  case 3:
    connectLaser.println(String(LCIDENT) + ";EM");
    mesaDy = 0;
    break;

  case 4:
    connectLaser.println("LSENA");
    mesaDy = 0;
    break;

      default:
    break;
  }
}

void BlinkCallback()
{
  // --Blink if BUS Error
  digitalWrite(BUSError, !digitalRead(BUSError));
}

void FlashCallback() {
  flash_led(1);
}

void DispOFF() {
  displayIsON = false;
  digitalWrite(REL_RS232, LOW);
  lcd.setBacklight(BACKLIGHToff);
  lcd.clear();
  but_led(0);
  flash_led(1);
}
// END OF TASKS ---------------------------------

// FUNCTIONS ------------------------------------
void noreg() {
  digitalWrite(SSR_Machine, LOW);
  lcd.setCursor(0, 1); lcd.print("No access, registed?");
  tM.enable();
  BadSound();
  but_led(0);
  flash_led(1);
}

void OnTimed(long min) {   // Turn on machine for nnn minutes
  onTime = true;
  timer = timer + min * 120;
  Serial.println(String(IDENT) + ";ont");
  char tbs[8];
  sprintf(tbs, "% 3d", timer / 120);
  lcd.setCursor(14, 1); lcd.print("-->"); lcd.print(tbs);
  granted();
}

void OnPerm(void)  {    // Turn on machine permanently (VIP-Users only)
  onTime = false;
  Serial.println(String(IDENT) + ";onp");
  lcd.setCursor(14, 1); lcd.print("->Perm");
  granted();
}

// Tag registered
void granted()  {
  tM.disable();
  tDF.disable();
  tU.enable();
  but_led(3);
  flash_led(1);
  GoodSound();
  digitalWrite(SSR_Machine, HIGH);
  mesaDy = 4; // LAS.ENA
  tMD.restartDelayed(TASK_SECOND / 2);
  checkMesa();
}

// Switch off machine and stop
void shutdown(void) {
  tU.disable();
  digitalWrite(SSR_Machine, LOW);
  timer = 0;
  but_led(2);
  Serial.println(String(IDENT) + ";off");
  tDF.restartDelayed(TASK_SECOND * 30);
  BadSound();
  flash_led(1);
  connectLaser.println("LSDIS");
  lcd.setCursor(0, 0);
  lcd.print("System shut down at");
  tB.disable();
  tM.enable(); // added by DieterH on 18.10.2017
}

void checkMesa()
{
  if (isPower)
  {
    tB.setCallback(blinkMESA);
    if (isEmerg)
    {
      but_led(2);
      mesaBg = 1;
      tB.setInterval(TASK_SECOND / 4);
      tB.enable();
    }

    if (isCover)
    {
      mesaBg = 1;
      tB.setInterval(TASK_SECOND / 2);
      tB.enable();
    }
  }
}

void blinkMESA()
{
  switch (mesaBg)
  {
  case 1:
    flashB = !flashB;
    if (flashB)
    { // toggle BLUE Button LED
      but_led(6);
    }
    else
    {
      but_led(5);
    }
    break;
  case 2:
    flashB = !flashB;
    if (flashB)
    { // toggle bl + rt Button LED
      but_led(6);
      but_led(2);
    }
    else
    {
      but_led(5);
      but_led(1);
    }
    break;

  default:
    break;
  }
}

void but_led(int var)
{
  switch (var)
  {
  case 0: // LEDs off
    lcd.pinLEDs(StopLEDrt, HIGH);
    lcd.pinLEDs(StopLEDgn, HIGH);
    lcd.pinLEDs(LCPOWUPLed, LOW);
    break;
  case 1: // LED rt & gn off
    lcd.pinLEDs(StopLEDrt, HIGH);
    lcd.pinLEDs(StopLEDgn, HIGH);
    break;
  case 2: // RED LED on
    lcd.pinLEDs(StopLEDrt, LOW);
    lcd.pinLEDs(StopLEDgn, HIGH);
    break;
  case 3: // GREEN LED on
    lcd.pinLEDs(StopLEDrt, HIGH);
    lcd.pinLEDs(StopLEDgn, LOW);
    break;
  case 4: // GREEN & RED LED on (new button only)
    lcd.pinLEDs(StopLEDrt, LOW);
    lcd.pinLEDs(StopLEDgn, LOW);
    break;
  case 5: // BLUE LED off
    lcd.pinLEDs(LCPOWUPLed, LOW);
    break;
  case 6: // BLUE LED on
    lcd.pinLEDs(LCPOWUPLed, HIGH);
    break;
  default:
    break;
  }
}

void flash_led(int var) {
  switch (var) {
    case 1:   // LEDs off
      lcd.pinLEDs(FlashLED_A, LOW);
      lcd.pinLEDs(FlashLED_B, LOW);
      break;
    case 2:
      lcd.pinLEDs(FlashLED_A, HIGH);
      lcd.pinLEDs(FlashLED_B, LOW);
      break;
    case 3:
      lcd.pinLEDs(FlashLED_A, LOW);
      lcd.pinLEDs(FlashLED_B, HIGH);
      break;
    case 4:
      lcd.pinLEDs(FlashLED_A, HIGH);
      lcd.pinLEDs(FlashLED_B, HIGH);
      break;
  }
}

void BuzzerOff()  {
  lcd.pinLEDs(buzzerPin, LOW);
  tBU.setCallback(&BuzzerOn);
}

void BuzzerOn()  {
  lcd.pinLEDs(buzzerPin, HIGH);
  tBU.setCallback(&BuzzerOff);
}

void BadSound(void) {   // added by DieterH on 22.10.2017
  tBU.setInterval(100);
  tBU.setIterations(6); // I think it must be Beeps * 2?
  tBU.setCallback(&BuzzerOn);
  tBU.enable();
}

void GoodSound(void) {
  lcd.pinLEDs(buzzerPin, HIGH);
  tBD.setCallback(&BuzzerOff);  // changed by DieterH on 18.10.2017
  tBD.restartDelayed(200);      // changed by DieterH on 18.10.2017
}

//  RFID ------------------------------
void dispRFID(void) {
  lcd.print("Sys  V" + String(Version).substring(0,3) + " starts at:");
  lcd.setCursor(0, 1); lcd.print("Wait Sync xBee:");
}

void displayON() {
  displayIsON = true;
  digitalWrite(REL_RS232, HIGH);
  lcd.setBacklight(BACKLIGHTon);
  tM.enable();
  lcd.setCursor(0, 2);
  lcd.print("VL=00.0\337C  RL=00.0\337C");
  lcd.setCursor(0,3);
  lcd.print("Flow=00.0l/m Laser? ");
  dispValues();
  mesaDy = 2; // OK
  tMD.restartDelayed(TASK_SECOND / 5);
}

void dispValues(void) {
  if (displayIsON) {
    lcd.setCursor(3,2); lcd.print(txtempV);
    lcd.setCursor(14,2); lcd.print(txtempR);
    lcd.setCursor(5,3); lcd.print(txtflow);
    lcd.setCursor(13, 3); lcd.print(txLaser);
  }
}
// End Funktions --------------------------------

// Funktions Serial Input (Event) ---------------
void evalSerialData() {
  inStr.toUpperCase();

  if (inStr.startsWith("OK")) {
    if (plplpl == 0) {
      ++plplpl;
      Serial.println("ATNI");
    } else {
      ++plplpl;
    }
  }

  if (inStr.startsWith("MA") && inStr.length() == 4) {
    Serial.println("ATCN");
    IDENT = inStr;
  }

  if (inStr.startsWith("TIME")) {
    inStr.concat("                   ");     // add blanks to string
    lcd.setCursor(0, 1); lcd.print(inStr.substring(4,24));
    tB.setInterval(TASK_SECOND / 2);
    getTime = 255;
  }

  if (inStr.startsWith("ONT") && inStr.length() >=4) {
    val = inStr.substring(3).toInt();
    OnTimed(val);
  }

  if (inStr.startsWith("ONP") && inStr.length() ==3) {
    OnPerm();
  }

  if (inStr.startsWith("OFF") && inStr.length() ==3) {
    shutdown(); // Turn OFF Machine
  }

  if (inStr.startsWith("NOREG")) {
    noreg();  // changed by D. Haude on 18.10.2017
  }

  if (inStr.startsWith("SETCE")) { // set time before ClosE machine
    CLOSE = inStr.substring(5).toInt();
  }

  if (inStr.startsWith("SETCN")) { // set time for longer CleaN on
    CLEAN = inStr.substring(5).toInt();
  }


  if (inStr.startsWith("SETCL")) { // set Current Level for switching on and off
    CURLEV = inStr.substring(5).toInt();
  }

  if (inStr.startsWith("DISON") && !digitalRead(SSR_Machine)) { // Switch display on for 60 sec
    displayON();
    tDF.restartDelayed(TASK_SECOND * 60);
  }

  if (inStr.substring(0, 3) == "R3T" && inStr.length() >3) {  // print to LCD row 3
    inStr.concat("                   ");     // add blanks to string
    lcd.setCursor(0,2);
    lcd.print(inStr.substring(3,23)); // cut string lenght to 20 char
  }

  if (inStr.substring(0, 3) == "R4T" && inStr.length() >3) {  // print to LCD row 4
    inStr.concat("                   ");     // add blanks to string
    lcd.setCursor(0,3);
    lcd.print(inStr.substring(3,23));   // cut string lenght to 20 char  changed by MM 10.01.2018
  }
}

void evalSoftSerialData() {
  inSfStr.toUpperCase();

  if (inSfStr.startsWith("LACU;") && inSfStr.length() == 8)
  {
    if (inSfStr.endsWith("POR"))
    {
      mesaDy = 2; // OK
      tMD.restartDelayed(TASK_SECOND);
      startRFID();
    }

    if (inSfStr.endsWith("ERR"))
    {
      mesaDy = 3; // EM
      tMD.restartDelayed(TASK_SECOND);
    }
  }

  if (inSfStr.startsWith("ER0;") && inSfStr.length() == 24)
  {
    lcd.setCursor(0,2);
    lcd.print(inSfStr.substring(4));
  }

  if (inSfStr.startsWith("ER1;") && inSfStr.length() == 24)
  {
    lcd.setCursor(0, 3);
    lcd.print(inSfStr.substring(4));
  }

  if (inSfStr.startsWith("MSP;") && inSfStr.length() == 5)
  {
    if (inSfStr.endsWith("1"))
    {
      isPower = true;
      checkMesa();
    }
    else if (inSfStr.endsWith("0"))
    {
      isPower = false;
    }
    lcd.pinLEDs(LCPOWUPLed, isPower);
  }

  if (inSfStr.startsWith("MSL;") && inSfStr.length() == 5)
  {
    if (inSfStr.endsWith("1"))
    {
      isLaser = true;
      txLaser = "LAS.ENA";
      lcd.setCursor(13, 3);
      lcd.print(txLaser);
    }
    else if(inSfStr.endsWith("0"))
    {
      isLaser = false;
      txLaser = "LAS.DIS";
      lcd.setCursor(13, 3);
      lcd.print(txLaser);
    }
  }

  if (inSfStr.startsWith("MSS;") && inSfStr.length() == 5)
  {
    if (inSfStr.endsWith("1"))
    {
      isEmerg = true;
      but_led(2);
      mesaBg = 1;
      tB.setCallback(blinkMESA);
      tB.setInterval(TASK_SECOND / 4);
      tB.enable();
    }
    else if (inSfStr.endsWith("0"))
    {
      isEmerg = false;
      tB.disable();
      if (isPower)
      {
        but_led(3);
        but_led(6);
      }
      else
      {
        but_led(0);
      }
    }
  }

  if (inSfStr.startsWith("MSE;") && inSfStr.length() == 5)
  {
    if (inSfStr.endsWith("1"))
    {
      isTmpSn = true;
      mesaBg = 2;
      tB.setCallback(blinkMESA);
      tB.setInterval(TASK_SECOND / 6);
      tB.enable();
    }
    else if (inSfStr.endsWith("0"))
    {
      isTmpSn = false;
      tB.disable();
      if (isPower)
      {
        but_led(3);
        but_led(6);
      }
      else
      {
        but_led(0);
      }
    }
  }

  if (inSfStr.startsWith("MSC;") && inSfStr.length() == 5)
  {
    if (inSfStr.endsWith("1"))
    {
      isCover = true;
      if (isPower)
      {
        mesaBg = 1;
        tB.setCallback(blinkMESA);
        tB.setInterval(TASK_SECOND / 2);
        tB.enable();
      }
    }
    else if (inSfStr.endsWith("0"))
    {
      isCover = false;
      tB.disable();
      if (isPower)
        but_led(6);
    }
  }

  if (inSfStr.startsWith("SDV;") && displayIsON && inSfStr.length() == 8)
  {
    txtempV = inSfStr.substring(4, 8);
  }

  if (inSfStr.startsWith("SDR;") && displayIsON && inSfStr.length() == 8)
  {
    txtempR = inSfStr.substring(4, 8);
  }

  if (inSfStr.startsWith("SDF;") && displayIsON && inSfStr.length() == 8)
  {
    txtflow = inSfStr.substring(4, 8);
  }
}

/* SerialEvent occurs whenever a new data comes in the
  hardware serial RX.  This routine is run between each
  time loop() runs, so using delay inside loop can delay
  response.  Multiple bytes of data may be available.
*/
void serialEvent() {
  char inChar = (char)Serial.read();
  if (inChar == '\x0d') {
    evalSerialData();
    inStr = "";
  } else if (inChar != '\x0a') {
    inStr += inChar;
  }
}
// End Funktions Serial Input -------------------

// PROGRAM LOOP AREA ----------------------------
void loop() {
  runner.execute();
}
