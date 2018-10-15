/* DESCRIPTION
  ====================
  started on 01JUN2017 - uploaded on 06.06.2017 by Dieter
  Code for machine and cleaner control over RFID
  reading IDENT from xBee, retrait sending ...POR until time responds

  Commands to Raspi
  'card;nn...' - uid_2 from reader
  'POR'        - machine power on reset
  'MA01;on'    - machine reporting ON-Status
  'MA01;off'   - machine reporting OFF-Status

  Commands from Raspi
  'onp'   - Machine permanent ON
  'ontxx' - Machine xxx minutes ON
  'off'   - Machine OFF
  'time'  - format time33.33.33 33:33:33
  'noreg' - uid_2 not registered
  'setce' - set time before ClosE machine
  'setcn' - set time for longer CleaN on
  'setcl' - set Current Level for switching on and off

  Commands to Lasercutter
  'laser;ok'- Lasercutter on?
  'lsdis'   - Laser disable
  'lsena'   - Laser enable
  'vLsOn'   - Visual Laser on
  'vLsOf'   - Visual Laser off
  'ok'      - Handshake return

  Commands from Lasercutter
  'laser;por' - Lasercutter power on reset
  's2l;12.0;34.0;67.0;xxx890x'
  's2d'     - Send 2 Display
  '00.0'    - Temperatur Vorlauf °C
  '00.0'    - Temperatur Rücklauf °C
  '00.0'    - Flowmeter value l/min
  'xxxxxxx' =:
  'las.dis' - Laser disabled
  'las.ena' - Laser enabled
  'dtt00.0' - Disabled Temperatur Tube to high
  'ok'      - Handshake return

  last change: 23.09.2018 by Michael Muehl
  changed: communication lasercutter with RFID started
*/
#define Version "6.1"

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
#define SSR_Machine A2  // SSR Machine on / off  (Machine [no used])
#define SSR_Vac     A3  // SSR Dust on / off  (Dust Collector [no used])

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
#define VLBUTTONLED  3  // LED Button Laser
// Switched High - Low - High - Low
#define StopLEDrt    4  // StopLEDrt (LED + Stop-Taster)
#define StopLEDgn    5  // StopLEDgn (LED - Stop-Taster)
// switch to HIGH Value (def .h)
// BUTTON_P1  2         // [VLBUTTON] Keyswitch
// BUTTON_P2  1         // StopSwitch
// BACKLIGHT for LCD-Display
#define BACKLIGHToff 0x0
#define BACKLIGHTon  0x1

// DEFINES
#define butTime       500 // ms Tastenabfragezeit
#define CLOSE2END      15 // MINUTES before activation is off
#define CLEANON         4 // SECONDS vac on for a time
#define SECONDS      1000 // multiplier for second
#define porTime         5 // wait seconds for sending Ident + POR

// CREATE OBJECTS
Scheduler runner;
LCDLED_BreakOUT lcd = LCDLED_BreakOUT();
MFRC522 mfrc522(SS_PIN, RST_PIN);  // Create MFRC522 instance

SoftwareSerial connectLaser(RxD, TxD);

// Callback methods prototypes
void checkXbee();       // Task for Mainfunction
void UnLoCallback();    // Task to Unlock machine
void BlinkCallback();   // Task to let LED blink - added by D. Haude 08.03.2017
void FlashCallback();   // Task to let LED blink - added by D. Haude 08.03.2017
void DispOFF();         // Task to switch display off after time
void BuzzerOn();        // added by DieterH on 22.10.2017
void OnTimed(long);
void flash_led(int);

void connectLaserEvent(); // Softserial Lasercutter

// TASKS
Task tM(butTime, TASK_FOREVER, &checkLASER);
Task tU(butTime, TASK_FOREVER, &UnLoCallback);
Task tB(5000, TASK_FOREVER, &BlinkCallback); // added M. Muehl
Task tBeeper(100, 6, &BuzzerOn);             // added by DieterH on 22.10.2017
Task tBD(1, TASK_ONCE, &FlashCallback);      // Flash Delay
Task tDF(1, TASK_ONCE, &DispOFF);            // display off

// Communication with Lasercutter
Task tV(SECONDS, TASK_FOREVER, &dispVALUES);
Task tS(10, TASK_FOREVER, &connectLaserEvent);

// VARIABLES
unsigned long val;
unsigned int timer = 0;
bool onTime = false;
int minutes = 0;
bool toggle = false;
byte getTime = porTime;
unsigned long code;
byte atqa[2];
byte atqaLen = sizeof(atqa);

uint8_t prevButt = 0;    // prevue button value
bool vl_mode = LOW;   // Visual Laser Mode false=inactive, true=active

// Variables can be set externaly: ---
// --- on timed, time before new activation
unsigned int CLOSE = CLOSE2END; // RAM cell for before activation is off
// --- for cleaning
unsigned int CLEAN = CLEANON; // RAM cell for Dust vaccu cleaner on
unsigned int CURLEV = 0;      // RAM cell for before activation is off

// Lasercutter ---------
String txtempV ="00.0";       // Temperature Vorlauf (V)
String txtempR ="00.0";       // Temperature Rücklauf (R)
String txflowR ="00.0";       // represents Liter/minute
String txLaser ="Laser?";     // Laser value: Dis-, Enable, TempK

// Serial
String inSlStr = ""; // a string to hold SeriaL incoming data
String IDENT = "";   // Machine identifier for remote access control
byte plplpl = 0;     // send +++ control AT sequenz

String inSfStr = ""; // a string to hold SoFtserial incoming data
bool setPOR = false;
bool setValues = false;
bool lcdLight = true;

// ======>  SET UP AREA <=====
void setup() {
  //init Serial
  Serial.begin(57600);  // Serial
  inSlStr.reserve(40);    // reserve for inslstr serial input
  IDENT.reserve(5);     // reserve for instr serial input
  //init Softserial
  connectLaser.begin(9600);
  inSfStr.reserve(30);    // reserve for insfstr serial input

  // initialize:
  Wire.begin();         // I2C
  lcd.begin(20,4);      // initialize the LCD
  SPI.begin();          // SPI
  mfrc522.PCD_Init();   // Init MFRC522
  mfrc522.PCD_SetAntennaGain(mfrc522.RxGain_max);

  // PIN MODES
  pinMode(BUSError, OUTPUT);
  pinMode(SSR_Machine, OUTPUT);
  pinMode(SSR_Vac, OUTPUT);

  // Set default values
  digitalWrite(BUSError, HIGH);	// turn the LED ON (init start)

  runner.init();
  runner.addTask(tM);
  runner.addTask(tU);
  runner.addTask(tB);
  runner.addTask(tBeeper);
  runner.addTask(tBD);
  runner.addTask(tDF);
  // Softserial
  runner.addTask(tS);
  // Display values
  runner.addTask(tV);

  // I2C _ Ports definition only for test if I2C is avilable
  Wire.beginTransmission(I2CPort);
  I2CTransmissionResult = Wire.endTransmission();
  if (I2CTransmissionResult == 0) {
    I2CFound++;
  }
  // I2C Bus mit slave vorhanden
  if (I2CFound != 0) {
    lcd.clear();
    lcd.pinLEDs(StopLEDrt, HIGH);
    lcd.pinLEDs(StopLEDgn, LOW);
    lcd.pinLEDs(VLBUTTONLED, LOW);
    flash_led(1);
    lcd.pinLEDs(VLBUTTONLED, LOW);
    lcd.pinLEDs(buzzerPin, LOW);
    but_led(1);
    lcd.pinLEDs(VLBUTTONLED, LOW);
    tS.enable();  // soft serial enable
    tM.enable();  // xBee check
    lcd.print("Laser ON?");
  } else {
    tB.enable();  // enable Task Error blinking
    tB.setInterval(SECONDS);
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

void checkLASER() {
  if (setPOR) {
    lcd.clear();
    dispRFID();
    Serial.print("+++");       //Starting the request of IDENT
    tM.setCallback(checkXbee); // xBee check
  }
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
  tDF.restartDelayed(30 * SECONDS); // restart display light
  if (getTime < porTime * 5) {
    Serial.println(String(IDENT) + ";POR");
    ++getTime;
    tB.setInterval(getTime * SECONDS);
    lcd.setCursor(0, 0); lcd.print(String(IDENT) + " ");
    lcd.setCursor(16, 1); lcd.print((getTime - porTime) * porTime);
  }
  else if (getTime == 255)
  {
    tM.setCallback(MainCallback);
    tM.enable();
    tB.disable();
    tV.enable();
    dispLaserON();
  }
}

void MainCallback() {   // 500ms Tick
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial())
  {
    code = 0;
    for (byte i = 0; i < mfrc522.uid.size; i++) {
      code = ((code + mfrc522.uid.uidByte[i]) * 10);
    }
    Serial.println("card;" + String(code));
    lcd.setCursor(5, 0); lcd.print("               ");
    lcd.setCursor(0, 0); lcd.print("Card# "); lcd.print(code);
    dispLaserON();
    if (!digitalRead(SSR_Machine))  { // Check if machine is switched on
      flash_led(4);
      tBD.setCallback(&FlashCallback);
      tBD.restartDelayed(100);
      tDF.restartDelayed(30 * SECONDS);
    }
    mfrc522.PICC_HaltA(); // Stop reading Michael Muehl added 17.07.18
    mfrc522.PCD_StopCrypto1();
  }
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
      lcd.setCursor(0, 0); lcd.print("Place Tag @ Reader");
      lcd.setCursor(0, 1); lcd.print("to extend Time ");
      tM.setInterval(butTime);
      tM.enable();
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
    if (vl_mode) {
      vl_mode = LOW;
      connectLaser.println("VLSOF"); //turn off only button led (1) // added by E. Terelle on 7.1.2017
      lcd.pinLEDs(VLBUTTONLED, LOW);
    } else {
      vl_mode = HIGH;
      connectLaser.println("VLSON"); //turn off only button led (1) // added by E. Terelle on 7.1.2017
      lcd.pinLEDs(VLBUTTONLED, HIGH);
    }
  }
}

void dispVALUES() {
  if (setValues) {
    dispValues();
    setValues = false;
    connectLaser.println("OK");
  }
}

void BlinkCallback() {
  // --Blink if BUS Error
  digitalWrite(BUSError, !digitalRead(BUSError));
}

void FlashCallback() {
  flash_led(1);
}

void DispOFF() {
  lcdLight = false;
  tM.setInterval(SECONDS);
  lcd.setBacklight(BACKLIGHToff);
  lcd.clear();
}
// END OF TASKS ---------------------------------

// FUNCTIONS ------------------------------------
void noreg() {
  BadSound();
  lcd.setCursor(0, 1); lcd.print("No access, registed?");
  tM.enable();
}

void OnTimed(long min)  {   // Turn on machine for nnn minutes
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
  lcd.setCursor(14, 1); lcd.print(" =Perm");
  granted();
}

// Tag registered
void granted()  {
  tDF.disable();
  but_led(3);
  GoodSound();
  tU.enable();
  flash_led(1);
  connectLaser.println("LSENA");
}

// Switch off machine and stop
void shutdown(void) {
  tU.disable();
  timer = 0;
  but_led(2);
  Serial.println(String(IDENT) + ";off");
  lcd.setCursor(0, 0); lcd.print("System shut down at");
  tDF.restartDelayed(30 * SECONDS);
  BadSound();
  flash_led(1);
  tM.enable();  // added by DieterH on 18.10.2017
  connectLaser.println("LSDIS");

}

void but_led(int var) {
  switch (var) {
    case 1:   // LEDs off
      lcd.pinLEDs(StopLEDrt, LOW);
      lcd.pinLEDs(StopLEDgn, LOW);
      break;
    case 2:   // RED LED on
      lcd.pinLEDs(StopLEDrt, HIGH);
      lcd.pinLEDs(StopLEDgn, LOW);
      break;
    case 3:   // GREEN LED on
      lcd.pinLEDs(StopLEDrt, LOW);
      lcd.pinLEDs(StopLEDgn, HIGH);
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
  tBeeper.setCallback(&BuzzerOn);
}

void BuzzerOn()  {
  lcd.pinLEDs(buzzerPin, HIGH);
  tBeeper.setCallback(&BuzzerOff);
}

void BadSound(void) {   // added by DieterH on 22.10.2017
  tBeeper.setInterval(100);
  tBeeper.setIterations(6); // I think it must be Beeps * 2?
  tBeeper.setCallback(&BuzzerOn);
  tBeeper.enable();
}

void GoodSound(void) {
  lcd.pinLEDs(buzzerPin, HIGH);
  tBD.setCallback(&BuzzerOff);  // changed by DieterH on 18.10.2017
  tBD.restartDelayed(200);      // changed by DieterH on 18.10.2017
}

//  RFID ------------------------------
void dispRFID(void) {
  lcd.print("Sys  V" + String(Version) + " starts at:");
  lcd.setCursor(0, 1); lcd.print("Wait Sync xBee:");
}

// Lasercutter ------------------------
void dispLaserON() {
  tM.setInterval(butTime);
  connectLaser.println("OK");
  lcd.setCursor(0,2); lcd.print("VL=00.0\337C  RL=00.0\337C");
  lcd.setCursor(0,3); lcd.print("Flow=00.0l/m Laser? ");
  lcdLight = true;
  lcd.setBacklight(BACKLIGHTon);
}

void dispValues(void) {
  if (lcdLight) {
    lcd.setCursor(3,2); lcd.print(txtempV);
    lcd.setCursor(14,2); lcd.print(txtempR);
    lcd.setCursor(5,3); lcd.print(txflowR);
    lcd.setCursor(13,3); lcd.print(txLaser);
  }
}
// End Funktions --------------------------------

// Funktions Serial Input (Event) ---------------
void evalSerialData() {
  inSlStr.toUpperCase();

  if (inSlStr.startsWith("OK")) {
    if (plplpl == 0) {
      ++plplpl;
      Serial.println("ATNI");
    } else {
      ++plplpl;
    }
  }

  if (inSlStr.startsWith("MA")) {
    Serial.println("ATCN");
    IDENT = inSlStr;
  }

  if (inSlStr.startsWith("ONT")) {
    val = inSlStr.substring(3).toInt();
    OnTimed(val);
    tM.disable();
  }

  if (inSlStr.startsWith("ONP")) {
    OnPerm();
    tM.disable();
  }

  if (inSlStr.startsWith("OFF")) {
    shutdown(); // Turn OFF Machine
  }

  if (inSlStr.startsWith("TIME")) {
    lcd.setCursor(0, 1); lcd.print(inSlStr.substring(4));
    tB.setInterval(500);
    getTime = 255;
  }

  if (inSlStr.startsWith("NOREG")) {
    noreg();  // changed by D. Haude on 18.10.2017
  }

  if (inSlStr.startsWith("SETCE")) { // set time before ClosE machine
    CLOSE = inSlStr.substring(5).toInt();
  }

  if (inSlStr.startsWith("SETCN")) { // set time for longer CleaN on
    CLEAN = inSlStr.substring(5).toInt();
  }


  if (inSlStr.startsWith("SETCL")) { // set Current Level for switching on and off
    CURLEV = inSlStr.substring(5).toInt();
  }
}

void evalSoftSerialData() {
  inSfStr.toUpperCase();

  if (inSfStr.startsWith("LASER")) {
    if (inSfStr.substring(6) = "POR") {
      setPOR = true;
      connectLaser.println("LASER;OK");
    }
  }

  if (inSfStr.startsWith("S2D") && inSfStr.length() == 26 && lcdLight) {
    setValues = true;
    txtempV = inSfStr.substring(4, 8);
    txtempR = inSfStr.substring(9, 13);
    txflowR = inSfStr.substring(14, 18);
    txLaser = inSfStr.substring(19);
  } else if (inSfStr.startsWith("S2D") && inSfStr.length() != 26) {
    connectLaser.println("OK");
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
    inSlStr = "";
  } else if (inChar != '\x0a') {
    inSlStr += inChar;
  }
}
// End Funktions Serial Input -------------------

// PROGRAM LOOP AREA ----------------------------
void loop() {
  runner.execute();
}
