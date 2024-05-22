/*  AZ/EL Antenna Rotator controller for Arduino - DC motors - PWM output
 *  =====================================================================
 *  Uses EasyComm protocol for tracking software
 *  Manual command via two rotary encoders AZ - EL
 *  
 *  based on code by Viorel, YO3RAK
 *  https://www.hackster.io/viorelracoviteanu/antenna-rotator-controller-compatible-with-tracking-software-48f9cd
 *  
 *  modified to suit my needs (Yaesu G-5500DC Az/El Rotator)
 *  v1.0 - initial version
 *  v1.1 - misc. code cleanups
 *  v2.0 - replaced relay and Mosfet boards with L298N board
 */
 
#include <Wire.h> // Library for I2C communication
#include <LiquidCrystal_I2C.h> // https://www.arduinolibraries.info/libraries/liquid-crystal-i2-c (Library for LCD)
// Wiring: SDA pin is connected to A4 and SCL pin to A5.
// Connect to LCD via I2C, default address 0x27 (A0-A2 not jumpered)
LiquidCrystal_I2C lcd(0x27, 16, 2); // address, chars, rows.

// declaring custom symbol for up/down arrow
 byte DownArrow[8] = {
  B00000,
  B00100,
  B00100,
  B00100,
  B10101,
  B01110,
  B00100,
  B00000
};
 byte UpArrow[8] = {
  B00000,
  B00100,
  B01110,
  B10101,
  B00100,
  B00100,
  B00100,
  B00000
};

/***********************************THIS IS WHERE YOU REALY TWEAK THE ANTENNA MOVEMENT***************/
// ANTENNA potentiometers CALIBRATION
  int AzMin = 11;       //begining of the potentiometer
  int AzMax = 740;      //end of the potentiometer 
  int ElMin = 18;
  int ElMax = 505;

// Allowed error for which antennna won't move
  int AzErr = 2;
  int ElErr = 2;

// Angle difference where soft stop begins
  int Amax = 15;        //azimuth
  int Emax = 15;        //elevation

// min and max power for motors, percents;
  int PwAzMin = 40;     //minimum power for which the motor doesn't stall and starts under load
  int PwAzMax = 90;     //full power for the fastest speed
  int PwElMin = 40;
  int PwElMax = 90;

/***************************************************************************************************/

  enum PinAssignments {
  AzPotPin = A0,        // input pin for the azim. potentiometer
  ElPotPin = A1,        // input pin for the elev. potentiometer    
  AzEncoderPinA = 2,    // Az encoder right
  AzEncoderPinB = 4,    // Az encoder left
  AzClearButton = 5,    // Az encoder push
  ElEncoderPinA = 3,    // El encoder right
  ElEncoderPinB = 6,    // El encoder left
  ElClearButton = 7,    // El encoder push
  ElPWMPin = 10,        // out pin for elevation PWM command L298 board
  ElRotPin1 = 8,        // out pin for El H-bridge direction L298 board
  ElRotPin2 = 9,        // out pin for El H-bridge direction L298 board
  AzRotPin1 = 12,       // out pin for Az H-bridge direction L298 board
  AzRotPin2 = 13,       // out pin for Az H-bridge direction L298 board
  AzPWMPin = 11,        // out pin for azimuth PWM command L298 board
    };
  
// Az encoder variables
  int AzEncBut = 1;                   // variable to toggle with encoder push button
  volatile boolean AzEncRot;          // if rotation occured
  volatile boolean AzEncUp;           // direction of rotation
  
// El encoder variables
  int ElEncBut = 1;
  volatile boolean ElEncRot;
  volatile boolean ElEncUp;
  
// movement variables
  int TruAzim = 0;      // calculated real azimuth value
  int ComAzim = 0;      // commanded azimuth value
  int TruElev = 0;      // calculated real elevation value
  int ComElev = 0;      // commanded elevation value
  int OldTruAzim = 0;   // to store previous azimuth value
  int OldComAzim = 0;
  int OldTruElev = 0;   // to store previous elevation value
  int OldComElev = 0;
  int StaAzim = 0;      // Start Azimuth angle for motor Soft-Start
  int StaElev = 0;      // Start Elevation angle for motor Soft-Start
  int PwAzStop = 0;     // calculated PWM (percent) for soft-stop
  int PwAzStar = 0;     // calculated PWM (percent) for soft-start
  int PwElStop = 0;     // calculated PWM (percent) for soft-stop
  int PwElStar = 0;     // calculated PWM (percent) for soft-start
  int PwAz = 0;         //calculated power to be transmitted to motor (percents);
  int PwEl = 0;
  char AzDir;           // symbol for azim rot display
  char ElDir;           // symbol for elev. rot display
  unsigned long NowTime;         // store the current millis() for timing purposes
    
// flags for AZ, EL tolerances
  bool AzStop = false;
  bool ElStop = false;
  int ElUp = 1;                  // 1 - Elevation Dn, 0 - Elevation STOP, 2 - Elevation Up
  
//averaging loop
  const int numReadings = 25;
  int readIndex = 0;             // the index of the current reading  
  int azimuth[numReadings];      // the readings from the analog input
  int elevation[numReadings];
  int totalAz = 0;               // the running total
  int totalEl = 0;

// variables for serial comm
  String Azimuth = "";
  String Elevation = "";
  String ComputerRead;
  String ComputerWrite;
  bool AZser = false;
  bool ELser = false;
  bool ANTser = false;

/*************** END VARIABLE DECLARATION  ************/

void setup() {
  Serial.begin(19200);
  Serial.setTimeout(50);           // miliseconds to wait for USB sata. Default 1000
  lcd.init();
  lcd.backlight();

// write on display name and version
  lcd.setCursor(0, 0);           // Set the cursor on the first column first row.(counting starts at 0!)
  lcd.print("EasyCom AntRotor"); // display "..."
  lcd.setCursor(0, 1);           // Set the cursor on the first column the second row
  lcd.print("DC1RDB   v2.0");
  delay(2000);

//creating custom symbol for up/dwn arrow
  lcd.createChar(1, DownArrow);
  lcd.createChar(2, UpArrow);
  
// pin declaration
  pinMode(AzRotPin1, OUTPUT);
  pinMode(AzRotPin2, OUTPUT);
  pinMode(AzPWMPin, OUTPUT);
  pinMode(ElRotPin1, OUTPUT);
  pinMode(ElRotPin2, OUTPUT);
  pinMode(ElPWMPin, OUTPUT);
  pinMode(AzPotPin, INPUT);
  pinMode(ElPotPin, INPUT);
  pinMode(AzEncoderPinA, INPUT);
  pinMode(AzEncoderPinB, INPUT);
  pinMode(AzClearButton, INPUT);
  pinMode(ElEncoderPinA, INPUT);
  pinMode(ElEncoderPinB, INPUT);

// Turn off motors - initial state
	digitalWrite(AzRotPin1, LOW);
	digitalWrite(AzRotPin2, LOW);
	digitalWrite(ElRotPin1, LOW);
	digitalWrite(ElRotPin2, LOW);

// Interrupt Service Routine for Az and El encoder
  attachInterrupt(0, doAzEnc, CHANGE);                               // Az encoder
  attachInterrupt(1, doElEnc, CHANGE);                               // El Encoder

/* initialization of the averaging loop
   this is to set Az/El-command the same value as real, not to jerk the antenna at start-up */
  TruAzim = (map(analogRead(AzPotPin), AzMin, AzMax, 0, 359));       // transforms potentiometer voltage into azimuth angle
  TruElev = (map(analogRead(ElPotPin), ElMin, ElMax, 0, 90));        // transforms potentiometer voltage into elevation angle

  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    azimuth[thisReading] = TruAzim;
    elevation[thisReading] = TruElev;
  }
  totalAz = TruAzim * numReadings;
  totalEl = TruElev * numReadings;

/* keep command values in range  */
  ComAzim = constrain(TruAzim, 0, 359);
  ComElev = constrain(TruElev, 0, 90);
  OldTruAzim = TruAzim;
  OldComAzim = ComAzim;
  OldTruElev = TruElev;
  OldComElev = TruElev;

  delay(1500);                     // keep for 1.5 seconds
// display Azim. and Elev. values
  lcd.setCursor(0, 0);
  lcd.print("AZ ---" + String(char(223)) + "=cmd ---" + String(char(223)));  // char(223) is degree symbol
  lcd.setCursor(0, 1); 
  lcd.print("EL  --" + String(char(223)) + "=cmd  --" + String(char(223)));
  DisplValue(TruAzim, 3,0);
  DisplValue(ComAzim,12,0);
  DisplValue(TruElev, 3,1);
  DisplValue(ComElev,12,1);
}
// end SETUP

void loop() {
/************** FYI, this loop repeats 500 times per second !!! **************/
// AZIMUTH/ELEVATION AVERAGING LOOP
  // subtract the oldest value
  totalAz = totalAz - azimuth[readIndex];
  totalEl = totalEl - elevation[readIndex];
  // read from the sensor:
  azimuth[readIndex] = (map(analogRead(AzPotPin), AzMin, AzMax, 0, 359));
  elevation[readIndex] = (map(analogRead(ElPotPin), ElMin, ElMax, 0, 90));
  // add the reading to the total:
  totalAz = totalAz + azimuth[readIndex];
  totalEl = totalEl + elevation[readIndex];
  // do the average
  TruAzim = totalAz / numReadings;
  TruElev = totalEl / numReadings;
/*  keep values in range
  TruAzim = constrain(TruAzim, 0, 359);
  TruElev = constrain(TruElev, 0, 90); */
  // advance to the next position in the array
  // if we're at the end of the array, wrap around to the beginning:
  readIndex = (readIndex + 1) % numReadings;

// read the command from encoder
  ReadAzimEncoder();
  ReadElevEncoder();
  
  if (Serial.available()) {SerComm();}          // read USB data

// update antenna position display only if value change
  if ((millis() - NowTime) > 300){              //every 0.3 seconds, not to flicker the display
    if (OldTruAzim!=TruAzim) {
      DisplValue(TruAzim,3,0);
      OldTruAzim = TruAzim;
    }
    if (OldTruElev!=TruElev) {
      DisplValue(TruElev,3,1);
      OldTruElev = TruElev;
    }
    NowTime = millis();
  }

// update target position display only if value change
  if (OldComAzim != ComAzim) {
    DisplValue(ComAzim,12,0);
    OldComAzim = ComAzim;
  }
  if (OldComElev != ComElev) {
    DisplValue(ComElev,12,1);
    OldComElev = ComElev;
  }

// this is to rotate in azimuth
  if (TruAzim == ComAzim) {                   // if equal, stop moving
    AzStop = true;
    analogWrite(AzPWMPin, 0);                 // Az motor power = 0
    StaAzim = TruAzim;                        // this will be the start azimuth for soft-start
    lcd.setCursor(7, 0);
    lcd.print("=");
  }
    else if ((abs(TruAzim - ComAzim)<=AzErr)&&(AzStop == false)) {  // if in tolerance, but it wasn't an equal, rotate
      AzimRotate();}
    else if (abs(TruAzim - ComAzim)>AzErr){   // if target is off tolerance
      AzStop = false;                         // it's not equal
      AzimRotate();                           // rotate
    }

// this is to rotate in elevation
  if (TruElev == ComElev) {                   // if equal, stop moving
    ElStop = true;
    analogWrite(ElPWMPin, 0);                 // El motor power = 0
    StaElev = TruElev;                        // this will be the start elevation for soft-start
    lcd.setCursor(7, 1);
    lcd.print("=");
    ElUp = 0;                                 // flag for elevation STOP
  }
  else if ((abs(TruElev - ComElev)<=ElErr)&&(ElStop == false)) {  // if in tolerance, but it wasn't an equal, rotate
    ElevRotate();}
  else if (abs(TruElev - ComElev)>ElErr){     // if target is off tolerance
    ElStop = false;                           // it's not equal
    ElevRotate();                             // rotate
  }
  
  // this is to interpret Az encoder x10 multiplication
  while (AzEncBut == 10) {                    // while toggled to x10
    analogWrite(AzPWMPin, 0);                 // STOP antenna rotation
    StaAzim = TruAzim;                        // this will be the start azimuth for soft-start
    analogWrite(ElPWMPin, 0);
    lcd.setCursor(7, 0);
    lcd.print("*");
    ReadAzimEncoder();
    if (OldComAzim != ComAzim){               // update display only if numbers change
      DisplValue(ComAzim, 12, 0);
      OldComAzim = ComAzim;
    }
    delay (100);
  }
}  // end main LOOP

//____________________________________________________
// ___________procedures definitions__________________

void DisplValue(int x, int y, int z) {
  char displayString[7] = "";
  sprintf(displayString, "%3d", x);           // outputs a fixed lenght number (3 integer)
  lcd.setCursor(y, z);                        // for leading zeros '007' use "%03d"
  lcd.print(displayString);
  
// ************** FOR CALIBRATION PURPOSES **************

  // Serial.print ("Az ");
  // Serial.println (analogRead(AzPotPin));
  // Serial.print ("El ");
  // Serial.println (analogRead(ElPotPin));

}  // end DisplValue()

void ReadAzimEncoder() {
  if (digitalRead(AzClearButton) == LOW )  {      // if encoder switch depressed
    delay (250);                                  // debounce switch
    lcd.setCursor(0, 0);                          // refresh all the writings on the screen
    lcd.print("AZ ---" + String(char(223)) + "=cmd ---" + String(char(223)));
    lcd.setCursor(0, 1); 
    lcd.print("EL  --" + String(char(223)) + "=cmd  --" + String(char(223)));
    DisplValue(TruAzim, 3,0);
    DisplValue(ComAzim,12,0);
    DisplValue(TruElev, 3,1);
    DisplValue(ComElev,12,1);
    if (AzEncBut == 1){
      AzEncBut = 10;                              // increment by 10 degrees
      ComAzim = int(ComAzim/10)*10;               // ComAzim in 10deg. steps 
    }
    else
      AzEncBut = 1;
  }

  if (AzEncRot){
    delay(20);                                    // debouncing
    if (AzEncUp)
      ComAzim += AzEncBut;
    else
      ComAzim -= AzEncBut;
    ComAzim = ((ComAzim + 360) % 360);            // Az Cmd between 0 and 359 deg continuous
    AzEncRot = false;
  }
} //end ReadAzimEncoder()

void ReadElevEncoder() {
  if (digitalRead(ElClearButton) == LOW )  {      // set Park Position Command Az/El
    delay(250);                                   // debounce switch
    ComAzim = 260;
    ComElev = 0;
  }

  if (ElEncRot){
    delay(20);                                    // debouncing
    if (ElEncUp)
      ComElev ++;
    else
      ComElev --;
    ComElev = constrain(ComElev, 0, 90);          // keep El Cmd value in range
    ElEncRot = false;
  }
}  // end of ReadElevEncoder()

// Interrupt Service Routine for a change to encoder pin A and B
void doAzEnc ()
{
  if (digitalRead (AzEncoderPinA))
    AzEncUp = digitalRead (AzEncoderPinB);
  else
    AzEncUp = !digitalRead (AzEncoderPinB);
  AzEncRot = true;
}  // end of doAzEnc

void doElEnc ()
{
  if (digitalRead (ElEncoderPinA))
    ElEncUp = digitalRead (ElEncoderPinB);
  else
    ElEncUp = !digitalRead (ElEncoderPinB);
    ElEncRot = true;
}  // end of doElEnc

void AzimRotate() {                                   // cold switching - stop motor before changing direction
    if (ComAzim > TruAzim) {                          // this to determine direction of rotation
        if (AzDir == char(127)) {                     // if previously rotating in the oposite direction
          analogWrite(AzPWMPin, 0);                   // STOP the motor
          StaAzim = TruAzim;                          // this will be the start azimuth for soft-start
          delay(200);                                 // pre-switch delay
          digitalWrite(AzRotPin1, LOW);               // set H-bridge to rotate right
          digitalWrite(AzRotPin2, HIGH);              // set H-bridge to rotate right
          delay(200);                                 // post-switch delay
        }
        else {                                        // same direction, no stop, no delay
          digitalWrite(AzRotPin1, LOW);               // set H-bridge to rotate right
          digitalWrite(AzRotPin2, HIGH);              // set H-bridge to rotate right
        }
          AzDir = char(126);                          // "->"
    }
      else {
        if (AzDir == char(126)) {                     // if previously rotating in the oposite direction
          analogWrite(AzPWMPin, 0);                   // STOP the motor
          StaAzim = TruAzim;                          // this will be the start azimuth for soft-start
          delay(200);                                 // pre-switch delay
          digitalWrite(AzRotPin1, HIGH);              // set H-bridge to rotate left
          digitalWrite(AzRotPin2, LOW);               // set H-bridge to rotate left
          delay(200);                                 // post-switch delay
        }
        else {                                        // same directin, no Stop, no delay
          digitalWrite(AzRotPin1, HIGH);              // set H-bridge to rotate left
          digitalWrite(AzRotPin2, LOW);               // set H-bridge to rotate left
        }
        AzDir = char(127);                            // "<-"
      }
    lcd.setCursor(7, 0);
    lcd.print(String(AzDir));
 // this activates azim PWM pin proportional with angle error (calculated in percents %)
    PwAzStop = PwAzMin + round((abs(ComAzim-TruAzim))*(PwAzMax-PwAzMin)/Amax);   //formula which outputs a power proportional with angle difference for Soft-Stop
    PwAzStar = PwAzMin + round((abs(StaAzim-TruAzim))*(PwAzMax-PwAzMin)/Amax);   //formula which outputs a power proportional with angle difference for Soft-Start
    if (PwAzStar > PwAzStop){
         PwAz = PwAzStop;                             //choose whichever value is smallest
      }
      else {PwAz = PwAzStar;}
    if (PwAz > PwAzMax) {PwAz = PwAzMax;}
    analogWrite(AzPWMPin, round(2.55*PwAz));          // activate Azim drive PWM pin 
}  // end AzimRotate()

void ElevRotate() {                                   // cold switching - stop motor before changing direction
    if (ComElev > TruElev) {                          // this to determine direction of rotation
      if (ElUp == 1) {                                // if previously rotating in the oposite direction
        analogWrite(ElPWMPin, 0);                     // STOP the motor
        StaElev = TruElev;                            // this will be the start elevation for soft-start
        delay(200);                                   // pre-switch delay
        digitalWrite(ElRotPin1, LOW);                 // set H-bridge to rotate up
        digitalWrite(ElRotPin2, HIGH);                // set H-bridge to rotate up
        delay(200);                                   // post-switch delay
      }
      else {                                          // same directin, no Stop, no delay
        digitalWrite(ElRotPin1, LOW);                 // set H-bridge to rotate up
        digitalWrite(ElRotPin2, HIGH);                // set H-bridge to rotate up
      }
      lcd.setCursor(7, 1);
      lcd.write(2);                                   // arrow up
      ElUp = 2;                                       // flag for elevation UP
    }
     else {
      if (ElUp == 2) {                                // if previously rotating in the oposite direction
        analogWrite(ElPWMPin, 0);                     // STOP the motor
        StaElev = TruElev;                            // this will be the start elevation for soft-start
        delay(200);                                   // pre-switch delay
        digitalWrite(ElRotPin1, HIGH);                // set H-bridge to rotate down
        digitalWrite(ElRotPin2, LOW);                 // set H-bridge to rotate down
        delay(200);                                   // post-switch delay
      }
      else {                                          // same directin, no Stop, no delay
        digitalWrite(ElRotPin1, HIGH);                // set H-bridge to rotate down
        digitalWrite(ElRotPin2, LOW);                 // set H-bridge to rotate down
      }
        lcd.setCursor(7, 1);
        lcd.write(1);                                 // arrow down
        ElUp = 1;                                     // flag for elevation DN
    }
 // this activates azim PWM pin proportional with angle error (calculated in percents %)
    PwElStop = PwElMin + round((abs(ComElev-TruElev))*(PwElMax-PwElMin)/Emax);   //formula which outputs a power proportional with angle difference for Soft-Stop
    PwElStar = PwElMin + round((abs(StaElev-TruElev))*(PwElMax-PwElMin)/Emax);   //formula which outputs a power proportional with angle difference for Soft-Start
    if (PwElStar > PwElStop){
         PwEl = PwElStop;                             //choose whichever value is smallest
      }
      else {PwEl = PwElStar;}
    if (PwEl > PwElMax) {PwEl = PwElMax;}
    analogWrite(ElPWMPin, round(2.55*PwEl));          // activate Elev drive PWM pin
}  // end ElevRotate()

void SerComm() {
  // initialize readings
  ComputerRead = "";
  Azimuth = "";
  Elevation = "";

  while(Serial.available()) {
    ComputerRead= Serial.readString();                // read the incoming data as string
//    Serial.println(ComputerRead);                   // echo the reception for testing purposes
  }
  
// looking for command <AZxxx.x>
    for (int i = 0; i <= ComputerRead.length(); i++) {
     if ((ComputerRead.charAt(i) == 'A')&&(ComputerRead.charAt(i+1) == 'Z')){ // if read AZ
      for (int j = i+2; j <= ComputerRead.length(); j++) {
        if (isDigit(ComputerRead.charAt(j))) {                                // if the character is number
          Azimuth = Azimuth + ComputerRead.charAt(j);
        }
        else {break;}
      }
     }
    }
    
// looking for command <ELxxx.x>
    for (int i = 0; i <= (ComputerRead.length()-2); i++) {
      if ((ComputerRead.charAt(i) == 'E')&&(ComputerRead.charAt(i+1) == 'L')){ // if read EL
        if ((ComputerRead.charAt(i+2)) == '-') {
          ComElev = 0;                  // if elevation negative
          break;
        }
        for (int j = i+2; j <= ComputerRead.length(); j++) {
          if (isDigit(ComputerRead.charAt(j))) {                               // if the character is number
            Elevation = Elevation + ComputerRead.charAt(j);
          }
          else {break;}
        }
      }
    }
    
// if <AZxx> received
    if (Azimuth != ""){
      ComAzim = Azimuth.toInt();
      ComAzim = ComAzim%360;          // keeping values between limits(for trackers with more than 360 deg. rotation)
      }

// if <ELxx> received
    if (Elevation != ""){
      ComElev = Elevation.toInt();
      if (ComElev>180) { ComElev = 0;}
      if (ComElev>90) {               //if received more than 90deg. (for trackers with 180deg. elevation)
        ComElev = 180-ComElev;        //keep below 90deg.
        ComAzim = (ComAzim+180)%360;  //and rotate the antenna on the back
      }
    }

// looking for <AZ EL> interrogation for antenna position
  for (int i = 0; i <= (ComputerRead.length()-4); i++) {
    if ((ComputerRead.charAt(i) == 'A')&&(ComputerRead.charAt(i+1) == 'Z')&&(ComputerRead.charAt(i+3) == 'E')&&(ComputerRead.charAt(i+4) == 'L')){
    // send back the antenna position <+xxx.x xx.x>
      ComputerWrite = "+"+String(TruAzim)+".0 "+String(TruElev)+".0";
    //ComputerWrite = "AZ"+String(TruAzim)+".0 EL"+String(TruElev)+".0"; //that's for Gpredict and HamLib
      Serial.println(ComputerWrite);
    }
  }
}  // end SerComm()
