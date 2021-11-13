/**
 * Firmware for Teensy4.0 inside the 4D 70CD screen based on DriverDisplay
 * code by Brian Kamusinga.
 * Author: Landon Reekstin
 * Contributors: Brian Kamusinga, Delwys Glokpor
 */

using namespace std;
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <genieArduino.h>
#include <canAddresses.h>
#include <string.h>
#include <math.h>
#include <map>
#include <stdlib.h> // test purposes
#include <time.h> // test purposes

/// enable CAN2.0 mode usage
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2; // can2 port
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1; // can1 port

int led = 13;
IntervalTimer timer;
//These allow for the mcu to be restarted
#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);

/****************
 * 
 * 
This are constants that do not change and are stored in flash to save ram at runtime
etch uses 33376 bytes (1%) of program storage space. Maximum is 2031616 bytes.
Global variables use 66228 bytes (12%) of dynamic memory, leaving 458060 bytes for local variables. Maximum is 524288 bytes.
 * 
 * 
 ********/
//Conversion Factors
#define MPH_MS_CONV 2.237
#define KPH_MS_CONV 3.6
#define METERS_TO_MILES_CONV 0.000621371
#define METERS_TO_KM_CONV 0.001
#define TIRE_DIAMETER_IN_METERS 0.62

int speedSelect = 0;   // 0 for MPH | 1 for KPH
int currentSelect = 0; // 0 for net | 1 for MC

//other variables
bool brakesignal = false;
bool DirectionFoward = true;
bool DirectionReverse = false;
bool MotorRESET = false;
bool leftTurn = false;
bool rightTurn = false;
// error code variables
int errorCodeStatus[45]; // 0 or 1 for t/f of error codes
int errorCodeMsg[45]; // 1-44 to write to display
// Corresponding description index with bms index
int errorValues[24] = {21, 15, 11, 21, 9, 16, 8, 4, 25, 26, 41, 27, 31, 23, 33, 35, 37, 39, 14, 13, 12, 1, 2, 3};

#define DUAL_MOTOR true
boolean bShowCells = false;

// for 4D display screen
#define CUR_MULT 0.1

//Variables for VFM
unsigned int VFM_GEAR_INDICATOR[8] = {0x00, 0x0E, 0x1C, 0x2A, 0x38, 0x46, 0x54, 0x64}; //percentage out
//VFM errors
String VFM_ERRORS[] = {"Sensor Error", "Power Error", "Motor Error"};

/*class ErrorValues // class to set corresponding description index in dsiplay with can bit
{
  private:
    int descrIndex;
    int canBit;

  public:
    ErrorValues(int canNum, int descriptionNum)
    {
      canBit = canNum;
      descrIndex = descriptionNum;
    }
    int getDescrIndex() {return descrIndex;}
    int getCanBit() {return canBit;}

}; */


union fVals
{
  byte b[4];
  float fVal;
} motorCurrent, BusCurrent, MotorRPM, busVolt, busCur, vel, mcCur, mcTemp, mcOdo, mcAh, dcCommand, mcVolt, mcRPM, bus1Volt, bus1Cur, vel1, mc1Cur, mc1Temp, mc1Odo, mc1Ah, dc1Command, mc1Volt, mc1RPM;

union inVals
{
  byte b[2];
  int inVal;
} BATDISCurrent, BATCHGCurrent, ChargerCurrent, ChargerVoltage;

//Variables for thes screen
Genie genie;
#define RESETLINE 5 //Make sure this matches the schematic

String MC_ERRORS[] = {"HWOC", "SWOC", "DC BUS OV", "HALL", "WATCHDOG", "CONFIG ERR", "15V RAIL UV", "DESAT....BAD!!"};
//motor status data
int limitFlags, errorFlags, activeMotor, canSendErr, canRecErr;
//motor status data
int limitFlags1, errorFlags1, activeMotor1, canSendErr1, canRecErr1;
// battery pack data 1
int packVolt, packCur, packAHr;

// battery pack data 2
byte relayStatus;
int dischargeRelay, chargeRelay, batSOC;

// battery pack data 3
int H_Volt, L_Volt, A_Volt, Delta_Volt;
#define CELL_MULTI_VOLT .0001

//battery pack data 4
int H_Temp, L_Temp, A_Temp;

//universal write variables
int uni_Vel, uni_Amp;

int CurrentBATDISCurrent = 0;
int CurrentBATCHGCurrent = 0;

void calculateCurrent()
{

  if (DUAL_MOTOR)
  {
    if (currentSelect == 0)
      uni_Amp = (mcCur.fVal + mc1Cur.fVal) * 10.0;
    else
      uni_Amp = busCur.fVal * 10.0;
  }
  else
  {
    if (currentSelect == 0)
      uni_Amp = mcCur.fVal * 10.0;
    else
      uni_Amp = busCur.fVal * 10.0;
  }

  Serial.print("Current sent to display:");
  Serial.println(abs(uni_Amp));
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 1, abs(uni_Amp));

  //reseting values once written on screen
}

int inRange(int value, int dist){ // TEST PURPOSES
  int returned = 0;
  if(random(2) != 0){
    returned = value + random(1, dist);
  }else{
    returned = value - random(1, dist);
  }
  return returned;
}

// For testing visi gini elements
void screenTest()
{
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0, inRange(120, 60)); // battvoltage
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 3, inRange(400, 350)); // SOC
  if(random(2) != 0){ // discharge and charge leds
    genie.WriteObject(GENIE_OBJ_USER_LED, 0, 1);
    genie.WriteObject(GENIE_OBJ_USER_LED, 1, 0);
  }else{
    genie.WriteObject(GENIE_OBJ_USER_LED, 0, 0);
    genie.WriteObject(GENIE_OBJ_USER_LED, 1, 1);
  }
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 8, inRange(160, 80)); // AmpsHour
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 7, inRange(70, 50)); // High cell;
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 6, inRange(30, 20)); // Low cell
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 4, inRange(50, 20)); // average
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 5, inRange(50, 40)); // delta
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 2, inRange(90, 60)); // high temp
}

void printCanMessage(const CAN_message_t msg)
{

  Serial.print(" BUS: ");
  Serial.print(msg.bus, HEX);
  Serial.println(" , ");
  Serial.print(" TS: ");
  Serial.print(msg.timestamp);
  Serial.println(" , ");
  Serial.print(" ID: ");
  Serial.print(msg.id, HEX);
  Serial.println(" , ");
  Serial.print(" Buffer: ");
  for (uint8_t i = 0; i < msg.len; i++)
  {
    Serial.print(msg.buf[i], HEX);
  }
  Serial.print("\n");
}

void readCanbus(const CAN_message_t &msg)
{ // global callback

  bool printRawCanMessage = false; // Change to `true` when debugging CAN traffic, otherwise leave `false`
  if (printRawCanMessage)
  {
    printCanMessage(msg);
  }

  switch (msg.id)
  {

  case BAT_CURRENT_LIMITS:

    BATDISCurrent.b[0] = msg.buf[0];
    BATCHGCurrent.b[0] = msg.buf[1];

    if (BATCHGCurrent.inVal != CurrentBATCHGCurrent)
    {
      genie.WriteObject(GENIE_OBJ_LED_DIGITS, 10, BATCHGCurrent.inVal);
      BATCHGCurrent.inVal = CurrentBATCHGCurrent;
    }
    if (CurrentBATDISCurrent != BATDISCurrent.inVal)
    {
      genie.WriteObject(GENIE_OBJ_LED_DIGITS, 11, BATDISCurrent.inVal);
      CurrentBATDISCurrent = BATDISCurrent.inVal;
    }

    break;

  case BMS_PACK_1:

    // convert byte[2] to short int
    packCur = ((int)msg.buf[0] << 8) | (int)msg.buf[1];
    // write pack voltage
    packVolt = ((unsigned int)msg.buf[2] << 8) | (unsigned int)msg.buf[3];
    Serial.print("Battery Pack Current:");
    Serial.println(packCur);
    Serial.print("Battery Pack Voltage");
    Serial.println(packVolt);
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0, packVolt);
    calculateCurrent();
//SOC as calculated by BMS
 
  if(batSOC!=msg.buf[4])
  {
  batSOC = msg.buf[4];
  genie.WriteObject(GENIE_OBJ_LED_DIGITS, 3, 10*int(batSOC*0.5));
  }
  Serial.print("Battery Pack SOC:");
  Serial.println(String(batSOC*0.5) +"%");
//Relay Status
    relayStatus = msg.buf[6];

    dischargeRelay = bitRead(relayStatus, 0);
    if (dischargeRelay == 1)
     { genie.WriteObject(GENIE_OBJ_USER_LED, 0, 1); // led is turned on when discharge is on
      Serial.println("Discharge is on:");}
    else
     { genie.WriteObject(GENIE_OBJ_USER_LED, 0, 0); // led is off when discharge is off
      Serial.println("Discharge is off:");}
    chargeRelay = bitRead(relayStatus, 1);
    if (chargeRelay == 1)
    { genie.WriteObject(GENIE_OBJ_USER_LED, 1, 1); // led is turned on when charge is on
      Serial.println("Charge is on:");}
    else
    { genie.WriteObject(GENIE_OBJ_USER_LED, 1, 0); // led is off when charge is off
      Serial.println("Charge is off:");}
  
    break;

    //-------------------------------------------------------------------------------
    // Orion BMS data pack 2/C
    // 0: 
  case BMS_PACK_2:
         packAHr = ((unsigned int)msg.buf[4] << 8) | (unsigned int)msg.buf[5];
         genie.WriteObject(GENIE_OBJ_LED_DIGITS, 8, (int)(packAHr * 10));
    break;

    //-------------------------------------------------------------------------------
    // Orion BMS data pack 3/D
    //
  case BMS_PACK_3:
    H_Volt = ((unsigned int)msg.buf[0] << 8) | (unsigned int)msg.buf[1];
    L_Volt = ((unsigned int)msg.buf[2] << 8) | (unsigned int)msg.buf[3];
    A_Volt = ((unsigned int)msg.buf[4] << 8) | (unsigned int)msg.buf[5];
    Delta_Volt = H_Volt - L_Volt;
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 7, (int)(H_Volt / 10));
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 6, (int)(L_Volt / 10));
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 4, (int)(A_Volt / 10));
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 5, Delta_Volt/10);

    Serial.print("Highest Cell volatge in 0.1mv: ");
    Serial.println(H_Volt);
    Serial.print("Lowest Cell Voltage in 0.1mv: ");
    Serial.println(L_Volt);
    Serial.print("Average Cell Voltage in 0.1mv: ");
    Serial.println(A_Volt);
    Serial.print("Delta Voltage: ");
    Serial.println(Delta_Volt);

    break;

    //-------------------------------------------------------------------------------
    // Orion BMS data pack 4/E
    //
  case BMS_PACK_4:
    H_Temp = (unsigned int)msg.buf[2];
    genie.WriteObject(GENIE_OBJ_LED_DIGITS, 2, H_Temp * 10);
    L_Temp = (unsigned int)msg.buf[3];
    //genie.WriteObject(GENIE_OBJ_LED_DIGITS, 4, L_Temp);
    Serial.print("Highest Cell Temperature: ");
    Serial.println(H_Temp);
    Serial.print("Lowest Cell Temperature: ");
    Serial.println(L_Temp);
    break;

  case BMS_PACK_5: // Error Codes
    // Read error codes from CAN and set in errorCodeStatus[]
    int errorCodeStatusIndex = 0;
    String printMsg = "";
    for (int i = 0; i < 8 && errorCodeStatusIndex < 24; i++)
    {
      for (int j = 0; j < 8 && errorCodeStatusIndex < 24; j++)
      {
        errorCodeStatusIndex++; // tracks current error code
        errorCodeStatus[errorCodeStatusIndex] = bitRead(msg.buf[i], j); // Sets status as t/f
        if (bitRead(msg.buf[i], j)) // If bit is true
        {
          int code = errorValues[errorCodeStatusIndex];
          String str = String(code); // covert int to string
          printMsg = printMsg + ", " + str; // Add to printMsg
        }
      }
    }

    genie.WriteStr(0, printMsg);

    break;
  }
}

void sendframe()
{
  //Add command to send any coms you want the screen to communicate
  //send screen temperature
}

void screenOptions()
{
  if (speedSelect) // true for MPH | false for KPH
    genie.WriteObject(GENIE_OBJ_STRINGS, 0, 1);
  else
    genie.WriteObject(GENIE_OBJ_STRINGS, 0, 0);

  if (currentSelect) // true for netA |  false mcA
    genie.WriteObject(GENIE_OBJ_STRINGS, 1, 0);
  else
    genie.WriteObject(GENIE_OBJ_STRINGS, 1, 1);
  // change to main screen
}

void screenEvents(void)
{

  genieFrame Event;
  genie.DequeueEvent(&Event);

  int slider_val = 15;

  //If the cmd received is from a Reported Event
  if (Event.reportObject.cmd == GENIE_REPORT_EVENT)
  {
    if (Event.reportObject.object == GENIE_OBJ_4DBUTTON) // If the reported message is from a Winbutton
    {

      if (Event.reportObject.index == 2) // If the reported message is from Current Button
      {
        // Write value to red LED
        speedSelect = !speedSelect;
        Serial.println("Speed Button"); // Write value to red LED
      }
      else if (Event.reportObject.index == 3)
      {
        currentSelect = !currentSelect;
        Serial.println("Current Button"); // Write value to red LED
      }
      screenOptions();
    }

    else if (Event.reportObject.object == GENIE_OBJ_SLIDER) // If the Reported Message is from a slider
    {
      int inputSlider = (Event.reportObject.data_msb << 8) + Event.reportObject.data_lsb; // data of message is stored
      Serial.println(inputSlider);                                                        // Write value to red LED

      slider_val = (inputSlider * 14) / 100 + 1;
      Serial.println(slider_val);
      genie.WriteContrast(slider_val);
      // Write value to red LED
    }
  }
}

void setup(void)
{
  randomSeed(analogRead(0)); // TEST PURPOSES

  Serial2.begin(200000); //connected to 4dscreen. Must be Serial2. Change to upload new 4D file
  genie.Begin(Serial2);
  Serial.println("Principia Solar Car Driver controls\0");

  pinMode(RESETLINE, OUTPUT); // Set D4 on Arduino to Output (4D Arduino Adaptor V2 - Display Reset)
  digitalWrite(RESETLINE, 0); // Reset the Display
  delay(100);
  digitalWrite(RESETLINE, 1); // unReset the Display
  delay(3300);                //let the display start up after the reset (This is important)
  Serial.println("Screen Reset");


  // Initialize error code status and msg arrays
  errorCodeStatus[0] = NULL; // 0 index not used
  errorCodeMsg[0] = NULL;
  for (int i = 1; i < 45; i++)
  {
    errorCodeStatus[i] = 0; // All error codes false
    errorCodeMsg[i] = i; // 1-44 loaded as msgs
  }


  //you can set the screen brightness here, using a value between 1 and 15.
  //0 will turn off the display

  genie.AttachEventHandler(screenEvents);

  /***********Some default display values*************************************/

  can2.begin();
  can2.setBaudRate(500000); // 500kbps data rate
  can2.enableFIFO();
  can2.enableFIFOInterrupt();
  can2.onReceive(FIFO, readCanbus);
  can2.mailboxStatus();

  can1.begin();
  can1.setBaudRate(500000); // 250kbps data rate
  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  can1.onReceive(FIFO, readCanbus);
  can1.mailboxStatus();

  timer.begin(sendframe, 50000); // Send frame every 50ms--100ms
}

void loop()
{
  genie.DoEvents(); // This calls the library each loop to process the queued responses from the display
  can1.events();
  can2.events();
  //delay(1000);
  //screenTest();
  // wait 2 milliseconds before the next loop for the analog-to-digital
  // converter to settle after the last reading:
}
