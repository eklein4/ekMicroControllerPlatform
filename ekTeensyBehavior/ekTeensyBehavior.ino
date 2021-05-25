// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// MicroController code by Eric Klein eric_klein@brown.edu, modified version of 
// csStateBehavior by Chris Deister --> cdeister@brown.edu
// 
// Programming for Teensy >=3.5 using Flexitimer to control signal generation and datalogging
// Intended to be used with a python program running pySerial or equivalent programming that can
// generate and read ASCII serial code. Serial communication syntax (headers, body, footers etc.) 
// created by Chris Deister and modified for speciific data structure 
//
// Structure from csStateBehacvior largely maintained but 'state' structure minimized
// to increase independence and flexibility in regard to signal generating behaviors. 
//
// TODO, test and update  On a Teensy 3.6, each interrupt takes ~50-100 us depending on how many variables are processed.
//
// Changes from Deister version:
//  Removal of unused I/O 
//  Removal of superfluous state processes
//  Restructure of state-change vs. signal generation dependences 
//  Addition of waveform parameter buffer. Serial-read structure only reads one command per interrupt, and thus only one 
//    perameter of behavior can be altered online per interrupt (1 ms by default)
//    previous version relied on low flexibility 'baseline' state to accumulate commands and set parameters to fire
//    simultaneously (if at all) on global change.
//    To accumulate parameters online, a buffer array is used to store incomming commands/parameters until an update
//    timepoint.   
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#include <math.h>
#include <Wire.h>
#include <FlexiTimer2.h>
#include <SPI.h>

// Other people's libraries
#include "HX711.h"
#include <MCP4922.h>


//-----------------------------
// ~~~~~~~ IO Pin Defs ~~~~~~~~
//-----------------------------
//
// a) Analog Input Pins
// TODO get rid of one of these
#define lickPinA  23      // Lick/Touch Sensor A 

#define genA0 A0
#define genA1 A1
#define genA2 A2
#define genA3 A3

#define analogMotion A16

// b) Digital Input Pins
#define scaleData  29
#define scaleClock  28

// c) Digital Interrupt Input Pins
// NO #define motionPin 36  // used for digital encoders not analog 
#define framePin  5
#define yGalvo  6

// d) Digital Output Pins
#define rewardPin 24  // Trigger/signal a reward
#define syncPin 25    // Trigger other things like a microscope and/or camera
#define syncPinPi 27  // send a session start to a Pi


#define sessionOver  26

#define rewardMirror 30

// session header
bool startSession = 0;  // Indicates whether an experiment has been triggered for the first loop


elapsedMillis trialTime;  //
elapsedMillis stateTime;  // leave for now, can enable some automatic state based stuff
elapsedMicros loopTime;   // this can be omitted after testing



// TODO this is hard coded into the Stim Gen, change back
uint8_t dacNum = 5;

//teensy 3.6 has true DACs on these channels, 3.3V, ideally add op amp for gain to 5V max
#define DAC1 A21
#define DAC2 A22

// ~~~ MCP DACs

// TODO Change LDAC pin (see readme) if appropriate
MCP4922 mDAC1(11, 13, 33, 37);
MCP4922 mDAC2(11, 13, 34, 32);


//--------------------------------
// ~~~~~~~ Variable Block ~~~~~~~~
//--------------------------------

// make a loadcell object and set variables
// TODO calibrate a scale 
#define calibration_factor 440000
#define zero_factor 8421804

HX711 scale(scaleData, scaleClock);
uint32_t weightOffset = 0;
float scaleVal = 0;

uint32_t lickSensorAValue = 0;
uint32_t genAnalogInput0 = 0;
uint32_t genAnalogInput1 = 0;
uint32_t genAnalogInput2 = 0;
uint32_t genAnalogInput3 = 0;
uint32_t analogAngle = 0;


// a) Set DAC and ADC resolution in bits.
uint8_t adcResolution = 12;
uint8_t dacResolution = 12;

// c) Frame Counter
volatile uint32_t pulseCount = 0;

// // d) Flyback Signal
//volatile uint32_t flybackVal = 0;

// e) State Machine (vStates) Interupt Timing
int sampsPerSecond = 1000;
float evalEverySample = 1.0; // number of times to poll the vStates funtion


// e) bidirectional dynamic variables
// ** These are variables you may want to have other devices/programs change
// ** To set each send over serial the header char, the value as an int and the close char '>'
// ** To get each variable ask over serial header char and the close char '<'
// ***** You can add to this, but add a single char header, the default value and increment the knownCount.
// EXAMPLE: To set the state to 2, send "a2>" over serial (no quotes). To get the current state send "a<"
// The following is the legend for each array entry:
// ____ State Machine Related
// a/0: teensyState (the teensy is considered to be the primary in the state heirarchy)
// ____ Reinforcement Related
// r/1: reward duration (if solenoid) or volume (if pump)
// g/2: deliver reward
// q/8: FB duration
// d/9: interpulse duration (us) of train X end the call with the DAC# so d1001 will set the IPI of DAC1 to 100.
// p/10: pulse duration (us) of train X end the call with the DAC# so p101 will set the pulse dur of DAC1 to 10.
// v/11: pulse amplitude of train X end the call with the DAC# so v40001 will set the pulse dur of DAC1 to 4000.
// t/12: stim type of train X end the call with the DAC# 0 is pulse train, 1 is ramp, 2 is asymmetric cosine stim; so t11> will set DAC1 to ramp.
// m/13: max pulses for a stimulus for channel X. m381> will set the number of pulses on DAC1 to 38.  (default = -2 disabled)
// l/14: current value on loadCell
// h/15: toggle a pin
//New Pulse Tracker Data
// n/3: Push next pulse, designated per channel as above, '0' (default) is real-time attibute update, when '1' parameters accumulated in Buffer
// f/4: Fire at state change (default ='0' channel channel fires at start time) = '1' channel fires at state change ends at duration or pulse max
// s/5: Train start time relative to 'trialTime' (default='0' disabled if F is activated)
// u/6: Train duration relative to S (disabled if 0)
// y/7: Asymmetrical waveforum pulse up duration (default 6 - applies only to whale stim or addtnl asym shapes) 


char knownHeaders[] =    {'a', 'r', 'g', 'n', 'f', 's', 'u', 'y', 'q', 'd', 'p', 'v', 't', 'm', 'l', 'h'};
 int32_t knownValues[] = { 0, 5, 0, 0, 0, -2, 2500, 6, 25, 90, 10, 0, 0, -2, 0, 0};
 int knownCount = 16;

// f) stim trains
// Pulse Train Array *****************************************
// 0: baseline (0) or pulse (1) states
// 1: stop bit; if you set a max pulse num, it will count down and flip this. If not 0, pulsing will stop.
// 2: baseline duration in interrupts
// 3: pulse duration in interrupts (ms by default) - compared to TRAINTIMER which is reset at every state change
// 4 baseline/ amplitude (as a 12-bit version of 3.3V e.g. 0V is 0 and 3.3V is 4095)
// 5: Stim amplitude (as a 12-bit version of 3.3V e.g. 0V is 0 and 3.3V is 4095)
// 6: Stim type (0 for pulse train; 2 for whale (ramp deleted))
// 7: Write Value (determined by the pulseGen function and is what we write to the DAC each interrupt).
// 8: number of pulses to complete
// 9: fire at state change
// 10: Train start
// 11: train duration
// 12: asym up time

// TODO add Bool to pass to stim gen for push next pulse
// embed the "Train ready" bool in the stim gen function 

// pulseTrainBuffer *****************************************
*// 0: baseline duration in interrupts
*// 1: pulse duration in interrupts (ms by default) - compared to TRAINTIMER which is reset at every state change
*// 2: Stim amplitude (as a 12-bit version of 3.3V e.g. 0V is 0 and 3.3V is 4095)
*// 3: Stim type (0 for pulse train; 2 for whale (ramp deleted))
*// 4: number of pulses to complete
*// 5: fire at state change
*// 6: Train start
*// 7: train duration
*// 8: asym up time

int32_t pulseTrainVars[][13] =
{ {1, 1, knownValues[9], knownValues[10], 0, knownValues[11], knownValues[12], 0, knownValues[13], knownValues[4], knownValues[5], knownValues[6],knownValues[7]},
  {1, 1, knownValues[9], knownValues[10], 0, knownValues[11], knownValues[12], 0, knownValues[13], knownValues[4], knownValues[5], knownValues[6],knownValues[7]},
  {1, 1, knownValues[9], knownValues[10], 0, knownValues[11], knownValues[12], 0, knownValues[13], knownValues[4], knownValues[5], knownValues[6],knownValues[7]},
  {1, 1, knownValues[9], knownValues[10], 0, knownValues[11], knownValues[12], 0, knownValues[13], knownValues[4], knownValues[5], knownValues[6],knownValues[7]},
  {1, 1, knownValues[9], knownValues[10], 0, knownValues[11], knownValues[12], 0, knownValues[13], knownValues[4], knownValues[5], knownValues[6],knownValues[7]}
};

int32_t pulseTrainBuffer[][9] =
{ {-1, -1, -1, -1, -1, -1, -1, -1, -1},
  {-1, -1, -1, -1, -1, -1, -1, -1, -1},
  {-1, -1, -1, -1, -1, -1, -1, -1, -1},
  {-1, -1, -1, -1, -1, -1, -1, -1, -1},
  {-1, -1, -1, -1, -1, -1, -1, -1, -1}
};

bool trainReceive[dacNum]; // tells functions whether a buffered stim train is being sent
bool trainReady[dacNum]; // tells functions whether a buffered stim train is being sent

// stim trains are timed with elapsedMicros timers, which we store in an array to loop with channels.
elapsedMillis trainTimer[dacNum];


uint32_t analogOutVals[] = {pulseTrainVars[0][7], pulseTrainVars[1][7], pulseTrainVars[2][7], pulseTrainVars[3][7], pulseTrainVars[4][7]};

uint32_t lastState = knownValues[0];  // We keep track of current state "knownValues[0]" and the last state to inform if something is run the first time

uint32_t loopCount = 0; // counts the recording state interrupt loops 
uint32_t trigTime = 10; // Duration (in interrupt time) of a sync out trigger.

uint32_t rewarding[2] = {0,0};  // keeps track of reward state for trigger duration

int headerStates[] = {0, 0, 0,};
int stateCount = 3;

void setup() {
   Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect 
  }
  // Start MCP DACs

  SPI.begin();
  pinMode(33, OUTPUT);
  pinMode(34, OUTPUT);

    // loadcell
  scale.set_scale(calibration_factor);
  scale.set_offset(zero_factor);
  scale.tare();
  
  // Analog In/Out
  analogReadResolution(12);
  analogWriteResolution(12);

    // Analog In/Out
  analogReadResolution(12);
  analogWriteResolution(12);

  // Interrupts
  // NO attachInterrupt(motionPin, rising, RISING);  // used for digital not analog
  attachInterrupt(framePin, frameCount, RISING);
  attachInterrupt(yGalvo, flybackStim_On, FALLING);

  // DIO Pin States
  pinMode(syncPin, OUTPUT);
  digitalWrite(syncPin, LOW);
  pinMode(sessionOver, OUTPUT);
  digitalWrite(sessionOver, LOW);
  //  pinMode(scaleData, OUTPUT);
  pinMode(syncPinPi, OUTPUT);
  digitalWrite(syncPinPi,LOW);

  pinMode(rewardPin, OUTPUT);
  digitalWrite(rewardPin, LOW);

  pinMode(syncMirror, INPUT);
  pinMode(rewardMirror, INPUT);

  // Start Program Timer
  FlexiTimer2::set(1, evalEverySample / sampsPerSecond, vStates);
  FlexiTimer2::start();


}

void loop() {
  // put your main code here, to run repeatedly:

}

void vStates() {
  // ***************************************************************************************
  // **** Loop Timing/Serial Processing:
  // Every loop resets the timer, then looks for serial variable changes.
  loopTime = 0;
  lastState = knownValues[0];

  // we then look for any changes to variables, or calls for updates
  int curSerVar = flagReceive(knownHeaders, knownValues);   // looks for serial messages, sets known values to message value
                                                            // once per call. returns -1 unless a new value is set, in which
                                                            // case it returns the header index of the set variable

  /* TODO
   *  This is what allows parameter values with a paired channel value to be pushed under a single serial header.
   *  To get rid of the STATE-Based stimulus structure, most of the "THINGS" that happen need to be handled this way.
   *  Reorganize for all necessary pushed variables, and make one single 'next time' header that is compatible
   */
  if ((curSerVar == 3) || (curSerVar == 4) || (curSerVar == 5) || (curSerVar == 6) || (curSerVar == 7) || (curSerVar == 9) || (curSerVar == 10) || (curSerVar == 11) || (curSerVar == 12) || (curSerVar == 13)) {
    setPulseTrainVars(curSerVar, knownValues[curSerVar], trainReceive, trainReady); //add update bool
  }


  // Some hardware actions need to complete before a state-change.
  // So, we have a latch for state change. We write over any change with lastState
 // if (blockStateChange == 1) {
 //   knownValues[0] = lastState;
 // }

  // ***************************************************************************************

  // **************************
  // State 0: Boot/Init State
  // **************************
  if (knownValues[0] == 0) {            // Check to see if Boot state is active, if so do the following

    // a) run a header for state 0        
    if (headerStates[0] == 0) {         // If the boot state isn't registered THINGS TO DO ONCE
                          
      genericHeader(0);                 // Do the things we always do  ONCE:
      loopCount = 0;                    // Set the 'in task' timmer to zero   
      pulseCount = 0;                   // Set the frame counter to zero
      // reset session header
      if (startSession == 1) {              // If task session was just exited
        digitalWrite(sessionOver, HIGH);    // send session-over TTL
        delay(10);
        digitalWrite(sessionOver, LOW);     
      }
      startSession = 0;                     // Register not in-session
    }
    // b) body for state 0
    genericStateBody();                     // do the second set of things we always do
  }

  // **************************
  // State != 0: (in task)
  // **************************
  if (knownValues[0] != 0) {

    if (loopCount == 0) {
      digitalWrite(syncPin, HIGH);
      trialTime = 0;
        for ( int i = 0; i < dacNum; i++) {       
          trainTimer[i] = 0;                  // this needs to be removed to allow for state independence
       // set their clock to zero // changed to evaluate whether to reset
          pulseTrainVars[i][0] = 1;
          pulseTrainVars[i][1] = 1;
          pulseTrainVars[i][7] = 0;
        }
    }
    // This ends the trigger.
    if (loopCount >= trigTime) {
      digitalWrite(syncPin, LOW);   
    }
    
    updatePulseTracker(pulseTrainVars, pulseTrainBuffer, trainReady); // check buffer for updates to pulse tracker
    
    //******************************************
    //@@@@@@ Start Non-Boot State Definitions.
    //******************************************

    // **************************
    // State 1: Boot/Init State
    // **************************
    if (knownValues[0] == 1) {
      // run this stuff once per session
      if (startSession == 0) {
        startSession = 1;   // used for session over
      }
      if (headerStates[1] == 0) {
        genericHeader(1);
        
      }
      stimGen(pulseTrainVars);                            ///
      setAnalogOutValues(analogOutVals, pulseTrainVars);
      genericStateBody();
    }

   // LEFT OFF, fuctionality through three build in above BUT StimGen needs rework still

    // ****************************************
    // State 8: Flyback Pulse State
    // ****************************************
    // Removed for now, if needed, roll into "1" with bit to activate and 'if' to zero analog outs 0-3
    // the FB interrupt will need be set to look for that bit rather than state
    /*
    else if (knownValues[0] == 8) {
      if (headerStates[8] == 0) {
        genericHeader(8);
      //  visStim(0);
        blockStateChange = 0;
      }
      stimGen(pulseTrainVars);
      // setAnalogOutValues doesn't do this if a zero array passed because it sets thru channel 4
      analogOutVals[0] = 0;
      analogOutVals[1] = 0;
      analogOutVals[2] = 0;
      analogOutVals[3] = 0;
      genericStateBody();
    }
    */
    // ******* Stuff we do for all non-boot states at the end.
    dataReport();
    loopCount++;
  }
}
// ******************************************************
// FUNCTIONS IN USE
// ******************************************************

//*******************************************************
/* Flag recieve changed to return specific non-variable integer (-1) if called w/o a serial buffer or if a paremeter 
 *  request is recieved. Returns header number only when paremeter change is rececieved. Also the max receieved int size limit
 *  was changed to reflect 12 bit (10 digits plus null) and the exceed case was changed to an error message escape with
 *  a buffer flush.
 *  NOTE: This WILL bug on slow serial (i.e. genuine Arduino / other slow TTL serial 9600 baud definitey bugs) unless
 *  blocking term is added after serial.read (e.g., while(rc != something))
 */

int flagReceive(char varAr[], int32_t valAr[]) {
  static byte ndx = 0;          // finds place being read
  char endMarker = '>';         // Designates recieve message
  char feedbackMarker = '<';    // Designates send/requested message
  char rc;                      // recieved variable header character
  uint32_t nVal;                // Variable value (I think)
  const byte numChars = 11;     // Size of string to store each digit of int 16 bit 10 digits + 1 null
  char writeChar[numChars];     // Make the string of this size
  int selectedVar = -1;          // Returned index of kV header of recieved variable for Pulse update TODO: pulses be updated here?
  static boolean recvInProgress = false;  // Bit indicating in recieve PERSISTS THROUGH CALLS TO THIS FUNCTION !
  bool newData = 0;               // Bit breaks loop when true, fliped when variable recieve is finished 
  int32_t negScale = 1;         // used to convert the ABS value of the variable recieved to negative when '-' read out

  while (Serial.available() > 0 && newData == 0) {    // set a loop to run if there is a bit in the buffer until the message is read
    rc = Serial.read();                               // read the (next) bit and store it to RC
    //delay(10);                                // this is for slow serial - 'while' statement is better but EITHER BLOCKS
    if (recvInProgress == false) {                // If this is a new message (variable; not partially read)
      for ( int i = 0; i < knownCount; i++) {   // Loop through the number of knownValues variables
        if (rc == varAr[i]) {                   // if the read byte == the looped to (any) kV Header char
          selectedVar = i;                      // set the return value to variable's the index of the kV Header 
          recvInProgress = true;                //  indicate the message is being read by flipping rcvInProg
          break;                                // added for efficency and prevent redunfance bugs 
        }
      }
    }

    else if (recvInProgress == true) {        // if the message is in the process of being read
      
      if (rc == endMarker ) {                 // if the bit is a revieve message stop 
        writeChar[ndx] = '\0';                // terminate the string by writing a non-integer to writeChar at present ndx
        recvInProgress = false;               // indicate finished message by flipping rcvInProg
        ndx = 0;                              // reset the index 
        newData = 1;                          // indecate that the message is read (and ready)
        nVal = int32_t(String(writeChar).toInt());  //convert the Char to string and then to integer and call it nVal
        valAr[selectedVar] = nVal * negScale; // process ABS value with +/- read and set the (global) array value equal to it
        return selectedVar;                   // message is done, return the variable's array index and break
      }

      else if (rc == feedbackMarker) {        // if the bit is a send message stop
        writeChar[ndx] = '\0';                // terminate the string by writing a non-integer to writeChar at present ndx
        recvInProgress = false;               // indicate finished message by flipping rcvInProg 
        ndx = 0;                              // reset the index 
        newData = 1;                          // indecate that the message is read (and ready)
                                  //don't write anything
        Serial.print("echo");                 // send a line through serial with the requested value
        Serial.print(',');
        Serial.print(varAr[selectedVar]);
        Serial.print(',');
        Serial.print(valAr[selectedVar]);
        Serial.print(',');
        Serial.println('~');
        selectedVar = -1;                     // prevent returning "1"
        return selectedVar;                   // message is done, return the variable's array index and break
      }
      else if (rc == '-') {                   // if "-" read out, store command to process the variable recieved
                                              // as negative TODO, BUG this could occur anywere before < >, requires
                                              // variable to store ndx where recieved was flagged to look only there
        negScale = -1;
      }
      else if ((rc != feedbackMarker || rc != endMarker) && isDigit(rc)) { // if the bit wasn't accounted for yet, assume it's a digit
                                                        // BUG if it's not a digit, could bug the string2int FIXED
        writeChar[ndx] = rc;          // Write the digit to the character array at ndx
        ndx++;                        // increment ndx
        //if (ndx >= numChars) {
        if (ndx >= numChars) {       // original 'if' here didn't make much sense if things are working but could cause
                                    // bugs in the recieved, so turned this into a error & escape
          //ndx = numChars - 1;
          recvInProgress = false;   //reset the statics
          ndx = 0;
          Serial.print("error");                 // send an error
          Serial.print(',');
          Serial.println('~');
          /* Optional buffer flush, could bug second message, should begin reading at next header with rcvInProgress False
           *  without this
           
          while (Serial.available()>0) {    // flush the buffer
            Serial.read();
          }
          */
          selectedVar = -1;                     // prevent returning "1"
          return selectedVar; 
        }
      }
    }
  }
  return selectedVar;                   // prevent returning "1" or "0"
}
//*******************************************************

void setPulseTrainVars(int recVar, int recVal, bool tRec, bool tRed) {

  int parsedChan = recVal % 10;  // ones digit is the channel     TODO May need to make '100' for more channels
  // int parsedValue = recVal * 0.1; // divide by 10 and round up   take care of round bug
  int parsedValue = (recVal-parsedChan) * 0.1; // remove 10's digit


  // Find whether buffer process is being controlled and flip the bit
  if (recVar == 3) {
    if ((tRec[parsedChan - 1] == true) && (parsedValue == 0)) { // check to see if buffer is buffer data is ready
      tRed[parsedChan - 1] == true;       // if so, flag channel ready
      tRec[parsedChan - 1] = false;       // reset buffer routing
    }
    else if (parsedValue == 1) {
      tRec[parsedChan - 1] = true;       // start buffer routing
    }
    else if (parsedValue == 0) {
      tRec[parsedChan - 1] = false;       // reset buffer routing
    }
  }

  if (tRec[parsedChan - 1] == true) {       // if buffer routing
    // FIRE AT STATE CHANGE
    if (recVar == 4) {
      pulseTrainBuffer[parsedChan - 1][5] = parsedValue;
    }
    // START TIME
    else if (recVar == 5) {
      pulseTrainBuffer[parsedChan - 1][6] = parsedValue;
    }
    // TRAIN DURATION
    else if (recVar == 6) {
      pulseTrainBuffer[parsedChan - 1][7] = parsedValue;
    }
    // ASYM UP DURATION
    else if (recVar == 7) {
      pulseTrainBuffer[parsedChan - 1][8] = parsedValue;
    }
    // IPI                  
    else if (recVar == 9) {
      pulseTrainBuffer[parsedChan - 1][0] = parsedValue;
    }
    else if (recVar == 10) {       // Pulse DUR
      pulseTrainBuffer[parsedChan - 1][1] = parsedValue;
    }
    else if (recVar == 11) {       // Pulse Amp
      pulseTrainBuffer[parsedChan - 1][2] = parsedValue;
    }
    else if (recVar == 12) {       // Stim shape
      pulseTrainBuffer[parsedChan - 1][3] = parsedValue;
    }
    else if (recVar == 13) {                            // pulse count
    // if you push pulses; make sure stop bit is off
      pulseTrainBuffer[parsedChan - 1][4] = parsedValue;    
      //pulseTrainVars[parsedChan - 1][1] = 0;
    }
  }
  else if (tRec[parsedChan - 1] == false) {       // if not buffer routing
    // FIRE AT STATE CHANGE
    if (recVar == 4) {
      pulseTrainVars[parsedChan - 1][9] = parsedValue;
    }
    // START TIME
    else if (recVar == 5) {
      pulseTrainVars[parsedChan - 1][10] = parsedValue;
    }
    // TRAIN DURATION
    else if (recVar == 6) {
      pulseTrainVars[parsedChan - 1][11] = parsedValue;
    }
    // ASYM UP DURATION
    else if (recVar == 7) {
      pulseTrainVars[parsedChan - 1][12] = parsedValue;
    }
    // IPI                  
    else if (recVar == 9) {
      pulseTrainVars[parsedChan - 1][2] = parsedValue;
    }
    else if (recVar == 10) {       // Pulse DUR
      pulseTrainVars[parsedChan - 1][3] = parsedValue;
    }
    else if (recVar == 11) {       // Pulse Amp
      pulseTrainVars[parsedChan - 1][5] = parsedValue;
    }
    else if (recVar == 12) {       // Stim shape
      pulseTrainVars[parsedChan - 1][6] = parsedValue;
    }
    else if (recVar == 13) {                            // pulse count
    // if you push pulses; make sure stop bit is off
      pulseTrainVars[parsedChan - 1][8] = parsedValue;    
      //pulseTrainVars[parsedChan - 1][1] = 0;  //removed for piulse handling updates
    }
  }
}

// ******************************************************


// ******************************************************
/*
 * This function/array actually keeps track of whether the "state" is being run for the first time.
 * This allows conditionals within a state
 * When called for whatever reason it sets all states to a first-time condition, 
 * this basically means "THE STATE CHANGED"
 */
void resetHeaders() {
  for ( int i = 0; i < stateCount; i++) {
    headerStates[i] = 0;
  }
}
// ******************************************************


// ******************************************************
void genericHeader(int stateNum) {                    // Called with the current header number
 
  //  reset header states and set current state's header to 1 (fired).
  resetHeaders();
  headerStates[stateNum] = 1;

  for ( int i = 0; i < dacNum; i++) {       
    if (pulseTrainVars[i][9]==1) {
        trainTimer[i] = 0;                  // this needs to be removed to allow for state independence
       // set their clock to zero // changed to evaluate whether to reset
        pulseTrainVars[i][0] = 1;
        pulseTrainVars[i][1] = 0;
        pulseTrainVars[i][7] = 0;
        pulseTrainVars[i][10] = trialTime;
      
    }                                    
  }
  // set analog output values to 0.
  /*
   * this is a decent check to misfires will be over ridden by state kill programming
   */
  analogOutVals[0] = 0;
  analogOutVals[1] = 0;
  analogOutVals[2] = 0;
  analogOutVals[3] = 0;
  analogOutVals[4] = 0;
  /*
   * Theis is a good check when states change, and it would be overwritten but the state kill programming 
   * remap these based on new pulse tracker if needed but current is 0,1,7,9 pulse/baseline, stop bit, write val, up.down
   */
  // pollToggle();  // !!! JUST DISABLING THIS, NO CURRENT NEED !!!
  // d: reset state timer.
  stateTime = 0;              //  leave for now - enables some state based automatic stuff
}
// ******************************************************

// ******************************************************
void frameCount() {
  pulseCount++;
}
// ******************************************************

// ******************************************************
void genericStateBody() {

  lickSensorAValue = analogRead(lickPinA);              // Read for licks once a loop (millisecond) 
  genAnalogInput0 = analogRead(genA0);                  // Read each of the analog inputs 
  genAnalogInput1 = analogRead(genA1);
  genAnalogInput2 = analogRead(genA2);
  genAnalogInput3 = analogRead(genA3);
  //pollToggle();                                         // removed because No current use, but code to add back below
  checkReward();
  pollRelays();                                           // look for manual hardware commands
  analogAngle = analogRead(analogMotion);
  writeAnalogOutValues(analogOutVals);                  // Set the pins to values in the Analog out values array          
  if (scale.is_ready()) {
    scaleVal = scale.get_units() * 22000;             // TODO test this conversion
    // this scale factor gives hundreths of a gram as the least significant int
    knownValues[14] = scaleVal;
  }
}

// ******************************************************


// ****************************************************************
// **************  Pulse Train Function ***************************
// ****************************************************************
void setAnalogOutValues(uint32_t dacVals[], uint32_t pulseTracker[][13]) {
  /*if (sDacGate1 == 0) {
    dacVals[0] = pulseTracker[0][7];
    dacVals[1] = pulseTracker[1][7];
  }
  else if (sDacGate1 == 1) {
    dacVals[0] = pulseTracker[1][7];
    dacVals[1] = pulseTracker[0][7];
  }
  if (sDacGate2 == 0) {
    dacVals[2] = pulseTracker[2][7];
    dacVals[3] = pulseTracker[3][7];
  }
  else if (sDacGate2 == 1) {
    dacVals[2] = pulseTracker[3][7];
    dacVals[3] = pulseTracker[2][7];
  } 
  // This dac-gate programming is an unused artifact, left here as tip for implementation if useful
  */
  dacVals[0] = pulseTracker[0][7];
  dacVals[1] = pulseTracker[1][7];
  dacVals[2] = pulseTracker[2][7];
  dacVals[3] = pulseTracker[3][7];
  dacVals[4] = pulseTracker[4][7];
}

// ******************************************************

void writeAnalogOutValues(uint32_t dacVals[]) {
  analogWrite(DAC1, dacVals[0]);
  analogWrite(DAC2, dacVals[1]);
  mDAC1.Set(dacVals[2], dacVals[3]);
  mDAC2.Set(dacVals[4], dacVals[4]);
}
// ******************************************************
void giveReward(bool startR) {
      if ((startR==true) && (rewarding[0]==0)) {
        digitalWrite(rewardPin, HIGH);
        rewarding[0] = 1;
        rewarding[1] = trialTime+knownValues[1];
        
      }
      else if ((rewarding[0] == 1) && (trialTime < rewarding[1])) {
        digitalWrite(rewardPin, HIGH);
        rewarding = 1;
      }
      else if ((rewarding[0] == 1) && (trialTime >= rewarding[1])) {
        digitalWrite(rewardPin, LOW);
        rewarding[0] = 0;
        rewarding[1] = 0;
      }
    }

// ******************************************************

void pollRelays() { // Looks for  hardware logic input commands currently only adding reward
  //bool rTrig;
  bool rRwd;
  //rTrig = digitalRead(syncMirror);
  rRwd = digitalRead(rewardMirror);
  //if (rTrig == 1) {
  //  digitalWrite(syncPin, HIGH);
  //  delay(5);
  //  digitalWrite(syncPin, LOW);
  //}
  if (rRwd == true) {
    giveReward(rRwd);
  }
}

// ******************************************************    

// ******************************************************
void checkReward() {
      if ((knownValues[2]==1) && (rewarding[0] == 1)) {
        giveReward((bool)0);
        knownValues[2]==0;
      }
      else if ((knownValues[2]==0) && (rewarding[0] == 1)) {
        giveReward((bool)0);
      }
      else if ((knownValues[2]==1) && (rewarding[0] == 0)) {
        giveReward((bool)1);
        knownValues[2]==0;
      }
      else {
        giveReward((bool)0);
      }
    }

// ******************************************************


// ******************************************************
void updatePulseTracker(int32_t pVars[][13], int32_t pBuffer[][9], bool tReady[]) {
  uint8_t varMap[] = {2, 3, 5, 6, 8, 9, 10, 11, 12};                    // This is a map from buffer array to pulse train array
  for ( int i = 0; i < dacNum; i++) {                                   // Loop through Dacs/ lines of tracker
      if ((tReady[i] == true) && ( (trialTime >= pBuffer[i][6])  || (pVars[i][1]==1) )) { // if pulse vars ready + the channel is not active or its time for the new train
        for (int idx = 0; idx < 9; idx++) {                             // loop through buffer parameters
          if (pBuffer[i][idx] != -1) {                                  // if not default value
            pVars[i][varMap[idx]] = pBuffer[i][idx];                    // copy new parameter to pulse train vars
            pBuffer[i][idx] = -1;                                       // set default value
          }
        }
        tReady[i] = false;                                              // reset redy bool when buffer flushed
      }  
  }
}
// ******************************************************

// ******************************************************

void stimGen(uint32_t pulseTracker[][13]) {

// f) stim trains
// Pulse Train Array *****************************************
// 0: baseline (0) or pulse (1) states
// 1: stop bit; if you set a max pulse num, it will count down and flip this. If not 0, pulsing will stop.
// 2: baseline duration in interrupts
// 3: pulse duration in interrupts (ms by default) - compared to TRAINTIMER which is reset at every state change
// 4 baseline/ amplitude (as a 12-bit version of 3.3V e.g. 0V is 0 and 3.3V is 4095)
// 5: Stim amplitude (as a 12-bit version of 3.3V e.g. 0V is 0 and 3.3V is 4095)
// 6: Stim type (0 for pulse train; 2 for whale (ramp deleted))
// 7: Write Value (determined by the pulseGen function and is what we write to the DAC each interrupt).
// 8: number of pulses to complete
// 9: fire at state change
// 10: Train start
// 11: train duration
// 12: asym up time


// everything is run in this loop except 'updateCount' being reset, actually its there to (redundant)
  int i;
  int updateCount = 0;
  for (i = 0; i < dacNum; i = i + 1) { 
// The first thing we need to do is create code to evaluate the start time, train timer and stop bit, edit them, and then if pulsing pass the rest as 'if'
  if (pulseTracker[i][10] != -2) {   // If the channel is not disabled  - evaluate active state, otherwise leave off - state fire flips bit in the header so not tested here
    // state fire tests start time = to trial time, so that needs to be set to '-2' when stop bit is flipped
    
    // ** Test pulse train start conditions
    if (pulseTracker[i][1]==1) {         // If the channel is off - evaluate start conditions and if pulsing initiated start train timer
      if (pulseTracker[i][10] <= trialTime) {   // if start time met - evaluate fire channel
        if (((pulseTracker[i][10] + pulseTracker[i][11]) > trialTime) && ((pulseTracker[i][8] > 0) || (pulseTracker[i][8] == -2))) {
          // if we haven't exceeded pulse time or pulse number limits, and are NOT pulsing, fire channel and set timing to zero
          pulseTracker[i][1] = 0;   // fire channel
          trainTimer[i] = 0;       // start waveform clock
          pulseTracker[i][0] = 1;   // start from pulse state
          pulseTracker[i][7] = 0;   // clear dac value  
        }
        else if ((pulseTracker[i][11] == 0) && (pulseTracker[i][8] > 0)) {
          // if we if we have pulses and no duration limit but start time is valid, and are NOT pulsing, fire channel and set timing to zero
          pulseTracker[i][1] = 0;   // fire channel
          trainTimer[i] = 0;       // start waveform clock
          pulseTracker[i][0] = 1;   // start from pulse state
          pulseTracker[i][7] = 0;   // clear dac value
        }
        else {
          pulseTracker[i][7] = pulseTracker[i][4]; // set baseline
        }
      }
      else {
        pulseTracker[i][7] = pulseTracker[i][4]; // set baseline
      }
    }
    // ** pulse train active, test stop conditions and set stop bit and negative start or execute pulses
    if (pulseTracker[i][1]==0) { // If the channel is active - evaluate turning it off by duration and by pulse count, if the channel shouldn't turn off - pulse tracking
      // if we exceed duration flip off channel, flip start -2
      if ((pulseTracker[i][10] + pulseTracker[i][11]) <= trialTime) {
        pulseTracker[i][1] = 1;   // kill channel
        pulseTracker[i][0] = 1;   // reset pulse state
        pulseTracker[i][7] = pulseTracker[i][4];   // dac to baseline
        pulseTracker[i][10] = -2; // turn off channel
      }
      // if we run out of pulses - flip off channel, flip start -2
      else if ((pulseTracker[i][8] < 1) && (pulseTracker[i][8] != -2)) {
        pulseTracker[i][1] = 1;   // kill channel
        pulseTracker[i][0] = 1;   // reset pulse state
        pulseTracker[i][7] = pulseTracker[i][4];   // dac to baseline
        pulseTracker[i][10] = -2; // turn off channel
      }
      else {  // generate stim wave form 
        int stimType = pulseTracker[i][6];            // Look at column 6 for a waveform Code (0=square, 2 = asym cosine (1 was going to be ramp))
        int pulseState = pulseTracker[i][0];          // Look st colimn 0 for whether the channel is in a pulse or a baseline state

        //**************************
        // *** 0 == Square Waves
        //**************************
        if (stimType == 0) {                          // If (main) the channel is set to a square wave

          // a) pulse state
          if (pulseState == 1) {                      // If (second) it is in a pulse phase
            // 
            if (trainTimer[i] >= pulseTracker[i][3]) {          // Look at TRAINTIMER array to determine whether pulse length has run up to set duration in PT[3]
             trainTimer[i] = 0; // reset counter               // If so, reset the traintimer line
             pulseTracker[i][0] = 0; // stop pulsing           // Move the channel to baseline phase PT[0] = 0
              pulseTracker[i][7] = pulseTracker[i][4];          // Set the output amplitude to baseline PT[7] to baseline amplitude PT[0]
              // *** 1b) This is where we keep track of pulses completed. (99999 prevents pulse counting)
             updateCount = 1;                                  // Indicate that the pulse should be counted UPDATE COUNT
           }
            // *** 2) determine pulse amplitude
            else {                                    // If  pulse phase not exceeded
                pulseTracker[i][7] = pulseTracker[i][5]; // set the Output value to the pulse amplitude 5 is the pulse amp; 7 is the current output.          
           }
         }
         // b) baseline/delay state                    
         else if (pulseState == 0) {                   // Else (second, not in pulse phase), if not in pulse phase 
           // if we are out of baseline time; move to stim state
           if (trainTimer[i] >=  pulseTracker[i][2]) {       // Look at TRAINTIMER array to determine whether baseline length PT[2] is exceeded
             trainTimer[i] = 0; // reset counter             // if so reset TRAINTIMER line
              pulseTracker[i][0] = 1; // start pulsing        // set the channel to pulse phase PT[0]=1
              pulseTracker[i][7] = pulseTracker[i][5];      //  set the output value PT[7] to the pulse amplitude PT[5]
           }
           else {                                            // else if the baseline duration was not exceeded
             pulseTracker[i][7] = pulseTracker[i][4];        // set the output value to the baseline value
            }
          }
        }
        
        // add ramp back here

       // 2) Asymm Cosine
       else if (stimType == 2) {                 // ELSE if (MAIN) the channel is set to a asym cosine wave 
        if (pulseState == 1) { 
          if (trainTimer[i] >= pulseTracker[i][3]) {  // if the wave timer has exceeded the pulse duration
            trainTimer[i] = 0; // reset counter               // If so, reset the traintimer line
            pulseTracker[i][0] = 0; // stop pulsing           // Move the channel to baseline phase PT[0] = 0
            pulseTracker[i][7] = pulseTracker[i][4];          // Set the output amplitude to baseline PT[7] to baseline amplitude
            updateCount = 1;                                  // Indicate that the pulse should be counted UPDATE COUNT 
            
            
          }
          else if ((trainTimer[i] >= 0) && (trainTimer[i] < pulseTracker[i][12]) ) {  // if timer is in a upward phase
            float curTimeSc = PI + ((PI * (trainTimer[i] + 1)) / pulseTracker[i][12]);       // Create a pi to 2 pi phase counter over up duration
            pulseTracker[i][7] = pulseTracker[i][5] * ((cosf(curTimeSc) + 1) * 0.5); // based on phase, multiply point on range between 0 to 1 with pulse amp
            
          }
          else if ((trainTimer[i] >= pulseTracker[i][12]) && (trainTimer[i] < pulseTracker[i][3])) { // if timer is in a downward phase
            float curTimeSc = ((PI * (trainTimer[i] + 1 - pulseTracker[i][12])) / (pulseTracker[i][3]-pulseTracker[i][12]));  // Create a pi to 2 pi phase counter over down duration
            pulseTracker[i][7] = pulseTracker[i][5] * ((cosf(curTimeSc) + 1) * 0.5); // based on phase, multiply point on range between 0 to 1 with pulse amp
            
          } 
         }
        `else if (pulseState == 0) {
          if (trainTimer[i] >= pulseTracker[i][2]) {   // if timer exceeds baseline time
            trainTimer[i] = 0; // reset counter               // If so, reset the traintimer line
            pulseTracker[i][0] = 1; // start pulsing           // Move the channel to pulse phase PT[0] = 1
            //then do up phase stuff 
            float curTimeSc = PI + ((PI * (trainTimer[i] + 1)) / pulseTracker[i][12]);       // Create a pi to 2 pi phase counter over up duration
            pulseTracker[i][7] = pulseTracker[i][5] * ((cosf(curTimeSc) + 1) * 0.5); // based on phase, multiply point on range between 0 to 1 with pulse amp 
          }
          else {
            pulseTracker[i][7] = pulseTracker[i][4];          // Set the output amplitude to baseline PT[7] to baseline amplitude
          }
          
        }
       }
       // *** Type Independent Stuff ***
       // *** This is where we keep track of pulses completed. (99999 prevents pulse counting)
       if (updateCount==1){                                                    // If a pulse just finished  - count pulses
        if ((pulseTracker[i][8]  > 0) && (pulseTracker[i][8] != -2)) {     // If there is a pulse number limit   
          pulseTracker[i][8] = pulseTracker[i][8] - 1;                        // count down the pulse that just finished
            if (pulseTracker[i][8] <= 0) {                                      // if the pulses are counted down (should be 0 but could hit -1 possibly?) 
             // flip the stop bit        
             pulseTracker[i][1] = 1;                                           // activate stop bit
             pulseTracker[i][10] = -2;                                           // turn off channel
             pulseTracker[i][8] = -2;                                           // disable pulse counting (default) because channel is dead and initial # is lost now
                                                                                // so would need to push more pulses anyway to count more
            }
          }
        updateCount = 0;                                                      // complete process of pulse count handling. 
        }
         
       
       } // end of stim type (main) conditionals and active stim train state code
        
      }  // END STIM GEN
    }
  }
  }
// ******************************************************


// ******************************************************
// OLD OR UNUSED FUNCTIONS
// ******************************************************

// ******************************************************
/*
 * This COULD be rolled into the setPulseTrainVars function as other things will be, ok with seprate for now, might help
 * Takes value read into knownVals, parses it for intended channel and sends channel TTL
 * 
 * TODO test whether "0" and "1" works rather than "1" and "2"
 * TODO CHANGE TO APPROPRIATE KV INDEX
 */
 /*
void pollToggle() {
  if (knownValues[15] != 0) {
    int parsedChan = knownValues[15] % 10;  // ones digit is the channel
    int parsedValue = (knownValues[15]-parsedChan) * 0.1; // remove 10's digit


    if (parsedValue == 1) {
      digitalWrite(parsedChan, 1);
    }
    else if (parsedValue == 0) {
      digitalWrite(parsedChan, 0);
    }
    else {
      digitalWrite(parsedChan, 0);
    }
    knownValues[15] = 0;
  }
}
*/
// ******************************************************
/*
// ******************************************************
void flybackStim_On() {

  pulseCount = pulseCount + 1;
  if (knownValues[0] == 8) {

    elapsedMicros pfTime;
    pfTime = 0;
    while (pfTime <= knownValues[16]) {
      stimGen(pulseTrainVars);
      analogWrite(DAC1, pulseTrainVars[0][7]);
      analogWrite(DAC2, pulseTrainVars[1][7]);
      mDAC1.Set(pulseTrainVars[2][7], pulseTrainVars[3][7]);
    }
    analogWrite(DAC1, 0);
    analogWrite(DAC2, 0);
    mDAC1.Set(0, 0);
  }
}
*/
