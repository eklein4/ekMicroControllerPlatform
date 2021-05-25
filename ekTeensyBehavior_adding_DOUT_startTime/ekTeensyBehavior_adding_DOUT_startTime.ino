/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  
  Notes on timing: Current implementation requires that subsecond clock frequency must divide evenly into 1s (e.g,
  1200Hz, or 5000Hz, not 5.3Hz). Supra second clock frequency must divide evently into hours e.g., .1Hz, or .05 Hz, but not
  .14 Hz [every 7 s] )

  Note on channel behavior: new start time must be re-entered every time boot state entered. Start times are erased
  at new boot state to prevent re-fires of trains that did not complete (and reset) before re-entering boot state
  
  UPDATES:

  3)  (Basicallly) all peripheral I/O is in and tested, frame counter interrupt is in but must be tested with Bruker systerm.
  To impliment flyback, a separate dacNum sized logic array must be implimented to test whether to pulse or zero w/o the
  y-galvo signal, and for the interrupt fuction to pulse with it. Also I would probably add a global bit if ANY are enabled
  to save looping that logic. Set pulse train vars will need to update this by channel, though I don't see a need to 
  buffer it (with update PTV). 

    - For flyback, either no analog reads can be enabled (one 20um interrupt over the current loop time at 10khz would cause an 
    underrun, however depending on the live scan freq, it would likely work withoutht the analog read), or -more likely
    the clock will just need to be run at 5kHz or slower, which is not a big deal and could allow even more reads. 


  

  Bug: Fix, since ones column currently assigns channel in serial commands, start times max at 10X shorter than intended (~30min)
  because int variable can not hold the extra digit. This then needs to be parsed in flag recieve. create a channel var array
  in Flag to compare to selected var, then add an if that parses the ones just ax the set pulse train does, and set another var
  equal to it.. that can be set to the setpulse train function along with the selected var. 

  fixed - flag recieve now parses the channel assignment directly, and calls set-PTV it self giving it the
  value without the channel, and the extra parameter designating the channel. sPTV no longer parses this. 

  sPTV is no longer called every interrupt loop. However updatePulseTracker to insure proper PTbuffer behavior

  tested with a 10000 second p duration, accepted variable (previously errored), and fired, did NOT wait 10K seconds 

  also tested dynamic update of (whale) pulse shaping and all parameters were dynamic and updated during the pulse


  

  2) Did #1 with a 1-hour clock rollovert because of limits to variable resolution. However, the data arrays 
  w/in the program, mostly because negative parameters, make storing timing at that resolution difficult.
  Further added basic serial send/recieve and part of the process is string conversion that does not work 
  for any very large datum (e.g., 64 bit double). Parameters could be stored as double (in effecient) and
  fuctions to translate the serial send/rcv strings could be written. However even in this case roll over
  handling at one hour still needs to happen (clock / parameter resets and subtractions). So if we need
  to build this anyway, may as well set it to every 30 minutes because the programming will be algorhythmically 
  equivalent and it inherently solves the data/variable types as well as the conversion issue because maximum
  time values of microseconds specified / counted will all fall within the range of 'long' or int32_t variables
  already in use. 

    * Add fuction that can be called by state change (experiment vs boot, we want boot at least for sync) or
    * by the main clock function that handles rollover at 30min and adjust all counted variables accordingly
    * 
    * Also add a streamed counter 

  1) In this iteration I am attempting to remove the elapsedmillis counter that runs concurrently to
  an elapsed micros counter as it is superfluous. Their role was a check to that the FT loop was running 
  on time and they would be added together to figure out millisecond and submilisecond time... Theoretically
  this works, and rollover is less a problem with the micro timer, but really all that is needed here is the 
  micro timer, and keeping track of microseconds whatsoever will cause a rollover problem in the accumulator at
  ~ 1HR 10min, so if this is rolling over, may as well accumulate base on the timer for simplicity, cheking
  for loop drift and mayby rounding off to the theoretical loop resolution (because micros will over count through
  the FLOPS after the loop that store it's value).  Also- the previous strategy was triggering a timmer overrun
  error (that I wrote) and so something was wrong there. 
    * Change accumulator to look at stored micros after initialization loop (used to measure delta from first zero)
    * re-work main body ranOnly, ranOnce, ranTwice logic as needed
  

    
  TIME DEBUGGING PROBLEM AND FIRST STRATEGY: ---------
  For some odd reason, when you reset the ELAPSEDMILLIS clock to zero within a FLEXITIMER period, the clock resolution seems
  utilized to increment the millis is greater than that of the reported coefficient, and only the latter is
  reset. If the millis are reported at a greater than 1kHz resolution driven by another interrupt (e.g.,
  FLEXITIMER), the offset between the clocks can cause the behavor of a milli rounding up at one point
  even if the FLEXITIMER rate evenly divides into 1Khz, effectivey skipping one, but only one period count.
  After the skip the coefficient sychronizes with FT so long as the clocking can keep up. 

     AS SUCH it is necessary to implement a sychronizing step. As reseting the millis does not correct
     the offset, you must initialize the count by setting it to zero and allowing FLEXITIMER to loop at
     a rate > 1kHz (@ rates < 1kHz this behavior does not manifest) while only monitoring for the first
     incrementaion of the millis variable (e.g., via an 'if' statement) at which point you set the millis
     variable and any counter incremented by FT looping to zero, and then proceed with the program. 
 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#include <math.h>
#include <Wire.h>
#include <FlexiTimer2.h>
#include <SPI.h>

// Other people's libraries
#include "HX711.h"
#include <MCP4922.h>

//  Interupt Timing vars
// -------------            SET THESE  --->mainly sampsPerSecond, this is your temporal resolution in Hz
// --------------                              it is effectively divided by evalEvery, leave that

const uint32_t sampsPerSecond = 10000; // temporal resolution of timer units in seconds * 1/ sampsPerSecond
const uint8_t evalEverySample = 1; // call fucntion after this many units
const uint32_t microsPerLoop = (uint32_t)((100000.0/sampsPerSecond)*evalEverySample); // 10us units/L 10micro s's per loop for time keeping quanta


// This is just early because variable dependency 
const uint8_t dacNum = 5; // number of DACs
const uint8_t dOutNum = 4; // number if generic digi-out and reward control

//  Serial Send-recieve Experimental Flow Control Data / Commands --------------

// ** These are variables you may want to have other devices/programs change
// ** To set each send over serial the header char, the value as an int and the close char '>'
// ** To get each variable ask over serial header char and the close char '<'
// ***** You can add to this, but add a single char header, the default value and increment the knownCount.
// EXAMPLE: To set the state to 2, send "a2>" over serial (no quotes). To get the current state send "a<"
// The following is the legend for each array entry:
// ---------------------
// a/0: teensyState (the teensy is considered to be the primary in the state heirarchy)
//  
// r/1: digital out (reward) signal duration, add 1's place to select a  channel (1:4)
// g/2: trigger digital out (reward)  -2 = toggle off, -1 = trigger channel, >=0 set trigger time , add 1's place to select a  channel (1:4)
//-------------------------- Pulse Tracker Data
// n/3: Push next pulse, designated per channel as above, '0' (default) is real-time attibute update, when '1' parameters accumulated in Buffer
// f/4: Fire at state change (default ='0' channel channel fires at start time) = '1' channel fires at state change ends at duration or pulse max
// s/5: Train start time relative to 'trialTime' (default='0' disabled if F is activated)
// u/6: Train duration relative to S (disabled if 0)
// y/7: Up Phase duration - Asymmetrical waveforum pulse (default 6 - applies only to whale stim or addtnl asym shapes) 
// q/8: Fly Back duration
// d/9: interpulse duration (us) of train X end the call with the DAC# so d1001 will set the IPI of DAC1 to 100.
// p/10: pulse duration (us) of train X end the call with the DAC# so p101 will set the pulse dur of DAC1 to 10.
// v/11: pulse amplitude of train X end the call with the DAC# so v40001 will set the pulse dur of DAC1 to 4000.
// t/12: stim type of train X end the call with the DAC# 0 is pulse train, 1 is ramp, 2 is asymmetric cosine stim; so t11> will set DAC1 to ramp.
// m/13: max pulses for a stimulus for channel X. m381> will set the number of pulses on DAC1 to 38.  (default = -2 disabled)
// l/14: current value on loadCell
// h/15: toggle a pin


char knownHeaders[] =    {'a', 'r', 'g', 'n', 'f', 's', 'u', 'y', 'q', 'd', 'p', 'v', 't', 'm', 'l', 'h'};
int32_t knownValues[] = { 0, 5000, 0, 0, 0, -2, 2500, 6, 25, 90, 10, 0, 0, -2, 0, 0};
int knownCount = 16;

bool trainReceive[dacNum]; // tells functions whether a buffered stim train is being sent
bool trainReady[dacNum]; // tells functions whether a buffered stim train is being sent

// use later to implement channel specific flyback interrupt
// bool flyBackOn = 0; //use to kick stimGen / write vals to flyback flow
// bool flyBackOnChan[dacNum]; // parse to apply FB flow or standard

// Stimulus Tracking Parameters-------------------

// Pulse Train Array *************
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

// pulseTrainBuffer ****************
// 0: baseline duration in interrupts
// 1: pulse duration in interrupts (ms by default) - compared to TRAINTIMER which is reset at every state change
// 2: Stim amplitude (as a 12-bit version of 3.3V e.g. 0V is 0 and 3.3V is 4095)
// 3: Stim type (0 for pulse train; 2 for whale (ramp deleted))
// 4: number of pulses to complete
// 5: fire at state change
// 6: Train start
// 7: train duration
// 8: asym up time

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

//   ----------  Peripheral i/o -----------------

// Default value on DACs, add more if dacNum changed

uint32_t analogOutVals[] = {pulseTrainVars[0][7], pulseTrainVars[1][7], pulseTrainVars[2][7], pulseTrainVars[3][7], pulseTrainVars[4][7]};

uint16_t encoderAngle = 0; // store the value from wheel encoder

    // Analog reads mostly not used for loop speed
uint16_t genAnalogInput0 = 0;       // ~73um per loop w/ encoder +1 analog reads
uint16_t genAnalogInput1 = 0;     // ~94um per loop w? encoder +2 analog reads
// uint16_t genAnalogInput2 = 0;    //   over run @ 10kHz

bool lickSensorValue=0;  // readout - indicates licks

// Scale adjustment stuff
const long setVal = -3471;  // used to adjust scale
long weightOffset = 8324525;
long scaleVal = 0; // readout for scale data


// Timing counters  Params:  // ------------------------

uint32_t trialTime = 0;  // main real-timer in um

elapsedMicros loopTime = 0; // time counter for running body of vStates and comparator for counting realtime

uint32_t loopedTime = 0; // storage of loopTime per loop

uint32_t timeNow = 0; // part of counter

uint8_t rolledHours = 0; // counter to allow micros to roll every hour (near max countable)

const uint32_t trigTime = ceil((10 * 100.0) / microsPerLoop); // duration for trigers in milliseconds (default 10ms) converted to loops round up using 10usUnit/loop var

// stim trains are timed with timers in this array, elapsedmillis or incremented float TODO: Make this an icremented by looping FLOAT
uint32_t trainTimer[dacNum];

// uint32_t stateTime = 0;  // TODO: add if and additional states used (unnecessary for now)

uint32_t loopCount = 0; // incremented by interrupt loop , used for start trigger, TODO replace this with main Time increment ?

uint16_t delayCount = 0; // incremented by interrupt loop , used for start and end trigger

bool underRun = false;

// uint32_t rewardCounter = 0; // counts down reward duration depreciated, changed to dOutCounter

uint32_t dOutCounter[dOutNum][2]; // digital out timing per channel col 0 =current count down, 1= duration setting

int32_t dOutStart[dOutNum];  // sets  a start time for dOuts if not instant

// uint32_t freqCheck = 0; // can be used to check timing with oscilliscope

uint32_t usedMicros = 0;

// Data Logging Things: -----------------

 volatile uint32_t pulseCount = 0;  //count imgaing frames

// // d) Flyback Signal
// volatile uint32_t flybackVal = 0;   flyback logic  TODO add, can this be bool?

uint32_t outSignals[3]; // used to report  digital or analog outs, see fuction for encoding


// EXPERIMENT / STATE machince logic vars ------------------------

uint8_t lastState = knownValues[0];  // We keep track of current state "knownValues[0]" and the last state to inform if something is run the first time
bool startSession = 0; // Used for exiting task
bool headerStates[] = {0, 0};   // to keep track of first run of state size of stateCount  assuming 3 for FB
uint8_t stateCount = 2;  // number of states  

bool ranOnce = false; // used on boot to trigger clock / flexitimer sychronizer functrion (syncTimers())
bool ranTwice = false; // prevents re-sync on first valid loop

// bool rewarding = 0;  // keeps track of reward state on/off, depreciated use dOut

bool dOutTracker[dOutNum][2]; // digital output trigger (col 0) and on/off state (col 1) 

// ---------  PERIPHERAL CHAIPS AND OTHER I/O PIN ASSIGNMENTS -----------

MCP4922 mDAC0(11,13,10,35);    // (MOSI,SCK,CS,LDAC) 
MCP4922 mDAC1(11,13,37,34);    // (MOSI,SCK,CS,LDAC) 
MCP4922 mDAC2(11,13,36,33);    // (MOSI,SCK,CS,LDAC)

#define syncPin 31   // Trigger other things like a microscope and/or camera
#define syncPinPi 30  // send a session start to a Pi
#define sessionOver 29  // send a session over signal
#define encoderPin A16  // data input for rotation encoder - ADDS 20um PER READ

//  analog input mostly  NOT USED because ~ ADDS 20um PER READ
#define genA0 A17        // ~73um per loop w/ encoder +1 analog reads
// #define genA1 A0     // ~94um per loop w/ encoder +2 analog reads
// #define genA2 A1 //   over run @ 10kHz

#define lickPin 28  // data out for capacitive touch sensor
#define scaleDout 25 // data input from HX711 scale amp
#define scaleSck 24 // serial clock for HX7811 scale amp
//#define rewardPin 32 // connect to servo for reward -depreciated nor dOUtPin 0
const uint8_t dOutPins[] = {32,2,3,4};  //these are indexed with dOutNum and reward rolled in at ind 0
// TODO add air suck pin

// interrupt pins TODO - not yet in use, implement frame counter then Flyback
 #define framePin 8   // count scope frames pin assignment
// #define yGalvo  9    // count flyback interrupt pin assignments


HX711 scale(scaleDout, scaleSck); // start scale


void setup() {
  // start coms--------------
   Serial.begin(115200);
   SPI.begin();
   
   // Analog In/Out
  analogReadResolution(12);    // save 3um per read by reducing to 10, 10um by reducing to 8
  analogWriteResolution(12);  // mcp DAC default but ok to make explicit

  // insure pin mode -----------

  pinMode(syncPin, OUTPUT);
  pinMode(syncPinPi, OUTPUT);
  pinMode(sessionOver, OUTPUT);
  pinMode(lickPin, INPUT);
  //pinMode(rewardPin, OUTPUT); // depreciated now dOUtPin 0
  pinMode(dOutPins[0], OUTPUT);
  pinMode(dOutPins[1], OUTPUT);
  pinMode(dOutPins[2], OUTPUT);
  pinMode(dOutPins[3], OUTPUT);
    

   
  // zero outputs---------------
  mDAC0.Set(0,0);
  mDAC1.Set(0,0);
  mDAC2.Set(0,0);
  digitalWrite(syncPin, LOW);   // zero the trigger after trigtime counted
  digitalWrite(syncPinPi, LOW); // zero aux trigger  
  digitalWrite(sessionOver, LOW); // zero session end indicator  
  //digitalWrite(rewardPin, LOW);
  digitalWrite(dOutPins[0], LOW);
  digitalWrite(dOutPins[1], LOW);
  digitalWrite(dOutPins[2], LOW);
  digitalWrite(dOutPins[3], LOW);

  // Interrupts
  attachInterrupt(framePin, frameCount, RISING);  //process to count frames
  // attachInterrupt(yGalvo, flybackStim_On, FALLING); TODO add later

  // --- update scale ---

  scale.set_offset(weightOffset);
  scale.tare();
  scale.set_scale(setVal);
  scale.tare();
  while (! scale.is_ready()) { 
    ; // wait for scale  
  }
  
  
  
  while (!Serial) {
    ; // wait for serial port to connect 
  }
  
   // Start Program Timer
  FlexiTimer2::set(evalEverySample,  1.0 / sampsPerSecond, vStates);
  FlexiTimer2::start();


}

  void loop() {
  // put your main code here, to run repeatedly:

}

void vStates() {

  if (!ranOnce) {
    syncTimers(); // initialize millis and FT loops by syncing the timers for accurate incrementation
  }

  if (ranOnce) {  // otherwise run your timer - based loop once sync'd, don't increment first loop (sync'd to time zero)
    
    incrementTime();
    

    if (!underRun) {
  // ***************************************************************************************
  // **** /Serial Processing:
  // 
  //lastState = knownValues[0]; // used for block state logic

  usedMicros = loopTime;

  // we then look for any changes to variables, or calls for updates
  int curSerVar = flagReceive(knownHeaders, knownValues);   // looks for serial messages, sets known values to message value
                                                            // once per call. returns -1 unless a new value is set, in which
                                                            // case it returns the header index of the set variable

   /*          //Depreciated                                               
   //  This is what allows parameter values with a paired channel value to be pushed under a single serial header.
   if ((curSerVar == 3) || (curSerVar == 4) || (curSerVar == 5) || (curSerVar == 6) || (curSerVar == 7) || 
   (curSerVar == 9) || (curSerVar == 10) || (curSerVar == 11) || (curSerVar == 12) || (curSerVar == 13)) {
   setPulseTrainVars(curSerVar, knownValues[curSerVar], trainReceive, trainReady); //add update bool
   }
   */

   updatePulseTracker(pulseTrainVars, pulseTrainBuffer, trainReady); // check buffer for updates to pulse tracker
  

   // ***************************************************************************************

  // **************************
  // State 0: Boot/Init State
  // **************************
  if (knownValues[0] == 0) {            // Check to see if Boot state is active, if so do the following

    // a) run a header for state 0        
    if (headerStates[0] == 0) {         // If the boot state isn't registered THINGS TO DO ONCE
      resetTimers();                    
      genericHeader(0);                 // Do the things we always do  ONCE: 
      // loopCount = 0;                    // Set the 'in task' timmer to zero   removed/replaced
      delayCount = 0;      
      pulseCount = 0;                   // TODO why? Set the frame counter to zero
      for (int i=0; i<dacNum; i++) {
        pulseTrainVars[i][10]=-2;   // insure no repeat/mistfire incase boot state entered before last pulse train completed                   
      }
       for (int ndx = 0; ndx < dOutNum; ndx++) { //reset dOut Signals, check function will kill pin-outs
        dOutTracker[ndx][0]=0;  //no trigger, function flips on/off dOT[][1]
        dOutCounter[ndx][0]=0;  //no remaining duration
        dOutStart[ndx] = -2;
        digitalWrite(dOutPins[ndx], LOW);        // turn off signal
      }
    }
    // reset session header
    if (startSession == 1) {              // If task session was just exited
      if (delayCount < trigTime) {
          digitalWrite(sessionOver, HIGH);    // send session-over TTL   TODO: Add with pin maping
          delayCount ++;
       }
       else {
          digitalWrite(sessionOver, LOW);   // TODO pin out
          delayCount = 0;
          startSession = 0; // Register not in-session  
        }   
      } 

    // b) body for state 0
    genericStateBody();                     // do the second set of things we always do 
    //dataReport(); //here only for debugging

  }

  // **************************
  // State != 0: (in task)
  // **************************
  if (knownValues[0] != 0) {
    // sent the trigger, start the trigger timer and reset the pulse counters coming out of boot state
    if (headerStates[0] == 1) {  // used to look at delayCount ==0 but changed
     digitalWrite(syncPin, HIGH); // trigger pin out
     digitalWrite(syncPinPi, HIGH); // trigger second computer
      resetTimers();                  // reset the trial time, micros, and rollover
      delayCount = 1;
        for ( int i = 0; i < dacNum; i++) {       
          trainTimer[i] = 0;                  // 
       // set their clock to zero // changed to evaluate whether to reset
          pulseTrainVars[i][0] = 1;
          pulseTrainVars[i][1] = 1;
          pulseTrainVars[i][7] = 0;
        }
    }
    // This ends the trigger.
    else if (delayCount < trigTime) {
      delayCount++;   
    }
    // This ends the trigger.
    else {
      digitalWrite(syncPin, LOW);   // zero the trigger after trigtime counted
      digitalWrite(syncPinPi, LOW); // zero aux trigger  
    }

    // This can test loop timing with a ocilliscope
    /*
    if (freqCheck < 1000) {
      digitalWrite(0, HIGH);
      freqCheck++;
    }
    else if ( freqCheck < 2000) {
      digitalWrite(0, LOW);
      freqCheck++;
    }
    else {
      freqCheck = 0;
    }
    */
    
    //******************************************
    //@@@@@@ Start Non-Boot State Definitions.
    //******************************************

    // **************************
    // State 1:Active / Experiment State
    // **************************
    if (knownValues[0] == 1) {
      // run this stuff once per session
      if (startSession == 0) {
        startSession = 1;   // used for session over
      }
      if (headerStates[1] == 0) {
        genericHeader(1);
        
      }
      stimGen(pulseTrainVars);                            
      setAnalogOutValues(analogOutVals, pulseTrainVars);    // leave for now, to add FB, FB command
                                                          // must be sent specifying channel requires
                                                          // dacNum size bool arry pushed by channel func
                                                          // here if FB dacVal = 0 else PTV7
                                                          // in FB interrupt function, change the wrte
                                                          // dac list to series of local array mapped 
                                                          // 'if' s that look at the bool to fire if true
      genericStateBody();
    }

   // 

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
    usedMicros = loopTime-usedMicros;
    dataReport();
    loopCount++; //counts loops - deemed useful for python system
  }

    /*
    Serial.print(trialTime);
    Serial.print(",");
    Serial.print(loopedTime);
    Serial.print(",");
    Serial.print(loopTime);
    Serial.print(",");
    Serial.println(rolledHours);
    */
    
  
    

    } //end underRun check
    else {
      underRunKill(); // print underRun error
    }
  }  // End main body (ranOnce = true logic)


} // End vStates


    
// ------------ FUNCTIONS ----------------------------

// ------------ General ------------------------------

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
    if (pulseTrainVars[i][9]==1) {  //fire on state change
        trainTimer[i] = 0;                 
       // set their clock to zero // changed to evaluate whether to reset
        pulseTrainVars[i][0] = 1;
        pulseTrainVars[i][1] = 0;
        pulseTrainVars[i][7] = 0;
        pulseTrainVars[i][10] = trialTime;  // fire at current time
        
   // Theis is a good check when states change
   // remap these based on new pulse tracker if needed but current is 0,1,7,9 pulse/baseline, stop bit, write val, up.down     
    }
    else {
      pulseTrainVars[i][7] = 0; // check to misfire (as below) and good for output reporting, should always be set actively by stimGen
    }
   // else {
   //   trainTimer[i] = 0;
   //   //all train timers need to reset at state change  removed to allow persistance through multiple states, also reset at start ses
                                          
   // }
  }
  // set analog output values to 0.
  // this is a decent check to misfires 
   
  analogOutVals[0] = 0;
  analogOutVals[1] = 0;
  analogOutVals[2] = 0;
  analogOutVals[3] = 0;
  analogOutVals[4] = 0;

 
   
  // pollToggle();  // !!! JUST DISABLING THIS, NO CURRENT NEED !!!
  // d: reset state timer. // done in state to allow pulse persistance
  //stateTime = 0;            // TODO: use only if more than one experimental state needed
}
// ******************************************************

// ******************************************************
void genericStateBody() {
  // TODO add back all I/O pins
  lickSensorValue = digitalRead(lickPin);              // Read for licks 
  
      // Read analog inputs         // !! NOT using these right not - each adds 20um per loop
  genAnalogInput0 = analogRead(genA0);                // ~73um per loop w/ encoder +1 analog reads
  // genAnalogInput1 = analogRead(genA1);             // ~94um per loop w/ encoder +2 analog reads
  // genAnalogInput2 = analogRead(genA2);             //over run @ 10Khz                  
              
  //pollToggle();                                         // removed because No current use, but code to add back below
  checkDOut();
  pollRelays();                                           // look for manual hardware commands
  encoderAngle = analogRead(encoderPin);         // read rotation encoder
  writeAnalogOutValues(analogOutVals);                  // Set the pins to values in the Analog out values array
  updateOutSignals();                        //this encodes a report of  DAC out, and dOut values          
   
   if (scale.is_ready()) {                              // TODO Add Pinout
    //scaleVal = scale.get_units() * 22000;             // TODO test this conversion
    scaleVal = scale.get_units(1);
    // this scale factor gives hundreths of a gram as the least significant int
    knownValues[14] = scaleVal;
  } 
 ;
}

// ******************************************************


// ------------ TIMING -------------------------------

void underRunKill() {
  Serial.println("Clock under run -> kill"); // print underRun error
  underRun = 0; // try to restart
  knownValues[0] = 0; // try to park in boot state
  resetTimers(); 
  ranOnce = false; 
  ranTwice = false;
}

// ************************************************ 
void rollOverVars() {
  // adjust stim parameters for 5 hour roll over
  for (int i = 0; i < dacNum; i++) {
  if ((pulseTrainVars[i][1]==0) && (pulseTrainVars[i][1]!=-2)) { // if the channel is on

    pulseTrainVars[i][11] = pulseTrainVars[i][11] - (1800000000 - pulseTrainVars[i][10]); // Adjust to only duration left to run 
     
    pulseTrainVars[i][10]=0;    // set start to zero along with clock, assumes clock cycles divide evenly into seconds||hours
  }
  
  else if (pulseTrainVars[i][10] >= 1800000000) { // if the start time is > 5hours

    pulseTrainVars[i][10] = pulseTrainVars[i][10] - 1800000000; // if channel not yet fired set start to start minus 5 hours
    
  }

  if (pulseTrainBuffer[i][6] >= 1800000000) { // if a waiting variable start time is > 5hours

    pulseTrainBuffer[i][6] = pulseTrainBuffer[i][6] - 1800000000; // set it to start minus 5 hours
    
  }
  }
  for (int ndx = 0; ndx < dOutNum; ndx++) {
    if (dOutTracker[ndx][1]==1) { // if the digital channel is on
      dOutCounter[ndx][0] = dOutCounter[ndx][0] - 1800000000; //duration vs trialtime already calculated so subtract 5hrs
      if (dOutCounter[ndx][0]<0){
        dOutCounter[ndx][0]=0; // ensure value is non-negative
      }
    }
    if (dOutStart[ndx] >= 1800000000) { // if impending digital signals are scheduled past the roll over
      dOutStart[ndx] = dOutStart[ndx] - 1800000000; // subtract the 5 hours, this paramn not used if channel is active so no more needed
    }
  }
}

// ************************************************ 


// ************************************************ 
void resetTimers() {
  // Reset clocks 
  loopTime = 0;
  rolledHours = 0;
  trialTime = 0;
  loopCount=0;
}
// ************************************************ 

// ************************************************  
void syncTimers() {
      // on first loop set timers to 0, to keep accuate register for second loop 
      loopTime = 0;
      ranOnce = true; // breaks out of first loop logic
    }
    
// *********************************************

void incrementTime() {

  if (!ranTwice) {      //time synced and time is zero
    ranTwice = true;
  }
  else {      // increment time

    timeNow = loopTime/10; // store loop to loop time in 10(us)
    
    loopedTime = (timeNow) + (360000000 * rolledHours); // convert count to 10um units, add rolled over hours

    if (timeNow < 360000000) {   // while accumulator is less than an hour


          // Check that time keeping incremented by the loop is accurate to 10kHz (less than 100us drift)
      if ( abs( (microsPerLoop) - ( loopedTime - trialTime )  ) <= 10 ) {
    
       trialTime = trialTime + microsPerLoop; // advance time by one loop time in 10us units
       // stateTime = stateTime + microsPerLoop; // TODO add only if more than one experimental state needed

       for (int i=0; i < dacNum; i++) {               //increment all DAC timers
        trainTimer[i] = trainTimer[i] + microsPerLoop;
       }
      }
      else {
       underRun = true;
        Serial.println("Clock under run -> kill");
      }
      
    }
    else if (rolledHours < 4) { // else roll over to zero if at less than 5 hours

      // Check that time keeping incremented by the loop is accurate to 10kHz (less than 100us drift)
      if ( abs( (microsPerLoop) - ( loopedTime - trialTime ) ) <= 10 ) {
    
        loopTime = 0;
        rolledHours++;
        trialTime = trialTime + microsPerLoop; // advance time by one loop time in 10us units
        // stateTime = stateTime + microsPerLoop; // TODO add only if more than one experimental state needed

        for (int i=0; i < dacNum; i++) {               //increment all DAC timers
        trainTimer[i] = trainTimer[i] + microsPerLoop;
       }
    
      }
      else {
        underRun = true;
        Serial.println("Clock under run -> kill");
      }

    }
    else  { // else roll all to zero and update variables

      // Check that time keeping incremented by the loop is accurate to 10kHz (less than 100us drift)
      if ( abs( (microsPerLoop) - ( loopedTime - trialTime ) ) <= 10 ) {
    
        rollOverVars();
        loopTime = 0;
        rolledHours = 0;
        trialTime = 0;
        loopCount=0;
        // stateTime = 0; // TODO add only if more than one experimental state needed
       

        for (int i=0; i < dacNum; i++) {    //increment all DAC timers should not roll but if does should have no effect
        trainTimer[i] = trainTimer[i] + microsPerLoop;
       }
    
      }
      else {
        underRun = true;
        Serial.println("Clock under run -> kill");
      }

    }

  }
}

// ***************************    


// ------------ SERIAL SEND / RECIEVE -----------------

//*******************************************************
/* Flag recieve changed to return specific non-variable integer (-1) if called w/o a serial buffer or if a paremeter 
 *  request is recieved. Returns header number only when paremeter change is rececieved. Also the max receieved int size limit
 *  was changed to reflect 32 bit range (10 digits plus null) and the exceed case was changed to an error message escape with
 *  a buffer flush.
 *  
 *  NOTE: This WILL bug on slow serial (i.e. genuine Arduino / other slow TTL serial 9600 baud definitey bugs) unless
 *  blocking term is added - not sure how though as flexi calls as an interrupt regardless of loop comlpetion 
 */

int flagReceive(char varAr[], int32_t valAr[]) {
  static byte ndx = 0;          // finds place being read
  char endMarker = '>';         // Designates recieve message
  char feedbackMarker = '<';    // Designates send/requested message
  char rc;                      // received variable header character
  uint32_t nVal;                // Variable value (I think)
  const byte numChars = 12;     // Size of string to store each digit of int 32 bit 10 digits + 1 null + 1 channel
  char writeChar[numChars];     // Make the string of this size
  char valChar[numChars-1];     // used to rip value for DAC channels
  char chanChar[2];             // used to rip channel for DAC updates
  int selectedVar = -1;          // Returned index of kV header of recieved variable for Pulse update TODO: pulses be updated here?
  static boolean recvInProgress = false;  // Bit indicating in recieve PERSISTS THROUGH CALLS TO THIS FUNCTION !
  bool newData = 0;               // Bit breaks loop when true, fliped when variable recieve is finished 
  int32_t negScale = 1;         // used to convert the ABS value of the variable recieved to negative when '-' read out
  uint8_t selectedChan =100;
  int vDex = 0;

  while (Serial.available() > 0 && newData == 0) {    // set a loop to run if there is a bit in the buffer until the message is read
    rc = Serial.read();                               // read the (next) bit and store it to RC

    //Find the header - turn on receive when found
    if (recvInProgress == false) {                // If this is a new message (variable; not partially read)
      for ( int i = 0; i < knownCount; i++) {   // Loop through the number of knownValues variables
        if (rc == varAr[i]) {                   // if the read byte == the looped to (any) kV Header char
          selectedVar = i;                      // set the return value to variable's the index of the kV Header 
          recvInProgress = true;                //  indicate the message is being read by flipping rcvInProg
          break;                                // added for efficency and prevent redundance bugs 
        }
      }
    }

    else if (recvInProgress == true) {        // if the message is in the process of being read

      // If you find the stop bit turn receive stuff, indicate newData/break loop, updata Array value, return variable index
      if (rc == endMarker ) {                 // if the bit is a revieve message stop 
        writeChar[ndx] = '\0';                // terminate the string by writing a non-integer to writeChar at present ndx
        

        // new implimentation of channel parse for variable size limit issue
        if ((selectedVar == 1) || (selectedVar == 2) || (selectedVar == 3) || (selectedVar == 4) 
        || (selectedVar == 5) || (selectedVar == 6) || (selectedVar == 7) || (selectedVar == 9) 
        || (selectedVar == 10) || (selectedVar == 11) || (selectedVar == 12) || (selectedVar == 13)) {   
          // look for output channel headers 

        // if Output channel recvd'....
        
          chanChar[0]=writeChar[ndx-1]; // record the channel number
          chanChar[1]='\0';
          while (vDex < ndx-1) {
            valChar[vDex]= writeChar[vDex]; // copy the value (only) to another string array
            vDex++;
          }
          valChar[vDex]='\0';

           selectedChan = uint8_t(String(chanChar).toInt());  //convert the Char to string and then to integer and call it nVal

           nVal = int32_t(String(valChar).toInt());  //convert the Char to string and then to integer and call it nVal
           valAr[selectedVar] = nVal * negScale; // process ABS value with +/- read and set the (global) array value equal to it
        
          // route data to correct management function
          if ((selectedVar == 1) || (selectedVar == 2)) {
            setDOutTracker(selectedVar, selectedChan-1, valAr[selectedVar],dOutTracker,dOutCounter,dOutStart);
          }
          else {
            setPulseTrainVars(selectedVar, valAr[selectedVar],trainReceive, trainReady, selectedChan); //add update bool
          }
           
           recvInProgress = false;               // indicate finished message by flipping rcvInProg
           ndx = 0;                              // reset the index 
           newData = 1;                          // indecate that the message is read (and ready)
           return selectedVar;                   // message is done, return the variable's array index and break
          
        } // end look for output channels
        else {

          recvInProgress = false;               // indicate finished message by flipping rcvInProg
        ndx = 0;                              // reset the index 
        newData = 1;                          // indecate that the message is read (and ready)
        nVal = int32_t(String(writeChar).toInt());  //convert the Char to string and then to integer and call it nVal
        valAr[selectedVar] = nVal * negScale; // process ABS value with +/- read and set the (global) array value equal to it
        return selectedVar;                   // message is done, return the variable's array index and break
          
        }
        
      }
      // If you find feedback marker close loop and read stuff, read the array for values, print them to serial
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

      // If the Byte is a digit, write it to the string increment the string index
      else if (rc != feedbackMarker && rc != endMarker && isDigit(rc)) { 

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
    } // End recieve in progress - post header process loop
  }   // end WHILE Loop
  
  return selectedVar;                   // prevent returning "1" or "0"
}
//*******************************************************

//*******************************************************

void setPulseTrainVars(int recVar, int parsedValue, bool tRec[], bool tRed[], uint8_t parsedChan) {
  // depreciated
  // int parsedChan = recVal % 10;  // ones digit is the channel     TODO May need to make '100' for more channels
  // int parsedValue = recVal * 0.1; // divide by 10 and round up   take care of round bug
  // int parsedValue = (recVal-parsedChan) * 0.1; // remove 10's digit


  // Find whether buffer process is being controlled and flip the bit
  if (recVar == 3) {
    if ((tRec[parsedChan - 1] == true) && (parsedValue == 0)) { // check to see if buffer is buffer data is ready
      tRed[parsedChan - 1] = true;       // if so, flag channel ready
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
void updateOutSignals() {
 // creates a code to report all dac and digi outs using three uint32#s for efficient computer side storage and serial com

 // Coding compresses right to left (units:billions) and across breaks, most to least likely active, saving leading "0" bits
 // compressed to three breaks/variables to allow for compression of 9 to 10 data streams to 3 uint32 lists
 // this sacrifices higher number of (less likely printed) serial bits to save memory mapped struct size and ram PC side.
 // compression structure, each one uint32 value composed of: 
 // variable 1: [bils:mils[dac4],hundThous:hunds[dac1],tens:units[dout2,dout0]
 // variable 2: [bils:mils[potential dac5],hundThous:hunds[dac2],tens:units[dout3,dout1]
 // variable 3: [n/a (extra 2),TenMils:TenThous[dac3],thous:units[dac0]
 // This must be called after checkDOut() and stimGen() to have apprpriate output values and before dataReport()
 // reporting pTV[][7] becaue for any analog out signal it should be set in stim gen, analogoutvals are not always used (FB)

 uint32_t outSig1 = (1000000 * pulseTrainVars[4][7]) + (100 * pulseTrainVars[1][7]) 
            + (10 * ((int)dOutTracker[2][1])) + ((int)dOutTracker[0][1]);

 uint32_t outSig2 = (100 * pulseTrainVars[2][7]) + (10 * ((int)dOutTracker[3][1])) 
              + ((int)dOutTracker[1][1]);   // potential dac add space with (1000000 * pulseTrainVars[5][7]) 

 uint32_t outSig3 = (10000 * pulseTrainVars[3][7]) + (pulseTrainVars[0][7]); //room for 2 more bytes

 outSignals[0] = outSig1;
 outSignals[1] = outSig2;
 outSignals[2] = outSig3;
}
// ******************************************************

// ***************************

void dataReport() {
 
  Serial.print("tData");
  Serial.print(',');
  Serial.print(loopCount);
  Serial.print(',');
  Serial.print(trialTime);
  Serial.print(',');
  Serial.print(knownValues[0]); //state
  Serial.print(',');
  Serial.print(knownValues[14]);  //load cell
  Serial.print(',');
  Serial.print(lickSensorValue); // lick sensor
  Serial.print(',');
  Serial.print(encoderAngle);     //rotary encoder value
  Serial.print(',');
  Serial.print(pulseCount);     // session frames counted
  Serial.print(',');
  Serial.print(genAnalogInput0);
  Serial.print(',');
  Serial.print(genAnalogInput1);
  Serial.print(',');
  Serial.print(outSignals[0]);
  Serial.print(',');
  Serial.print(outSignals[1]);
  Serial.print(',');
  Serial.println(outSignals[2]);
  /*
   
  Serial.print(',');
  Serial.print(dOutStart[0]);
  Serial.print(',');
  Serial.print(dOutCounter[0][0]);
  Serial.print(',');
  Serial.print(dOutCounter[0][1]);
  Serial.print(',');
  Serial.print(dOutTracker[0][0]);
  Serial.print(',');
  Serial.print(dOutTracker[0][1]);
  Serial.print(loopTime);
  Serial.print(',');
  
  Serial.print(',');
  Serial.print(pulseTrainVars[0][3]);
  Serial.print(',');
  Serial.print(scaleVal);
  Serial.print(',');
  Serial.print(encoderAngle);
  Serial.print(',');
  Serial.print(usedMicros);
  Serial.print(',');
  Serial.println(usedMicros);
   */
}

// ************************************
//dOutCounter[dOutNum][2]  col 0 =current count down, 1= duration setting
// dOutTracker[dOutNum][2]  digital output trigger (col 0) and on/off state (col 1) 
// r/1: reward duration (if solenoid) or volume (if pump)  / Must not exceed 30 min - not sure if this is true anymore
// g/2: deliver reward

    // takes serial message values and applies them to digital out tracker values
    
void setDOutTracker(int varType, uint8_t varChan, int32_t varVal, bool outTracking[][2], 
                        uint32_t outCounting[][2], int32_t dOutSt[]) {
  if ((varType == 1) && (varVal>0)) { // reward duration update 
    
    int trackCountDiff = varVal - outCounting[varChan][1]; // store difference incase countdown update needed
    
    outCounting[varChan][1] = varVal;   // update duration setting
    
    if (outTracking[varChan][1] == 1) { // if the channel is actively sending, however, update the countdown too
      if ((outCounting[varChan][0] + trackCountDiff)>=0) {        // confirm that the update won't go negative
        outCounting[varChan][0] = outCounting[varChan][0] + trackCountDiff;   // if so add difference to countdown
      }
      else {                          // else if it would have gone negative
        outCounting[varChan][0]=0;     // set the count to zero
      }  
    }
  }
  else if (varType == 2) { // if triggering channel or toggling it off
    if (varVal == -2) {
      outCounting[varChan][0] = 0; //to toggle off, counter needs to terminate, active bit is flipped off elsewhere
      outTracking[varChan][0] = 0; // check no trigger conflict
      dOutSt[varChan] = -2; // check no trigger conflict
    }
    else if (varVal == -1) { // if triggering the channel, just flip the trigger bit on
      outTracking[varChan][0] = 1;
      dOutSt[varChan] = -2; // over rides schedule - check no trigger conflict
    }
    else if (varVal >= 0) {  // else if scheduling a trigger, 
      dOutSt[varChan] = varVal; // set the trigger time
    }
  }
}
// ************************************

// --------------- BEHAVIOR / EXPERIMENT FLOW / I/O --------------


// ******************************************************
void checkDOut() {      // called per loop, evaluates if digital signal is initiated, or just running, then calls pin-setter
      for (int i = 0; i < dOutNum; i++) {   // for all digital outputs
        if ((dOutTracker[i][0] == 1) && (dOutTracker[i][1] == 0)) { // if triggering new signal
          sendDOut(i, (bool)1);                   // resolve signal start pinout behavior
          dOutTracker[i][0] = 0;                  // flip off trig signal
          dOutStart[i] = -2;                     // over rides schedule - check no trigger conflict
        }
        else if ((dOutStart[i]>=0) && (trialTime >= dOutStart[i]) && (knownValues[0] != 0)) {
          sendDOut(i, (bool)1);                   // resolve signal start pinout behavior
          dOutTracker[i][0] = 0;                  // flip off trig signal - redundant double check
          dOutStart[i] = -2;                     // erase used waiting trigger
        }
        else if ((dOutTracker[i][0] == 1) && (dOutTracker[i][1] == 1)) { //if chan trigger stil up and already sending (can't trig twice)
          sendDOut(i, (bool)0);                      // resolve pinout behavior
          dOutTracker[i][0] = 0;                    // flip off trig signal 
        }
        else if ((dOutTracker[i][0] == 0) && (dOutTracker[i][1] == 1)) {  //if sending
          sendDOut(i, (bool)0);                    // resolve pinout behavior
        }
      }
    }

    // r/1: reward duration (if solenoid) or volume (if pump)  / Must not exceed 30 min - not sure if this is true anymore
// g/2: deliver reward
//dOutCounter[dOutNum][2]  col 0 =current count down, 1= duration setting
// dOutTracker[dOutNum][2]  digital output trigger (col 0) and on/off state (col 1) 

// ******************************************************

// ******************************************************
void sendDOut(int outChan, bool startR) {  //evaluates digital out timine and sets pin no need to loop here (done by caller)
      if ((dOutTracker[outChan][1] == 1) && (trialTime >= dOutCounter[outChan][0])) { // if duration completed
        digitalWrite(dOutPins[outChan], LOW);        // turn off signal
        dOutTracker[outChan][1] = 0;                // zero out all tht tracking
        dOutCounter[outChan][0] = 0;
      }
      else if ((startR==true) && (dOutTracker[outChan][1]==0)) {  //if start signal trig sent set tracker/counter and send pin high
        digitalWrite(dOutPins[outChan], HIGH);        // channel fire pin
        dOutTracker[outChan][1] = 1;                  // track sending active
        dOutCounter[outChan][0] = trialTime+dOutCounter[outChan][1]; // set count down equal to time+duration        
      }
      else if ((dOutTracker[outChan][1] == 1) &&  (trialTime < dOutCounter[outChan][0]) ) { //If sending and incomplete duration
        digitalWrite(dOutPins[outChan], HIGH);       // ensure pin high
        dOutTracker[outChan][1] = 1;                 // ensure track sending active 
      }
      

      //Note dOutTracker[any][1] should NEVER be set outside of this function to maintain correct track of
      // dOut and serial sending behaviors. To kill an out, set dOutCounter[any][0] equal to zero ahead of
      // the checkDOut() call in genericBody()
    }

// ******************************************************

// ******************************************************

void pollRelays() { // Looks for  hardware logic input commands currently only adding reward
  //bool rTrig;
  bool rRwd;
  // rTrig = digitalRead(syncMirror);
  // rRwd = digitalRead(rewardIn);    // TODO  assign reward pin mirror input pullup mode
  //if (rTrig == 1) {
  //  digitalWrite(syncPin, HIGH);  // do not add back unless needed
  //  delay(5);
  //  digitalWrite(syncPin, LOW);
  //}
  if (rRwd == true) {
    sendDOut(0, (bool)1);
  }
}

// ******************************************************   


// ****************************************************************
void setAnalogOutValues(uint32_t dacVals[], int32_t pulseTracker[][13]) {
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

// ******************************************************

void writeAnalogOutValues(uint32_t dacVals[]) {
  mDAC0.Set(dacVals[0], dacVals[1]);
  mDAC1.Set(dacVals[2], dacVals[3]);
  mDAC2.Set(dacVals[4], 0);     //  one more DAC available if needed dacVals[5]);

  //old for T3.6 w/2 dac chips
 /* analogWrite(DAC1, dacVals[0]);
  analogWrite(DAC2, dacVals[1]);
  mDAC1.Set(dacVals[2], dacVals[3]);
  mDAC2.Set(dacVals[4], dacVals[4]); */     // TO DO Add Pinouts, change all to MCPs
}
// ******************************************************


// ******************************************************

void stimGen(int32_t pulseTracker[][13]) {

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


// everything is run in this loop except 'updateCount' being reset, actually its there too (redundant)
  int i;
  int updateCount = 0;
  for (i = 0; i < dacNum; i = i + 1) { 
// The first thing we need to do is create code to evaluate the start time, train timer and stop bit, edit them, and then if pulsing pass the rest as 'if'
  if (pulseTracker[i][10] != -2) {   // If the channel is not disabled  - evaluate active state, otherwise leave off - state fire flips bit in the header so not tested here
    // state fire tests start time = to trial time, so that needs to be set to '-2' when stop bit is flipped
    
    // ** Test pulse train start conditions
    if (pulseTracker[i][1]==1) {         // If the channel is off - evaluate start conditions and if pulsing initiated start train timer
      if (pulseTracker[i][10] <= trialTime) {   // if start time met - evaluate fire channel
        
        // !!!!!  (pulseTracker[i][10] + pulseTracker[i][11]) < pulseTracker[i][10] part solves clock rollover problem up to 5hr stim
        
        if ((((pulseTracker[i][10] + pulseTracker[i][11]) > trialTime) || ((pulseTracker[i][10] + pulseTracker[i][11]) < pulseTracker[i][10]))
          && ((pulseTracker[i][8] > 0) || (pulseTracker[i][8] == -2))) {
          // if we haven't exceeded pulse time (or rollover) or pulse number limits, and are NOT pulsing, fire channel and set timing to zero
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
      } // end if pt10 trialtime met
      else {
        pulseTracker[i][7] = pulseTracker[i][4]; // set baseline
      }
    } //end if pt1 ==1
    // ** pulse train active, test stop conditions and set stop bit and negative start or execute pulses
    if (pulseTracker[i][1]==0) { // If the channel is active - evaluate turning it off by duration and by pulse count, if the channel shouldn't turn off - pulse tracking
      // if we exceed duration flip off channel, flip start -2 to disable (won't enable by timer)
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
          if (pulseState == 1) {                      // If  it is in a pulse phase
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
        else if (pulseState == 0) {
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
         
       
       } // end of stim waveform generation (else)
        
      }  // END Channel active PT1==0
    }
    else {      // if disable flipped, zero channel

       pulseTracker[i][1] = 1;   // kill channel
        pulseTracker[i][0] = 1;   // reset pulse state
        pulseTracker[i][7] = pulseTracker[i][4];   // dac to baseline
        pulseTracker[i][10] = -2; // turn off channel
      
    }
  }
  }
// ******************************************************

// ------- Interrupts ------------------------------

// ******************************************************
void frameCount() {
  pulseCount++;
}
// ******************************************************

// ******************************************************
// OLD OR UNUSED FUNCTIONS
// ******************************************************

/*
// ---------- May use or not 

// Clock test, put ahead of time increment
if (!ranTwice) {
      loopTime = 3599999800;
    trialTime = 1439999980;
    rolledHours = 3;
    }

*/

// ******************************************************
/*
 * This COULD be rolled into the setPulseTrainVars function as other things will be, ok with seprate for now, might help
 * Takes value read into knownVals, parses it for intended channel and sends channel TTL
 * 
 * TODO test whether "0" and "1" works rather than "1" and "2"
 * TODO CHANGE TO APPROPRIATE KV INDEX
 */
 /* 
  * 
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

  //icrement flybnack val for some reason?
  
  if (knownValues[0] == 8) {

    elapsedMicros pfTime;
    pfTime = 0;
    while (pfTime <= knownValues[16]) {
      stimGen(pulseTrainVars);
      analogWrite(DAC1, pulseTrainVars[0][7]);
      analogWrite(DAC2, pulseTrainVars[1][7]);
      mDAC1.Set(pulseTrainVars[2][7], pulseTrainVars[3][7]);
    }
    analogWrite(DAC1, 0);   // THIS WILL NEED TO CHANGE TO CHECK A BOOL ARRAY BEFORE ACTING ON DAC CHANNEL
    analogWrite(DAC2, 0);
    mDAC1.Set(0, 0);
  }
}

// ******************************************************
void checkReward() {      //change to for i < dOUtNum  // TODO add fiction called by flag Rcv that flips a new kV2 replacement- allows time change
      if ((knownValues[2]==1) && (rewarding == 1)) {  // TODO rewarding to indexed boolean array
        giveReward((bool)0);                      // kV2 changed to an input val and chan# (i-1)
        knownValues[2]=0;   //if already sending, zero the trigger bit TODO this can probably be removed - zero at trigger on
      }
      else if ((knownValues[2]==0) && (rewarding == 1)) {
        giveReward((bool)0);
      }
      else if ((knownValues[2]==1) && (rewarding == 0)) {
        giveReward((bool)1);
        knownValues[2]=0;
      }
    //  else {
    //    giveReward((bool)0); // TODO might be gratuitous and slow program
    //  }
    }

// ******************************************************

// ******************************************************
void giveReward(bool startR) {                // TODO no need to loop here (done above) just have it called with the channel index
      if ((startR==true) && (rewarding==0)) {  // if start reward signal sent indicate rewarding and send pin high
        //digitalWrite(rewardPin, HIGH);        // TODO Pin assignment
        rewarding = 1;
        rewardCounter = trialTime+knownValues[1];
        
      }
      else if ((rewarding == 1) &&  (trialTime < rewardCounter) ) {
        //digitalWrite(rewardPin, HIGH);       // TODO Pin assignment
        rewarding = 1;
      }
      else if ((rewarding == 1) && (trialTime >= rewardCounter)) {
        //digitalWrite(rewardPin, LOW);        // TODO Pin assignment
        rewarding = 0;
        rewardCounter = 0;
      }
    }

// ******************************************************

*/
