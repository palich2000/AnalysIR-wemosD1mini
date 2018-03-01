//* AnalysIR Firmware for ESP8266 via Serial & WiFi
//* Date: 10th July 2017
//* Release: 1.0.3
//* Author: AnalysIR via https://www.AnalysIR.com/
//* License: Creative Commons, Attribution-NonCommercial-ShareAlike 4.0 International
//*      http://creativecommons.org/licenses/by-nc-sa/4.0/
//* Attribution: Please credit the Author on all media and provide a link to https://www.AnalysIR.com/
//* Feedback: We would love to hear about how you use this software & suggestions for improvement.
//* Questions & Issues via: https://IRforum.AnalysIR.com/  for AnalysIR, Shield & IR related queries only (excludes IDE, WiFi & NodeMCU support which should be directed to the appropriate forum)
//* Note: Please use and read in conjunction with the README and the Getting Started Guides/Infographics.
//* Licence: Subject to the above.
//*          Free to use & modify without restriction or warranty for non-commercial uses.
//*          Free to use & modify without restriction or warranty for commercial uses with a registered copy of AnalysIR.
//*          Free to use & modify without restriction or warranty for commercial uses with an original A.IR ESP8226 Shield by  AnalysIR.
//*          For Commercial use or alternative License: Please contact the Author, via the website above
//*          Please acknowledge AnalysIR as the Author in any derivative and include a link to http://www.AnalysIR.com in any publication.
//*
//* Tested with Arduino IDE  1.6.5 / ESP8266 Arduino Core 2.3.0 & A.IR Shield ESP8266 & A.IR Shield ESP8266 Tx (Tx variant has no Rx capability)
//* Assumes user familiarity with ESP8266 NodeMCU & Arduino IDE
/*
ESP8266 NodeMCU GPIO pin mapping
D0   = 16;  //GPIO16
D1   = 5;   //GPIO5
D2   = 4;   //GPIO4
D3   = 0;   //GPIO0
D4   = 2;   //GPIO2 .... often configured for LED (e.g. witty dev board). Also, used for IR Tx on A.IR Shield ESP8266
D5   = 14;  //GPIO14
D6   = 12;  //GPIO12
D7   = 13;  //GPIO13
D8   = 15;  //GPIO15
D9   = 3;   //GPIO3
D10  = 1;   //GPIO1
NB: IMPORTANT - Please verify the pin mapping of your own device before making any connections or powering-up
 */


#include "uart_register.h"

#define IRLEARNER true //set true if you have an IR LEarner connected (TSMP58000), otherwise false
#define DUTYCYCLE 50  //50 or 33 are most common. 50% if Pc powered, 33% if battery powered


//Definitions for Buffers here...size RAM dependent
#define modPULSES 256 //increase until memory is used up, max 256, leave at 256.
#define byte uint8_t
#define boolean uint8_t

//General Definitions
void rxIR_Interrupt_Handler(void);
void rxIR3_Interrupt_Handler(void);
void initDutyCycle(uint8_t dutyCycle);
void initUPWM(uint8_t carrier);
void mark( uint16_t mLen);
void space( uint16_t sLen);

#define IR_Rx_PIN D5   //IR Receiver. //D7/D8 or D1/D2 also available via Jumpers on rear of PCB...consult guide
#define IR_Mod_PIN D6  //IR Learner //D7/D8 or D1/D2 also available via Jumpers on rear of PCB...consult guide
#define IR_Tx_PIN D4  //Actually GPIO12 on ESP8266 (Dont use D4/GPIO2 as it often has the LED attached)
#define IRLEARNER true //set true if you have an IR LEarner connected (TSMP58000), otherwise false
#define ledPin    99   //make sure this pin is not used for any other function..used here as dummy
#define pinRxHIGH digitalRead(IR_Rx_PIN)
#define maxPULSES 1024 //More RAM is available on this ESP8266
//IMPORTANT never let actual LED pin go HIGH on D4...as this is also now the IR Tx Pin.


#define enableIRrx attachInterrupt(digitalPinToInterrupt(IR_Rx_PIN), rxIR_Interrupt_Handler, CHANGE) //set up interrupt handler for IR rx pin  - demodulated signal
#define disableIRrx detachInterrupt(digitalPinToInterrupt(IR_Rx_PIN)) //disable interrupt handler for IR rx pin - demodulated signal
#if IRLEARNER
#define enableIRrxMOD attachInterrupt(digitalPinToInterrupt(IR_Mod_PIN), rxIR3_Interrupt_Handler, FALLING) //Same for IR Learner - modulated signal
#define disableIRrxMOD detachInterrupt(digitalPinToInterrupt(IR_Mod_PIN)) //Same for IR Learner - modulated signal
#else //no interrupts, if learner not connected/configured
#define enableIRrxMOD
#define disableIRrxMOD
#endif

//Baud rate is now fixed to 115200 for all devices, to avoid issues with some platforms
#define BAUDRATE 115200

#define SIGNALGAP 125000 //determines gap between signals (typical range 100000->125000)

//WiFi settings
#define useWIFI true  //<<<<<< Change this to true for WiFi use & configure settings below
#if useWIFI
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266WebServer.h> //server used for Sending IR (data is sent from AnalysIR to ESP)
#include <ESP8266HTTPClient.h>

#define  ipAddressAnalysIR  "192.168.0.233" //enter the IP address of your AnalysIR PC here,only required  if using WiFi
#define mySSID "YOURSSID" //enter your own here,only required  if using WiFi
#define myWiFiPassword "YOURWIFIPASSWORD" //enter your own here,only required  if using WiFi

const int webServerPortAnalysIR = 9449;
char webBuffer[4 * 1024]; //used to send POST request to AnalysIR Web Server

ESP8266WiFiMulti myWiFi;
HTTPClient http;
ESP8266WebServer webServer(webServerPortAnalysIR);

#endif
//...............................................................................................

uint16_t pulseIR[maxPULSES]; //temp store for pulse durations (demodulated)
volatile uint8_t modIR[modPULSES]; //temp store for modulation pulse durations - changed in interrupt

//General variables...volatile when used in ISR
volatile uint8_t state = 127; //defines initial value for state normally HIGH or LOW. Set in ISR.
uint8_t oldState = 127; //set both the same to start. Used to test for change in state signalled from ISR
uint32_t usLoop, oldTime; //timer values in uSecs. usLoops stores the time value of each loop, oldTime remembers the last time a state change was received
volatile uint32_t newMicros;//passes the time a state change occurred from ISR to main loop
uint32_t oldMicros = 0; //passes the time a state change occurred from ISR to main loop
uint16_t countD = 0; // used as a pointer through the buffers for writing and reading (de-modulated signal)
volatile uint8_t countM = 0; // used as a pointer through the buffers for writing and reading (modulated signal)
uint32_t sum = 0; //used in calculating Modulation frequency

uint8_t carrierFreq = 38000; //default
uint8_t DUTY = 0xF0; //50% default
uint32_t sigTime = 0; //used in mark & space functions to keep track of time

//Serial Tx buffer - uses Serial.write for faster execution
uint8_t txBuffer[5]; //Key(+-)/1,count/1,offset/4,CR/1   <= format of packet sent to AnalysIR over serial

void setup() {
  //first disable WiFi as we are only using serial over USB

  Serial.begin(115200);//fixed at 115200 bps for all platforms
  // delay(500);//to avoid potential conflict with boot-loader on some systems
  //while(!Serial);
  txBuffer[4] = 13; //init ascii decimal value for CR in tx buffer

  pinMode(IR_Rx_PIN, INPUT_PULLUP);
  pinMode(IR_Mod_PIN, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);

  pinMode(IR_Tx_PIN, OUTPUT); //IR TX pin as output
  digitalWrite(IR_Tx_PIN, LOW); //turn off IR output initially
  
  initUPWM(carrierFreq); //start off @ 38000

  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println("!AnalysIR www.AnalysIR.com!"); // HELLO STRING - ALL COMMENTS SENT IN !....! FORMAT
  //Dump pinouts to GPIO on this platform
  Serial.println("!Pin-outs"); // //includes comment start/terminator '!' for AnalysIR
  Serial.print("D0: GPIO"); Serial.println(D0);
  Serial.print("D1: GPIO"); Serial.println(D1);
  Serial.print("D2: GPIO"); Serial.println(D2);
  Serial.print("D3: GPIO"); Serial.println(D3);
  Serial.print("D4: GPIO"); Serial.println(D4);
  Serial.print("D5: GPIO"); Serial.println(D5);
  Serial.print("D6: GPIO"); Serial.println(D6);
  Serial.print("D7: GPIO"); Serial.println(D7);
  Serial.println();
  Serial.print("IR Receiver Input pin: GPIO"); Serial.println(IR_Rx_PIN);
#if IRLEARNER
  Serial.print("IR Learner Input pin: GPIO"); Serial.println(IR_Mod_PIN);
#endif
  Serial.print("AIRSHIELD Tx output pin: GPIO"); Serial.println(IR_Tx_PIN);
  Serial.println("!"); //comment start/terminator '!' for AnalysIR

#if useWIFI
  //set up WiFi if configured
  Serial.print("!Connecting to WiFi on SSID: ");
  Serial.print(mySSID);
  myWiFi.addAP(mySSID, myWiFiPassword);
  while (myWiFi.run() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.print(WiFi.localIP());
  Serial.println("!");


  //config webserver to listen for requests
  webServer.on("/analysir.signal", []() {
    irWebSend();
  });//listen for requests from AnalysIR
  webServer.onNotFound ( handleNotFound ); //handle invalid requests.

  webServer.begin();
  Serial.println("!HTTP server started!");
#else
  //  do nothing //
#endif

  //Initialise State
  oldState = digitalRead(IR_Rx_PIN);
  state = oldState;

  //Initialise Times
  oldTime = 0; //init
  newMicros = micros(); //init
  oldMicros = newMicros;

  initDutyCycle(DUTYCYCLE); //we should only need ot set this once

  //following line not required - just reports free RAM on Arduino if there are problems
  //reportFreeRAM(0xFFFF);// report free ram to host always, use max UInt value of 0xFFFF. 8Bit AVRs only
  //

  //turn on interrupts and GO!
  enableIRrxMOD; //set up interrupt handler for modulated IR rx on pin 3 - full modulated signal
  enableIRrx;//set up interrupt handler for IR rx on pin 2 - demodulated signal
  yield(); //avoid watchdog issues etc.

}

void ICACHE_RAM_ATTR loop() {

  delay(0);  //yield();

  usLoop = micros(); //used once every loop rather than multiple calls
  if (oldState != state && countD < maxPULSES) {
    oldState = state;
    if (oldState) { //if the duration is longer than 0xFFFF(65535 uSecs) then multiple repeat pulses are stored, whith LSB used to signal mark or space
      sum = (newMicros - oldMicros); //re-use sum var here, to save RAM (normally used in reportperiod)
      while (sum > 0xFFFF && countD < (maxPULSES - 1) && countD) { //this allows for a mark/space of greater than 65535 uSecs (0xFFFF), ignore first signal
        sum -= 65535;//this assumes the length is not longer than 131070
        pulseIR[countD++] = 65535 | 0x0001; //store for later & include state
      }
      pulseIR[countD++] = sum | 0x0001; //store for later & include state
    }
    else {
      sum = (newMicros - oldMicros); //re-use sum var here, to save RAM (normally used in reportperiod)
      while (sum > 0xFFFF && countD < (maxPULSES - 1) && countD) { //this allows for a mark/space of greater than 65535 uSecs (0xFFFF), ignore first signal
        sum -= 65535;//this assumes the length is not longer than 131070
        pulseIR[countD++] = 65535 & 0xFFFE; //store for later & include state
      }
      pulseIR[countD++] = sum & 0xFFFE; //store for later & include state
    }
    oldMicros = newMicros; //remember for next time

    oldTime = newMicros; //last time IR was received
  }

  if (state && countD > 0 && (countD == maxPULSES ||  (micros() - oldTime) > SIGNALGAP)) { //if we have received maximum pulses or its 100ms since last one
    disableIRrx;  //disable interrupt handler for IR rx   on pin 2 - demodulated signal
    disableIRrxMOD;  //Same for Pin3 - modulated signal

#if useWIFI
    postSignalWiFi();
#else
    reportPulses();
    reportPeriod();//reports modulation frequency to host over serial
#endif
    countD = 0; //reset value for next time
    countM = 0; //reset
    //turn on interrupts and GO!
    enableIRrxMOD; //set up interrupt handler for modulated IR rx on pin 3 - full modulated signal
    enableIRrx;//set up interrupt handler for IR rx on pin 2 - demodulated signal
  }

  //check to see if incomming request from PC to send IR signal
#if useWIFI //
  webServer.handleClient();
#else
  if (!countD && Serial.available() > 0 && Serial.read() == '$') { //$ signifies start of send sequence and ignores any other chars
    //ignore if actively receiving a signal....countD>0

    disableIRrx;  //disable interrupt handler for IR rx   on pin 2 - demodulated signal
    disableIRrxMOD;  //Same for Pin3 - modulated signal

    sendIR(); //process sending of IR signal received from PC

    //reset/enable rx again now that sending is finished
    countD = 0; //reset value for next time
    countM = 0; //reset

    enableIRrx;//set up interrupt handler for IR rx on pin 2 - demodulated signal
    enableIRrxMOD; //set up interrupt handler for modulated IR rx on pin 3 - full modulated signal
  }

  //check for requets from AnalysIR to send IR signals


#endif
  /*  this code only used, for debugging, if we are having problems with available RAM
  reportFreeRAM(200);//report freeram as comment to host if less than 200 bytes
  */



}

void  reportPulses() {
  yield(); //avoid watchdog issues
  for (uint16_t i = 0; i < countD; i++) {
    //the following logic takes care of the inverted signal in the IR receiver
    if (pulseIR[i] & 0x01) txBuffer[0] = '+'; //Mark is sent as +...LSB bit of pulseIR is State(Mark or Space)
    else txBuffer[0] = '-';           //Space is sent as -
    txBuffer[1] = (uint8_t) (i & 0xFF); //count
    txBuffer[3] = pulseIR[i] >> 8; //byte 1
    txBuffer[2] = pulseIR[i] & 0xFE; //LSB 0 ..remove lat bit as it was State
    Serial.write(txBuffer, 5);
    yield(); //avoid watchdog issues
  }
}

void  reportPeriod() { //report period of modulation frequency in nano seconds for more accuracy
#if IRLEARNER
  yield(); //avoid watchdog issues
  uint8_t sigLen = 0; //used when calculating the modulation period. Only a byte is required.
  uint8_t countM2 = 0; //used as a counter, for the number of modulation samples used in calculating the period.
  uint8_t j;
  sum = 0; //reset before using
  for (j = 1; j < (modPULSES - 1); j++) { //i is byte
    sigLen = (modIR[j] - modIR[j - 1]); //siglen is byte
    if (sigLen > 50 || sigLen < 10) continue; //this is the period range length exclude extraneous ones
    sum += sigLen; // sum is UL
    countM2++; //countM2 is byte
    modIR[j - 1] = 0; //finished with it so clear for next time
  }
  modIR[j - 1] = 0; //now clear last one, which was missed in loop

  if (countM2 == 0) return; //avoid div by zero = nothing to report
  sum =  sum * 1000 / countM2; //get it in nano secs
  // now send over serial using buffer
  txBuffer[0] = 'M'; //Modulation report is sent as 'M'
  txBuffer[1] = countM2; //number of samples used
  txBuffer[3] = sum >> 8 & 0xFF; //byte Period MSB
  txBuffer[2] = sum & 0xFF; //byte Period LSB
  Serial.write(txBuffer, 5);

#else //IR Learner not connected...just send end signal
  Serial.println("! Signal End !\r\n");
#endif
  yield(); //avoid watchdog issues

}

void ICACHE_RAM_ATTR rxIR_Interrupt_Handler() { //important to use few instruction cycles here
  newMicros = micros(); //record time stamp for main loop
  state = pinRxHIGH; //read changed state of interrupt pin
}

void ICACHE_RAM_ATTR rxIR3_Interrupt_Handler() { //important to use few instruction cycles here
  //digital pin 3 on Arduino - FALLING edge only

  modIR[countM++] = (uint8_t) micros(); //just continually record the time-stamp, will be mostly modulations
  //just save LSByte as we are measuring values of 20-50 uSecs only - so only need a byte (LSB)
}



void AIR_sendRAW(uint16_t carrier) { //send the signal stored in pulseIR buffer
  ESP.wdtFeed(); //avoid watchdog issues
  sigTime = micros(); //keeps rolling track of signal time to avoid impact of loop & code execution delays
  if (carrierFreq != carrier) initUPWM(carrier); //only need to do this if carrier changes
  for (uint16_t i = 0; i < countD; i++) {
    mark(pulseIR[i++]); //also move pointer to next position
    if (i < countD) { //check we have a space remaining before sending it
      space(pulseIR[i]); //pointer will be moved by for loop
    }
  }
}

void mark( uint16_t mLen) { //uses sigTime as end parameter
  if (mLen & 1) sigTime += mLen + 0x10000; //mark ends at new sigTime..allow for extra long signal
  else sigTime += mLen; //mark ends at new sigTime

  uint32_t startTime = micros();
  uint32_t dur = sigTime - startTime; //allows for rolling time adjustment due to code execution delays

  if (dur == 0) return;

  uint16_t cycleCount = dur / ((1000 + carrierFreq / 2) / carrierFreq); // get number of cycles & do rounding with integer maths
  ESP.wdtFeed(); //avoid watchdog issues
  while (cycleCount) {
    // while (true) { //send continuous carrier, for testing, signal generator or just generic PWM
    Serial1.write(DUTY); //write a character to emulate carrier, character value determines duty cycle.
    --cycleCount;
    if ((cycleCount ^ 0x1F) == 0)ESP.wdtFeed(); //avoid watchdog issues
  }

  while ((micros() - startTime) < dur) {} //just wait here until time is up

}

void space(uint16_t sLen) { //uses sigTime as end parameter
  if (sLen & 1) sigTime += sLen + 0x10000; //mark ends at new sigTime..allow for extra long signal
  else  sigTime += sLen; //space ends at new sigTime

  uint32_t startTime = micros();
  uint32_t dur = sigTime - startTime; //allows for rolling time adjustment due to code execution delays

  if (dur == 0) return;

  uint16_t cycleCount = 0; //
  ESP.wdtFeed();  //avoid watchdog issues
  while ((micros() - startTime) < dur) { //just wait here until time is up
    if ((cycleCount++ ^ 0x1f) == 0)ESP.wdtFeed(); //avoid watchdog issues
  }
}

void initDutyCycle(unsigned char dutyCycle) {
  //now do Duty cycle - we simply set the character to be sent, which creates the duty cycle for us.
  switch (dutyCycle) {
    case 50  : //50%
      DUTY = 0xF0;
      break;

    case 40  : // 40%
      DUTY = 0xF8;
      break;

    case 30  : // 30%
      DUTY = 0xFC;
      break;

    case 20  : // 20%
      DUTY = 0xFE;
      break;

    case 10  : // 10%
      DUTY = 0xFF;
      break;

    default : // 50% for any invalid values
      DUTY = 0xF0;
      break;
  }
}

void initUPWM(unsigned char carrier) { // Assumes standard 8-bit Arduino, running at 16Mhz
  //supported values are 30, 33, 36, 38, 40, 56 kHz, any other value defaults to 38kHz
  //duty cycle is limited to 50, 40, 30, 20, 10 % - other values will be set to 40%
  if (carrier == carrierFreq) return; //nothing to do if it is the same as last time
  switch (carrier) { // set the baud rate to 10 time the carrier frequency

    case 20 ... 60  : // kHz
      Serial1.begin(carrier * 10000);
      break;

    //default is 38kHz
    default : //If not one of the supported frequencies, the ndefault t0 38kHz
      Serial1.begin(380000);
      break;
  }
  //The following inverts the UART1 output on ESP8266..refer to Register definitiions for explanation.
#define SET_PERI_REG_MASK(reg, mask)   WRITE_PERI_REG((reg), (READ_PERI_REG(reg)|(mask)))
  SET_PERI_REG_MASK(UART_CONF0(UART1) , BIT22);

  carrierFreq = carrier;
}

#if useWIFI

void handleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += webServer.uri();
  message += "\nMethod: ";
  message += ( webServer.method() == HTTP_GET ) ? "GET" : "POST";
  message += "\nArguments: ";
  message += webServer.args();
  message += "\n";

  for ( uint8_t i = 0; i < webServer.args(); i++ ) {
    message += " " + webServer.argName ( i ) + ": " + webServer.arg ( i ) + "\n";
  }

  webServer.send ( 404, "text/plain", message );
}

void postSignalWiFi(void) {

  if ((myWiFi.run() == WL_CONNECTED)) {
    //generate RAW signal to send to server, buy filling bupper
    yield(); //avoid watchdog issues
    char *p = webBuffer; //used as pointer offset into web POST request payload
    // p += sprintf( p, "%05d %03d LIR: %02X ", beaconCount++, WiFi.localIP()[3], ir.carrier); //include beacon count at start of message

    //first add in Carrier frequency

    yield(); //avoid watchdog issues

    uint8_t modLen = 0; //used when calculating the modulation period. Only a byte is required.
    uint8_t countM2 = 0; //used as a counter, for the number of modulation samples used in calculating the period.
    uint8_t j;
    sum = 0; //reset before using
    for (j = 1; j < (modPULSES - 1); j++) { //i is byte
      modLen = (modIR[j] - modIR[j - 1]); //siglen is byte
      if (modLen > 50 || modLen < 10) continue; //this is the period range length exclude extraneous ones
      sum += modLen; // sum is UL
      countM2++; //countM2 is byte
      modIR[j - 1] = 0; //finished with it so clear for next time
    }
    modIR[j - 1] = 0; //now clear last one, which was missed in loop

    if (countM2 > 0) { //avoid div by zero = nothing to report
      //sum =  sum * 1000 / countM2; //get it in nano secs
      sum =  ((1000000 * countM2) + sum / 2) / sum; //get it in kHz
    }
    yield(); //avoid watchdog issues

    //now we have the option to submit the signal in RAW or AIR format
    //
#define RAWFORMAT false //set to true for RAW format or false for AIR FORMAT
#define AIRFORMAT !RAWFORMAT //dont change this line

#if RAWFORMAT
    p += sprintf( p, "format=raw&channel=1&CarrierHz=%d", sum);
    p += sprintf( p, "&analysir=Raw (%d): ", countD - 2); //first slot is ignored also.
    for (uint16_t i = 1; i < countD; i++) {
      uint32_t sigLen = pulseIR[i];

      //the following logic takes care of the inverted signal in the IR receiver
      if (pulseIR[i] &  1) { //mark
        while (pulseIR[i + 1] & 1) { //account for repeated values  & combine
          i++;
          sigLen += pulseIR[i];
        }
        p += sprintf( p, "%lu,", sigLen);
      }
      else //space
      {
        while (!(pulseIR[i + 1] & 1)) { //account for repeated values & combine
          i++;
          sigLen += pulseIR[i];
        }
        p += sprintf( p, "-%lu,", sigLen);
      }

      yield(); //avoid watchdog issues
    }
    p += sprintf( p - 1, " "); //overwrite trailing comma, to clean up

#else //AIR format below
    p += sprintf( p, "format=air&channel=1&CarrierHz=%d", sum);
    p += sprintf( p, "&analysir=$%d:", sum); //first slot is ignored also.
    for (uint16_t i = 1; i < countD; i++) {
      uint32_t sigLen = pulseIR[i];

      //the following logic takes care of the inverted signal in the IR receiver
      if (pulseIR[i] &  1) { //mark
        while (pulseIR[i + 1] & 1) { //account for repeated values  & combine
          i++;
          sigLen += pulseIR[i];
        }
        p += sprintf( p, "%lu,", sigLen);
      }
      else //space
      {
        while (!(pulseIR[i + 1] & 1)) { //account for repeated values & combine
          i++;
          sigLen += pulseIR[i];
        }
        p += sprintf( p, "-%lu,", sigLen);
      }

      yield(); //avoid watchdog issues
    }
    p += sprintf( p, ";"); //';' is the terminator for AIR format

#endif
    //Serial.print("![HTTP] begin...!\n");
    //http.begin("http://192.168.0.233:9449/analysir.signal"); //HTTP
    http.begin(ipAddressAnalysIR, webServerPortAnalysIR, "/analysir.signal");

    // start connection and send HTTP header
    //http.addHeader("Content-Length",0)
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    //int httpCode = http.POST(data);
    int httpCode = http.POST(webBuffer);
    //http.writeToStream(&Serial);
    http.end();
    //Serial.println(webBuffer);
    //Serial.print("![HTTP] POST...!\n");

    // httpCode will be negative on error
    if (httpCode > 0) {
      // HTTP header has been send and Server response header has been handled
      //Serial.printf("![HTTP] POST... code: %d!\n", httpCode);

      // file found at server
      if (httpCode == HTTP_CODE_OK) {
        String payload = http.getString();
        Serial.println(payload);
      }
    } else {
      Serial.printf("![HTTP] POST... failed, error: %s!\n", http.errorToString(httpCode).c_str());
    }

    // http.end();
  }
  else { //send over Serial, if WiFi not active
    reportPulses();
    reportPeriod();//reports modulation frequency to host over serial
  }



}

void irWebSend() { //we gat a web request rfom AnalysIR to send an IR  signal


  //  if (webServer.hasArg("analysir")) {
  //    //Serial.println(webServer.arg("analysir"));
  //  }
  if (webServer.method() == HTTP_GET || !webServer.hasArg("analysir")) { //only accept POST requests
    //handleNotFound();
    webServer.send(200, "text/html", webBuffer); //issue response
    Serial.println("!INVALID - Web request rxed!");
  }
  else { //valid POST request
    //any partially rxed Ir signal is now discarded/ignored
    disableIRrx;  //disable interrupt handler for IR rx   on pin 2 - demodulated signal
    disableIRrxMOD;  //Same for Pin3 - modulated signal
    countD = 0; //reset value for next time
    sendWebIR();//send the IR signal ASAP!...so a longer delay in web response????

    //reset/enable rx again now that sending is finished
    countD = 0; //reset value for next time
    countM = 0; //reset

    enableIRrx;//set up interrupt handler for IR rx on pin 2 - demodulated signal
    enableIRrxMOD; //set up interrupt handler for modulated IR rx on pin 3 - full modulated signal
//now send response
#define VERBOSETX true
#if VERBOSETX //verbose returns a webpage which can be used to submit a FORM from a web  Browser
  sprintf(webBuffer, "<html><head><title>A.IR  Shield ESP8266 (Tx) - AnalysIR.com</title></head><body><h1>AnalysIR - IR Signal web submission Form</h1>"
          "<form method='post' action='http://%d.%d.%d.%d:%d/analysir.signal'><br>"
          "Signal:<br><textarea name='analysir' rows='10' cols='80'></textarea><br><br><input type='submit' value='Submit Signal'></form>"
          "<a href='https://www.analysir.com/blog/tag/esp8266/'>Visit AnalysIR.com</a></body></html>", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3], webServerPortAnalysIR);


#else //VERBOSETX false
  webBuffer[0] = 0; //set as null string
#endif //VERBOSETX

    webServer.send(200, "text/html", webBuffer); //issue response
    Serial.println("\n!Web request rxed & Signal sent!");
  }

}

void sendWebIR() { //retrieve timings from webBuffer, fill Tx buffer and send IR
  //expect formatted string from PC web request
  //$38:1000,1000,2000,1000,3000,1000,4000,1000,5000,;
  //$38:1000,-1000,2000,-1000,3000,-1000,4000,-1000,5000,;
  //$40:10000,10000,10000,10000,10000,10000,10000,10000,10000,;
  //$40:10000,10000,5000,5000,4000,4000,3000,3000,2000,2000,1000,1000,500,;  (useful generic test signals)
  //$38:9000,4500,560,560,560,560,560,1690,560,560,560,560,560,560,560,560,560,560,560,1690,560,1690,560,560,560,1690,560,1690,560,1690,560,1690,560,1690,560,560,560,560,560,560,560,1690,560,560,560,560,560,560,560,560,560,1690,560,1690,560,1690,560,560,560,1690,560,1690,560,1690,560,1690,560,39915,9000,2325,560,;
  //values less than 0x10000 have the LSBit set to 0, otherwise set to 1 - an uneven number has 0x10000 added to it
  //first time value ALWAYS must be a mark
  uint8_t fail = true; //assume we fail unless successful
  uint32_t sigLen = 0;
  uint16_t carrier = 0;

  String signalIR = webServer.arg("analysir"); //get a temp copy of the request to process below

  unsigned int sLen = signalIR.length();

  for (int i = 0; i < signalIR.length(); i++) {
    if (!signalIR[i]) break; //exit once we hit end of string (null char)
    //Serial.print(signalIR[i]);
  }

  //now extract signal to IR buffer for sending below
  //send signal
  countD = 0; //reset pointer
  //start at 1=1 to skip initial $
  for (int i = 1; i < sLen; i++) { //iterate thru request contents

    uint8_t decChar = signalIR[i]; //get next char from request
    if (decChar == ';') { //termination char
      fail = false; //message received correctly
      break; //exit while loop and move on to sending signal
    }
    else if (decChar == ',') { //signifies end of  DEC value
      if (sigLen & 0xFFFF0000) pulseIR[countD++] = sigLen | 0x1; //trick to store longer values in 16 bits vs 32 bits
      else pulseIR[countD++] = sigLen & 0xFFFE; //trick to store longer values in 16 bits vs 32 bits
      sigLen = 0; //start again
    }
    else if (decChar >= '0' && decChar <= '9' ) { //0-9 decimal conversion
      sigLen = (sigLen * 10) + (decChar - '0');
    }
    else if (decChar == ':') { //signifies end of DEC value for carrier
      carrier = sigLen;
      if (carrier>60) carrier = (carrier+1000/2)/1000; //default is 20->60 in kHz. Otherwise assume its in Hz
      //If the carrier is not valid(20-60), it will be sent as 38kHz..later
      sigLen = 0; //start again
      //Serial.println(carrier);
      ////////carrierInit(carrier, DUTYCYCLE);
      //initUPWM(carrier);
    }
    else if ((decChar == '-') || (decChar == ' ') || (decChar == '+') ) { //ignore minus signs and spaces, to allow for more liberal formatting...must start with Mark
      continue;
    }
    else {
      Serial.print(F("!Invalid Char: ")); Serial.write(decChar); Serial.println(F(" !"));
      countD = 0; //reset
      return; //invalid char so return without sending
    }

  }

  if (fail || countD == 0) return; //failed to get full message so return, without sending

  //now send message in the following format - must start with a mark (uSecs)
  //$38:1000,1000,2000,1000,3000,1000,4000,1000,5000,;
  //$40:10000,10000,10000,10000,10000,10000,10000,10000,10000,;
  //$40:10000,10000,5000,5000,4000,4000,3000,3000,2000,2000,1000,1000,500,;  (useful generic test signals)

  AIR_sendRAW(carrier); //send the signal now
//  Serial.println(carrier);
//  uint16_t i;
//  for (i = 0; i < countD; i++) {
//    Serial.print(pulseIR[i]); Serial.print(", ");
//  }
//  Serial.println(countD);
  Serial.println(F("!A.IR Shield - IR send OK!"));
}

#else //useWIFI false
// Serial IR sending code below here

void sendIR() { //retrieve timings over serial, fill buffer and send IR
  //expect formatted string from PC
  //$38:1000,1000,2000,1000,3000,1000,4000,1000,5000,;
  //$40:10000,10000,10000,10000,10000,10000,10000,10000,10000,;
  //$40:10000,10000,5000,5000,4000,4000,3000,3000,2000,2000,1000,1000,500,;  (useful generic test signals)
  //$38:9000,4500,560,560,560,560,560,1690,560,560,560,560,560,560,560,560,560,560,560,1690,560,1690,560,560,560,1690,560,1690,560,1690,560,1690,560,1690,560,560,560,560,560,560,560,1690,560,560,560,560,560,560,560,560,560,1690,560,1690,560,1690,560,560,560,1690,560,1690,560,1690,560,1690,560,39915,9000,2325,560,;
  //values less than 0x10000 have the LSBit set to 0, otherwise set to 1 - an uneven number has 0x10000 added to it
  //first time value must be a mark
  uint8_t fail = true; //assume we fail unless successful
  uint32_t timeOut = micros(); //must get all signal in 1 sec.
  uint32_t sigLen = 0;
  uint16_t carrier = 0;

  countD = 0; //reset pointer
  while ((micros() - timeOut) < 9000000) {
    if (Serial.available()) {
      uint8_t decChar = Serial.read();
      if (decChar == ';') { //termination char
        fail = false; //message received correctly
        break; //exit while loop and move on to sending signal
      }

      else if (decChar == ',') { //signifies end of  DEC value
        if (sigLen & 0xFFFF0000) pulseIR[countD++] = sigLen | 0x1; //trick to store longer values in 16 bits vs 32 bits
        else pulseIR[countD++] = sigLen & 0xFFFE; //trick to store longer values in 16 bits vs 32 bits
        sigLen = 0; //start again
      }

      else if (decChar >= '0' && decChar <= '9' ) { //0-9 decimal conversion
        sigLen = (sigLen * 10) + (decChar - '0');
      }

      else if (decChar == ':') { //signifies end of DEC value for carrier
        carrier = sigLen;
        sigLen = 0; //start again
        //Serial.println(carrier);
        ////////carrierInit(carrier, DUTYCYCLE);
        //initUPWM(carrier);
      }
      else if ((decChar == '-') || (decChar == ' ')  || (decChar == '+') ) { //ignore minus signs and spaces, to allow for more liberal formatting
        continue;
      }
      else {
        Serial.print(F("!Invalid Char: ")); Serial.write(decChar); Serial.println(F(" !"));
        countD = 0; //reset
        return; //invalid char so return without sending
      }
    }
  }

  if (fail || countD == 0) return; //failed to get full message so return, without sending

  //now send message in the following format - must start with a mark (uSecs)
  //$38:1000,1000,2000,1000,3000,1000,4000,1000,5000,;
  //$40:10000,10000,10000,10000,10000,10000,10000,10000,10000,;
  //$40:10000,10000,5000,5000,4000,4000,3000,3000,2000,2000,1000,1000,500,;  (useful generic test signals)

  AIR_sendRAW(carrier); //send the signal now
//  Serial.println(carrier);
//  uint16_t i;
//  for (i = 0; i < countD; i++) {
//    Serial.print(pulseIR[i]); Serial.print(", ");
//  }
//  Serial.println(countD);
  Serial.println(F("!A.IR Shield - IR send OK!"));
}

#endif //useWIFI
