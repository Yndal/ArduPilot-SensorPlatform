#include <i2c_t3.h>

// Command definitions
#define MEASURE  0x10
#define COLLECT  0x11
#define TYPE     0x20
#define SETRATE  0x30
#define TEST     0x40

// Function prototypes
void receiveEvent(size_t len);
void requestEvent(void);

// vars
#define OUT_LEN 32
uint8_t out[OUT_LEN];
uint8_t cmd, hi, lo;
i2c_rate rate;
uint8_t _type;

enum TEENSY_SENSOR_TYPE {
  TEENSY_SENSOR_TYPE_TEST = 0,
  TEENSY_SENSOR_TYPE_TEMPERATURE = 1,
  TEENSY_SENSOR_TYPE_LIGHT = 2,
  TEENSY_SENSOR_TYPE_RSSI = 3
};

boolean isMeasuring = false;

//
// Setup
//
void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN,OUTPUT); // LED

  // Setup for Slave mode, address 0x44, pins 18/19, external pullups, 100kHz
  Wire.begin(I2C_SLAVE, 0x44, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_100);

  // init vars
  cmd = 0;
  rate = I2C_RATE_100;
  
  // SET TYPE!!
  _type = TEENSY_SENSOR_TYPE_TEST;

  // register events
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

void loop() {
  digitalWrite(LED_BUILTIN,HIGH); // double pulse LED while waiting for I2C requests
  delay(10);                      // if the LED stops the slave is probably stuck in an ISR
  digitalWrite(LED_BUILTIN,LOW);
  delay(50);
  digitalWrite(LED_BUILTIN,HIGH);
  delay(10);
  digitalWrite(LED_BUILTIN,LOW);
  delay(930);
  /*Serial.println("Getting value");
  int value = getAccumulatedBecaonValue();
  Serial.print("Got value: ");
  Serial.println(value);*/
}

//
// handle Rx Event (incoming I2C request/data)
//
void receiveEvent(size_t len) {
  if(Wire.available()) {
    // grab command
    cmd = Wire.readByte();
    switch(cmd) {
    case MEASURE:
      if(isMeasuring)
        return;
      isMeasuring = true;
      Serial.print("Measuring...");
      measure();
      Serial.println("DONE");
      isMeasuring = false;
      break;
    case SETRATE:
      rate = (i2c_rate)Wire.readByte();  // grab rate
      Wire.setRate(F_BUS, rate);         // set rate
      break;
    }
  }
}

//
// handle Tx Event (outgoing I2C data)
//
void requestEvent(void) {
  //Serial.println("Requesting something...");
  switch(cmd) {
  case COLLECT:
    //Serial.println("Collection requested...");
    Wire.write(out, 3);
    out[2] = 0;
    break;
  case TEST:
    Wire.write(42);
    break;
  case TYPE:
    Wire.write(_type);
    break;
  }
}

//
// Perform sensor readings and store for i2c retrieval
void measure() {
  out[2] = 1;
  
  int temp = 0;
  //temp = (int) (getDS18Temp()*100);
  temp = getAccumulatedBecaonValue(); //iBeaconScanner must be updated according to the iBeaconTest program!         getAccumulatedBecaonValue();//ReadTemt6000();//WiFiGetAccumulatedValue();//2701;
  
 
  out[0] = temp >> 8;
  out[1] = (uint8_t)temp;

  
 /*test = getAccumulatedBecaonValue(); //iBeaconScanner must be updated according to the iBeaconTest program!         getAccumulatedBecaonValue();//ReadTemt6000();//WiFiGetAccumulatedValue();//2701;
  
  // int acc_rssi = someMethod();
  out[0] = test >> 8;
  out[1] = (uint8_t)test;*/
  //test++;
}

