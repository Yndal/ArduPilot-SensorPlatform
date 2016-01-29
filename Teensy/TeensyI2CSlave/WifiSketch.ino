//#include <SoftwareSerial.h>

#define MAX_NETWORKS 64

#define esp Serial1

/*
+CWLAP:(3,"Lars's iPhone",-46,"3a:f6:43:a3:42:40",1)
+CWLAP:(3,"TelenorF2F164",-96,"a4:b1:e9:f2:f1:64",1)
+CWLAP:(3,"TelenorD60395",-86,"9c:97:26:d6:03:95",6)
+CWLAP:(4,"tgsm3hjz",-85,"20:e5:2a:f4:40:1c",11)
+CWLAP:(4,"Svendsen",-38,"00:18:e7:f9:6b:70",13)

OK
*/

String networks[MAX_NETWORKS];
int stringAmount = 0;
int signalStrengths[MAX_NETWORKS] = {0};

const String AT_RESET = "AT+RST";
const String AT_LIST_NETWORKS = "AT+CWLAP";
const String AT_SET_MODE = "AT+CWMODE=1";

static boolean hasStarted = false;  

void WiFiStart() {
  //delay(5000);
  clearNetworks();
  //Serial.begin(9600);
  Serial1.begin(9600);
  
  delay(1000);
  Serial1.println(AT_SET_MODE);
  delay(1000);
  
  WifiWaitFor("ready", 10000); //Give the device max 10 seconds to get ready
}

//void loop() {
  //getWifiNearby();
  //printStrings();

  //collectSignalStrengths(); //&values);
  //printValues(signalStrengths);
  
 // getAccumulatedValue();

//}

/*void printValues(int values[]){
  Serial.println("Values:");
  for(int i=0; i<stringAmount; i++)
    Serial.println(signalStrengths[i]);
  Serial.println();
}

void printStrings(){
  for(int i=0; i<stringAmount; i++){
    Serial.print(i); Serial.print(": "); Serial.println(networks[i]);
  }
}*/

void clearNetworks(){
  for(int i=0; i<MAX_NETWORKS; i++)
    networks[i] = "";
}

void getWifiNearby(){
  clearNetworks();
//  printStrings();
  esp.println(AT_LIST_NETWORKS);
  WifiWaitFor(AT_LIST_NETWORKS, 2000);
  
  stringAmount = 0;
  
  int counter = 0;
  unsigned long endTime = millis() + 10000L; //Wait up to 10 seconds for this to end
  while(millis() < endTime){
    if(esp.available()){
      char c = esp.read();
   //  Serial.print(c);
      if(c == '\n'){ //If new line
        continue; 
      } else if(c == '\r'){ //If return carret
        if(networks[stringAmount].startsWith("OK")){ //All data is collected
        //  stringAmount--;
          return; 
        } else if(counter != 0){
          stringAmount++;
          counter = 0;
        }
      } else {
        networks[stringAmount] += c;
        counter++;
      }
    }
  }
  //stringAmount = counter;
}

void collectSignalStrengths(){ 
  for(int i=0; i<stringAmount; i++){
    int first = networks[i].indexOf('"');
    int last = networks[i].lastIndexOf('"');
    String s = networks[i].substring(first+1, last);
    
    first = s.indexOf('"');
    last = s.lastIndexOf('"');
    s = s.substring(first+1, last);
    
    first = s.indexOf(',');
    last = s.lastIndexOf(',');
    s = s.substring(first+1, last);
    
    //Serial.print(s); Serial.print("=>"); Serial.println(s.toInt());
    signalStrengths[i] = s.toInt();
  }
}


boolean WifiWaitFor(String cmd, long timeout){
  unsigned int counter = 0;
  unsigned long endTime = millis() + timeout;
  while(millis() < endTime){
    if(esp.available()){
      char c = esp.read();
     // Serial.print(c);
      if(cmd[counter] == c){
        if(counter == cmd.length()-1){
        //  Serial.print("Found "); Serial.println(cmd);
          return true;
        }
        else
          counter++;
      }
    }
  }
  return false;
}

/*****************
* Main method - this will get measurements, extract values and return the sum of these
*****************/
int WiFiGetAccumulatedValue(){
  if(hasStarted){
    WiFiStart();
    hasStarted = true;
  }
  getWifiNearby();
  collectSignalStrengths();
  
  int value = -1;
  for(int i=0; i<stringAmount; i++)
    value += signalStrengths[i];
    
 // Serial.print("StringAmount: "); Serial.println(stringAmount);
  //Serial.print("Value: "); Serial.println(value);
  
  return value;
}

