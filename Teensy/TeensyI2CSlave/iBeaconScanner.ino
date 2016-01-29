//#define MAX_BEACONS 32


boolean iBeaconScanner_started = false;

void iBeaconScanner_start(){
  Serial1.begin(9600);
  delay(100);
  
  //Configuration sequence
  Serial1.print("AT+ROLE1"); //Set as Master
  iBeaconScanner_waitFor("OK+Set:1", 500);
  Serial1.print("AT+IMME1"); //Set Module work type as 1
  iBeaconScanner_waitFor("OK+Set:1", 500);
//  Serial.print("iBeaconScanner started");
}



int getAccumulatedBecaonValue(){
  if(!iBeaconScanner_started){
    iBeaconScanner_start();
    iBeaconScanner_started = true; 
  }
  
  //Empty buffer
  while(Serial1.available())
    Serial1.read();
  
  Serial1.print("AT+DISI?");
  String s = "";
  while(!s.endsWith("OK+DISCE")){
    s += Serial1.readString();
  }
 // Serial.println("AAAAAAAAA:");
 // Serial.println(s);
  
  //Collecting RSSIs
  //Example with 2 iBeacons
  //OK+DISISOK+DISC:4C000215:80CC52E5A25E40A784F4425A474323DB:00000000C5:7469AA8356B1:-051OK+DISC:4C000215:5A4BCFCE174E4BACA814092E77F6B7E5:00000000C5:6B648DAEE2DC:-057OK+DISCE

  //How many are there
  String sTemp = s.substring(0);
  String DISC_STRING = "OK+DISC";
  int i = sTemp.indexOf(DISC_STRING);
  int N = -1;
  while(-1 < i){
  //  Serial.print(sTemp); Serial.print(": "); Serial.println(i);
    N++;
    sTemp = sTemp.substring(i + DISC_STRING.length());
    i = sTemp.indexOf(DISC_STRING);
   
    if(100 < N) //Something went wrong....
      return 0;
    
  }
  
  Serial.print("Found "); Serial.print(N); Serial.println(" iBeacons");
  
  if(N == 0)
    return 0;
  Serial.println("N is not 0");
  
  //int* ints = createArray(N);
  int rssiSum = 0;
  const char colon = ':';
  int i1 = 0;
  for(int n=0; n<N; n++){
    i1 = s.indexOf(colon);
    s = s.substring(i1+1); //Skip OK+DISC:
    i1 = s.indexOf(colon);
    String s1 = s.substring(0,i1); //Factory ID
    s = s.substring(i1+1); //Skip colon
    
    i1 = s.indexOf(colon);
    String s2 = s.substring(0,i1); //UUID
    s = s.substring(i1+1); //Skip colon
    
    i1 = s.indexOf(colon);
    String s3 = s.substring(0,i1); //Major + Minor + Measured Power
    s = s.substring(i1+1); //Skip colon
    
    i1 = s.indexOf(colon);
    String s4 = s.substring(0,i1); //MAC
    s = s.substring(i1+1); //Skip colon
    
    i1 = s.indexOf("OK+");
    String s5 = s.substring(0,i1); //RSSI
    s = s.substring(i1+1); //Skip colon
    
    //For debugging
  /*  Serial.print("#"); Serial.println(n);
    Serial.print("Factory ID: "); Serial.println(s1);
    Serial.print("UUID: "); Serial.println(s2);
    Serial.print("Majer, Minor, Power: "); Serial.println(s3);
    Serial.print("MAC: "); Serial.println(s4);
    Serial.print("RSSI: "); Serial.println(s5);
 */   
    int rssi = s5.toInt();
   // Serial.print("RSSI is interpretted to "); Serial.println(rssi);
    if(s2.toInt() != 0)
      rssiSum += rssi + 100; //Add 100 in order to get a usable sum
/*    ints[n] = 0;
    if(s2.toInt() != 0)
      ints[n] = rssi;*/
  }
  
 // printInts(ints, N);

  
//  Serial.print("CCCCCCC: "); Serial.println(sizeof(ints));
  
/*  int result = 0;
  //Sum up values and return this
  for(int i=0;  i<N; i++)
    if(ints[i] != 1){
      Serial.print(i); Serial.print(": "); Serial.println(ints[i]);
      result += ints[i];
    }*/
    
  Serial.print("rssiSum: ");  Serial.println(rssiSum);
  return rssiSum;
}



void printInts(int* ints, int N){
  Serial.println("PRINTING INTS*******************");
  for(int i=0;  i<N; i++){
      Serial.print(i); Serial.print(": "); Serial.println(ints[i]);
  } 
  Serial.println("DONE PRINTING INTS**************");
}



//Copied from post #2 at http://forum.arduino.cc/index.php?topic=106013.0 28th of May 2015
int * createArray(uint8_t size)
{
 // if (size > 0) 
  //{
    int* p = (int*) malloc(size * sizeof(int));
    for (int i = 0; i< size; i++) *p++ = 1;
    return p;
  //}
 // return null;
}





boolean iBeaconScanner_waitFor(String cmd, long timeout){
  unsigned int counter = 0;
  unsigned long endTime = millis() + timeout;
  while(millis() < endTime){
    if(Serial1.available()){
      char c = Serial1.read();
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
