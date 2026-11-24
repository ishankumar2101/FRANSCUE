// FRANSCUE beacon bench-test draft. ESP32-C3; Arduino-ESP32 BLE library.
// Not compiled or tested on hardware in this workspace.
#include <BLEDevice.h>
#include <BLEAdvertising.h>
constexpr int BUTTON=4, LED=5;
constexpr unsigned long HOLD_MS=2000, BLINK_MS=300;
constexpr char NAME[]="FRANSCUE-07";
bool active=false, timing=false;
unsigned long pressed=0, started=0;
void setup(){
  pinMode(BUTTON,INPUT_PULLUP); pinMode(LED,OUTPUT); digitalWrite(LED,LOW);
  Serial.begin(115200); BLEDevice::init(NAME);
  Serial.println("Ready: hold button for 2 seconds.");
}
void loop(){
  unsigned long now=millis();
  if(active){digitalWrite(LED,((now-started)/BLINK_MS)%2==0?HIGH:LOW);}
  else if(digitalRead(BUTTON)==LOW){
    if(!timing){timing=true;pressed=now;}
    if(now-pressed>=HOLD_MS){
      BLEAdvertisementData data;data.setFlags(0x06);data.setName(NAME);
      auto *adv=BLEDevice::getAdvertising();
      adv->setAdvertisementType(ADV_TYPE_NONCONN_IND);adv->setAdvertisementData(data);adv->start();
      active=true;started=now;Serial.println("SOS active: FRANSCUE-07");
    }
  }else timing=false;
  delay(10);
}
