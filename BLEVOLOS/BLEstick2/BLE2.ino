#include <Arduino.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include "M5StickCPlus2.h"

#include "ssFont.h"
#include "mFont.h"
#include "hFont.h"
#include "logo.h"
M5Canvas img(&StickCP2.Display);
M5Canvas clc(&StickCP2.Display);

static BLEUUID btHomeUUID((uint16_t)0xFCD2);

BLEScan *pBLEScan;

std::string add[4]={"a4:c1:38:db:06:a9","a4:c1:38:82:5e:21","a4:c1:38:2e:ce:c3","a4:c1:38:3b:46:fa"};
String rooms[4]={"UPSTAIRS","LOUNGE","GARAGE","DINING"};

  float temp[4];
  float humi[4];
  float voltage[4];
 
  static bool isScanning = false;

  int posx[4]={5,90,5,90};
  int posy[4]={5,5,70,70};
  unsigned short blue=0x024E;
  unsigned short blue2=0x01EC; 
  unsigned short blue3=0x469F;
  unsigned short grays[13];
  int vol=0;
  int volE; 
  unsigned long tt=0;
  unsigned long tt2=0;
  int period=1000;
  int s=0;
  bool freeL[4]={1,1,1,1};

// The data locations are from thermometers using BTHomev2 format
// including packetId, temperature, humidity, battery voltage,
// (and seemingly a power status?)
// These come in multiple advertising packets. Examples:
// [40][00][8B][01][64][02][5B][08][03][58][16]
// [40][00][91][0C][6E][0B][10][00]
// See https://bthome.io/format/ for more information.
// If actively scanning the received data is longer,
// as it includes the device name.
static void notifyCallback(
  int s,
  uint8_t* pData,
  size_t length) {
  Serial.print("Notify callback for SENSOR ");
  Serial.print(rooms[s].c_str());
  if (pData[0]!=0x40)
  {
    Serial.println("Received wrong format");
    return;
  }
  freeL[s]=0;
  if ((length >= 6) && (pData[3]==0x0C)) {
    voltage[s] = (pData[4] | (pData[5] << 8)) * 0.001; //little endian
  }
  if ((length >= 7) && (pData[5]==0x02)) {
    temp[s] = (pData[6] | (pData[7] << 8)) * 0.01; //little endian
  }
  if ((length >= 11) && (pData[8]==0x03)) {
    humi[s] = (pData[9] | (pData[10] << 8)) * 0.01; //little endian
  }
  Serial.printf(" temp = %.1f C ; humidity = %.1f %% ; voltage = %.3f V\n", temp[s], humi[s], voltage[s]);
  freeL[s]=1;
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    uint8_t cServiceData[255]; // The entire payload is limited to 255 bytes
    if (advertisedDevice.haveServiceData() == true) {
      for (int sdi = 0; sdi < advertisedDevice.getServiceDataCount(); sdi++) {
        BLEUUID sdUUID = advertisedDevice.getServiceDataUUID(sdi);
        if (btHomeUUID.equals(sdUUID)) {
          for (int addr = 0; addr < 4; addr++) {
            if(advertisedDevice.getAddress().toString().c_str()==add[addr])
            {
              String strServiceData = advertisedDevice.getServiceData(sdi);
              memcpy(cServiceData, strServiceData.c_str(), strServiceData.length());
              notifyCallback(addr, cServiceData, strServiceData.length());
            }
          }
        }
      }
    }
  }
};

static void scanCompleteCallback(BLEScanResults scanResults) {
  isScanning = false;
}

void setup() {

 auto cfg = M5.config();
    StickCP2.begin(cfg);
    //StickCP2.Rtc.setDateTime( { { 2023, 12, 30 }, { 22, 43, 0 } } );
   
    StickCP2.Display.setBrightness(38);
    StickCP2.Display.setRotation(3);
    StickCP2.Display.fillScreen(RED);
    img.createSprite(170,135);
    clc.createSprite(60,125);
    clc.setSwapBytes(true);
  
  Serial.begin(115200);
  Serial.println("Starting BLE advertising monitor...");

  BLEDevice::init("");

     int co=220;
     for(int i=0;i<13;i++)
     {grays[i]=StickCP2.Display.color565(co, co, co);
     co=co-20;}

  pBLEScan = BLEDevice::getScan();  //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(), true, true);
  pBLEScan->setActiveScan(false);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);
  StickCP2.Display.fillScreen(BLACK);

} // End of setup.


const uint8_t notificationOff[] = {0x0, 0x0};
const uint8_t notificationOn[] = {0x1, 0x0};
bool onoff = true;

void drawCL()
{
clc.fillSprite(BLACK);
clc.setTextDatum(0);
clc.setTextColor(grays[0],BLACK);
auto dt = StickCP2.Rtc.getDateTime();
clc.loadFont(hFont);
clc.drawString(String(dt.time.hours)+":"+String(dt.time.minutes),0,0);
clc.unloadFont();
clc.pushImage(0,65,60,60,logo);
clc.fillSmoothRoundRect(36,26,24,20,2,grays[8]);
clc.drawRect(0,26,29,20,grays[1]);
clc.fillRect(29,30,3,12,grays[1]);

for(int i=0;i<volE;i++)
clc.fillRect(3+(i*5),29,3,14,GREEN);

clc.loadFont(ssFont);
clc.setTextColor(grays[0],grays[8]);
clc.setTextDatum(middle_center);
clc.drawString(String(dt.time.seconds),48,36);
clc.setTextDatum(0);
clc.setTextColor(RED,BLACK);
clc.drawString("VOLOS",0,48);

clc.unloadFont();
clc.pushSprite(175,5);
delay(5);
}

void draw()
{
  img.fillSprite(BLACK);
 
  for(int i=0;i<4;i++)
  {
    img.fillSmoothRoundRect(posx[i],posy[i],80,60,3,blue);
    
    img.fillSmoothCircle(posx[i]+10,posy[i]+25,2,ORANGE);
    img.setTextDatum(middle_center);
    img.setTextColor(WHITE,blue);
    img.loadFont(mFont);
    img.drawString(String(temp[i]),posx[i]+50,posy[i]+32);
    img.unloadFont();
    img.setTextDatum(0);
    
    img.loadFont(ssFont);
    img.setTextColor(blue3,blue);
    img.drawString(String((int)humi[i])+"%",posx[i]+4,posy[i]+45);
    img.setTextColor(grays[1],blue);
    img.drawString(rooms[i],posx[i]+4,posy[i]+2);
    img.unloadFont();
    
    img.fillSmoothRoundRect(posx[i]+42,posy[i]+46,38,14,2,BLACK);
    img.setTextColor(grays[1],BLACK);
    img.drawString(String(voltage[i])+"V",posx[i]+46,posy[i]+50);
    img.drawFastHLine(posx[i]+4,posy[i]+18,60,0x66DF);
  }
  img.pushSprite(0,0);
  delay(10);
}


void loop() {
    if (!isScanning) {
      isScanning = true;
      pBLEScan->start(60, scanCompleteCallback, true);
    }

   vTaskDelay(20);

    if(millis()>tt2+period){
    s++;
    if(s==60) s=0; 
    drawCL();
    tt2=millis();  
    }
    
    if(millis()>tt+period){
    vol = StickCP2.Power.getBatteryVoltage();
    volE=map(vol,3000,4180,0,5); 
    if(freeL[0]==1 && freeL[1]==1 && freeL[2]==1 && freeL[3]==1)
    draw();
    tt=millis();
    }
}