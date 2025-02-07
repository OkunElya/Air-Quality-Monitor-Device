#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSans12pt7b.h>

#include <Adafruit_BMP085.h>
#include <Adafruit_SCD30.h>
#include <PMserial.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

// DEVICES PART
//  I2C device at address 0x3C display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET 4
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT);
// I2C device at address 0x61 senserion SDC30
Adafruit_SCD30 scd30;
// I2C device at address 0x77 bmp180 +
Adafruit_BMP085 bmp;
float bmpTemp = 0;
int32_t bmpPressure = 0;
// UART  2 (PMSA003) set on 32 REset on 33
#define PMS_RES 33
#define PMS_SET 32
SerialPM pms(PMSA003, Serial2);

struct lastUpdated
{
  long unsigned int bmp = 0;
  long unsigned int pms = 0;
  long unsigned int scd = 0;
  long unsigned int ble = 0;
};
struct lastUpdated lu;

void updatePMS();
void updateSensors();
void updateBLEValues();
// buttons
#define PRESS_BUFF_LEN 10
volatile char presses[PRESS_BUFF_LEN];
#define B_UP 25
#define B_CE 26
#define B_DW 27
#define readBUP bitRead(GPIO.in, B_UP) // faster than digitalRead()
#define readBCE bitRead(GPIO.in, B_CE) // faster than digitalRead()
#define readBDW bitRead(GPIO.in, B_DW) // faster than digitalRead()
volatile unsigned long int UpPressed = 0;
volatile unsigned long int UpReleased = 0;
volatile unsigned long int CenterPressed = 0;
volatile unsigned long int CenterReleased = 0;
volatile unsigned long int DownPressed = 0;
volatile unsigned long int DownReleased = 0;

void buttonUpIsr()
{
  volatile unsigned long int t = millis();
  if (readBUP)
  {
    if ((t - UpPressed) > 20)
      UpReleased = t;
  }
  else
  {
    if ((t - UpReleased) > 20)
      UpPressed = t;
    // buton is pressed
  }
}
long unsigned int bup() // function to use in button checks, returns press length
{
  if ((UpPressed == 0) || ((millis() - UpPressed) < 20))
    return 0;
  if ((UpReleased != 0 && (UpPressed < UpReleased)) || readBUP)
  {
    long unsigned int retVal = UpReleased - UpPressed;
    UpPressed = 0;
    UpReleased = 0;
    return retVal;
  }

  return 0;
}

void buttonCenterIsr()
{
  volatile unsigned long int t = millis();
  if (readBCE)
  {
    if ((t - CenterPressed) > 20)
      CenterReleased = t;
  }
  else
  {
    if ((t - CenterReleased) > 20)
      CenterPressed = t;
    // buton is pressed
  }
}
long unsigned int bce()
{
  if ((CenterPressed == 0) || ((millis() - CenterPressed) < 20))
    return 0;
  if ((CenterReleased != 0 && (CenterPressed < CenterReleased)) || readBCE)
  {
    long unsigned int retVal = CenterReleased - CenterPressed;
    CenterPressed = 0;
    CenterReleased = 0;
    return retVal;
  }
  return 0;
}

void buttonDownIsr()
{
  volatile unsigned long int t = millis();
  if (readBDW)
  {
    if ((t - DownPressed) > 20)
      DownReleased = t;
  }
  else
  {
    if ((t - DownReleased) > 20)
      DownPressed = t;
    // buton is pressed
  }
}
long unsigned int bdw()
{
  if ((DownPressed == 0) || ((millis() - DownPressed) < 20))
    return 0;
  if ((DownReleased != 0 && (DownPressed < DownReleased)) || readBDW)
  {
    long unsigned int retVal = DownReleased - DownPressed;
    DownPressed = 0;
    DownReleased = 0;
    return retVal;
  }
  return 0;
}

// BLUETOOTH PART
#define SERVICE_UUID "181A" // enviromental sensing device
// BMP
BLECharacteristic bmpTemperatureCelsiusCharr("2A1F", BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic bmpTemperatureCharr("2A6E", BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic bmpPresureCharr("2A6D", BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
// SCD30
BLECharacteristic scdTemperatureCelsiusCharr("2A1F", BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic scdTemperatureCharr("2A6E", BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic scdCo2ConcentrationCharr("2B8C", BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic scdHumidityCharr("2A6F", BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
// PMSA003
BLECharacteristic pmsPM1_0Charr("2BD5", BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic pmsPM2_5Charr("2BD6", BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic pmsPM10_0Charr("2BD7", BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
// normal concentration
BLECharacteristic pmsN0_3Charr("b884812d-094f-4c0d-8ff0-f530f5ed7208", BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic pmsN0_5Charr("517b441e-2d87-41cb-ae81-625ef34c2e49", BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic pmsN1_0Charr("3402d95f-317c-4915-9efd-f73d13ef9b2c", BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic pmsN2_5Charr("553ff836-97d2-4b6c-93ff-4abf7b00112f", BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic pmsN5_0Charr("bc97c96c-072f-4b09-aa01-0deae4eb9a26", BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic pmsN10_0Charr("058d6250-21a7-469d-ab7f-f3fe260388a2", BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

BLEServer *pServer;
BLEService *pService;
BLEAdvertising *pAdvertising;

bool deviceConnected = false;

class MyServerCallbacks : public BLEServerCallbacks // copied somewhere
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
    Serial.println("***** Connect");
  }
  void onDisconnect(BLEServer *pServer)
  {
    Serial.println("***** Disconnect");
    deviceConnected = false;
    BLEDevice::startAdvertising();
  }
};
MyServerCallbacks bleCallbacks;

// DISPALY PART
#define TIME_BEFORE_DIMMING 15000
long unsigned int dispLastUsed = 1;
bool keepDisplayOn=false;
int currentMenu = 0;
void simpleTextDisp(const char text[])
{
  display.clearDisplay();
  display.setTextColor(1); // white
  display.setCursor(0, 0);
  display.write(text);
  display.display();
}
void updateDisp();
void mainScreen();
void scdBmpScreen();
void PMScreen();

void setup()
{
  // set pin states
  pinMode(PMS_RES, OUTPUT);
  pinMode(PMS_SET, OUTPUT);
  pinMode(B_UP, INPUT_PULLUP);
  pinMode(B_CE, INPUT_PULLUP);
  pinMode(B_DW, INPUT_PULLUP);
  digitalWrite(PMS_RES, 1);
  digitalWrite(PMS_SET, 1);
  // start buttons
  attachInterrupt(B_UP, buttonUpIsr, CHANGE);
  attachInterrupt(B_CE, buttonCenterIsr, CHANGE);
  attachInterrupt(B_DW, buttonDownIsr, CHANGE);

  // init interface
  Wire.begin(22, 19);
  Serial.begin(115200);
  Serial2.begin(9600);

  // init peripherals
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  { //
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }
  simpleTextDisp("BME init..");

  if (!bmp.begin(BMP085_ULTRAHIGHRES)) // latency is not an issue
  {
    Serial.println("Could not find a valid BMP sensor, check wiring!");
    for (;;)
      ;
  }

  simpleTextDisp("SCD init..");
  if (!scd30.begin(0x61))
  {
    Serial.println("Failed to find SCD30 chip");
    for (;;)
      ;
  }

  simpleTextDisp("PMS init..");
  pms.init();
  // delay(1000);
  // pms.read();//fails,don't want to make setup slower
  // if (!pms.OK)
  // {
  //   Serial.println("Failed to initialize PMSX003");
  //   for (;;)
  //     ;
  // }

  simpleTextDisp("periph.init done.");

  display.clearDisplay();
  display.ssd1306_command(SSD1306_SETCONTRAST); // 0x81
  display.ssd1306_command(2);//dimming disaply to prevent burnout

  scd30.setMeasurementInterval(15); //
  scd30.read();

  simpleTextDisp("BLE init...");
  BLEDevice::init("Air Quality Monitor");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(&bleCallbacks);
  // bmp
  BLEUUID ServiceUuid = BLEUUID(SERVICE_UUID);
  pService = pServer->createService(ServiceUuid, 50, 0);
  pService->addCharacteristic(&bmpPresureCharr);
  // pService->addCharacteristic(&bmpTemperatureCelsiusCharr);
  // pService->addCharacteristic(&bmpTemperatureCharr);

  // scd
  pService->addCharacteristic(&scdCo2ConcentrationCharr);
  // pService->addCharacteristic(&scdTemperatureCelsiusCharr);
  pService->addCharacteristic(&scdTemperatureCharr);
  pService->addCharacteristic(&bmpTemperatureCelsiusCharr); //!

  pService->addCharacteristic(&scdHumidityCharr);

  // pms
  pService->addCharacteristic(&pmsPM1_0Charr);
  pService->addCharacteristic(&pmsPM2_5Charr);
  pService->addCharacteristic(&pmsPM10_0Charr);

  pService->addCharacteristic(&pmsN0_3Charr);
  pService->addCharacteristic(&pmsN0_5Charr);
  pService->addCharacteristic(&pmsN1_0Charr);
  pService->addCharacteristic(&pmsN2_5Charr);
  pService->addCharacteristic(&pmsN5_0Charr);
  pService->addCharacteristic(&pmsN10_0Charr);

  pService->start();
  pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->setAdvertisementType(ADV_TYPE_IND);
  pAdvertising->setScanResponse(true); // was true
  pAdvertising->setMinPreferred(0x06); // functions that help with iPhone connections issue
  pAdvertising->setMaxPreferred(0x12);
  pAdvertising->setAppearance(0x0542);

  BLEDevice::startAdvertising();
  Serial.println("STARTED!");

  display.clearDisplay();
  display.display();
}

void loop()
{
  updateSensors();
  updateBLEValues();
  updateDisp();
}

void updatePMS()
{
  pms.read();
  switch (pms.status)
  {
  case pms.OK:
    Serial.printf("PM1.0 %2d, PM2.5 %2d, PM10 %2d [ug/m3]\n",
                  pms.pm01, pms.pm25, pms.pm10);
    Serial.printf("N0.3 %4d, N0.5 %3d, N1.0 %2d, N2.5 %2d, N5.0 %2d, N10 %2d [#/100cc]\n",
                  pms.n0p3, pms.n0p5, pms.n1p0, pms.n2p5, pms.n5p0, pms.n10p0);
    break;
  case pms.ERROR_TIMEOUT:
    Serial.println(F(PMS_ERROR_TIMEOUT));
    break;
  case pms.ERROR_MSG_UNKNOWN:
    Serial.println(F(PMS_ERROR_MSG_UNKNOWN));
    break;
  case pms.ERROR_MSG_HEADER:
    Serial.println(F(PMS_ERROR_MSG_HEADER));
    break;
  case pms.ERROR_MSG_BODY:
    Serial.println(F(PMS_ERROR_MSG_BODY));
    break;
  case pms.ERROR_MSG_START:
    Serial.println(F(PMS_ERROR_MSG_START));
    break;
  case pms.ERROR_MSG_LENGTH:
    Serial.println(F(PMS_ERROR_MSG_LENGTH));
    break;
  case pms.ERROR_MSG_CKSUM:
    Serial.println(F(PMS_ERROR_MSG_CKSUM));
    break;
  case pms.ERROR_PMS_TYPE:
    Serial.println(F(PMS_ERROR_PMS_TYPE));
    break;
  default:
    Serial.println(F("Unknown error"));
    break;
  }
}
void updateSensors()
{
  unsigned long int ms = millis();
  if (ms - lu.bmp > 2000)
  {
    lu.bmp = ms;
    bmpTemp = bmp.readTemperature();
    bmpPressure = bmp.readPressure();
  }
  if (ms - lu.pms > 10000)
  {
    updatePMS();
    lu.pms = ms;
  }
  if (ms - lu.scd > 1000)
  {
    lu.scd = ms;
    if (scd30.dataReady())
    {
      if (!scd30.read())
      {
        Serial.println("Error reading sensor data");
      }
    }
  }
}
void updateBLEValues()
{
  if ((millis() - lu.ble) > 10000)
  {
    lu.ble = millis();
    // bmp
    int bmpTempCelsBuf = bmpTemp * 10;
    bmpTemperatureCelsiusCharr.setValue(bmpTempCelsBuf);
    bmpTempCelsBuf *= 10;
    bmpTemperatureCharr.setValue(bmpTempCelsBuf);

    int bmpPressurePasBuf = bmpPressure * 10;
    bmpPresureCharr.setValue(bmpPressurePasBuf);

    // scd
    int scdTempCelsBuf = scd30.temperature * 10;
    scdTemperatureCelsiusCharr.setValue(scdTempCelsBuf);
    scdTempCelsBuf *= 10;
    scdTemperatureCharr.setValue(scdTempCelsBuf);

    int scdHumidityBuf = scd30.relative_humidity * 100;
    scdHumidityCharr.setValue(scdHumidityBuf);

    uint16_t scdCo2ConcentrationBuf = scd30.CO2;
    scdCo2ConcentrationCharr.setValue(scdCo2ConcentrationBuf);

    // pms
    float pmsPM1_0 = pms.pm01;
    pmsPM1_0Charr.setValue(pmsPM1_0);

    float pmsPM2_5 = pms.pm25;
    pmsPM2_5Charr.setValue(pmsPM2_5);

    float pmsPM10_0 = pms.pm10;
    pmsPM10_0Charr.setValue(pmsPM10_0);

    pmsN0_3Charr.setValue(pms.n0p3);
    pmsN0_5Charr.setValue(pms.n0p5);
    pmsN1_0Charr.setValue(pms.n1p0);
    pmsN2_5Charr.setValue(pms.n2p5);
    pmsN5_0Charr.setValue(pms.n5p0);
    pmsN10_0Charr.setValue(pms.n10p0);
  }
}

void updateDisp()
{
  display.setFont(); // reset to default font
  if (deviceConnected)
  {
    display.setCursor(120, 0);
    display.print("O");
  }
  else
  {
    display.setCursor(120, 0);
    display.print("X");
  }
  if((!keepDisplayOn)&&((millis()-dispLastUsed)>TIME_BEFORE_DIMMING))//if display is disabled
  {
    if(bup()||bce()||bdw()){
      dispLastUsed=millis();
    }
    else{
      display.clearDisplay();
      display.display();
      return;
    }

  }


  switch (currentMenu)
  {
  case 0:
    mainScreen();
    break;
  case 1:
    scdBmpScreen();
    break;
  case 2:
    PMScreen();
    break;
  default:
    break;
  }

 
  if (bup())
  {
    currentMenu--;
    if (currentMenu < 0)
    {
      currentMenu = 2;
    }
  }
  if (bdw())
  {
    currentMenu++;
    if (currentMenu > 2)
    {
      currentMenu = 0;
    }
  }

  long unsigned int pressLen=bce();//long press (>1000ms) to enable constant display
  if (pressLen)
  {
    dispLastUsed=millis();
    if(pressLen<1000){
      currentMenu = 0;
    }
    else{
      keepDisplayOn=!keepDisplayOn;
      simpleTextDisp(keepDisplayOn? "dimming:false":"dimming:true");
      delay(500);
    }
  }
}

void scdBmpScreen()
{
  display.setFont(); // reset to default font
  display.setCursor(0, 0);
  display.print("BMP:\nPress:");
  display.print(bmpPressure);
  display.print("\n");

  display.print("Temp:");
  display.print(bmpTemp);
  display.print("\n");

  display.print("SCD:\nTemp:");
  display.println(scd30.temperature);

  display.print("Humid: ");
  display.print(scd30.relative_humidity);
  display.println(" %");

  display.print("CO2: ");
  display.print(scd30.CO2, 3);
  display.println(" ppm");
  display.println("");

  display.display();
  display.clearDisplay();
}
void PMScreen()
{
  display.setFont(); // reset to default font
  display.setCursor(0, 0);
  display.print(" Particles:[ug/m3]");
  display.print("\n");
  display.print("PM 1.0: ");
  display.print(pms.pm01);
  display.print("\n");

  display.print("PM 2.5: ");
  display.print(pms.pm25);
  display.print("\n");

  display.print("PM 10.0: ");
  display.print(pms.pm10);

  // conc

  display.print("\n Number conc [cc] \n");
  display.print("N0.3:");
  display.print(pms.n0p3);
  display.print(" 0.5:");
  display.print(pms.n0p5);

  display.print("\n 1.0: ");
  display.print(pms.n1p0);
  display.print(" 2.5:");
  display.print(pms.n2p5);

  display.print("\n 5.0: ");
  display.print(pms.n5p0);
  display.print(" 10:");
  display.print(pms.n10p0);

  display.display();
  display.clearDisplay();
}
void mainScreen()
{
  display.setFont();
  display.setCursor(0, 4);
  display.print("Co2\nppm");

  display.setFont(&FreeSans12pt7b);
  display.setCursor(19, 18);
  display.print(int(scd30.CO2));
  //
  display.setFont();
  display.setCursor(0, 4 + 20);
  display.print("PM2.5\nug/m3");

  display.setFont(&FreeSans12pt7b);
  display.setCursor(31, 18 + 20);
  display.print(int(pms.pm25));

  //
  display.setFont();
  display.setCursor(86, 48);
  display.print((float)((scd30.temperature + bmpTemp - 1.5) / 2.), 1); // hmmm looks like sensor heats up by 1.5 c (temp offset found in the esphome) to lazy to figure out nicer formula
  display.print("C");

  display.setCursor(86, 56);
  display.print(scd30.relative_humidity);
  display.print("%");

  display.setCursor(0, 48);
  display.print(bmpPressure / 133.322387415);
  display.setCursor(0, 56);

  display.print("mmHg");

  display.display();
  display.clearDisplay();
}