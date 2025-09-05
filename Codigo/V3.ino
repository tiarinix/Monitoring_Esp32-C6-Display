#include "Display_ST7789.h"
#include <Adafruit_ADS1X15.h>
#include <Adafruit_AHTX0.h>
#include "LVGL_Driver.h"
#include <Wire.h>
#include "ui.h"

#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"
#include "Adafruit_TCS34725.h"

#define SDA_PIN 9
#define SCL_PIN 18


#define ADDR_TEMP_HUM   0x38    // sensor temp/hum (0x38)
#define ADDR_LUX_COLOR  0x29    // sensor de luz y color (0x29)
#define ADDR_SOIL       0x48    // sensor analógico humedad suelo I2C (0x48)
#define ADDR_TDS        0x49    // sensor de TDS
#define ADDR_RAIN       0x4A    // sensor de lluvia


Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);
Adafruit_ADS1115 ADS_SOIL;
Adafruit_ADS1115 ADS_TDS;
Adafruit_ADS1115 ADS_RAIN;
Adafruit_AHTX0 aht;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

float tdsCompensationTemperature = 15;
float tdsCorrectionFactor = 1.668;
int maxTDSppmValue = 1000;
int minTDSppmValue = 0;
int thresholdppmValue = 5;

void setup()
{       
  Serial.begin(115200);
  // Espera a que el puerto abra
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0 < 3000)) { /* wait up to 3s */ }

  Serial.println();
  Serial.println("Booting...");

  // Inicia I2C (orden correcto: SDA, SCL)
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);   // 100 kHz (estable)
  Wire.setTimeOut(50);     // evita bloqueos si el bus se queda colgado

  LCD_Init();
  Lvgl_Init();
  ui_init();
}

uint8_t scanner(){      //----------ESCANER I2C------------
  

  uint8_t error;
  int devicesFound = 0;
  uint8_t addressFound = 0;

  Serial.println("Scanning...");

  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission(true); // enviar STOP

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(" !");
      devicesFound++;
      addressFound = address;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (devicesFound == 0){ 
    Serial.println("No I2C devices found");
  }
  return addressFound;
}

float measureTDS() {
  if(!ADS_TDS.begin(ADDR_TDS)) return 0;

  int16_t val_0 = ADS_TDS.readADC_SingleEnded(0);
  float ADS_volts = ADS_TDS.computeVolts(val_0);
  float compensationFactor = 1.0 + 0.02 * (tdsCompensationTemperature-25.0); 

  Serial.printf("Cuentas ADC = %d , voltaje medido por ADC = %f\n", val_0, ADS_volts);
  float cv = ADS_volts / compensationFactor;

  float tdsValue = 0.5 * (133.42*cv*cv*cv 
                        - 255.86*cv*cv 
                        + 857.39*cv);

  tdsValue = (tdsValue < thresholdppmValue) ? 0 : tdsValue;

  return constrain(tdsValue * tdsCorrectionFactor, minTDSppmValue, maxTDSppmValue);
}

float measureSoilHumidityPercent(){
  if(!ADS_SOIL.begin(ADDR_SOIL)) return 0.0;
  int dry = 17600; // value for dry sensor
  int wet = 5500; // value for wet sensor

  int16_t val_0 = ADS_SOIL.readADC_SingleEnded(0);
  // float voltageFactor = ADS_SOIL.computeVolts(val_0);
  Serial.printf("val = %d\n", val_0);

  int percentageHumididy = map(val_0, wet, dry, 100, 0);

  return constrain(percentageHumididy, 0, 100);
}

float measureRainPercent(){
  if(!ADS_RAIN.begin(ADDR_RAIN)) return -1.0;

  int dry = 14880; // value for dry sensor
  int wet = 17648; // value for wet sensor

  int16_t val_0 = ADS_RAIN.readADC_SingleEnded(0);
  Serial.printf("val = %d\n", val_0);
  // float voltageFactor = ADS_RAIN.computeVolts(val_0);

  int percentageHumididy = map(val_0, dry, wet, 100, 0);  

  return constrain(percentageHumididy, 0, 100);
}

void loop(){
  //-----------Lógica para selección de pantallas-------------
  uint8_t addressFound = scanner();

  Serial.print("address==   ");
  Serial.println(addressFound,HEX);

  switch(addressFound){
    case ADDR_TEMP_HUM:       //TEMPERATURA Y HUMEDAD
      Serial.println("Pantalla Temperatura y humedad");

      //-------Lógica del sensor-------
      if (! aht.begin()) {
        Serial.println("Could not find AHT? Check wiring");
        while (1) delay(10);
      } 

      sensors_event_t humidity, temp;
      aht.getEvent(&humidity, &temp);

      Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" degrees C");
      Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");

      //------Display------
      lv_scr_load(ui_TempHumScreen);
      lv_label_set_text(ui_TempVar, ((String(temp.temperature)+"°C").c_str()));
      lv_label_set_text(ui_HumidityVar, ((String(humidity.relative_humidity)+"%").c_str()));
      lv_arc_set_value(ui_HumidityArc, humidity.relative_humidity);

      break;

    case ADDR_LUX_COLOR:      //LUZ Y COLOR
      Serial.println("Pantalla Luz o Color");
      if (tsl.begin()) {  //LUZ
        Serial.println(F("Found a TSL2591 sensor"));
        Serial.println("Pantalla Luminosidad");

        //----------Lógica Sensor------------

        uint32_t lum = tsl.getFullLuminosity();
        uint16_t ir, full, lux;
        ir = lum >> 16;
        full = lum & 0xFFFF;
        lux = tsl.calculateLux(full,ir);
        Serial.print(F("Lux: ")); Serial.println(tsl.calculateLux(full, ir), 6);

        //---------DISPLAY----------
        lv_scr_load(ui_LuxScreen);
        lv_label_set_text(ui_LuxVar, (String(lux)).c_str());


      } else if (tcs.begin()){    //COLOR
        Serial.println(F("Found a TCS34725 sensor"));

        //-----------LÓGICA SENSOR----------
        uint16_t r, g, b, c, colorTemp, lux;

        tcs.getRawData(&r, &g, &b, &c);
        colorTemp = tcs.calculateColorTemperature(r, g, b);

        int red = map(r, 3299, 18703, 0, 65536);
        int green = map(g, 4519, 23942, 0, 65536);
        int blue = map(b, 3948, 21593, 0, 65536);
        
        red = ((float) r / (float) c ) * (float) 255;
        green = ((float)g / (float)c) * (float) 255;
        blue = ((float)b / (float)c) * (float) 255;

        long hexColor = ((long)red << 16L) | ((long)green << 8L) | (long)blue;

        Serial.print("R: "); Serial.print(red, DEC); Serial.print(" ");
        Serial.print("G: "); Serial.print(green, DEC); Serial.print(" ");
        Serial.print("B: "); Serial.print(blue, DEC); Serial.print(" ");

        Serial.print("Hex color: 0x");
        Serial.println(hexColor, HEX);
        

        //---------DISPLAY----------
        lv_scr_load(ui_ColorScreen);
        lv_obj_set_style_bg_color(ui_ColorImage, lv_color_hex(hexColor), LV_PART_MAIN | LV_STATE_DEFAULT);

      }

      else{
        Serial.println("ERROR");
        lv_scr_load(ui_MainScreen);
      }

      break;

    case ADDR_RAIN:           //LLUVIA
    {
      Serial.println("Pantalla Lluvia");

      //------Lógica Sensor--------
      float rainPercent = measureRainPercent();

      //-------DISPLAY--------
      lv_scr_load(ui_RainScreen);
      lv_label_set_text(ui_RainVar, (String(rainPercent)+"%").c_str());
      lv_arc_set_value(ui_RainArc, rainPercent);

      break;
    }

    case ADDR_TDS:            //TDS
    {
      Serial.println("Pantalla TDS");

      //------Lógica Sensor--------
      int tds_value = (int)measureTDS();
      lv_label_set_text(ui_TdsVar, (String(tds_value)+" ppm").c_str());
      //-------DISPLAY--------
      lv_scr_load(ui_TdsScreen);

      break;
    }

    case ADDR_SOIL:           //HUMEDAD DE SUELO
    {
      Serial.println("Pantalla Humedad de suelo");

      //------Lógica Sensor--------
      float soilHumPercent = measureSoilHumidityPercent();

      //-------DISPLAY--------
      lv_scr_load(ui_SoilScreen);
      lv_label_set_text(ui_SoilVar, (String(soilHumPercent)+"%").c_str());
      lv_arc_set_value(ui_SoilArc, soilHumPercent);
      
      break;
    }

    default:                  //MAIN SCREEN
      Serial.println("No corresponde a ningún sensor ingresado");
      Serial.println("Pantalla principal");
      lv_scr_load(ui_MainScreen);
    break;
  }

  Serial.println();
  Timer_Loop();
  delay(500);
}
