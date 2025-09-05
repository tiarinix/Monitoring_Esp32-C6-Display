# Monitoreo Iot con ESP32-C6 y Sensores I2C (LVGL)
Se implementa un sistema de lectura de sensores usando I2C y se muestra la información en una pantalla integrada en el microcontrolador usando LVGL. El código detecta automáticamente qué sensor está conectado y despliega la pantalla correspondiente.

---
El Objetivo del código es:
- Escanear el bus I2C y detectar sensores disponibles.
- Mostrar en pantalla los valores de cada sensor (temperatura/humedad, luz, color, humedad de suelo, TDS, lluvia).
- Proveer un sistema modular y escalable para pruebas de sensado educativo.

---

## Configuración:
- Microcontrolador: ESP32-C6 con display integrado ST7789.
- I2C: SDA = GPIO9, SCL = GPIO18.

### Sensores incorporados: 
En caso de conectar un sensor diferente a los de la tabla el programa no leerá.

| Sensor           | Dirección I2C | Pantalla         |
|------------------|---------------|------------------|
| AHT20 (Temp/Hum) | `0x38`        | Temp/Hum         |
| TSL2591 (Lux)    | 0x29          | Luminosidad      |
| TCS34725 (Color) | 0x29          | Color            |
| ADS1115 @ Soil   | 0x48          | Humedad de suelo |
| ADS1115 @ TDS    | 0x49          | TDS (ppm)        |
| ADS1115 @ Rain   | 0x4A          | Lluvia           |
  
---

## Librerías usadas
- Adafruit_Sensor
- Adafruit_AHTX0
- Adafruit_TSL2591
- Adafruit_TCS34725
- Adafruit_ADS1X15
- LVGL + Driver ST7789
- Archivos `ui.h/ui.c` (UI generada por software externo)

---

## Flujo del programa (simplificado)
1. Inicializa Serial, I2C, pantalla y LVGL.
2. Escanea bus I2C y obtiene la última dirección detectada.
3. Selecciona la pantalla según el sensor detectado:
   - `0x38` → Temp/Hum
   - `0x29` → Lux o Color
   - `0x48` → Suelo
   - `0x49` → TDS
   - `0x4A` → Lluvia
   - Ninguno → Pantalla principal
4. Refresca cada 500 ms.

---

## Calibraciones a realizar
- Suelo
- Lluvia
- TDS

---

## Cosas a tener en cuenta
- [ ] Pull-ups I2C integrados
- [ ] Direcciones de ADS1115 configuradas correctamenter (A0/A1)  
- [ ] Nombre de la pantalla coincide con nombres en el código  
- [ ] Calibración de los sensores análogos  
