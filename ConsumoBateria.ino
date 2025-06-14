#include <Wire.h>
#include "HT_SSD1306Wire.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_sleep.h"
#include "esp_wifi.h"
#include "esp_bt.h"
#include "LoRaWan_APP.h"  // LoRaWAN stack completo
#include "LoRa.h"
#include "sx126x.h"

// --- DEFINICIONES ---
#define DEBUG_MODE false

#define LORAWAN_DEVADDR  (uint32_t)0x260111FD
const uint8_t LORAWAN_NWKSKEY[16] = { 0x88, 0xBC, 0xDE, 0xAD, 0xBE, 0xEF, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA };
const uint8_t LORAWAN_APPSKEY[16] = { 0x77, 0x4A, 0xBC, 0xDE, 0xFE, 0xED, 0xCA, 0xFE, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };

// --- DISPLAY ---
static SSD1306Wire display(0x3c, 500000, 17, 18, GEOMETRY_128_64, 21);
SemaphoreHandle_t display_mutex;
typedef void (*Demo)(void);

// --- I2C SENSOR ---
#define I2C_SCL_SENSOR 42
#define I2C_SDA_SENSOR 41
#define SENSOR_ADDR 0x7F
float Fullscale_P = 1000.0f;
SemaphoreHandle_t i2c_mutex;

float lastPressure = 0.0;
float lastTemp = 0.0;
/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t loraWanClass = CLASS_A;

void VextON() {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
  Serial.println("[Main] Vext habilitado");
}

void VextOFF() {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, HIGH);
  Serial.println("[Main] Vext deshabilitado");
}

void drawFontFaceDemo() {
  if (xSemaphoreTake(display_mutex, portMAX_DELAY)) {
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);
    String linea = "Presión: " + String(lastPressure, 2) + " kPa";
    display.drawString(0, 0, linea);
    linea = "Temp: " + String(lastTemp, 2) + " °C";
    display.drawString(0, 15, linea);
    display.display();
    xSemaphoreGive(display_mutex);
  }
}

void displayTask(void *pvParameters) {
  Serial.println("[DisplayTask] Inicializando DisplayTask");
  while (true) {
    drawFontFaceDemo();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void I2C_WriteReg(uint8_t reg, uint8_t cmd) {
  if (xSemaphoreTake(i2c_mutex, portMAX_DELAY)) {
    Wire1.beginTransmission(SENSOR_ADDR);
    Wire1.write(reg);
    Wire1.write(cmd);
    Wire1.endTransmission();
    xSemaphoreGive(i2c_mutex);
  }
}

void I2C_ReadNByte(uint8_t reg, uint8_t *buf, size_t len) {
  if (xSemaphoreTake(i2c_mutex, portMAX_DELAY)) {
    Wire1.beginTransmission(SENSOR_ADDR);
    Wire1.write(reg);
    Wire1.endTransmission(false);
    Wire1.requestFrom(SENSOR_ADDR, len);
    for (size_t i = 0; i < len && Wire1.available(); i++) {
      buf[i] = Wire1.read();
    }
    xSemaphoreGive(i2c_mutex);
  }
}


// Crear y configurar SleepParams_t
  SleepParams_t sleepParams;
  sleepParams.Fields.WakeUpRTC->0;   // No usar RTC wakeup
  sleepParams.Fields.Reset = 0;       // No reset al salir del sleep
  sleepParams.Fields.WarmStart = 0;   // Sleep completo (cold start)
  sleepParams.Fields.Reserved = 0;    // Siempre en 0 (por compatibilidad)
  sleepParams.Value = 0x00;

void send_lorawan() {
  uint8_t payload[4];
  int16_t p = (int16_t)(lastPressure * 10);
  int16_t t = (int16_t)(lastTemp * 10);
  payload[0] = (p >> 8) & 0xFF;
  payload[1] = p & 0xFF;
  payload[2] = (t >> 8) & 0xFF;
  payload[3] = t & 0xFF;

  memcpy(appData, payload, 4);
  appDataSize = 4;
  //LoRaWAN.send();
  //Serial.println("[LoRaWAN] Paquete enviado");
  SX126xSetSleep(sleepParams);
}

void sensorTask(void *pvParameters) {
  Serial.println("[SensorTask] Inicializando SensorTask");

  uint8_t Pressure[3], Temp[2];
  char data[3];
  float Cal_PData1, Cal_PData2, Pressure_data;
  float Cal_TData1, Cal_TData2, Temp_data;

  I2C_WriteReg(0x30, 0x0A);
  vTaskDelay(pdMS_TO_TICKS(100));

  I2C_ReadNByte(0x06, Pressure, 3);
  data[0] = Pressure[0]; data[1] = Pressure[1]; data[2] = Pressure[2];
  Cal_PData1 = data[0] * 65536 + data[1] * 256 + data[2];
  Cal_PData2 = (Cal_PData1 > 8388608) ? (Cal_PData1 - 16777216) / 8388608.0 : Cal_PData1 / 8388608.0;
  Pressure_data = Cal_PData2 * Fullscale_P;
  Serial.printf("[SensorTask] Presión: %.2f kPa\n", Pressure_data);

  I2C_ReadNByte(0x09, Temp, 2);
  data[0] = Temp[0]; data[1] = Temp[1];
  Cal_TData1 = data[0] * 256 + data[1];
  Cal_TData2 = (Cal_TData1 > 32768) ? (Cal_TData1 - 65536) / 256.0 : Cal_TData1 / 256.0;
  Temp_data = Cal_TData2;
  Serial.printf("[SensorTask] Temperatura: %.2f °C\n", Temp_data);

  lastPressure = Pressure_data;
  lastTemp = Temp_data;

  Serial.printf("[SensorTask] Enviando por LoRaWAN: %.2f kPa | %.2f °C\n", lastPressure, lastTemp);

  //send_lorawan();
  vTaskDelay(pdMS_TO_TICKS(3000));

  VextOFF();

  if (!DEBUG_MODE) {
    Serial.println("[LoRaWAN] Entrando en Sleep completo");
   // LoRaWAN.sleep(loraWanClass);
     LoRaWAN.sleep(loraWanClass);
  }
  
  if (!DEBUG_MODE) {
    Serial.println("[SensorTask] Entrando en Deep Sleep por 10s");
    esp_sleep_enable_timer_wakeup(10 * 1000000ULL);
    esp_deep_sleep_start();
  }
}


// Estas son las definiciones que tenés que agregar:

//DeviceClass_t loraWanClass = CLASS_A;
LoRaMacRegion_t loraWanRegion = LORAMAC_REGION_EU868;
bool overTheAirActivation = false;

uint32_t devAddr = 0x260111FD;

uint8_t nwkSKey[16] = { 0x88, 0xBC, 0xDE, 0xAD, 0xBE, 0xEF, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA };
uint8_t appSKey[16] = { 0x77, 0x4A, 0xBC, 0xDE, 0xFE, 0xED, 0xCA, 0xFE, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };

uint8_t devEui[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };  // No usados en ABP pero necesarios para el linker
uint8_t appEui[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };  // No usados en ABP pero necesarios para el linker
uint8_t appKey[16] = { 0x00 };  // No usados en ABP pero necesarios

bool isTxConfirmed = false;
uint8_t appPort = 2;
uint8_t confirmedNbTrials = 8;
bool loraWanAdr = true;
uint32_t appTxDutyCycle = 30000;
uint16_t userChannelsMask[6];
void setup() {
  Serial.begin(115200);
  while (!Serial);

  VextON();
  delay(100);

  display.init();
  display.setFont(ArialMT_Plain_10);
  Serial.println("[DisplayTask] Display inicializado");

  Wire1.begin(I2C_SDA_SENSOR, I2C_SCL_SENSOR);
  Serial.printf("[Main] I2C inicializado en SDA=%d SCL=%d\n", I2C_SDA_SENSOR, I2C_SCL_SENSOR);

  display_mutex = xSemaphoreCreateMutex();
  i2c_mutex = xSemaphoreCreateMutex();

  // Inicializamos LoRaWAN (modo ABP)
  loraWanClass = CLASS_A;
  loraWanRegion = LORAMAC_REGION_EU868;
  overTheAirActivation = false;
  devAddr = LORAWAN_DEVADDR;
  memcpy(nwkSKey, LORAWAN_NWKSKEY, 16);
  memcpy(appSKey, LORAWAN_APPSKEY, 16);
  appTxDutyCycle = 30000;
  isTxConfirmed = false;
  loraWanAdr = true;
  confirmedNbTrials = 8;
  appPort = 2;

  LoRaWAN.init(loraWanClass, loraWanRegion);
  LoRaWAN.join();

  if (display_mutex != NULL && i2c_mutex != NULL) {
    Serial.println("[Main] Creando tareas...");
    xTaskCreate(displayTask, "DisplayTask", 4096, NULL, 1, NULL);
    xTaskCreate(sensorTask, "SensorTask", 4096, NULL, 2, NULL);
  } else {
    Serial.println("[Main] Error al crear los mutex");
  }
}

void loop() {}