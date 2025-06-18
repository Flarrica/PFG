#include <Wire.h>
#include <heltec_unofficial.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// --- SENSOR ---
#define I2C_SCL_SENSOR 42
#define I2C_SDA_SENSOR 41
#define SENSOR_ADDR 0x7F
float Fullscale_P = 1000.0f;

SemaphoreHandle_t i2c_mutex;

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

void Sensor() {
  uint8_t Pressure[3], Temp[2];
  char data[3];
  float Cal_PData1, Cal_PData2, Pressure_data;
  float Cal_TData1, Cal_TData2, Temp_data;

  I2C_WriteReg(0x30, 0x0A);
  vTaskDelay(pdMS_TO_TICKS(100));
  I2C_ReadNByte(0x06, Pressure, 3);

  data[0] = Pressure[0];
  data[1] = Pressure[1];
  data[2] = Pressure[2];
  Cal_PData1 = data[0] * 65536 + data[1] * 256 + data[2];
  Cal_PData2 = (Cal_PData1 > 8388608) ? 
                (Cal_PData1 - 16777216) / 8388608.0 :
                 Cal_PData1 / 8388608.0;
  Pressure_data = Cal_PData2 * Fullscale_P;

  I2C_ReadNByte(0x09, Temp, 2);
  data[0] = Temp[0];
  data[1] = Temp[1];
  Cal_TData1 = data[0] * 256 + data[1];
  Cal_TData2 = (Cal_TData1 > 32768) ? 
                (Cal_TData1 - 65536) / 256.0 :
                 Cal_TData1 / 256.0;
  Temp_data = Cal_TData2;

  Serial.printf("Presión: %.2f kPa | Temperatura: %.2f °C\n", Pressure_data, Temp_data);
}

// --- TASK ---
void sensorTask(void *pvParameters) {
  if (heltec_wakeup_was_timer()) {
    both.println("¡Me desperté y leo sensor!");
    Sensor();
  }
  vTaskDelay(pdMS_TO_TICKS(2000));
  both.println("Entrando en deep sleep...");
  vTaskDelay(pdMS_TO_TICKS(100));
  heltec_deep_sleep();
}

// --- SETUP ---
void setup() {
  heltec_setup();
  Serial.begin(115200);
  delay(100);

  Wire1.begin(I2C_SDA_SENSOR, I2C_SCL_SENSOR);
  i2c_mutex = xSemaphoreCreateMutex();

  esp_sleep_enable_timer_wakeup(10 * 1000000ULL); // 10 segundos

  if (i2c_mutex != NULL) {
    xTaskCreate(sensorTask, "SensorTask", 4096, NULL, 1, NULL);
  }
}

void loop() {
  // No se usa
}