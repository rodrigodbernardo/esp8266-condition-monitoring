#include <Wire.h>
#include <PubSubClient.h>
#include <ESP8266WiFiMulti.h>
#include <ArduinoJson.h>

#include "secure.hpp"

//----------------------------------------------------
// Detalhes do firmware

#define Version "1.0.0.0"
#define MakeFirmwareInfo(k, v) "&_FirmwareInfo&k=" k "&v=" v "&FirmwareInfo_&"

//----------------------------------------------------
// Definição dos pinos de comunicação I2C com o MPU6050

#define MPU_SDA D6
#define MPU_SCL D5

//----------------------------------------------------
// Definição dos endereços de memória no MPU6050

const uint8_t MPU_ADDR = 0x68;
const uint8_t WHO_AM_I = 0x75;
const uint8_t PWR_MGMT_1 = 0x6B;
const uint8_t GYRO_CONFIG = 0x1B;  // Registrador que configura a escala do giroscópio.
const uint8_t ACCEL_CONFIG = 0x1C; // Registrador que configura a escala do acelerômetro.
const uint8_t ACCEL_XOUT = 0x3B;   //
const uint8_t GYRO_SCALE = 8;      // Escala do giroscópio
const uint8_t ACCEL_SCALE = 8;     // Escala do acelerômetro

//----------------------------------------------------
// Variáveis e constantes de temporização

unsigned long long prevCheckTime = 0;
unsigned long long currTime;
unsigned long long sample_period = 1600;
unsigned long long interval = 0;

//----------------------------------------------------
// Variáveis e constantes de armazenamento de dados

int16_t buffer[7];

//----------------------------------------------------
// Variáveis e constantes gerais

String names[7] = {"AcX:", ",AcY:", ",AcZ:", ",GyX:", ",GyY:", ",GyZ:", ",Tmp:"};

//----------------------------------------------------
// Variáveis e constantes gerais

String command;

//----------------------------------------------------
// Objetos

ESP8266WiFiMulti wifiMulti;

//----------------------------------------------------
//----------------------------------------------------
//----------------------------------------------------
// Inicio da rotina

void setup()
{
  Serial.begin(500000);
  delay(1000);

  Serial.println("Sistema de monitoramento de condição - LARI IFCE");
  Serial.println("Desenvolvido por: Rodrigo D. B. de Araújo");
  Serial.print("Versão do firmware: ");
  Serial.println(Version);
  Serial.println("\n");

  Wire.begin(MPU_SDA, MPU_SCL);

  sensor_wakeup();

  if (wifiMulti.run() != WL_CONNECTED)
  {
    esp_setwifi();
  }

  command = "monitoramento-de-condicao";
}

void loop() {
  while (command == "analise-em-tempo-real")
  {
    // Inicio da rotina de captura de dados

    currTime = micros();
    interval = currTime - prevCheckTime;
    if (interval >= sample_period)
    {
      if (wifiMulti.run() != WL_CONNECTED)
        esp_setwifi();


      //Serial.printf("Intervalo atual: %i\n", interval);
      Serial.print(interval);

      //yield();

      sensor_read();
      sensor_print();
      // sensor_send_data_continuous();
      // mqtt.loop();

      prevCheckTime = currTime; // Testar com micros();
    }
  }

  while (command == "monitoramento-de-condicao")
  {
    sensor_read(100);

    currTime = micros();
    interval = currTime - prevCheckTime;
    if (interval >= sample_period)
    {
      if (wifiMulti.run() != WL_CONNECTED)
        esp_setwifi();


      //Serial.printf("Intervalo atual: %i\n", interval);
      Serial.print(interval);

      //yield();

      sensor_read();
      sensor_print();
      // sensor_send_data_continuous();
      // mqtt.loop();

      prevCheckTime = currTime; // Testar com micros();
    }
  }
}

//----------------------------------------------------
//----------------------------------------------------
// Funções de controle do MPU6050

void sensor_write(int reg, int val)
{
  //
  // COMUNICACAO I2C COM O SENSOR - NAO MODIFICAR
  //
  Wire.beginTransmission(MPU_ADDR); // inicia comunicação com endereço do MPU6050
  Wire.write(reg);                  // envia o registro com o qual se deseja trabalhar
  Wire.write(val);                  // escreve o valor no registro
  Wire.endTransmission();           // termina a transmissão
}

void sensor_wakeup()
{
  //
  // INICIA A CONFIGURAÇÃO DO SENSOR
  //
  sensor_write(PWR_MGMT_1, 0);             // ACORDA O SENSOR
  sensor_write(GYRO_CONFIG, GYRO_SCALE);   // CONFIGURA A ESCALA DO GIROSCÓPIO - +-250 °/s -->
  sensor_write(ACCEL_CONFIG, ACCEL_SCALE); // CONFIGURA A ESCALA DO ACELERÔMETRO - +-4G
}

void sensor_read()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)6);

  for (int axis = 0; axis < 7; axis++) // LÊ OS DADOS DE ACC
    buffer[axis] = Wire.read() << 8 | Wire.read();
}

void sensor_print()
{
  /*
    for (int j = 0; j < 7; j++)
    {
    Serial.print(names[j]);
    Serial.print(v_data[0][j]);
    }
  */
  for (int j = 0; j < 3; j++)
  {
    Serial.print(names[j]);
    Serial.print(buffer[j]);
  }
  Serial.println();
}

//----------------------------------------------------
//----------------------------------------------------
// Funções de controle do ESP8266

void esp_setwifi()
{
  WiFi.mode(WIFI_STA);

  for (int i = 0; i < network_number; i++)
  {
    wifiMulti.addAP(WLAN_SSID[i], WLAN_PASS[i]);
  }

  Serial.print("Conectando à rede Wi-Fi.");
  for (int retry = 15; (retry >= 0 && wifiMulti.run() != WL_CONNECTED); retry--)
  {
    if (retry == 0)
      while (1)
        ;
    Serial.print(".");
  }

  Serial.print("\nWi-Fi conectada. IP ");
  Serial.println(WiFi.localIP());
}
