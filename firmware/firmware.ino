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
    Serial.print("Versão do firmware: "); Serial.println(Version);
    Serial.println("\n");

    Wire.begin(MPU_SDA, MPU_SCL);

    sensor_wakeup();
}

void loop()
{
    if (wifiMulti.run() != WL_CONNECTED)
    {
        esp_setwifi();
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

//----------------------------------------------------
//----------------------------------------------------
// Funções de controle do ESP8266

void esp_setwifi()
{
  WiFi.mode(WIFI_STA);

  for (int i = 0; i < network_number; i++){
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
