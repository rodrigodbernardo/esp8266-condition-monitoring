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

unsigned long long interval = 0;

//----------------------------------------------------
// Variáveis e constantes referentes às especificações do sistema

const int n_capt = 100;
const int n_sensors = 3;
unsigned long long sample_period = 100;

//----------------------------------------------------
// Variáveis e constantes relativas à extração de caracteristicas

const int window_size = 11;
const int window_border = window_size / 2;
double rms;

//----------------------------------------------------
// Variáveis e constantes de armazenamento de dados

int16_t buffer[n_sensors];
int16_t MPU_data[n_capt][n_sensors];
double MSD[n_capt - (2*window_border)][n_sensors];

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

  wakeupMPU();

  if (wifiMulti.run() != WL_CONNECTED)
  {
    setWifi();
  }

  command = "monitoramento-de-condicao";
}

void loop()
{
  delay(10000);
  Serial.println("Preparando para capturar dados");

  readMPU();

  featureExtraction();
  dataClassification();

  sendData();

  Serial.println("Captura realizada. Entrando em estado de hibernação...");
}

//----------------------------------------------------
//----------------------------------------------------
// Funções de controle do MPU6050

void writeMPU(int reg, int val)
{
  //
  // COMUNICACAO I2C COM O SENSOR - NAO MODIFICAR
  //
  Wire.beginTransmission(MPU_ADDR); // inicia comunicação com endereço do MPU6050
  Wire.write(reg);                  // envia o registro com o qual se deseja trabalhar
  Wire.write(val);                  // escreve o valor no registro
  Wire.endTransmission();           // termina a transmissão
}

void wakeupMPU()
{
  //
  // INICIA A CONFIGURAÇÃO DO SENSOR
  //
  writeMPU(PWR_MGMT_1, 0);             // ACORDA O SENSOR
  writeMPU(GYRO_CONFIG, GYRO_SCALE);   // CONFIGURA A ESCALA DO GIROSCÓPIO - +-250 °/s -->
  writeMPU(ACCEL_CONFIG, ACCEL_SCALE); // CONFIGURA A ESCALA DO ACELERÔMETRO - +-4G
}

void readMPU()
{
  for (int capture = 0; capture < n_capt; capture++)
  {
    delay(sample_period);
    Serial.printf("Captura %i", capture);

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(ACCEL_XOUT);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)(n_sensors * 2));

    for (int axis = 0; axis < n_sensors; axis++) // LÊ OS DADOS DE ACC
      MPU_data[capture][axis] = Wire.read() << 8 | Wire.read();
  }
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

void setWifi()
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

void featureExtraction()
{
  for (int axis = 0; axis < n_sensors; axis++){

    // Calcula o MSD

    for (int centralElement = window_border; centralElement < n_capt - window_border; centralElement++)
	{
      // 1. Media
          
      double msd_mean = 0;
      for (int i = (centralElement - window_border); i <= centralElement + window_border; i++)
      {
        msd_mean += (MPU_data[i][axis]);
      }
      msd_mean /= window_size;
        
      // 2. Variancia
      
      double msd_variance = 0;
      
      for (int i = (centralElement - window_border); i <= centralElement + window_border; i++){
        msd_variance += pow((MPU_data[i][axis] - msd_mean),2);
      }
    
      msd_variance /= window_size-1;    
      MSD[centralElement - window_border][axis] = sqrt(msd_variance);
    }
  }
  
  double media[3] = {0,0,0};

  for (int position = 0; position < (n_capt - (2 * window_border)); position++)
  {
    for (int axis = 0; axis < n_sensors; axis++)
    	media[axis] += (MSD[position][axis]);
  }
  
  for (int axis = 0; axis < n_sensors; axis++)
  	media[axis] /= (n_capt - (2 * window_border));

  for(int axis = 0; axis < n_sensors; axis++)
    rms += pow(media[axis],2);

  rms = sqrt(rms/3);
  Serial.printf("\nRMS final = %f",rms);

}

void dataClassification()
{
}

void sendData()
{
}
