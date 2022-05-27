#include <Wire.h>
#include <PubSubClient.h>
#include <ESP8266WiFiMulti.h>
#include <ArduinoJson.h>
//#include "WiFiTask.h"


#include "secure.hpp"

//----------------------------------------------------
// Detalhes do firmware

#define Version "1.0.0.1"
#define MakeFirmwareInfo(k, v) "&_FirmwareInfo&k=" k "&v=" v "&FirmwareInfo_&"

//----------------------------------------------------
// Definição dos pinos de comunicação I2C com o MPU6050

#define MPU_SDA D6
#define MPU_SCL D5

#define MPU_POW D1

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
int n_pack = 1;

//const int n_sensors = 7;
unsigned long long sample_period = 5000;

//----------------------------------------------------
// Variáveis e constantes relativas à extração de caracteristicas

const int window_size = 11;
const int window_border = window_size / 2;
double rms_acc;
double rms_gyr;

//----------------------------------------------------
// Variáveis e constantes relativas à extração de caracteristicas

const int n_classes = 3;
double threshold[n_classes] = {7255, 7666, 9672};
int result_class = 0;

//----------------------------------------------------
// Variáveis e constantes de armazenamento de dados

//int16_t buffer[n_sensors];
int16_t MPU_data[n_capt][6];
int16_t MPU_temp[n_capt];
double MSD[n_capt - (2 * window_border)][6];

//----------------------------------------------------
// Variáveis e constantes gerais

String names[7] = {"AcX:", ",AcY:", ",AcZ:", ",GyX:", ",GyY:", ",GyZ:", ",Tmp:"};

//----------------------------------------------------
// Variáveis e constantes gerais

int command;
String status;

unsigned long pressedTime  = 0;
unsigned long releasedTime = 0;
int currentState;     // the current reading from the input pin
int lastState = HIGH;  // the previous state from the input pin
float D0Value = 0;
float D4Value = 0;
bool captErrorFlag = HIGH;
bool netErrorFlag = LOW;


////////////////////////////////////////////////////////  MQTT CALLBACK  ////////////////////////////////////////////////////////

void callback(char *topic, byte *payload, unsigned int length)
{
  StaticJsonDocument<256> mqtt_input;
  deserializeJson(mqtt_input, payload);

  Serial.println("Mensagem recebida");
  command = mqtt_input["cmd"];


  n_pack = mqtt_input["npk"];
  //n_capt = mqtt_input["ncp"];
  sample_period = mqtt_input["spe"];

  status  = "Comando recebido: ";
  status += command ;
  status += " -- ";
  status += n_pack;
  status += " pacotes -- ";
  status += n_capt;
  status += " capturas -- " ;
  status += sample_period;
  status += "ms ";

  Serial.println(status);

  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

////////////////////////////////////////////////////////  OBJECTS  //////////////////////////////////////////////////////


ESP8266WiFiMulti wifiMulti;

WiFiClient espClient;
PubSubClient mqtt(IO_SERVER, IO_SERVERPORT, callback, espClient);


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////// SETUP //////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  pinMode(0, INPUT_PULLUP);

  pinMode(MPU_POW , OUTPUT);
  pinMode(D0      , OUTPUT);
  pinMode(D4      , OUTPUT);

  Serial.begin(500000);

  Serial.println("Sistema de monitoramento de condição - LARI IFCE");
  Serial.println("Desenvolvido por: Rodrigo D. B. de Araújo");
  Serial.print("Versão do firmware: ");
  Serial.println(Version);
  Serial.println("\n");

  setWifi();

  digitalWrite(MPU_POW, HIGH);
  delay(500);

  Wire.begin(MPU_SDA, MPU_SCL);

  wakeupMPU();

  command = 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////  LOOP  ////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop()
{
  updateVisualFeedback();

  if (checkMPU())
  {
    command = -1;
    captErrorFlag = HIGH;
  }
  else
  {
    captErrorFlag = LOW;
  }

  if (netErrorFlag==LOW)
  {
    mqtt.loop();

    if (!mqtt.connected())
    {
      setMqtt();
    }
  }
  /*
    if (wifiMulti.run() != WL_CONNECTED)
    {
    setWifi();
    }

  */


  currentState = digitalRead(0);

  if (lastState == HIGH && currentState == LOW)       // button is pressed
    pressedTime = millis();
  else if (lastState == LOW && currentState == HIGH)
  { // button is released
    releasedTime = millis();

    long pressDuration = releasedTime - pressedTime;

    if (pressDuration > 2000)
    {
      Serial.println("A long press is detected");
      digitalWrite(D0, 0);
      delay(10);
      digitalWrite(D0, 1);
      delay(100);
      digitalWrite(D0, 0);
      delay(10);
      digitalWrite(D0, 1);
      checkMPU();

    } else
    {
      //configActive = 0;

      command++;
      if (command > 2)
        command = 0;

      switch (command)
      {
        case 1:
          Serial.println("\n1 - ANÁLISE EM TEMPO REAL\n");
          break;
        case 2:
          Serial.println("\n2 - CLASSIFICAÇÃO LOCAL CONTÍNUA\n");
          break;
      }
    }

  }
  lastState = currentState;


/*
  switch (command)
  {
    case 1: // Analise em tempo real
      {

        readRawMPU(1);
        //Serial.println("Print here");
        printMPU();
        //mqtt.loop();
        break;
      }
    case 2: // Classificação de dados
      {

        readRawMPU(n_capt);
        featureExtraction();
        dataClassification();

        sendClass();

        //mqtt.loop();

        delay(10000);
        break;
      }
    case 3: // Envio de dados (analise rapida)
      {
        for (int packet = 0; packet < n_pack; packet++)
        {
          readRawMPU(n_capt);
          sendData();
          //mqtt.loop();
        }

        command = 0;
        // Envia MQTT bruto
        break;
      }
    case -1:
      {
        Serial.println("Verifique dispositivo - MPU6050 NÃO disponível!");
        digitalWrite(MPU_POW, LOW);
        delay(500);
        digitalWrite(MPU_POW, HIGH);
        delay(500);
        wakeupMPU();

        break;
      }

    default:
      {
        Serial.println("NENHUM COMANDO RECEBIDO.");
        break;
      }
  }
  */
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////  MPU CONTROL  ////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool checkMPU()
{
  //findMPU(MPU_ADDR);

  if (readRegMPU(WHO_AM_I) == 104)
  {
    //Serial.println("MPU6050 Dispositivo respondeu OK! (104)");

    //if(readRegMPU(PWR_MGMT_1) == 64) Serial.println("MPU6050 em modo SLEEP! (64)");
    //else Serial.println("MPU6050 em modo ACTIVE!");

    return 0;
  }
  return 1;
}


uint8_t readRegMPU(uint8_t reg)        // aceita um registro como parâmetro
{
  uint8_t data;
  Wire.beginTransmission(MPU_ADDR);     // inicia comunicação com endereço do MPU6050
  Wire.write(reg);                      // envia o registro com o qual se deseja trabalhar
  Wire.endTransmission(false);          // termina transmissão mas continua com I2C aberto (envia STOP e START)
  Wire.requestFrom(MPU_ADDR, 1);        // configura para receber 1 byte do registro escolhido acima
  data = Wire.read();                   // lê o byte e guarda em 'data'
  return data;                          //retorna 'data'
}

//---------------------------------------------------------------


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
/*
  void readMPU()
  {
  prevCheckTime = micros();
  int capt_counter = 0;

  while (capt_counter < n_capt)
  {
    currTime = micros();
    interval = currTime - prevCheckTime;

    if ( interval >= sample_period)
    {
      //delay(sample_period);
      prevCheckTime = currTime;
      yield();

      //Serial.printf("Captura %i: ", capt_counter);

      Wire.beginTransmission(MPU_ADDR);
      Wire.write(ACCEL_XOUT);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU_ADDR, (uint8_t)(14));

      //String message = "";
      for (int axis = 0; axis < 7; axis++) // LÊ OS DADOS DE ACC
      {
        MPU_data[capt_counter][axis] = Wire.read() << 8 | Wire.read();
        //message += names[axis];
        //message += MPU_data[capt_counter][axis];
      }

      for (int i = 0; i < 3; i++) // LÊ OS DADOS DE ACC
      {
        MPU_acc[capt_counter][i] = Wire.read() << 8 | Wire.read();
      }

      MPU_temp[capt_counter] = Wire.read() << 8 | Wire.read(); //LÊ OS DADOS DE TEMP

      for (int i = 0; i < 3; i++) // LÊ OS DADOS DE GYRO
      {
        MPU_gyr[capt_counter][i] = Wire.read() << 8 | Wire.read();
      }

      //Serial.print(message);
      //Serial.println();
      capt_counter++;
    }
  }
  }
*/
void readRawMPU(int iterations)
{

  prevCheckTime = micros();
  int capt_counter = 0;

  while (capt_counter < iterations)
  {
    currTime = micros();
    interval = currTime - prevCheckTime;

    if (interval >= sample_period)
    {
      //Serial.println(capt_counter);
      //Serial.println(interval);
      //delay(sample_period);
      prevCheckTime = currTime;
      yield();

      //Serial.printf("Captura %i: ", capt_counter);

      Wire.beginTransmission(MPU_ADDR);
      Wire.write(ACCEL_XOUT);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU_ADDR, (uint8_t)(14));

      for (int i = 0; i < 3; i++) // LÊ OS DADOS DE ACC
        MPU_data[capt_counter][i] = Wire.read() << 8 | Wire.read();

      MPU_temp[capt_counter] = Wire.read() << 8 | Wire.read(); //LÊ OS DADOS DE TEMP

      for (int i = 3; i < 6; i++) // LÊ OS DADOS DE GYR
        MPU_data[capt_counter][i] = Wire.read() << 8 | Wire.read();

      capt_counter++;
    }
  }
}

void printMPU()
{
  for (int i = 0; i < 6; i++) // LÊ OS DADOS DE ACC
  {
    Serial.print(names[i]);
    Serial.print(MPU_data[0][i]);
  }
  //Serial.print(names[6]);
  //Serial.print(MPU_temp[0]);
  Serial.println();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////  ESP CONTROL  ////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setWifi()
{
  WiFi.mode(WIFI_STA);

  for (int i = 0; i < network_number; i++)
  {
    wifiMulti.addAP(WLAN_SSID[i], WLAN_PASS[i]);
  }

  Serial.println("Conectando à rede Wi-Fi...");
  for (int tries = 5; wifiMulti.run(1000) != WL_CONNECTED && tries >= 0; tries--)
  {
    Serial.print(".");

    if (tries == 0)
    {
      netErrorFlag = HIGH;
      Serial.println("WiFi não encontrado. Modo offline ativado.");
      
    }
  }
}

void sendData()
{
  String out_msg = "";
  mqtt.publish(topicESP_SCADA, "AcX,AcY,AcZ,GyX,GyY,GyZ,Tmp");

  for (int i = 0; i < n_capt; i++) {
    out_msg = "";

    for (int j = 0; j < 6; j++)
    {
      //out_msg += names[j];
      out_msg += MPU_data[i][j];
      out_msg += ",";
    }
    out_msg += MPU_temp[i];
    mqtt.publish(topicESP_SCADA, out_msg.c_str());
    Serial.println(out_msg);
  }
  mqtt.publish(topicESP_SCADA, "fim");
}

void sendClass()
{
        //Serial.printf("RMS_ACC: %f,", rms_acc);
        //Serial.printf("RMS_GYR: %f,", rms_gyr);
        String acc_res = "", gyr_res = "", class_res = "";

        acc_res += rms_acc;
        gyr_res += rms_gyr;
        class_res += result_class;
        
        mqtt.publish(topic_acc_rms, acc_res.c_str());
        mqtt.publish(topic_gyr_rms, gyr_res.c_str());
        mqtt.publish(topic_acc_class, class_res.c_str());
}

void setMqtt()
{
  // Loop until we're reconnected
  if (!mqtt.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (mqtt.connect(clientId.c_str(), "cliente", "cliente"))
    {
      Serial.println("connected");
      // Once connected, publish an announcement...
      // mqtt.publish(topico_saida, "hello world");
      // ... and resubscribe
      mqtt.subscribe(topicSCADA_ESP);
    }
    else
    {
      // Serial.print("failed, rc=");
      // Serial.print(mqtt.state());
      //Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(500);
    }
  }
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////  MODEL CONTROL  ////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void featureExtraction()
{
  for (int axis = 0; axis < 6; axis++)
  {

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
      //Serial.printf("\nMedia = %f\n", msd_mean);

      // 2. Variancia

      double msd_variance = 0;

      for (int i = (centralElement - window_border); i <= centralElement + window_border; i++)
      {
        msd_variance += pow((MPU_data[i][axis] - msd_mean), 2);
      }

      msd_variance /= window_size ;//- 1;
      MSD[centralElement - window_border][axis] = sqrt(msd_variance);
      //Serial.printf("\nDesvio padrao da janela = %f\n",MSD[centralElement - window_border][axis]);
    }
  }

  double media[6] = {0, 0, 0, 0, 0, 0};

  for (int position = 0; position < (n_capt - (2 * window_border)); position++)
  {
    for (int axis = 0; axis < 6; axis++)
      media[axis] += (MSD[position][axis]);
  }

  for (int axis = 0; axis < 6; axis++) {
    media[axis] /= (n_capt - (2 * window_border));
    //Serial.printf("\nMedia do sensor %i = %f\n", axis, media[axis]);
  }

  rms_acc = 0;
  rms_gyr = 0;

  for (int axis = 0; axis < 3; axis++)
  {
    rms_acc += pow(media[axis  ], 2);
    rms_gyr += pow(media[axis + 4], 2);
  }

  rms_acc = sqrt(rms_acc / 3);
  rms_gyr = sqrt(rms_gyr / 3);

  //rms -= 500;
  Serial.printf("RMS_ACC: %f,", rms_acc);
  Serial.printf("RMS_GYR: %f,", rms_gyr);
}

void dataClassification()
{
  result_class = 0;

  for (int i = 0; i < (n_classes - 1); i++)
  {
    if (rms_acc >= threshold[i])
    {
      result_class = i + 1;
    }
  }

  Serial.printf("CLASS: %i\n", result_class);
}

void updateVisualFeedback()
{
  //  if (netErrorFlag)
  //  {
  analogWrite(D0, abs(-cos(D0Value) * 128 - 128));
  D0Value += (0.05 * netErrorFlag);

  if (D0Value >= 2 * M_PI)
  {
    D0Value = 0;
  }
  /* }
    else
    {
     digitalWrite(D1, HIGH);
    }
  */
  analogWrite(D4, sin(D4Value) * 128 + 128);
  D4Value += (0.005 + 0.019 * captErrorFlag);

  if (D4Value >= 2 * M_PI)
  {
    D4Value = 0;
  }

}
