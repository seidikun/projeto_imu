#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#include <Wire.h>

// CONSTANTES --------------------------------------------------------------------------------------------------------*/

// MPU6050
const uint8_t kSdaPin = 21; // Armazena GPIO conectados ao pino I2C SDA do MPU6050.
const uint8_t kSclPin = 22; // Armazena GPIO conectado ao pino I2C SCL do MPU6050.

const uint8_t kMpu6050SlaveAddress       = 0x68; // Endereço de dispositivo escravo MPU6050/MPU6500.
const uint16_t kAccelerometerScaleFactor = 2; // Sensibilidade MPU6050 16384 (2g), 8192 (4g), 4096 (8g), 2048 (16g).
const uint16_t kGyroscopeScaleFactor     = 131; // Sensibilidade MPU6050 131 (2g), 655 (4g), 328 (8g), 164 (16g).

const uint8_t kMpu6050RegisterSmplrtDiv     = 0x19; // Endereço do registrador 25 Sample Rate Divider.          Same as MPU6500
const uint8_t kMpu6050RegisterUserCtrl      = 0x6A; // Endereço do registrador 106 User Control.                Same as MPU6500
const uint8_t kMpu6050RegisterPwrMgmt1      = 0x6B; // Endereço do registrador 107 Power Management 1.          Same as MPU6500
const uint8_t kMpu6050RegisterPwrMgmt2      = 0x6C; // Endereço do registrador 108 Power Management 2.          Same as MPU6500
const uint8_t kMpu6050RegisterConfig        = 0x1A; // Endereço do registrador 26 Configuration.                Same as MPU6500
const uint8_t kMpu6050RegisterGyroConfig    = 0x1B; // Endereço do registrador 27 Gyroscope Configuration.      Same as MPU6500
const uint8_t kMpu6050RegisterAccelConfig   = 0x1C; // Endereço do registrador 28 Accelerometer Configuration.  Same as MPU6500
const uint8_t kMpu6050RegisterFifoEn        = 0x23; // Endereço do registrador 35 FIFO Enable.                  Same as MPU6500
const uint8_t kMpu6050RegisterIntEnable     = 0x38; // Endereço do registrador 56 Interrupt Enable.             Same as MPU6500
const uint8_t kMpu6050RegisterAccelXoutH    = 0x3B; // Endereço do registradores 59 Accelerometer Measurements. Same as MPU6500
const uint8_t kMpu6050RegisterSignalReset   = 0x68; // Endereço do registrador 104 Signal Path Reset.           Same as MPU6500

 // VARIÁVEIS GLOBAIS -------------------------------------------------------------------------------------------------*/

uint16_t kPersistenceSamplingInterval = 60000; // Frequência de amostragem para persistência de dados no Data Bucket.
uint16_t kRealTimeSamplingInterval = 500; // Frequência de amostragem para exibição em tempo real no Dashboard .

int16_t accel_x_raw_value, accel_y_raw_value, accel_z_raw_value; // Armazenarão dados de acelerômetro "brutos".
int16_t gyro_x_raw_value, gyro_y_raw_value, gyro_z_raw_value; // Armazenarão dados de giroscópio "brutos".
int16_t temperature_raw_value; // Armazenarão dados de temperatura "brutos".

bool debug_state = false; // Armazerá estado de impressão de dados dos sensores via serial monitor.

typedef struct message {
  int id;
  double time;
  double accel_x_value, accel_y_value, accel_z_value; // Armazenarão dados de acelerômetro em valores angulares.
  double gyro_x_value, gyro_y_value, gyro_z_value; // Armazenarão dados de giroscópio em valores angulares.
  double temperature_value; // Armazenarão dados de temperatura em graus Celsius.
}; 
 
struct message myMessage;
 
// DECLARAÇÃO DE FUNÇÕES GLOBAIS -------------------------------------------------------------------------------------*/

void Mpu6050Init(void); // Configuração/Inicialização MPU6050.
void ReadRawValue(uint8_t, uint8_t); // Aquisição de dados brutos dos sensores do MPU6050.
void ConvertRawToAngularValues(void); // Conversão de dados brutos em valores angulares.
void I2CWrite(uint8_t device_address, uint8_t reg_address, uint8_t data); // Escrita de registradores.
void debug(bool debug_state); // Impressão de dados via serial para "debug".


void onDataReceiver(const uint8_t * mac, const uint8_t *incomingData, int len) {
  //Serial.println("Message received.");
  // Não usamos mac para verificar o remetente
  // Vamos transformar os dados recebidos em nossa estrutura de mensagem
  memcpy(&myMessage, incomingData, sizeof(myMessage));
  //Serial.println("=== Data ===");
  //Serial.print("endereço MAC: ");
  //for (int i = 0; i < 6; i++) {
  //    Serial.print("0x");
  //    Serial.print(mac[i], HEX);
  //    Serial.print(":");
  //}
      
  //Serial.print("Ax: ");  Serial.print(myMessage.accel_x_value); // Enviado dado do eixo X do acelerômetro via Serial.
  //Serial.print("\tAy: "); Serial.print(myMessage.accel_y_value); // Enviado dado do eixo Y do acelerômetro via Serial.
  //Serial.print("\tAz: "); Serial.print(myMessage.accel_z_value); // Enviado dado do eixo Z do acelerômetro via Serial.
  //Serial.print("\tT: ");  Serial.print(myMessage.temperature_value); // Enviado dado do de temperatura via Serial.
  //Serial.print("\tGx: "); Serial.print(myMessage.gyro_x_value); // Enviado dado do eixo X do giroscópio via Serial.
  //Serial.print("\tGy: "); Serial.print(myMessage.gyro_y_value); // Enviado dado do eixo Y do giroscópio via Serial.
  //Serial.print("\tGz: "); Serial.println(myMessage.gyro_z_value); // Enviado dado do eixo Z do giroscópio via Serial.
  
  Serial.print(myMessage.id);
  Serial.print(";"); Serial.print(myMessage.time); // Enviado dado do eixo X do acelerômetro via Serial.
  Serial.print(";"); Serial.print(myMessage.accel_x_value); // Enviado dado do eixo X do acelerômetro via Serial.
  Serial.print(";"); Serial.print(myMessage.accel_y_value); // Enviado dado do eixo Y do acelerômetro via Serial.
  Serial.print(";"); Serial.print(myMessage.accel_z_value); // Enviado dado do eixo Z do acelerômetro via Serial.
  Serial.print(";"); Serial.print(myMessage.temperature_value); // Enviado dado do de temperatura via Serial.
  Serial.print(";"); Serial.print(myMessage.gyro_x_value); // Enviado dado do eixo X do giroscópio via Serial.
  Serial.print(";"); Serial.print(myMessage.gyro_y_value); // Enviado dado do eixo Y do giroscópio via Serial.
  Serial.print(";"); Serial.println(myMessage.gyro_z_value); // Enviado dado do eixo Z do giroscópio via Serial.
}
 
void setup() {
 Serial.begin(115200);
 WiFi.mode(WIFI_STA);
 
 // Obter Mac Add
 Serial.print("Endereço MAC:");
 Serial.print(WiFi.macAddress());
 Serial.println("ESP32 ESP-Now Broadcast");
 
 // Inicializando o ESP-NOW
 if (esp_now_init() != 0) {
   Serial.println("Problema durante a inicialização ESP-NOW");
   return;
 }

 Serial.print("Id;Time;Ax;Ay;Az;T;Gx;Gy;Gz");  
 esp_now_register_recv_cb(onDataReceiver);

 //Wire.begin(kSdaPin, kSclPin); // Configurando comunicação I2C de acordo com os pinos definidos para SDA e SCL.
 //Mpu6050Init(); // Configurando/Inicializando I2C.

}
 
void loop() {
 // coloque seu código principal aqui, para executar repetidamente:
                 // Wait for two seconds (to demonstrate the active low LED)

  //ReadRawValue(kMpu6050SlaveAddress, kMpu6050RegisterAccelXoutH); // Obtendo dados brutos do sensor MPU6050.
  //ConvertRawToAngularValues(); // Convertendo dados brutos do sensor MPU6050 em valores angulares.
  //debug(debug_state); // Envia dados via serial caso debug_state seja definido como true.

}


// DEFINIÇÃO DE FUNÇÕES GLOBAIS --------------------------------------------------------------------------------------*/

// Realiza escrita nos registradores do MPU6050.
void I2CWrite(uint8_t device_address, uint8_t reg_address, uint8_t data) {
    Wire.beginTransmission(device_address); // Iniciando transmissão dos dados via I2C.
    Wire.write(reg_address);                // Adicionando ao buffer de transmissão o endereço do registrador a ser escrito.
    Wire.write(data);                       // Adicioando dado a ser escrito no registrador definido na linha anterior.
    Wire.endTransmission();                 // Encerrando transmissão dos dados.
}

// Realiza leitura dos dados dos 14 registradores do MPU6050.
void ReadRawValue(uint8_t device_address, uint8_t reg_address) {
    Wire.beginTransmission(device_address); // Iniciando transmissão dos dados via I2C.
    Wire.write(reg_address); // Adicionando endereço do registrador de dados ao buffer de transmissão.
    Wire.endTransmission(); // Encerrando transmissão dos dados.
    Wire.requestFrom(device_address, (uint8_t)14); // Solicitando dados do dispositivo MPU6050 escravo.
    accel_x_raw_value     = (((int16_t)Wire.read() << 8) | Wire.read()); // Lendo dado bruto do eixo X do acelerômetro.
    accel_y_raw_value     = (((int16_t)Wire.read() << 8) | Wire.read()); // Lendo dado bruto do eixo Y do acelerômetro.
    accel_z_raw_value     = (((int16_t)Wire.read() << 8) | Wire.read()); // Lendo dado bruto do eixo Z do acelerômetro.
    temperature_raw_value = (((int16_t)Wire.read() << 8) | Wire.read()); // Lendo dado do de temperatura.
    gyro_x_raw_value      = (((int16_t)Wire.read() << 8) | Wire.read()); // Lendo dado bruto do eixo X do giroscópio.
    gyro_y_raw_value      = (((int16_t)Wire.read() << 8) | Wire.read()); // Lendo dado bruto do eixo Y do giroscópio.
    gyro_z_raw_value      = (((int16_t)Wire.read() << 8) | Wire.read()); // Lendo dado bruto do eixo Z do giroscópio.
}

// Converte dados de acelerômetro e giroscópio em valores angulares e temperatura em Celsius.
void ConvertRawToAngularValues() {
    myMessage.accel_x_value     = (double)accel_x_raw_value / kAccelerometerScaleFactor;
    myMessage.accel_y_value     = (double)accel_y_raw_value / kAccelerometerScaleFactor;
    myMessage.accel_z_value     = (double)accel_z_raw_value / kAccelerometerScaleFactor;
    myMessage.gyro_x_value      = (double)gyro_x_raw_value / kGyroscopeScaleFactor;
    myMessage.gyro_y_value      = (double)gyro_y_raw_value / kGyroscopeScaleFactor;
    myMessage.gyro_z_value      = (double)gyro_z_raw_value / kGyroscopeScaleFactor;
    myMessage.temperature_value = (double)temperature_raw_value / 340 + 36.53;
}

// Configura MPU6050.
void Mpu6050Init() {
    delay(150);
    I2CWrite(kMpu6050SlaveAddress, kMpu6050RegisterSmplrtDiv, 0x07);
    I2CWrite(kMpu6050SlaveAddress, kMpu6050RegisterPwrMgmt1, 0x01);
    I2CWrite(kMpu6050SlaveAddress, kMpu6050RegisterPwrMgmt2, 0x00);
    I2CWrite(kMpu6050SlaveAddress, kMpu6050RegisterConfig, 0x00);
    I2CWrite(kMpu6050SlaveAddress, kMpu6050RegisterGyroConfig, 0x00);
    I2CWrite(kMpu6050SlaveAddress, kMpu6050RegisterAccelConfig, 0x00);
    I2CWrite(kMpu6050SlaveAddress, kMpu6050RegisterFifoEn, 0x00);
    I2CWrite(kMpu6050SlaveAddress, kMpu6050RegisterIntEnable, 0x01);
    I2CWrite(kMpu6050SlaveAddress, kMpu6050RegisterSignalReset, 0x00);
    I2CWrite(kMpu6050SlaveAddress, kMpu6050RegisterUserCtrl, 0x00);
}

// Imprime/Envia dados do sensor via serial.
void debug(bool debug_state) {
    if(debug_state) {
        Serial.print("Ax: ");  Serial.print(myMessage.accel_x_value); // Enviado dado do eixo X do acelerômetro via Serial.
        Serial.print(" Ay: "); Serial.print(myMessage.accel_y_value); // Enviado dado do eixo Y do acelerômetro via Serial.
        Serial.print(" Az: "); Serial.print(myMessage.accel_z_value); // Enviado dado do eixo Z do acelerômetro via Serial.
        Serial.print(" T: ");  Serial.print(myMessage.temperature_value); // Enviado dado do de temperatura via Serial.
        Serial.print(" Gx: "); Serial.print(myMessage.gyro_x_value); // Enviado dado do eixo X do giroscópio via Serial.
        Serial.print(" Gy: "); Serial.print(myMessage.gyro_y_value); // Enviado dado do eixo Y do giroscópio via Serial.
        Serial.print(" Gz: "); Serial.println(myMessage.gyro_z_value); // Enviado dado do eixo Z do giroscópio via Serial.
    }
}