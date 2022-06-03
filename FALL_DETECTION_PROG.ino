#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "Task.h"

#include "I2Cdev.h"
#include "Wire.h"
#include "MPU6050.h"
#include <CircularBuffer.h>

#define MPU                     0x68                                    // Endereço I2C para comunicação com a MPU
#define SERVICE_UUID            "7895d8d8-3220-11ec-8d3d-0242ac130003"  // UART service UUID
#define CHARACTERISTIC_UUID_TX  "7895dbee-3220-11ec-8d3d-0242ac130003"  // Characteristic UUID para transmissão dos dados

MPU6050 mpu(MPU);                                                       // Iniciar Comunicação com a MPU 6050
BLECharacteristic *pCharacteristic;                                     // Caracteristica do serviço BLE

float AccX, AccY, AccZ;   // Armazenar o valor da leitura de aceleração dos eixos X, Y e Z
float TotalAcceleration;  // Armazenar o valor da soma vetorial da aceleração dos eixos X, Y e Z
int cont, aux1;

float LSB_Sensitivity = 2048.00;        //Sensibilidade configurada para a escala da medida de aceleração
CircularBuffer<float, 100> circ_buffer; //Buffer cicular para armazenar até 100 amostras do tipo float

float getTotalAcceleration() {
  AccX = mpu.getAccelerationX() / LSB_Sensitivity; // Obtem aceleração do eixo X
  AccY = mpu.getAccelerationY() / LSB_Sensitivity; // Obtem aceleração do eixo Y
  AccZ = mpu.getAccelerationZ() / LSB_Sensitivity; // Obtem aceleração do eixo Z

  /*
    * Realiza o cálculo da soma vetorial dos 3 eixos de aceleração 
    * para obter o valor da aceleração total
  */
  return sqrt(pow(AccX, 2) + pow(AccY, 2) + pow(AccZ, 2));
}

class MyNotifyTask: public Task {
  void run(void *data) {
    uint8_t value = 0;
    while(1) {
      cont = 0;
      //Obter a aceleração total dos três eixos
      TotalAcceleration = getTotalAcceleration();
    
      if (TotalAcceleration < 0.4) {
        //Se a aceleração for menor que 0.4G
        for (int i = 0; i < 100; i++) {
          //Alimentar um buffer de 100 possições com os dados
          circ_buffer.push(TotalAcceleration);
          delay(10);
          TotalAcceleration = getTotalAcceleration();
        }

        for (int i = 0; i < 100; i++) {
          //Contar os picos de aceleração acima de 5G
          if (circ_buffer[i] > 5) {
            cont++;
            aux1 = i;
          }
        }

        if(cont > 3) {
          //VERIFICA MÉDIA APÓS ULTIMA LEITURA DE DESACELERAÇÃO
          float total_sum = 0;
          for(int i = aux1; i < 100; i++) {
            total_sum += circ_buffer[i];
          }
          float media = total_sum/(100 - aux1);
    
          //Média entre 1.5G e 0.9G = usuário estático
          if(media < 1.5 && media > .09) {
            //Queda detectada!!!
            value++;
            pCharacteristic->setValue(&value, 1);
            pCharacteristic->notify(); // Envia o valor para o aplicativo!
          }
        }
      }
      
      delay(10);
    } // While 1
  } // run
}; // MyNotifyTask

MyNotifyTask *pMyNotifyTask;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      pMyNotifyTask->start();
    };
 
    void onDisconnect(BLEServer* pServer) {
      pMyNotifyTask->stop();
      pServer->getAdvertising()->start();
    }
};

void setup()
{
  Wire.begin();

  /** Set sleep mode status.
  * @param enabled New sleep mode enabled status
  */
  mpu.setSleepEnabled(false);

  /** Set clock source setting.
  * An internal 8MHz oscillator, gyroscope based clock, or external sources can
  * be selected as the MPU-6050 clock source. When the internal 8 MHz oscillator
  * or an external source is chosen as the clock source, the MPU-6050 can operate
  * in low power modes with the gyroscopes disabled.
  *
  * CLK_SEL | Clock Source
  * --------+--------------------------------------
  * 0       | Internal oscillator
  * 1       | PLL with X Gyro reference
  * 2       | PLL with Y Gyro reference
  * 3       | PLL with Z Gyro reference
  * 4       | PLL with external 32.768kHz reference
  * 5       | PLL with external 19.2MHz reference
  * 6       | Reserved
  * 7       | Stops the clock and keeps the timing generator in reset
  *
  * @param source New clock source setting
  */
  mpu.setClockSource(0);

  /** Set temperature sensor enabled status.
  *
  * @param enabled New temperature sensor enabled status
  */
  mpu.setTempSensorEnabled(false);

  /** Set gyroscope standby enabled status.
  * @param New X-axis standby enabled status
  * @param New Y-axis standby enabled status
  * @param New Z-axis standby enabled status
  */
  mpu.setStandbyXGyroEnabled(true);
  mpu.setStandbyYGyroEnabled(true);
  mpu.setStandbyZGyroEnabled(true);

  /** Set full-scale accelerometer range.
  * The FS_SEL parameter allows setting the full-scale range of the accelerometer
  * sensors, as described in the table below.
  *
  * 0 = +/- 2g
  * 1 = +/- 4g
  * 2 = +/- 8g
  * 3 = +/- 16g
  *
  * @param range New full-scale accelerometer range setting
  */
  mpu.setFullScaleAccelRange(1);

  pMyNotifyTask = new MyNotifyTask();
  pMyNotifyTask->setStackSize(8000);

  // Criar o dispositivo bluetooth
  BLEDevice::init("ESP DEVICE"); // Nomear o dispositivo

  // Configura o dispositivo como Servidor BLE
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Cria o servico UART
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Cria uma Característica BLE para envio dos dados
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  pCharacteristic->addDescriptor(new BLE2902());

  // Inicia o serviço
  pService->start();

  // Inicia a descoberta do ESP32
  pServer->getAdvertising()->start();
}

void loop()
{}