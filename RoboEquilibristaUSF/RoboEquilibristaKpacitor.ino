#include <Wire.h>
#include <Kalman.h>

uint8_t i2c_data[14];
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;

uint32_t timer;

Kalman KalmanX;
Kalman KalmanY;
Kalman KalmanZ;

double KalAngleX;
double KalAngleY;
double KalAngleZ;

double gyroXangle;
double gyroYangle;



/************************************************************************
 * Setup Inicial com as Configurações para i2C.
 ************************************************************************/

void setup() {

  /* Inicializando a Serial para exibir mensagens de Debug */
  Serial.begin(115200);

  /* Inicializando o Barramento i2c para comunicação com a MPU6050 */
  Wire.begin();

#if ARDUINO >= 157
  Wire.setClock(400000UL);  // Freq = 400kHz.
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2;  // Freq = 400kHz
#endif

  i2c_data[0] = 7;    /* 0x19 - Taxa de amostragem  8kHz/(7 + 1) = 1000Hz */
  i2c_data[1] = 0x00; /* 0x1A - Desabilitar FSYNC, Configurar o Filtro de ACC 260Hz, Configurar Filtro de Gyro 256Hz, Amostragem de 8Khz */
  i2c_data[2] = 0x00; /* 0x1B - Configurar o fundo de escala do Gyro ±250deg/s - Faixa */
  i2c_data[3] = 0x00; /* 0x1C - Configurar o fundo de escala do Acelerômetro para ±2g - Faixa */

  /* Configirações do i2c*/
  while (i2cWrite(0x19, i2c_data, 4, false))
    ;

  /* PLL tenha como referência o gyro de eixo X, Desabilitando Sleep Mode */
  while (i2cWrite(0x6B, 0x01, true))
    ;

  // Aguarda estabilização
delay(150);

// tenta ler WHO_AM_I com retries mas aceita 0x68/0x69/0x70
uint8_t who = 0;
bool ok = false;
const int MAX_TRIES = 6;

for (int t = 0; t < MAX_TRIES; ++t) {
  uint8_t rc = i2cRead(0x75, &who, 1);
  if (rc == 0) {
    Serial.print(F("WHO_AM_I lido = 0x"));
    if (who < 16) Serial.print("0");
    Serial.println(who, HEX);
    if (who == 0x68 || who == 0x69 || who == 0x70) {
      ok = true;
      break;
    } else {
      Serial.println(F("WHO_AM_I diferente do esperado (aceitando como debug)."));
      // opcional: ainda set ok = true se quiser forçar sempre
    }
  } else {
    Serial.print(F("Leitura WHO_AM_I falhou (rcode="));
    Serial.print(rc);
    Serial.println(F(")"));
  }
  delay(150);
}

if (!ok) {
  Serial.println(F("AVISO: MPU não detectada após retries. Continuando sem sensor."));
  // marque uma flag para pular leituras do MPU no loop, se preferir
} else {
  Serial.println(F("MPU detectada (aceita 0x68/0x69/0x70). Continuando inicializacao."));
  // executar configurações do sensor (SMPLRT_DIV, PWR_MGMT_1 etc.)
  uint8_t cfg[4] = {7, 0x00, 0x00, 0x00};
  i2cWrite(0x19, cfg, 4, true);
  i2cWrite(0x6B, 0x01, true);
  delay(100);
}

  /* Tempo de estabilização do Sensor MPU6050 */
  delay(100);

  /* 1 - Leitura dos dados de Acc XYZ */
  while (i2cRead(0x3B, i2c_data, 14))
    ;

  /* 2 - Organizar os dados de Acc XYZ */
  accX = (int16_t)((i2c_data[0] << 8) | i2c_data[1]);  // ([ MSB ] [ LSB ])
  accY = (int16_t)((i2c_data[2] << 8) | i2c_data[3]);  // ([ MSB ] [ LSB ])
  accZ = (int16_t)((i2c_data[4] << 8) | i2c_data[5]);  // ([ MSB ] [ LSB ])

  /* 3 - Calculo de Pitch e Roll */
  double pitch = atan(accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;

  /* 4 - Inicialização do Filtro de Kalman XY */
  KalmanX.setAngle(roll);
  KalmanY.setAngle(pitch);

  gyroXangle = roll;
  gyroYangle = pitch;

  timer = micros();

  //Serial.print("Fim Setup\n");

  init_motores();
}

void loop() {

  /* Leitura dos Dados de Aceleração e Gyro do sensor MPU6050 */
  while (i2cRead(0x3B, i2c_data, 14))
    ;

  /*Aceleração*/
  accX = (int16_t)((i2c_data[0] << 8) | i2c_data[1]);  // ([ MSB ] [ LSB ])
  accY = (int16_t)((i2c_data[2] << 8) | i2c_data[3]);  // ([ MSB ] [ LSB ])
  accZ = (int16_t)((i2c_data[4] << 8) | i2c_data[5]);  // ([ MSB ] [ LSB ])

  /*Giroscópio*/
  gyroX = (int16_t)((i2c_data[8] << 8) | i2c_data[9]);    // ([ MSB ] [ LSB ])
  gyroY = (int16_t)((i2c_data[10] << 8) | i2c_data[11]);  // ([ MSB ] [ LSB ])
  gyroZ = (int16_t)((i2c_data[12] << 8) | i2c_data[13]);  // ([ MSB ] [ LSB ])


  // Serial.print("AccXYZ"); Serial.print("\t");
  //Serial.print(accX); Serial.print("\n");
 
  // Serial.print(accY); Serial.print("\t");
  // Serial.print(accZ); Serial.print("\n");

  // Serial.print("GiroXYZ"); Serial.print("\t");
  // Serial.print(gyroX); Serial.print("\t");
  // Serial.print(gyroY); Serial.print("\t");
  // Serial.print(gyroZ); Serial.print("\n");



  /******************* Filtro de Kalman *************************/

  /* Calculo do Delta Time */
  double dt = (double)(micros() - timer) / 1000000;
  timer = micros();

  double pitch = atan(accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;

  /* Convertendo de Rad/Segundo para Graus/Segundo Calculo da Taxa angular baseado no Giroscópio */
  gyroXangle = gyroX / 131.0;  //deg/s
  gyroYangle = gyroY / 131.0;  //deg/s

  /* Estimativa de Ângulo nos Eixos X e Y usando Filtro de Kalman */
  KalAngleX = KalmanX.getAngle(roll, gyroXangle, dt);
  KalAngleY = KalmanY.getAngle(pitch, gyroYangle, dt);

  /* Mensagens de Debug para verificação dos resultados obtidos com Filtro de Kalman e Calculos dos Angulos com os Acelerômetros */
  // Serial.print(KalAngleX); Serial.print("\n"); //Angulo estimado com o filtro de Kalman
 Serial.print(KalAngleY);
 Serial.print("\n");  //Angulo estimado com o filtro de Kalman
 // Serial.print(pitch);
  //Serial.print("\t");  //Angulo Calculado com os dados de aceleração da MPU6050
  //Serial.print(roll); Serial.print("\n"); //Angulo Calculado com os dados de aceleração da MPU6050

  double res = Compute(KalAngleY);
  PMWControleMotores(res);
}
