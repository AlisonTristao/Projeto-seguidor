#include <Arduino.h>
#include <QTRSensors.h>
#include <BluetoothSerial.h>

QTRSensors qtr;
BluetoothSerial SerialBT;

// multtasks
TaskHandle_t observer;

// constantes do PID
#define Kp 0.005
#define Ki 0.005
#define Kd 0.005

// pinos da ponte H
#define PWMA 21
#define PWMB 19

// velocidade do carrinho
#define speed 4095/2

// config da linha de sensores
const uint8_t qtdSensores = 8;
uint16_t sensorValues[qtdSensores];

// valores do tempo
unsigned long temp_anterior=0, temp_atual, t;

// variaveis para o PID
int P, I, D, erroPassado, erroSomado = 0;
float erro, PID;
bool volta1 = true;

void enviaDados(void * pvParameters){
  for(;;){ // loop infinito
    delay(150);
    SerialBT.println("{"+String(PID)+"}");
  }
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("Esp32 seguidor v1");

  // configuração da linha de sensores
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){13, 12, 14, 27, 26, 25, 33, 32}, qtdSensores);
  qtr.setEmitterPin(15);

  // usamos apenas para iniciar as variaveis da biblioteca
  qtr.calibrate();

  // regula o valor minimo e maximo do sensor
  for (uint8_t i = 0; i < qtdSensores; i++){
    qtr.calibrationOn.minimum[i] = 0;
  }
  for (uint8_t i = 0; i < qtdSensores; i++){
   qtr.calibrationOn.maximum[i] = 4095;
  }

  // config ponte H
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  ledcSetup(0, 5000, 12); // canal para esquerdo
  ledcSetup(1, 5000, 12); // canal para o direito
  ledcAttachPin(PWMA, 0);
  ledcAttachPin(PWMB, 1);
  // inicia como 0
  //ledcWrite(0, 0);
  //ledcWrite(1, 0);

  // inicia a task bluetooth
  xTaskCreatePinnedToCore(
    enviaDados,   //  Função da tarefa 
    "taskBT",     //  nome da tarefa 
    10000,        //  Tamanho (bytes) 
    NULL,         //  parâmetro da tarefa 
    2,            //  prioridade da tarefa (tem q ser 2 pra n dar conflito) 
    &observer,    //  observa a tarefa criada
    0);           //  nucleo
}

void calculaPID(){
  // salva o tempo passado pra calcular a derivada
  temp_atual = millis();
  // calcula o tempo passado 
  t = temp_atual - temp_anterior;

  // define os valores
  P = erro;
  I = (erro + erroSomado)*t;
  D = (erro - erroPassado)/t;

  // calcula o PID
  PID = (Kp * P) + (Ki * I) + (Kd * D);

  // anti-windup 
  // caso o PID passe o valor da velocidade max n soma o erro
  if(PID>speed){
    PID = speed;
  }else if(PID<-speed){
    PID = -speed;
  }else{
    erroSomado += erro;
  }

  // caso esteja na primeira volta ele usa metade da velocidade
  if(volta1){
    map(PID,-speed,speed,-speed/2,speed/2);
  }

  // salva o erro passado e tempo passado
  erroPassado = erro;
  temp_anterior = temp_atual;

  Serial.println(PID);
  delay(100);
}

void motors(){
  // controle de curva com PID
  if(PID == 0){
    ledcWrite(1, speed);
    ledcWrite(0, speed);
  }else if(PID > 0){
    ledcWrite(1, speed);
    ledcWrite(0, speed - PID);
  }else{
    ledcWrite(1, speed + PID);
    ledcWrite(0, speed);
  }
}

void loop() {
  // calcula a posição da linha 
  uint16_t position = qtr.readLineWhite(sensorValues);

  for (uint8_t i = 0; i < qtdSensores; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }

  // calcula o erro 
  erro = ((int)position-3500);
  Serial.print((int)position-3500);
  Serial.print("\t");

  // calcula o PID
  calculaPID();

  // controla a velocidade dos motores
  motors();
}