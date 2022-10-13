#include <Arduino.h>
#include <QTRSensors.h>
#include <BluetoothSerial.h>

QTRSensors qtr;
BluetoothSerial SerialBT;

// multtasks
TaskHandle_t observer;

// constantes do PID
float Kp = 0.005, Ki = 0.005, Kd = 0.005;

// pinos da ponte H
#define PWMA 21
#define PWMB 19

// velocidade do carrinho
float speed = 4095/2;

// config da linha de sensores
const uint8_t qtdSensores = 8;
uint16_t sensorValues[qtdSensores];

// valores do tempo
unsigned long temp_anterior=0, temp_atual, t;

// variaveis para o PID
int P, I, D, erroPassado, erroSomado = 0;
float erro, PID;

// variaveis para execoes
#define intercessao 4095
bool ligado;
int total;
int volta = 0;

String texto = "";
void recebeDados(){
  // recebe as chars e soma em um texto
  char a = SerialBT.read();
  texto += a;

  // separa as constantes quando recebe o texto todo
  if(a == '}'){
    // muda as constantes 
    Kp = (texto.substring(0, texto.indexOf('/'))).toFloat();
    Ki = (texto.substring(texto.indexOf('/')+1, texto.indexOf('%'))).toFloat();
    Kd = (texto.substring(texto.indexOf('%')+1, texto.indexOf('&'))).toFloat();
    speed = (texto.substring(texto.indexOf('&')+1, texto.indexOf('}'))).toFloat();

    // printa os valores
    Serial.print(Kp);     Serial.print("\t");  
    Serial.print(Ki);     Serial.print("\t"); 
    Serial.print(Kd);     Serial.print("\t"); 
    Serial.print(speed);  Serial.println("");  

    // limpa a variavel para o proximo loop
    texto = "";
  }
}

void enviaDados(void * pvParameters){
  for(;;){ // loop infinito
    // caso ele receba algum dado ele altera as constantes 
    if(SerialBT.available()){
      recebeDados();
    }else{
      // senao ele envia os dados 
      delay(150);
      // caso queirma mudar oq ele envia mude aqui
      SerialBT.println("{"+String(erro)+"}");
    }
  }
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("Esp32 seguidor v1");

  // configuração da linha de sensores
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){35,32, 33, 25, 26, 27, 14, 12}, qtdSensores);
  //qtr.setSensorPins((const uint8_t[]){13, 12, 14, 27, 26, 25, 33, 32}, qtdSensores);
  qtr.setEmitterPin(13);

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
  ledcAttachPin(PWMA, 1);
  ledcAttachPin(PWMB, 0);
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
  if(volta == 0){
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

  // nao sei se essa parte é melhor assim @alison
  int velEsq = speed, velDir = speed;

  if (volta == 0){
    if(PID > 0){
      velEsq = speed - PID;
    }else{
      velDir = speed + PID;
    }

    map(velEsq, -speed, speed, 1000, speed);
    map(velDir, -speed, speed, 1000, speed);

    ledcWrite(1, velEsq);
    ledcWrite(0, velDir);
  }
  /*else{
      ledcWrite(1, 0);
      ledcWrite(0, 0); 

  }*/
}

/*void Linha_de_chegada() {
    delay(50);
  if (volta == 0){
    ligado = false;
  }
  volta += 1;
}*/

void loop() {
  // calcula a posição da linha 
  uint16_t position = qtr.readLineWhite(sensorValues);

  // printa os valores lidos
  for (uint8_t i = 0; i < qtdSensores; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
    total += sensorValues[i];
  }
  if (total <= intercessao){ //caso haja intercessao
    delay(250);
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