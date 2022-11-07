#include <Arduino.h>
#include <QTRSensors.h>
#include <BluetoothSerial.h>

QTRSensors qtr;
BluetoothSerial SerialBT;

// multtasks
TaskHandle_t observer;

// constantes do PID
float kp = 117, ki = 0.2, kd = 0.1;

// pinos da ponte H
#define PWMA 21
#define PWMB 19

// velocidade do carrinho
bool largada = false;
float speed = 4095/2;
float v = 1;
int velEsq = speed, velDir = speed;
float cte = 2.7;

unsigned long int t_intercessao;

// config da linha de sensores
const uint8_t qtdSensores = 8;
uint16_t sensorValues[qtdSensores];

// valores do tempo
unsigned long temp_anterior=0, temp_atual, t;

// variaveis para o PID
int P, I, D, erroPassado, erroSomado = 0;
float erro, PID;

// variaveis para execoes
#define intercessao 3000
#define botao 18
#define sensoresquerdo 13
#define sensordireito 36
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
    kp = (texto.substring(1, texto.indexOf('/'))).toFloat();
    ki = (texto.substring(texto.indexOf('/')+1, texto.indexOf('%'))).toFloat();
    kd = (texto.substring(texto.indexOf('%')+1, texto.indexOf('&'))).toFloat();
    //cte = (texto.substring(texto.indexOf('&')+1, texto.indexOf('}'))).toFloat();
    speed = (texto.substring(texto.indexOf('&')+1, texto.indexOf('}'))).toFloat();

    // printa os valores
    Serial.print(kp);     Serial.print("\t");  
    Serial.print(ki);     Serial.print("\t"); 
    Serial.print(kd);     Serial.print("\t"); 
    Serial.print(speed);  Serial.print("\t");  
    //Serial.print(cte);  Serial.println("");

    // limpa a variavel para o proximo loop
    texto = "";
  }
}

void enviaDados(void * pvParameters){
  for(;;){ // loop perpétuo
    // caso ele receba algum dado ele altera as constantes 
    if(SerialBT.available()){
      recebeDados();
    }else{
      // senao ele envia os dados 
      delay(150);
      // caso queirma mudar oq ele envia mude aqui
      SerialBT.printf("{%f}", PID);
    }
  }
}

/*void contaVolta(){
  volta += 1;
  digitalWrite(2, !digitalRead(2));
}*/

void setup() {
  Serial.begin(115200);
  SerialBT.begin("Esp32 seguidor v1");

  //attachInterrupt(digitalPinToInterrupt(sensordireito), contaVolta, CHANGE);

  // liga o led por estilo
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);

  pinMode(sensordireito, INPUT);
  pinMode(sensoresquerdo, INPUT);
  // configuração da linha de sensores
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){39, 34, 35, 32, 33, 25, 26, 27}, qtdSensores);
  qtr.setEmitterPin(14);

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
  pinMode(botao, INPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  ledcSetup(0, 5000, 12); // canal para esquerdo
  ledcSetup(1, 5000, 12); // canal para o direito
  ledcAttachPin(PWMA, 1);
  ledcAttachPin(PWMB, 0);

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
  t = (temp_atual - temp_anterior)*1000;

  // define os valores
  P = erro * kp;
  I = (erroSomado)* ki;
  D = ((erro - erroPassado)) * kd;

  // calcula o PID
  PID = (P) + (I) + (D);

  // anti-windup 
  // caso o PID passe o valor da velocidade max n soma o erro
  if(PID>speed){
    PID = speed;
  }else if(PID<-speed){
    PID = -speed;
  }

  if((I>=speed)&&(erro >=0)){ 
    I = speed;
  }else if((I<=-speed)&&(erro <=0)){
    I = -speed;
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

  /*Serial.print("P: ");
  Serial.print(P);
  Serial.print("\t");
  Serial.print("I: ");
  Serial.print(I);
  Serial.print("\t");
  Serial.print("D: ");
  Serial.print(D);
  Serial.print("\t");
  Serial.print("PID: ");
  Serial.print(PID);
  Serial.print(" ");*/
}

void motors(){
  // controle de curva com PID
  if(PID > 0){
    velEsq = speed - PID;
    velDir = speed; //- PID*PID/(cte*speed);
  }else{
    velDir = speed + PID;
    velEsq = speed; // - PID*PID/(cte*speed);
  }

  ledcWrite(1, velEsq);
  ledcWrite(0, velDir);

  /*Serial.print("motor esq: ");
  Serial.print(velEsq);
  Serial.print("\t motor dir:");
  Serial.print(velDir);
  Serial.print("\t");*/

}

void Linha_de_chegada() {
  if(largada == false){
    //Serial.println("Tá DESLIGADO");
    while(digitalRead(botao) == 1){
      speed = 0;
      ledcWrite(1,speed);
      ledcWrite(0,speed);
    }
    largada = true;
  }
  if(largada == true){
    speed = 4095/2;
    //Serial.println("LIGADO!!!");
    //Serial.println(digitalRead(sensordireito));
    if((digitalRead(sensordireito)==0) and (millis() > (t_intercessao+2000))){
      //Serial.printf("detectou %d", volta);
      while(digitalRead(sensordireito) == 0){}
      volta += 1;
      digitalWrite(2, !digitalRead(2));
    }
    if(volta == 2){
      delay(50);
      volta = 0;
      largada = false;
    }
  }
}

void loop() {
  // calcula a posição da linha 
  uint16_t position = qtr.readLineWhite(sensorValues);

  // soma os valores dos sensores
  total = 0;
  for (uint8_t i = 0; i < qtdSensores; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
    total += sensorValues[i];
  }
  //Serial.print(total);
  if (!(total >= intercessao)){ //caso haja intercessao
    t_intercessao = millis();
    //Serial.println("INTERCESSAO");
  }
  Linha_de_chegada();

  //calcula o erro 
  //Serial.print("Erro: ");
  erro = (((int)position-3500)/100);
  //Serial.print(erro);
  Serial.println();

  // calcula o PID
  calculaPID();

  // controla a velocidade dos motores
  motors();
}