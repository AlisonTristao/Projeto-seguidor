#include <Arduino.h>
#include <QTRSensors.h>
#include <BluetoothSerial.h>

QTRSensors qtr;
BluetoothSerial SerialBT;

// multtasks
TaskHandle_t observer;

// constantes do PID
float kp = 170, ki = 0.065, kd = 470;
//float kp1 = 270, ki1 = 0.6, kd1 = 430;

// pinos da ponte H
#define PWMA 19
#define PWMB 21
#define esq1 15
#define esq2 4
#define dir1 5
#define dir2 3

// velocidade do carrinho
bool largada = false;
int32_t speed = 3500;
int32_t oldSpeed = speed;

// velocidade dos motores esquerdo e direito
int32_t velEsq = speed, velDir = speed;
float cte = 0.7; // const para diminuir q roda gira para tras

unsigned long int t_intercessao;

// config da linha de sensores
const uint8_t qtdSensores = 8;
uint16_t sensorValues[qtdSensores];

// valores do tempo
unsigned long temp_anterior=0, temp_atual, t;

// variaveis para o PID
int P, I, D, erroPassado, erroSomado = 0, erro, PID;

// variaveis para execoes
#define intercessao 4000
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
    //kp1 = (texto.substring(texto.indexOf('A')+1, texto.indexOf('/'))).toFloat();

    ki = (texto.substring(texto.indexOf('/')+1, texto.indexOf('%'))).toFloat();
    //ki1 = (texto.substring(texto.indexOf('B')+1, texto.indexOf('%'))).toFloat();

    kd = (texto.substring(texto.indexOf('%')+1, texto.indexOf('&'))).toFloat();
    //kd1 = (texto.substring(texto.indexOf('C')+1, texto.indexOf('&'))).toFloat();

    cte = (texto.substring(texto.indexOf('&')+1, texto.indexOf('*'))).toFloat();
    speed = (texto.substring(texto.indexOf('*')+1, texto.indexOf('}'))).toFloat();

    // printa os valores
    Serial.print(kp);     Serial.print("\t");
    //Serial.print(kp1);     Serial.print("\t"); 

    Serial.print(ki);     Serial.print("\t");
    //Serial.print(ki1);     Serial.print("\t"); 

    Serial.print(kd);     Serial.print("\t"); 
    //Serial.print(kd1);     Serial.print("\t"); 

    Serial.print(cte);     Serial.print("\t"); 
    Serial.print(speed);  Serial.print("\t");  
    oldSpeed = speed;
    //Serial.print(cte);  Serial.println("");

    // limpa a variavel para o proximo loop
    //Serial.println(texto);
    texto = "";
  }
}

void enviaDados(void * pvParameters){
  for(;;){ // loop perpétuo
    // caso ele receba algum dado ele altera as constantes 
    if(SerialBT.available()){
      recebeDados();
    }else{
      // senao ele envia os dados para montar o grafico
      delay(150);
      // caso queirm mudar oq ele envia mude aqui
      SerialBT.printf("{%d}", PID);
    }
  } 
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("Esp32 seguidor v1");

  pinMode(sensordireito, INPUT);
  pinMode(sensoresquerdo, INPUT);
  pinMode(botao, INPUT);
  pinMode(esq1, OUTPUT);
  pinMode(esq2, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(2, OUTPUT);
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
  //temp_atual = millis();
  // calcula o tempo passado 
  //t = (temp_atual - temp_anterior)*1000;

  // define os valores
  P = erro * kp;
  I = (erroSomado)* ki;
  D = ((erro - erroPassado)) * kd;

  if((I>=(speed/2))&&(erro >=0)){ 
    I = (speed/2);
  }else if((I<=-(speed/2))&&(erro <=0)){
    I = -(speed/2);
  }else{
    erroSomado += erro;
  }

  // calcula o PID
  PID = (P) + (I) + (D);

  // anti-windup 
  // caso o PID passe o valor da velocidade max n soma o erro
  // e nao satura a integral
  if(PID>speed*2){
    PID = speed*2;
  }else if(PID<(-speed*2)){
    PID = -speed*2;
  }

  // salva o erro passado e tempo passado
  erroPassado = erro;
  //temp_anterior = temp_atual;

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
    velDir = speed - PID;
    velEsq = speed;
  }else{
    velEsq = speed + PID;
    velDir = speed;
  }

  if(velDir < 0){
    // esquerda
    digitalWrite(esq1, HIGH);
    digitalWrite(esq2, LOW);

    // direita
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, HIGH);

    velDir *= cte;
  }else if(velEsq < 0){
    // esquerda
    digitalWrite(esq1, LOW);
    digitalWrite(esq2, HIGH);

    // direita
    digitalWrite(dir1, HIGH);
    digitalWrite(dir2, LOW);

    velEsq *= cte;
  }else{
    // esquerda
    digitalWrite(esq1, LOW);
    digitalWrite(esq2, HIGH);

    // direita
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, HIGH);
  }

  ledcWrite(1, map(abs(velEsq), 0, speed, 400, speed));
  ledcWrite(0, map(abs(velDir), 0, speed, 400, speed));

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
      //speed = 0;
      ledcWrite(1, 0);
      ledcWrite(0, 0);
    }
    delay(1000);
    largada = true;
  }
  if(largada == true){
    //Serial.println("LIGADO!!!");
    //Serial.println(digitalRead(sensordireito));
    if((digitalRead(sensordireito)==0) and (millis() > (t_intercessao+700))){
      //Serial.printf("detectou %d", volta);
      while(digitalRead(sensordireito) == 0){}
      volta++;

    }

    // conta o final da pista e salva o tempo para parar
    if(volta == 2){
      t_intercessao = millis();
      volta++;
    }

    // espera um tempo para parar no final da pista (usa millis para nao terminar torto)
    if((millis() >= t_intercessao + 100) and (volta == 3)){
      volta = 0;
      largada = false;
      digitalWrite(2, HIGH);
    }
  }
}


void loop() {
  // calcula a posição da linha 
  uint16_t position = qtr.readLineWhite(sensorValues);

  // soma os valores dos sensores
  total = 0;
  for (uint8_t i = 0; i < qtdSensores; i++) {
    //Serial.print(sensorValues[i]);
    //Serial.print('\t');
    total += sensorValues[i];
  }
  //Serial.print(total);
  if (!(total >= intercessao)){ //caso haja intercessao 
    t_intercessao = millis(); 
    //Serial.println("INTERCESSAO");
    ///digitalWrite(2, HIGH); // acende o led
  }else{
    //digitalWrite(2, LOW); // apaga o led
  }

  // verifica as sinalizações da pista
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