#include <Servo.h>
//=============== Motores ===============
#define motorDireito1 12;
#define motorDireito2 13;
#define motorEsquerdo1 11;
#define motorEsquerdo2 10;

//=============== Sensores ===============
const int SH0 = 0;  //Sensor 0
const int SH1 = 1;  //Sensor 1
const int SH2 = 2;  //Sensor 2
const int SH3 = 3;  //Sensor 3

const int THRESHOLD = 150;
int sensorMaisDireito = 0;
int sensorMaisEsquerdo = 0;

//=============== Auxiliares ===============
int mode = 0;
#define STOPPED 0
#define FOLLOWING_LINE 1
#define NO_LINE 2

# define CONT_LINE 3
# define POS_LINE 4
# define RIGHT_TURN 5
# define LEFT_TURN 6

#define direita 1
#define esquerda -1

int LFSensor[4] = {0, 0, 0, 0};

//PID controller
float Kp = 50;
float Ki = 0;
float Kd = 0;

float error = 0, P = 0, I = 0, D = 0, PIDvalue = 0;
float erroAnterior = 0, previousI = 0;

const int ligar = 250;
const int valorInicalMotor = 250;
const int adj = 0;
float adjTurn = 8;
int extraInch = 200;
int adjGoAndTurn = 800;

Servo servoDireito;
Servo servoEsquerdo;

//Variaveis para otimização do caminho
/* Legenda para o caminho do carrinho
  'E' para esquerda
  'D' para direita
  'R' para reto
  'V' para voltar
*/
unsigned char dir; //direção atual do carrinho
char caminho[100] = "";  //armazena o caminho percorrido
unsigned char tamCaminho = 0; //tamanho do caminho
int indexCaminho = 0;  //posição do caminho
unsigned int status = 0; //Resolvendo = 0 e Chegar ao fim = 1

/*==========================================================================*/
void setup() {

  Serial.begin(9600);

  //Sensores de linha
  pinMode(SH0, INPUT);
  pinMode(SH1, INPUT);
  pinMode(SH2, INPUT);
  pinMode(SH3, INPUT);

  //Servos
  servoEsquerdo.attach(10);
  servoDireito.attach(13);

  mode = STOPPED;
  status = 0;
}

void loop() {

  Serial.println("Primeiro Passo - Começando");
  //Ler os sensores de linha
  lerSensores();

  //Percorre pelo labirinto para mapear o caminho a primeira vez
  resolveCaminho();
  Serial.println("Primeiro Passo - Completo");

  delay(10000);

  Serial.println("Segundo Passo - Começando");
  //Percorre pelo labirinto de forma mais rapida
  indexCaminho = 0;
  status = 0;
  resolveCaminhoOtimizado();
  Serial.println("Segundo Passo - Completo");

  delay(10000);

  mode = STOPPED;
  status = 0; // 1st pass
  indexCaminho = 0;
  tamCaminho = 0;
}

void lerSensores() {
  /*
    Valores de erro do vetor
    0 0 0 1   3
    0 0 1 1   2
    0 0 1 0   1
    0 1 1 0   0
    0 1 0 0  -1
    1 1 0 0  -2
    1 0 0 0  -3

    0 0 0 0  Não encontrou linha: vira 180
    1 1 1 1  Encontrou linha continua: Parar
  */

  LFSensor[0] = digitalRead(SH0);
  LFSensor[1] = digitalRead(SH1);
  LFSensor[2] = digitalRead(SH2);
  LFSensor[3] = digitalRead(SH3);

  sensorMaisDireito = LFSensor[3];
  sensorMaisEsquerdo = LFSensor[0];

  Serial.println("Sensor 1");
  Serial.println(LFSensor[0]);
  Serial.println("Sensor 2");
  Serial.println(LFSensor[1]);
  Serial.println("Sensor 3");
  Serial.println(LFSensor[2]);
  Serial.println("Sensor 4");
  Serial.println(LFSensor[3]);


  if ((LFSensor[0] == 0 ) && (LFSensor[1] == 0 ) && (LFSensor[2] == 0 ) && (LFSensor[3] == 1))
  {
    mode = FOLLOWING_LINE;
    error = 3;
  }
  else if ((LFSensor[0] == 0 ) && (sensorMaisEsquerdo < THRESHOLD)) {
    mode = RIGHT_TURN;
    error = 0;
  }
  else if ((LFSensor[3] == 0 ) && (sensorMaisDireito < THRESHOLD)) {
    mode = LEFT_TURN;
    error = 0;
  }
  else if ((LFSensor[0] == 0 ) && (LFSensor[1] == 0 ) && (LFSensor[2] == 1 ) && (LFSensor[3] == 1 ))
  {
    mode = FOLLOWING_LINE;
    error = 2;
  }

  else if ((LFSensor[0] == 0 ) && (LFSensor[1] == 0 ) && (LFSensor[2] == 1 ) && (LFSensor[3] == 0 ))
  {
    mode = FOLLOWING_LINE;
    error = 1;
  }

  else if ((LFSensor[0] == 0 ) && (LFSensor[1] == 1 ) && (LFSensor[2] == 1 ) && (LFSensor[3] == 0 ))
  {
    mode = FOLLOWING_LINE;
    error = 0;
  }

  else if ((LFSensor[0] == 0 ) && (LFSensor[1] == 1 ) && (LFSensor[2] == 0 ) && (LFSensor[3] == 0 ))
  {
    mode = FOLLOWING_LINE;
    error = -1;
  }

  else if ((LFSensor[0] == 1 ) && (LFSensor[1] == 1 ) && (LFSensor[2] == 0 ) && (LFSensor[3] == 0 ))
  {
    mode = FOLLOWING_LINE;
    error = -2;
  }

  else if ((LFSensor[0] == 1 ) && (LFSensor[1] == 0 ) && (LFSensor[2] == 0 ) && (LFSensor[3] == 0 ))
  {
    mode = FOLLOWING_LINE;
    error = -3;
  }

  else if ((LFSensor[0] == 1 ) && (LFSensor[1] == 1 ) && (LFSensor[2] == 1 ) && (LFSensor[3] == 1 ))
  {
    mode = CONT_LINE;
    error = 0;
  }

  else if ((LFSensor[0] == 0 ) && (LFSensor[1] == 0 ) && (LFSensor[2] == 0 ) && (LFSensor[3] == 0 ))
  {
    mode = NO_LINE;
    error = 0;
  }

}

void resolveCaminho() {
  while (!status) {
    lerSensores();
    //Verifica o estado do robo
    switch (mode)
    {
      case NO_LINE:  //Sem linha
        pararMotor();
        caminharEVirar(esquerda, 180);
        inserirCaminho('V');
        break;

      case CONT_LINE:  //Linha continua
        runExtraInch();
        lerSensores();
        if (mode != CONT_LINE) {
          caminharEVirar(esquerda, 90);
          inserirCaminho('E'); //Em caso de linha em " T " vire para esquerda
        }
        else {
          finalCaminho();
        }
        break;

      case RIGHT_TURN:  //Girar para direita
        runExtraInch();
        lerSensores();
        if (mode == NO_LINE) {
          caminharEVirar(direita, 90);
          inserirCaminho('D');
        }
        else {
          inserirCaminho('R'); //Continua reto
        }
        break;

      case LEFT_TURN: //Girar para esquerda
        caminharEVirar(esquerda, 90);
        inserirCaminho('E');
        break;

      case FOLLOWING_LINE:
        seguirLinha();
        break;
    }
  }
}

void pararMotor() {
  servoEsquerdo.writeMicroseconds(1500);
  servoDireito.writeMicroseconds(1500);
  delay(200);
}

void girarMotor(int direcao, int degrees) {
  servoEsquerdo.writeMicroseconds(1500 - valorInicalMotor * direcao);
  servoDireito.writeMicroseconds(1500 - valorInicalMotor * direcao);
  delay (round(adjTurn * degrees + 1));
  pararMotor();
}

void caminharEVirar(int direcao, int degrees) {
  controleMotor();
  delay(adjGoAndTurn);
  girarMotor(direcao, degrees );
}

void calcularPID() {
  P = error;
  I = I + error;
  D = error - erroAnterior;
  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  erroAnterior = error;
}

void runExtraInch(void) {
  controleMotor();
  delay(extraInch);
  pararMotor();
}

void controleMotor() {
  int velocidadeMotorEsq = 1500 - valorInicalMotor - PIDvalue;
  int velocidadeMotorDir = 1500 + valorInicalMotor * adj - PIDvalue;

  //Controla o limite da velocidade dos servos
  constrain(velocidadeMotorEsq, 1000, 2000);
  constrain(velocidadeMotorDir, 1000, 2000);

  servoEsquerdo.writeMicroseconds(velocidadeMotorEsq);
  servoDireito.writeMicroseconds(velocidadeMotorDir);
}

void inserirCaminho(char ch) {
  caminho[tamCaminho] = ch; //Salva o caminho atual na variavel
  tamCaminho++; //Incrementa a posição
  simplificarCaminho(); //Simplifica o caminho para melhor desempenho
}

void simplificarCaminho() {
  //apenas simplifica o caminho se o penúltimo for um 'V'
  if (tamCaminho < 3 || caminho[tamCaminho - 2] != 'V') {
    return;
  }

  int anguloTotal = 0;
  for (int i = 1; i <= 3; i++) {
    switch (caminho[tamCaminho - i]) {
      case 'D':
        anguloTotal += 90;
        break;

      case 'E':
        anguloTotal += 270;
        break;

      case 'V':
        anguloTotal += 180;
        break;
    }
  }

  //Obtem o ângulo como um número entre 0 e 360 ​​graus.
  anguloTotal = anguloTotal % 360;

  switch (anguloTotal) {
    case 0:
      caminho[tamCaminho - 3] = 'R'; //Reto
      break;

    case 90:
      caminho[tamCaminho - 3] = 'D'; //Direita
      break;

    case 180:
      caminho[tamCaminho - 3] = 'V'; //Volta
      break;

    case 270:
      caminho[tamCaminho - 3] = 'E'; //Esquerda
      break;
  }

  //O caminho agora é dois passos mais curto.
  tamCaminho -= 2;
}

void finalCaminho() {
  pararMotor();
  Serial.println(" Caminho ");
  for (int i = 0; i < tamCaminho; i++) {
    Serial.println(caminho[i]);
  }
  Serial.println("");
  Serial.print("Tamanho do Caminho: ");
  Serial.print(tamCaminho);

  status = 1;
  mode = STOPPED;
}

void seguirLinha() {
  calcularPID();
  controleMotor();
}

void resolveCaminhoOtimizado() {
  while (!status) {
    lerSensores();

    switch (mode) {
      case FOLLOWING_LINE:
        seguirLinha();
        break;

      case CONT_LINE:
        if (indexCaminho >= tamCaminho) {
          finalCaminho();
        }
        else {
          virarCaminho(caminho[indexCaminho]);
          indexCaminho++;
        }
        break;

      case LEFT_TURN:
        if (indexCaminho >= tamCaminho) {
          finalCaminho();
        }
        else {
          virarCaminho(caminho[indexCaminho]);
          indexCaminho++;
        }
        break;

      case RIGHT_TURN:
        if (indexCaminho >= tamCaminho) {
          finalCaminho();
        }
        else {
          virarCaminho(caminho[indexCaminho]);
          indexCaminho++;
        }
        break;
    }
  }
}

void virarCaminho(char dir) {
  switch (dir) {
    case 'E':    //Vira para esquerda
      caminharEVirar (esquerda, 90);
      break;

    case 'D': //Vira para direita
      caminharEVirar (direita, 90);
      break;

    case 'V': //Volta
      caminharEVirar (direita, 800);
      break;

    case 'R': //Continua reto
      runExtraInch();
      break;
  }
}
