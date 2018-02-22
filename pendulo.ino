//*********************************************************************/1 /
// Código Arduino em C++ para controle de um pêndulo invertido //
// Autor: Rafael Faria Felix //
// Faculdade de Talentos Humanos – Uberaba, MG – Engenharia Mecânica //
//*********************************************************************//

//entradas
#define POT A0 //leitura do potenciômetro
#define BTN 52 //leitura do botão
#define ENC 53 //leitura do encoder linear
#define FCE 22 //leitura do fim de curso da esquerda
#define FCD 23 //leitura do fim de curso da direita

//saídas
#define MD1 11 //gira o motor no sentido 1
#define MD2 12 //gira o motor no sentido 2
#define MSP 13 //define a velocidade de giro do motor (PWM)
#define LUZ LED_BUILTIN //LED indicador de estado

//ajuste
#define PID_P 1
#define PID_I 0
#define PID_D 0

//limites
#define VCAL 255 //velocidade de calibração
#define VMIN 140 //velocidade mínima
#define VMAX 255 //velocidade máxima
#define TPOS 2 //tolerância do contador ao encontrar o fim do trilho
#define TANG 1 //tolerância do potenciômetro antes de tentar equilibrar a barra
#define TLD 500 //intervalo de tempo em que o LED pisca

#include <PID_v1.h>

double in, out, sp, input, output, setpoint; //variáveis do PID
char dir = 'p'; //direção do movimento: p-parado; e-esquerda; d-direita
volatile int pos = 0; //posição do carro
volatile int fim_trilho = 0; //tamanho do trilho (em pulsos do encoder)
PID pendulo(&in,&out,&sp,PID_P,PID_I,PID_D,DIRECT);

void setup() {
	Serial.begin(115200);

	pinMode(BTN, INPUT);
	pinMode(ENC, INPUT);
	pinMode(FCE, INPUT);
	pinMode(FCD, INPUT);
	pinMode(MD1, OUTPUT);
	pinMode(MD2, OUTPUT);
	pinMode(MSP, OUTPUT);
	pinMode(LUZ, OUTPUT);

	sp = 0;
	pendulo.SetSampleTime(20);
	pendulo.SetMode(AUTOMATIC);
	pendulo.SetOutputLimits(-255,255);

	attachInterrupt(digitalPinToInterrupt(ENC), encoder, FALLING);
	centraliza();
}

void encoder() {
	if(dir == 'e') {
		pos -= 1; //diminui o contador caso o movimento seja à esquerda
	} else {
		pos += 1; //aumenta o contador caso o movimento seja à direita
	}
	if(fim_trilho != 0 && (pos < -TPOS || pos > (fim_trilho + TPOS))) {
		//recalibrar caso haja divergência nas informações de posição
		fim_trilho = 0;
		centraliza();
	}
}

void mover(int velocidade) {
	if(dir == 'e') { //mover à esquerda
		digitalWrite(MD1,LOW);
		digitalWrite(MD2,HIGH);
	} else if(dir == 'd') { //mover à direita
		digitalWrite(MD1,HIGH);
		digitalWrite(MD2,LOW);
	} else { //parar
		digitalWrite(MD1,LOW);
		digitalWrite(MD2,LOW);
	}
	analogWrite(MSP,velocidade);
}

void centraliza() {

	//limpa interruptores dos sensores de fim de curso caso haja algum
	detachInterrupt(digitalPinToInterrupt(FCE));
	detachInterrupt(digitalPinToInterrupt(FCD));

	if(fim_trilho == 0) { //caso não tenha sido calculado o fim do trilho ainda

		//se não estiver à esquerda, mover à esquerda
		int fce = digitalRead(FCE);
		if(fce == 0) {
			dir = 'e'; //direção: esquerda
			mover(VCAL);
			while(fce == 0) {
				fce = digitalRead(FCE);
			}
		}

		//mover à direita e contar os pulsos do encoder
		pos = 0;
		dir = 'd';
		mover(VCAL);
		int fcd = digitalRead(FCD);
		while(fcd == 0) {
			fcd = digitalRead(FCD);
		}

		fim_trilho = pos; //determinar o final do trilho
	}

	//se à esquerda do trilho, mover à direita e vice-versa
	if(pos <= int(fim_trilho / 2)) {
		dir = 'd';
	} else {
		dir = 'e';
	}
	
	mover(VCAL); //mover até o meio do trilho
	while(pos != int(fim_trilho / 2)); //espera

	//aguarda até que o botão seja pressionado e pisca o led
	bool aguarda = true;
	int tempoAnterior = millis();
	int botao = digitalRead(BTN);
	while(aguarda) {
		delay(20); //debounce
		if(digitalRead(BTN) && botao) {
			aguarda = false;
		} else {
			botao = digitalRead(BTN);
		}
		if(millis() – tempoAnterior >= TLD) {
			digitalWrite(LUZ, !digitalRead(LUZ));
			tempoAnterior = millis();
		}
	}

	setpoint = analogRead(POT);

	//volta ao centro caso chegue ao final do trilho
	attachInterrupt(digitalPinToInterrupt(FCE), centraliza, RISING);	attachInterrupt(digitalPinToInterrupt(FCD), centraliza, RISING);
}

void loop() {

	input = analogRead(POT);
	int val = setpoint - input;
	in = val;
	pendulo.Compute();
	if(abs(val) > TANG) {
		if(out > 0) {
			dir = 'e'; //move à esquerda
		} else {
			dir = 'd'; //move à direita
		}
		output=abs(out) + VMIN;
		if(output > VMAX) {
			output = VMAX;
		}
		mover(output);
	} else {
		dir = 'p'; //parar
		mover(0);
	}

int tempo = millis();
Serial.print(tempo); Serial.print(";"); //tempo decorrido, em milissegundos
	Serial.print(setpoint); Serial.print(";"); //setpoint do PID
	Serial.print(in); Serial.print(";"); //entrada do PID
	Serial.print(out); Serial.print(";"); //saída do PID
	Serial.print(input); Serial.print(";"); //leitura do sensor
	Serial.print(output); Serial.print(";"); //saída para o motor
}