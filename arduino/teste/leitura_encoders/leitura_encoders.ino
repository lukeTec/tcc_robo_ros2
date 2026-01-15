#include <Arduino.h>

// --- CLASSES E VARIÁVEIS DA BIBLIOTECA ENCODER ---

// Constantes internas da biblioteca
#define LEFT 0
#define RIGHT 1 

// --- DEFINIÇÕES DOS PINOS DO ENCODER (MOTOR 2) ---
// Pinos 30 e 31 são pinos de interrupção válidos em muitas placas Arduino (e.g., Mega/Due).
const int ENCODER_L_A_PIN = 10;   // Placeholder (Não usado)
const int ENCODER_L_B_PIN = 11;   // Placeholder (Não usado)
const int ENCODER_R_A_PIN = 30;  // Canal A - PINO 30 (Motor 2)
const int ENCODER_R_B_PIN = 31;  // Canal B - PINO 31 (Motor 2)

// --- Variáveis globais (simplificadas e corrigidas) ---
// Variáveis essenciais para a contagem bidirecional
volatile double encoderCount[2] = {0, 0};
volatile unsigned long pulseTime[2] = {0, 0};
volatile double encoderPulseCount[2] = {0, 0};

// Variáveis de estado do encoder (necessárias para o algoritmo de quadratura)
volatile bool curAR, curBR, prevAR, prevBR;
volatile bool curAL, curBL, prevAL, prevBL; 

// Variáveis da sua biblioteca original (mantidas para compatibilidade)
int encoderPPR = 2400; 
volatile bool firstRead[2] = {true, true};
// ... (Outras variáveis da sua biblioteca que não são mostradas, mas estariam aqui) ...


// Declaração da Classe Encoder
class Encoder {
public:
    int DI_ENCODER_CH_AL;
    int DI_ENCODER_CH_BL;
    int DI_ENCODER_CH_AR;
    int DI_ENCODER_CH_BR;

    Encoder(int pin_AL, int pin_BL, int pin_AR, int pin_BR);
    void setup();
    double readPulses(int encoder_side);
    void reset();

    static Encoder* obj_Encoder;
    
    // Declarações das funções estáticas para interrupção (Corrigidas: 'IRAM_ATTR' removido)
    static void interruptionChAL();
    static void interruptionChBL();
    static void interruptionChAR();
    static void interruptionChBR();
};

// Inicialização da variável estática
Encoder* Encoder::obj_Encoder = 0;

// Definições das Funções da Classe
Encoder::Encoder(int pin_AL, int pin_BL, int pin_AR, int pin_BR){
    this-> DI_ENCODER_CH_AL = pin_AL;
    this-> DI_ENCODER_CH_BL = pin_BL;
    this-> DI_ENCODER_CH_AR = pin_AR;
    this-> DI_ENCODER_CH_BR = pin_BR;
    
    // Configuração dos pinos
    pinMode(DI_ENCODER_CH_AL, INPUT_PULLUP);
    pinMode(DI_ENCODER_CH_BL, INPUT_PULLUP);
    pinMode(DI_ENCODER_CH_AR, INPUT_PULLUP);
    pinMode(DI_ENCODER_CH_BR, INPUT_PULLUP);

    // Inicializa o estado 'anterior' do encoder RIGHT
    prevAR = digitalRead(DI_ENCODER_CH_AR);
    prevBR = digitalRead(DI_ENCODER_CH_BR);
}

void Encoder::setup()
{
    obj_Encoder = this;
    
    // Anexamos as interrupções para o Motor 2 (Pinos 30 e 31)
    attachInterrupt(digitalPinToInterrupt(DI_ENCODER_CH_AR), Encoder::interruptionChAR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(DI_ENCODER_CH_BR), Encoder::interruptionChBR, CHANGE);
}

// Funções de Leitura
double Encoder::readPulses(int encoder_side){
    // É crucial proteger a leitura da variável 'volatile double' com noInterrupts()
    noInterrupts();
    double ticks = encoderCount[encoder_side];
    interrupts();
    return ticks;
}

void Encoder::reset()
{
  noInterrupts();
  encoderCount[LEFT] = 0;
  encoderCount[RIGHT] = 0;
  interrupts();
}

// --- IMPLEMENTAÇÃO DAS INTERRUPÇÕES (ISRs) ---

// Interrupções Dummy para o lado Esquerdo (Não usadas no teste)
void Encoder::interruptionChAL() { }
void Encoder::interruptionChBL() { }


// Interrupção do Canal A (Pino 30) - Lógica de contagem
void Encoder::interruptionChAR()
{
    // Lemos o estado atual do Canal A
    bool curA = digitalRead(obj_Encoder-> DI_ENCODER_CH_AR);
    
    if(curA != prevAR) 
    {
        // Lemos o Canal B para determinar a direção
        bool curB = digitalRead(obj_Encoder->DI_ENCODER_CH_BR);
        
        // Lógica de Contagem: Se A e B estão em estados diferentes -> Frente, senão -> Trás.
        // Essa lógica pode precisar de inversão dependendo da fiação física.
        if (curA != curB) { 
            encoderCount[RIGHT]++; 
        } else {
            encoderCount[RIGHT]--; 
        }
        
        // Atualiza os estados para o próximo ciclo
        prevAR = curA;
        // Não atualizamos prevBR aqui se estamos usando o B para interrupção também (a interrupção B fará isso)
        encoderPulseCount[RIGHT]++;
    }
}

// Interrupção do Canal B (Pino 31) - Lógica de contagem
void Encoder::interruptionChBR()
{
    // Lemos o estado atual do Canal B
    bool curB = digitalRead(obj_Encoder-> DI_ENCODER_CH_BR);

    if(curB != prevBR)
    {
        // Lemos o Canal A para determinar a direção
        bool curA = digitalRead(obj_Encoder->DI_ENCODER_CH_AR);

        // Lógica de Contagem para o Canal B
        // Para a contagem bidirecional correta (quadratura), a lógica deve ser invertida
        // em relação ao Canal A para evitar contagem dupla na mesma direção ou erros.
        if (curA == curB) {
            encoderCount[RIGHT]++; // Assume uma direção
        } else {
            encoderCount[RIGHT]--; // Assume a direção oposta
        }
        
        // Atualiza os estados
        prevBR = curB;
        // Não atualizamos prevAR aqui se estamos usando o A para interrupção também
        encoderPulseCount[RIGHT]++;
    }
}


// --- CÓDIGO PRINCIPAL (setup e loop) ---

// Cria a instância da classe Encoder, passando os pinos
Encoder encoder_obj(
    ENCODER_L_A_PIN, ENCODER_L_B_PIN, 
    ENCODER_R_A_PIN, ENCODER_R_B_PIN  
);


void setup() {
  Serial.begin(115200);
  Serial.println("--- Iniciando Teste de Contagem Bidirecional (Classe Encoder) ---");
  
  // Chama a função de setup da classe Encoder.
  encoder_obj.setup();
  encoder_obj.reset(); 
}

void loop() {
  // Lê a contagem de pulsos para o lado RIGHT (índice 1)
  double ticks_atuais = encoder_obj.readPulses(RIGHT);

  Serial.print("Contador (Pinos 30/31): ");
  Serial.println(ticks_atuais, 0); 
  
  delay(200);
}