#include <Arduino.h>

// --- CONSTANTES ---
#define LEFT 0
#define RIGHT 1 
const double TAREFA_TICKS = 6550.0; // Ticks para uma volta
const int VELOCIDADE_TESTE = 150;   // Velocidade PWM (0-255)

// ==========================================================
// --- DEFINIÇÕES DOS PINOS FINAIS ---
// ==========================================================

// --- PINOS DE CONTROLE (BTS7960) ---
// Motor 1 (LEFT/0) - USANDO PINOS 8/9
const int M1_PWM_R_PIN = 8;    
const int M1_PWM_L_PIN = 9;    
// Motor 2 (RIGHT/1) - USANDO PINOS 10/11
const int M2_PWM_R_PIN = 10;    
const int M2_PWM_L_PIN = 11;    

// --- PINOS DO ENCODER ---
const int ENCODER_L_A_PIN = 28; // Motor 1 (LEFT) Canal A
const int ENCODER_L_B_PIN = 29; // Motor 1 (LEFT) Canal B
const int ENCODER_R_A_PIN = 30; // Motor 2 (RIGHT) Canal A
const int ENCODER_R_B_PIN = 31; // Motor 2 (RIGHT) Canal B

// ==========================================================
// --- VARIÁVEIS GLOBAIS (Manutenção da Estrutura) ---
// ==========================================================

int encoderPPR = 2400; 
volatile bool firstRead[2] = {true, true};

// Variáveis de estado do encoder
volatile bool curAL, curBL, prevAL, prevBL; 
volatile bool curAR, curBR, prevAR, prevBR; 

// Contadores e Erros
volatile unsigned long pulseTime[2] = {0, 0};
volatile double encoderCount[2] = {0, 0};
volatile double encoderPulseCount[2] = {0, 0};
volatile double encoderPulseError[2] = {0, 0};
volatile bool encoderErro[2] = {0, 0};

// Outras variáveis
double curAngle[2] = {0, 0};
volatile unsigned long pulseTimeLatch[2] = {0, 0};
volatile unsigned long pulsePrevTime[2] = {0, 0};
volatile unsigned long pulsePrevPrevTime[2] = {0, 0};
volatile unsigned long pulseDeltaTime[2] = {0, 0};
volatile double encoderCountLatch[2] = {0, 0}; 
volatile double encoderPrevCount[2] = {0, 0};
volatile double encoderPrevPrevCount[2] = {0, 0};
volatile double encoderDeltaCount[2] = {0, 0};
volatile bool lastPulseForward[2] = {0, 0};
volatile bool lastPulseBackward[2] = {0, 0};
volatile double curRPM[2] = {0, 0};
volatile double prevRPM[2] = {0, 0};
volatile double highestRPM[2] = {0, 0};
volatile int filterGain[2] = {20, 20};
volatile double curRPM_Filtered[2] = {0, 0};
volatile int motor_gear = 60 ;
volatile int encoder_gear = 20; 
volatile int relation_motor_encoder = motor_gear/encoder_gear;


// ==========================================================
// --- CLASSE ENCODER (Lógica Consistente) ---
// ==========================================================

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
    double readAngle(int encoder_side);
    double readRPM(int encoder_side);

    static Encoder* obj_Encoder;
    
    static void interruptionChAL();
    static void interruptionChBL();
    static void interruptionChAR();
    static void interruptionChBR();
};

Encoder* Encoder::obj_Encoder = 0;

Encoder::Encoder(int pin_AL, int pin_BL, int pin_AR, int pin_BR){
    this-> DI_ENCODER_CH_AL = pin_AL;
    this-> DI_ENCODER_CH_BL = pin_BL;
    this-> DI_ENCODER_CH_AR = pin_AR;
    this-> DI_ENCODER_CH_BR = pin_BR;
    
    pinMode(DI_ENCODER_CH_AL, INPUT_PULLUP);
    pinMode(DI_ENCODER_CH_BL, INPUT_PULLUP);
    pinMode(DI_ENCODER_CH_AR, INPUT_PULLUP);
    pinMode(DI_ENCODER_CH_BR, INPUT_PULLUP);

    prevAL = digitalRead(DI_ENCODER_CH_AL);
    prevBL = digitalRead(DI_ENCODER_CH_BL);
    prevAR = digitalRead(DI_ENCODER_CH_AR);
    prevBR = digitalRead(DI_ENCODER_CH_BR);
}

void Encoder::setup()
{
    obj_Encoder = this;
    
    attachInterrupt(digitalPinToInterrupt(DI_ENCODER_CH_AL), Encoder::interruptionChAL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(DI_ENCODER_CH_BL), Encoder::interruptionChBL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(DI_ENCODER_CH_AR), Encoder::interruptionChAR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(DI_ENCODER_CH_BR), Encoder::interruptionChBR, CHANGE);
}

double Encoder::readPulses(int encoder_side){
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

double Encoder::readAngle(int encoder_side){ return 0; }
double Encoder::readRPM(int encoder_side) { return 0; } 

// --- IMPLEMENTAÇÃO DAS INTERRUPÇÕES ---

// Motor 1 (LEFT/0) - Pinos 28/29 (SINAL CORRIGIDO)
void Encoder::interruptionChAL()
{
    pulseTime[LEFT] = micros();
    bool curA = digitalRead(obj_Encoder-> DI_ENCODER_CH_AL);
    
    if(curA != prevAL) {
        bool curB = digitalRead(obj_Encoder->DI_ENCODER_CH_BL);
        // Lógica de Quadratura Robusta (SINAL INVERTIDO)
        if (curA != curB) { 
            encoderCount[LEFT]--; 
        } else {
            encoderCount[LEFT]++; 
        }
        prevAL = curA;
        encoderPulseCount[LEFT]++;
    }
}

void Encoder::interruptionChBL()
{
    pulseTime[LEFT] = micros();
    bool curB = digitalRead(obj_Encoder->DI_ENCODER_CH_BL);

    if(curB != prevBL) {
        bool curA = digitalRead(obj_Encoder->DI_ENCODER_CH_AL);
        // Lógica de Quadratura Robusta (SINAL INVERTIDO)
        if (curA == curB) {
            encoderCount[LEFT]--; 
        } else {
            encoderCount[LEFT]++; 
        }
        prevBL = curB;
        encoderPulseCount[LEFT]++;
    }
}


// Motor 2 (RIGHT/1) - Pinos 30/31 (SINAL MANTIDO)
void Encoder::interruptionChAR()
{
    pulseTime[RIGHT] = micros();
    bool curA = digitalRead(obj_Encoder-> DI_ENCODER_CH_AR);
    
    if(curA != prevAR) {
        bool curB = digitalRead(obj_Encoder->DI_ENCODER_CH_BR);
        // Lógica de Quadratura Robusta (Mantida)
        if (curA != curB) { 
            encoderCount[RIGHT]++; 
        } else {
            encoderCount[RIGHT]--; 
        }
        prevAR = curA;
        encoderPulseCount[RIGHT]++;
    }
}

void Encoder::interruptionChBR()
{
    pulseTime[RIGHT] = micros();
    bool curB = digitalRead(obj_Encoder->DI_ENCODER_CH_BR);

    if(curB != prevBR) {
        bool curA = digitalRead(obj_Encoder->DI_ENCODER_CH_AR);
        // Lógica de Quadratura Robusta (Mantida)
        if (curA == curB) {
            encoderCount[RIGHT]++; 
        } else {
            encoderCount[RIGHT]--; 
        }
        prevBR = curB;
        encoderPulseCount[RIGHT]++;
    }
}


// ==========================================================
// --- FUNÇÕES DE CONTROLE DE MOTOR (CORREÇÃO DE DIREÇÃO AQUI) ---
// ==========================================================

Encoder encoder_obj(
    ENCODER_L_A_PIN, ENCODER_L_B_PIN, 
    ENCODER_R_A_PIN, ENCODER_R_B_PIN  
);

/**
 * @brief Controla a velocidade de um motor específico.
 * * Agora, inverte o comando de PWM para que 'forward_cmd = true' 
 * resulte em ticks positivos para ambos os motores.
 */
void setMotorSpeed(int motor_side, int speed, bool forward_cmd) {
    
    // *** CORREÇÃO: INVERTER O COMANDO DE PWM ***
    bool reverse_cmd = !forward_cmd;
    
    int speed_fwd = reverse_cmd ? speed : 0; 
    int speed_rev = reverse_cmd ? 0 : speed; 
    
    if (motor_side == LEFT) {
        // Motor 1 (LEFT/0): Pinos 8/9
        analogWrite(M1_PWM_R_PIN, speed_rev); // Comando de FRENTE agora vai para o pino de RÉ e vice-versa
        analogWrite(M1_PWM_L_PIN, speed_fwd); 
    } else {
        // Motor 2 (RIGHT/1): Pinos 10/11
        analogWrite(M2_PWM_R_PIN, speed_rev); // Comando de FRENTE agora vai para o pino de RÉ e vice-versa
        analogWrite(M2_PWM_L_PIN, speed_fwd); 
    }
}

/**
 * @brief Para ambos os motores.
 */
void stopAllMotors() {
    setMotorSpeed(LEFT, 0, true);
    setMotorSpeed(RIGHT, 0, true);
}


// ==========================================================
// --- FUNÇÃO DE TESTE SIMULTÂNEO ---
// ==========================================================

void testarMotoresSimultaneo(double target_ticks, int speed, bool direction_forward) {
    
    Serial.println(direction_forward ? "\n*** TESTE SIMULTÂNEO: FRENTE ***" : "\n*** TESTE SIMULTÂNEO: TRÁS ***");
    Serial.print("Meta Ticks: "); Serial.println(target_ticks, 0);
    
    encoder_obj.reset();
    
    // Inicia os dois motores simultaneamente
    setMotorSpeed(LEFT, speed, direction_forward); // Motor 1
    setMotorSpeed(RIGHT, speed, direction_forward); // Motor 2
    
    double ticks_L, ticks_R;
    unsigned long startTime = millis();
    
    bool M_LEFT_Done = false; // Motor 1
    bool M_RIGHT_Done = false; // Motor 2

    // Loop principal: Roda até que ambos os motores atinjam a meta
    while (true) {
        ticks_L = encoder_obj.readPulses(LEFT); // Ticks M1
        ticks_R = encoder_obj.readPulses(RIGHT); // Ticks M2
        
        // Verifica se as metas foram atingidas
        if (abs(ticks_L) >= target_ticks) {
            if (!M_LEFT_Done) Serial.println("-> MOTOR 1 (LEFT) PARADO.");
            setMotorSpeed(LEFT, 0, true); 
            M_LEFT_Done = true;
        }

        if (abs(ticks_R) >= target_ticks) {
            if (!M_RIGHT_Done) Serial.println("-> MOTOR 2 (RIGHT) PARADO.");
            setMotorSpeed(RIGHT, 0, true);
            M_RIGHT_Done = true;
        }

        // Condição de parada: Ambos atingiram a meta
        if (M_LEFT_Done && M_RIGHT_Done) {
            break;
        }
        
        // Timeout de segurança (para o caso de um motor falhar)
        if (millis() - startTime > 15000) { 
            Serial.println("\nALERTA: Tempo limite de 15s atingido. Parando motores.");
            break;
        }
        
        // Impressão dos ticks
        Serial.print("M1 (Left): "); Serial.print(ticks_L, 0); 
        Serial.print(" | M2 (Right): "); Serial.println(ticks_R, 0);
        delay(50); 
    }
    
    stopAllMotors(); // Garante que ambos estão parados
    
    Serial.println("\n--- TESTE SIMULTÂNEO CONCLUÍDO ---");
    Serial.print("Finais M1 (Left): "); Serial.println(encoder_obj.readPulses(LEFT), 0); 
    Serial.print("Finais M2 (Right): "); Serial.println(encoder_obj.readPulses(RIGHT), 0); 
}


// ==========================================================
// --- CÓDIGO PRINCIPAL (setup e loop) ---
// ==========================================================

void setup() {
  Serial.begin(115200);
  Serial.println("--- INÍCIO DO TESTE FINAL (Sinais de Direção e Encoder Corrigidos) ---");
  
  // Configura todos os pinos PWM como OUTPUT
  pinMode(M1_PWM_R_PIN, OUTPUT);
  pinMode(M1_PWM_L_PIN, OUTPUT);
  pinMode(M2_PWM_R_PIN, OUTPUT);
  pinMode(M2_PWM_L_PIN, OUTPUT);

  encoder_obj.setup();
  stopAllMotors(); 
  
  delay(2000); 
}

void loop() {
  
  // Teste 1: Rotação para FRENTE (Ambos devem ter Ticks POSITIVOS)
  testarMotoresSimultaneo(
    TAREFA_TICKS, 
    VELOCIDADE_TESTE, 
    true 
  );

  delay(5000); 

  // Teste 2: Rotação para TRÁS (Ambos devem ter Ticks NEGATIVOS)
  testarMotoresSimultaneo(
    TAREFA_TICKS, 
    VELOCIDADE_TESTE, 
    false 
  );

  Serial.println("\n--- TESTES FINAIS CONCLUÍDOS. LOOP PARADO. ---");
  while(true); 
}