// validacao_andar_1metro.ino
// Versão final corrigida (interruptionChBR, contadores long)
// TICKS_POR_VOLTA configurados para 6770L conforme suas medições.

// ================= CONFIGURAÇÃO =================
// TEST_ONE_REV = true -> testa 1 volta por roda (usar com roda levantada)
// TEST_ONE_REV = false -> executa teste de 1 metro com PWM fixo
const bool TEST_ONE_REV = false;

// --- CONSTANTES DE ENCODER/MOTOR ---
#define LEFT 0
#define RIGHT 1 

// 🔧 TICKS POR VOLTA (ajuste se fizer nova calibração)
const long TICKS_POR_VOLTA_L = 6770L;  // Motor esquerdo (medido)
const long TICKS_POR_VOLTA_R = 6770L;  // Motor direito (medido)

const double WHEEL_DIAMETER_M = 0.127; // Diâmetro da roda em metros

// --- PINOS DE CONTROLE (BTS7960) ---
const int M1_PWM_R_PIN = 8;    
const int M1_PWM_L_PIN = 9;    
const int M2_PWM_R_PIN = 10;    
const int M2_PWM_L_PIN = 11;    

// --- PINOS DO ENCODER ---
const int ENCODER_L_A_PIN = 28; 
const int ENCODER_L_B_PIN = 29; 
const int ENCODER_R_A_PIN = 30; 
const int ENCODER_R_B_PIN = 31; 

// --- PWM FIXO PARA TESTE ---
const int PWM_TESTE = 120; // ajuste se necessário (80..160 recomendado)

#include <Arduino.h>

// ==========================================================
// --- VARIÁVEIS GLOBAIS E CLASSE ENCODER ---
// ==========================================================
volatile long encoderCount[2] = {0, 0};
volatile bool prevAL, prevBL, prevAR, prevBR; 
volatile long encoderPrevCount[2] = {0, 0};
volatile unsigned long lastTimeRPM[2] = {0, 0};
volatile double currentRPM[2] = {0, 0};

class Encoder {
public:
    int DI_ENCODER_CH_AL, DI_ENCODER_CH_BL, DI_ENCODER_CH_AR, DI_ENCODER_CH_BR;
    Encoder(int pin_AL, int pin_BL, int pin_AR, int pin_BR);
    void setup();
    long readPulses(int encoder_side);
    void reset();
    double readRPM(int encoder_side); 
    static Encoder* obj_Encoder;
    static void interruptionChAL(); static void interruptionChBL();
    static void interruptionChAR(); static void interruptionChBR();
};

Encoder* Encoder::obj_Encoder = 0;

Encoder::Encoder(int pin_AL, int pin_BL, int pin_AR, int pin_BR) {
    this->DI_ENCODER_CH_AL = pin_AL; this->DI_ENCODER_CH_BL = pin_BL;
    this->DI_ENCODER_CH_AR = pin_AR; this->DI_ENCODER_CH_BR = pin_BR;
    pinMode(DI_ENCODER_CH_AL, INPUT_PULLUP); pinMode(DI_ENCODER_CH_BL, INPUT_PULLUP);
    pinMode(DI_ENCODER_CH_AR, INPUT_PULLUP); pinMode(DI_ENCODER_CH_BR, INPUT_PULLUP);
    prevAL = digitalRead(DI_ENCODER_CH_AL); prevBL = digitalRead(DI_ENCODER_CH_BL);
    prevAR = digitalRead(DI_ENCODER_CH_AR); prevBR = digitalRead(DI_ENCODER_CH_BR);
}

void Encoder::setup() {
    obj_Encoder = this;
    attachInterrupt(digitalPinToInterrupt(DI_ENCODER_CH_AL), Encoder::interruptionChAL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(DI_ENCODER_CH_BL), Encoder::interruptionChBL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(DI_ENCODER_CH_AR), Encoder::interruptionChAR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(DI_ENCODER_CH_BR), Encoder::interruptionChBR, CHANGE);
}

long Encoder::readPulses(int encoder_side){
    noInterrupts(); long ticks = encoderCount[encoder_side]; interrupts(); return ticks;
}

void Encoder::reset(){
  noInterrupts(); encoderCount[LEFT] = 0; encoderCount[RIGHT] = 0; interrupts();
}

double Encoder::readRPM(int encoder_side) {
    noInterrupts();
    double rpm = currentRPM[encoder_side];
    interrupts();
    return rpm;
}

void Encoder::interruptionChAL() { 
    bool curA = digitalRead(obj_Encoder->DI_ENCODER_CH_AL);
    if(curA != prevAL) {
        bool curB = digitalRead(obj_Encoder->DI_ENCODER_CH_BL);
        if (curA != curB) { encoderCount[LEFT]--; } else { encoderCount[LEFT]++; }
        prevAL = curA;
    }
}
void Encoder::interruptionChBL() {
    bool curB = digitalRead(obj_Encoder->DI_ENCODER_CH_BL);
    if(curB != prevBL) {
        bool curA = digitalRead(obj_Encoder->DI_ENCODER_CH_AL);
        if (curA == curB) { encoderCount[LEFT]--; } else { encoderCount[LEFT]++; }
        prevBL = curB;
    }
}
void Encoder::interruptionChAR() { 
    bool curA = digitalRead(obj_Encoder->DI_ENCODER_CH_AR);
    if(curA != prevAR) {
        bool curB = digitalRead(obj_Encoder->DI_ENCODER_CH_BR);
        if (curA != curB) { encoderCount[RIGHT]++; } else { encoderCount[RIGHT]--; }
        prevAR = curA;
    }
}
void Encoder::interruptionChBR() {
    // CORREÇÃO: ler DI_ENCODER_CH_BR corretamente
    bool curB = digitalRead(obj_Encoder->DI_ENCODER_CH_BR);
    if(curB != prevBR) {
        bool curA = digitalRead(obj_Encoder->DI_ENCODER_CH_AR);
        if (curA == curB) { encoderCount[RIGHT]++; } else { encoderCount[RIGHT]--; }
        prevBR = curB;
    }
}

// ==========================================================
Encoder encoder_obj(
    ENCODER_L_A_PIN, ENCODER_L_B_PIN, 
    ENCODER_R_A_PIN, ENCODER_R_B_PIN  
);

void setMotorPWM(int motor_side, int pwm_value) {
    if (pwm_value > 255) pwm_value = 255;
    if (pwm_value < -255) pwm_value = -255;
    
    bool forward_cmd = (pwm_value >= 0);
    int speed = abs(pwm_value);
    bool reverse_cmd = !forward_cmd;
    int speed_fwd = reverse_cmd ? speed : 0; 
    int speed_rev = reverse_cmd ? 0 : speed; 
    
    if (motor_side == LEFT) {
        analogWrite(M1_PWM_R_PIN, speed_rev); 
        analogWrite(M1_PWM_L_PIN, speed_fwd); 
    } else {
        analogWrite(M2_PWM_R_PIN, speed_rev); 
        analogWrite(M2_PWM_L_PIN, speed_fwd); 
    }
}

void stopAllMotors() {
    setMotorPWM(LEFT, 0);
    setMotorPWM(RIGHT, 0);
}

void calculateRPM(int motor_side) {
    noInterrupts();
    long currentTicks = encoderCount[motor_side];
    unsigned long currentTime = millis();
    interrupts();

    long deltaTicks = currentTicks - encoderPrevCount[motor_side];
    unsigned long deltaTime = currentTime - lastTimeRPM[motor_side];

    if (deltaTime > 0) {
        long ticks_per_rev = (motor_side == LEFT) ? TICKS_POR_VOLTA_L : TICKS_POR_VOLTA_R;
        double rpm = (abs(deltaTicks) / (double)ticks_per_rev) / ((double)deltaTime / 60000.0);
        
        noInterrupts();
        currentRPM[motor_side] = rpm;
        encoderPrevCount[motor_side] = currentTicks;
        lastTimeRPM[motor_side] = currentTime;
        interrupts();
    }
}

// ==========================================================
// --- ROTINA: TESTE 1 VOLTA (ROBÔ LEVANTADO) ---
void testarUmaVolta(int motor_side, int pwm) {
    long target = (motor_side == LEFT) ? TICKS_POR_VOLTA_L : TICKS_POR_VOLTA_R;
    Serial.print("\n=== TESTE 1 VOLTA (motor ");
    Serial.print(motor_side==LEFT ? "LEFT" : "RIGHT");
    Serial.println(") ===");
    Serial.print("Target ticks configurado: "); Serial.println(target);
    encoder_obj.reset();
    delay(50);
    setMotorPWM(motor_side, pwm);
    unsigned long lastPrint = 0;
    while (true) {
        long ticks = encoder_obj.readPulses(motor_side);
        if (millis() - lastPrint > 200) {
            double dist = (ticks / (double)target) * PI * WHEEL_DIAMETER_M;
            Serial.print("ticks="); Serial.print(ticks);
            Serial.print("  dist(m)="); Serial.println(dist, 4);
            lastPrint = millis();
        }
        if (abs(ticks) >= target) break;
    }
    stopAllMotors();
    long final_ticks = encoder_obj.readPulses(motor_side);
    double final_dist = (final_ticks / (double)target) * PI * WHEEL_DIAMETER_M;
    Serial.print("Resultado FINAL motor ");
    Serial.print(motor_side==LEFT ? "LEFT" : "RIGHT");
    Serial.print(" -> ticks finais = "); Serial.print(final_ticks);
    Serial.print(" , dist(m) = "); Serial.println(final_dist, 6);
    Serial.println("=== FIM TESTE 1 VOLTA ===\n");
    delay(400);
}

// ==========================================================
// --- ROTINA: ANDAR 1 METRO COM PWM FIXO ---
void moverDistanciaControlada() {
    const double DISTANCIA_ALVO_M = 1.0;
    const double TAREFA_TICKS_L = (TICKS_POR_VOLTA_L / (PI * WHEEL_DIAMETER_M)) * DISTANCIA_ALVO_M;
    const double TAREFA_TICKS_R = (TICKS_POR_VOLTA_R / (PI * WHEEL_DIAMETER_M)) * DISTANCIA_ALVO_M;
    
    Serial.println("\n╔════════════════════════════════════════════════════════╗");
    Serial.println("║          TESTE: ANDAR 1 METRO COM PWM FIXO            ║");
    Serial.println("╚════════════════════════════════════════════════════════╝");
    Serial.print("Meta ticks L: "); Serial.println((long)TAREFA_TICKS_L);
    Serial.print("Meta ticks R: "); Serial.println((long)TAREFA_TICKS_R);
    Serial.print("PWM fixo: "); Serial.println(PWM_TESTE);
    Serial.println("\nIniciando em 3s...");
    delay(3000);
    
    encoder_obj.reset();
    encoderPrevCount[LEFT] = 0; encoderPrevCount[RIGHT] = 0;
    lastTimeRPM[LEFT] = millis(); lastTimeRPM[RIGHT] = millis();
    
    setMotorPWM(LEFT, PWM_TESTE);
    setMotorPWM(RIGHT, PWM_TESTE);
    
    long ticks_L = 0, ticks_R = 0;
    unsigned long lastPrintTime = 0;
    unsigned long startTime = millis();

    while (true) {
        if (millis() - lastTimeRPM[LEFT] >= 100) calculateRPM(LEFT);
        if (millis() - lastTimeRPM[RIGHT] >= 100) calculateRPM(RIGHT);

        ticks_L = encoder_obj.readPulses(LEFT); 
        ticks_R = encoder_obj.readPulses(RIGHT); 

        bool motor_L_done = abs(ticks_L) >= (long)TAREFA_TICKS_L;
        bool motor_R_done = abs(ticks_R) >= (long)TAREFA_TICKS_R;

        if (motor_L_done && motor_R_done) {
            Serial.println("\n✅ AMBOS OS MOTORES ATINGIRAM A META!");
            break;
        }

        if (motor_L_done) setMotorPWM(LEFT, 0);
        if (motor_R_done) setMotorPWM(RIGHT, 0);
        
        if (millis() - lastPrintTime > 200) {
             Serial.print("t(s): "); Serial.print((millis()-startTime)/1000.0, 2);
             Serial.print(" | L ticks: "); Serial.print(ticks_L);
             Serial.print(" | R ticks: "); Serial.println(ticks_R);
             double dist_L_m = (ticks_L / (double)TICKS_POR_VOLTA_L) * PI * WHEEL_DIAMETER_M;
             double dist_R_m = (ticks_R / (double)TICKS_POR_VOLTA_R) * PI * WHEEL_DIAMETER_M;
             Serial.print(" dist_L(m): "); Serial.print(dist_L_m,4);
             Serial.print(" dist_R(m): "); Serial.println(dist_R_m,4);
             lastPrintTime = millis();
        }
    }
    
    stopAllMotors(); 
    
    unsigned long tempo_total = millis() - startTime;
    double dist_L = encoder_obj.readPulses(LEFT) / (double)TICKS_POR_VOLTA_L * PI * WHEEL_DIAMETER_M;
    double dist_R = encoder_obj.readPulses(RIGHT) / (double)TICKS_POR_VOLTA_R * PI * WHEEL_DIAMETER_M;
    
    Serial.print("\nTempo total: "); Serial.print(tempo_total/1000.0,2); Serial.println(" s");
    Serial.print("Dist L: "); Serial.print(dist_L,4); Serial.print(" m  Dist R: "); Serial.print(dist_R,4); Serial.println(" m");
    Serial.println("Fim do teste 1m.");
}

// ==========================================================
void setup() {
  Serial.begin(115200);
  pinMode(M1_PWM_R_PIN, OUTPUT); pinMode(M1_PWM_L_PIN, OUTPUT);
  pinMode(M2_PWM_R_PIN, OUTPUT); pinMode(M2_PWM_L_PIN, OUTPUT);

  encoder_obj.setup();
  stopAllMotors(); 
  encoder_obj.reset();

  Serial.println("\n=== validacao_andar_1metro (final) ===");
  delay(200);
}

void loop() {
  if (TEST_ONE_REV) {
      testarUmaVolta(LEFT, PWM_TESTE);
      delay(500);
      testarUmaVolta(RIGHT, PWM_TESTE);
      Serial.println("Teste 1 volta finalizado. Atualize TICKS_POR_VOLTA_* com os 'ticks finais' se desejar.");
      while(true);
  } else {
      moverDistanciaControlada();
      Serial.println("Reset para repetir.");
      while(true);
  }
}