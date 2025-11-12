#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <string.h>
#include <math.h> // Para PI

// --- CONSTANTES GLOBAIS E PINOS ---
#define LEFT 0
#define RIGHT 1

// Publicar Odometria a cada 50ms (20 Hz)
const int ODOM_PUB_INTERVAL = 50;

// PINOS DE CONTROLE (BTS7960)
const int M1_PWM_R_PIN = 8;    
const int M1_PWM_L_PIN = 9;    
const int M2_PWM_R_PIN = 10;    
const int M2_PWM_L_PIN = 11;    

// PINOS DO ENCODER
const int ENCODER_L_A_PIN = 28;
const int ENCODER_L_B_PIN = 29;
const int ENCODER_R_A_PIN = 30;
const int ENCODER_R_B_PIN = 31;

// ==========================================================
// --- VARIÁVEIS DE ESTADO E CALIBRAÇÃO ---
// ==========================================================

// Variáveis de Estado do Encoder
volatile long encoderCount[2] = {0, 0}; // Alterado para long para maior precisão
volatile long encoderPrevCount[2] = {0, 0};
volatile unsigned long lastTimeSpeed[2] = {0, 0};
volatile double currentTicksPerSecond[2] = {0.0, 0.0};
int pwm_L_last = 0;
int pwm_R_last = 0;
const int PWM_SLEW_STEP = 8; // máx variação por ciclo (~8/255)

// Variáveis de Estado do IMU
Adafruit_MPU6050 mpu;
volatile double currentTheta = 0.0; // Ângulo Theta/Yaw em RADIANOS
unsigned long lastIMUTime = 0;
double gyro_bias_z = 0.0;             // Desvio (bias) do giroscópio Z
bool imu_calibrated = false;          // Flag de calibração

// Constantes de Cinemática
const double MAX_RPM_SETPOINT = 40.0;
const double WHEEL_DIAMETER_M = 0.127;
const double BASE_WIDTH_M = 0.66; // distância entre rodas
const double WHEEL_RADIUS_M = WHEEL_DIAMETER_M / 2.0;
const double MAX_LINEAR_VEL_MS = (MAX_RPM_SETPOINT / 60.0) * 2.0 * PI * WHEEL_RADIUS_M; // ~0.266 m/s

// Variáveis de Comando (Velocidades Desejadas do ROS)
double target_v_linear = 0.0;
double target_omega_angular = 0.0;

// Variável para o tempo limite de segurança
unsigned long last_cmd_time = 0;
const long CMD_TIMEOUT_MS = 1200; // 1200 milissegundos é um bom valor padrão

//
// --- HELPER ---
//
int slew_limit(int target, int last, int step) {
    if (target > last + step) return last + step;
    if (target < last - step) return last - step;
    return target;
}

// ==========================================================
// --- CLASSE ENCODER (Lógica da Interrupção) ---
// ==========================================================

class Encoder {
public:
    int DI_ENCODER_CH_AL, DI_ENCODER_CH_BL, DI_ENCODER_CH_AR, DI_ENCODER_CH_BR;
    Encoder(int pin_AL, int pin_BL, int pin_AR, int pin_BR);
    void setup();
    long readPulses(int encoder_side);
    double readTicksPerSecond(int encoder_side);
    void reset();
    static Encoder* obj_Encoder;
    
    // Funções estáticas para interrupção (agora mais limpas)
    static void interruptionChAL(); static void interruptionChBL();
    static void interruptionChAR(); static void interruptionChBR();
};
Encoder* Encoder::obj_Encoder = 0;

// Implementações da Classe Encoder (Lógica padronizada)
Encoder::Encoder(int pin_AL, int pin_BL, int pin_AR, int pin_BR) {
    this-> DI_ENCODER_CH_AL = pin_AL; this-> DI_ENCODER_CH_BL = pin_BL;
    this-> DI_ENCODER_CH_AR = pin_AR; this-> DI_ENCODER_CH_BR = pin_BR;
    pinMode(DI_ENCODER_CH_AL, INPUT_PULLUP); pinMode(DI_ENCODER_CH_BL, INPUT_PULLUP);
    pinMode(DI_ENCODER_CH_AR, INPUT_PULLUP); pinMode(DI_ENCODER_CH_BR, INPUT_PULLUP);
}
void Encoder::setup() {
    obj_Encoder = this;
    attachInterrupt(digitalPinToInterrupt(DI_ENCODER_CH_AL), Encoder::interruptionChAL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(DI_ENCODER_CH_BL), Encoder::interruptionChBL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_R_A_PIN), Encoder::interruptionChAR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_R_B_PIN), Encoder::interruptionChBR, CHANGE);
}
long Encoder::readPulses(int encoder_side){
    noInterrupts(); long ticks = encoderCount[encoder_side]; interrupts(); return ticks;
}
double Encoder::readTicksPerSecond(int encoder_side) {
    noInterrupts(); double ticks_s = currentTicksPerSecond[encoder_side]; interrupts(); return ticks_s;
}
void Encoder::reset(){
  noInterrupts();
  encoderCount[LEFT] = 0; encoderCount[RIGHT] = 0;
  encoderPrevCount[LEFT] = 0; encoderPrevCount[RIGHT] = 0;
  lastTimeSpeed[LEFT] = millis(); lastTimeSpeed[RIGHT] = millis();
  interrupts();
}

// --- Funções de Interrupção (Lógica de Contagem Simples) ---
// NOTA: A inversão de sinal (incremento/decremento) agora deve ser tratada
// de forma simétrica ao que funcionou no código de teste (assumindo que o LEFT é invertido).

// Motor Esquerdo (LEFT) - Invertido (Para frente é '++')
void Encoder::interruptionChAL() {
    bool curA = digitalRead(obj_Encoder-> DI_ENCODER_CH_AL);
    bool curB = digitalRead(obj_Encoder->DI_ENCODER_CH_BL);
    if (curA != curB) { encoderCount[LEFT]--; } else { encoderCount[LEFT]++; }
}
void Encoder::interruptionChBL() {
    bool curB = digitalRead(obj_Encoder->DI_ENCODER_CH_BL);
    bool curA = digitalRead(obj_Encoder->DI_ENCODER_CH_AL);
    if (curA == curB) { encoderCount[LEFT]--; } else { encoderCount[LEFT]++; }
}

// Motor Direito (RIGHT) - Não Invertido (Para frente é '--')
void Encoder::interruptionChAR() {
    bool curA = digitalRead(obj_Encoder-> DI_ENCODER_CH_AR);
    bool curB = digitalRead(obj_Encoder->DI_ENCODER_CH_BR);
    if (curA != curB) { encoderCount[RIGHT]++; } else { encoderCount[RIGHT]--; }
}
void Encoder::interruptionChBR() {
    bool curB = digitalRead(obj_Encoder->DI_ENCODER_CH_BR);
    bool curA = digitalRead(obj_Encoder->DI_ENCODER_CH_AR);
    if (curA == curB) { encoderCount[RIGHT]++; } else { encoderCount[RIGHT]--; }
}


// ==========================================================
// --- INSTÂNCIAS E FUNÇÕES AUXILIARES DE BAIXO NÍVEL ---
// ==========================================================

Encoder encoder_obj(
    ENCODER_L_A_PIN, ENCODER_L_B_PIN,
    ENCODER_R_A_PIN, ENCODER_R_B_PIN  
);

/**
 * @brief Aplica o valor de PWM (-255 a 255).
 * CORRIGIDO: Usa a lógica de Avanço/Ré para o driver BTS7960.
 */
void setMotorPWM(int motor_side, int pwm_value) {
    
    // Limitador zero-zone para evitar vibração
    if (abs(pwm_value) < 10) pwm_value = 0;

    // Garantir que o valor está no range [-255, 255]
    if (pwm_value > 255) pwm_value = 255;
    if (pwm_value < -255) pwm_value = -255;
    
    // Determina a direção (True se a velocidade for positiva ou zero)
    bool forward_cmd = (pwm_value >= 0);
    int speed = abs(pwm_value);

    // Determina o comando de Ré (True se a velocidade for negativa)
    bool reverse_cmd = !forward_cmd;
    
    // Prepara os valores de PWM:
    // speed_fwd recebe 'speed' apenas se o comando for de Ré (inversão)
    int speed_fwd = reverse_cmd ? speed : 0;
    // speed_rev recebe 'speed' apenas se o comando for de Frente (inversão)
    int speed_rev = reverse_cmd ? 0 : speed;
    
    if (motor_side == LEFT) {
        // Motor Esquerdo
        // M1_PWM_R_PIN e M1_PWM_L_PIN
        // Aplicamos a lógica que funcionou: Pinos invertidos
        analogWrite(M1_PWM_R_PIN, speed_rev); // Recebe o comando de FRENTE (speed_rev)
        analogWrite(M1_PWM_L_PIN, speed_fwd); // Recebe o comando de RÉ (speed_fwd)
    } else {
        // Motor Direito
        // M2_PWM_R_PIN e M2_PWM_L_PIN
        // Aplicamos a lógica que funcionou: Pinos invertidos (se necessário)
        // OBS: Se o motor 2 estiver ligado de forma invertida, isso garante a simetria com o Motor 1
        analogWrite(M2_PWM_R_PIN, speed_rev); // Recebe o comando de FRENTE
        analogWrite(M2_PWM_L_PIN, speed_fwd); // Recebe o comando de RÉ
    }
}
void stopAllMotors() {
    setMotorPWM(LEFT, 0);
    setMotorPWM(RIGHT, 0);
}

/**
 * @brief Calcula a taxa de Ticks por Segundo (velocidade bruta) e armazena.
 */
void calculateTicksRate(int motor_side) {
    noInterrupts();
    long currentTicks = encoderCount[motor_side];
    unsigned long currentTime = millis();
    interrupts();

    long deltaTicks = currentTicks - encoderPrevCount[motor_side];
    unsigned long deltaTime = currentTime - lastTimeSpeed[motor_side];

    if (deltaTime > 0) {
        double ticks_per_second = (deltaTicks / (double)deltaTime) * 1000.0;
        
        noInterrupts();
        currentTicksPerSecond[motor_side] = ticks_per_second;
        encoderPrevCount[motor_side] = currentTicks;
        lastTimeSpeed[motor_side] = currentTime;
        interrupts();
    }
}

/**
 * @brief Calibra o giroscópio Z do MPU-6050
 * Mede o desvio (bias) enquanto o robô está PARADO
 */
void calibrateIMU() {
    Serial.println(F("[IMU] Iniciando calibração do giroscópio..."));
    const int samples = 500;
    double sum = 0.0;
    sensors_event_t a, g, temp;

    for (int i = 0; i < samples; i++) {
        mpu.getEvent(&a, &g, &temp);
        sum += g.gyro.z;  // leitura em rad/s
        delay(5);         // 500 * 5ms = 2.5s de amostragem
    }

    gyro_bias_z = sum / samples;
    imu_calibrated = true;
    Serial.print(F("[IMU] Calibração concluída. Bias Z = "));
    Serial.println(gyro_bias_z, 6);
}

/**
 * @brief Lê o giroscópio do MPU-6050 e integra o yaw (Z) em radianos.
 * Aplica compensação de bias e normalização de ângulo.
 */
void readIMUTheta() {
    if (!imu_calibrated) return;  // garante calibração antes de integrar

    unsigned long currentTime = micros();
    double dt = (currentTime - lastIMUTime) / 1e6;  // Δt em segundos
    lastIMUTime = currentTime;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Compensa o bias e integra o yaw
    double yaw_rate_rad_s = g.gyro.z - gyro_bias_z;
    currentTheta += yaw_rate_rad_s * dt;

    // Mantém o ângulo dentro de [-PI, PI]
    while (currentTheta > PI)  currentTheta -= 2.0 * PI;
    while (currentTheta < -PI) currentTheta += 2.0 * PI;
}


// ==========================================================
// --- FUNÇÕES DE ALTO NÍVEL (Cinemática e Protocolo) ---
// ==========================================================

/**
 * @brief Executa a Cinemática Inversa e converte a velocidade desejada (m/s) para PWM.
 * NOTA: A lógica de cinemática inversa deve considerar a largura da base (BASE_WIDTH)
 * para a variável omega. O seu código estava simplificado (assumindo BASE_WIDTH = 2).
 * Vamos manter a variável simples por enquanto.
 */
void calculateAndSetPWM(double v, double omega) {
    
    // 1. CINEMÁTICA INVERSA (Valores de Referência em m/s de roda)
    // Formula Simplificada (Base Largura = 2) - Use uma constante de largura de base real em projetos futuros!
    double v_ref_L = v - (omega * BASE_WIDTH_M / 2.0);
    double v_ref_R = v + (omega * BASE_WIDTH_M / 2.0);

    // 2. CONVERSÃO DE VELOCIDADE (m/s) PARA PWM (0 a 255)
    double max_v = MAX_LINEAR_VEL_MS;
    
    // Regra de três: PWM = (V_ref / V_max) * 255
    int pwm_L = (int)((v_ref_L / max_v) * 255.0);
    int pwm_R = (int)((v_ref_R / max_v) * 255.0);

    pwm_L = slew_limit(pwm_L, pwm_L_last, PWM_SLEW_STEP);
    pwm_R = slew_limit(pwm_R, pwm_R_last, PWM_SLEW_STEP);
    pwm_L_last = pwm_L;
    pwm_R_last = pwm_R;
    
    // 3. ATUAÇÃO NO MOTOR
    setMotorPWM(LEFT, pwm_L);
    setMotorPWM(RIGHT, pwm_R);
}

/**
 * @brief Lê o buffer Serial e atualiza as velocidades alvo (V, Omega).
 */
void readSerialCommand() {
    static char buffer[30];
    static int bufferIndex = 0;
    
    if (Serial.available()) {
        char inChar = Serial.read();

        if (inChar != '\n' && bufferIndex < sizeof(buffer) - 1) {
            buffer[bufferIndex++] = inChar;
            return;
        }
        
        buffer[bufferIndex] = '\0';
        bufferIndex = 0;

        // PROTOCOLO: V,<V_LINEAR>,<OMEGA_ANGULAR>
        if (buffer[0] == 'V') {
            char *token;
            double values[2];
            int i = 0;
            
            // Pula o 'V,' (2 caracteres)
            token = strtok(buffer + 2, ",");
            
            while (token != NULL && i < 2) {
                values[i++] = atof(token);
                token = strtok(NULL, ",");
            }
            
            if (i == 2) {
                target_v_linear = values[0];
                target_omega_angular = values[1];

                // Marca o tempo do último comando válido
                last_cmd_time = millis();
            }
        }
    }
}

/**
 * @brief Publica odometria com o ângulo atualizado pelo IMU.
 */
void publishOdometry(unsigned long currentTime) {
    // --- Variáveis locais "seguras" ---
    long ticksL_safe, ticksR_safe;
    unsigned long lastTimeL_safe, lastTimeR_safe;
    long prevTicksL_safe, prevTicksR_safe;
    double ticks_sL = 0.0, ticks_sR = 0.0;

    // --- Cópia atômica das variáveis voláteis ---
    noInterrupts();
    ticksL_safe = encoderCount[LEFT];
    ticksR_safe = encoderCount[RIGHT];
    lastTimeL_safe = lastTimeSpeed[LEFT];
    lastTimeR_safe = lastTimeSpeed[RIGHT];
    prevTicksL_safe = encoderPrevCount[LEFT];
    prevTicksR_safe = encoderPrevCount[RIGHT];
    lastTimeSpeed[LEFT] = currentTime;
    lastTimeSpeed[RIGHT] = currentTime;
    encoderPrevCount[LEFT] = ticksL_safe;
    encoderPrevCount[RIGHT] = ticksR_safe;
    interrupts();

    // --- Cálculo das velocidades ---
    long deltaTicksL = ticksL_safe - prevTicksL_safe;
    long deltaTicksR = ticksR_safe - prevTicksR_safe;
    unsigned long deltaTimeL = currentTime - lastTimeL_safe;
    unsigned long deltaTimeR = currentTime - lastTimeR_safe;
    if (deltaTimeL > 0) ticks_sL = (deltaTicksL / (double)deltaTimeL) * 1000.0;
    if (deltaTimeR > 0) ticks_sR = (deltaTicksR / (double)deltaTimeR) * 1000.0;
    
    if (abs(deltaTicksL) > 2000 || abs(deltaTicksR) > 2000) {
        encoder_obj.reset();
        return;
    }

    // --- Usa o ângulo atualizado pelo IMU ---
    double theta = 0;

    // --- Publicação via Serial ---
    char buffer[96];
    snprintf(buffer, sizeof(buffer), "O,%ld,%ld,%.1f,%.1f,%.5f",
            ticksL_safe, ticksR_safe,
            ticks_sL, ticks_sR,
            theta);
    Serial.println(buffer);
}    
    
// ==========================================================
// --- CÓDIGO PRINCIPAL (setup e loop) ---
// ==========================================================
void setup() {
    Serial.begin(115200);
    pinMode(M1_PWM_R_PIN, OUTPUT); pinMode(M1_PWM_L_PIN, OUTPUT);
    pinMode(M2_PWM_R_PIN, OUTPUT); pinMode(M2_PWM_L_PIN, OUTPUT);

    encoder_obj.setup();
    stopAllMotors();
    encoder_obj.reset();

    // --- Inicializa o MPU-6050 ---
    Wire.begin();
    if (!mpu.begin()) {
        Serial.println(F("[ERRO] MPU6050 não detectado!"));
    } else {
        mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
        delay(500);
        calibrateIMU();          // Realiza calibração inicial (robô deve estar parado)
        lastIMUTime = micros();  // Marca tempo inicial
    }
}
    
void loop() {
    static unsigned long last_odom_time = 0;
    unsigned long current_time_ms = millis();

    // 1. Receber comandos do ROS (V, ω)
    readSerialCommand();

    // 2. Verifica timeout de segurança
    if (current_time_ms - last_cmd_time > CMD_TIMEOUT_MS) {
        stopAllMotors();
    }

    // 3. Cinemática inversa e controle de motores
    calculateAndSetPWM(target_v_linear, target_omega_angular);

    // 4. Publicar odometria a cada intervalo
    if (current_time_ms - last_odom_time >= ODOM_PUB_INTERVAL) {
        readIMUTheta();                  // Atualiza yaw com IMU
        publishOdometry(current_time_ms);
        last_odom_time = current_time_ms;
    }
}
