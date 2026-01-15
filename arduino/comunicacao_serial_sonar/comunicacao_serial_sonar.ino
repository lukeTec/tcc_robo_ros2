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
// Publicar Ultrassom a cada 100ms (10 Hz) - Sensores são mais lentos
const int SONAR_PUB_INTERVAL = 100;

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

// --- NOVOS PINOS ULTRASSÔNICOS ---
// Frontal
const int US_FRONT_TRIG = 24;
const int US_FRONT_ECHO = 25;
// Traseiro
const int US_REAR_TRIG  = 22;
const int US_REAR_ECHO  = 23;
// Esquerdo
const int US_LEFT_TRIG  = 26;
const int US_LEFT_ECHO  = 27;
// Direito (Pulei 28-31 pois são encoders)
const int US_RIGHT_TRIG = 32;
const int US_RIGHT_ECHO = 33;

// Distância Máxima de Leitura (para não travar o loop)
// 18000 microsegundos ~= 3 metros. Se demorar mais que isso, retorna 0.
const unsigned long MAX_PULSE_TIMEOUT = 18000; 

// ==========================================================
// --- VARIÁVEIS DE ESTADO E CALIBRAÇÃO ---
// ==========================================================

// Variáveis de Estado do Encoder
volatile long encoderCount[2] = {0, 0}; 
volatile long encoderPrevCount[2] = {0, 0};
volatile unsigned long lastTimeSpeed[2] = {0, 0}; 
volatile double currentTicksPerSecond[2] = {0.0, 0.0}; 
int pwm_L_last = 0;
int pwm_R_last = 0;
const int PWM_SLEW_STEP = 8; 

// Variáveis de Estado do IMU
Adafruit_MPU6050 mpu;
volatile double currentTheta = 0.0; // Ângulo Theta/Yaw em RADIANOS
unsigned long lastIMUTime = 0;
double gyro_bias_z = 0.0;             
bool imu_calibrated = false;          

// Constantes de Cinemática
const double MAX_RPM_SETPOINT = 40.0; 
const double WHEEL_DIAMETER_M = 0.127; 
const double BASE_WIDTH_M = 0.66; 
const double WHEEL_RADIUS_M = WHEEL_DIAMETER_M / 2.0;
const double MAX_LINEAR_VEL_MS = (MAX_RPM_SETPOINT / 60.0) * 2.0 * PI * WHEEL_RADIUS_M; 

// Variáveis de Comando (Velocidades Desejadas do ROS)
double target_v_linear = 0.0; 
double target_omega_angular = 0.0; 

// Variável para o tempo limite de segurança
unsigned long last_cmd_time = 0;
const long CMD_TIMEOUT_MS = 1200; 

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
    
    static void interruptionChAL(); static void interruptionChBL();
    static void interruptionChAR(); static void interruptionChBR();
};
Encoder* Encoder::obj_Encoder = 0;

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
// --- INSTÂNCIAS E FUNÇÕES AUXILIARES ---
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

void calibrateIMU() {
    Serial.println(F("[IMU] Iniciando calibração do giroscópio..."));
    const int samples = 500;
    double sum = 0.0;
    sensors_event_t a, g, temp;

    for (int i = 0; i < samples; i++) {
        mpu.getEvent(&a, &g, &temp);
        sum += g.gyro.z;  
        delay(5);         
    }

    gyro_bias_z = sum / samples;
    imu_calibrated = true;
    Serial.print(F("[IMU] Calibração concluída. Bias Z = "));
    Serial.println(gyro_bias_z, 6);
}

void readIMUTheta() {
    if (!imu_calibrated) return; 

    unsigned long currentTime = micros();
    double dt = (currentTime - lastIMUTime) / 1e6; 
    lastIMUTime = currentTime;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    double yaw_rate_rad_s = g.gyro.z - gyro_bias_z;
    currentTheta += yaw_rate_rad_s * dt;

    while (currentTheta > PI)  currentTheta -= 2.0 * PI;
    while (currentTheta < -PI) currentTheta += 2.0 * PI;
}

// --- FUNÇÃO DE LEITURA DE ULTRASSOM ---
float readSonarDistance(int trigPin, int echoPin) {
    // 1. Limpa o Trigger
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    
    // 2. Envia pulso de 10us
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    // 3. Lê o Echo com TIMEOUT para não travar o robô
    // MAX_PULSE_TIMEOUT deve ser configurado para ~3 metros
    long duration = pulseIn(echoPin, HIGH, MAX_PULSE_TIMEOUT);
    
    if (duration == 0) {
        // Timeout (muito longe ou erro) -> Retorna -1 ou max range
        return 999.0; // 9.99 metros (indica livre)
    }
    
    // 4. Converte para cm
    return (duration * 0.0343) / 2.0;
}

// ==========================================================
// --- FUNÇÕES DE ALTO NÍVEL ---
// ==========================================================

void calculateAndSetPWM(double v, double omega) {
    double v_ref_L = v - (omega * BASE_WIDTH_M / 2.0);
    double v_ref_R = v + (omega * BASE_WIDTH_M / 2.0);

    double max_v = MAX_LINEAR_VEL_MS; 
    
    int pwm_L = (int)((v_ref_L / max_v) * 255.0);
    int pwm_R = (int)((v_ref_R / max_v) * 255.0);

    pwm_L = slew_limit(pwm_L, pwm_L_last, PWM_SLEW_STEP);
    pwm_R = slew_limit(pwm_R, pwm_R_last, PWM_SLEW_STEP);
    pwm_L_last = pwm_L;
    pwm_R_last = pwm_R;
    
    setMotorPWM(LEFT, pwm_L); 
    setMotorPWM(RIGHT, pwm_R);
}

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

        if (buffer[0] == 'V') {
            char *token;
            double values[2];
            int i = 0;
            
            token = strtok(buffer + 2, ","); 
            
            while (token != NULL && i < 2) {
                values[i++] = atof(token); 
                token = strtok(NULL, ",");
            }
            
            if (i == 2) {
                target_v_linear = values[0];
                target_omega_angular = values[1];
                last_cmd_time = millis();
            }
        }
    }
}

void publishOdometry(unsigned long currentTime) {
    long ticksL_safe, ticksR_safe;
    unsigned long lastTimeL_safe, lastTimeR_safe;
    long prevTicksL_safe, prevTicksR_safe;
    double ticks_sL = 0.0, ticks_sR = 0.0;

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

    long deltaTicksL = ticksL_safe - prevTicksL_safe;
    long deltaTicksR = ticksR_safe - prevTicksR_safe;
    unsigned long deltaTimeL = currentTime - lastTimeL_safe;
    unsigned long deltaTimeR = currentTime - lastTimeR_safe;
    if (deltaTimeL > 0) ticks_sL = (deltaTicksL / (double)deltaTimeL) * 1000.0;
    if (deltaTimeR > 0) ticks_sR = (deltaTicksR / (double)deltaTimeR) * 1000.0;

    double theta = currentTheta;

    char buffer[96]; 
    snprintf(buffer, sizeof(buffer), "O,%ld,%ld,%.1f,%.1f,%.5f",
            ticksL_safe, ticksR_safe,
            ticks_sL, ticks_sR,
            theta);
    Serial.println(buffer);
}    

// --- Nova Função: Publicar Sensores ---
void publishSensors() {
    // Leitura Sequencial com timeout (segura para o loop)
    float d_front = readSonarDistance(US_FRONT_TRIG, US_FRONT_ECHO);
    float d_rear  = readSonarDistance(US_REAR_TRIG,  US_REAR_ECHO);
    float d_left  = readSonarDistance(US_LEFT_TRIG,  US_LEFT_ECHO);
    float d_right = readSonarDistance(US_RIGHT_TRIG, US_RIGHT_ECHO);

    // Formato: S,frente,tras,esq,dir
    Serial.print("S,");
    Serial.print(d_front, 1); Serial.print(",");
    Serial.print(d_rear, 1);  Serial.print(",");
    Serial.print(d_left, 1);  Serial.print(",");
    Serial.println(d_right, 1);
}
    
// ==========================================================
// --- SETUP E LOOP ---
// ==========================================================
void setup() {
    Serial.begin(115200);
    
    // Configura Motores
    pinMode(M1_PWM_R_PIN, OUTPUT); pinMode(M1_PWM_L_PIN, OUTPUT);
    pinMode(M2_PWM_R_PIN, OUTPUT); pinMode(M2_PWM_L_PIN, OUTPUT);

    // Configura Ultrassônicos
    pinMode(US_FRONT_TRIG, OUTPUT); pinMode(US_FRONT_ECHO, INPUT);
    pinMode(US_REAR_TRIG,  OUTPUT); pinMode(US_REAR_ECHO,  INPUT);
    pinMode(US_LEFT_TRIG,  OUTPUT); pinMode(US_LEFT_ECHO,  INPUT);
    pinMode(US_RIGHT_TRIG, OUTPUT); pinMode(US_RIGHT_ECHO, INPUT);

    encoder_obj.setup();
    stopAllMotors(); 
    encoder_obj.reset();

    Wire.begin();
    if (!mpu.begin()) {
        Serial.println(F("[ERRO] MPU6050 não detectado!"));
    } else {
        mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
        delay(500);
        calibrateIMU();          
        lastIMUTime = micros();  
    }
}
    
void loop() {
    static unsigned long last_odom_time = 0;
    static unsigned long last_sonar_time = 0; // Timer para o ultrassom
    unsigned long current_time_ms = millis();

    // 1. Receber comandos do ROS (V, ω)
    readSerialCommand();

    // 2. Verifica timeout de segurança
    if (current_time_ms - last_cmd_time > CMD_TIMEOUT_MS) {
        target_v_linear = 0.0;
        target_omega_angular = 0.0;
    }

    // 3. Cinemática inversa e controle de motores
    calculateAndSetPWM(target_v_linear, target_omega_angular);

    // 4. Publicar odometria (20Hz - Prioridade Alta)
    if (current_time_ms - last_odom_time >= ODOM_PUB_INTERVAL) {
        readIMUTheta(); 
        publishOdometry(current_time_ms);
        last_odom_time = current_time_ms;
    }

    // 5. Publicar Sensores (10Hz - Prioridade Média)
    // Fazemos separado para não engargar a Serial
    if (current_time_ms - last_sonar_time >= SONAR_PUB_INTERVAL) {
        publishSensors();
        last_sonar_time = current_time_ms;
    }
}