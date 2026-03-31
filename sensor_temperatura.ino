#include <LiquidCrystal.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Configuração LCD (Padrão DFRobot Shield - NÃO MUDAR)
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// Configuração Sensores (Pinos livres para evitar conflito com o LCD)
#define ONE_WIRE_BUS 2 
#define BUZZER_PIN 3
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Variáveis de Controle
float historico[5] = {0, 0, 0, 0, 0};
int leiturasSalvas = 0;
int modoAtual = 0; 
unsigned long tempoEstavel = 0;
float ultimaTemp = 0;
bool bipRealizado = false;

// CONSTANTE DE CORREÇÃO
const float AJUSTE_TEMP = 1.0; // Soma 1 grau ao valor lido

int ler_botao() {
  int x = analogRead(A0);
  if (x < 60)  return 0; // Direita
  if (x < 200) return 1; // Cima
  if (x < 400) return 2; // Baixo
  if (x < 600) return 3; // Esquerda
  if (x < 800) return 4; // Select
  return -1;
}

void setup() {
  lcd.begin(16, 2);
  sensors.begin();
  pinMode(BUZZER_PIN, OUTPUT);
  lcd.print("Termometro v2.1");
  lcd.setCursor(0, 1);
  lcd.print("Calibracao: +1C");
  delay(2000);
  lcd.clear();
}

void loop() {
  int botao = ler_botao();

  if (modoAtual == 0) medirTemperatura(botao);
  else if (modoAtual == 1) exibirMenu(botao);
  else if (modoAtual == 2) verHistorico(botao);
  delay(150);
}

void medirTemperatura(int btn) {
  sensors.requestTemperatures();
  float leituraReal = sensors.getTempCByIndex(0);
  
  // APLICAÇÃO DA CORREÇÃO
  float tempC = leituraReal + AJUSTE_TEMP;

  // Se o sensor desconectar (-127), não aplica o ajuste para não mostrar -126
  if (leituraReal < -100) tempC = -127.0; 

  lcd.setCursor(0, 0);
  if (tempC < -100) {
    lcd.print("ERRO: SENSOR!   ");
  } else {
    lcd.print("Lendo: ");
    lcd.print(tempC);
    lcd.write(223);
    lcd.print("C  ");
  }

  // Lógica de Estabilização e Diagnóstico baseada na temperatura corrigida
  if (abs(tempC - ultimaTemp) < 0.10) {
    if (millis() - tempoEstavel > 10000 && !bipRealizado && tempC > 30) {
      digitalWrite(BUZZER_PIN, HIGH);
      delay(500);
      digitalWrite(BUZZER_PIN, LOW);
      bipRealizado = true;
      lcd.setCursor(0, 1);
      if (tempC >= 37.8) lcd.print("FEBRE!        ");
      else if (tempC < 35.0) lcd.print("TEMP. BAIXA!   ");
      else lcd.print("ESTAVEL/NORMAL ");
    }
  } else {
    tempoEstavel = millis();
    bipRealizado = false;
    ultimaTemp = tempC;
    lcd.setCursor(0, 1);
    lcd.print("Estabilizando...");
  }

  if (btn == 4 && tempC > 0) { // SELECT para Salvar
    salvarNoHistorico(tempC);
    modoAtual = 1;
  }
  if (btn == 3) modoAtual = 1;
}

void salvarNoHistorico(float t) {
  for (int i = 4; i > 0; i--) historico[i] = historico[i - 1];
  historico[0] = t;
  if (leiturasSalvas < 5) leiturasSalvas++;
  lcd.clear();
  lcd.print("Salvo: "); lcd.print(t); lcd.print("C");
  delay(1500);
}

void exibirMenu(int btn) {
  static int selMenu = 0;
  lcd.setCursor(0, 0);
  lcd.print(">MENU PRINCIPAL ");
  lcd.setCursor(0, 1);
  if (selMenu == 0) lcd.print("1. Voltar Medir ");
  if (selMenu == 1) lcd.print("2. Ver Memoria  ");
  if (selMenu == 2) lcd.print("3. Limpar Tudo  ");

  if (btn == 1) selMenu = (selMenu <= 0) ? 2 : selMenu - 1;
  if (btn == 2) selMenu = (selMenu >= 2) ? 0 : selMenu + 1;
  
  if (btn == 4) {
    if (selMenu == 0) modoAtual = 0;
    if (selMenu == 1) modoAtual = 2;
    if (selMenu == 2) {
      for(int i=0; i<5; i++) historico[i] = 0;
      leiturasSalvas = 0;
      lcd.clear(); lcd.print("Memoria Limpa!"); delay(1000);
    }
    lcd.clear();
  }
}

void verHistorico(int btn) {
  static int idxH = 0;
  lcd.setCursor(0, 0);
  lcd.print("LOG "); lcd.print(idxH + 1); lcd.print("/5: ");
  lcd.setCursor(0, 1);
  if (historico[idxH] == 0) lcd.print("Vazio           ");
  else {
    lcd.print(historico[idxH]);
    lcd.write(223); lcd.print("C            ");
  }
  if (btn == 1) idxH = (idxH <= 0) ? 4 : idxH - 1;
  if (btn == 2) idxH = (idxH >= 4) ? 0 : idxH + 1;
  if (btn == 3) { modoAtual = 1; lcd.clear(); }
}