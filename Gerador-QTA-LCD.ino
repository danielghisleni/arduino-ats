/*erador-QTA-LCD
 * Versão 1.1.1
 * 
 * Por: Daniel Ghisleni
 * Agosto/2018
 * Última alteração: 03/02/2019
 */
// Modulo RTC no endereco 0x68
#define DS1307_ADDRESS 0x68
#define LEAP_YEAR(Y) ((Y > 0) && !(Y % 4) && ( (Y % 100) || !(Y % 400) ))

#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <Wire.h>
#include "RTClib.h"

// Inicializa o display no endereco 0x27
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
RTC_DS1307 rtc;

// Definições dos endereços EEPROM
const byte EE_MODO_OPER = 0;               // Posição 0
const byte EE_ERR_CODE  = 1;               // Posição 1
const byte EE_EX_SEM    = 2;               // Posições 2 até 7 (ativo/inativo, dia da semana, hora, minuto, duração, com carga)
const byte EE_TMP_USO   = 8;               // Posições 8 até 10 (minutos*255^2, minutos*255^1, minutos*255^0)
const byte EE_HP_INICIO = 11;              // Posição 11 (ligar gerador somente a partir das # horas)
const byte EE_HP_FIM    = 12;              // Posição 12 (ligar gerador somente até as # horas)

// Definições dos pinos - Saída
const byte pReleAtAfog  = 5;               // Pino digital  5 aciona atuador do afogador
const byte pReleIgnicao = 10;              // Pino digital 10 aciona rele de parada do gerador
const byte pRelePartida = 11;              // Pino digital 11 aciona rele de partida do gerador
const byte pLedAutom    = 3;               // Pino digital  3 informa modo automático/manual
const byte pLedErro     = 4;               // Pino digital  4 indica condição de erro
const byte pReleContRP  = 7;               // Pino digital  7 aciona contatora da rede pública
const byte pReleContGG  = 8;               // Pino digital  8 aciona contatora da do grupo gerador

// Definições dos pinos - Entrada
const byte pButPartPar  = 13;              // Pino digital 13 aciona partida/parada manualmente / seta para cima
const byte pButRPGG     = 6;               // Pino digital  6 aciona contatoras manualmente     / seta para baixo
const byte pButAM       = 9;               // Pino digital  9 alterna modo automático/manual    / seta para direita
const byte pButRst      = 12;              // Pino digital 12 comanda reinicialização           / seta para esquerda
const byte pSensorRP    = A0;              // Pino analógico A0 monitora rede pública
const byte pSensorGG    = A1;              // Pino analógico A1 monitora rede do gerador
const byte pSensorCorr  = A2;              // Pino analógico A2 monitora a corrente
const byte pSQW         = 2;               // Pino digital 2 lê SQW do RTC

// Limiares
const int  limMaxTRP    = 350;             // Coef. limite máximo aceitável de tensão na rede pública
const int  limMinTRP    = 150;             // Coef. limite mínimo aceitável de tensão na rede pública
const int  limMaxTGG    = 600;             // Coef. limite máximo aceitável de tensão no grupo gerador
const int  limMinTGG    = 150;             // Coef. limite mínimo aceitável de tensão no grupo gerador

// Temporizadores
const unsigned int tmpIgnicao[3]  = {2000, 2500, 3500};  // Tempos de ignição conforme número da tentativa
const unsigned int tmpCiclo       = 1000;  // Tempo de espera para cada ciclo em milissegundos
const unsigned int tmpContat      = 1000;  // Tempo de espera após acionar ou desacionar uma contatora
const unsigned int tmpAtuadorAfog = 2000;  // Tempo de espera após do atuador do afogador após acionar ignição
const unsigned int tmpAposComutGG = 8000;  // Tempo de espera após comutar alimentação para o grupo gerador
const unsigned int tmpAposComutRP = 8000;  // Tempo de espera após comutar alimentação para a rede pública
const unsigned int tmpAposVoltaRP = 6000;  // Tempo de espera para comutar para rede pública após ela voltar ao normal
const unsigned int tmpAposFaltaRP = 6000;  // Tempo de espera para iniciar ciclo de partida após faltar energia na rede pública
const unsigned int tmpEntreIgPart = 1000;  // Tempo de espera entre a ignição e a primeira tentativa de partida
const unsigned int tmpRefriger    = 15000; // Tempo de espera para refrigeração antes de comandar o ciclo de desligamento
const unsigned int tmpAntesTstLig = 8000;  // Tempo de espera antes de testar se tensão do grupo gerador está entre limites
const unsigned int tmpAntesTstDes = 4000;  // Tempo de espera antes de testar se tensão do grupo gerador é zero
const unsigned int tmpAntesLib    = 5000;  // Tempo de espera antes de liberar carga para o grupo gerador
const unsigned int tmpAntProxTent = 3000;  // Tempo de espera antes de tentar novamente após tentativa infrutífera
const unsigned int tmpPresBut     = 1000;  // Tempo entre pressionamentos consecutivos de um mesmo botão
const unsigned int tmpInicializ   = 3000;  // Tempo de exibição de mensagens
const unsigned int tmpAcess2aFunc = 2000;  // Tempo com o botão pressionado para acessar o menu
const unsigned int tmpMenuTimeout = 20000; // Tempo de timeout do menu
const unsigned int tmpAtualizES   = 250;   // Tempo de atualização do display durante o exercício semanal
const unsigned int tmpBotao       = 50;    // Tempo para estabilizar contato do botão
const byte numTentLigar   = 3;             // Número de tentativas de ligar o grupo gerador antes de gerar erro
const byte zero           = 0x00;          // Zero em hexadecimal

// Códigos de mensagens
const byte ERR_PARTIDA    =  1;            // Erro ao comandar a partida
const byte ERR_PARADA     =  2;            // Erro ao comandar a parada
const byte ERR_APAGOU     =  3;            // Estava em funcionamento e parou sem comandar parada
const byte ERR_AC_LOW     =  4;            // Tensão AC abaixo do limiar
const byte ERR_AC_HIGH    =  5;            // Tensão AC acima do limiar
const byte ERR_GG_LOW     =  6;            // Tensão GG abaixo do limiar
const byte ERR_GG_HIGH    =  7;            // Tensão GG acima do limiar
const byte ERR_ST_AC      =  8;            // Erro na leitura do sensor de tensão da rede pública
const byte ERR_ST_GG      =  9;            // Erro na leitura do sensor de tensão do grupo gerador
const byte INF_PARTMAN    = 10;            // Informação de partida manual
const byte INF_PARAMAN    = 11;            // Informação de parada manual
const byte INF_EXS_ON     = 12;            // Informação de exercício semanal ligado
const byte INF_REFRIG     = 13;            // Informação de ciclo de refrigeração
const byte INF_ALIM_GG    = 14;            // Informação de alimentação pelo grupo gerador
const byte MOD_AUTOM      = 15;            // Informação de status automático
const byte MOD_MANUAL     = 16;            // Informação de status manual
const byte MOD_FORA_HOR   = 17;            // Informação de fora de horário permitido

// Variáveis
bool alimEstaPeloGG       = false;         // Status da alimentação da carga
bool cancelarCicloPart    = true;          // Cancelar ciclo de partida
bool faltouEnergia        = false;         // Status de falta de energia
bool ativarExerSem        = false;         // Ativar exercício semanal
bool esEmCurso            = false;         // Exercício semanal em curso
bool esMotorLigado        = false;         // Exercício semanal motor ligado
bool emCicloRefrig        = false;         // Está em ciclo de refrigeração
bool geradorLigado        = false;         // Gerador está ligado
unsigned long momEntrouGG = 0;             // Momento em que a alimentação saiu do GG
unsigned long momEntrouRP = 0;             // Momento em que a alimentação saiu da RP
unsigned long momVoltouRP = 0;             // Momento em que a alimentação da rede pública voltou
unsigned long momFaltouRP = 0;             // Momento em que a alimentação da rede pública faltou
unsigned long momDesligGG = 0;             // Momento em que o grupo gerador foi desligado
unsigned long momLigGG    = 0;             // Momento em que o grupo gerador foi ligado
unsigned long momPresAM   = 0;             // Momento em que pressionou botão Automatico/Manual/Inibido
unsigned long momPresRst  = 0;             // Momento em que pressionou botão Reset
unsigned long momPresLM   = 0;             // Momento em que pressionou botão ligar motor
unsigned long momPresDM   = 0;             // Momento em que pressionou botão desligar motor
unsigned long momPresAltC = 0;             // Momento em que pressionou botão alternar contatoras
unsigned long momPresMenu = 0;             // Momento em que pressionou botão menu
unsigned long momPresED   = 0;             // Momento em que pressionou os botões esquerdo e direito juntos
unsigned long momEntrouES = 0;             // Momento em que entrou no exercício semanal
unsigned long segAnterior = 0;             // Segundo anterior
unsigned long tmpExercSem = 0;             // Tempo de exercício semanal em milissegundos
unsigned int minTotaisUso = 0;             // Minutos totais de uso do motor
unsigned int p8, p9, p10;                  // Bytes que armazenam o tempo de uso do motor na EEPROM
byte errCode              = 0;             // Códigos de erro
byte ultErrCode           = 0;             // Último códigos de erro
byte cmAnt                = 0;             // Código da mensagem anterior
byte codInfoDisplay       = 0;             // Código da informação a ser mostrada no display
byte dataHoraByte[7];                      // Data e hora
byte modoOperacao;                         // Status do sistema de monitoramento
byte esDiaSemana;                          // Dia do exercício semanal
byte esHora;                               // Hora do exercício semanal
byte esMinuto;                             // Minuto do exercício semanal
byte esDurMin;                             // Duração do exercício semanal em minutos
byte esAtivo;                              // Status de execução do exercício semanal (ativo/inativo)
byte horPermitInicio;                      // Horário a partir do qual é permitido ligar o gerador
byte horPermitFim;                         // Horário até o qual é permitido ligar o gerador
bool esComCarga;                           // Status de carga do exercício semanal (com carga/sem carga)
bool comecar = true;                       // Controla quando soltou o botão menu
bool partAutom = false;                    // Partida automática ocorreu

// Configurações
void setup() {
  pinMode(pRelePartida, OUTPUT);           // Seta pino como saída
  pinMode(pReleIgnicao, OUTPUT);           // Seta pino como saída
  pinMode(pReleContRP,  OUTPUT);           // Seta pino como saída
  pinMode(pReleContGG,  OUTPUT);           // Seta pino como saída
  pinMode(pLedErro,     OUTPUT);           // Seta pino como saída
  pinMode(pLedAutom,    OUTPUT);           // Seta pino como saída
  pinMode(pReleAtAfog,  OUTPUT);           // Seta pino como saída
  pinMode(pButAM,       INPUT);            // Seta pino como entrada
  pinMode(pButRst,      INPUT);            // Seta pino como entrada
  pinMode(pButPartPar,  INPUT);            // Seta pino como entrada
  pinMode(pButRPGG,     INPUT);            // Seta pino como entrada
  pinMode(pSQW,         INPUT);            // Seta pino como entrada
  pinMode(pButRst,      INPUT);            // Seta pino como entrada
  lcd.begin (16, 2);
  lcd.setBacklight(HIGH);
  inicializar();
  Wire.begin();
  rtc.begin();
  delay(250);
  //setSQWRTC();
  //zerarTemporizadorUso();
  //Serial.begin(115200);
}

// Inicialização de variáveis
void inicializar() {
  digitalWrite(pRelePartida, LOW);
  digitalWrite(pReleIgnicao, LOW);
  digitalWrite(pReleContRP, HIGH);
  digitalWrite(pReleContGG, LOW);
  digitalWrite(pLedErro, LOW);
  digitalWrite(pLedAutom, HIGH);
  digitalWrite(pReleAtAfog, HIGH);
  setMensagem("v1.1");                     // Exibe versão do firmware
  modoOperacao = getModoOperacao();        // Status do sistema de monitoramento gravado na EEPROM
  errCode      = getErrCode();             // Código de erro gravado na EEPROM
  getExerSemanal();                        // Dados de exercício semanal gravados na EEPROM
  getHorariosPermitidos();                 // Dados de horário em que é permitido ligar o gerador
  alimEstaPeloGG       = false;            // Status da alimentação da carga
  cancelarCicloPart    = true;             // Cancelar ciclo de partida
  esEmCurso            = false;            // Tirar do exercício semanal
  reiniciaMomentos();                      // Reinicia variáveis de tempo
  leTempoDeUso();                          // Lê tempo de uso do motor
}

// Função reset
void (* resetFunc) (void) = 0;             // Declara função resetFunc no endereço 0

// Programa
void loop() {
  // Lê relógio RTC
  leRelogio();
  // Se deve monitorar, chamar o monitoramento
  if (modoOperacao == MOD_AUTOM) {
    // Executa ciclo de monitoramento se não houver erro crítico e estiver em horário permitido
    if (!erroCritico() && podeLigar()) executaCicloMonitoramento();
  }
  /*
  Serial.print("RP:");
  Serial.print(analogRead(A0));
  Serial.print("  GG:");
  Serial.println(analogRead(A1));
  */
  // Aguardar para executar o próximo ciclo
  sleep(tmpCiclo);
}

// Acerta relógio com o do PC conectado via USB
void setSQWRTC() {
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
}

// Acerta relógio com data 15/06/2019 12:30:30
void setSQRTC_MID() {
  dataHoraByte[0] = 0;
  dataHoraByte[1] = 30;
  dataHoraByte[2] = 12;
  dataHoraByte[4] = 15;
  dataHoraByte[5] = 6;
  dataHoraByte[6] = 19;
  gravaRelogio();
}

// Zerar temporizadores de uso do gerador
void zerarTemporizadorUso() {
  EEPROM.write(EE_TMP_USO, 0);
  EEPROM.write(EE_TMP_USO + 1, 0);
  EEPROM.write(EE_TMP_USO + 2, 0);
  leTempoDeUso();
}

// Confere se está na hora de executar o exercício semanal
void isTimeExerSemanal() {
  // Verifica se exercício semanal está ativado
  if (esAtivo == 1 && podeLigar()) {
    // Se for o dia da semana do exercício semanal
    if (esDiaSemana == dataHoraByte[3]) {
      if (esHora == dataHoraByte[2]) {
        if (esMinuto == dataHoraByte[1]) {
          // Executa
          executarExercicioSemanal();
        }
      }
    }
  }
}

// Executa exercício semanal
void executarExercicioSemanal() {
  // Se não estiver alimentando pelo gerador
  if (!alimEstaPeloGG) {
    // Sinaliza que está em curso
    esEmCurso = true;
    setErrCode(executarCicloPartidaGG());
    if (erroCritico()) {
      esEmCurso = false;
      esMotorLigado = false;
    } else {
      momEntrouES = millis();
      tmpExercSem = (unsigned long) (esDurMin * 60l * 1000l);
      esMotorLigado = true;
    }
  }
}

// Reinicia todos os registradores de momentos
void reiniciaMomentos() {
  unsigned long agora = millis();
  momEntrouGG = agora;           // Momento em que a alimentação saiu do GG
  momEntrouRP = agora;           // Momento em que a alimentação saiu da RP
  momVoltouRP = agora;           // Momento em que a alimentação da rede pública voltou
  momFaltouRP = agora;           // Momento em que a alimentação da rede pública faltou
  momDesligGG = agora;           // Momento em que o grupo gerador foi desligado
  momLigGG    = agora;           // Momento em que o grupo gerador foi ligado
  momPresAM   = agora;           // Momento em que pressionou botão Automatico/Manual
}

// Obtém modo de operação gravado na EEPROM
byte getModoOperacao() {
  byte modo = EEPROM.read(EE_MODO_OPER);
  if (modo != MOD_AUTOM && modo != MOD_MANUAL) {
    modo = MOD_AUTOM;
    setModoOperacao(modo);
  }
  return modo;
}

// Obtém código de erro gravado na EEPROM
byte getErrCode() {
  byte errC = EEPROM.read(EE_ERR_CODE);
  if (errC != ERR_PARTIDA && errC != ERR_PARADA && errC != 0) {
    errC = 0;
    setErrCode(errC);
  }
  return errC;
}

// Obtém horários permitidos para ligar o gerador
void getHorariosPermitidos() {
  byte es;
  es = EEPROM.read(EE_HP_INICIO);
  horPermitInicio = (es > 23 || es < 0 ? 0 : es);
  es = EEPROM.read(EE_HP_FIM);
  horPermitFim = (es > 23 || es < 0 ? 0 : es);
}

// Verifica se pode ligar o gerador neste horário
bool podeLigar() {
  bool retorno = false;
  // Se inicio e fim forem zeros, permitir
  if (horPermitInicio == 0 && horPermitFim == 0) retorno = true;
  else {
    // Se horário de inicio maior que horário de fim
    if (horPermitInicio < horPermitFim) {
      if (dataHoraByte[2] >= horPermitInicio && dataHoraByte[2] < horPermitFim) retorno = true;
    } else {
      byte hpf = (byte) (horPermitFim + 24);
      byte dhb = (dataHoraByte[2] < horPermitInicio ? (byte) (dataHoraByte[2] + 24) : dataHoraByte[2]);
      if (dhb >= horPermitInicio && dhb < hpf) retorno = true;
    }
  }
  return retorno;
}

// Obtém dados do exercicio semanal da EEPROM
void getExerSemanal() {
  byte es;
  // Ativo/inativo
  es = EEPROM.read(EE_EX_SEM);
  if (es < 0 || es > 1) {
    esAtivo = 0;
    EEPROM.write(EE_EX_SEM, 0);
  } else esAtivo = es;
  // Dia da semana
  es = EEPROM.read(EE_EX_SEM + 1);
  if (es < 0 || es > 6) {
    esDiaSemana = 0;
    EEPROM.write(EE_EX_SEM + 1, 0);
  } else esDiaSemana = es;
  // Hora
  es = EEPROM.read(EE_EX_SEM + 2);
  if (es < 0 || es > 23) {
    esHora = 10;
    EEPROM.write(EE_EX_SEM + 2, 10);
  } else esHora = es;
  // Minuto
  es = EEPROM.read(EE_EX_SEM + 3);
  if (es < 0 || es > 59) {
    esMinuto = 10;
    EEPROM.write(EE_EX_SEM + 3, 10);
  } else esMinuto = es;
  // Duração em minutos
  es = EEPROM.read(EE_EX_SEM + 4);
  if (es < 3 || es > 180) {
    esDurMin = 15;
    EEPROM.write(EE_EX_SEM + 4, 15);
  } else esDurMin = es;
  // Com ou sem carga
  es = EEPROM.read(EE_EX_SEM + 5);
  if (es < 0 || es > 1) {
    esComCarga = 0;
    EEPROM.write(EE_EX_SEM + 5, 0);
  } else esComCarga = es;
}

// Grava dados do exercício semanal na EEPROM
void setExerSemanal() {
  EEPROM.write(EE_EX_SEM, esAtivo);
  EEPROM.write(EE_EX_SEM + 1, esDiaSemana);
  EEPROM.write(EE_EX_SEM + 2, esHora);
  EEPROM.write(EE_EX_SEM + 3, esMinuto);
  EEPROM.write(EE_EX_SEM + 4, esDurMin);
  EEPROM.write(EE_EX_SEM + 5, esComCarga);
}

// Muda o modo de operação e grava na EEPROM
void setModoOperacao(byte modo) {
  modoOperacao = modo;
  EEPROM.write(EE_MODO_OPER, modo);
}

// Grava dados dos horários permitidos
void setHorPermit() {
  EEPROM.write(EE_HP_INICIO, horPermitInicio);
  EEPROM.write(EE_HP_FIM, horPermitFim);
}

// Ciclo de monitoramento
void executaCicloMonitoramento() {

  // Verifica status da rede pública
  if (redePublicaOK()) {
    // Zerar momento em que faltou energia na rede pública
    momFaltouRP = millis();
    // Verifica se o gerador está ligado
    if (redeGGOK()) {
      // Gerador está ligado, verifica agora se a alimentação da carga está pelo gerador
      if (alimEstaPeloGG) {
        if ((unsigned long) (millis() - momEntrouGG) > tmpAposComutGG && (unsigned long) (millis() - momVoltouRP) > tmpAposVoltaRP) {
          // Comutar para a rede pública
          setMensagem("Mudando para RP");
          comutarGGparaRP();
          // Zerar momento em que voltou a rede pública
          momVoltouRP = millis();
          // Código de mensagem
          errCode = 0;
        }
      } else {
        // Verifica se o gerador está ligado
        if (momLigGG > 0) {
          // Se a partida foi automática ou estiver em modo automático
          if (partAutom || modoOperacao == MOD_AUTOM) {
            // Verifica se exercício semanal não está em curso
            if (!esEmCurso) {
              // Verifica se já passou o período de refrigeração
              if ((unsigned long) (millis() - momLigGG) > tmpRefriger && (unsigned long) (millis() - momVoltouRP) > tmpRefriger) {
                setMensagem("Ciclo de parada");
                executarCicloParadaGG();
                errCode = 0;
                setErrCode(0);
                geradorLigado = false;
                partAutom = false;
                momLigGG = 0;
                ultErrCode++;
              } else if (geradorLigado && ((unsigned long) (millis() - momVoltouRP) < tmpRefriger)) setErrCode(INF_REFRIG);
            }
          }
        }
      }
    } else {
      partAutom = false;
      // Força atualização da mensagem
      if (faltouEnergia) {
        cmAnt++;
        momFaltouRP = millis();
        faltouEnergia = false;
      }
    }
  } else {
    momVoltouRP = millis();
    // Rede pública não está ok, cancelar exercício semanal se estiver em curso
    esEmCurso = false;
    // Verificar se há erro crítico
    if (!erroCritico()) {
      // Não está com nenhum código de erro, guardar momento em que faltou energia se for o caso, e verificar se gerador está ligado
      if (!alimEstaPeloGG && momFaltouRP == 0) momFaltouRP = millis();
      // Verifica se o gerador está ligado
      if (redeGGOK()) {
        // Gerador está em funcionamento, verificar se a alimentação ainda não está por ele
        if (!alimEstaPeloGG) {
          // Alimentação ainda não está pelo grupo gerador, comutar se já passou o tempo mínimo
          if ((unsigned long) (millis() - momEntrouRP) > tmpAposComutRP) {
            setMensagem("Mudando para GG");
            // Comutar para o grupo gerador
            comutarRPparaGG();
            // Código de mensagem
            errCode = INF_ALIM_GG;
            // Zerar momento em que faltou energia na rede pública
            momFaltouRP = 0;
          }
        }
      } else {
        // Se alimentação já está pelo gerador, ele apagou, desacionar ignição, comutar para RP e gerar erro
        if (alimEstaPeloGG) {
          setMensagem("Parando gerador");
          // Compandar mudança da alimentação de grupo gerador para rede pública
          comutarGGparaRP();
          // Armazenar erro à errCode
          setErrCode(ERR_APAGOU);
        } else {
          // Gerador não está em funcionamento, executar o ciclo de partida se já passou tempo mínimo
          if ((unsigned long) (millis() - momFaltouRP) > tmpAposFaltaRP) {
            setMensagem("Ciclo partida");
            setErrCode(executarCicloPartidaGG());
            partAutom = true;
          } else {
            if ((unsigned long) (millis() - momFaltouRP) < tmpAposFaltaRP) {
              setMensagem("Sem energia RP");
              momVoltouRP = millis();
              faltouEnergia = true;
            }
          }
        }
      }
    }
  }
}

// Processa código de erro
void setErrCode(byte e) {
  if (e != ultErrCode && e > 0) {
    setMensagem(getMensagem(e - 1));
    errCode = e;
  }
  if (e == 0 || erroCritico()) {
    EEPROM.write(EE_ERR_CODE, e);
  }
  ultErrCode = e;
}

// Exibe mensagem no display
void setMensagem(String m) {
  lcdWrite(1, m);
  ultErrCode++;
}

void atualizaInterface() {
  // Se tiver erro crítico, informar no painel
  if (erroCritico()) {
    // Informar no painel
    digitalWrite(pLedErro, HIGH);
  }
  atualizaDisplay();
}

// Atualiza display
void atualizaDisplay() {
  // Código da mensagem a exibir
  byte cm;
  // Controla leds do modo de operação
  controlaLedModoOp();
  // Atualiza primeira linha
  if (esEmCurso) lcdWrite(0, getMensagem(11));
  else {
    switch (codInfoDisplay) {
      case 0: lcdWrite(0, data() + " " + hora());                       break; // Data e hora
      case 1: lcdWrite(0, String("Tensao RP: ") + getTensaoRP() + "V"); break; // Tensão na AC
      case 2: lcdWrite(0, String("Tensao GG: ") + getTensaoGG() + "V"); break; // Tensão no gerador
      case 3: lcdWrite(0, String("Corrente: ") + getCorrente() + "A");  break; // Carga em amperes
      case 4: lcdWrite(0, String("Consumo: ") + getConsumo() + "W");    break; // Consumo em Watts
      case 5: lcdWrite(0, String("Tmp uso: ") + getTempoUso());         break; // Tempo de uso do gerador
    }
  }
  // Atualiza segunda linha
  if (esEmCurso && esMotorLigado) {
    lcdWrite(1, getTempo(((unsigned long) (tmpExercSem - millis() + momEntrouES)) / 1000l));
  } else {
    if (errCode == 0) {
      if (podeLigar()) cm = modoOperacao;
      else cm = MOD_FORA_HOR;
    }
    else cm = errCode;
    // Só reescreve se mudou a mensagem
    if (cm != cmAnt && cm > 0) lcdWrite(1, getMensagem(cm - 1));
    // Atualiza código da última mensagem
    cmAnt = cm;
  }
}

// Registra na EEPROM o tempo ligado
void registrarTempoLigado() {
  unsigned long tLigado = (unsigned int) (millis() - momLigGG);
  // Lê informação gravada
  byte xp8  = EEPROM.read(EE_TMP_USO),
       xp9  = EEPROM.read(EE_TMP_USO + 1),
       xp10 = EEPROM.read(EE_TMP_USO + 2);
  // Calcula
  minTotaisUso = (unsigned int) ((xp8 + (xp9 * 256) + (xp10 * 65536)) + (unsigned int) (tLigado / 60000));
  // Grava nova informação
  p8 = (byte) (minTotaisUso % 256);
  p9 = (byte) (minTotaisUso / 256);
  p10 = (byte) (minTotaisUso / 65536);
  EEPROM.write(EE_TMP_USO, p8);
  EEPROM.write(EE_TMP_USO + 1, p9);
  EEPROM.write(EE_TMP_USO + 2, p10);
}

// Retorna o tempo de uso em horas e minutos
String getTempoUso() {
  unsigned int totalMin = minTotaisUso;
  unsigned int horas;
  byte minutos;
  // Se motor estiver ligado, soma minutos
  if (redeGGOK()) totalMin += (unsigned int) ((unsigned long) (millis() - momLigGG) / 60000);
  horas = (unsigned int) (totalMin / 60);
  minutos = (byte) totalMin % 60;
  if (horas > 0) return (String(horas) + "h" + String(minutos));
  else return (String(minutos) + "min");
}

// Lê bytes do tempo de uso
void leTempoDeUso() {
  p8  = EEPROM.read(EE_TMP_USO);
  p9  = EEPROM.read(EE_TMP_USO + 1);
  p10 = EEPROM.read(EE_TMP_USO + 2);
  minTotaisUso = (unsigned int) ((p10 * 65536) + (p9 * 256) + p8);
}

// Retorna tensão na leitura do sensor RP
unsigned int getTensaoRP() {
  unsigned int leitura = analogRead(pSensorRP);
  return (unsigned int) (leitura * 0.75f);
}

// Retorna tensão na leitura do sensor GG
unsigned int getTensaoGG() {
  unsigned int leitura = analogRead(pSensorGG);
  return (unsigned int) (leitura * 1.1f);
}

// Retorna corrente na leitura do sensor de corrente em Amperes
byte getCorrente() {
  byte leitura = analogRead(pSensorCorr);
  return leitura;
}

// Retorna consumo em Watts
unsigned int getConsumo() {
  unsigned int leitura = getCorrente();
  return (unsigned int) (leitura * (alimEstaPeloGG ? getTensaoGG() : getTensaoRP()));
}

// Próxima informação do display
void proximaInfoDisplay() {
  codInfoDisplay++;
  if (codInfoDisplay > 5) codInfoDisplay = 0;
  atualizaDisplay();
}

// Retorna tempo em formato mm:ss
String getTempo(unsigned int t) {
  byte segs = t % 60;
  byte mins = t / 60;
  return leadZero(mins) + ":" + leadZero(segs);
}

String getMensagem(byte cm) {
  switch (cm) {
    case  0: return "Erro na partida";  break;
    case  1: return "Erro na parada";   break;
    case  2: return "Parada inesperad"; break;
    case  3: return "RP abaixo do lim"; break;
    case  4: return "RP acima do lim";  break;
    case  5: return "GG abaixo do lim"; break;
    case  6: return "GG acima do lim";  break;
    case  7: return "Erro sensor RP";   break;
    case  8: return "Erro sensor GG";   break;
    case  9: return "Partida manual";   break;
    case 10: return "Parada manual";    break;
    case 11: return "Exerc. semanal";   break;
    case 12: return "Refrigerando...";  break;
    case 13: return "Ger. alimentando"; break;
    case 14: return "Modo automatico";  break;
    case 15: return "Modo manual";      break;
    case 16: return "Fora de horario";  break;
  }
}

// Controla os leds do modo de operação
void controlaLedModoOp() {
  digitalWrite(pLedAutom, modoOperacao == MOD_AUTOM ? HIGH : LOW);
}

// Escreve mensagem em uma linha do LCD
void lcdWrite(byte lin, String msg) {
  lcd.setCursor(0, lin);
  lcd.print(String(msg) + "                ");
}

// Retorna true se erro for crítico
bool erroCritico() {
  // Erros críticos que impedem o uso da AC do gerador
  if (errCode == ERR_PARTIDA || errCode == ERR_APAGOU) return true;
  else return false;
}

// Retorna true se o gerador está em funcionamento
bool redeGGOK() {
  int leituraTensaoGG = analogRead(pSensorGG);
  bool retorno = false;
  // Se a leitura é válida, prosseguir
  if (leituraSTValida(leituraTensaoGG, ERR_ST_GG)) {
    // Se a leitura está dentro dos limiares, retornar true
    if (leituraTensaoGG <= limMaxTGG && leituraTensaoGG >= limMinTGG) {
      retorno = true;
    }
  }
  return retorno;
}

// Retorna true se rede pública está com a tesão entre os limiares
bool redePublicaOK() {
  bool retorno = false;
  // Se estiver em exercício semanal com carga, simular falta de energia (experimental)
  if (esAtivo && esComCarga) retorno = false;
  else {
    int leituraTensaoRP = analogRead(pSensorRP);
    // Se a leitura é válida, prosseguir
    if (leituraSTValida(leituraTensaoRP, ERR_ST_AC)) {
      // Se leitura está dentro dos limites, retornar true
      if (leituraTensaoRP <= limMaxTRP && leituraTensaoRP >= limMinTRP) {
        retorno = true;
      }
    }
  }
  return retorno;
}

// Verifica se a leitura de tensão do sensor é válida
bool leituraSTValida(int leitura, unsigned int codigoErro) {
  bool retorno = true;
  // Se a leitura for inválida
  if (leitura < 0 || leitura > 800) {
    // Armazenar código de erro e retornar false
    errCode |= codigoErro;
    retorno = false;
  }
  return retorno;
}

// Comuta a alimentação da carga de grupo gerador para rede pública
void comutarGGparaRP() {
  // Desativa comando da contatora do grupo gerador, cortando assim a alimentação da carga
  releContatoraGG(false);
  // Ativa comando da contatora do AC, alimentando assim a carga com RP
  releContatoraRP(true);
  // Armazena momento em que a alimentação foi comutada para RP
  momEntrouRP = millis();
  // Atualiza variável com status da alimentação
  alimEstaPeloGG = false;
}

// Comuta a alimentação da carga de AC para grupo gerador
void comutarRPparaGG() {
  // Desativa comando da contatora do AC, cortando assim a alimentação da carga
  releContatoraRP(false);
  // Ativa comando da contatora do GG, alimentando assim a carga com o gerador
  releContatoraGG(true);
  // Armazena momento em que a alimentação foi comutada para GG
  momEntrouGG = millis();
  // Atualiza variável com status da alimentação
  alimEstaPeloGG = true;
}

// Controla o relé da contatora da AC
void releContatoraRP(bool st) {
  // Se for true, ativar a contatora
  if (st) {
    digitalWrite(pReleContRP, HIGH);
  } else {
    digitalWrite(pReleContRP, LOW);
  }
  delay(tmpContat);
}

// Controla o relé da contatora do grupo gerador
void releContatoraGG(bool st) {
  // Se for true, ativar a contatora
  if (st) {
    digitalWrite(pReleContGG, HIGH);
  } else {
    digitalWrite(pReleContGG, LOW);
  }
  delay(tmpContat);
}

byte executarCicloParadaGG() {
  byte retorno = 0;
  // Variável para armazenar se gerador está ligado
  bool estaLigado = true;
  // Executa parada
  estaLigado = doParada();
  if (estaLigado) {
    retorno = ERR_PARADA;
  } else {
    momDesligGG = millis();
    geradorLigado = false;
  }
  registrarTempoLigado();
  return retorno;
}

// Executa parada cortando a ignição
bool doParada() {
  bool estaLigado;
  // Aciona o rele de ignição, cortando a alimentação
  acionaReleIgnicao(true);
  // Aguarda 4 segundos
  delay(4000);
  // Libera rele de ignição
  acionaReleIgnicao(false);
  // Aguarda um tempo antes de verificar se está parado
  delay(tmpAntesTstDes);
  // Verifica se motor está operante
  estaLigado = redeGGOK();
  // Atualiza variável
  geradorLigado = false;
  // Retorna
  return estaLigado;
}

// Executa ciclo de partida do motor
byte executarCicloPartidaGG() {
  byte retorno = 0;
  cancelarCicloPart = false;
  // Inicia contador de tentativas
  int tentativa = 0;
  // Variável para armazenar se gerador está ligado
  bool estaLigado = false;
  // Enquanto numero de tentativas menor que o máximo e não estiver operante
  while (tentativa < numTentLigar && !estaLigado && !cancelarCicloPart) {
    lcdWrite(1, String("Ciclo partida ") + (tentativa + 1));
    // Corta ignição pra evitar partida com motor ligado
    doParada();
    // Executa partida
    estaLigado = doPartida(tentativa);
    // Verifica se não voltou a energia da rede pública
    if (redePublicaOK()) {
      cancelarCicloPart = true;
    }
    // Aguarda antes de liberar ou tentar novmanete, conforme o caso
    sleep(estaLigado ? tmpAntesLib : tmpAntProxTent);
    // Incrementa o número da tentativa
    tentativa++;
  }
  // Se saiu do laço e rede pública estiver OK, não fazer mais nada
  if (!redePublicaOK()) {
    if (!cancelarCicloPart) {
      // Se saiu do laço e estiver ligado, armazenar o momento atual
      if (estaLigado) {
        momLigGG = millis();
      } else retorno = ERR_PARTIDA;
     // Finalizar ciclo
      cancelarCicloPart = true;
    }
  }
  // Atualiza variável
  geradorLigado = estaLigado;
  // Retorna
  return retorno;
}

// Comanda a partida através dos reles
bool doPartida(int tentativa) {
  bool retorno;
  // Comanda partida do motor
  acionaRelePartida(tentativa);
  // Aguarda um tempo antes de verificar se está operante
  sleep(tmpAntesTstLig);
  // Verifica se motor está operante
  retorno = redeGGOK();
  // Retorna
  return retorno;
}

// Aciona o rele de ignição
void acionaReleIgnicao(bool stat) {
  digitalWrite(pReleIgnicao, (stat ? HIGH : LOW));
}

// Aciona o rele de partida pelo tempo estipulado
void acionaRelePartida(int tent) {
  // Aciona rele do atuador do afogador
  acionaReleAtAfog(true);
  // Espera 1 segundo
  delay(1000);
  // Aciona rele de partida
  digitalWrite(pRelePartida, HIGH);
  // Espera o tempo estipulado
  delay(tmpIgnicao[tent]);
  // Desaciona rele
  digitalWrite(pRelePartida, LOW);
  // Espera um tempo
  delay(tmpAtuadorAfog);
  // Desaciona rele do atuador do afogador
  acionaReleAtAfog(false);
}

// Aciona relé do atuador do afogador (lógica inversa)
void acionaReleAtAfog(bool ativo) {
  digitalWrite(pReleAtAfog, ativo ? LOW : HIGH);
}

// Cancelar exercício semanal
void cancelarExerSem() {
  esEmCurso = false;
}

/*
  pButPartPar  = 13;              // Pino digital 13 aciona partida/parada manualmente / seta para cima
  pButRPGG     = 6;               // Pino digital  6 aciona contatoras manualmente     / seta para baixo
  pButAM       = 9;               // Pino digital  9 alterna modo automático/manual    / seta para direita
  pButRst      = 12;              // Pino digital 12 comanda reinicialização           / seta para esquerda
 */

// Aguarda um tempo
void sleep(unsigned long ms) {
  unsigned long inicio = millis();
  unsigned long inicioTmpDisplayES = millis();
  while (millis() - inicio < ms) {
    // Se pressionou dois botões juntos (esquerda e direita)
    if (digitalRead(pButRst) == HIGH && digitalRead(pButAM) == HIGH) {
      momPresED = millis();
      while (digitalRead(pButRst) == HIGH && digitalRead(pButAM) == HIGH && (unsigned long) (millis() - momPresED) < tmpAcess2aFunc) {
        delay(tmpBotao);
      }
      // Se continuam pressionados, resetar
      if ((unsigned long) (millis() - momPresED) >= tmpAcess2aFunc) {
        setMensagem("Zerando contador");
        zerarTemporizadorUso();
        delay(tmpInicializ);
        momPresED = millis();
        // Se continua pressionando, próximo passo é resetar o relógio com a data de 15/06/2019
        while (digitalRead(pButRst) == HIGH && digitalRead(pButAM) == HIGH && (unsigned long) (millis() - momPresED) < tmpAcess2aFunc) {
          delay(tmpBotao);
        }
        // Se continuam pressionados, resetar
        if ((unsigned long) (millis() - momPresED) >= tmpAcess2aFunc) {
          setMensagem("Zerando relogio");
          setSQWRTC();
          delay(tmpInicializ);
        }
        cmAnt++;
        atualizaInterface();
      }
    } else {
      // Se pressionou botão Reset (seta para esquerda)
      if (digitalRead(pButRst) == HIGH && (unsigned long) (millis() - momPresRst) > tmpPresBut) {
        momPresRst = millis();
        // Aguarda até que o botão seja liberado
        unsigned long pressionou = millis();
        while (digitalRead(pButRst) == HIGH && (unsigned long) (millis() - pressionou) < tmpAcess2aFunc) {
          delay(tmpBotao);
        }
        // Se ficou mais tempo pressionado, comanda reset
        if ((unsigned long) (millis() - pressionou) >= tmpAcess2aFunc) {
          // Se exercício semanal estiver em curso, cancelar
          if (esEmCurso) {
            ms = 0;
            cancelarExerSem();
          }
          momPresRst = millis();
          cancelarCicloPart = true;
          setErrCode(0);
          codInfoDisplay = 0;
          setMensagem("Reinicializando");
          //inicializar();
          sleep(tmpInicializ);
          //ms = 0;
          //cmAnt++;
          resetFunc();
          //atualizaInterface();
        } else {
          // Vai para a próxima informação no display
          proximaInfoDisplay();
        }
      }
      // Se pressionou botão Aut/Man (seta para direita)
      if (digitalRead(pButAM) == HIGH) {
        momPresAM = millis();
        // Verifica quanto tempo ficou pressionado o botão
        unsigned long pressionou = millis();
        // Aguarda até que o botão seja liberado
        while (digitalRead(pButAM) == HIGH && (unsigned long) (millis() - pressionou) < tmpAcess2aFunc) {
          delay(tmpBotao);
        }
        // Se ainda continua pressionado, chamar o menu, senão é só um clique
        if (digitalRead(pButAM) == HIGH) {
          // Chama o menu
          comecar = false;
          goMenu();
          cmAnt++;
          atualizaInterface();
          while (digitalRead(pButAM) == HIGH) {
            delay(tmpBotao);
          }
        } else {
          // Se exercício semanal estiver em curso, cancelar
          if (esEmCurso) {
            ms = 0;
            cancelarExerSem();
          }
          // Alterna modo de operação
          setModoOperacao(modoOperacao == MOD_AUTOM ? MOD_MANUAL : MOD_AUTOM);
          // Cancelar ciclo de partida se estiver em andamento
          cancelarCicloPart = true;
          // Reinicializa momentos
          reiniciaMomentos();
          // Expira tempo de espera para essa rotina
          atualizaInterface();
        }
      }
      // Se pressionou botão Partida/Parada (seta para cima)
      if (digitalRead(pButPartPar) == HIGH && (unsigned long) (millis() - momPresLM) > tmpPresBut && !esEmCurso) {
        momPresLM = millis();
        momFaltouRP = millis();
        // Se estiver em modo manual, comutar entre ligado e desligado
        if (modoOperacao == MOD_MANUAL) {
          if (redeGGOK()) setErrCode(executarCicloParadaGG());
          else doPartida(1);
        }
      }
      // Se pressionou botão Alternar contatoras (seta para baixo)
      if (digitalRead(pButRPGG) == HIGH && (unsigned long) (millis() - momPresAltC) > tmpPresBut) {
        //momFaltouRP = millis();
        // Se exercício semanal estiver em curso, cancelar
        if (esEmCurso) {
          ms = 0;
          cancelarExerSem();
        }
        momPresAltC = millis();
        // Se estiver em modo manual, comutar entre rede pública e gerador
        if (modoOperacao == MOD_MANUAL) {
          if (alimEstaPeloGG) {
            comutarGGparaRP();
          } else {
            comutarRPparaGG();
          }
        }
      }
    }
    // Ve se precisa executar alguma tarefa
    if (millis() > segAnterior  && !esEmCurso) {
      if (!erroCritico()) {
        isTimeExerSemanal();
        segAnterior = millis() + 1000;
      }
    }
    // Se está no exercício semanal, atualizar a cada segundo
    if (esEmCurso && esMotorLigado) {
      // Atualiza interface
      if ((unsigned long) (millis() - inicioTmpDisplayES) > tmpAtualizES) {
        atualizaInterface();
        inicioTmpDisplayES = millis();
      }
      // Se terminou o tempo, desligar o motor
      if ((unsigned long) (millis() - momEntrouES > tmpExercSem)) {
        // Se a rede pública está OK e alimentação não está pelo gerador
        if (redePublicaOK() && !alimEstaPeloGG) {
          // Se o exercício semanal foi com carga
          if (esComCarga) {
            // Ativa refrigeração do motor
            momLigGG = millis();
          } else {
            // Executa parada imediatamente
            setErrCode(executarCicloParadaGG());
          }
        }
        // Finaliza
        esEmCurso = false;
        esMotorLigado = false;
        cmAnt++;
      } else {
        // Não terminou o tempo do exercício ainda, verificar se motor está ok
        if (!redeGGOK()) {
          // Finaliza e sinaliza erro
          esEmCurso = false;
          esMotorLigado = false;
          cmAnt++;
          setErrCode(ERR_PARADA);
        }
      }
    }
  }
  // Atualiza o display
  atualizaInterface();
}

// Le dados do relógio RTC
void leRelogio() {
  Wire.beginTransmission(DS1307_ADDRESS);
  Wire.write(zero);
  Wire.endTransmission();
  Wire.requestFrom(DS1307_ADDRESS, 7);
  for (char cont = 0; cont < 7; cont++) dataHoraByte[cont] = converteHexParaDec(Wire.read());
}

// Verifica se a data contida no array é válida
bool validaData() {
  bool retorno = false;
  switch (dataHoraByte[5]) {
    case 2: retorno = (dataHoraByte[4] <= (LEAP_YEAR(2000 + dataHoraByte[6] ? 29 : 28))); break;
    case 1: case 3: case 5:
    case 7: case 8: case 10:
    case 12: retorno = (dataHoraByte[4] <= 31); break;
    case 4: case 6: case 9:
    case 11: retorno = (dataHoraByte[4] <= 30); break;
  }
  return retorno;
}

// Retorna o dia da semana (mandar o ano com 4 dígitos)
byte dayOfWeek(uint16_t year, uint8_t month, uint8_t day) {
  uint16_t months[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, 365};
  uint32_t days = year * 365;
  for (uint16_t i = 4; i < year; i += 4) {
    if (LEAP_YEAR(i)) days++;
  }
  days += months[month - 1] + day;
  if ((month > 2) && LEAP_YEAR(year)) days++;
  return days % 7;
}

// Grava dados no relógio RTC
void gravaRelogio() {
  if (validaData()) {
    Wire.beginTransmission(DS1307_ADDRESS);
    Wire.write(zero);
    for (byte cont = 0; cont < 7; cont++) Wire.write(converteDecParaHex(cont == 3 ? dayOfWeek(2000 + dataHoraByte[6], dataHoraByte[5], dataHoraByte[4]) : dataHoraByte[cont]));
    Wire.write(zero);
    Wire.endTransmission();
  }
}

// Dia da semana
String diaDaSemana(byte dds) {
  switch (dds) {
    case 0: return "Dom"; break;
    case 1: return "Seg"; break;
    case 2: return "Ter"; break;
    case 3: return "Qua"; break;
    case 4: return "Qui"; break;
    case 5: return "Sex"; break;
    default: return "Sab"; break;
  }
}

// Monta data
String data() {
  return diaDaSemana(dataHoraByte[3]) + " " + getDia() + "/" + getMes();
}

// Monta hora
String hora() {
  return getHora() + ":" + getMinuto();
}

// Retorna a hora com 2 dígitos
String getHora() {
  return leadZero(dataHoraByte[2]);
}

// Retorna o minuto com 2 dígitos
String getMinuto() {
  return leadZero(dataHoraByte[1]);
}

// Retorna o dia com 2 dígitos
String getDia() {
  return leadZero(dataHoraByte[4]);
}

// Retorna o mês com 2 dígitos
String getMes() {
  return leadZero(dataHoraByte[5]);
}

// Retorna o ano com 4 dígitos
String getAno() {
  return leadZero(dataHoraByte[6]);
}

// Converte de hexadecimal para decimal
byte converteHexParaDec(byte v) {
  return ((v >> 4) * 10) + (v % 16);
}

// Converte de decimal para hexadecimal
byte converteDecParaHex(byte c) {
  return ((c / 10) << 4) + (c % 10);
}

// Se o número for menor que 10, colocar 0 na frente
String leadZero(byte v) {
  return (v > 9 ? String(v) : String("0") + String(v));
}

// Menu
void goMenu() {
  // Enquanto não teclar menu de novo
  momPresMenu = millis();
  bool sairMenu = false;
  bool comecar = false;
  bool pressionouAlgo = false;
  byte opcaoAtual = 0, opcaoAnterior = 1;
  byte valOpcaoAtual;
  byte tmpPBMenu = 100;
  unsigned long momEntrouMenu = millis();
  while (!sairMenu) {
    // Mensagem da primeira linha
    switch (opcaoAtual) {
      case 0:
        leRelogio();
        valOpcaoAtual = dataHoraByte[4];
        if (opcaoAnterior != opcaoAtual) lcdWrite(0, "Dia atual");
        break;
      case 1:
        leRelogio();
        valOpcaoAtual = dataHoraByte[5];
        if (opcaoAnterior != opcaoAtual) lcdWrite(0, "Mes atual");
        break;
      case 2:
        leRelogio();
        valOpcaoAtual = dataHoraByte[6];
        if (opcaoAnterior != opcaoAtual) lcdWrite(0, "Ano atual");
        break;
      case 3:
        leRelogio();
        valOpcaoAtual = dataHoraByte[2];
        if (opcaoAnterior != opcaoAtual) lcdWrite(0, "Hora atual");
        break;
      case 4:
        leRelogio();
        valOpcaoAtual = dataHoraByte[1];
        if (opcaoAnterior != opcaoAtual) lcdWrite(0, "Minuto atual");
        break;
      case 5:
        getExerSemanal();
        valOpcaoAtual = esAtivo;
        if (opcaoAnterior != opcaoAtual) lcdWrite(0, "Ativa exer. sem.");
        lcdWrite(1, valOpcaoAtual == 0 ? "Inativo" : "Ativo");
        break;
      case 6:
        getExerSemanal();
        valOpcaoAtual = esDiaSemana;
        if (opcaoAnterior != opcaoAtual) lcdWrite(0, "Dia exercicio");
        lcdWrite(1, String(diaDaSemana(esDiaSemana)));
        break;
      case 7:
        getExerSemanal();
        valOpcaoAtual = esHora;
        if (opcaoAnterior != opcaoAtual) lcdWrite(0, "Hora exercicio");
        break;
      case 8:
        getExerSemanal();
        valOpcaoAtual = esMinuto;
        if (opcaoAnterior != opcaoAtual) lcdWrite(0, "Minuto exercicio");
        break;
      case 9:
        getExerSemanal();
        valOpcaoAtual = esDurMin;
        if (opcaoAnterior != opcaoAtual) lcdWrite(0, "Duracao exercicio");
        break;
      case 10:
        getExerSemanal();
        valOpcaoAtual = esComCarga;
        if (opcaoAnterior != opcaoAtual) lcdWrite(0, "Carga no exerc.");
        lcdWrite(1, valOpcaoAtual == 0 ? "Sem carga" : "Com carga");
        break;
      case 11:
        getHorariosPermitidos();
        valOpcaoAtual = horPermitInicio;
        if (opcaoAnterior != opcaoAtual) lcdWrite(0, "Hora perm inicio");
        break;
      case 12:
        getHorariosPermitidos();
        valOpcaoAtual = horPermitFim;
        if (opcaoAnterior != opcaoAtual) lcdWrite(0, "Hora perm fim");
        break;
    }
    // Memoriza a última opção
    opcaoAnterior = opcaoAtual;
    // Exibição do display se valores são obtidos diretamente das variáveis
    if (opcaoAtual != 5 && opcaoAtual != 6 && opcaoAtual != 10) lcdWrite(1, leadZero(valOpcaoAtual));
    // Enquanto não pressionou nada
    pressionouAlgo = false;
    if (!comecar) {
      while (digitalRead(pButAM) == HIGH) {
        delay(tmpBotao);
      }
      comecar = true;
    }
    while (!pressionouAlgo) {
      // Não pressionou nada e acabou o tempo do menu
      if ((unsigned long) (millis() - momEntrouMenu) >= tmpMenuTimeout) {
        sairMenu = true;
        pressionouAlgo = true;
      }
      // Pressionou partida/parada (seta acima)
      if (digitalRead(pButPartPar) == HIGH) {
        // Sinaliza que pressionou alguma tecla
        pressionouAlgo = true;
        // Vai pra opção anterior
        if (opcaoAtual > 0) opcaoAtual--;
      }
      // Pressionou comuta contatoras (seta abaixo)
      if (digitalRead(pButRPGG) == HIGH) {
        // Sinaliza que pressionou alguma tecla
        pressionouAlgo = true;
        // Vai pra próxima opção
        if (opcaoAtual < 12) opcaoAtual++;
      }
      // Pressionou aut/man (seta direita)
      if (digitalRead(pButAM) == HIGH) {
        // Sinaliza que pressionou alguma tecla
        pressionouAlgo = true;
        unsigned long pressionou = millis();
        while (digitalRead(pButAM) == HIGH && millis() < pressionou + tmpAcess2aFunc) {
          delay(tmpBotao);
        }
        // Se ficou mais de 2 segundos pressionado, sai do menu
        if (millis() >= pressionou + tmpAcess2aFunc) {
          // Sai do menu
          sairMenu = true;
        } else {
          // Somente se o botão esquerdo não estiver ainda pressionado
          if (digitalRead(pButAM) == LOW) {
            switch (opcaoAtual) {
              case 0: if (valOpcaoAtual < 32) dataHoraByte[4]++;    break;
              case 1: if (valOpcaoAtual < 12) dataHoraByte[5]++;    break;
              case 2: if (valOpcaoAtual < 99) dataHoraByte[6]++;    break;
              case 3: if (valOpcaoAtual < 23) dataHoraByte[2]++;    break;
              case 4: if (valOpcaoAtual < 59) dataHoraByte[1]++;    break;
              case 5: esAtivo = (valOpcaoAtual == 0 ? 1 : 0);       break;
              case 6: if (valOpcaoAtual < 6) esDiaSemana++;         break;
              case 7: if (valOpcaoAtual < 23) esHora++;             break;
              case 8: if (valOpcaoAtual < 59) esMinuto++;           break;
              case 9: if (valOpcaoAtual < 180) esDurMin++;          break;
              case 10: esComCarga = (valOpcaoAtual == 0 ? 1 : 0);   break;
              case 11: if (horPermitInicio < 23) horPermitInicio++; break;
              case 12: if (horPermitFim < 23) horPermitFim++;       break;
            }
            // Grava dados
            if (opcaoAtual < 5) gravaRelogio();
            else if (opcaoAtual < 11) setExerSemanal();
            else setHorPermit();
          }
        }
      }
      // Pressionou reset (seta esquerda)
      if (digitalRead(pButRst) == HIGH) {
        // Sinaliza que pressionou alguma tecla
        pressionouAlgo = true;
        switch (opcaoAtual) {
          case 0: if (valOpcaoAtual > 1) dataHoraByte[4]--;    break;
          case 1: if (valOpcaoAtual > 1) dataHoraByte[5]--;    break;
          case 2: if (valOpcaoAtual > 1) dataHoraByte[6]--;    break;
          case 3: if (valOpcaoAtual > 0) dataHoraByte[2]--;    break;
          case 4: if (valOpcaoAtual > 0) dataHoraByte[1]--;    break;
          case 5: esAtivo = (valOpcaoAtual == 0 ? 1 : 0);      break;
          case 6: if (valOpcaoAtual > 0) esDiaSemana--;        break;
          case 7: if (valOpcaoAtual > 0) esHora--;             break;
          case 8: if (valOpcaoAtual > 0) esMinuto--;           break;
          case 9: if (valOpcaoAtual > 3) esDurMin--;           break;
          case 10: esComCarga = (valOpcaoAtual == 0 ? 1 : 0);  break;
          case 11: if (horPermitInicio > 0) horPermitInicio--; break;
          case 12: if (horPermitFim > 0) horPermitFim--;       break;
        }
        // Grava dados
        if (opcaoAtual < 5) gravaRelogio();
        else if (opcaoAtual < 11) setExerSemanal();
        else setHorPermit();
      }
    }
    // Se pressionou algo, processar no próximo loop
    if (digitalRead(pButAM) == HIGH && !sairMenu) {
      if (comecar) {
        pressionouAlgo = true;
      }
    } else {
      if (digitalRead(pButRst) == HIGH || digitalRead(pButPartPar) == HIGH || digitalRead(pButRPGG) == HIGH) {
        pressionouAlgo = true;
      }
    }
    // Se pressionou algo, aguardar (delay estava dentro do if)
    if (pressionouAlgo) {
      momEntrouMenu = millis();
    }
    delay(tmpPBMenu);
  }
}
