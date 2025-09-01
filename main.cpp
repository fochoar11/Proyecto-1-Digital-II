/******************************************/
// Universidad del Valle de Guatemala
// BE3029 - Electronica Digital 2
// Javier Ochoa
// 01/09/2025
// Proyecto 1
// MCU: ESP32 - WROOM-32
/******************************************/

/******************************************/
// Librerias
/******************************************/
#include <Arduino.h>
#include <stdint.h> 
#include <driver/ledc.h>
#include "config.h"           
#include "AdafruitIO_WiFi.h"  

/******************************************/
// Definiciones
/******************************************/
//pines de cada componente
#define sensor_pin 34
#define bot1 23
#define led_v 26
#define led_a 27
#define led_r 14
#define servo_pin   12
#define COMMON_CATHODE 1 //indicamos que el display es catodo comun
//definimos los pines del display y los pines del transistor para el multiplexeo
const int dA  = 2;
const int dB  = 4;
const int dF  = 22;
const int dG  = 13;
const int dE  = 5; 
const int dD  = 18;
const int dC  = 19;
const int dP  = 21;
const int dig1 = 33;
const int dig2 = 25;
const int dig3 = 32;
// valor logico que va tomar la logica dependiendo si va apagar o encender cada segmente o digito del transistor
#define seg_on   (COMMON_CATHODE ? HIGH : LOW)
#define seg_off  (COMMON_CATHODE ? LOW  : HIGH)
#define dig_on   (COMMON_CATHODE ? HIGH : LOW)
#define dig_off  (COMMON_CATHODE ? LOW  : HIGH)

//pwm de servo
const int ch_servo = 3;
const int servo_freq  = 50; //señal pwm del servo que puede ser equivalente a un pulso cada 20ms
const int servo_res = 16; //resolucion de 16 bits para el duty cicle para que no haga saltos muy grandes
const int servo_min_us = 500; //puede corresponer a 0 grados
const int servo_max_us = 2500; //puede corresponer a un angulo cercano a 180

// Adafruit IO
#define IO_LOOP_DELAY 5000UL // le puse eso porque equivale a 5 segundos y la pagina se me satura

/******************************************/
// Prototipos de funciones
/******************************************/
static inline uint32_t usToDuty(int microseconds); //convertimos el ancho de pulso en microsegundos al valor que espera el pwm
static inline void writeServoAngle(float angle_deg); //traduce el angulo a un pulso 
static inline void setSegments(bool a,bool b,bool c,bool d,bool e,bool f,bool g,bool dp);//enciende y apaga los segmentos 
static inline void drawDigit(uint8_t val, bool dp);//para escribir el numero en el display del 0 al 9 
static inline void allDigitsOff(void);//apaga los 3 digitos en el multiplexeo
void IRAM_ATTR displayISR(void); //interrupcion para apagar digitos y eleigr el siguiente

static void  updateDisplayFromTemp(float t);//toma la temperatura y la convierte a 3 digitos
static void  initDisplayPins(void);
static void  initDisplayTimer(void);//cambia el timer y junta la ISR
static float readTempRawC(void);//para leer el ADC y lo convierte a mv para deespues a temperatura
static float readTempFilteredC(void);//aplica el filtrado con las restrcciones que le coloque

/******************************************/
// Variables globales
/******************************************/
AdafruitIO_Feed* feed_proyecto = io.feed("Proyecto");//es la comunicacion con el dashboard que creamos
unsigned long lastUpdate = 0;//guarda el ultimo datos y se compara con el io_loop_delay 

int   sensor_val   = 0;//valor en crudo del adc
float temp         = 0.0f;//temperatura en grados celcius
float temp_disp    = 0.0f;//temperatura redondeada a 1 decimal por los displays
int   bot_state    = 0;//estado actual del boton
int   last_bot_state = 0;//el estado anterior 
unsigned long last_time = 0;
unsigned long deb_delay = 50;//tiempo de antirrebote
//varianles que tienen el duty cicle que cambian segun el rango de temperatura asignado
int led_v_pwm = 0;
int led_a_pwm = 0;
int led_r_pwm = 0;

// Display multiplexado
volatile uint8_t disp_digits[3] = { 0, 0, 0 };//el arreglo de 3 digitos para los displays
volatile bool disp_dp[3] = { false, false, false };//control de punto decimal
hw_timer_t* dispTimer = nullptr;
portMUX_TYPE dispMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint8_t currentDigit   = 0;

// Filtro de temperatura
static bool  filt_init = false;//para verificar que el filtro empezo o no
static float temp_filt = 0.0f;//guarda le temperatura fitlrada actual
static const float ALPHA        = 0.2f;//constante alpha del suavizado de filtro exponencial
static const float MAX_STEP     = 1.5f;//la temperatura no puede variar mas de 1.5 grados
// la diferencia de temperatura entre la cruda y filtrada es mas de 20 se ignora 
static const float HARD_OUTLIER = 20.0f;
static const int   NSAMPLES     = 8;//numero de temperaturas que se promedian
static const float OFFSET_C     = 2.0f;//offset pequeño para compensar un corrimiento que lee mi sensor

/******************************************/
// ISRs Rutinas de Interrupcion
/******************************************/
void IRAM_ATTR displayISR() {
  portENTER_CRITICAL_ISR(&dispMux);//bloquea las variables compartidas para evitar mal contacto con la ISR
  allDigitsOff();//todos los digitos apagados
  currentDigit = (currentDigit + 1) % 3;//para quedar en el siguiente numero de forma ciclica
  uint8_t val = disp_digits[currentDigit];
  bool    dp  = disp_dp[currentDigit];
  drawDigit(val, dp);
  switch (currentDigit) { //enciende solo el digito que es
    case 0: digitalWrite(dig1, dig_on); break;
    case 1: digitalWrite(dig2, dig_on); break;
    case 2: digitalWrite(dig3, dig_on); break;
  }
  portEXIT_CRITICAL_ISR(&dispMux);//con esto ya pueden trabajr las otras variables
}

/******************************************/
// Configuracion
/******************************************/
void setup() {
  Serial.begin(115200); //iniciar la comunicacion serial
  Serial.println("Connecting to Adafruit IO...");

  io.connect();//conexion wifi con adafruit con el config.h
  while (io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.println(io.statusText());//imprime cuando ya este conectado

  pinMode(bot1, INPUT_PULLDOWN);//boton como entrada en pulldown

  //display
  initDisplayPins();//inicializa los segmentos y los transistores
  updateDisplayFromTemp(0.0f);//para que muestre 0.0 al inicio 
  initDisplayTimer();//configuracion del temporizador

  //ADC
  analogSetPinAttenuation(sensor_pin, ADC_11db);//nivel de atenuación en los canales para el rango del adc

  //leds pwm
  ledcSetup(0, 5000, 8); //canal independiente con 5000 hz para evitar parapadeos que se vea y 8 bits
  ledcAttachPin(led_v, 0);//conectar el canal pwm a la led que es
  ledcSetup(1, 5000, 8);  //canal independiente con 5000 hz para evitar parapadeos que se vea y 8 bits
  ledcAttachPin(led_a, 1);//conectar el canal pwm a la led que es
  ledcSetup(2, 5000, 8);  //canal independiente con 5000 hz para evitar parapadeos que se vea y 8 bits  
  ledcAttachPin(led_r, 2);//conectar el canal pwm a la led que es

  // Servo
  ledcSetup(ch_servo, servo_freq, servo_res);
  ledcAttachPin(servo_pin, ch_servo);
  writeServoAngle(0.0f);//posicion inicial del servo

  // empezar el  filtro con una lectura
  temp_filt = readTempFilteredC();
}

/******************************************/
// Loop Principal
/******************************************/
void loop() {
  io.run();

  const unsigned long now = millis();
  if (now - lastUpdate >= IO_LOOP_DELAY) {
    Serial.print("Sending.. ");
    Serial.println(temp_filt);
    feed_proyecto->save(temp_filt);
    lastUpdate = now;
  }

  int reading = digitalRead(bot1);
  if (reading != last_bot_state) {
    last_time = millis();
  }

  if ((millis() - last_time) > deb_delay) {
    if (reading != bot_state) {//si ya paso el tiempo del antirrebote el estado del boton cambia
      bot_state = reading;
      if (bot_state == HIGH) {
        float tC = readTempFilteredC();//temperatura en filtrado
        temp = tC;//guarda la lectura
        temp_disp = roundf(temp * 10.0f) / 10.0f;

        Serial.print("Temperatura : ");
        Serial.println(temp_disp, 1);
//condiciones del semaforo
        if (temp_disp < 22.0) {
          led_v_pwm = 255;  
          led_a_pwm = 0;   
          led_r_pwm = 0;
        } else if (temp_disp < 25.0) {
          led_v_pwm = 0;    
          led_a_pwm = 255; 
          led_r_pwm = 0;
        } else {
          led_v_pwm = 0;    
          led_a_pwm = 0;   
          led_r_pwm = 255;
        }
        ledcWrite(0, led_v_pwm);
        ledcWrite(1, led_a_pwm);
        ledcWrite(2, led_r_pwm);
//control de servo
        float angle = 90.0f;
        if (temp_disp < 22.0) angle = 180.0f;//empieza en 180 por conveniencia de la maqueta
        else if (temp_disp < 25.0) angle = 90.0f;
        else                       angle = 0.0f;
        writeServoAngle(angle);

        Serial.print("angulo servo: ");
        Serial.println(angle);

        updateDisplayFromTemp(temp_disp);
        delay(100);
      }
    }
  }
  last_bot_state = reading;
}

/******************************************/
// Otras funciones
/******************************************/
static inline uint32_t usToDuty(int microseconds) {
  const uint32_t period_us = 20000UL;
  const uint32_t maxDuty   = (1UL << servo_res) - 1UL;
  return (uint32_t)(((uint64_t)microseconds * maxDuty) / period_us);
}

static inline void writeServoAngle(float angle_deg) {
  if (angle_deg < 0)   angle_deg = 0;
  if (angle_deg > 180) angle_deg = 180;//limita el angulo entre 0 y 180
  int pulse_us = servo_min_us + (int)((servo_max_us - servo_min_us) * (angle_deg / 180.0f));//convierte el ancho de pulso en microsegundos
  ledcWrite(ch_servo, usToDuty(pulse_us));//convierte el ancho a duty cicle
}

static inline void setSegments(bool a,bool b,bool c,bool d,bool e,bool f,bool g,bool dp) {
  digitalWrite(dA, a ? seg_on : seg_off);
  digitalWrite(dB, b ? seg_on : seg_off);
  digitalWrite(dC, c ? seg_on : seg_off);
  digitalWrite(dD, d ? seg_on : seg_off);
  digitalWrite(dE, e ? seg_on : seg_off);
  digitalWrite(dF, f ? seg_on : seg_off);
  digitalWrite(dG, g ? seg_on : seg_off);
  digitalWrite(dP, dp? seg_on : seg_off);
}

static inline void drawDigit(uint8_t val, bool dp) {
  if (val == 0xFF) { setSegments(false,false,false,false,false,false,false, dp); return; }
  switch (val) {//"tabla de verdad"
    case 0: setSegments(true, true, true, true, true, true, false, dp); break;
    case 1: setSegments(false,true, true, false,false,false,false, dp); break;
    case 2: setSegments(true, true, false,true, true, false, true, dp); break;
    case 3: setSegments(true, true, true, true, false,false, true, dp); break;
    case 4: setSegments(false,true, true, false,false, true, true, dp); break;
    case 5: setSegments(true, false,true, true, false, true, true, dp); break;
    case 6: setSegments(true, false,true, true, true, true, true, dp); break;
    case 7: setSegments(true, true, true, false,false,false,false, dp); break;
    case 8: setSegments(true, true, true, true, true, true, true, dp); break;
    case 9: setSegments(true, true, true, true, false, true, true, dp); break;
    default:setSegments(false,false,false,false,false,false,false, dp); break;
  }
}

static inline void allDigitsOff() {
  digitalWrite(dig1, dig_off);
  digitalWrite(dig2, dig_off);
  digitalWrite(dig3, dig_off);
}

static void updateDisplayFromTemp(float t) {
  int t10 = (int)roundf(t * 10.0f);
  //limite la temperatura para impedir negativos o valores mayores a 3 digitos
  if (t10 < 0)   t10 = 0;
  if (t10 > 999) t10 = 999;

  int tens   = (t10 / 100) % 10;//centenas
  int ones   = (t10 / 10)  % 10;//decenas
  int tenths =  t10 % 10;//unidad

  portENTER_CRITICAL(&dispMux);
  disp_digits[0] = (t10 >= 100) ? tens : 0xFF;
  disp_digits[1] = ones;
  disp_digits[2] = tenths;
  disp_dp[0] = false;
  disp_dp[1] = true;
  disp_dp[2] = false;
  portEXIT_CRITICAL(&dispMux);
}

static void initDisplayPins() {
  pinMode(dA, OUTPUT); pinMode(dB, OUTPUT); pinMode(dC, OUTPUT); pinMode(dD, OUTPUT);
  pinMode(dE, OUTPUT); pinMode(dF, OUTPUT); pinMode(dG, OUTPUT); pinMode(dP, OUTPUT);
  pinMode(dig1, OUTPUT); pinMode(dig2, OUTPUT); pinMode(dig3, OUTPUT);
  setSegments(false,false,false,false,false,false,false,false);
  allDigitsOff();
}

static void initDisplayTimer() {
//timer
  dispTimer = timerBegin(0, 80, true);
  //asocia la ISR a el timer
  timerAttachInterrupt(dispTimer, &displayISR, true);
  timerAlarmWrite(dispTimer, 1000, true);
  timerAlarmEnable(dispTimer);
}

static float readTempRawC() {
  long acc = 0;
  for (int k = 0; k < NSAMPLES; ++k) {//para hacer las pruebas de promedio 
    acc += analogRead(sensor_pin);
    delayMicroseconds(1500);
  }
  float meanADC   = acc / (float)NSAMPLES;
  float millivolts= meanADC * (3300.0f / 4095.0f);//convierte el ADC a milivoltios
  float tC        = (millivolts / 10.0f) + OFFSET_C;//10 mv por cada grado c de temperaturea con el offset (a conveniencia)
  return tC;
}

static float readTempFilteredC() {
  float t_raw = readTempRawC();

  if (!filt_init) {
    temp_filt = t_raw;
    filt_init = true;
    return temp_filt;
  }

  float delta = t_raw - temp_filt;//calcula la diferencia con la temperatura cruda y filtrada
  if (fabsf(delta) > HARD_OUTLIER) {//si la diferencia es mas de la temperatura se considera outlier entonces se ignora
    return temp_filt;
  }
  if (delta >  MAX_STEP) t_raw = temp_filt + MAX_STEP;
  if (delta < -MAX_STEP) t_raw = temp_filt - MAX_STEP;//para que no de brincos de temperatura altos 

  temp_filt = (ALPHA * t_raw) + ((1.0f - ALPHA) * temp_filt);//filtro exponencial EMA
  return temp_filt;
}
