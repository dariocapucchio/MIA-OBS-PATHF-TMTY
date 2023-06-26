/*********************************************************************************
 *  @file main.cpp 
 *  @brief Telemetria MIA
 *            
 *          Hardware:
 *            Raspberry pi Pico
 *            ENC28J60 - Ethernet
 *            INA3221 - Sensor de tension y corriente
 *            DS18B20 - Sensor de temperatura
 *            ACS712 - Sensor de corriente por efecto Hall
 *
 *  @author Dario Capucchio
 *  @date 31-05-2023
 *  @version 0.1
 *
 *********************************************************************************/
/* ============= LIBRERIAS ===================================================== */
#include <Arduino.h>
#include <EthernetENC.h>
#include <PubSubClient.h>
#include <PID_v1.h>
#include "one_wire.h"
#include "ina3221.h"
/* ============= DEFINICIONES ================================================== */
// HARDWARE
#define DO1_PIN       6   // Salida digital 1 - GPIO 6 (pico pin 9)
#define DO2_PIN       7   // Salida digital 2 - GPIO 7 (pico pin 10)
#define DO3_PIN       8   // Salida digital 3 - GPIO 8 (pico pin 11)
#define V_OUT_PIN    20   // Salida digital fuente - GPIO 20 (pico pin 26)
//#define ONE_WIRE_PIN 21   // Sensores de temperatura - GPIO 21 (pico pin 27)
#define ONE_WIRE_PIN 14   // Sensores de temperatura - GPIO 14 (pico pin 19)
#define PWM_PIN      22   // Salida PWM - GPIO 22 (pico pin 29)
#define LED1_PIN     27   // LED - GPIO 27 (pico pin 32)
#define LED2_PIN     28   // LED - GPIO 28 (pico pin 34)
#define DS_DEVICE_DISCONNECTED -1000 // Valor de error del sensor
#define PWM_MAX_VALUE 255
#define DELAY_MQTT 3000   // Tiempo de espera entre publicaciones
#define CLIENT_ID "MIA_TMTY_01"  // ID del cliente (esta placa)

// FUNCIONES

float medirTemp(float temp, rom_address_t addr);
void medirTodo(void);
void callback(char *topic, byte *payload, unsigned int length); // Funcion para la recepcion via MQTT
void reconnect(void);                                           // Reconectar al servidor mqtt
void enviarDatosMQTT (void);
void pinToggle(int pin);
void imprimirTodo(void);

// DEFINICIONES PARA LA CONEXION ETHERNET CON ENC28J60
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xAE, 0xAF};  // Dirección MAC del módulo Ethernet
IPAddress server(163, 10, 43, 68);                  // IP del broker MQTT
EthernetClient client;                              // Cliente Ethernet
PubSubClient mqttClient(client);                    // Cliente mqtt

long previousMillis;  // Variable para contar el tiempo entre publicaciones
//bool flag_mqtt;       // Flag para reconocer la recepcion por mqtt
uint16_t dato_mqtt;   // Dato recibido por mqtt

// DEFINICIÓN DE SENSORES DE TEMPERTURA DS18B20
One_wire one_wire(ONE_WIRE_PIN);
rom_address_t address_null{};
rom_address_t address1 = { 0x28, 0x4A, 0x93, 0xCA, 0x0A, 0x00, 0x00, 0x15 };  // Dirección del sensor 1
rom_address_t address2 = { 0x28, 0x10, 0x8F, 0xCA, 0x0A, 0x00, 0x00, 0xA9 };  // Dirección del sensor 2
rom_address_t address3 = { 0x28, 0xCE, 0x17, 0xCC, 0x0A, 0x00, 0x00, 0xFE };  // Dirección del sensor 3
rom_address_t address4 = { 0x28, 0x6D, 0xD1, 0xCC, 0x0A, 0x00, 0x00, 0x45 };  // Dirección del sensor 4
rom_address_t address5 = { 0x28, 0x7B, 0xAF, 0xCA, 0x0A, 0x00, 0x00, 0x6A };  // Dirección del sensor 5
//rom_address_t address6 = { 0x28, 0x08, 0x82, 0x96, 0xF0, 0x01, 0x3C, 0x4B };  // Dirección del sensor 6
//rom_address_t address6 = { 0x28, 0x7C, 0xA8, 0x96, 0xF0, 0x01, 0x3C, 0x4A };  // Dirección del sensor 6
rom_address_t address6 = { 0x28, 0x51, 0xA5, 0x49, 0xF6, 0xDF, 0x3C, 0x94 };

float temp1;  // Temperatura lado caliente
float temp2;  // Temperatura lado frio
float temp3;  // Temperatura ambiente 1
float temp4;  // Temperatura ambiente 2
float temp5;  // Temperatura ambiente 3
float temp6;  // Temperatura ambiente 4

uint8_t ds_err;   // Contador para las señales con error

// DEFINICION DE VARIABLES PARA EL PID
double Setpoint, Input, Output;
double Kp = 20.0, Ki = 5.0, Kd = 1.0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
uint16_t cuentaPWM = 0;

// DEFINICION DE VARIABLES PARA ACS712
// Utiliza la entrada analogica
float I = 0.0;
float Iant = 0.0;
uint8_t n = 0;
float Sensibilidad = 0.066;  // Sensibilidad en V/A para sensor de 30A

uint16_t offsetI = 1604;
double spanI = -0.032;

// DEFINICION DE OBJETO PARA SENSOR DE CORRIENTE INA3221
INA3221 ina_0(INA3221_ADDR40_GND);  // Direccion 0x40 (A0 pin -> GND)
INA3221 ina_1(INA3221_ADDR41_VCC);  // Direccion 0x41 (A0 pin -> VCC)

float ina0_i1, ina0_v1; // Corriente y tension del amplificador 1
float ina0_i2, ina0_v2; // Corriente y tension del amplificador 2
float ina1_i1, ina1_v1; // Corriente y tension del amplificador 1
float ina1_i2, ina1_v2; // Corriente y tension del amplificador 2
float ina0_i1_span = 1.00, ina0_v1_span = 1.00;
float ina0_i2_span = 1.00, ina0_v2_span = 1.00;
float ina1_i1_span = 1.00, ina1_v1_span = 1.00;
float ina1_i2_span = 1.00, ina1_v2_span = 1.00;

// VARIABLES DE FLUJO DE PROGRAMA

char comando;         // Comando para la maquina de estados
bool flag_comando;    // Alerta nuevo comado
/* ============= SETUP CORE 0 ================================================== */
void setup() {
  Serial.begin(115200);   // Comunicacion serie con la PC - USB
  // Configuracion de entradas y salidas
  pinMode(V_OUT_PIN, OUTPUT);     // Pin Vout como salida
  pinMode(DO1_PIN, OUTPUT);       // Pin salida digital 1 como salida
  pinMode(DO2_PIN, OUTPUT);       // Pin salida digital 2 como salida
  pinMode(DO3_PIN, OUTPUT);       // Pin salida digital 3 como salida
  pinMode(LED1_PIN, OUTPUT);      // Pin led como salida
  pinMode(LED2_PIN, OUTPUT);      // Pin led como salida
  pinMode(PWM_PIN, OUTPUT);       // Pin PWM como salida
  pinMode(A0, INPUT);             // Pin A0 como entrada analogica
  analogReadResolution(12);       // con resolicion de 12 bit
  digitalWrite(V_OUT_PIN, LOW);   // Vout OFF
  digitalWrite(DO1_PIN, LOW);     // DO1 OFF
  digitalWrite(DO2_PIN, LOW);     // DO2 OFF
  digitalWrite(DO3_PIN, LOW);     // DO3 OFF
  digitalWrite(LED1_PIN, LOW);    // LED OFF
  digitalWrite(LED2_PIN, LOW);    // LED OFF
  analogWrite(PWM_PIN, PWM_MAX_VALUE);    // PWM en alto - celda OFF
  myPID.SetMode(AUTOMATIC);        // PID ON
  // Espero a que se aprete una tecla para poder verificar por puerto
  // serie la conexion al broker mqtt. Despues se comenta
  /*while(!Serial.available()){
    Serial.println("-> Apreta una tecla cualquiera");
    delay(500);
  }*/
  // Primer mensaje
  Serial.println("----------------------------------------");
  Serial.println("- MIA PATHFINDER - IAR ");
  Serial.println("- Placa telemetria ");
  Serial.println("----------------------------------------");
  // Inicio comunicacion oneWire
  Serial.print("-> Iniciando comunicacion one wire . . . ");
  one_wire.init();
  Serial.println("OK");
  // Inicio INA3221
  Serial.print("-> Iniciando comunicacion I2C con modulos INA3221 . . . ");
  ina_0.begin(&Wire);
  ina_0.reset();
  ina_0.setShuntRes(100, 100, 100);
  ina_0.setFilterRes(10, 10, 10);
  ina_1.begin(&Wire);
  ina_1.reset();
  ina_1.setShuntRes(100, 100, 100);
  ina_1.setFilterRes(10, 10, 10);
  Serial.println("OK");
  
  // Inicio modulo Ethernet
  Serial.print("-> Inicializando modulo Ethernet.... ");
  Serial.println(Ethernet.begin(mac));
  if (Ethernet.hardwareStatus() == EthernetNoHardware)
  { // Verifico la comunicacion SPI con el modulo
    Serial.println("--> No hay conexión con el módulo de Ethernet :(");
  }
  if (Ethernet.linkStatus() == LinkOFF)
  { // Verifico si esta el cable de Ethernet conectado
    Serial.println("--> No está conectado el cable de Ethernet!");
  }
  Serial.print("-> IP local: "); // Verifico la direccion de ip
  Serial.println(Ethernet.localIP());
  Serial.print("-> connecting to host...");
  while (!client.connect(server, 1883))
  {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("-> connectado!");
  mqttClient.setServer(server, 1883); // Servidor MQTT
  mqttClient.setCallback(callback);   // Callback para la recepcion via MQTT
  Serial.print("-> Conexion al broker MQTT = ");
  Serial.println(mqttClient.connect(CLIENT_ID)); // Conexion al broker MQTT
  if (!client.connected())
  {
    reconnect();
  }
  // Suscripciones MQTT
  mqttClient.subscribe("control/comando");
  mqttClient.subscribe("control/pid");
  mqttClient.subscribe("control/digital");
  mqttClient.subscribe("servicio/fallas_ds_reset");
  mqttClient.subscribe("servicio/hw_reset_response");
  mqttClient.subscribe("servicio/power_cal");

  mqttClient.publish("servicio/hw_reset","R");
  mqttClient.loop();

  // Inicio variables
  ds_err = 0;
  Setpoint = 20.0;
  flag_comando = false;
  previousMillis = millis();
}
/* ============= LOOP CORE 0 =================================================== */
void loop() {
  /*
  if (Serial.available() > 0) {  // Si llego algo
    comando = Serial.read();     // Leo dato por el puerto serie
    flag_comando = true;         // Recuerdo que llego algo
  }
  */
  switch (comando) {
    case 'A':   // Si llego una A -- modo automatico PID
      if (flag_comando == true) {                        // Ejecuto esto una sola vez
        digitalWrite(LED1_PIN, HIGH);       // LED ON
        Serial.println("-> PID ON");
        mqttClient.publish("estado", "PID ON");  // Publico el estado
        flag_comando = false;               // Reinicio el flag
      }
      one_wire.convert_temperature(address2,true,false);        // Leo temparatura del sensor
      Input = medirTemp(temp2, address2);   // Temperatura del lado frio a la entrada del PID
      myPID.Compute();                      // Computo del PID
      analogWrite(PWM_PIN, Output);         // Salida a la celda
      break;
    case 'M':  // Si llego una M -- modo manual
      if (flag_comando == true) {
        digitalWrite(LED1_PIN, HIGH);         // LED ON
        Serial.println("-> MANUAL ON");
        mqttClient.publish("estado", "MANUAL");  // Publico el estado
        flag_comando = false;                            // Reinicio el flag
      }
      analogWrite(PWM_PIN, 255-cuentaPWM);  // PWM
      break;
    case 'X':  // Si llego una X -- parada
      if (flag_comando == true) {
        digitalWrite(LED1_PIN, LOW);       // LED OFF
        Serial.println("-> CELDA OFF");
        analogWrite(PWM_PIN,PWM_MAX_VALUE);    // PWM en alto - celda OFF
        mqttClient.publish("estado", "PRADO");  // Publico el estado
        flag_comando = false;                           // Reinicio el flag
        Iant=0.0;
        I=0.0;        
      }
      break;
    default: // OTRO CARACTER
      //Serial.println("-> Comando incorrecto :(");
      break;
  }

  I = Iant + ((analogRead(A0) - offsetI) * spanI);  // Conversion a corriente
  n++;                                              // Cuento las mediciones
  Iant = I;                                         // Guardo el valor
  
  if (millis() - previousMillis > DELAY_MQTT) {  // Envio todo al broker cada DELAY_MQTT

    Serial.println("mide --> ");
    medirTodo();
    //imprimirTodo();
    
    Serial.print("-> MQTT ");
    enviarDatosMQTT();
    Serial.println("enviado :)");
    previousMillis = millis();
    // Toggle led - Alive test
    pinToggle(LED2_PIN);
  }
  
  if (!client.connected()) {
    Serial.println("reconectando...");
    reconnect();
  }
  mqttClient.loop();
  
  delay(100);
}
/* ============= FUNCIONES ===================================================== */

/**
 * @brief Mide los valores de todos los sensores conectados
*/
void medirTodo(void)
{
  one_wire.convert_temperature(address_null, true, true); // Convierten todos
  delay(100);
  temp1 = medirTemp(temp1, address1);  // Temperatura lado caliente
  temp2 = medirTemp(temp2, address2);  // Temperatura lado frio
  temp3 = medirTemp(temp3, address3);  // Temperatura ambiente 1
  temp4 = medirTemp(temp4, address4);  // Temperatura ambiente 2
  temp5 = medirTemp(temp5, address5);  // Temperatura ambiente 3
  temp6 = medirTemp(temp6, address6);  // Temperatura ambiente 4

  //I = (analogRead(A0) - offsetI) * spanI;   // Lectura ADC y conversion a corriente
  I = Iant / (n + 1);                       // Promedio los valores de corriente
  n = 0;                                    // Reinicio la cuenta
  Iant = I;                                 // Guardo el promedio

  ina0_i1 = ina_0.getCurrent(INA3221_CH1) * ina0_i1_span;   // Coriente amplificador 1
  ina0_v1 = ina_0.getVoltage(INA3221_CH1) * ina0_v1_span;   // Tension amplificador 1
  ina0_i2 = ina_0.getCurrent(INA3221_CH2) * ina0_i2_span;   // Coriente amplificador 2
  ina0_v2 = ina_0.getVoltage(INA3221_CH2) * ina0_v2_span;   // Coriente amplificador 2
  ina1_i1 = ina_1.getCurrent(INA3221_CH1) * ina1_i1_span;   // Coriente amplificador 3
  ina1_v1 = ina_1.getVoltage(INA3221_CH1) * ina1_v1_span;   // Coriente amplificador 3
  ina1_i2 = ina_1.getCurrent(INA3221_CH2) * ina1_i2_span;   // Coriente amplificador 4
  ina1_v2 = ina_1.getVoltage(INA3221_CH2) * ina1_v2_span;   // Coriente amplificador 4
}

/**
 * @brief Imprime por puerto serie todos los valores medidos
*/
void imprimirTodo(void)
{
  // Linea 1 - Sensore de temperatura DS18B20
  Serial.print("T1: ");
  Serial.print(temp1);
  Serial.print("C - T2: ");
  Serial.print(temp2);
  Serial.print("C - T3: ");
  Serial.print(temp3);
  Serial.print("C - T4: ");
  Serial.print(temp4);
  Serial.print("C - T5: ");
  Serial.print(temp5);
  Serial.print("C - T6: ");
  Serial.print(temp6);
  Serial.println("C");
  // Linea 2 - Corriente de la celda de peltier
  Serial.print("I: ");
  Serial.print(I);
  Serial.println("A");
  // Linea 3 - Sensor INA0
  Serial.print("INA0 I1: ");
  Serial.print(ina0_i1);
  Serial.print("A - INA0 V1: ");
  Serial.print(ina0_v1);
  Serial.print("V - INA0 I2: ");
  Serial.print(ina0_i2);
  Serial.print("A - INA0 V2: ");
  Serial.print(ina0_v2);
  Serial.println("V");
  // Linea 4 - Sensor INA1
  Serial.print("INA1 I1: ");
  Serial.print(ina1_i1);
  Serial.print("A - INA1 V1: ");
  Serial.print(ina1_v1);
  Serial.print("V - INA1 I2: ");
  Serial.print(ina1_i2);
  Serial.print("A - INA1 V2: ");
  Serial.print(ina1_v2);
  Serial.println("V");
}

/**
 * @brief Medicion de temperatura con sensor DS18B20
 * 
 * @param temp variable con el valor anterior de temperatura
 * @param addr direccion del sensor de temperatura 
 * @returns el valor de temperatura en float, o el valor anterior 
 * provisto por la variable temp en caso de error
 * 
 * @note Incrementa la variable ds_err en 1 si detecta un error, y
 * lo publica por mqtt
*/
float medirTemp(float temp, rom_address_t addr)
{
  float aux = one_wire.temperature(addr);     // Tomo el valor del sensor
  delay(100);
  if (aux == DS_DEVICE_DISCONNECTED) {        // Si es un valor erroneo
    Serial.println("DS - error de lectura");
    ds_err++;                                 // incremento la cuenta de errores
    /*
    char dato[4];
    sprintf(dato,"%d",ds_err);
    mqttClient.publish("servicio/fallas_ds", dato);  // Publico el estado
    */
    return temp;                              // Devuelvo el valor anterior
  } else {                                    // Si el valor no es erroneo
    return aux;                               // Devuelvo el valor obtenido
  }
}

/**
 * @brief Calback MQTT
 * 
 * @param topic : topico mqtt
 * @param payload : payload mqtt
 * @param length : longitud del payload
*/
void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("-> Mensaje en topico [");
  Serial.print(topic);
  Serial.print("] : ");
  String topic_str(topic); // Topico a String
  String string_aux;       // Payload a String
  for (unsigned int i = 1; i < length; i++)
  {
    string_aux += (char)payload[i];
  }
  if (topic_str == "control/comando") {
    if ((char)payload[0] == 'W') {
      cuentaPWM = string_aux.toInt();
      Serial.print("PWM = ");
      Serial.print(cuentaPWM);
    } else {
      comando = (char)payload[0];
      Serial.print(comando);
      flag_comando = true;  // Recuerdo que llego un comando
    }
  } else if (topic_str == "control/pid") {
    switch ((char)payload[0]) {
      case 'P':
        Kp = string_aux.toDouble();
        Serial.print("Kp = ");
        Serial.print(Kp);
        break;
      case 'I':
        Ki = string_aux.toDouble();
        Serial.print("Ki = ");
        Serial.print(Ki);
        break;
      case 'D':
        Kd = string_aux.toDouble();
        Serial.print("Kd = ");
        Serial.print(Kd);
        break;
      case 'S':
        Setpoint = string_aux.toDouble();
        Serial.print("Setpoint = ");
        Serial.print(Setpoint);
        break;
    }  // Fin switch payload PID
  } else if (topic_str == "control/digital") {
    switch ((char)payload[0]) {
      case 'v':
        if ((char)payload[1] == 't') {
          digitalWrite(V_OUT_PIN, HIGH);  // Vout ON
          Serial.print("Vout ON");
        } else {
          digitalWrite(V_OUT_PIN, LOW);  // Vout OFF
          Serial.print("Vout OFF");
        }
        break;
      case '1':
        if ((char)payload[1] == 't') {
          digitalWrite(DO1_PIN, HIGH);  // Vout ON
          Serial.print("DO1 ON");
        } else {
          digitalWrite(DO1_PIN, LOW);  // Vout OFF
          Serial.print("DO1 OFF");
        }
        break;
      case '2':
        if ((char)payload[1] == 't') {
          digitalWrite(DO2_PIN, HIGH);  // Vout ON
          Serial.print("DO2 ON");
        } else {
          digitalWrite(DO2_PIN, LOW);  // Vout OFF
          Serial.print("DO2 OFF");
        }
        break;
      case '3':
        if ((char)payload[1] == 't') {
          digitalWrite(DO3_PIN, HIGH);  // Vout ON
          Serial.print("DO3 ON");
        } else {
          digitalWrite(DO3_PIN, LOW);  // Vout OFF
          Serial.print("DO3 OFF");
        }
        break;
    }  // Fin switch payload digital
  } else {
    Serial.print("topico desconocido :(");
    mqttClient.publish("alertas","topico desconocido");
  }
  /*comando = (char)payload[0];
  dato_mqtt = string_aux.toFloat() * 100;
  Serial.print(comando);
  Serial.print(dato_mqtt);
  flag_comando = true;*/
  Serial.println();
}

/**
 * @brief Reconexion MQTT
*/
void reconnect()
{
  // Loop until we're reconnected
  while (!mqttClient.connected())
  {
    Serial.print("--> Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect(CLIENT_ID))
    {
      Serial.println("--> connected");
    }
    else
    {
      Serial.print("--> failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


/**
 * @brief Envia todos los datos por MQTT
*/
void enviarDatosMQTT (void)
{
  if (!client.connected()) {
    Serial.println("-> reconectando...");
    reconnect();
  }

  char dato[6];
  sprintf(dato,"%.2f",temp1);
  mqttClient.publish("medicion/temperatura", dato);
  sprintf(dato,"%.2f",temp2);
  mqttClient.publish("medicion/temperatura", dato);
  sprintf(dato,"%.2f",temp3);
  mqttClient.publish("medicion/temperatura", dato);
  sprintf(dato,"%.2f",temp4);
  mqttClient.publish("medicion/temperatura", dato);
  sprintf(dato,"%.2f",temp5);
  mqttClient.publish("medicion/temperatura", dato);
  sprintf(dato,"%.2f",temp6);
  mqttClient.publish("medicion/temperatura", dato);
  sprintf(dato,"%.2f",I);
  mqttClient.publish("medicion/i_celda", dato);

  sprintf(dato,"%d",ds_err);                        // Cantidad de fallas
  mqttClient.publish("servicio/fallas_ds", dato);   // Publico el estado

  sprintf(dato,"%.2f",ina0_i1);
  mqttClient.publish("medicion/RF", dato);
  sprintf(dato,"%.2f",ina0_v1);
  mqttClient.publish("medicion/RF", dato);
  sprintf(dato,"%.2f",ina0_i2);
  mqttClient.publish("medicion/RF", dato);
  sprintf(dato,"%.2f",ina0_v2);
  mqttClient.publish("medicion/RF", dato);
  sprintf(dato,"%.2f",ina1_i1);
  mqttClient.publish("medicion/RF", dato);
  sprintf(dato,"%.2f",ina1_v1);
  mqttClient.publish("medicion/RF", dato);
  sprintf(dato,"%.2f",ina1_i2);
  mqttClient.publish("medicion/RF", dato);
  sprintf(dato,"%.2f",ina1_v2);
  mqttClient.publish("medicion/RF", dato);
}

/**
 * @brief Cambia el estado de un pin
 * 
 * @param pin numero de pin a cambiar de estado
*/
void pinToggle(int pin)
{
  (digitalRead(pin)) ? digitalWrite(pin, LOW) : digitalWrite(pin, HIGH);
}
/* ============================================================================= */
// EOF