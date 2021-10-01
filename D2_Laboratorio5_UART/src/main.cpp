//******************************************************************************************
//Universidad del Valle de Guatemala
//BE3015: Electronica Digital 2
//Estefany Eleuteria Batz Cantor
//Laboratorio 5.UART
//******************************************************************************************

//******************************************************************************************
//Librerias
//******************************************************************************************
#include <Arduino.h>
#include <LiquidCrystal.h>

//******************************************************************************************
//Definicion de los pines
//******************************************************************************************
//Potenciometros
#define Poten1 35
#define Poten2 34

#define LedR 18
#define LedG 17
#define LedB 16

//Senal PWM de los Leds
#define PWMLedR 1 //Salida en GPIO 18
#define PWMLedG 2
#define PWMLedB 3

#define resolution 8 //Resolucion

#define frequPWMLed 5000 //frecuencia de los leds

//Pantalla LCD
#define RS 32
#define E 33
#define D4 25
#define D5 26
#define D6 27
#define D7 14

//******************************************************************************************
//Prototipos de funciones
//******************************************************************************************
void Potenciometros(void);
void PantallLCD(void);
void ValoresPotenciometros(void);
void ConfiguracionPWM(void); //PWM de los Leds

//******************************************************************************************
//Variables globales
//******************************************************************************************
//Para los potenciometros
float ADCPoten1 = 0.0; //Recibe el valor ADC
double VoltajePoten1 = 0;

float ADCPoten2 = 0.0; //Recibe el valor ADC
double VoltajePoten2 = 0;

double alpha = 0.09; // Filtro para estabilizar

int DCLedR = 0;
int DCLedG = 0;

LiquidCrystal LCD(RS, E, D4, D5, D6, D7);
uint8_t P1Centena, P1Decena, P1Unidad;
uint8_t P2Centena, P2Decena, P2Unidad;
//******************************************************************************************
//Configuraciones
//******************************************************************************************
void setup()
{
  Serial.begin(9600);
  Potenciometros();
  ConfiguracionPWM();

  pinMode(LedR, OUTPUT);
  pinMode(LedG, OUTPUT);
  pinMode(LedB, OUTPUT);

  LCD.begin(16, 2);
}

//******************************************************************************************
//Loop principal
//******************************************************************************************
void loop()
{
  Potenciometros();
  ValoresPotenciometros();
  LCD.setCursor(0, 0);
  LCD.print("Rojo");
  LCD.setCursor(0, 1);
  LCD.print(P1Centena);
  LCD.print(P1Decena);
  LCD.print(P1Unidad);

  LCD.setCursor(6, 0);
  LCD.print("Verde");
  LCD.setCursor(7, 1);
  LCD.print(P2Centena);
  LCD.print(P2Decena);
  LCD.print(P2Unidad);

  LCD.setCursor(12, 0);
  LCD.print("Azul");
  delay(100);
  LCD.setCursor(12, 1);

  ledcWrite(PWMLedR, DCLedR);
  ledcWrite(PWMLedG, DCLedG);
  ledcWrite(PWMLedB, 0);
}

//******************************************************************************************
//Potenciometros
//******************************************************************************************
void Potenciometros(void)
{
  ADCPoten1 = (analogReadMilliVolts(Poten1) * 255) / 3150;
  VoltajePoten1 = (alpha * ADCPoten1) + ((1.0 - alpha) * VoltajePoten1); //Filtro pasa bajas
  Serial.print("Voltaje potenciometro 1: ");
  Serial.println(VoltajePoten1);

  ADCPoten2 = (analogReadMilliVolts(Poten2) * 255) / 3150;
  VoltajePoten2 = (alpha * ADCPoten2) + ((1.0 - alpha) * VoltajePoten2); //Filtro pasa bajas
  Serial.print("Voltaje potenciometro 2: ");
  Serial.println(VoltajePoten2);
}

//******************************************************************************************
//Valores Potenciometros
//******************************************************************************************
void ValoresPotenciometros(void)
{
  DCLedR = VoltajePoten1;
  P1Centena = VoltajePoten1 / 100;
  P1Decena = (VoltajePoten1 - (P1Centena * 100)) / 10;
  P1Unidad = (VoltajePoten1 - P1Centena * 100) - (P1Decena * 10);

  /*Serial.print("P1 Centena");
  Serial.println(P1Centena);
  Serial.print("P1 Decena");
  Serial.println(P1Decena);
  Serial.print("P1 Unidad");
  Serial.println(P1Unidad);*/

  DCLedG = VoltajePoten2;
  P2Centena = VoltajePoten2 / 100;
  P2Decena = (VoltajePoten2 - (P2Centena * 100)) / 10;
  P2Unidad = (VoltajePoten2 - P2Centena * 100) - (P2Decena * 10);
}

//******************************************************************************************
//Valores Potenciometros
//******************************************************************************************
void ConfiguracionPWM(void)
{
  ledcSetup(PWMLedR, frequPWMLed, resolution);
  ledcAttachPin(LedR, PWMLedR);

  ledcSetup(PWMLedG, frequPWMLed, resolution);
  ledcAttachPin(LedG, PWMLedG);

  ledcSetup(PWMLedB, frequPWMLed, resolution);
  ledcAttachPin(LedB, PWMLedB);
}