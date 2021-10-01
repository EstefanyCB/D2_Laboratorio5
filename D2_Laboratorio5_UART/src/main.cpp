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
#define Poten1 35
#define Poten2 34

#define LedR 18
#define LedG 17
#define LedB 16

#define RS 25
#define E 26
#define D4 27
#define D5 14
#define D6 12
#define D7 13
//******************************************************************************************
//Prototipos de funciones
//******************************************************************************************
void Potenciometros(void);
void PantallLCD(void);
void ValoresPotenciometros(void);
//******************************************************************************************
//Variables globales
//******************************************************************************************
//Para los potenciometros
float ADCPoten1 = 0.0; //Recibe el valor ADC
double VoltajePoten1 = 0;

float ADCPoten2 = 0.0; //Recibe el valor ADC
double VoltajePoten2 = 0;

double alpha = 0.09; // Filtro para estabilizar

LiquidCrystal LCD(RS, E, D4, D5, D6, D7);
uint8_t P1Centena, P1Decena, P1Unidad;
uint8_t P2Centena, P3Decena, P4Unidad;
//******************************************************************************************
//Configuraciones
//******************************************************************************************
void setup()
{
  Serial.begin(115200);
  Potenciometros();

  LCD.begin(16, 2);
}

//******************************************************************************************
//Loop principal
//******************************************************************************************
void loop()
{
  Potenciometros();
  ValoresPotenciometros();
  delay(500);
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
  P1Centena = VoltajePoten1 / 100;
  P1Decena = (VoltajePoten1 - (P1Centena * 100)) / 10;
  P1Unidad = (VoltajePoten1 - P1Centena * 100) - (P1Decena * 10);
  Serial.print("P1 Centena");
  Serial.println(P1Centena);
  Serial.print("P1 Decena");
  Serial.println(P1Decena);
  Serial.print("P1 Unidad");
  Serial.println(P1Unidad);
}