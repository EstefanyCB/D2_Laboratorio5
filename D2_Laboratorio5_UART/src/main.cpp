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

//******************************************************************************************
//Variables globales
//******************************************************************************************
//Para los potenciometros
float ADCPoten1 = 0.0; //Recibe el valor ADC
double VoltajePoten1 = 0;

float ADCPoten2 = 0.0; //Recibe el valor ADC
double VoltajePoten2 = 0;

double alpha = 0.09; // Filtro para estabilizar

//******************************************************************************************
//Configuraciones
//******************************************************************************************
void setup()
{
  Serial.begin(9600);
  Potenciometros();
}

//******************************************************************************************
//Loop principal
//******************************************************************************************
void loop()
{
  Potenciometros();
}

//******************************************************************************************
//Potenciometros
//******************************************************************************************
void Potenciometros(void)
{
  ADCPoten1 = (analogReadMilliVolts(Poten1)*255) / 3150;
  VoltajePoten1 = (alpha * ADCPoten1) + ((1.0 - alpha) * VoltajePoten1); //Filtro pasa bajas
  Serial.print("Voltaje potenciometro 1: ");
  Serial.println(VoltajePoten1);

  ADCPoten2 = (analogReadMilliVolts(Poten2)*255) / 3150;
  VoltajePoten2 = (alpha * ADCPoten2) + ((1.0 - alpha) * VoltajePoten2); //Filtro pasa bajas
  Serial.print("Voltaje potenciometro 2: ");
  Serial.println(VoltajePoten2);
}

//