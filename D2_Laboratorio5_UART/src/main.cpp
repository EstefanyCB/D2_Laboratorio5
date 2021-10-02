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

#define LedR 19
#define LedG 18
#define LedB 5

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
void UART(void);

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
int DCLedB = 0;

char inByte = 0;
String mensaje = "";

LiquidCrystal LCD(RS, E, D4, D5, D6, D7);
uint8_t P1Centena, P1Decena, P1Unidad;
uint8_t P2Centena, P2Decena, P2Unidad;
uint8_t ConCentena, ConDecena, ConUnidad;
//******************************************************************************************
//Configuraciones
//******************************************************************************************
void setup()
{
  Serial.begin(9600);
  Potenciometros();
  ConfiguracionPWM();
  UART();

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
  UART();
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
  LCD.print(ConCentena);
  LCD.print(ConDecena);
  LCD.print(ConUnidad);

  ledcWrite(PWMLedR, DCLedR);
  ledcWrite(PWMLedG, DCLedG);
  ledcWrite(PWMLedB, DCLedB);
}

//******************************************************************************************
//Potenciometros
//******************************************************************************************
void Potenciometros(void)
{
  ADCPoten1 = analogRead(Poten1);
  VoltajePoten1 = (alpha * ADCPoten1) + ((1.0 - alpha) * VoltajePoten1); //Filtro pasa bajas
  DCLedR = map(VoltajePoten1, 0, 4095, 0, 255);
  //Serial.print("Voltaje potenciometro 1: ");
  //Serial.println(DCLedR);
  delay(100);

  ADCPoten2 = analogRead(Poten2);
  VoltajePoten2 = (alpha * ADCPoten2) + ((1.0 - alpha) * VoltajePoten2); //Filtro pasa bajas
  DCLedG = map(VoltajePoten2, 0, 4095, 0, 255);
  //Serial.print("Voltaje potenciometro 2: ");
  //Serial.println(DCLedG);
  delay(100);
}

//******************************************************************************************
//Valores Potenciometros
//******************************************************************************************
void ValoresPotenciometros(void)
{
  P1Centena = DCLedR / 100;
  P1Decena = (DCLedR - (P1Centena * 100)) / 10;
  P1Unidad = (DCLedR - P1Centena * 100) - (P1Decena * 10);

  P2Centena = DCLedG / 100;
  P2Decena = (DCLedG - (P2Centena * 100)) / 10;
  P2Unidad = (DCLedG - P2Centena * 100) - (P2Decena * 10);

  ConCentena = DCLedB / 100;
  ConDecena = (DCLedB - (ConCentena * 100)) / 10;
  ConUnidad = (DCLedB - ConCentena * 100) - (ConDecena * 10);
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

//******************************************************************************************
//UART
//******************************************************************************************
void UART(void)
{
  if (Serial.available() > 0)
  {
    mensaje = Serial.readStringUntil('\n');

    Serial.print("Recibi el siguiente mensaje: ");
    Serial.println(mensaje);
  }

  if (mensaje == "+")
  {
    DCLedB++;
    Serial.print("El valor de led azul aumento a: ");
    Serial.println(DCLedB);
    mensaje = "";
    if (DCLedB > 255)
    {
      DCLedB = 0;
    }
  }

  else if (mensaje == "-")
  {
    DCLedB--;
    Serial.print("El valor de led azul disminuyo a: ");
    Serial.println(DCLedB);
    mensaje = "";
    if (DCLedB < 0)
    {
      DCLedB = 255;
    }
  }
}