

/*_______________________________________________________________________
 Didactic Platform for SCARA Robotics - Servodrives control
 
   By:
               S. Lihour
               Mechatronic Engineer
               lyhour457@gmail.com

   Platform:   ARDUINO IDE 2.2.1
   Version: 1.0

   Phnom Penh, February 2024
_______________________________________________________________________
*/

#include <SPI.h>
#include <Servo.h>
#include <stdio.h>
#include <avr/pgmspace.h>

#define bufferSerieSize		128

// union structure: translation between IEEE754, DOUBLE Y LONG
union u_tag
{
  char worth[4];
  double worthDoble;
  long worthEntero;
}u;

// Declaration of parameters and variables
const char CoordinatesXY[] PROGMEM = "*XY";
const char anglesM1M2[] PROGMEM = "*MS";
const char angleMI[] PROGMEM = "*MI";
const char angleMD[] PROGMEM = "*MD";
const char velocidadTrazo[] PROGMEM = "*VE";

uint8_t estadoActualSwitche = false, estadoFormerSwitche = false, contadorPaquete = 0;
uint8_t buffer1[40], datoListo = 0, paqueteListo = 1, longitudDato = 0;
char buffer2[40], nuevaPosicion = true, coordinateAProcesar = 0, coordinateProcessed = 0;
int headBufferCoordinates = 0, colaBufferCoordinates = 0, a = 0, i = 0, j = 0, m = 0, contadorBytesRx = 0, contadorBufferRx = 0, inicio = 0, final = 0,
  velocidad = 3, CounterSamples[6], worthAD[6], worthADFormer[6];
long worth[6] = {0, 0, 0, 0, 0, 0};
double X, Y, X1, Y1, bufferCoordinates[bufferSerieSize];

// Servodrives calibration - it will be commented by default.
//#define CALIBRACION

// Standar servodrives - it will be commented in case of mini - servodrives
#define servoNormal

// Reference -zero- position for servodrives
#ifdef servoNormal
  #define SERVOLEFTNULL 1000
  #define SERVORIGHTNULL 2135
  // 90 degrees of displacement factor
  #define SERVOFACTORLEFT 660
  #define SERVOFACTORRIGTH 663
#else
  // Reference -zero- position for mini-servodrives
  #define SERVOLEFTNULL 1000
  #define SERVORIGHTNULL 2135
  // 90 degrees of displacement factor
  #define SERVOFACTORLEFT 660
  #define SERVOFACTORRIGTH 660
#endif

#define PINSERVOLEFT   8
#define PINSERVORIGTH     9

// link´s lengths
#define L1 133
#define L2 122
#define L3 0

// fixed point coordinates
#define O1X 0
#define O1Y 0
#define O2X 28
#define O2Y 0

#include <Servo.h>
Servo servoLeft;
Servo servoRight;

volatile double pXFormer = O2X/2, centroX = O2X/2, motorRight, motorLeft;
volatile double pYFormer = 145, centroY = 145;

// Kinematic model
double angle(double a, double b, double c) 
{
  // angle for kinematics calculation
  return acos((a * a + c * c - b * b) / (2 * a * c));
}

void set_XY(double Tx, double Ty)
{
  delay(1);
  double dx, dy, c, a1, a2, b1, b2, Hx, Hy;

  dx = Tx - O1X;
  dy = Ty - O1Y;

  // length for kinematics calculation
  c = sqrt(dx * dx + dy * dy);
  b1 = atan2(dy, dx);
  b2 = angle(L1, L2, c);

  Hx = Tx;
  Hy = Ty;

  dx = Hx - O2X;
  dy = Hy - O2Y;

  c = sqrt(dx * dx + dy * dy);
  a1 = atan2(dy, dx);
  a2 = angle(L1, L2, c);
  
  double gradosMotorLeft = (b1 + b2) * 180/M_PI;
  double gradosMotorRight = (a1 - a2) * 180/M_PI;
  #ifdef servoNormal
    if ((gradosMotorLeft > 55) && (gradosMotorLeft < 215))
      motorLeft = (-(b1 + b2 - M_PI) * SERVOFACTORLEFT) + SERVOLEFTNULL;
  #else
    if ((gradosMotorLeft > 55) && (gradosMotorLeft < 215))
      motorLeft = (-(b1 + b2 - M_PI) * SERVOFACTORLEFT) + SERVOLEFTNULL;
  #endif
  
  #ifdef servoNormal
    if ((gradosMotorRight > -35) && (gradosMotorRight < 135))
      motorRight = (-(a1 - a2) * SERVOFACTORRIGTH) + SERVORIGHTNULL;
  #else
    if ((gradosMotorRight > -35) && (gradosMotorRight < 135))
      motorRight = (-(a1 - a2) * SERVOFACTORRIGTH) + SERVORIGHTNULL;
  #endif
  
  // driving the servodrives
  servoRight.writeMicroseconds(floor(motorRight));
  servoLeft.writeMicroseconds(floor(motorLeft));  
}

// path calculations
void draw(double pXActual, double pYActual)
{
  double dx, dy, c;

  // dx, dy - differential position with respect to the last point position
  dx = pXActual - pXFormer;
  dy = pYActual - pYFormer;
  
  // path length -mm-,  4 is for 4 steps/mm
  c = floor(velocidad * sqrt(dx * dx + dy * dy));
  if (c < 1) 
    c = 1;
  for (int j = 0; j <= c; j++)
  {
    // drawing a line point by point
    set_XY(pXFormer + (j * dx / c), pYFormer + (j * dy / c));
    recibirSerie();
  }
  pXFormer = pXActual;
  pYFormer = pYActual;
}

// Buffering
void recibirSerie(void)
{
  if(Serial.available() > 0)
  {
    buffer1[contadorBytesRx] = Serial.read();
    if(buffer1[contadorBytesRx] == '*')
    {
      if(paqueteListo)
      {
        inicio = contadorBytesRx;
        contadorPaquete = 0;
        paqueteListo = 0;
        datoListo = 0;
      }
    }
    else if(buffer1[contadorBytesRx] == '#')
    {
      if(contadorPaquete > 11)
      {
        final = contadorBytesRx + 1;
        datoListo = 1;
        paqueteListo = 1;
        contadorBytesRx = 0;
      }
    }
    if(contadorBytesRx > 39)
      contadorBytesRx = 0;
    else
    {
      contadorBytesRx++;
      contadorPaquete++;
    }
    if(datoListo)
    {
      contadorBufferRx = 0;
      for(int i = inicio; i < final; i++)
      {
        buffer2[contadorBufferRx] = buffer1[i];
        contadorBufferRx++;
      }
    }
  }
  
  if(datoListo)
  {
  // Message capture  
    datoListo = 0;
    longitudDato = contadorBufferRx;
    
    // XY coordinates
    if((strncasecmp_P(buffer2, CoordinatesXY, 3) == 0))
    {
      u.worth[3] = buffer2[7];
      u.worth[2] = buffer2[6];
      u.worth[1] = buffer2[5];
      u.worth[0] = buffer2[4];
      X = u.worthDoble;
      u.worth[3] = buffer2[11];
      u.worth[2] = buffer2[10];
      u.worth[1] = buffer2[9];
      u.worth[0] = buffer2[8];
      Y = u.worthDoble;

      bufferCoordinates[colaBufferCoordinates] = X;
      colaBufferCoordinates ++;
      if (colaBufferCoordinates >= bufferSerieSize)
        colaBufferCoordinates = 0;
      bufferCoordinates[colaBufferCoordinates] = Y;
      colaBufferCoordinates ++;
      if (colaBufferCoordinates >= bufferSerieSize)
        colaBufferCoordinates = 0;
    }
    
    // angles - Degrees- both servodrives
    if((strncasecmp_P(buffer2, anglesM1M2, 3) == 0))
    {
      u.worth[3] = buffer2[7];
      u.worth[2] = buffer2[6];
      u.worth[1] = buffer2[5];
      u.worth[0] = buffer2[4];
      #ifdef servoNormal
        servoLeft.writeMicroseconds((-(u.worthDoble*M_PI/180)*SERVOFACTORLEFT) + SERVOLEFTNULL);
      #else
        servoLeft.writeMicroseconds((-(u.worthDoble*M_PI/180)*SERVOFACTORLEFT) + SERVOLEFTNULL);
      #endif
      //Serial.println(u.worthDoble,6);
      u.worth[3] = buffer2[11];
      u.worth[2] = buffer2[10];
      u.worth[1] = buffer2[9];
      u.worth[0] = buffer2[8]; 
      #ifdef servoNormal
        servoRight.writeMicroseconds((-(u.worthDoble*M_PI/180)*SERVOFACTORRIGTH) + SERVORIGHTNULL);
      #else
        servoRight.writeMicroseconds((-(u.worthDoble*M_PI/180) * SERVOFACTORRIGTH) + SERVORIGHTNULL);
      #endif     
    }
    
    // Path speed control
    if((strncasecmp_P(buffer2, velocidadTrazo, 3) == 0))
    {
      u.worth[3] = buffer2[7];
      u.worth[2] = buffer2[6];
      u.worth[1] = buffer2[5];
      u.worth[0] = buffer2[4];
      velocidad = int(u.worthDoble);
      Serial.println(velocidad);      
    }
  }
}

void setup() 
{
  Serial.begin(9600);
  draw(centroX, centroY);
  servoLeft.attach(PINSERVOLEFT);  // servo Left
  servoRight.attach(PINSERVORIGTH);      // servo Right
  pinMode(13,INPUT_PULLUP);                   // switche final carrera
  pinMode(13,OUTPUT);
  Serial.println('R');
  
  // Calibration function
  #ifdef CALIBRACION  
    #ifdef servoNormal
      motorRight = (-(0*M_PI/180) * SERVOFACTORRIGTH) + SERVORIGHTNULL;
      motorLeft = (-(180*M_PI/180 - M_PI) * SERVOFACTORLEFT) + SERVOLEFTNULL;
    #else
      motorRight = (-(0*M_PI/180) * SERVOFACTORRIGTH) + SERVORIGHTNULL;
      motorLeft = (-(90*M_PI/180 - M_PI) * SERVOFACTORLEFT) + SERVOLEFTNULL;
    #endif
    servoLeft.writeMicroseconds(motorLeft);
    servoRight.writeMicroseconds(motorRight);
    while(1);
  #endif
}

void loop() 
Coordinates{
  recibirSerie();
  if (headBufferCoordinates != colaBufferCoordinates)
  {
    if (nuevaPosicion)
    {
      X1 = bufferCoordinates[headBufferCoordinates];
      headBufferCoordinates++;
      if (headBufferCoordinates >= bufferSerieSize)
        headBufferCoordinates = 0;
      Y1 = bufferCoordinates[headBufferCoordinates];
      coordinateAProcesar = 1;
      nuevaPosicion = false;
    }

    if (coordinateProcessed)
    {
      coordinateProcessed = 0;
      nuevaPosicion = true;
      headBufferCoordinates ++;
      if (headBufferCoordinates >= bufferSerieSize)
        headBufferCoordinates = 0;
    }
  }
  if (coordinateAProcesar)
  {
    draw(X1, Y1);
    coordinateProcessed = 1;
    coordinateAProcesar = 0;
  }
  
  // External switch registration - Not implemented
  estadoActualSwitche = digitalRead(2);
  if(estadoActualSwitche != estadoFormerSwitche)
  {
    estadoFormerSwitche = estadoActualSwitche;
    digitalWrite(13,estadoActualSwitche);
    Serial.write('D');
    Serial.println(estadoActualSwitche);
  }
  
  // Analog channel registration  - Not implemented
  //canal 0 A/D
  CounterSamples[0] ++;
  if (CounterSamples[0] < 100)
    worth[0] += analogRead(0);	// 0..1023
  else
  {
    CounterSamples[0] = 0;
    worth[0] /= 99;
    worthAD[0] = worth[0];
    worth[0] = 0;
  }			
  if (abs(worthADFormer[0] - worthAD[0]) > 2)
  {
    worthADFormer[0] = worthAD[0];
    Serial.write('A');
    Serial.println(worthAD[0]);
  }	
}
