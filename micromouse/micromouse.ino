#include "BluetoothSerial.h"
#include <analogWrite.h>
#include <Wire.h>
#include <math.h>                            // necesaria para utilizar función atan()
#define PI 3.1415926535897932384626433832795 // definición del número PI

BluetoothSerial SerialBT;
// Constantes y variables para sensores
const int pinIRIzquierdo = 4;
const int pinIRDerecho = 2;
const int pinIRFrontal = 15;

const int pinSensorRanurado1 = 16;
const int pinSensorRanurado2 = 17;

const int enA = 5;
const int in1 = 18;
const int in2 = 19;
const int in3 = 21;
const int in4 = 22;
const int enB = 23;

int valorIRIzquierdo;
int valorIRDerecho;
int valorIRFrontal;
int valorRanurado1;
int valorRanurado2;
int fase = 1;

void leerSensores();
void enviarDatos();
void recibirComandos();
void ejecutarMovimientos(char comando);

void setup()
{
    Serial.begin(9600);
    SerialBT.begin("Micromouse");

    pinMode(pinIRIzquierdo, INPUT);
    pinMode(pinIRDerecho, INPUT);
    pinMode(pinIRFrontal, INPUT);

    // pinMode(pinSensorRanurado1, INPUT);
    // pinMode(pinSensorRanurado2, INPUT);
    attachInterrupt(digitalPinToInterrupt(pinSensorRanurado1), REncoder, RISING); // linea para añadir una interrupciòn a un PIN
    attachInterrupt(digitalPinToInterrupt(pinSensorRanurado2), LEncoder, RISING); // linea para añadir una interrupciòn a un PIN

    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    pinMode(enB, OUTPUT);

    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enA, 0);
    analogWrite(enB, 0);

    Serial.println("Dispositivo Bluetooth iniciado");
}

// Odometry

// Valores que no se mueven

float x = 0;          // distancia recorrida eje X
float y = 0;          // distancia recorrida eje Y

volatile unsigned muestreoActual = 0; // variables para definiciòn del tiempo de muestreo
volatile unsigned muestreoAnterior = 0;
volatile unsigned deltaMuestreo = 0;

float error = 0; // error variables
int PWMr = 0;    // PWM de la llanta derecha (señal de control llanta derecha)
int PWMl = 0;    // PWM de la llanta izquierda (señal de control llanta izquierda)

///------------------------------- Variables Posición del robot---------------------------------------------
float Cdistancia = 0; // distancia recorrido punto central

///------------------------------- Variables del robot  ---------------------------------------------

float W = 0;           // Velocidad Angular del carro

///------------------------------- Variables de motor derecho---------------------------------------------

volatile unsigned muestreoActualInterrupcionR = 0; // variables para definiciòn del tiempo de interrupciòn y calculo de la velocidad motor derecho
volatile unsigned muestreoAnteriorInterrupcionR = 0;
double deltaMuestreoInterrupcionR = 0;

int encoderR = 3; // pin de conexiòn del encoder derecho
int llantaR = 11; // pin de conexiòn de llanta derecha   (pin de PWM)

double frecuenciaR = 0;                           // frecuencia de interrupciòn llanta R
double Wr = 0;                                    // Velocidad angular R
double Vr = 0;                                    // velocidad Lineal
int CR = 0;                                       // contador ticks
float vectorR[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // vector de almacenamiento de datos para promedio del tiempo de interrupciones

float Rdistancia = 0; // distancia recorrida llanta derecha
int Rtick = 0;        // ticks del encoder derecho
int RtickAnt = 0;     // ticks del encoder derecho anteriores
int deltaRtick = 0;   // diferencia del encoder derecho

//------------------------------  Variables de motor Izquierdo ------------------------------------------------

volatile unsigned muestreoActualInterrupcionL = 0; // variables para definiciòn del tiempo de interrupciòn y calculo de la velocidad motor Izquierdo
volatile unsigned muestreoAnteriorInterrupcionL = 0;
double deltaMuestreoInterrupcionL = 0;

int encoderL = 2; // pin de conexiòn del encoder Izquierdo
int llantaL = 10; // pin de conexiòn de llanta Izquierda   (pin de PWM)

double frecuenciaL = 0;                           // frecuencia de interrupciòn llanta Izquierda
double Wl = 0;                                    // Velocidad angular L
double Vl = 0;                                    // velocidad Lineal
int CL = 0;                                       // contador Ticks
float vectorL[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // vector de almacenamiento de datos para promedio del tiempo de interrupciones

float Ldistancia = 0; // distancia recorrida llanta izquierda
int Ltick = 0;        // ticks del encoder izquierdo
int LtickAnt = 0;     // ticks del encoder izquier anteriores
int deltaLtick = 0;   // diferencia del encoder izquierdo

//////////////////////////// VALORES INICIALES A MOVER///////////////////////////////

int N = 20;            // nùmero de ranuras del encoder
int contadorTicks = 40; // número de ticks para cálculo de velocidad (recordar que entre menor sea el valor mayor ruido de la medida)
int tam = 10;          // tamaño del vector del calculo de promedio (Este valor depende del tamaño de los vectores de promedio vectorL y vectorR)
int k = 50;            // tiempo de muestreo
float Kp = 4000;   // Contante proporcional control
int xd = 100;
int PWMmin = 100; // PWM mìnimo
int PWMmax = PWMmin+30; // PWM màximo

float V = PWMmin+10;           // Velocidad lineal del carro

///------------------------------- Variables Posición deseada ---------------------------------------------
float Xd = 5;
float Yd = 0;
float Phid= atan2(Yd-y, Xd-x);

float phi = 0;        // posición angular inicial

float diametro = 6.5;  // diametro de la llanta cm
float longitud = 9.7; // longitud del robot entre llantas

void enviarDatos() {
  // Envía el valor del sensor IR izquierdo
  SerialBT.print("IRL,");
  SerialBT.print(valorIRIzquierdo);
  SerialBT.print(",");
  // Envía el valor del sensor IR derecho
  SerialBT.print("IRD,");
  SerialBT.print(valorIRDerecho);
  SerialBT.print(",");
  // Envía el valor del sensor IR frontal
  SerialBT.print("IRF,");
  SerialBT.print(valorIRFrontal);
    SerialBT.print(",");
  
  SerialBT.print("phi,");
  SerialBT.print(phi);

//   SerialBT.print(",");
  // Envía el valor del primer sensor ranurado
//   SerialBT.print("SR1,");
//   SerialBT.print(valorRanurado1);
//   SerialBT.print(",");
  // Envía el valor del segundo sensor ranurado
//   SerialBT.print("SR2,");
//   SerialBT.print(valorRanurado2);
  // Indica el final de la línea de datos
  SerialBT.println();
}

void REncoder()
{            // función de interrupción del enconder llanta derecha
  Serial.println("Hola R");
    Rtick++; // Nùmero de ticks llanta derecha
    CR++;    // incremento del contador de ticks
    if (CR == contadorTicks)
    {                    // si el contador de ticks alcanza el valor de ticks determinado para el cálculo del tiempo
        float media = 0; // variable creada para cálculo del promedio
                         //-------------------------------------- -----------------------------    Filtro promedio    -----------------------------------------------------------------------------//
        for (int i = 0; i < tam - 1; i++)
        { // relleno del vector para cálculo posterior del promedio
            vectorR[i] = vectorR[i + 1];
        }
        vectorR[tam - 1] = deltaMuestreoInterrupcionR; // ùltimo dato del vector (medida actual)

        for (int i = 0; i < tam; i++)
        { // Suma de los valores del vector
            media = vectorR[i] + media;
        }
        media = media / tam;                                         // división por el total de datos del vector
        deltaMuestreoInterrupcionR = media;                          // se reemplaza por el valor de su medío.
                                                                     //-------------------------------------- ----------------------------- ---------------------------------------------------------------------------------------------------//
        frecuenciaR = (1000) / deltaMuestreoInterrupcionR;           // frecuencia de interrupciòn
        muestreoAnteriorInterrupcionR = muestreoActualInterrupcionR; // se actualiza el tiempo de interrupciòn anterior
        CR = 0;                                                      // Reinicio de contador de ticks
    }
}

void LEncoder()
{            // funciòn de interrupciòn del enconder llanta izquierda
  Serial.println("Hola L");
    Ltick++; // Nùmero de ticks llanta izquierda
    CL++;    // incremento del contador de ticks
    if (CL == contadorTicks)
    {                    // si el contador de ticks alcanza el valor de ticks determinado para el cálculo del tiempo
        float media = 0; // variable creada para cálculo del promedio
                         //-------------------------------------- -----------------------------    Filtro promedio    -----------------------------------------------------------------------------//
        for (int i = 0; i < tam - 1; i++)
        { // relleno del vector para calculo posterior del promedio
            vectorL[i] = vectorL[i + 1];
        }
        vectorL[tam - 1] = deltaMuestreoInterrupcionL; // último dato del vector (medida actual)

        for (int i = 0; i < tam; i++)
        { // Suma de los valores del vector
            media = vectorL[i] + media;
        }
        media = media / tam;                                         // división por el total de datos del vector
        deltaMuestreoInterrupcionL = media;                          // se reemplaza por el valor de su medío.
                                                                     //-------------------------------------- ----------------------------- ---------------------------------------------------------------------------------------------------//
        frecuenciaL = (1000) / deltaMuestreoInterrupcionL;           // frecuencia de interrupciòn
        muestreoAnteriorInterrupcionL = muestreoActualInterrupcionL; // se actualiza el tiempo de interrupciòn anterior
        CL = 0;                                                      // Reinicio de contador de ticks
    }
}


String dataString;
bool dataComplete = false;
int state = 0; // 0 = waiting, 1 = executing

void changeToWaitingState(){
    state = 0;
    dataString = "";
    dataComplete = false;
}

void setOdometryValuesToDefault(){
    x = 0;          // distancia recorrida eje X
    y = 0;          // distancia recorrida eje Y
    
    muestreoActual = 0; // variables para definiciòn del tiempo de muestreo
    muestreoAnterior = 0;
    deltaMuestreo = 0;

    error = 0; // error variables
    PWMr = 0;    // PWM de la llanta derecha (señal de control llanta derecha)
    PWMl = 0;    // PWM de la llanta izquierda (señal de control llanta izquierda)

    ///------------------------------- Variables Posición del robot---------------------------------------------
    Cdistancia = 0; // distancia recorrido punto central

    ///------------------------------- Variables del robot  ---------------------------------------------

    W = 0;           // Velocidad Angular del carro

    ///------------------------------- Variables de motor derecho---------------------------------------------

    muestreoActualInterrupcionR = 0; // variables para definiciòn del tiempo de interrupciòn y calculo de la velocidad motor derecho
    muestreoAnteriorInterrupcionR = 0;
    deltaMuestreoInterrupcionR = 0;

    encoderR = 3; // pin de conexiòn del encoder derecho
    llantaR = 11; // pin de conexiòn de llanta derecha   (pin de PWM)

    frecuenciaR = 0;                           // frecuencia de interrupciòn llanta R
    Wr = 0;                                    // Velocidad angular R
    Vr = 0;                                    // velocidad Lineal
    CR = 0;                                       // contador ticks
    for(int i=0;i<tam;i++){
        vectorR[i]=0;
    }
    
    Rdistancia = 0; // distancia recorrida llanta derecha
    Rtick = 0;        // ticks del encoder derecho
    RtickAnt = 0;     // ticks del encoder derecho anteriores
    deltaRtick = 0;   // diferencia del encoder derecho

    //------------------------------  Variables de motor Izquierdo ------------------------------------------------

    muestreoActualInterrupcionL = 0; // variables para definiciòn del tiempo de interrupciòn y calculo de la velocidad motor Izquierdo
    muestreoAnteriorInterrupcionL = 0;
    deltaMuestreoInterrupcionL = 0;

    encoderL = 2; // pin de conexiòn del encoder Izquierdo
    llantaL = 10; // pin de conexiòn de llanta Izquierda   (pin de PWM)

    frecuenciaL = 0;                           // frecuencia de interrupciòn llanta Izquierda
    Wl = 0;                                    // Velocidad angular L
    Vl = 0;                                    // velocidad Lineal
    CL = 0;                                       // contador Ticks
    for(int i=0;i<tam;i++){
        vectorL[i]=0;
    }
    Ldistancia = 0; // distancia recorrida llanta izquierda
    Ltick = 0;        // ticks del encoder izquierdo
    LtickAnt = 0;     // ticks del encoder izquier anteriores
    deltaLtick = 0;   // diferencia del encoder izquierdo
}

void setNewTargets(){

}

void changeToExecutingState(){
    state = 1;
    setOdometryValuesToDefault();
}

void odometriaLoop(){
    //Serial.println("Hola");

    muestreoActual = millis();              // Tiempo actual de muestreo
    muestreoActualInterrupcionR = millis(); // se asigna el tiempo de ejecuciòn a el muestreo actual
    muestreoActualInterrupcionL = millis(); // se asigna el tiempo de ejecuciòn a el muestreo actual

    //Serial.println("Hola2");

    deltaMuestreo = (double)muestreoActual - muestreoAnterior; // delta de muestreo
    if (deltaMuestreo >= k)                                    // se asegura el tiempo de muestreo
    {
      //Serial.println("Hola3");
        float Phid = atan2(Yd - y, Xd - x); // Recalcular el ángulo deseado en cada iteración, dado que el cambia con respecto  a cada movimiento

        deltaMuestreoInterrupcionR = muestreoActualInterrupcionR - muestreoAnteriorInterrupcionR; // diferencia tiempos de interruciones de ticks del motor
        deltaMuestreoInterrupcionL = muestreoActualInterrupcionL - muestreoAnteriorInterrupcionL; // diferencia tiempos de interruciones de ticks del motor

        if (deltaMuestreoInterrupcionR >= 200 * contadorTicks)
        {                    // Esta es la forma de definir cuando el motor se encuentra quieto. Si deltaMuestreoInterrupcionR es mayor a 40 milisegundos por el preescalado de ticks
            frecuenciaR = 0; // 40 mS es el tiempo que màximo se tarda un tick a la menor velocidad del motor
        }
        if (deltaMuestreoInterrupcionL >= 200 * contadorTicks)
        {                    // Esta es la forma de definir cuando el motor se encuentra quieto. Si deltaMuestreoInterrupcionR es mayor a 40 milisegundos por el preescalado de ticks
            frecuenciaL = 0; // 40 mS es el tiempo que màximo se tarda un tick a la menor velocidad del motor
        }

        Wr = contadorTicks * ((2 * PI) / N) * frecuenciaR; // frecuencia angular Rad/s
        Vr = Wr * (diametro / 2);                          // velocidad lineal cm/s
        Wl = contadorTicks * ((2 * PI) / N) * frecuenciaL; // frecuencia angular Rad/s
        Vl = Wl * (diametro / 2);                          // velocidad lineal cm/s

        //        V = (Vr+Vl)/2;                                                                                    // calculo de la velocidad del robot
        //V = 50;                                // velocidad constante para alcanzar el àngulo
        error = Phid - phi;                    // error angular Angulo deseado menos el angulo del robot
        W = (Vr - Vl) / longitud + Kp * error; // Càlculo de la velocidad angular con las variables de control
        PWMr = V + (W * longitud) / 2;         // Señal de control PWM llanta derecha
        PWMl = V - (W * longitud) / 2;         // Señal de control PWM llanta izquierda

        //-------------------------------------- condicionales para limites de la señal de PWM ---------------------------------------------------------------------------------------------------//

        if (PWMr > PWMmax)
        {
            PWMr = PWMmax;
        }
        if (PWMr < PWMmin)
        {
            PWMr = PWMmin;
        }
        if (PWMl > PWMmax)
        {
            PWMl = PWMmax;
        }
        if (PWMl < PWMmin)
        {
            PWMl = PWMmin;
        }

        //Serial.println("Hola3.5");

        if (abs(x - Xd) < 1 && abs(y - Yd) < 1)
        {
            analogWrite(enA, 0);
            analogWrite(enB, 0);

            enviarDatos();

            changeToWaitingState();
        }
        else
        {
          //Serial.print("PWNr: ");
          //Serial.println(PWMr);
            analogWrite(enA, PWMr);
            analogWrite(enB, PWMl);
        }

        //        analogWrite(llantaR,PWMr);                                                                           // PWM de la llanta derecha
        //        analogWrite(llantaL,PWMl);                                                                           // PWM de la llanta izquierda

        //        analogWrite(llantaR,0);                                                                           // PWM de la llanta derecha
        //        analogWrite(llantaL,0);                                                                           // PWM de la llanta izquierda

      //Serial.println("Hola4");

        odometria(); // cálculo de la odometría

        // Serial.print(Ltick);
        // Serial.print(","); // se muestra el tiempo entre TIC y TIC
        // Serial.print(Rtick);
        // Serial.print(","); // se muestra el tiempo entre TIC y TIC
        
        // Serial.print(x);   // se muestra el tiempo entre TIC y TIC
        // Serial.print(","); // se muestra el tiempo entre TIC y TIC
        // Serial.println(y); // se muestra el tiempo entre TIC y TIC

        muestreoAnterior = muestreoActual; // actualización del muestreo anterior

        //Serial.println("Hola5");
    }
}

void serialEvent(){
    while(SerialBT.available()){
        char inChar = (char) SerialBT.read();
        dataString += inChar;
        if(inChar == '\n'){
            dataComplete = true;
        }
    }
}

void loop()
{
    if(state == 0){
        if(SerialBT.available()) serialEvent();
        if(dataComplete){
            Xd = dataString.toInt();
            
            if(SerialBT.available()) serialEvent();
            if(dataComplete){
                Yd = dataString.toInt();

                changeToExecutingState();

                float Phid= atan2(Yd-y, Xd-x);
            }
        }
    }
    else if(state == 1){
        odometriaLoop();
    }
}

void odometria()
{

    deltaRtick = Rtick - RtickAnt;                          // comparación de los ticks recorridos desde el último cálculo llanta derecha
    Rdistancia = PI * diametro * (deltaRtick / (double)20); // distancia recorrida por la llanta derecha desde el último cálculo

    deltaLtick = Ltick - LtickAnt;                          // comparación de los ticks recorridos desde el último cálculo llanta izquierda
    Ldistancia = PI * diametro * (deltaLtick / (double)20); // distancia recorrida por la llanta izquierda desde el último cálculo

    Cdistancia = (Rdistancia + Ldistancia) / 2; // distancia recorrida por el punto central desde el último cálculo

    x = x + Cdistancia * cos(phi); // posición del punto X actual
    y = y + Cdistancia * sin(phi); // posición del punto Y actual

    phi = phi + ((Rdistancia - Ldistancia) / longitud); // posición Angular actual
    phi = atan2(sin(phi), cos(phi));                    // transformación de la posición angular entre -PI y PI

    RtickAnt = Rtick; // actualización de la variable RtickAnt con los valores de Rtick
    LtickAnt = Ltick; // actualización de la variable LtickAnt con los valores de Ltick
}