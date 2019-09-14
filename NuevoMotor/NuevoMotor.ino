// Pines
int Referencia = A0;
int Encoder = A1;
byte CERO = 2;
//**************** SALIDAS ********************************//
byte TRIAC = 5; //

//Variables

double Ref = 0;
double En = 0;    //input entrada
double output = 0;
double lasttime=0;
double errsum, lastinput;
double kp=3, ki=3, kd=0;
int sampletime = 1; //en milisegundos
double salida=0;
float smax = 1024; // Máxima señal de control
float smin = 0; // Minima señal de control
double disparotriac=500;
double tiempodisparo=800;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(CERO, INPUT); //INT1 PASO POR CERO
  digitalWrite(CERO, HIGH);
  pinMode(TRIAC, OUTPUT);
  digitalWrite(TRIAC, LOW); //Inicializacion del disparo del triac (off)
  Serial.println("Encendido");
  //Interrupciones
  attachInterrupt(0, cero, FALLING); // pin 3 CERO
  sei();//allow interrupts
}




void loop() {
  Ref = analogRead(Referencia);
  En = analogRead(Encoder);  
  salida = Compute(En,Ref);
   disparotriac = map(salida, 0, 1024, 500, 7000);
   Serial.println(disparotriac);
}

//* Interrupción de detección de pasovelocidada por cero y disparo del triac *//
void cero() {
 delayMicroseconds(disparotriac);
 digitalWrite(TRIAC, HIGH); //Inicializacion del disparo del triac (off)
 delayMicroseconds(tiempodisparo);
 digitalWrite(TRIAC, LOW); //Inicializacion del disparo del triac (off)
}



double Compute(double input, double setpoint){
     unsigned long now = millis();
     int timechange = (now-lasttime);
     if (timechange >= sampletime){
        double error = setpoint - En;
        errsum += error;
        double dinput = (input - lastinput);
        output = kp * error + ki * errsum - kd * dinput;  //SALIDA DEL PID
        // Limitacón de la señal de control //
        if (output > smax) {
            output = smax;
        }
        else if (output < smin) {
            output = smin;
        }
        else
        output = output;
        //Guardamos el valor de algunas variables para el proximo ciclo
        lastinput = En; 
        lasttime = now;
       return output; 
      }   
  }



