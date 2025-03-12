
// Version v2.1 for controlling BIXO v2 with leadscrew and ON-OFF controller. Diogo Fonseca, Sept. 2023






//for on-off controller
float error[5] = { 0, 0, 0, 0, 0 };  //pressure error (A1, A2, A3, A4, A5)
int tolerance = 1;                   //differential gap (a.k.a. hysteresis)
byte pwm[5] = { 0, 0, 0, 0, 0 };     //PWM duty cycle (0-255)(A1, A2, A3, A4, A5)

//shoulders
const byte pwm_6v = 255;    // pwm value for the shoulder motor
bool ShoulderUp = false;    // 0 = not up, 1 = up
bool ShoulderDown = false;  // 0 = not down, 1 = down
const byte UpGatePin = 13;
const byte DownGatePin = 11;

//actuator parameters
byte maxPressure = 130;

//other vars
byte incomingBytes[5];
byte setCoordinate[5];    // [0] A1; [1] A2; [2] A3; [3] A4; [4] A5
float currCoordinate[5];  // TEMPORÁRIO PARA TESTES
byte msg[5];              // aux byte array to store entering and exiting data

//PINs for shield control：
byte M1 = 4;   //enable stage 1 (A1)(S1Ch2)
byte M2 = 8;   //enable stage 2 (A2)(S2Ch2)
byte M3 = 12;  //enable stage 3 (A3)(S3Ch1)
byte M4 = 7;   //enable stage 3 (A4)(S2Ch1)
byte M5 = 2;   //enable stage 3 (A5)(S1Ch1)
byte E1 = 5;   //PWM stage 1 (A1)(S1Ch2)
byte E2 = 9;   //PWM stage 2 (A2)(S2Ch2)
byte E3 = 10;  //PWM stage 3 (A3)(S3Ch1)
byte E4 = 6;   //PWM stage 3 (A4)(S2Ch1)
byte E5 = 3;   //PWM stage 3 (A5)(S1Ch1)



void setup() {

  //define digital pin modes
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(M3, OUTPUT);
  pinMode(M4, OUTPUT);
  pinMode(M5, OUTPUT);
  pinMode(E1, OUTPUT);
  pinMode(E2, OUTPUT);
  pinMode(E3, OUTPUT);
  pinMode(E4, OUTPUT);
  pinMode(E5, OUTPUT);
  //define analogue pin modes
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(UpGatePin, INPUT_PULLUP);
  pinMode(DownGatePin, INPUT_PULLUP);

  //ensure no current flows to actuators at this point
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
  digitalWrite(M3, LOW);
  digitalWrite(M4, LOW);
  digitalWrite(M5, LOW);
  analogWrite(E1, 0);  //PWM Control to zero
  analogWrite(E2, 0);  //PWM Control to zero
  analogWrite(E3, 0);  //PWM Control to zero
  analogWrite(E3, 0);  //PWM Control to zero
  analogWrite(E5, 0);  //PWM Control to zero

  // USB serial handshake
  handshake();
}




void loop() {
  // check if there are new commands comming in
  if (Serial.available() > 0) {
    delay(1);
    if (Serial.available() == 5) {
      // read the incoming bytes:
      for (int i = 0; i <= 4; i++) {
        incomingBytes[i] = Serial.read();
      }
      // validate recieved data:
      for (int i = 0; i <= 4; i++) {
        if (incomingBytes[i] >= 0 && incomingBytes[i] <= maxPressure) {
          setCoordinate[i] = incomingBytes[i];  // set new set pressure
        }
      }
    }
    //send back receieved setCoordinate for confirmation
    Serial.write(setCoordinate, 5);
    //round and load (float)currCoordinate to (byte)msg variable
    for (int i = 0; i <= 4; i++) {
      msg[i] = (byte)currCoordinate[i];
    }
    //send msg with current pressure values (kPa)
    Serial.write(msg, 5);
    Serial.write(pwm, 5);
    //clear serial buffer
    while (Serial.available()) {
      Serial.read();
    }
  }



  //Update shoulder position
  if (digitalRead(UpGatePin) == LOW) {
    ShoulderUp = true;
  }
  if (digitalRead(DownGatePin) == LOW) {
    ShoulderDown = true;
  }


  //pressure measurements (Honeywell sensors)
  currCoordinate[0] = getPressure(A1);                 //actuator A1
  currCoordinate[1] = getPressure(A2);                 //actuator A2
  currCoordinate[2] = ShoulderUp * 10 + ShoulderDown;  //actuator A3(MOTOR) 10 = up, 01 = down, 00 = neither, 11 = both
  currCoordinate[3] = getPressure(A4);                 //actuator A4
  currCoordinate[4] = getPressure(A5);                 //actuator A5


  //Update error values
  error[0] = setCoordinate[0] - currCoordinate[0];  //actuator A1
  error[1] = setCoordinate[1] - currCoordinate[1];  //actuator A2
  error[3] = setCoordinate[3] - currCoordinate[3];  //actuator A4
  error[4] = setCoordinate[4] - currCoordinate[4];  //actuator A5


  //ON_OFF controller to get (byte)PWM values
  pwm[0] = OnOffControl(tolerance, error[0], pwm[0]);  //actuator A1
  pwm[1] = OnOffControl(tolerance, error[1], pwm[1]);  //actuator A2
  pwm[3] = OnOffControl(tolerance, error[3], pwm[3]);  //actuator A4
  pwm[4] = OnOffControl(tolerance, error[4], pwm[4]);  //actuator A5


  //send PWM values to shield
  digitalWrite(M1, LOW);  // non-inverted polarity //actuator A1
  digitalWrite(M2, LOW);  //non-inverted polarity //actuator A2
  digitalWrite(M4, LOW);  //non-inverted polarity //actuator A4
  digitalWrite(M5, LOW);  //non-inverted polarity //actuator A5


  analogWrite(E1, pwm[0]);  //actuator A1
  analogWrite(E2, pwm[1]);  //actuator A2
  analogWrite(E4, pwm[3]);  //actuator A4
  analogWrite(E5, pwm[4]);  //actuator A5


  // move shoulders
  if (setCoordinate[2] == 1 && ShoulderUp == false) {  // if command up and shoulder is down
    ShoulderDown = false;
    pwm[2] = pwm_6v;  // just to send to app for monitoring
    moveShoulderUp(pwm[2], M3, E3);
  } else if (setCoordinate[2] == 0 && ShoulderDown == false) {  // if command down and shoulder is up
    ShoulderUp = false;
    pwm[2] = pwm_6v;  // just to send to app for monitoring
    moveShoulderDown(pwm[2], M3, E3);
  } else {
    moveShoulderStop(E3);  // stop motor
    pwm[2] = 0;            // just to send to app for monitoring
  }
}




void handshake() {
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }
  Serial.begin(57600);

  char a = 'b';
  while (a != 'a') {
    //acknowledgement routine
    Serial.println("a");
    //wait response from PC
    a = Serial.read();
  }
}



float getPressure(byte pin) {
  //pressure measurements (Honeywell sensors)
  int adc;       // analogue to digital conversion value (0 - 1023)
  float result;  //pressure value converted to kPa

  adc = analogRead(pin);  //0-1023 from top sensor

  result = (adc - 0.1 * 1023) * (60 / (0.8 * 1023));  //calculate pressure in PSI
  result = result * 6.89475729;                       //convert to kPa
  if (result < 0) { result = 0; }                     //avoid noise in case no sensor is connected
  return result;
}



void moveShoulderUp(byte pwm, byte pinPolarity, byte pinPower) {
  /* Function commands the upwards movement of the shoulder motor. 
              INPUTS: pwm value for the motor for the shield
                      pinPolarity = pin for polarity of the shield
                      pinPower = pin for power control of the shield
  */
  digitalWrite(pinPolarity, LOW);  //non-inverted polarity (shoulder going up)
  analogWrite(pinPower, pwm);      //motor command
}



void moveShoulderDown(byte pwm, byte pinPolarity, byte pinPower) {
  /* Function commands the upwards movement of the shoulder motor. 
              INPUTS: pwm value for the motor for the shield
                      pinPolarity = pin for polarity of the shield
                      pinPower = pin for power control of the shield
  */
  digitalWrite(pinPolarity, HIGH);  //inverted polarity (shoulder going down)
  analogWrite(pinPower, pwm);       //motor command
}



void moveShoulderStop(byte pinPower) {
  /* Function commands the upwards movement of the shoulder motor. 
              INPUTS: pinPower = pin for power control of the shield
  */
  analogWrite(pinPower, 0);  //stop motor command
}



byte OnOffControl(int diffgap, float Error, byte last_output) {
  //////////INPUTS////////////
  //      diffgap - differential gap (symmetric around set)
  //      Error - array with last recorded error
  //      last_output - is the previous output of the function (0 or 255)
  //////////OUTPUTS///////////
  //      result = pwm val
  ////////////////////////////

  byte result;

  bool last_state = last_output == 255;  //last cycle's controller output: true = ON, false = OFF.

  if (Error >= diffgap) {
    result = 255;
  } else if (last_state && -diffgap < Error && Error < diffgap) {
    result = 255;
  } else if (!last_state && -diffgap < Error && Error < diffgap) {
    result = 0;
  } else if (Error <= -diffgap) {
    result = 0;
  }
  return result;
}
