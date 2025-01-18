
bool bstate1 = false, bstate2 = false, bstate3 = false, bstate4 = false,
     bstate5 = false;


const int tempSensorPin1 = 2;
const int transSensorPin1 = 23;
const int tempSensorPin2 = 15;
const int transSensorPin2 = 21;

float currentTemp1 = 0.0;
float currentTemp2 = 0.0;
float targetTemp1;
float targetTemp2;

const int heatingPower1 = 4096;
const int heatingPower2 = 3072;
const int heatingPower3 = 2048;
const int heatingPower4 = 1024;

float voltage1;
float voltage2;

const int button1 = 0, button2 = 35;
int count = 0, con = 6;
String c, states;

float r_Kp, r_Ki, r_Kd;
float d_Kp, d_Ki, d_Kd;
float error1, lasterror1, integral1;
unsigned long previousMillis1 = 0;
float error2, lasterror2, integral2;
unsigned long previousMillis2 = 0;

unsigned long lastButtonPress = 0;  // To track the last button press time
const unsigned long debounceDelay = 1000;  // Debounce delay in milliseconds


#define PWM1_Freq 5000
#define PWM1_Res 12
#define PWM1_Ch 0
#define PWM2_Freq 5000
#define PWM2_Res 12
#define PWM2_Ch 1

void setup() {
  Serial.begin(9600);

  lasterror1 = 0;
  integral1 = 0;
  lasterror2 = 0;
  integral2 = 0;

  // pinMode ( transSensorPin1 , OUTPUT ) ;
  // pinMode ( transSensorPin2 , OUTPUT ) ;
  ledcAttachPin(transSensorPin1, PWM1_Ch);
  ledcSetup(PWM1_Ch, PWM1_Freq, PWM1_Res);
  ledcAttachPin(transSensorPin2, PWM2_Ch);
  ledcSetup(PWM2_Ch, PWM2_Freq, PWM2_Res);

  pinMode(button1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(button1), Controller,RISING);

  pinMode(button2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(button2), Controller, RISING);

}


void Controller() 
{
  unsigned long currentMillis = millis();
  // Serial.println("Button pressed");

  if (currentMillis - lastButtonPress > debounceDelay) 
  {  
    // Serial.println("Inside button pressed");
    // Serial.println(con);
    con += 1;

    if (con > 7)
    {
      con =7 ;
      count ++;
    }
    

    if (count >3)
    {
      con =5 ;
      count =1;
    }

    if (count == 1) {
      states = "1 ";
      bstate1 = true;
      bstate2 = false;
      bstate3 = false;
      bstate4 = false;
      bstate5 = false;

    } else if (count == 2) {
      bstate1 = false;
      bstate2 = true;
      bstate3 = false;
      bstate4 = false;
      bstate5 = false;
      states = "2 ";
    } else if (count == 3) {
      bstate1 = false;
      bstate2 = false;
      bstate3 = true;
      bstate4 = false;
      bstate5 = false;
      states = "3 ";
    } else if (count == 4) {
      bstate1 = false;
      bstate2 = false;
      bstate3 = false;
      bstate4 = true;
      bstate5 = false;
      states = "4 ";
    } else if (count == 5) {
      bstate1 = false;
      bstate2 = false;
      bstate3 = false;
      bstate4 = false;
      bstate5 = true;
      states = "5 ";
    }

    lastButtonPress = currentMillis;
  }
}

void loop() {
  //UPDATE P,PI,PD,PID values
  if (con > 7) {
    con = 7;
  }
  if (con == 1) {
    r_Kp = 0.3638;
    r_Kd = 0.0000;
    r_Ki = 0.0071;
    c = " Ref RL 1";
  } else if (con == 2 ) {
    r_Kp = 0.1642;
    r_Kd = 1.2046;
    r_Ki = 0.0056;
    c = " Ref RL 2 ";
  } else if (con == 3) {
    d_Kp = 0.305082828676907;
    d_Kd = 0;
    d_Ki = 0.006175369667180;
    c = " Ref FR 1 ";
  } else if (con == 4 ) {
    r_Kp = 0.282626671472667;
    r_Kd = 0;
    r_Ki = 0.005766488820061;
    c = " Ref FR 2 ";
  } 
  else if (con == 5 ) {
    d_Kp = 0.305082828676907;
    d_Kd = 0;
    d_Ki = 0.006175369667180;
    c = " Dis RL 1 ";
  } else if (con == 6 ) {
    d_Kp = 0.282626671472667;
    d_Kd = 0;
    d_Ki = 0.005766488820061;
    c = " Dis RL 2 ";
  } else if (con == 7 ) {
    d_Kp = 0.305082828676907;
    d_Kd = 0;
    d_Ki = 0.006175369667180;
    c = " Dis FR 1 ";
  } else if (con == 8 ) {
    d_Kp = 0.282626671472667;
    d_Kd = 0;
    d_Ki = 0.005766488820061;
    c = " Dis FR 2";
  } 


  if (bstate1) {
    state1();
  } else if (bstate2) {
    state2();
  } else if (bstate3) {
    state3();
  } else if (bstate4) {
    state4();
  } else if (bstate5) {
    state5();
  }
  delay(1000);
}

int getRefPWM1_1(float currentTemp, float target) {
  int PWM;
  unsigned long currentMillis = millis();
  float elapsedTime = (currentMillis - previousMillis1) / 1000.0;

  error1 = target - currentTemp;
  integral1 += (error1 * elapsedTime) * r_Ki;

  // Limit the integral to avoid windup
  float maxIntegral = 1000;  // Set an appropriate max limit
  float minIntegral = -1000; // Set an appropriate min limit
  if (integral1 > maxIntegral) {
    integral1 = maxIntegral;
  }
  if (integral1 < minIntegral) {
    integral1 = minIntegral;
  }

  float derivative = 0;
  if (elapsedTime > 0) {
    derivative = (error1 - lasterror1) / elapsedTime;
  }

  float pidOutput = r_Kp * error1 + integral1 + r_Kd * derivative;

  // Scale to PWM range
  PWM = (pidOutput / 3.3) * 4095;

  // Limit PWM value
  if (PWM > 4095) {
    PWM = 4095;
  }
  if (PWM <0) {
    PWM = 0;
  }

  lasterror1 = error1;
  previousMillis1 = currentMillis;

  return PWM;
}



int getRefPWM1_2(float currentTemp, float target) {
  int PWM;
  unsigned long currentMillis = millis();
  float elapsedTime = (currentMillis - previousMillis2) / 1000.0;

  error2 = target - currentTemp;
  integral2 += (error2 * elapsedTime) * r_Ki;

  // Limit the integral to avoid windup
  float maxIntegral = 1000;  // Set an appropriate max limit
  float minIntegral = -1000; // Set an appropriate min limit
  if (integral2 > maxIntegral) {
    integral2 = maxIntegral;
  }
  if (integral2 < minIntegral) {
    integral2 = minIntegral;
  }

  float derivative = 0;
  if (elapsedTime > 0) {
    derivative = (error2 - lasterror2) / elapsedTime;
  }

  float pidOutput = r_Kp * error2 + integral2 + r_Kd * derivative;

  // Scale to PWM range
  PWM = (pidOutput / 3.3) * 4095;

  // Limit PWM value
  if (PWM > 4095) {
    PWM = 4095;
  }
  if (PWM <0) {
    PWM = 0;
  }

  lasterror2 = error2;
  previousMillis2 = currentMillis;

  return PWM;
}


int getDisPWM2_1(float currentTemp, float target) {
  int PWM;
  unsigned long currentMillis = millis();
  float elapsedTime = (currentMillis - previousMillis1) / 1000.0;

  error1 = target - currentTemp;
  integral1 += (error1 * elapsedTime) * d_Ki;

  // Limit the integral to avoid windup
  float maxIntegral = 1000;  // Set an appropriate max limit
  float minIntegral = -1000; // Set an appropriate min limit
  if (integral1 > maxIntegral) {
    integral1 = maxIntegral;
  }
  if (integral1 < minIntegral) {
    integral1 = minIntegral;
  }

  float derivative = 0;
  if (elapsedTime > 0) {
    derivative = (error1 - lasterror1) / elapsedTime;
  }

  float pidOutput = d_Kp * error1 + integral1 + d_Kd * derivative;

  // Scale to PWM range
  PWM = (pidOutput / 3.3) * 4095;

  // Limit PWM value
  if (PWM > 4095) {
    PWM = 4095;
  }
  if (PWM <0) {
    PWM = 0;
  }

  lasterror1 = error1;
  previousMillis1 = currentMillis;

  return PWM;
}


int getDisPWM2_2(float currentTemp, float target) {
  int PWM;
  unsigned long currentMillis = millis();
  float elapsedTime = (currentMillis - previousMillis2) / 1000.0;

  error2 = target - currentTemp;
  integral2 += (error2 * elapsedTime) * d_Ki;

  // Limit the integral to avoid windup
  float maxIntegral = 1000;  // Set an appropriate max limit
  float minIntegral = -1000; // Set an appropriate min limit
  if (integral2 > maxIntegral) {
    integral2 = maxIntegral;
  }
  if (integral2 < minIntegral) {
    integral2 = minIntegral;
  }

  float derivative = 0;
  if (elapsedTime > 0) {
    derivative = (error2 - lasterror2) / elapsedTime;
  }

  float pidOutput = d_Kp * error2 + integral2 + d_Kd * derivative;

  // Scale to PWM range
  PWM = (pidOutput / 3.3) * 4095;

  // Limit PWM value
  if (PWM > 4095) {
    PWM = 4095;
  }
  if (PWM <0) {
    PWM = 0;
  }

  lasterror2 = error2;
  previousMillis2 = currentMillis;

  return PWM;
}

void state1() {

  targetTemp1 = 35;
  currentTemp1 = readTemp(tempSensorPin1);
  int PWM1;
  targetTemp2 = 35;
  currentTemp2 = readTemp(tempSensorPin2);
  int PWM2;
  if (con <4)
  {
    PWM1 = getRefPWM1_1(currentTemp1, targetTemp1);
    PWM2 = getRefPWM1_2(currentTemp2, targetTemp2);
  }
  else
  {
    PWM1 = getDisPWM2_1(currentTemp1, targetTemp1);
    PWM2 = getDisPWM2_2(currentTemp2, targetTemp2);
  }
  // Serial . println ( PWM1 ) ;
  voltage1 = 3.3 * PWM1 / 4095;
  voltage2 = 3.3 * PWM2 / 4095;

  if (currentTemp1 < targetTemp1) {
    ledcWrite(PWM1_Ch, PWM1);
    ledcWrite(PWM2_Ch, PWM2);
    // analogWrite ( transSensorPin1 , PWM1 ) ;
    // analogWrite ( transSensorPin2 , 0) ;
    currentTemp1 = readTemp(tempSensorPin1);

    delay(500);
    currentTemp2 = readTemp(tempSensorPin2);
    Serial.print(count);
    Serial.print(" ,");
    Serial.print(con);
    Serial.print(" ,");
    Serial.print(currentTemp1);
    Serial.print(" ,");
    Serial.print(currentTemp2);
    Serial.print(" ,");
    Serial.print(voltage1, 2);
    Serial.print(" ,");
    Serial.println(voltage2, 2);
  } else {
    ledcWrite(PWM1_Ch, PWM1);
    ledcWrite(PWM2_Ch, PWM2);
    // analogWrite ( transSensorPin1 , 0) ;
    // analogWrite ( transSensorPin2 , 0) ;
    currentTemp1 = readTemp(tempSensorPin1);

    delay(500);
    currentTemp2 = readTemp(tempSensorPin2);
    Serial.print(count);
    Serial.print(" ,");
    Serial.print(con);
    Serial.print(" ,");
    Serial.print(currentTemp1);
    Serial.print(" ,");
    Serial.print(currentTemp2);
    Serial.print(" ,");
    Serial.print(voltage1);
    Serial.print(" ,");
    Serial.println(voltage2);
  }
}
void state2() {
  // Serial . println ("2") ;
  targetTemp1 = 60;
  currentTemp1 = readTemp(tempSensorPin1);
  int PWM1;
  targetTemp2 = 35;
  currentTemp2 = readTemp(tempSensorPin2);
  int PWM2;
  if (con <4)
  {
    PWM1 = getRefPWM1_1(currentTemp1, targetTemp1);
    PWM2 = getRefPWM1_2(currentTemp2, targetTemp2);
  }
  else
  {
    PWM1 = getDisPWM2_1(currentTemp1, targetTemp1);
    PWM2 = getDisPWM2_2(currentTemp2, targetTemp2);
  }
  // Serial . println ( PWM1 ) ;
  voltage1 = 3.3 * PWM1 / 4095;
  voltage2 = 3.3 * PWM2 / 4095;


  // Serial . println ( PWM2 ) ;
  if (currentTemp1 < targetTemp1) {
    ledcWrite(PWM1_Ch, PWM1);
    ledcWrite(PWM2_Ch, PWM2);

    // analogWrite ( transSensorPin1 , PWM2 ) ;
    // analogWrite ( transSensorPin2 , PWM2 /2) ;
    currentTemp1 = readTemp(tempSensorPin1);

    delay(500);
    currentTemp2 = readTemp(tempSensorPin2);
    Serial.print(count);
    Serial.print(" ,");
    Serial.print(con);
    Serial.print(" ,");
    Serial.print(currentTemp1);
    Serial.print(" ,");
    Serial.print(currentTemp2);
    Serial.print(" ,");
    Serial.print(voltage1);
    Serial.print(" ,");
    Serial.println(voltage2);
  } else {
    ledcWrite(PWM1_Ch, PWM1);
    ledcWrite(PWM2_Ch, PWM2);

    // analogWrite ( transSensorPin2 , PWM2 /2) ;
    currentTemp1 = readTemp(tempSensorPin1);

    delay(500);
    currentTemp2 = readTemp(tempSensorPin2);
    Serial.print(count);
    Serial.print(" ,");
    Serial.print(con);
    Serial.print(" ,");
    Serial.print(currentTemp1);
    Serial.print(" ,");
    Serial.print(currentTemp2);
    Serial.print(" ,");
    Serial.print(voltage1);
    Serial.print(" ,");
    Serial.println(voltage2);
  }
}
void state3() {
  // Serial . println ("3") ;
 targetTemp1 = 60;
  currentTemp1 = readTemp(tempSensorPin1);
  int PWM1;
  targetTemp2 = 50;
  currentTemp2 = readTemp(tempSensorPin2);
  int PWM2;
  if (con <4)
  {
    PWM1 = getRefPWM1_1(currentTemp1, targetTemp1);
    PWM2 = getRefPWM1_2(currentTemp2, targetTemp2);
  }
  else
  {
    PWM1 = getDisPWM2_1(currentTemp1, targetTemp1);
    PWM2 = getDisPWM2_2(currentTemp2, targetTemp2);
  }
  // Serial . println ( PWM1 ) ;
  voltage1 = 3.3 * PWM1 / 4095;
  voltage2 = 3.3 * PWM2 / 4095;

  if (currentTemp1 < targetTemp1) {
    ledcWrite(PWM1_Ch, PWM1);
    ledcWrite(PWM2_Ch, PWM2 );

    // analogWrite ( transSensorPin1 , PWM2 ) ;
    // analogWrite ( transSensorPin2 , PWM2 /2) ;
    currentTemp1 = readTemp(tempSensorPin1);

    delay(500);
    currentTemp2 = readTemp(tempSensorPin2);
    Serial.print(count);
    Serial.print(" ,");
    Serial.print(con);
    Serial.print(" ,");
    Serial.print(currentTemp1);
    Serial.print(" ,");
    Serial.print(currentTemp2);
    Serial.print(" ,");
    Serial.print(voltage1);
    Serial.print(" ,");
    Serial.println(voltage2);
  } else {
    ledcWrite(PWM1_Ch, PWM1);
    ledcWrite(PWM2_Ch, PWM2 );
    // analogWrite ( transSensorPin1 , 0) ;
    voltage1 = 3.3 * PWM1 / 4095;
    voltage2 = 3.3 * PWM2  / 4095;
    // analogWrite ( transSensorPin2 , PWM2 /2) ;
    currentTemp1 = readTemp(tempSensorPin1);

    delay(500);
    currentTemp2 = readTemp(tempSensorPin2);
    Serial.print(count);
    Serial.print(" ,");
    Serial.print(con);
    Serial.print(" ,");
    Serial.print(currentTemp1);
    Serial.print(" ,");
    Serial.print(currentTemp2);
    Serial.print(" ,");
    Serial.print(voltage1);
    Serial.print(" ,");
    Serial.println(voltage2);
  }
}

void state4() {
  // Serial . println ("4") ;
  targetTemp1 = 50;
  currentTemp1 = readTemp(tempSensorPin1);
  int PWM1;
  targetTemp2 = 40;
  currentTemp2 = readTemp(tempSensorPin2);
  int PWM2;
  if (con <4)
  {
    PWM1 = getRefPWM1_1(currentTemp1, targetTemp1);
    PWM2 = getRefPWM1_2(currentTemp2, targetTemp2);
  }
  else
  {
    PWM1 = getDisPWM2_1(currentTemp1, targetTemp1);
    PWM2 = getDisPWM2_2(currentTemp2, targetTemp2);
  }
  // Serial . println ( PWM1 ) ;
  voltage1 = 3.3 * PWM1 / 4095;
  voltage2 = 3.3 * PWM2 / 4095;

  if (currentTemp1 < targetTemp1) {
    ledcWrite(PWM1_Ch, PWM1);
    ledcWrite(PWM2_Ch, PWM2);
    // analogWrite ( transSensorPin1 , PWM1 ) ;
    // analogWrite ( transSensorPin2 , 0) ;
    currentTemp1 = readTemp(tempSensorPin1);

    delay(500);
    currentTemp2 = readTemp(tempSensorPin2);
    Serial.print(count);
    Serial.print(" ,");
    Serial.print(con);
    Serial.print(" ,");
    Serial.print(currentTemp1);
    Serial.print(" ,");
    Serial.print(currentTemp2);
    Serial.print(" ,");
    Serial.print(voltage1);
    Serial.print(" ,");
    Serial.println(voltage2);
  } else {
    ledcWrite(PWM1_Ch, PWM1);
    ledcWrite(PWM2_Ch, PWM2);

    // analogWrite ( transSensorPin1 , 0) ;
    // nalogWrite ( transSensorPin2 , 0) ;
    currentTemp1 = readTemp(tempSensorPin1);

    delay(500);
    currentTemp2 = readTemp(tempSensorPin2);
    Serial.print(count);
    Serial.print(" ,");
    Serial.print(con);
    Serial.print(" ,");
    Serial.print(currentTemp1);
    Serial.print(" ,");
    Serial.print(currentTemp2);
    Serial.print(" ,");
    Serial.print(voltage1);
    Serial.print(" ,");
    Serial.println(voltage2);
  }
}
void state5() {
  // Serial . println ("5") ;
  targetTemp1 = 30;
  currentTemp1 = readTemp(tempSensorPin1);
  int PWM1;
  targetTemp2 = 30;
  currentTemp2 = readTemp(tempSensorPin2);
  int PWM2;
  if (con <4)
  {
    PWM1 = getRefPWM1_1(currentTemp1, targetTemp1);
    PWM2 = getRefPWM1_2(currentTemp2, targetTemp2);
  }
  else
  {
    PWM1 = getDisPWM2_1(currentTemp1, targetTemp1);
    PWM2 = getDisPWM2_2(currentTemp2, targetTemp2);
  }
  // Serial . println ( PWM1 ) ;
  voltage1 = 3.3 * PWM1 / 4095;
  voltage2 = 3.3 * PWM2 / 4095;

  if (currentTemp1 < targetTemp1) {
    ledcWrite(PWM1_Ch, PWM1);
    ledcWrite(PWM2_Ch, PWM2);
    // analogWrite ( transSensorPin1 , PWM1 ) ;
    // analogWrite ( transSensorPin2 , 0) ;
    currentTemp1 = readTemp(tempSensorPin1);

    delay(500);
    currentTemp2 = readTemp(tempSensorPin2);
    Serial.print(con);
    Serial.print(" ,");
    Serial.print(c);
    Serial.print(" ,");
    Serial.print(currentTemp1);
    Serial.print(" ,");
    Serial.print(currentTemp2);
    Serial.print(" ,");
    Serial.print(voltage1);
    Serial.print(" ,");
    Serial.println(voltage2);
  } else {
    ledcWrite(PWM1_Ch, PWM1);
    ledcWrite(PWM2_Ch, PWM2);
    // analogWrite ( transSensorPin1 ,0 ) ;
    // analogWrite ( transSensorPin2 , 0) ;
    currentTemp1 = readTemp(tempSensorPin1);

    delay(500);
    currentTemp2 = readTemp(tempSensorPin2);
    Serial.print(con);
    Serial.print(" ,");
    Serial.print(c);
    Serial.print(" ,");
    Serial.print(currentTemp1);
    Serial.print(" ,");
    Serial.print(currentTemp2);
    Serial.print(" ,");
    Serial.print(voltage1);
    Serial.print(" ,");
    Serial.println(voltage2);
  }
}

float readTemp(int sensorPin) {
  int sensorVal = analogRead(sensorPin);
  float temp = (sensorVal * (3.3 / 4095.0) - 0.5) / 0.01;
  return temp;
}
