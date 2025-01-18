
bool bstate1 = false, bstate2 = false, bstate3 = false, bstate4 = false,
     bstate5 = false;


const int tempSensorPin1 = 2;
const int transSensorPin1 = 23;
const int tempSensorPin2 = 15;
const int transSensorPin2 = 21;

float currentTemp1 = 0.0;
float currentTemp2 = 0.0;
float targetTemp1;

const int heatingPower1 = 4096;
const int heatingPower2 = 3072;
const int heatingPower3 = 2048;
const int heatingPower4 = 1024;

float voltage1;
float voltage2;

const int button1 = 0, button2 = 35;
int count = 1, con = 7;
String c, states;

float r_Kp, r_Ki, r_Kd;
float d_Kp, d_Ki, d_Kd;
float error, lasterror, integral;
unsigned long previousMillis = 0;

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

  lasterror = 0;
  integral = 0;

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

void nextState() {
  


 
}
void Controller() {
  unsigned long currentMillis = millis();
  // Serial.println("Button pressed");

  if (currentMillis - lastButtonPress > debounceDelay) 
  {  
    // Serial.println("Inside button pressed");
    // Serial.println(con);
    con += 1;

    if (con > 8)
    {
      con =1 ;
      count ++;
    }
    

    if (count >4)
    {
      con =1 ;
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
  if (con > 8) {
    con = 1;
  }
  if (con == 1) {
    r_Kp = 1.1332;
    r_Kd = 0;
    r_Ki = 0;
    c = " Ref P ";
  } else if (con == 2 ) {
    r_Kp = 1.864547036007359;
    r_Kd = 5.577163121352382;
    r_Ki = 0;
    c = " Ref PD ";
  } else if (con == 3) {
    r_Kp = 0.178558558781629;
    r_Kd = 0;
    r_Ki = 0.005249026784141;
    c = " Ref PI ";
  } else if (con == 4 ) {
    r_Kp = 0.1786;
    r_Kd = 1.3225;
    r_Ki = 0.006;
    c = " Ref PID ";
  } 
  else if (con == 5 ) {
    d_Kp = 1.1332;
    d_Kd = 0;
    d_Ki = 0;
    c = " Dis P ";
  } else if (con == 6 ) {
    d_Kp = 1.864547036007359;
    d_Kd = 5.577163121352382;
    d_Ki = 0;
    c = " Dis PD ";
  } else if (con == 7 ) {
    d_Kp = 0.178558558781629;
    d_Kd = 0;
    d_Ki = 0.005249026784141;
    c = " Dis PI ";
  } else if (con == 8 ) {
    d_Kp = 0.1786;
    d_Kd = 1.3225;
    d_Ki = 0.006;
    c = " Dis PID ";
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

int getPWM1(float currentTemp, float target) {
  int PWM;
  unsigned long currentMillis = millis();
  float elapsedTime = (currentMillis - previousMillis) / 1000.0;

  error = target - currentTemp;
  integral += (error * elapsedTime) * r_Ki;

  // Limit the integral to avoid windup
  float maxIntegral = 1000;  // Set an appropriate max limit
  float minIntegral = -1000; // Set an appropriate min limit
  if (integral > maxIntegral) {
    integral = maxIntegral;
  }
  if (integral < minIntegral) {
    integral = minIntegral;
  }

  float derivative = 0;
  if (elapsedTime > 0) {
    derivative = (error - lasterror) / elapsedTime;
  }

  float pidOutput = r_Kp * error + integral + r_Kd * derivative;

  // Scale to PWM range
  PWM = (pidOutput / 3.3) * 4095;

  // Limit PWM value
  if (PWM > 4095) {
    PWM = 4095;
  }
  if (PWM <1000) {
    PWM = 1000;
  }

  lasterror = error;
  previousMillis = currentMillis;

  return PWM;
}


int getPWM2(float currentTemp, float target) {
  int PWM;
  unsigned long currentMillis = millis();
  float elapsedTime = (currentMillis - previousMillis) / 1000.0;

  error = target - currentTemp;
  integral += (error * elapsedTime) * d_Ki;

  // Limit the integral to avoid windup
  float maxIntegral = 1000;  // Set an appropriate max limit
  float minIntegral = -1000; // Set an appropriate min limit
  if (integral > maxIntegral) {
    integral = maxIntegral;
  }
  if (integral < minIntegral) {
    integral = minIntegral;
  }

  float derivative = 0;
  if (elapsedTime > 0) {
    derivative = (error - lasterror) / elapsedTime;
  }

  float pidOutput = d_Kp * error + integral + d_Kd * derivative;

  // Scale to PWM range
  PWM = (pidOutput / 3.3) * 4095;

  // Limit PWM value
  if (PWM > 4095) {
    PWM = 4095;
  }
  if (PWM <1000) {
    PWM = 1000;
  }

  lasterror = error;
  previousMillis = currentMillis;

  return PWM;

}

void state1() {

  targetTemp1 = 35;
  currentTemp1 = readTemp(tempSensorPin1);
  int PWM1;
  if (con <4)
  {
    PWM1 = getPWM1(currentTemp1, targetTemp1);
  }
  else
  {
    PWM1 = getPWM2(currentTemp1, targetTemp1);
  }
  // Serial . println ( PWM1 ) ;
  voltage1 = 3.3 * PWM1 / 4095;
  voltage2 = 3.3 * 0 / 4095;

  if (currentTemp1 < targetTemp1) {
    ledcWrite(PWM1_Ch, PWM1);
    ledcWrite(PWM2_Ch, 0);
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
    ledcWrite(PWM2_Ch, 0);
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
  targetTemp1 = 50;
  currentTemp1 = readTemp(tempSensorPin1);
  int PWM1;
  if (con <4)
  {
    PWM1 = getPWM1(currentTemp1, targetTemp1);
  }
  else
  {
    PWM1 = getPWM2(currentTemp1, targetTemp1);
  }
  voltage1 = 3.3 * PWM1 / 4095;
  voltage2 = 3.3 * 0;

  // Serial . println ( PWM2 ) ;
  if (currentTemp1 < targetTemp1) {
    ledcWrite(PWM1_Ch, PWM1);
    ledcWrite(PWM2_Ch, 0);

    voltage1 = 3.3 * PWM1 / 4095;
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
    ledcWrite(PWM2_Ch, 0);
    // analogWrite ( transSensorPin1 , 0) ;
    voltage1 = 3.3 * PWM1 / 4095;
    voltage2 =0 ;

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
  targetTemp1 = 50;
  currentTemp1 = readTemp(tempSensorPin1);
  int PWM2;
  if (con <4)
  {
    PWM2 = getPWM1(currentTemp1, targetTemp1);
  }
  else
  {
    PWM2 = getPWM2(currentTemp1, targetTemp1);
  }
  voltage1 = 3.3 * PWM2 / 4095;
  voltage2 = 3.3 * PWM2 / 2 / 4095;
  if (currentTemp1 < targetTemp1) {
    ledcWrite(PWM1_Ch, PWM2);
    ledcWrite(PWM2_Ch, PWM2 / 2);

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
    ledcWrite(PWM1_Ch, PWM2);
    ledcWrite(PWM2_Ch, PWM2 / 2);
    // analogWrite ( transSensorPin1 , 0) ;
    voltage1 = 3.3 * PWM2 / 4095;
    voltage2 = 3.3 * PWM2 / 2 / 4095;
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
  targetTemp1 = 40;
  currentTemp1 = readTemp(tempSensorPin1);
  int PWM1;
  if (con <4)
  {
    PWM1 = getPWM1(currentTemp1, targetTemp1);
  }
  else
  {
    PWM1 = getPWM2(currentTemp1, targetTemp1);
  }
  voltage1 = 3.3 * PWM1 / 4095;
  voltage2 = 3.3 * 0 / 4095;
  if (currentTemp1 < targetTemp1) {
    ledcWrite(PWM1_Ch, PWM1);
    ledcWrite(PWM2_Ch, 0);
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
    ledcWrite(PWM2_Ch, 0);
    voltage1 = 3.3 * PWM1 / 4095;
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
  targetTemp1 = 35;
  currentTemp1 = readTemp(tempSensorPin1);
  int PWM1 = getPWM1(currentTemp1, targetTemp1);
  voltage1 = 5 * PWM1 / 254;
  voltage2 = 5 * 0 / 254;
  if (currentTemp1 < targetTemp1) {
    ledcWrite(PWM1_Ch, PWM1);
    ledcWrite(PWM2_Ch, 0);
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
    ledcWrite(PWM1_Ch, 0);
    ledcWrite(PWM2_Ch, 0);
    voltage1 = 0;
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
