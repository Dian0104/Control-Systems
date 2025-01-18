

bool bstate1=false,bstate2= false,bstate3 = true,bstate4= false,bstate5 = false;

const int tempSensorPin1 = 2;
const int transSensorPin1 = 23;

const int tempSensorPin2 = 15;
const int transSensorPin2 = 21;

float currentTemp1 = 0.0;
float currentTemp2 = 0.0;

float targetTemp1 ;
float targetTemp2;


const int heatingPower1 = 4096; 
const int heatingPower2 = 3072;
const int heatingPower3 = 2048; 
const int heatingPower4 = 1024;

float voltage1;
float voltage2;

const int button1 =0 ; 
int count = 0,con;
String c ,states;

float r_Kp , r_Ki, r_Kd ;
float d_Kp , d_Ki, d_Kd ;

float error, lasterror,integral;
unsigned long previousMillis = 0;

bool pressed ;

#define PWM1_Freq 5000
#define PWM1_Res 8
#define PWM1_Ch 0

#define PWM2_Freq 5000
#define PWM2_Res 8
#define PWM2_Ch 1



void setup() {
  Serial.begin(9600);
  


  // Set up PWM
  ledcSetup(PWM1_Ch, PWM1_Freq, PWM1_Res);
  ledcAttachPin(transSensorPin1, PWM1_Ch);
  
  ledcSetup(PWM2_Ch, PWM2_Freq, PWM2_Res);
  ledcAttachPin(transSensorPin2, PWM2_Ch);

  pinMode(button1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(button1), nextState, RISING);

}

void nextState()
{
  
  if (count ==3 )
  {
    count = 1;
  }
  else
  {
    count += 1;
  }

  if (count ==1 )
    {

      states ="1";
      bstate1 =true;
      bstate2 =false;
      bstate3 =false;
      bstate4 =false;
      bstate5 =false;
     

    }
    else if (count ==2)
    {
      bstate1 =false;
      bstate2 =true;
      bstate3 =false;
      bstate4 =false;
      bstate5 =false;
      states ="2";
      
    }
    else if (count == 3)
    {
      bstate1 =false;
      bstate2 =false;
      bstate3 =true;
      bstate4 =false;
      bstate5 =false;
    }

    

}

void Controller()
{
  pressed = true;
  
}

int getPWM1(float currentTemp, float target)
{
  int PWM = (target - currentTemp)*3.3/4096;



  return PWM;

}

int getPWM2(float currentTemp, float target)
{
  int PWM = (target - currentTemp)*3.3/4096;
  


  return PWM;

}

void loop() {
  checkMessage();

  if (pressed)
  {
    con+=1;
    pressed = false;
  }

  if (con ==1)
  {
    
    c = "T1";
  }
  else if (con ==2)
  {
    
    c = "T2";

  }

  if (bstate1)
  {
    State1();
  }
  else if(bstate2)
  {
    State2();
  }
  else if (bstate3)
  {
    State3();
  }


}

void checkMessage()
{
  if (Serial.available())
  {
    
    char data = Serial.read();

    // Serial.print(data);

    if (data == '1')
    {
      bstate1 = true;

      bstate2 = false;

      bstate3 = false;
    }
    else if (data == '2')
    {
      bstate2 = true;
      bstate3 = false;
      bstate1 = false;
    }else if (data == '3')
    {
      bstate3 = true;
      bstate1 = false;
      bstate2 = false;
    }
  }
}

void State1()
{
  targetTemp1 = 60;

  currentTemp1 = readTemp(tempSensorPin1);

  
  int PWM1 = getPWM1(currentTemp1,targetTemp1);

  //Serial.println(PWM1);
  voltage1 = 5;
  voltage2 = 0;
  ledcWrite(PWM2_Ch,0);
  
  if (currentTemp1<targetTemp1)
  {    

    ledcWrite(PWM1_Ch,4096);
  
    //analogWrite(transSensorPin1, PWM1);
    //analogWrite(transSensorPin2, 0);
    currentTemp1 = readTemp(tempSensorPin1);
    currentTemp2 = readTemp(tempSensorPin2);
   
    delay(500);
    Serial.print(currentTemp1);
    Serial.print(",");
    Serial.print(currentTemp2);
    Serial.print(",");
    Serial.print(voltage1, 2);
    Serial.print(",");
    Serial.println(voltage2, 2);



  }
  else
  {
    ledcWrite(PWM1_Ch,0);
    voltage1 =0;
  
    //analogWrite(transSensorPin1, PWM1);
    //analogWrite(transSensorPin2, 0);
    currentTemp1 = readTemp(tempSensorPin1);
    currentTemp2 = readTemp(tempSensorPin2);
   
    delay(500);
    Serial.print(currentTemp1);
    Serial.print(",");
    Serial.print(currentTemp2);
    Serial.print(",");
    Serial.print(voltage1, 2);
    Serial.print(",");
    Serial.println(voltage2, 2);
  }

}


void State2()
{
  targetTemp2 = 60;

  currentTemp2 = readTemp(tempSensorPin2);

  
  int PWM2 = getPWM1(currentTemp2,targetTemp2);
  ledcWrite(PWM1_Ch,0);

  //Serial.println(PWM1);
  voltage2 = 5;
  
  if (currentTemp2<targetTemp2)
  {    


    ledcWrite(PWM2_Ch,4096);
    
    //analogWrite(transSensorPin1, PWM1);
    //analogWrite(transSensorPin2, 0);
  
    currentTemp1 = readTemp(tempSensorPin1);
    currentTemp2 = readTemp(tempSensorPin2);
    voltage1 =0;
   
    
    delay(500);

    Serial.print(currentTemp1);
    Serial.print(",");
    Serial.print(currentTemp2);
    Serial.print(",");
    Serial.print(voltage1, 2);
    Serial.print(",");
    Serial.println(voltage2, 2);



  }
  else
  {
    ledcWrite(PWM2_Ch,0);
    voltage2 = 0;
  
    //analogWrite(transSensorPin1, PWM1);
    //analogWrite(transSensorPin2, 0);
    currentTemp1 = readTemp(tempSensorPin1);
    currentTemp2 = readTemp(tempSensorPin2);
   
    delay(500);
    Serial.print(currentTemp1);
    Serial.print(",");
    Serial.print(currentTemp2);
    Serial.print(",");
    Serial.print(voltage1, 2);
    Serial.print(",");
    Serial.println(voltage2, 2);
  }
}


void State3()
{
  targetTemp2 = 40;

  currentTemp2 = readTemp(tempSensorPin2);

  
  int PWM2 = getPWM1(currentTemp2,targetTemp2);

  //Serial.println(PWM1);
  voltage1 =0 ;
  voltage2 = 0;
  ledcWrite(PWM1_Ch,0);
  ledcWrite(PWM2_Ch,0);
  
  if (currentTemp1<targetTemp1)
  {    


    
    
    currentTemp1 = readTemp(tempSensorPin1);
    currentTemp2 = readTemp(tempSensorPin2);
   
    
    delay(500);

    Serial.print(currentTemp1);
    Serial.print(",");
    Serial.print(currentTemp2);
    Serial.print(",");
    Serial.print(voltage1, 2);
    Serial.print(",");
    Serial.println(voltage2, 2);



  }
}



float readTemp(int sensorPin)
{
  int sensorVal = analogRead(sensorPin);
  float temp = (sensorVal* (3.3 / 4095.0) -0.5)/0.01;
  return temp;
}














