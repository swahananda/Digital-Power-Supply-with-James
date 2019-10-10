

#include "Wire.h" 
/* 
* If using standard pins on LCD without interface
* #include "LiquidCrystal.h"                                 
* const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2; (if using non-I2C interfaced LCD's)
* LiquidCrystal lcd(rs, en, d4, d5, d6, d7);                     
*/
#include <LiquidCrystal_I2C.h>                        
LiquidCrystal_I2C lcd(0x27,20,4);             //Hex address is typically 0x27 or 0x3f

//Inputs/outputs
int voltage_in = A2;
int set_voltage_in = A1;
int set_current_in = A3;
int current_in = A0;
int PWM_PIN = 13;


//Editable variables
float real_curren_offset = -0.03;
//int Delay= 1000; defaul delay in control of refresh
int Delay= 50;
//float max_output_voltage= 15.0;
float max_output_voltage= 33.0;


//Other Variables
int pwm_value = 1;
float set_voltage = 4.30;
float serial_lecture = 0;
float real_output = 0;
float real_output2 = 0;
float RawValue= 0;
float real_current = 0;
float real_current_ma = 0;
float set_volt = 0;
float set_curr= 0;
//int pwm_display = ((pwm_value / 255) * 100); //pwm duty cycle display percentage
unsigned long previousMillis = 0; 
unsigned long currentMillis = 0;
float map_divider = 0;





//THIS FUNCTION WILL MAP THE float VALUES IN THE GIVEN RANGE
float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void setup() {



//lcd.begin(20, 4); 
  lcd.init(); 
  lcd.backlight(); 
  pinMode(voltage_in,INPUT);
  pinMode(set_voltage_in,INPUT);
  pinMode(set_current_in,INPUT);
  pinMode(current_in,INPUT);
  pinMode(PWM_PIN,OUTPUT);
  analogWriteFrequency(13, 280000);
  //TCCR2B = TCCR2B & B11111000 | B00000001;      // pin 3 PWM frequency of 31372.55 Hz
  Serial.begin(9600);
  currentMillis = millis();
  map_divider = 1024.00/max_output_voltage;    //I want maximum voltage of 15V in my case. SO 1024 digital read divided by 15 =  68.2
}

void loop() {
  set_voltage = analogRead(set_voltage_in)/map_divider;          //map_divider = 69.2 in my case Why 68.2? Well: I want maximum voltage of 15V. SO 1024 digital read divided by 15 =  68.2
  set_curr = analogRead(set_current_in)/1.024;                   //Why divided by 1.024? Well: I want maximum current of 1000mA. SO 1024 digital read divided by 1000mA =  1.024
  RawValue = analogRead(current_in);                             //Read the feedback for current from the MAX471 sensor
  real_current = (RawValue * 5.0 )/ 1024.0;                      //Scale the ADC, we get current value in Amps
  real_current = real_current - real_curren_offset;              //Substract the current error. Make tests in order to find the real_curren_offset value, in my case an error of -0.03A
  real_output= analogRead(voltage_in);                           //We read the feedback voltage (0 - 1024)
  real_output2 = real_output/map_divider;                        //Divide by 69.2 and we get range to 15V
  
  real_current_ma = real_current*1000;                           //We pass from A to mA
  //real_current_ma = constrain(real_current_ma,0,2000); OG value
  real_current_ma = constrain(real_current_ma,0,150); //test value based on Teensy 150mA 3.3V limit  


  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //If the set current value is higher than the feedback current value, we make normal control of output voltage
  if(real_current_ma < set_curr)
  {
    //Now, if set voltage is higher than real value from feedback, we decrease PWM width till we get same value
    if(set_voltage > real_output2)
    {
      pwm_value = pwm_value - 1;                  //When we decrease PWM width, the volage gets higher at the output. 
      pwm_value = constrain(pwm_value, 0, 255);
    }
    //If set voltage is lower than real value from feedback, we increase PWM width till we get same value
    if(set_voltage < real_output2)
    {
      pwm_value = pwm_value + 1;                  //When we increase PWM width, the volage gets lower at the output. 
      pwm_value = constrain(pwm_value, 0, 255);
    }  
  }//end of real_current_ma < set_curr
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /*if the set current value is lower than the feedback current value, that means we need to lower the voltage at the output
  in order to amintain the same current value*/
  if(real_current_ma > set_curr)
  {
    pwm_value = pwm_value + 1;                  //When we increase PWM width, the volage gets lower at the output. 
    pwm_value = constrain(pwm_value, 0, 255);
  }
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //We write PWM value on PWM pin out
  analogWrite(PWM_PIN,pwm_value);
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  
  //Each Delay value we print values on the LCD screen
  currentMillis = millis();
  if(currentMillis - previousMillis >= Delay){  
    previousMillis += Delay;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(" VOLTAGE    CURRENT ");

    lcd.setCursor(0,1);
    lcd.print(set_voltage,0);
    lcd.print("V       ");

    lcd.print(set_curr,0);
    lcd.print("mA");

    lcd.setCursor(0,3);
    lcd.print(real_output2,1);
    lcd.print("V       ");

    lcd.print(real_current_ma,0);
    //lcd.setCursor(19,1);
    lcd.print("mA      ");

    //lcd.print(pwm_display,0);
    //lcd.print("%");

    
  }//end of currentMillis - previousMillis >= Delay
}//end of void loop
