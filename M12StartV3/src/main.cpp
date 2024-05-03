#include <Arduino.h>
#include <Adafruit_MPU6050.h> //
#include <Adafruit_Sensor.h> //
#include <Wire.h> //I2C komunikacija prema MPU6050
#include <PID_v1.h> //PID regulacija s ugrađdženim errorima
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

//
//----------------------------------------------------------------
//
//DEFINES:
//
#define MOTOR_PIN_2_A 18
#define MOTOR_PIN_2_B 19
#define MOTOR_PIN_1_A 32
#define MOTOR_PIN_1_B 33
#define MOTOR_PIN_4_A 25
#define MOTOR_PIN_4_B 26
#define MOTOR_PIN_3_A 27
#define MOTOR_PIN_3_B 13


const char* ssid = "Kunovicdrono";
const char* password = "12345678";

#define BOOT_PIN 0  // Using GPIO0, but ensure it's safe to do so

bool motorsEnabled = false; // Initially, motors are enabled
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers


int pwm1=0;
int pwm2=0;
int pwm3 = 0;
int pwm4=0;

double InputX,InputY,InputZ; //kut
double OutputX,OutputY,OutputZ; //napon prema motorima (0-255)

// Declare PID parameters for each axis
double KpX, KiX, KdX, SetPointX;
double KpY, KiY, KdY, SetPointY;
double KpZ, KiZ, KdZ, SetPointZ;

// Create instances of PID for each axis - assumed to be defined and initialized elsewhere in your code
PID myPIDX(&InputX, &OutputX, &SetPointX, KpX, KiX, KdX, DIRECT);
PID myPIDY(&InputY, &OutputY, &SetPointY, KpY, KiY, KdY, DIRECT);
PID myPIDZ(&InputZ, &OutputZ, &SetPointZ, KpZ, KiZ, KdZ, DIRECT);

Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;

AsyncWebServer server(80);

const char index_html[] PROGMEM = R"rawliteral(

</body></html><!DOCTYPE HTML>
<html>
<body>

 <h3>Motor Control for Testing</h3>
  PWM1: <input type="range" min="-512" max="512" step="1" id="pwm1" value="0"><span id="pwm1Val">0</span><br>
  PWM2: <input type="range" min="-512" max="512" step="1" id="pwm2" value="0"><span id="pwm2Val">0</span><br>
  PWM3: <input type="range" min="-512" max="512" step="1" id="pwm3" value="0"><span id="pwm3Val">0</span><br>
  PWM4: <input type="range" min="-512" max="512" step="1" id="pwm4" value="0"><span id="pwm4Val">0</span><br>

  <h3>X Axis</h3>
  KpX: <input type="range" min="0" max="6" step="0.1" id="KpX" value="2.5"><span id="KpXVal">2.5</span><br>
  KiX: <input type="range" min="0" max="2" step="0.1" id="KiX" value="1"><span id="KiXVal">1</span><br>
  KdX: <input type="range" min="0" max="2" step="0.1" id="KdX" value="0"><span id="KdXVal">0</span><br>
  SetPointX: <input type="range" min="-30" max="30" step="1" id="SetPointX" value="0"><span id="SpXVal">0</span><br>

  <h3>Y Axis</h3>
  KpY: <input type="range" min="0" max="6" step="0.1" id="KpY" value="2.5"><span id="KpYVal">2.5</span><br>
  KiY: <input type="range" min="0" max="2" step="0.1" id="KiY" value="1"><span id="KiYVal">1</span><br>
  KdY: <input type="range" min="0" max="2" step="0.1" id="KdY" value="0"><span id="KdYVal">0</span><br>
  SetPointY: <input type="range" min="-30" max="30" step="1" id="SetPointY" value="0"><span id="SpYVal">0</span><br>

  <h3>Z Axis</h3>
  KpZ: <input type="range" min="0" max="6" step="0.1" id="KpZ" value="2.5"><span id="KpZVal">2.5</span><br>
  KiZ: <input type="range" min="0" max="2" step="0.1" id="KiZ" value="1"><span id="KiZVal">1</span><br>
  KdZ: <input type="range" min="0" max="2" step="0.1" id="KdZ" value="0"><span id="KdZVal">0</span><br>
  SetPointZ: <input type="range" min="-30" max="30" step="1" id="SetPointZ" value="0"><span id="SpZVal">0</span><br>

  <script>
  function setupSlider(sliderId, valueId, urlPath) {
    var slider = document.getElementById(sliderId);
    var valueSpan = document.getElementById(valueId);
    slider.oninput = function() {
      fetch('/' + urlPath + '?value=' + this.value);
      valueSpan.textContent = this.value;
    }
  }

 setupSlider("pwm1", "pwm1Val", "setPWM1");
  setupSlider("pwm2", "pwm2Val", "setPWM2");
  setupSlider("pwm3", "pwm3Val", "setPWM3");
  setupSlider("pwm4", "pwm4Val", "setPWM4");


  setupSlider("KpX", "KpXVal", "setKpX");
  setupSlider("KiX", "KiXVal", "setKiX");
  setupSlider("KdX", "KdXVal", "setKdX");
  setupSlider("SetPointX", "SpXVal", "setSetPointX");

  setupSlider("KpY", "KpYVal", "setKpY");
  setupSlider("KiY", "KiYVal", "setKiY");
  setupSlider("KdY", "KdYVal", "setKdY");
  setupSlider("SetPointY", "SpYVal", "setSetPointY");

  setupSlider("KpZ", "KpZVal", "setKpZ");
  setupSlider("KiZ", "KiZVal", "setKiZ");
  setupSlider("KdZ", "KdZVal", "setKdZ");
    setupSlider("SetPointZ", "SpZVal", "setSetPointZ");
  </script>

)rawliteral";




void setup(void) {
  
  pinMode(MOTOR_PIN_1_A,OUTPUT);
  pinMode(MOTOR_PIN_1_B,OUTPUT);
  pinMode(MOTOR_PIN_2_A,OUTPUT);
  pinMode(MOTOR_PIN_2_B,OUTPUT);
  pinMode(MOTOR_PIN_3_A,OUTPUT);
  pinMode(MOTOR_PIN_3_B,OUTPUT);
  pinMode(MOTOR_PIN_4_A,OUTPUT);
  pinMode(MOTOR_PIN_4_B,OUTPUT);

  analogWriteFrequency(55000);

  analogWrite(MOTOR_PIN_1_A,65);
  analogWrite(MOTOR_PIN_1_B,0);
   delay(200);
  analogWrite(MOTOR_PIN_2_A,65);
  analogWrite(MOTOR_PIN_2_B,0);
   delay(200);
  analogWrite(MOTOR_PIN_3_A,65);
  analogWrite(MOTOR_PIN_3_B,0);
   delay(200);
  analogWrite(MOTOR_PIN_4_A,65);
  analogWrite(MOTOR_PIN_4_B,0);
  delay(200);
  analogWrite(MOTOR_PIN_1_A,0);
  analogWrite(MOTOR_PIN_1_B,0);
  analogWrite(MOTOR_PIN_2_A,0);
  analogWrite(MOTOR_PIN_2_B,0);
  analogWrite(MOTOR_PIN_3_A,0);
  analogWrite(MOTOR_PIN_3_B,0);
  analogWrite(MOTOR_PIN_4_A,0);
  analogWrite(MOTOR_PIN_4_B,0);
  
  mpu.begin();
  Serial.begin(115200);

  WiFi.softAP(ssid, password);  //dodati
  Serial.println();
  Serial.print("IP address: ");   //dodati
  Serial.println(WiFi.softAPIP());  
    
  ////////////////////////////////////////////////////////////////
  //
  // SERVER:
  //
  
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });

 // PWM control routes
  server.on("/setPWM1", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("value")) {
      pwm1 = request->getParam("value")->value().toInt();
      analogWrite(MOTOR_PIN_1_A, pwm1);
      request->send(200, "text/plain", "PWM1 set to: " + String(pwm1));
    } else {
      request->send(200, "text/plain", "No value sent for PWM1");
    }
  });

  server.on("/setPWM2", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("value")) {
      pwm2 = request->getParam("value")->value().toInt();
      analogWrite(MOTOR_PIN_2_A, pwm2);
      request->send(200, "text/plain", "PWM2 set to: " + String(pwm2));
    } else {
      request->send(200, "text/plain", "No value sent for PWM2");
    }
  });

  server.on("/setPWM3", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("value")) {
      pwm3 = request->getParam("value")->value().toInt();
      analogWrite(MOTOR_PIN_3_A, pwm3);
      request->send(200, "text/plain", "PWM3 set to: " + String(pwm3));
    } else {
      request->send(200, "text/plain", "No value sent for PWM3");
    }
  });

  server.on("/setPWM4", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("value")) {
      pwm4 = request->getParam("value")->value().toInt();
      analogWrite(MOTOR_PIN_4_A, pwm4);
      request->send(200, "text/plain", "PWM4 set to: " + String(pwm4));
    } else {
      request->send(200, "text/plain", "No value sent for PWM4");
    }
  });


    // Handlers for X axis
  server.on("/setKpX", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("value")) {
      KpX = request->getParam("value")->value().toFloat();
      myPIDX.SetTunings(KpX, KiX, KdX);
      request->send(200, "text/plain", "KpX changed to: " + String(KpX));
    } else {
      request->send(200, "text/plain", "No value sent for KpX");
    }
  });

  server.on("/setKiX", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("value")) {
      KiX = request->getParam("value")->value().toFloat();
      myPIDX.SetTunings(KpX, KiX, KdX);
      request->send(200, "text/plain", "KiX changed to: " + String(KiX));
    } else {
      request->send(200, "text/plain", "No value sent for KiX");
    }
  });

  server.on("/setKdX", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("value")) {
      KdX = request->getParam("value")->value().toFloat();
      myPIDX.SetTunings(KpX, KiX, KdX);
      request->send(200, "text/plain", "KdX changed to: " + String(KdX));
    } else {
      request->send(200, "text/plain", "No value sent for KdX");
    }
  });

  server.on("/setSetPointX", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("value")) {
      SetPointX = request->getParam("value")->value().toFloat();
      request->send(200, "text/plain", "SetPointX changed to: " + String(SetPointX));
    } else {
      request->send(200, "text/plain", "No value sent for SetPointX");
    }
  });

  // Repeat the setup for Ki, Kd, SetPoint for Y and Z axes similarly

  // Handlers for Y axis
  server.on("/setKpY", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("value")) {
      KpY = request->getParam("value")->value().toFloat();
      myPIDY.SetTunings(KpY, KiY, KdY);
      request->send(200, "text/plain", "KpY changed to: " + String(KpY));
    } else {
      request->send(200, "text/plain", "No value sent for KpY");
    }
  });

  server.on("/setKiY", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("value")) {
      KiY = request->getParam("value")->value().toFloat();
      myPIDY.SetTunings(KpY, KiY, KdY);
      request->send(200, "text/plain", "KiY changed to: " + String(KiY));
    } else {
      request->send(200, "text/plain", "No value sent for KiY");
    }
  });

  server.on("/setKdY", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("value")) {
      KdY = request->getParam("value")->value().toFloat();
      myPIDY.SetTunings(KpY, KiY, KdY);
      request->send(200, "text/plain", "KdY changed to: " + String(KdY));
    } else {
      request->send(200, "text/plain", "No value sent for KdY");
    }
  });

  server.on("/setSetPointY", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("value")) {
      SetPointY = request->getParam("value")->value().toFloat();
      request->send(200, "text/plain", "SetPointY changed to: " + String(SetPointY));
    } else {
      request->send(200, "text/plain", "No value sent for SetPointY");
    }
  });

  // Handlers for Z axis
  server.on("/setKpZ", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("value")) {
      KpZ = request->getParam("value")->value().toFloat();
      myPIDZ.SetTunings(KpZ, KiZ, KdZ);
      request->send(200, "text/plain", "KpZ changed to: " + String(KpZ));
    } else {
      request->send(200, "text/plain", "No value sent for KpZ");
    }
  });

  server.on("/setKiZ", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("value")) {
      KiZ = request->getParam("value")->value().toFloat();
      myPIDZ.SetTunings(KpZ, KiZ, KdZ);
      request->send(200, "text/plain", "KiZ changed to: " + String(KiZ));
    } else {
      request->send(200, "text/plain", "No value sent for KiZ");
    }
  });

  server.on("/setKdZ", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("value")) {
      KdZ = request->getParam("value")->value().toFloat();
      myPIDZ.SetTunings(KpZ, KiZ, KdZ);
      request->send(200, "text/plain", "KdZ changed to: " + String(KdZ));
    } else {
      request->send(200, "text/plain", "No value sent for KdZ");
    }
  });

  server.on("/setSetPointZ", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("value")) {
      SetPointZ = request->getParam("value")->value().toFloat();
      request->send(200, "text/plain", "SetPointZ changed to: " + String(SetPointZ));
    } else {
      request->send(200, "text/plain", "No value sent for SetPointZ");
    }
  });





  server.begin();




  while (!Serial)
    delay(10);

  Serial.println("Adafruit MPU6050 test!");



  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  
  Serial.println("MPU6050 Found!");
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  // rest of your setup code...
  // ...

  // Initialize PID Controller for X axis
  SetPointX = 0; // Initial setpoint for X axis
  myPIDX.SetMode(AUTOMATIC); // Turn the PID on
  myPIDX.SetOutputLimits(-255, 255); // Set output limits for PWM output range

  // Initialize PID Controller for Y axis
  SetPointY = 0; // Initial setpoint for Y axis
  myPIDY.SetMode(AUTOMATIC); // Turn the PID on
  myPIDY.SetOutputLimits(-255, 255); // Set output limits for PWM output range

  // Initialize PID Controller for Z axis
  SetPointZ = 0; // Initial setpoint for Z axis
  myPIDZ.SetMode(AUTOMATIC); // Turn the PID on
  myPIDZ.SetOutputLimits(-255, 255); // Set output limits for PWM output range
  
  
}

void loop() {
  /* Get new sensor events with the readings */


  mpu.getEvent(&a, &g, &temp);

  InputX = -(atan(a.acceleration.z / a.acceleration.x)) * 180 / PI ;
  InputY = -(atan(a.acceleration.y / a.acceleration.x)) * 180 / PI ;
  InputZ = g.gyro.z; // Assuming Z axis input is the gyro z-axis data


  // PID computation and Motor control
  bool calculatedX = myPIDX.Compute();
  bool calculatedY = myPIDY.Compute();
  bool calculatedZ = myPIDZ.Compute();

// Calculate motor outputs based on PID outputs
  // The specific equations will depend on how the outputs should be applied to the motors
  // Below is a general approach, assuming OutputX and OutputY control roll and pitch, respectively, and OutputZ controls yaw

  int motorSpeed1 = constrain(pwm1 - OutputX + OutputY + OutputZ, 0, 255); 
  int motorSpeed2 = constrain(pwm2 - OutputX - OutputY - OutputZ, 0, 255); 
  int motorSpeed3 = constrain(pwm3 + OutputX + OutputY - OutputZ, 0, 255); 
  int motorSpeed4 = constrain(pwm4 + OutputX - OutputY + OutputZ, 0, 255); 

  if (!digitalRead(BOOT_PIN)) {
    motorsEnabled=!motorsEnabled;
    analogWrite(MOTOR_PIN_2_A, 0);
    analogWrite(MOTOR_PIN_3_A, 0);
    analogWrite(MOTOR_PIN_1_A, 0);
    analogWrite(MOTOR_PIN_4_A, 0);
    delay(300);
    if (motorsEnabled){
    // Write motor speeds to the ESCs
    analogWrite(MOTOR_PIN_1_A, 65);
    analogWrite(MOTOR_PIN_2_A, 0);
    analogWrite(MOTOR_PIN_3_A, 0);
    analogWrite(MOTOR_PIN_4_A, 0);
    delay(300);
  }else{
    analogWrite(MOTOR_PIN_1_A, 0);
    analogWrite(MOTOR_PIN_2_A, 65);
    analogWrite(MOTOR_PIN_3_A, 0);
    analogWrite(MOTOR_PIN_4_A, 0);
    delay(300);
  }
  }

  if (motorsEnabled){
    // Write motor speeds to the ESCs
    analogWrite(MOTOR_PIN_1_A, motorSpeed1);
    analogWrite(MOTOR_PIN_2_A, motorSpeed2);
    analogWrite(MOTOR_PIN_3_A, motorSpeed3);
    analogWrite(MOTOR_PIN_4_A, motorSpeed4);
  }else{
    analogWrite(MOTOR_PIN_2_A, 0);
    analogWrite(MOTOR_PIN_3_A, 0);
    analogWrite(MOTOR_PIN_1_A, 0);
    analogWrite(MOTOR_PIN_4_A, 0);
  }

  // Debugging outputs
  Serial.print("MotorSpeed1: "); Serial.print(motorSpeed1);
  Serial.print(", MotorSpeed2: "); Serial.print(motorSpeed2);
  Serial.print(", MotorSpeed3: "); Serial.print(motorSpeed3);
  Serial.print(", MotorSpeed4: "); Serial.print(motorSpeed4);


/* Print out the values */
  Serial.print("Roll: ");
  Serial.print(InputX);
    Serial.print("Pitch: ");
  Serial.print(InputY);
  Serial.print(", Motor Output: ");


/*   /* Print out the values */
  Serial.print("OutputX: ");
  Serial.print(OutputX);
  Serial.print(",  OutputY: ");
  Serial.print (OutputY);
    Serial.print(", ax :");
  Serial.print(a.acceleration.x);
    Serial.print(", ay :");
  Serial.print(a.acceleration.y);
    Serial.print(", PID :");
  Serial.println(calculatedX); 

// Serial.print(", OUTPUT : ");
//   Serial.print(Output);
//   Serial.print(", erP : ");
//   Serial.print(errorP);
//   Serial.print(", erI : ");
//   Serial.print(errorI);
//   Serial.print(", erD : ");
//   Serial.println(errorD);

  delay(1);
  
}