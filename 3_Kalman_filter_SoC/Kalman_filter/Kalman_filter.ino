// v.2 : Extended kalman Filter Implementation (Compiles for both Arduino and ESP32 Dev Boards)

// including library for matrix operations
#include "BasicLinearAlgebra.h"

// Library for I2C communication with OLED
#include <Wire.h>

#include <WiFi.h>
#include <WebServer.h>

// including OLED libraries
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// including libraries for sensors(current, coltage and temperature)
#include <Adafruit_ADS1X15.h>
#include <WiFi.h>
#include <WebServer.h>

// To store in ROM
#include <Preferences.h>

//MQTT
#include <PubSubClient.h>

//Json
#include <ArduinoJson.h>

Preferences preferences;
 
Adafruit_ADS1115 ads;

using namespace BLA;
// Wifi Credentials

const char* ssid = "Dark Exodus";  // Enter SSID here
const char* password = "nathan06";  //Enter Password here
const char* brokerUser = "ocbshoyv";/// MQTT Broker USerName
const char* brokerPass = "u7RF9Xts1g1r"; //broker password
const char *broker = "broker.hivemq.com";
int mqttPort = 1883;

/* Put IP Address details */
//IPAddress local_ip(192,168,1,226);
//IPAddress gateway(192,168,1,1);
//IPAddress subnet(255,255,255,0);

WebServer server(80);

WiFiClient espClient;
PubSubClient client(espClient);

//////////////////////// GLOBAL CONSTANTS //////////////////////////////////////////////////////
const float e = 2.718281828459045;  

// battery and circuit parameters
int batteryCapacity = 12;
//calculated values of Kalman Equivalent circuit constants
float R0 = 0.107;
float R1 = 0.053;
float time_diff = 5000;                     // assumed 3 seconds (Cycle repetition time)
float time_constant = 476000;               // tau - 476 sec time constant tau = 5*R1*C1

// open circuit voltage lookup table : ocv[soc] represents the practically observed open circuit voltage for a soc of integer index "soc"
// i.e ocv[0] = 23.3319 v (0 % SOC) ; ocv[100] = 26.92084718 v (100 % SOC)
BLA::Matrix<101,1> ocv = {23.3071, 23.89901199, 24.08331717, 24.21668862, 24.32499108, 24.41779792,
 24.49987154 ,24.57397574, 24.64187737, 24.70478408, 24.76356294, 24.81886051,
 24.87117354, 24.92089315, 24.96833354, 25.01375149, 25.05735991, 25.09933758,
 25.13983631, 25.1789862, 25.21689979, 25.25367512, 25.2893982, 25.32414501,
 25.35798297, 25.3909723, 25.42316697, 25.45461563, 25.48536224, 25.51544674,
 25.5449055, 25.57377174, 25.60207594, 25.62984609, 25.657108, 25.68388553,
 25.71020075, 25.73607415, 25.76152481, 25.78657047, 25.81122772, 25.83551208,
 25.85943807, 25.88301932, 25.90626865, 25.92919811, 25.95181907, 25.97414225,
 25.99617777, 26.01793522, 26.03942368, 26.06065175, 26.0816276, 26.10235899,
 26.12285331, 26.14311756, 26.16315846, 26.18298238, 26.20259542, 26.2220034,
 26.24121188, 26.26022618, 26.27905141, 26.29769244, 26.31615396, 26.33444046,
 26.35255626, 26.37050549, 26.38829214, 26.40592005, 26.42339288, 26.4407142
 ,26.45788743, 26.47491586, 26.49180266, 26.50855091, 26.52516357, 26.5416435,
 26.55799345, 26.57421611, 26.59031405, 26.60628977, 26.62214569, 26.63788416,
 26.65350744, 26.66901773, 26.68441717, 26.69970781, 26.71489167, 26.7299707,
 26.74494677, 26.75982173, 26.77459735, 26.78927537, 26.80385748, 26.8183453,
 26.83274044, 26.84704443, 26.86125879, 26.87538499, 26.88942445};

/*{23.3319, 23.45, 23.58304852 , 23.80765487 ,24.00768307 ,24.25500805 ,24.4414175, 24.5786138,  24.69821588,24.80176113,
                          24.83070727, 24.86643428, 24.91024622, 24.9333732,  24.96697321, 24.98213404, 24.99987516, 25.01114964, 25.01684596, 25.02779002, 
                          25.0347469,  25.05842286, 25.06946717, 25.10847401, 25.11598437, 25.12248794, 25.13842499,25.14418827, 25.15012488, 25.1565382,  
                          25.16368975,25.16780108, 25.17105568, 25.17560083, 25.17954955, 25.18298245, 25.1839496,25.1844725,  25.18654586, 25.18913961, 25.19120067, 
                          25.19565495, 25.20840915,25.21435272, 25.2243597,  25.24129064, 25.25899448, 25.27731043, 25.2960699,25.30509832, 25.31421711, 25.32324551, 
                          25.33200249, 25.35030866, 25.35798813, 25.36487042, 25.37079233, 25.37559986, 25.38515007, 25.39131299, 25.4019735,25.41103324, 25.41841246,
                          25.42405195, 25.42791492, 25.42998887, 25.4302875, 25.43885261, 25.44575596, 25.45110117, 25.45502565, 25.45770242, 25.45934205,25.46019455, 
                          25.46455123, 25.47014661, 25.47116031, 25.48221895, 25.48439802,25.48822377, 25.49027512, 25.49318553, 25.49564491, 25.49640149, 25.49726374,
                          25.50210221, 25.50685147, 25.51951197, 25.53115195, 25.56290933, 25.58599357,25.60168758, 25.64134965, 25.68641525, 25.7818399,  25.90889655, 
                          26.06958641,26.25223192, 26.45868309, 26.69087849, 26.92084718};*/

// voltage reading probe across battery (other terminal is GND)
int16_t analogPinMeasureBatteryVoltage = 0;                         //first voltage probe connected to analog pin 1 = A0
float valueProbeMeasureBatteryVoltage = 0;                      //variable to store the analog value of analogPinMeasureBatteryVoltage
float voltageProbeMeasureBatteryVoltage = 0;                    //calculated voltage at analogPinMeasureBatteryVoltage as per ADC resolution and maximum range capacity

// analog pin to measure temperature
int16_t analogPinTemp = 1;                    //second voltage probe connected to analog pin 2 = A1
float valueProbeTemp = 0;                     //variable to store the value of analogPinTemp
float temperatureC = 0;                       //calculated voltage at analogPinTemp
float prev_temperatureC = 0;                  //variable to store the previous temperature

// pin to measure current from current sensor
int16_t analogPinCurrent = 0;                     //first voltage probe connected to analog pin 3 = A2
float valueProbeCurrent = 0;                  //variable to store the analog value of analogPinCurrent
float voltageProbeCurrent = 0;                //calculated voltage at analogPinCurrent

float voltageDifference = 0;                  //difference in voltage between analogPinMeasureBatteryVoltage and GND
float batteryVoltage = 0;                     //calculated voltage of battery (considering voltage drop across power resistor)

float current = 0;                            //calculated current through the load (in mA)
//float slope = (ocv(100,1) - ocv(0,1))/101;    //(v_final - v_init)/(I_final - I_init) --> rate of change of voltage wrt soc  ==> if prev_soc - soc = 0 (rate becomes infinte )conditon
float slope = 0.03553;
float prev_current = 0;                       //variable to store value of previous current in A
float t = 0;
float Qdis = 0.0;                         // Charge discharged

float ocv_global = 26.92;

// Kalman Variables Initialisation
BLA::Matrix<2,1> X = {0.0, 0.95};              //state variables - [Vcap, SOC] 
BLA::Matrix<2,1> U = {0.0, 0.0};             //Input variables : [current,temp diff]
BLA::Matrix<2,2> P = {1.0 ,0.0 ,
                      0.0 ,1.0}; 
BLA::Matrix<2,1> K = {0.0, 0.0};
BLA::Matrix<2,2> ak = {0.993717298298,0,
                       0, 1};
BLA::Matrix<2,2> bk = {-0.00033298319,0.0,
                      0.0,0.0};
BLA::Matrix<1,2> ck = {-1, slope};
BLA::Matrix<1,2> dk = {R0, -0.05};            // 0.05 is assumed 
BLA::Matrix<1> voltageError = {0};
BLA::Matrix<2,2> Qk = {1,0,0,1};
BLA::Matrix<1> Rk = {0.13};
BLA::Matrix<1> DK_U = {0};

BLA::Matrix<1> measured_voltage = {voltageProbeMeasureBatteryVoltage};
//BLA::Matrix<1> battery_voltage = {batteryVoltage};
BLA::Matrix<1> v_pred = {26.6};             // Voltage Predicted (initial assumed)
float static soc = 100.0;                             // % Current SOC
BLA::Matrix<2,2> M = {1.0, 0,
                      0, 1.0};
float static prev_soc = 100.0;                       // % previous SOC (ssumed)
BLA::Matrix<1> temp = {0};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
//  pinMode(analogPinMeasureBatteryVoltage, INPUT);              // voltage probe
//  pinMode(analogPinTemp, INPUT);               // temperature
//  pinMode(analogPinCurrent, INPUT);
  ads.setGain(GAIN_ONE);       // +/- 4.096V  1 bit = 0.125mV 
  ads.begin();
//Wifi 
preferences.begin("SOC", false);
soc = preferences.getFloat("SOC", 0.0);
WiFi.begin(ssid, password);
//  WiFi.softAP(ssid, password);
//  WiFi.softAPConfig(local_ip, gateway, subnet);
//  delay(100);
while (WiFi.status() != WL_CONNECTED) 
{
delay(1000);
Serial.print(".");
}
Serial.println("");
Serial.println("WiFi connected..!");
//  server.on("/", handle_OnConnect);
//  server.onNotFound(handle_NotFound);

  server.begin();
  Serial.println("HTTP server started");
  //MQTT client
  client.setServer(broker, mqttPort);
  Serial.println("Connected ");
 Serial.print("MQTT Server ");
 Serial.print(broker);
 Serial.print(":");
 Serial.println(String(mqttPort));
 Serial.print("ESP32 IP: ");
 Serial.println(WiFi.localIP());
}


void loop() {
  server.handleClient();
  measure_temperature();
  measure_voltage();
//  delay(10);
  measure_current();

   // Extended Kalman Filter Implementation
  U(0,0) = prev_current;
  U(1,0) = prev_temperatureC - 25;
  ak(0,0) = pow(e, - time_diff/time_constant);
  bk(0,0) = (1 - pow(e, -time_diff/time_constant))*R1;
  bk(1,0) = - time_diff/(batteryCapacity*3600000);            // Time diff is in millis, 1hr = 3600000 ms
  DK_U = -dk*U;
  // prediction step
  X = ak*X + bk*U;
  Serial.println(X(1,0)*100);
//  Serial.println(DK_U(0));
//  t = X(1,0);
  P = ak*(P*(~ak)) + Qk;
  ocv_map();
  v_pred = DK_U + ocv_global;
  //- X(0,0); 
  Serial.print("Voltage Predicted: ");
  Serial.println(v_pred(0));
  Serial.print("OCV: ");
  Serial.println(ocv(int(X(1,0)*100),0));
  //update step 
  voltageError = measured_voltage - v_pred;
  Serial.print("Voltage Error: ");
  Serial.println(voltageError(0));
  temp = ck*(P*(~ck)) + Rk;
  K = P*(~ck)/temp(0,0);
  X = X + K*voltageError;
//  Serial.println(K(0,0)); 
//  Serial.println(bk(1,0));
  P = (M - K*ck)*P;

  calculate_soc();
  preferences.putFloat("SOC", soc);
  update_variables();
  delay(time_diff);
//  server.send(200, "text/html", SendHTML(temperatureC,current,voltageProbeMeasureBatteryVoltage,voltageError(0),soc));

    StaticJsonDocument<256> JSONbuffer;

    JSONbuffer["SOC"] = soc;
    JSONbuffer["Current"] = current;
    JSONbuffer["Voltage"] = voltageProbeMeasureBatteryVoltage;
    JSONbuffer["Temperature"] = temperatureC;
    JSONbuffer["Voltage Error"] = voltageError(0);

    char buffer[128];
    size_t n = serializeJson(JSONbuffer, buffer);
    
    if (client.publish("Magnes/SOC", buffer, n) == true) {
    Serial.println("Success sending message");
  } 
  else {
    Serial.println("Error sending message");
  }

    if (!client.connected()){
        reconnect();
    }

    client.loop();
}

/* Updates the current and previous variables for electric current, voltage, soc and other variables after each iteration*/
void update_variables(){
  prev_current = current;
  prev_soc = soc;
  prev_temperatureC = temperatureC; 

}

void calculate_soc(){ //rounding in lookup table for soc values (indexing)
  soc = X(1,0)*100;
  Serial.println("==================================================");
  Serial.print("State of Charge (SOC): ");
  soc = (prev_soc + soc)/2;
  Serial.println(soc);
  Serial.println("==================================================");
  
  /*display.print("State of Charge (SOC):");
  display.setTextSize(3);
  display.println(soc);*/
}

void measure_temperature(){
  valueProbeTemp = ads.readADC_SingleEnded(0);      //Read temperature ADC value
  temperatureC = valueProbeTemp*0.125/10;           // converting that reading to voltage
  Serial.print("Temperature (degrees C) ");         //display the temperature in degrees C
  Serial.println(temperatureC); 
}

void measure_voltage(){
  // voltage across probes ==> R1 = 39k & R2 = 5.1k
  valueProbeMeasureBatteryVoltage = ads.readADC_SingleEnded(1);                         // read the input value at voltage probe
  voltageProbeMeasureBatteryVoltage = (valueProbeMeasureBatteryVoltage)*(0.125/1000)* 8.64705;        //calculate voltage at probe one in Volts (Max value = 30 V) => 16 bit ADC
  Serial.print("Voltage Probe One (V): ");          //display voltage at probe one
  Serial.println(voltageProbeMeasureBatteryVoltage);  
    
  measured_voltage(0) = {voltageProbeMeasureBatteryVoltage};
//  batteryVoltage = voltageDifference - current*resistance;     //calculated battery voltage = voltage difference across probes - voltage drop across power resistor
//  Serial.print("Battery Voltage (V): ");            //display battery voltage
//  Serial.println(batteryVoltage);
}

void measure_current(){
  valueProbeCurrent = ads.readADC_SingleEnded(2);     // read the input value at probe three
  voltageProbeCurrent = valueProbeCurrent*0.125/1000; //calculate voltage at probe one in Amperes (Full Max. value = 50 A) => 16 bit ADC
  current = (voltageProbeCurrent-1.578)/0.015;       // actual current
  Qdis += current*3;
  
  Serial.print("Battery Current (A): ");              //display actual current
  Serial.println(current);
  Serial.print("Discharged Current (Ah): ");              //Discharged current
  Serial.println(Qdis);  
}

void ocv_map(){
  int index;
  if (X(1,0)*100 > 100) {
  ocv_global = ocv(100,0);
  X(1,0) = 1.0;
  }
  else if (X(1,0)*100 < 5) {
  ocv_global = ocv(5,0);
  X(1,0) = 0.05;
  }
  else {
    index = X(1,0)*100;
    ocv_global = ocv(index);
  }
  }
  
void reconnect(){
    while(!client.connected()){
        Serial.println("connecting to");
        Serial.println(broker);
        if(client.connect(broker,brokerUser,brokerPass)){
            Serial.println("connected to");
            Serial.println(broker);
        } else{
            Serial.println("Trying to connect again");
            delay(500);
        }


    }
}
//  void handle_OnConnect() {
//  server.send(200, "text/html", SendHTML(temperatureC,current,voltageProbeMeasureBatteryVoltage,voltageError(0),soc)); 
//}
//
//void handle_NotFound(){
//  server.send(404, "text/plain", "Not found");
//}

//String SendHTML(double temp,double current, double voltage, double VoltageErr, uint8_t SOC){
//  String ptr = "<!DOCTYPE html> <html>\n";
//  ptr +="<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
//  ptr +="<title>State of charge</title>\n";
//  ptr +="<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n";
//  ptr +="body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h3 {color: #444444;margin-bottom: 50px;}\n";
//  ptr +="p {font-size: 14px;color: #888;margin-bottom: 10px;}\n";
//  ptr +="</style>\n";
//  ptr +="</head>\n";
//  ptr +="<body>\n";
//  ptr +="<h1>SOC Estimation</h1>\n";
//  ptr +="<h3>Using Access Point(AP) Mode</h3>\n";
//  
//  ptr +="<p>Temperature: " + String(temp) + " </p>\n";
//  ptr +="<p>Current: " + String(current) + "</p>\n";
//  ptr +="<p>Voltage: " + String(voltage) + "</p>\n";
//  ptr +="<p>Voltage Error: " + String(VoltageErr) + "</p>\n";
//  ptr +="<p>SOC: " + String(SOC) + "</p>\n";
//
//  ptr +="</body>\n";
//  ptr +="</html>\n";
//  return ptr;
//}
