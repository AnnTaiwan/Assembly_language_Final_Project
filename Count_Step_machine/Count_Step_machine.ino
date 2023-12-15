#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>


const char*   ssid            = "Ann";                        //your network SSID
const char*   pwd             = "Ann12345";                                   //your network password
int           i;
const char*   mqtt_server     = "iiot.ideaschain.com.tw";
const int     mqtt_port       = 1883;
const char*   client_ID       = "0e5935d0-9a76-11ee-bcc4-2b6af254e752";       //client ID
const char*   username        = "ovCJ5jaPy5Bv0S47H43o";                       //device access token
char*         password        = "";                                           //nop need to use the password
const char*   subscribe_topic = "v1/devices/me/telemetry";                   //fixed format, do not modify
const char*   publish_topic   = "v1/devices/me/telemetry";                    //fixed format, do not modify
const char*   publish_payload = "{\"PM2.5\":\"10\",\"PM10\":\"20\"}";
byte* payload_copy;

WiFiClient wifi_client;
PubSubClient client(wifi_client);
// pin
int sensorPin = 34;    //聲音感測器 OUT pin 接 Arduino A40
int sensorValue = 0;   //定義 Arduino 讀到的聲音感測器感測值為 sensorValue

int LEDPIN = 16; //LED output

int redPin = 18;    //紅外線感測器 OUT pin 
int redValue = 0;   //定義 Arduino 讀到的紅外線感測器感測值為 redValue

int beePin = 19;    //蜂鳴器 OUT pin 

////WIFI///////////////////////////////////////////////////////////
void printWiFiData(){
  Serial.print("");
  Serial.print("IP Address : ");
  Serial.println(WiFi.localIP());
  Serial.print("WiFi RSSI : ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
}

  // callback function : to deal with the received message
void callback(char* topic, byte* payload, unsigned int length){
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] : ");
    for(i=0 ; i<length ; i++){
    Serial.print((char)payload[i]);
    }

  Serial.println();
  Serial.println("----------------------------------------");
}
  // reconnect the server
void reconnect(){
    //loop until we reconnected
    while(!client.connected()) {
      Serial.println("Attempting the MQTT connection ...");
      //Attempt to connect
      if(client.connect(client_ID, username, password)) {
        Serial.println("MQTT server connected");
        //publish announcement once we connect the server
        client.publish(publish_topic, publish_payload);
        //subcribe topic
        client.subscribe(subscribe_topic);
      }
      else {
        Serial.print("fail connected , rc = ");
        Serial.println(client.state());
        Serial.println("Try connected after 5 seconds");
        delay(5000);
      }
    }
  }
////////////////////////////////////////////////
// ADXL345 sensortest
/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

void displaySensorDetails(void)
{
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displayDataRate(void)
{
  Serial.print  ("Data Rate:    "); 
  
  switch(accel.getDataRate())
  {
    case ADXL345_DATARATE_3200_HZ:
      Serial.print  ("3200 "); 
      break;
    case ADXL345_DATARATE_1600_HZ:
      Serial.print  ("1600 "); 
      break;
    case ADXL345_DATARATE_800_HZ:
      Serial.print  ("800 "); 
      break;
    case ADXL345_DATARATE_400_HZ:
      Serial.print  ("400 "); 
      break;
    case ADXL345_DATARATE_200_HZ:
      Serial.print  ("200 "); 
      break;
    case ADXL345_DATARATE_100_HZ:
      Serial.print  ("100 "); 
      break;
    case ADXL345_DATARATE_50_HZ:
      Serial.print  ("50 "); 
      break;
    case ADXL345_DATARATE_25_HZ:
      Serial.print  ("25 "); 
      break;
    case ADXL345_DATARATE_12_5_HZ:
      Serial.print  ("12.5 "); 
      break;
    case ADXL345_DATARATE_6_25HZ:
      Serial.print  ("6.25 "); 
      break;
    case ADXL345_DATARATE_3_13_HZ:
      Serial.print  ("3.13 "); 
      break;
    case ADXL345_DATARATE_1_56_HZ:
      Serial.print  ("1.56 "); 
      break;
    case ADXL345_DATARATE_0_78_HZ:
      Serial.print  ("0.78 "); 
      break;
    case ADXL345_DATARATE_0_39_HZ:
      Serial.print  ("0.39 "); 
      break;
    case ADXL345_DATARATE_0_20_HZ:
      Serial.print  ("0.20 "); 
      break;
    case ADXL345_DATARATE_0_10_HZ:
      Serial.print  ("0.10 "); 
      break;
    default:
      Serial.print  ("???? "); 
      break;
  }  
  Serial.println(" Hz");  
}

void displayRange(void)
{
  Serial.print  ("Range:         +/- "); 
  
  switch(accel.getRange())
  {
    case ADXL345_RANGE_16_G:
      Serial.print  ("16 "); 
      break;
    case ADXL345_RANGE_8_G:
      Serial.print  ("8 "); 
      break;
    case ADXL345_RANGE_4_G:
      Serial.print  ("4 "); 
      break;
    case ADXL345_RANGE_2_G:
      Serial.print  ("2 "); 
      break;
    default:
      Serial.print  ("?? "); 
      break;
  }  
  Serial.println(" g");  
}
//////////////////////////////////////////////////////////////

void setup(void) 
{
// #ifndef ESP8266
//   while (!Serial); // for Leonardo/Micro/Zero
// #endif
  Serial.begin(115200);
  Serial.print("Y: "); 
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pwd);
  while(WiFi.status() != WL_CONNECTED){
      Serial.print("Attempt to connect ");
      Serial.println(ssid);
      delay(5000);
    } 

  printWiFiData();  //print the wifi data and the signal strength


  client.setServer(mqtt_server, mqtt_port); //client set the mqtt server and mqtt port
  client.setCallback(callback); 
    //Allow hardware to sort itself out
  delay(1000);
  //////////////////////////////////////////////////////////////////////
  // ADXL345
  Serial.println("Accelerometer Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }

  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL345_RANGE_16_G);
  // accel.setRange(ADXL345_RANGE_8_G);
  // accel.setRange(ADXL345_RANGE_4_G);
  // accel.setRange(ADXL345_RANGE_2_G);
  
  /* Display some basic information on this sensor */
  displaySensorDetails();
  
  /* Display additional settings (outside the scope of sensor_t) */
  displayDataRate();
  displayRange();
  Serial.println("");
  //////
  pinMode(sensorPin, INPUT);
  pinMode(redPin, INPUT);
  pinMode(LEDPIN, OUTPUT);
  pinMode(beePin, OUTPUT);
  digitalWrite(LEDPIN, LOW);
  digitalWrite(beePin, LOW);  

}


void loop(void) 
{
  

  char upload_Mes [30];
  char* keyName1 = "x_acceleration";
  char* keyName2 = "y_acceleration";
  char* keyName3 = "z_acceleration";

  char* Steps_keyName3 = "Steps";
  /* Get a new sensor event */ 
  sensors_event_t event; 
  

  int C = 0, steps = 0;
  float record_x[50];
  float record_y[50];
  int arr_Steps[50];
  while(C < 50) // 50 次 3s => 150s
  {
    int count_time = 0;
    
    float points[50][3];

    float avg_x = 0, avg_y = 0;
    float max_x = -1, idx_x, max_y = -1, idx_y, min_x = 100, min_y = 100;
    // 3s 內取50點 (=60ms 取一次資料)，這50點後來要取平均
    float acc_x, acc_y;
    while(count_time < 51)
    {
      if(count_time == 0)
      {
        accel.getEvent(&event);
        count_time ++;
        continue;
      }
      acc_x = event.acceleration.x;
      acc_y = event.acceleration.y;
      accel.getEvent(&event);

      /* Display the results (acceleration is measured in m/s^2) */
      // Serial.print("X: "); 
      // Serial.print(event.acceleration.x); Serial.print("  ");
      // Serial.print("Y: "); 
      // Serial.print(event.acceleration.y); Serial.print("  ");
      // Serial.print("Z: "); 
      // Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");

      points[count_time][0] = event.acceleration.x - acc_x;
      points[count_time][1] = event.acceleration.y - acc_y;
      
      avg_x += points[count_time][0];
      avg_y += points[count_time][1];



      if(points[count_time][0] > max_x)
      {
        idx_x = count_time;
        max_x = points[count_time][0];
      }
      if(points[count_time][1] > max_y)
      {
        idx_y = count_time;
        max_y = points[count_time][1];
      }

      if(points[count_time][0] < min_x)
      {
        
        min_x = points[count_time][0];
      }
      if(points[count_time][1] < min_y)
      {
        
        min_y = points[count_time][1];
      }

      delay(10); 
      count_time++;
    }
    //cal average
    avg_x /= 50.0;
    avg_y /= 50.0;
    // Find the 7 points near the idx_y
    int arr_y[7];
    if(idx_y < 3)
    {
      for(int i = 0; i < 7; i++)
      {
        arr_y[i] = points[i][1];
      }
    }
    else if (idx_y >= 3 && idx_y <= 46)
    {
      for(int i = idx_y - 3; i < idx_y + 4; i++)
      {
        arr_y[i + 3] = points[i][1];
      }
    }
    else
    {
      for(int i = 49; i > 42; i--)
      {
        arr_y[i-43] = points[i][1];
      }
    }
    Serial.println(max_x);
    Serial.println(min_x);
    Serial.println(max_y);
    Serial.println(min_y);
    Serial.println(avg_x);
    Serial.println(avg_y);

    // decide the number of steps
    if((max_x - min_x > 1.5) && (max_y - min_y > 1) && (max_x > avg_x) && (max_y > avg_y))
    {
      steps++;
    }
    Serial.print("Steps:  ");
    Serial.println(steps);

    record_x[C] = max_x;
    record_y[C] = max_y;
    arr_Steps[C] = steps;

    C++;
    /////Sensor
    // sound + lightpub
    sensorValue = analogRead(sensorPin);    //類比讀入聲音感測器的感測值存入 sensorValue
    Serial.print("Sound detector: ");
    Serial.println(sensorValue);            //印出感測值於序列螢幕
    if(sensorValue > 500)
    {
      Serial.println("Turn on LED.");
      digitalWrite(LEDPIN, HIGH);
    }
    else
    {
      Serial.println("Turn off LED.");
      digitalWrite(LEDPIN, LOW);
    }
  //   // avoid obstacle + louder(蜂鳴器)
    redValue = digitalRead(redPin);
    Serial.print("Obstacle detector: ");
    Serial.println(redValue);            //印出感測值於序列螢幕
    if(redValue==0)
    {
      Serial.println("**********Bee Bee Bee***********");
      digitalWrite(beePin, LOW);
      delay(500); //0.5 sec
      digitalWrite(beePin, HIGH);
      delay(500); //0.5 sec
    }
    else
    {
      digitalWrite(beePin, LOW);
    }
  // // upload data to IDEAS CHAIN
    
  //   char* keyName4 = "Infrared";
  //   char* keyName5 = "Sound sensor";
  //   char* keyName6 = "Step";

 

    delay(10);
  }


 
  digitalWrite(beePin, LOW);
  digitalWrite(LEDPIN, LOW);

 

  





  ///////////////////////////////
  // esp32
  if (!client.connected()) {
      reconnect();
    }

  C= 0;
  Serial.println("Start to upload the data.");
  while(C < 50)
  {
    sprintf(upload_Mes, "{\"%s \":\"%f \"}", keyName1, record_x[C] * 100);
    client.publish(publish_topic, upload_Mes);
    delay(500);
    sprintf(upload_Mes, "{\"%s \":\"%f \"}", keyName2, record_y[C] * 100);
    client.publish(publish_topic, upload_Mes);
    delay(500);
    // sprintf(upload_Mes, "{\"%s \":\"%f \"}", keyName3, event.acceleration.z);
    // client.publish(publish_topic, upload_Mes);
    // delay(10);
    sprintf(upload_Mes, "{\"%s \":\"%d \"}", Steps_keyName3, arr_Steps[C]);
    client.publish(publish_topic, upload_Mes);
    delay(500);
    client.loop();
    delay(500);
    C++;
  }
  

  delay(500);
}
