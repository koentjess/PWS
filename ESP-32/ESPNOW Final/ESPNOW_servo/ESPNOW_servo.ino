#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>


//EEPROM
#include <EEPROM.h>
#define EEPROM_SIZE 512


//receivedchars
const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data
int receivedint;
boolean newData = false;


//only for convertion
String receivedstring;
String receivedstringeen;
String receivedstringtwee;
String receivedstringdrie;
String receivedstringvier;
String receivedstringvijf;
String receivedstringzes;
String receivedstringeentotdrie;
String receivedstringviertotzes;
int receivedstringeentotdrieint;



//write ints
int bereikbegin1;
int bereikbegin2;
int bereikbegin3;
int bereikbegin4;
int bereikbegin5;


int bereikeind1;
int bereikeind2;
int bereikeind3;
int bereikeind4;
int bereikeind5;



int berpos1 = 0;
int berpos2 = 0; 
int berpos3 = 0; 
int berpos4 = 0; 
int berpos5 = 0;


//write bools
bool receiveinput = true;
bool knipperlampje = false;
bool servocontrol = true;




//read bool
bool onetimesettingsinfo = false;

bool directwaarde1 = false;
bool directwaarde2 = false;
bool directwaarde3 = false;
bool directwaarde4 = false;
bool directwaarde5 = false;


bool berekendewaarde1 = false;
bool berekendewaarde2 = false;
bool berekendewaarde3 = false;
bool berekendewaarde4 = false;
bool berekendewaarde5 = false;

bool ontvangstbevestiging = false;


//servo setup
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;

int servo1Pin = 12;
int servo2Pin = 13;
int servo3Pin = 26;
int servo4Pin = 27;
int servo5Pin = 14;

int minUs = 1000;
int maxUs = 2000;

int freq = 1000;

int pos1 = 0;
int pos2 = 0; 
int pos3 = 0; 
int pos4 = 0; 
int pos5 = 0; // position in degrees
ESP32PWM pwm;

#define LED_PIN 2
int WIFI_CHANNEL = 1;
static uint8_t broadcast_mac[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };


typedef struct __attribute__((packed)) esp_now_msg_t
{
  uint32_t address;
  uint32_t counter;
  uint16_t servo1;
  uint16_t servo2;
  uint16_t servo3;
  uint16_t servo4;
  uint16_t servo5;
  // Can put lots of things here...
} esp_now_msg_t;

void setup() {

  
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);
  eepromget();
  pinMode(LED_PIN,OUTPUT);
  digitalWrite(LED_PIN,LOW);

  network_setup();

  //servo setup
  servo1.setPeriodHertz(50);      // Standard 50hz servo
  servo2.setPeriodHertz(50);      // Standard 50hz servo
  servo3.setPeriodHertz(50);      // Standard 50hz servo
  servo4.setPeriodHertz(50);      // Standard 50hz servo
  servo5.setPeriodHertz(50);      // Standard 50hz servo

  servo1.attach(servo1Pin, 600, 2500);
  servo2.attach(servo2Pin, 500, 2500);
  servo3.attach(servo3Pin, minUs, maxUs);
  servo4.attach(servo4Pin, 500, 2500);
  servo5.attach(servo5Pin, 600, 2500); 
}

static void network_setup(void)
{
  //Puts ESP in STATION MODE
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != 0)
  {
    return;
  }
  esp_now_peer_info_t peer_info;
  peer_info.channel = WIFI_CHANNEL;
  memcpy(peer_info.peer_addr, broadcast_mac, 6);
  peer_info.ifidx = ESP_IF_WIFI_STA;
  peer_info.encrypt = false;
  esp_err_t status = esp_now_add_peer(&peer_info);
  if (ESP_OK != status)
  {
    Serial.println("Could not add peer");
    Serial.println(status);
  }
   // Set up callback
  status = esp_now_register_recv_cb(msg_recv_cb);
  if (ESP_OK != status)
  {
    Serial.println("Could not register callback");
    Serial.println(status);
  }

  status = esp_now_register_send_cb(msg_send_cb);
  if (ESP_OK != status)
  {
    Serial.println("Could not register send callback");
    Serial.println(status);
  }
}

void loop() {
  // Need some delay for watchdog feeding in loop
  delay(25);

  
  recvWithEndMarker();
  serialcheck();
  berekening();
  printing();
  
  #ifdef SENDER
  static uint32_t counter = 0;
  esp_now_msg_t msg;
  msg.address = 0;
  msg.counter = ++counter;
  send_msg(&msg);
  if (knipperlampje == true)
  {
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
  #endif
}

static void send_msg(esp_now_msg_t * msg)
{
  // Pack
  uint16_t packet_size = sizeof(esp_now_msg_t);
  uint8_t msg_data[packet_size];
  memcpy(&msg_data[0], msg, sizeof(esp_now_msg_t));

  esp_err_t status = esp_now_send(broadcast_mac, msg_data, packet_size);
  if (ESP_OK != status)
  {
    Serial.println("Error sending message");
    Serial.println(status);
  }
}

static void msg_send_cb(const uint8_t* mac, esp_now_send_status_t sendStatus)
{

  switch (sendStatus)
  {
    case ESP_NOW_SEND_SUCCESS:
      Serial.println("Send success");
      break;

    case ESP_NOW_SEND_FAIL:
      Serial.println("Send Failure");
      break;

    default:
      break;
  }
}

static void msg_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{
  if (len == sizeof(esp_now_msg_t))
  {
    if (receiveinput == true)
    {
    esp_now_msg_t msg;
    memcpy(&msg, data, len);
    if (ontvangstbevestiging == true)
    {
      Serial.println("Received Succesfull");
    }


    pos1 = msg.servo1;
    pos2 = msg.servo2;
    pos3 = msg.servo3;
    pos4 = msg.servo4;
    pos5 = msg.servo5; 
    if (knipperlampje == true)
    {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
      servowrite();
    }
  }
}

void servowrite ()
{
 
   if (servocontrol == true)
   {
    servo1.write(berpos1);
    servo2.write(berpos2);
    servo3.write(berpos3);
    servo4.write(berpos4);
    servo5.write(berpos5);
   }
}

void recvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
   
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();
        

        
        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            newData = true;
        }
    }
}


void serialcheck ()
{
  if(newData == true)
  {
    receivedint = atoi(receivedChars);
    receivedstring = receivedint;
    receivedstringeen = receivedstring[3];
    receivedstringtwee = receivedstring[4];
    receivedstringdrie = receivedstring[5];
    receivedstringvier = receivedstring[6];
    receivedstringvijf = receivedstring[7];
    receivedstringzes = receivedstring[8];
    receivedstringeentotdrie = receivedstringeen + receivedstringtwee + receivedstringdrie;
    receivedstringviertotzes = receivedstringvier + receivedstringvijf + receivedstringzes;
    receivedstringeentotdrieint = receivedstringeentotdrie.toInt();
    
    
    //write mode
    if (receivedChars[0] == 49)
    {
      //servo waarde
      if (receivedChars[1] == 49)
      {
        if(receivedChars[2] == 49)
        {
            bereikbegin1 = receivedstringeentotdrieint;
            bereikeind1 = receivedstringviertotzes.toInt();
            EEPROM.write(0, bereikbegin1);
            EEPROM.write(1, bereikeind1);
            EEPROM.commit();
        }
        if(receivedChars[2] == 50)
        {
            bereikbegin2 = receivedstringeentotdrieint;
            bereikeind2 = receivedstringviertotzes.toInt();
            EEPROM.write(2, bereikbegin2);
            EEPROM.write(3, bereikeind2);
            EEPROM.commit();
        }
        if(receivedChars[2] == 51)
        {
            bereikbegin3 = receivedstringeentotdrieint;
            bereikeind3 = receivedstringviertotzes.toInt();
            EEPROM.write(4, bereikbegin3);
            EEPROM.write(5, bereikeind3);
            EEPROM.commit();
        }
        if(receivedChars[2] == 52)
        {
            bereikbegin4 = receivedstringeentotdrieint;
            bereikeind4 = receivedstringviertotzes.toInt();
            EEPROM.write(6, bereikbegin4);
            EEPROM.write(7, bereikeind4);
            EEPROM.commit();
        }
        if(receivedChars[2] == 53)
        {
            bereikbegin5 = receivedstringeentotdrieint;
            bereikeind5 = receivedstringviertotzes.toInt();
            EEPROM.write(8, bereikbegin5);
            EEPROM.write(9, bereikeind5);
            EEPROM.commit();
        }
      }
      //tijdelijke servo
      if (receivedChars[1] == 50)
      {
        if(receivedChars[2] == 49)
        {
            servo1.write(receivedstringeentotdrieint);
        }
        if(receivedChars[2] == 50)
        {
            servo2.write(receivedstringeentotdrieint);
        }
        if(receivedChars[2] == 51)
        {
            servo3.write(receivedstringeentotdrieint);
        }
        if(receivedChars[2] == 52)
        {
            servo4.write(receivedstringeentotdrieint);
        }
        if(receivedChars[2] == 53)
        {
            servo5.write(receivedstringeentotdrieint);
        }
      }
      //receiver
      if (receivedChars[1] == 51)
      {
        if(receivedChars[2] == 49)
        {
            receiveinput = true;
            EEPROM.write(10, 1);
            EEPROM.commit();
        }
        if(receivedChars[2] == 48)
        {
            receiveinput = false;
            EEPROM.write(10, 0);
            EEPROM.commit();
        }
      }
      //knipper lampje
      if (receivedChars[1] == 52)
      {
        if(receivedChars[2] == 49)
        {
            knipperlampje = true;
            EEPROM.write(11, 1);
            EEPROM.commit();
        }
        if(receivedChars[2] == 48)
        {
            knipperlampje = false;
            EEPROM.write(11, 0);
            EEPROM.commit();
            digitalWrite(LED_PIN,LOW);
        }
      }
      //servo control
      if (receivedChars[1] == 53)
      {
        if(receivedChars[2] == 49)
        {
            servocontrol = true;
            EEPROM.write(12, 1);
            EEPROM.commit();
        }
        if(receivedChars[2] == 48)
        {
            servocontrol = false;
            EEPROM.write(12, 0);
            EEPROM.commit();
        }
      }
    }









    //read mode
    if (receivedChars[0] == 50)
    {
      //directe waarden
      if (receivedChars[1] == 49)
      {
        if(receivedChars[2] == 49)
        {
            if(receivedChars[3] == 49)
            {
              directwaarde1 = true;
            }
            if(receivedChars[3] == 48)
            {
              directwaarde1 = false;
            }
        }
        if(receivedChars[2] == 50)
        {
            if(receivedChars[3] == 49)
            {
              directwaarde2 = true;
            }
            if(receivedChars[3] == 48)
            {
              directwaarde2 = false;
            }
        }
        if(receivedChars[2] == 51)
        {
            if(receivedChars[3] == 49)
            {
              directwaarde3 = true;
            }
            if(receivedChars[3] == 48)
            {
              directwaarde3 = false;
            }
        }
        if(receivedChars[2] == 52)
        {
            if(receivedChars[3] == 49)
            {
              directwaarde4 = true;
            }
            if(receivedChars[3] == 48)
            {
              directwaarde4 = false;
            }
        }
        if(receivedChars[2] == 53)
        {
            if(receivedChars[3] == 49)
            {
              directwaarde5 = true;
            }
            if(receivedChars[3] == 48)
            {
              directwaarde5 = false;
            }
        }
      }
      //berekende waarden
      if (receivedChars[1] == 50)
      {
        if(receivedChars[2] == 49)
        {
            if(receivedChars[3] == 49)
            {
              berekendewaarde1 = true;
            }
            if(receivedChars[3] == 48)
            {
              berekendewaarde1 = false;
            }
        }
        if(receivedChars[2] == 50)
        {
            if(receivedChars[3] == 49)
            {
              berekendewaarde2 = true;
            }
            if(receivedChars[3] == 48)
            {
              berekendewaarde2 = false;
            }
        }
        if(receivedChars[2] == 51)
        {
            if(receivedChars[3] == 49)
            {
              berekendewaarde3 = true;
            }
            if(receivedChars[3] == 48)
            {
              berekendewaarde3 = false;
            }
        }
        if(receivedChars[2] == 52)
        {
            if(receivedChars[3] == 49)
            {
              berekendewaarde4 = true;
            }
            if(receivedChars[3] == 48)
            {
              berekendewaarde4 = false;
            }
        }
        if(receivedChars[2] == 53)
        {
            if(receivedChars[3] == 49)
            {
              berekendewaarde5 = true;
            }
            if(receivedChars[3] == 48)
            {
              berekendewaarde5 = false;
            }
        }
      }
      //ontvangstbevestiging
      if (receivedChars[1] == 51)
      {
        if(receivedChars[2] == 49)
        {
            ontvangstbevestiging = true;
        }
        if(receivedChars[2] == 48)
        {
            ontvangstbevestiging = false;
        }
      }
      //overzicht waarden
      if (receivedChars[1] == 52)
      {
        settingsinfo();
      }
    }
    newData = false;
  }
}

void berekening ()
{
  if(bereikeind1-bereikbegin1 != 0)
  {
  berpos1 = pos1/(4095/(bereikeind1-bereikbegin1))+bereikbegin1;
  }

  if(bereikeind2-bereikbegin2 != 0)
  {
  berpos2 = pos2/(4095/(bereikeind2-bereikbegin2))+bereikbegin2;
  }

  if(bereikeind3-bereikbegin3 != 0)
  {
  berpos3 = pos3/(4095/(bereikeind3-bereikbegin3))+bereikbegin3;
  }

  if(bereikeind4-bereikbegin4 != 0)
  {
  berpos4 = pos4/(4095/(bereikeind4-bereikbegin4))+bereikbegin4;
  }

  if(bereikeind5-bereikbegin5 != 0)
  {
  berpos5 = pos5/(4095/(bereikeind5-bereikbegin5))+bereikbegin5;
  }
}

void eepromget ()
{
    bereikbegin1 = EEPROM.read(0);
    bereikeind1 = EEPROM.read(1);

    bereikbegin2 = EEPROM.read(2);
    bereikeind2 = EEPROM.read(3);

    bereikbegin3 = EEPROM.read(4);
    bereikeind3 = EEPROM.read(5);

    bereikbegin4 = EEPROM.read(6);
    bereikeind4 = EEPROM.read(7);

    bereikbegin5 = EEPROM.read(8);
    bereikeind5 = EEPROM.read(9);

    if (EEPROM.read(10) == 1)
    {
      receiveinput = true;
    }
    else
    {
      receiveinput = false;
    }

    if (EEPROM.read(11) == 1)
    {
      knipperlampje = true;
    }
    else
    {
      knipperlampje = false;
    }

    if (EEPROM.read(12) == 1)
    {
      servocontrol = true;
    }
    else
    {
      servocontrol = false;
    }
    
}

void settingsinfo()
{
  Serial.print("Begin1: ");
  Serial.println(bereikbegin1);
  Serial.print("Eind1: ");
  Serial.println(bereikeind1);

  Serial.print("Begin2: ");
  Serial.println(bereikbegin2);
  Serial.print("Eind2: ");
  Serial.println(bereikeind2);

  Serial.print("Begin3: ");
  Serial.println(bereikbegin3);
  Serial.print("Eind3: ");
  Serial.println(bereikeind3);

  Serial.print("Begin4: ");
  Serial.println(bereikbegin4);
  Serial.print("Eind4: ");
  Serial.println(bereikeind4);

  Serial.print("Begin5: ");
  Serial.println(bereikbegin5);
  Serial.print("Eind5: ");
  Serial.println(bereikeind5);

  if (receiveinput == true)
  {
    Serial.println("Ontvang gegevens: Ja");
  }
  else
  {
    Serial.println("Ontvang gegevens: Nee");
  }

  if (knipperlampje == true)
  {
    Serial.println("Knipperlampje: Ja");
  }
  else
  {
    Serial.println("Knipperlampje: Nee");
  }

  if (servocontrol == true)
  {
    Serial.println("Bestuur servo's: Ja");
  }
  else
  {
    Serial.println("Bestuur servo's: Nee");
  }
  
}

void printing()
{
  if (directwaarde1 == true)
  {
    Serial.println(pos1);
  }
  if (directwaarde2 == true)
  {
    Serial.println(pos2);
  }
  if (directwaarde3 == true)
  {
    Serial.println(pos3);
  }
  if (directwaarde4 == true)
  {
    Serial.println(pos4);
  }
  if (directwaarde5 == true)
  {
    Serial.println(pos5);
  }

  if (berekendewaarde1 == true)
  {
    Serial.println(berpos1);
  }
  if (berekendewaarde2 == true)
  {
    Serial.println(berpos2);
  }
  if (berekendewaarde3 == true)
  {
    Serial.println(berpos3);
  }
  if (berekendewaarde4 == true)
  {
    Serial.println(berpos4);
  }
  if (berekendewaarde5 == true)
  {
    Serial.println(berpos5);
  }


}
