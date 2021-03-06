#include <esp_now.h>
#include <WiFi.h>

#define LED_PIN 2
int WIFI_CHANNEL = 1;
static uint8_t broadcast_mac[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };


//EEPROM
#include <EEPROM.h>
#define EEPROM_SIZE 512



//receivedchars
const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data
int receivedint;
boolean newData = false;
int begin1;
int begin2;
int begin3;
int begin4;
int begin5;
int eind1;
int eind2;
int eind3;
int eind4;
int eind5;


int laatstebegin1;
int laatstebegin2;
int laatstebegin3;
int laatstebegin4;
int laatstebegin5;
int laatsteeind1;
int laatsteeind2;
int laatsteeind3;
int laatsteeind4;
int laatsteeind5;



int beginvalue1;
int beginvalue2;
int beginvalue3;
int beginvalue4;
int beginvalue5;



int eindvalue1;
int eindvalue2;
int eindvalue3;
int eindvalue4;
int eindvalue5;



float bereik1;
float bereik2;
float bereik3;
float bereik4;
float bereik5;



bool vingerserial1 = false;;
bool vingerserial2 = false;;
bool vingerserial3 = false;;
bool vingerserial4 = false;;
bool vingerserial5 = false;;

bool vingerserialcalc1 = false;;
bool vingerserialcalc2 = false;;
bool vingerserialcalc3 = false;;
bool vingerserialcalc4 = false;;
bool vingerserialcalc5 = false;;

bool sendinput = true;
bool sendnotification = false;

bool onetimesettingsinfo;




bool knipperlampje = false;


//only for convertion
String receivedstring;
String receivedstringvijf;
String receivedstringzes;
String receivedstringzeven;
String receivedstringacht;
String receivedstringnegen;
String receivedsnel;
String receivedprecies;




//potentiometer pins
const int potPin1 = 33;
const int potPin2 = 32;
const int potPin3 = 35;
const int potPin4 = 34;
const int potPin5 = 39;

//potentiometer values
int potValue1 = 0;
int potValue2 = 0;
int potValue3 = 0;
int potValue4 = 0;
int potValue5 = 0;


//berekende waarden
int BerPotValue1 = 0;
int BerPotValue2 = 0;
int BerPotValue3 = 0;
int BerPotValue4 = 0;
int BerPotValue5 = 0;


#define SENDER
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
  potentiometerread();
  berekening();
  if(sendinput == true)
  {
  #ifdef SENDER
  static uint32_t counter = 0;
  esp_now_msg_t msg;
  msg.address = 0;
  msg.counter = ++counter;
  msg.servo1 = BerPotValue1;
  msg.servo2 = BerPotValue2;
  msg.servo3 = BerPotValue3;
  msg.servo4 = BerPotValue4;
  msg.servo5 = BerPotValue5;
  
  
  send_msg(&msg);
  if (knipperlampje == true)
  {
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
  #endif
  }
  
  
  //receive data
  recvWithEndMarker();
  firstserialcheck();
  serialprint();
  
 
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
      if (sendnotification == true)
      {
        Serial.println("Send succes");
      }
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
    esp_now_msg_t msg;
    memcpy(&msg, data, len);

    
  }
}

void potentiometerread ()
{
  potValue1 = analogRead(potPin1);
  potValue2 = analogRead(potPin2);
  potValue3 = analogRead(potPin3);
  potValue4 = analogRead(potPin4);
  potValue5 = analogRead(potPin5);
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



void firstserialcheck()
{
  
  if(newData == true)
  {

  receivedint = atoi(receivedChars);
  receivedstring = receivedint;
  receivedstringvijf = receivedstring[4];
  receivedstringzes = receivedstring[5];
  receivedstringzeven = receivedstring[6];
  receivedstringacht = receivedstring[7];
  receivedstringnegen = receivedstring[8];
  receivedsnel = receivedstringvijf + receivedstringzes;
  receivedprecies = receivedstringzeven + receivedstringacht + receivedstringnegen;

    //write mode
    //als 1
    if(receivedChars[0] == 49)
    {
      //als 1
      if(receivedChars[1] == 49)
      {
        //als 1
        if(receivedChars[2] == 49)
        {
          if(receivedChars[3] == 49)
          {
            
            begin1 = receivedsnel.toInt();
            laatstebegin1 = receivedprecies.toInt();
            EEPROM.write(0, begin1);
            EEPROM.write(1, laatstebegin1);
            EEPROM.commit();
          }
          if(receivedChars[3] == 50)
          {
            eind1 = receivedsnel.toInt();
            laatsteeind1 = receivedprecies.toInt();
            EEPROM.write(2, eind1);
            EEPROM.write(3, laatsteeind1);
            EEPROM.commit();
          }
        }
        //als 2
        if(receivedChars[2] == 50)
        {
          if(receivedChars[3] == 49)
          {
            begin2 = receivedsnel.toInt();
            laatstebegin2 = receivedprecies.toInt();
            EEPROM.write(4, begin2);
            EEPROM.write(5, laatstebegin2);
            EEPROM.commit();
          }
          if(receivedChars[3] == 50)
          {
            eind2 = receivedsnel.toInt();
            laatsteeind2 = receivedprecies.toInt();
            EEPROM.write(6, eind2);
            EEPROM.write(7, laatsteeind2);
            EEPROM.commit();
          }
        }
        //als 3
        if(receivedChars[2] == 51)
        {
          if(receivedChars[3] == 49)
          {
            begin3 = receivedsnel.toInt();
            laatstebegin3 = receivedprecies.toInt();
            EEPROM.write(8, begin3);
            EEPROM.write(9, laatstebegin3);
            EEPROM.commit();
          }
          if(receivedChars[3] == 50)
          {
            eind3 = receivedsnel.toInt();
            laatsteeind3 = receivedprecies.toInt();
            EEPROM.write(10, eind3);
            EEPROM.write(11, laatsteeind3);
            EEPROM.commit();
          }
        }
        //als 4
        if(receivedChars[2] == 52)
        {
          if(receivedChars[3] == 49)
          {
            begin4 = receivedsnel.toInt();
            laatstebegin4 = receivedprecies.toInt();
            EEPROM.write(12, begin4);
            EEPROM.write(13, laatstebegin4);
            EEPROM.commit();
          }
          if(receivedChars[3] == 50)
          {
            eind4 = receivedsnel.toInt();
            laatsteeind4 = receivedprecies.toInt();
            EEPROM.write(14, eind4);
            EEPROM.write(15, laatsteeind4);
            EEPROM.commit();
          }
        }
        //als 5
        if(receivedChars[2] == 53)
        {
          if(receivedChars[3] == 49)
          {
            begin5 = receivedsnel.toInt();
            laatstebegin5 = receivedprecies.toInt();
            EEPROM.write(16, begin5);
            EEPROM.write(17, laatstebegin5);
            EEPROM.commit();
          }
          if(receivedChars[3] == 50)
          {
            eind5 = receivedsnel.toInt();
            laatsteeind5 = receivedprecies.toInt();
            EEPROM.write(18, eind5);
            EEPROM.write(19, laatsteeind5);
            EEPROM.commit();
          }
        }
      }

      //turn off sending
      if (receivedChars[1] == 50)
      {
        if (receivedChars[2] == 49)
        {
          sendinput = true;
          EEPROM.write(20, 1);
          EEPROM.commit();
        }
        if (receivedChars[2] == 48)
        {
          sendinput = false;
          EEPROM.write(20, 0);
          EEPROM.commit();
        }
      }


      if (receivedChars[1] == 51)
      {
        if (receivedChars[2] == 49)
        {
          knipperlampje = true;
          EEPROM.write(21, 1);
          EEPROM.commit();
        }
        if (receivedChars[2] == 48)
        {
          knipperlampje = false;
          EEPROM.write(21, 0);
          EEPROM.commit();
          digitalWrite(LED_PIN,LOW);
        }
      }
    }










    //read mode
    //als 2
    if(receivedChars[0] == 50)
    {
      //als 1
      if(receivedChars[1] == 49)
      {
        //als 1
        if(receivedChars[2] == 49)
        {
          if(receivedChars[3] == 49)
          {
            vingerserial1 = true;
          }
          if(receivedChars[3] == 48)
          {
            vingerserial1 = false;
          }
        }
        //als 2
        if(receivedChars[2] == 50)
        {
          if(receivedChars[3] == 49)
          {
            vingerserial2 = true;
          }
          if(receivedChars[3] == 48)
          {
            vingerserial2 = false;
          }
        }
        //als 3
        if(receivedChars[2] == 51)
        {
          if(receivedChars[3] == 49)
          {
            vingerserial3 = true;
          }
          if(receivedChars[3] == 48)
          {
            vingerserial3 = false;

          }
        }
        //als 4
        if(receivedChars[2] == 52)
        {
          if(receivedChars[3] == 49)
          {
            vingerserial4 = true;
          }
          if(receivedChars[3] == 48)
          {
            vingerserial4 = false;
          }
        }
        //als 5
        if(receivedChars[2] == 53)
        {
          if(receivedChars[3] == 49)
          {
            vingerserial5 = true;
          }
          if(receivedChars[3] == 48)
          {
            vingerserial5 = false;
          }
        }
      }

      //als 2
      if(receivedChars[1] == 50)
      {
        //als 1
        if(receivedChars[2] == 49)
        {
          if(receivedChars[3] == 49)
          {
            vingerserialcalc1 = true;
          }
          if(receivedChars[3] == 48)
          {
            vingerserialcalc1 = false;
          }
        }
        //als 2
        if(receivedChars[2] == 50)
        {
          if(receivedChars[3] == 49)
          {
            vingerserialcalc2 = true;
          }
          if(receivedChars[3] == 48)
          {
            vingerserialcalc2 = false;
          }
        }
        //als 3
        if(receivedChars[2] == 51)
        {
          if(receivedChars[3] == 49)
          {
            vingerserialcalc3 = true;
          }
          if(receivedChars[3] == 48)
          {
            vingerserialcalc3 = false;
          }
        }
        //als 4
        if(receivedChars[2] == 52)
        {
          if(receivedChars[3] == 49)
          {
            vingerserialcalc4 = true;
          }
          if(receivedChars[3] == 48)
          {
            vingerserialcalc4 = false;
          }
        }
        //als 5
        if(receivedChars[2] == 53)
        {
          if(receivedChars[3] == 49)
          {
            vingerserialcalc5 = true;
          }
          if(receivedChars[3] == 48)
          {
            vingerserialcalc5 = false;
          }
        }
      }


      if (receivedChars[1] == 51)
      {
        if (receivedChars[2] == 49)
        {
          sendnotification = true;
        }
        
        if (receivedChars[2] == 48)
        {
          sendnotification = false;
          
        }
      }




      //get all settings
      if (receivedChars[1] == 52)
      {
        onetimesettingsinfo = true;
      }
    }
    newData = false;
  }
}




void serialprint()
{
  if(vingerserial1 == true)
  {
    Serial.println(potValue1);
  }
  if(vingerserial2 == true)
  {
    Serial.println(potValue2);
  }
  if(vingerserial3 == true)
  {
    Serial.println(potValue3);
  }
  if(vingerserial4 == true)
  {
    Serial.println(potValue4);
  }
  if(vingerserial5 == true)
  {
    Serial.println(potValue5);
  }






  //berekende waarden
  if(vingerserialcalc1 == true)
  {
    Serial.println(BerPotValue1);
  }
  if(vingerserialcalc2 == true)
  {
    Serial.println(BerPotValue2);
  }
  if(vingerserialcalc3 == true)
  {
    Serial.println(BerPotValue3);
  }
  if(vingerserialcalc4 == true)
  {
    Serial.println(BerPotValue4);
  }
  if(vingerserialcalc5 == true)
  {
    Serial.println(BerPotValue5);
  }





  if(onetimesettingsinfo == true)
  {
    onetimesettingsinfo = false;
    //eerste
    Serial.print("Begin1: ");
    Serial.println(begin1);
    Serial.print("Laatste begin1: ");
    Serial.println(laatstebegin1);

    Serial.print("Eind1: ");
    Serial.println(eind1);
    Serial.print("Laatste eind1: ");
    Serial.println(laatsteeind1);

    //tweede
    Serial.print("Begin2: ");
    Serial.println(begin2);
    Serial.print("Laatste begin2: ");
    Serial.println(laatstebegin2);

    Serial.print("Eind2: ");
    Serial.println(eind2);
    Serial.print("Laatste eind2: ");
    Serial.println(laatsteeind2);

    //derde
    Serial.print("Begin3: ");
    Serial.println(begin3);
    Serial.print("Laatste begin3: ");
    Serial.println(laatstebegin3);

    Serial.print("Eind3: ");
    Serial.println(eind3);
    Serial.print("Laatste eind3: ");
    Serial.println(laatsteeind3);

    //vierde
    Serial.print("Begin4: ");
    Serial.println(begin4);
    Serial.print("Laatste begin4: ");
    Serial.println(laatstebegin4);

    Serial.print("Eind4: ");
    Serial.println(eind4);
    Serial.print("Laatste eind4: ");
    Serial.println(laatsteeind4);

    //vijfde
    Serial.print("Begin5: ");
    Serial.println(begin5);
    Serial.print("Laatste begin5: ");
    Serial.println(laatstebegin5);

    Serial.print("Eind5: ");
    Serial.println(eind5);
    Serial.print("Laatste eind5: ");
    Serial.println(laatsteeind5);


    
    if (sendinput == true)
    {
    Serial.println("Send info ja ");
    }
    else
    {
      Serial.println("Send info nee");
    }

    
    
  }
}




void eepromget ()
{
    begin1 = EEPROM.read(0);
    laatstebegin1 = EEPROM.read(1);
    eind1 = EEPROM.read(2);
    laatsteeind1 = EEPROM.read(3);

    begin2 = EEPROM.read(4);
    laatstebegin2 = EEPROM.read(5);
    eind2 = EEPROM.read(6);
    laatsteeind2 = EEPROM.read(7);

    begin3 = EEPROM.read(8);
    laatstebegin3 = EEPROM.read(9);
    eind3 = EEPROM.read(10);
    laatsteeind3 = EEPROM.read(11);

    begin4 = EEPROM.read(12);
    laatstebegin4 = EEPROM.read(13);
    eind4 = EEPROM.read(14);
    laatsteeind4 = EEPROM.read(15);

    begin5 = EEPROM.read(16);
    laatstebegin5 = EEPROM.read(17);
    eind5 = EEPROM.read(18);
    laatsteeind5 = EEPROM.read(19);

    if (EEPROM.read(20) == 1)
    {
    sendinput = true;
    }
    else
    {
      sendinput = false;
    }
    
    if (EEPROM.read(21) == 1)
    {
    knipperlampje = true;
    }
    else
    {
      knipperlampje = false;
    }
}




void berekening ()
{
  beginvalue1 = begin1*256 + laatstebegin1;
  eindvalue1 = eind1*256 + laatsteeind1;

  beginvalue2 = begin2*256 + laatstebegin2;
  eindvalue2 = eind2*256 + laatsteeind2;

  beginvalue3 = begin3*256 + laatstebegin3;
  eindvalue3 = eind3*256 + laatsteeind3;

  beginvalue4 = begin4*256 + laatstebegin4;
  eindvalue4 = eind4*256 + laatsteeind4;

  beginvalue5 = begin5*256 + laatstebegin5;
  eindvalue5 = eind5*256 + laatsteeind5;

  if (potValue1 >= beginvalue1)
  {
    if(potValue1 <= eindvalue1)
    {
      bereik1 = eindvalue1 - beginvalue1;
      BerPotValue1 = (potValue1 - beginvalue1)*(4095/(eindvalue1 - beginvalue1));
    }
    else
    {
      BerPotValue1 = 4095;
    }
  }
  else
  {
    BerPotValue1 = 0;
  }







  if (potValue2 >= beginvalue2)
  {
    if(potValue2 <= eindvalue2)
    {
      bereik2 = eindvalue2 - beginvalue2;
      BerPotValue2 = (potValue2 - beginvalue2)*(4095/(bereik2));
    }
    else
    {
      BerPotValue2 = 4095;
    }
  }
  else
  {
    BerPotValue2 = 0;
  }








  if (potValue3 >= beginvalue3)
  {
    if(potValue3 <= eindvalue3)
    {
      bereik3 = eindvalue3 - beginvalue3;
      BerPotValue3 = (potValue3 - beginvalue3)*(4095/(bereik3));
    }
    else
    {
      BerPotValue3 = 4095;
    }
  }
  else
  {
    BerPotValue3 = 0;
  }







  if (potValue4 >= beginvalue4)
  {
    if(potValue4 <= eindvalue4)
    {
      bereik4 = eindvalue4 - beginvalue4;
      BerPotValue4 = (potValue4 - beginvalue4)*(4095/(bereik4));
    }
    else
    {
      BerPotValue4 = 4095;
    }
  }
  else
  {
    BerPotValue4 = 0;
  }





  if (potValue5 >= beginvalue5)
  {
    if(potValue5 <= eindvalue5)
    {
      bereik5 = eindvalue5 - beginvalue5;
      BerPotValue5 = (potValue5 - beginvalue5)*(4095/(bereik5));
    }
    else
    {
      BerPotValue5 = 4095;
    }
  }
  else
  {
    BerPotValue5 = 0;
  }
}
