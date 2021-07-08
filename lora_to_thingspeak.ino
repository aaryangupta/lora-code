#include <ThingSpeak.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <Console.h>
#include <Process.h>
#include <String.h>

RH_RF95 rf95;

//If you use Dragino IoT Mesh Firmware, uncomment below lines.
//For product: LG01. 
#define BAUDRATE 115200

String thingSpeakAPI = "IORVFLN22SLCKNMW";
String talkBackAPIKey = "OXI8CKM42Q9ATQ6E";
const int checkTalkBackInterval = 10 * 1000;    // Time interval in milliseconds to check TalkBack (number of seconds * 1000 = interval)
String null_data = "NULL"; // null data for post command

int gateway_id = 0;
int address = 0;
long lastConnectionTime = 0;
String talkBackID = "33433";
float frequency = 868100000;

void setup()
{
    Bridge.begin(BAUDRATE);
    Console.begin();

    if (!rf95.init())
        Console.println("init failed");  
    // Setup ISM frequency
    rf95.setFrequency(frequency);
    // Setup Power,dBm
    rf95.setTxPower(13);
}

void loop()
{  
    // Check ThingSpeak for TalkBack Commands
    checkTalkBack();//Check if there is talkback command
    delay(checkTalkBackInterval); 
     
}

void checkTalkBack()//Check if there is talkback command
{
    char talkBackCommand[200] = {0};
    Console.println("Checking Talkback from Server");
    int count=0;
    String talkBackURL =  "https://" + thingSpeakAPI + "/talkbacks/" + talkBackID + "/commands/execute.json?api_key=" + talkBackAPIKey;
    Process p;
    p.begin("curl");
    p.addParameter("-k");
    p.addParameter(talkBackURL);
    p.run();    // Run the process and wait for its termination    

    Console.print("Get Result:");
    while (p.available()>0)
    {
        talkBackCommand[count] = p.read();//
        Console.write(talkBackCommand[count]);
        count++;
    }
    Console.println("");
    Console.print("Command Length: ");
    Console.println(count);

    if (count > 2)
    {
        unsigned char data[50] = {0} ;//The data to be sent to LoRa Node 
        int quota_count = 0;
        int start_pos = 0;
        int stop_pos = 0;
        boolean payload_bit = false;
        for (int i=0;i < count ; i++)
        {
          if ( talkBackCommand[i] == 34 ) 
          {
            if( i > 1) 
            {
              if ( talkBackCommand[i-1] == 92) quota_count--; // ignore the quota if it is after \.
            }
            quota_count++; // discover quota ";
          }
          if ( quota_count == 5 && start_pos == 0  ) start_pos = i;
          if ( quota_count == 6 && stop_pos == 0 ) 
          {
            stop_pos = i-1;
            //break;
          }
        }

        Console.println("_____________________________________" );
        Console.print("Get Command String: " );
        int j=0;
        for (int i=start_pos+1; i<= stop_pos; i++)
        {
          data[j]=talkBackCommand[i];
          Console.write(data[j]);
          j++;
        }
        Console.println("" );
        
        rf95.send(data, j+1); //Send data to LoRa Node.
        rf95.waitPacketSent();  //wait for sending
        Console.flush(); 
    }
    else
    {
        Console.println("No new command from server");
    }
    delay(1000);
    Console.println("");
}
