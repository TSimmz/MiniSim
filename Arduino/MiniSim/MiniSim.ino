//================================================================
// MiniSim.ino
//================================================================

#include "Defines.h"
#include "Simulation.h"

#define PITCH_STICK 0
#define ROLL_STICK  1
#define YAW_STICK   2

Simulation sim = Simulation();

int movementArray[6] = {0};
int prevMovementArray[6] = {0};

int requestedPlatform[6] = {0};
int calculatedServoPWM[6] = {0};

byte macAddr[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ipAddr(192, 168, 0, 123);
unsigned int rxPort = 9000;
unsigned int txPort = 9009;

IPAddress serverIP(192,168,0,210);
unsigned int serverPort = 9000;

// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  // buffer to hold incoming packet,
char ReplyBuffer[] = "acknowledged";        // a string to send back

EthernetUDP Udp;

//======================================================
// 
//======================================================
void setup() 
{
  Serial.begin(38400);

  Ethernet.begin(macAddr, ipAddr);

  if(!isEthernetConnected())
  {
    while (true)
      delay(1);  
  } 
  
  Serial.println("Ethernet is connected correctly..");
  
  Udp.begin(serverPort);
}

//======================================================
// 
//======================================================
void loop() 
{
  //checkForUPD();

  // Send a reply to the IP address and port that sent us the packet we received
  Udp.beginPacket(serverIP, serverPort);
  Udp.write(ReplyBuffer);
  Udp.endPacket();
  
  delay(10);
  
  if ( prevMovementArray[SURGE] != movementArray[SURGE] ||
       prevMovementArray[SWAY]  != movementArray[SWAY]  ||
       prevMovementArray[HEAVE] != movementArray[HEAVE] ||
       prevMovementArray[ROLL]  != movementArray[ROLL]  ||
       prevMovementArray[PITCH] != movementArray[PITCH] ||
       prevMovementArray[YAW]   != movementArray[YAW]   )
  
  {
    sim.run(movementArray);
    copyMovementPositions();
  }
}

//======================================================
// 
//======================================================
void copyMovementPositions()
{
  prevMovementArray[SURGE] = movementArray[SURGE];
  prevMovementArray[SWAY]  = movementArray[SWAY];
  prevMovementArray[HEAVE] = movementArray[HEAVE];
  prevMovementArray[ROLL]  = movementArray[ROLL];
  prevMovementArray[PITCH] = movementArray[PITCH];
  prevMovementArray[YAW]   = movementArray[YAW];
}

//======================================================
// 
//======================================================
bool isEthernetConnected()
{
  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware)
  {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    return false;
  }
  
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
    return false;
  }

  return true;
}

//======================================================
// 
//======================================================
void checkForUPD()
{
  int packetSize = Udp.parsePacket();
  
  Serial.print("Packet Size: ");
  Serial.println(packetSize);
  
  if (packetSize)
  {
    Serial.print("Packet Size: ");
    Serial.println(packetSize);
    
    Serial.print("From: ");
    IPAddress remote = Udp.remoteIP();
    
    for (int i=0; i < 4; i++) {
      Serial.print(remote[i], DEC);
      if (i < 3) {
        Serial.print(".");
      }
    }
    Serial.print(", port ");
    Serial.println(Udp.remotePort());

    // Read the packet into packetBufffer
    Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    Serial.println("Data:");
    Serial.println(packetBuffer);

    // Send a reply to the IP address and port that sent us the packet we received
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(ReplyBuffer);
    Udp.endPacket();
  }
//  else
//  {
//    Serial.println("No packets received");
//  }
  delay(10);
}
