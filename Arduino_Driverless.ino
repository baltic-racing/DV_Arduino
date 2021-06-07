
#include <Ethernet.h>
#include <EthernetUdp.h>

/*  UDP-Protokolle (eckige Klammern = Index vom Received Buffer):
 *   
 *  Selbst steuern und währenddessen messen
 *  [0]
 *  1
 *  
 *  Autonomous Mission:
 *  [0]       [1]-[4]           [6]-[9]         [10]-[12] 
 *   2   ; Steering Angle (°) ; Torque [Nm] ; Brake (0-100%)
 */


// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(192, 168, 1, 177);

unsigned int localPort = 8888;      // local port to listen on

// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  // buffer to hold incoming packet,
char ReplyBuffer[] = "acknowledged";        // a string to send back
String s;
double acc, steer, brake;

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

int ControllerState = 1;    // States for Measuring (=1 ; default) and Steering (=2)

//Storage for measured PWM-Frequencies from Servos
float SteeringServoMinDuty;   // Lowest measured value from Steering Servo
float SteeringServoMaxDuty;   // Highest measured value from Steering Servo
float MotorMinDuty;           // Lowest measured value from Acceleration Motor
float MotorMaxDuty;           // Highest measured value from Acceleration Motor
  // Idle-Freq for both?
  
// PWM-Pins on Arduino Uno: 3,5,6,9,10,11
int ReadSteeringServoPin = 3;
int ReadAccelerationMotorPin = 5;
int WriteSteeringServoPin = 6;
int WriteAccelerationMotorPin = 9;
int RelayPin = 12;

// Variables for measurement
int ontime,offtime,duty;
float freq,period;

void setup() {
 
  // Setting PinModes
  pinMode(ReadSteeringServoPin, INPUT);
  pinMode(ReadAccelerationMotorPin, INPUT);
  pinMode(WriteSteeringServoPin, OUTPUT);
  pinMode(WriteAccelerationMotorPin, OUTPUT);
  pinMode(RelayPin, OUTPUT);
  
  // You can use Ethernet.init(pin) to configure the CS pin
  //Ethernet.init(10);  // Most Arduino shields

  // start the Ethernet
  Ethernet.begin(mac, ip);

  // Open serial communications and wait for port to open:
  /*
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true) {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }
  */
  // start UDP
  Udp.begin(localPort);
}

void loop() {
  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  
  if (packetSize) {
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remote = Udp.remoteIP();
    for (int i=0; i < 4; i++) {
      Serial.print(remote[i], DEC);
      if (i < 3) {
        Serial.print(".");
      }
    }
    Serial.print(", port ");
    Serial.println(Udp.remotePort());
    // read the packet into packetBufffer
    Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    //Serial.println("Contents:");
    //Serial.println(packetBuffer);
    
/*
    // send a reply to the IP address and port that sent us the packet we received
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(ReplyBuffer);   // ReplyBuffer us the received buffer (char Array)
    Udp.endPacket();
*/  
    ControllerState = int(packetBuffer[0]);
    if(ControllerState == 1)
    {
        MeasureServo();
        MeasureMotor();
    }
    
    if(ControllerState == 2)
    {
        // s = "";
        // Timer oder sowas muss hier noch rein
        // packetBuffer in gesamtes String umformen
        /* for(int i = 1, i < packetBuffer.length(), i++)
        s = s + packetBuffer[i];
        zerlegen mit substring
        */
        DriveServo();
        DriveMotor();
    }
  }
}

int MeasureServo(){
   ontime = pulseIn(ReadSteeringServoPin,HIGH);
   offtime = pulseIn(ReadSteeringServoPin,LOW);
   period = ontime+offtime;
   freq = 1000000.0/period;
   duty = (ontime/period)*100; 
   
   if(duty < SteeringServoMinDuty){
    SteeringServoMinDuty = duty;
    }

   if(duty > SteeringServoMinDuty){
    SteeringServoMaxDuty = duty;
    }
   // Noch vernünftige DEFAULT-Werte für min max oder vorherige Abfrage bei null
}

int MeasureMotor(){
    ontime = pulseIn(ReadAccelerationMotorPin,HIGH);
    offtime = pulseIn(ReadAccelerationMotorPin,LOW);
    period = ontime+offtime;
    freq = 1000000.0/period;
    duty = (ontime/period)*100; 

    if(duty < MotorMinDuty){
    SteeringServoMinDuty = duty;
    }

   if(duty > MotorMaxDuty){
    SteeringServoMaxDuty = duty;
    }
}

int DriveServo(){
    //analogWrite(Pin, val);
}

int DriveMotor(){
    //analogWrite(Pin, val);
}

/*
 TO-DO:
 - MIN MAX Festlegen bei PWM-Messungen
 - Parsen von REC Buffer für State 2
 - Ansteuerung
 */
