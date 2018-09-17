#include <Servo.h>
#include <WiievaConsole.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiievaPlayer.h>
#include <SdFat.h>

enum Pin {
  kTrigPin = 7,
  kEchoPin = 8,
  kServoPin = 9,
  kMotorApowerPin = 10,
  kMotorBpowerPin = 11,
  kMotorAdirectionPin = 12,
  kMotorBdirectionPin = 13
};

enum Direction { kForward = 0x0, kLeft = 0x1, kRight = 0x2,kBackward = 0x3, kStop=0x4};
const char *DirectionNames[] = {"forward", "left", "right", "back", "stop",nullptr};
enum Mode {kModeAuto, kModeManual};

const char* kWifiSSID = "gp_home";
const char* kWifiPassword = "12345ABCDE";

const int kLeftSpeedCompensation = 110;
const int kMaxSpeed = 100;
const int kCenterServoAngle = 100;

WiievaConsole con;
Servo servo;
ESP8266WebServer server(80);
SdFat SD;
WiievaPlayer player (0x2000);

Direction curDir = kStop;
int curDirTs = 0;
int scheduledStopTs = 0;
int scheduledServoStopTs = 0;
int curMode = kModeAuto;
int distance = 0;

void move(Direction dir,int speed=100) {
  speed = (dir&kStop)?0:speed;
  analogWrite(kMotorApowerPin, speed * kMaxSpeed/100);
  analogWrite(kMotorBpowerPin, speed * kMaxSpeed * kLeftSpeedCompensation/10000);
  digitalWrite(kMotorAdirectionPin, dir & kRight);
  digitalWrite(kMotorBdirectionPin, dir & kLeft);
  if (curDir != dir) {
    con.yellow().println (DirectionNames[dir]);
    curDir = dir;
    curDirTs = millis ();
  }
}

void turnServo(int angle) {
  static int prev = 0;
  int val = kCenterServoAngle + angle;
  if (prev != val) {
    con.normal().println (String("turn ") + val);
    servo.attach (kServoPin);
    servo.write (val); 
    prev = val;
    scheduledServoStopTs = millis () + 600;
  }
}

int measureDistance() {
  int duration, distance;
  digitalWrite(kTrigPin, HIGH);
  delayMicroseconds(1000);
  digitalWrite(kTrigPin, LOW);
  duration = pulseIn(kEchoPin, HIGH, 3000);
  distance = (duration / 2) / 29.1;
  if (distance < 30 && distance > 0) {
     con.red ().println (String("dist = ") + distance);
  }
  return distance;
}

int measureDistanceAsync() {
  static int distance;
  enum State {Idle,Sending,WaitLow,WaitHigh};
  static int state;
  static long lastStateUs;


  long curUs = micros ();
  long elapsedUs = curUs - lastStateUs;

  switch (state) {
    case Idle:
      digitalWrite(kTrigPin, HIGH);
      state = Sending;
      lastStateUs = curUs;
      break;
    case Sending:
      if (elapsedUs > 1000) {
        digitalWrite(kTrigPin, LOW);
         state = WaitLow;
         lastStateUs = curUs;
      }
      break;
    case WaitLow:
      if (elapsedUs > 3000) {
        distance = 0;
        state = Idle;
      } else if (digitalRead (kEchoPin) == HIGH) {
        state = WaitHigh;
        lastStateUs = curUs;
        for (int i = 0; i < 10 && state == WaitHigh; i++) {
          measureDistanceAsync ();
        }
      }
      break;
    case WaitHigh:
      if (elapsedUs > 3000) {
        distance = 0;
        state = Idle;
      } else if (digitalRead (kEchoPin) == LOW) {
        state = Idle;
        distance = (elapsedUs / 2) / 29.1;
        if (distance <= 7) {
           distance = 0;
        }
        if (distance < 30 && distance > 0) {
           con.red ().println (String("dist = ") + distance);
        }   
    }
    break;
  }
  return distance;
}

String musicFiles[16];
int musicCount=0;
bool needMusic = false;
int lastMusicTs = 0;

void searchMusicFiles ()
{
    char name[256];
    auto dir = SD.open("/");
    if (!dir)
        return;


    for (;musicCount < 16;) {
        auto entry =  dir.openNextFile();
        if (! entry)
            break;
        if (!entry.isDirectory() && entry.getName(name,sizeof (name)) && name[0] != '.')
        {
            musicFiles[musicCount++] = name;
            entry.close();
        }
        Serial.println (name);
    }
    dir.close();
}


void playMusic () {
  if (!musicCount) {
    return;
  }
  File f = SD.open(musicFiles[rand () % musicCount]);

  if (!f) {
      Serial.println ("Can't open file");
      return;
  }

  Serial.println ("Start playing");

  player.start (AIO_AUDIO_OUT_MP3);

  while (f.available()) {
      player.run(f);
      server.handleClient();
      delay (10);
  }

  Serial.println ("Stop playing");
  player.stop ();
  f.close ();
}

void handleHttpMove() {
  String dirStr = server.arg ("dir");
  Direction dir = kStop;
  for (int i = 0; DirectionNames[i]; i++) {
     if (dirStr == DirectionNames[i]) dir = static_cast<Direction>(i);
  }

  int speed = atoi (server.arg ("speed").c_str());
  int duration = atoi (server.arg ("duration").c_str());
  if (speed <= 0) speed = 100;
  if (duration > 0) {
     scheduledStopTs = millis () + duration;
  }
  move (dir,speed);
  curMode = kModeManual;
  server.sendHeader ("Access-Control-Allow-Origin","*");
  server.send(200, "application/json", "{}");
}


void handleHttpMode() {
  String modeStr = server.arg ("mode");
  if (modeStr == "auto") curMode = kModeAuto;
  if (modeStr == "manual") curMode = kModeManual;
  server.sendHeader ("Access-Control-Allow-Origin","*");
  server.send(200, "application/json", "{}");
  move (kStop);
}

void handleHttpDistance () {
  server.sendHeader ("Access-Control-Allow-Origin","*");
  server.send(200, "application/json", String ("{ \"distance\":") + distance + "}");
}

void handleHttpServo() {
  int angle = atoi (server.arg ("angle").c_str());
  if (angle >= -90 && angle < 90) {
    turnServo (angle);
  }
  curMode = kModeManual;
  server.sendHeader ("Access-Control-Allow-Origin","*");
  server.send(200, "application/json", "{}");
}

void handleHttpMusic () {
  server.sendHeader ("Access-Control-Allow-Origin","*");
  server.send(200, "application/json", "{}");
  needMusic = true;
}


extern const char page[];
void handleHttpRoot() {
  server.sendHeader ("Access-Control-Allow-Origin","*");
   server.send(200, "text/html", page);
}


void setup() {
  pinMode(kServoPin, OUTPUT);
  pinMode(kMotorAdirectionPin, OUTPUT);
  pinMode(kMotorBdirectionPin, OUTPUT);
  pinMode(kMotorApowerPin, OUTPUT);
  pinMode(kMotorBpowerPin, OUTPUT);
  pinMode(kTrigPin, OUTPUT);
  pinMode(kEchoPin, INPUT);
 
  con.begin (WiievaConsole::FontSmall,WiievaConsole::Portrait);
  WiFi.begin(kWifiSSID, kWifiPassword);
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  con.println(String ("Connected to ") + kWifiSSID );
  con.print ("IP address: ");
  con.println (WiFi.localIP());

  server.on("/move", handleHttpMove);
  server.on("/mode", handleHttpMode);
  server.on("/servo", handleHttpServo);
  server.on("/distance", handleHttpDistance);
  server.on("/music", handleHttpMusic);
  server.on("/", handleHttpRoot);
  server.begin();
  Serial.println("HTTP server started");
  // Init MictoSD
  if (!SD.begin(WIIEVA_SD_CS))
     Serial.println("Error init microsd card!");  searchMusicFiles ();
  Serial.println("Found " + String(musicCount) + "music files");;
}


void loopAuto () {
  distance = measureDistance();
  turnServo(0);
  if (distance >= 30 || distance <= 0) {
    // If the robot senses no obstacles within distance,
    if (curDir == kForward && (millis () - curDirTs) > 20000) {
      move(kStop), delay(10);     // Motors Coast
      move((rand ()%100 >50)?kRight:kLeft), delay(300);  // Motors Turn Right
      move(kStop), delay(100);    // Motors Coast
    } else {
      move(kForward), delay(100);  // Move Forward
    }
  } else {
    // If robot senses somthing within distance
    move (kStop);
    turnServo(-40);  // Turn Head Right
    delay (500);
    distance = measureDistance();
    if (distance >= 30 || distance <= 0) {
      move(kBackward), delay(50); // Motors Back Up
      move(kStop), delay(10);     // Motors Coast
      move(kRight), delay(300);  // Motors Turn Right
      move(kStop), delay(100);    // Motors Coast
    } else {
      move (kStop);
      turnServo(40);       // Turn Head Left
      delay (500);
      move(kBackward), delay(50);   // Motors Back Up
      move(kStop), delay(10);    // Motors Coast
      move(kLeft), delay(300);  // Motors Turn Left
      move(kStop), delay(100);   // Motors Coast
    }
  }
}

void loopManual () {
//  distance = measureDistanceAsync();
}

void loop() {
  // Turn Head to Center
  server.handleClient();
  if (scheduledStopTs && millis () > scheduledStopTs) {
    scheduledStopTs = 0;
    move (kStop);
  }
  if (scheduledServoStopTs && millis () > scheduledServoStopTs) {
    scheduledServoStopTs = 0;
    servo.detach ();
  }
  if (needMusic || millis() > lastMusicTs + 30000) {
     move(kStop), delay(10);
     playMusic ();
     lastMusicTs = millis ();
      needMusic = false;
  } 
   
  switch (curMode) {
    case kModeAuto:
      loopAuto ();
      break;
    case kModeManual:
      loopManual ();
      break;
  }
}


// cat scruface.html | xxd -i >scruface.html.h
const char page[] = {
#include "scruface.html.h"
,0};

