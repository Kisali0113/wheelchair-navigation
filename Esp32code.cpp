#include <ESP32Servo.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"

// ─── WiFi Credentials ────────────────────────────────
const char* SSID = "YOUR_SSID";
const char* PASSWORD = "YOUR_PASSWORD";

// ─── Firebase Credentials ────────────────────────────
const char* API_KEY = "AIzaSyB9G_7nb0FC2FQKwVBDHtcCh7XBc_E2OHE";
const char* DATABASE_URL = "https://smart-wheelchair-91084-default-rtdb.asia-southeast1.firebasedatabase.app";
const char* USER_EMAIL = "kisalithumara2002@gmail.com";
const char* USER_PASSWORD = "kslthmr2002@";

// ─── Firebase Objects ────────────────────────────────
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
unsigned long sendDataPrevMillis = 0;
bool signupOK = false;

// ─── Pins ─────────────────────────────────────────────
const int Left_PIN_SERVO  = 5;
const int Right_PIN_SERVO = 4;

const int PIN_BTN_OPEN  = 2;
const int PIN_BTN_CLOSE = 15;

// ─── Servo Positions ─────────────────────────────────
const int ANGLE_OPEN_LEFT     = 95;
const int ANGLE_CLOSED_LEFT   = 85;

const int ANGLE_OPEN_RIGHT    = 90;
const int ANGLE_CLOSED_RIGHT  = 98;

// ─── Movement Timing ───────────────────────────────── 
const int STEP_DELAY_MS = 200;

// ─── Gate Status ────────────────────────────────────
enum GateStatus { GATE_CLOSED, GATE_OPEN };
GateStatus gateStatus = GATE_CLOSED;
unsigned long lastStatusUpdate = 0;
const unsigned long STATUS_UPDATE_INTERVAL = 2000; // Update Firebase every 2 seconds

// ─── Servo Objects ───────────────────────────────────
Servo servoLeft;
Servo servoRight;

// Start in closed position
int currentAngleLeft  = ANGLE_CLOSED_LEFT;
int currentAngleRight = ANGLE_CLOSED_RIGHT;

// ─── Open Gates ──────────────────────────────────────
void openGates() {

  if (currentAngleLeft == ANGLE_OPEN_LEFT &&
      currentAngleRight == ANGLE_OPEN_RIGHT) {
    return;
  }

  while (currentAngleLeft != ANGLE_OPEN_LEFT ||
         currentAngleRight != ANGLE_OPEN_RIGHT) {

    if (currentAngleLeft < ANGLE_OPEN_LEFT) {
      currentAngleLeft++;
      servoLeft.write(currentAngleLeft);
    }

    if (currentAngleRight > ANGLE_OPEN_RIGHT) {
      currentAngleRight--;
      servoRight.write(currentAngleRight);
    }

    delay(STEP_DELAY_MS);
  }

  // Update gate status
  gateStatus = GATE_OPEN;
  publishGateStatus();
}

// ─── Close Gates ─────────────────────────────────────
void closeGates() {

  if (currentAngleLeft == ANGLE_CLOSED_LEFT &&
      currentAngleRight == ANGLE_CLOSED_RIGHT) {
    return;
  }

  while (currentAngleLeft != ANGLE_CLOSED_LEFT ||
         currentAngleRight != ANGLE_CLOSED_RIGHT) {

    if (currentAngleLeft > ANGLE_CLOSED_LEFT) {
      currentAngleLeft--;
      servoLeft.write(currentAngleLeft);
    }

    if (currentAngleRight < ANGLE_CLOSED_RIGHT) {
      currentAngleRight++;
      servoRight.write(currentAngleRight);
    }

    delay(STEP_DELAY_MS);
  }

  // Update gate status
  gateStatus = GATE_CLOSED;
  publishGateStatus();
}

// ─── Publish Gate Status to Firebase ─────────────────
void publishGateStatus() {

  if (!signupOK) return;

  bool isOpen = (gateStatus == GATE_OPEN);

  if (Firebase.RTDB.setBool(
        &fbdo,
        "/wheelchair/isOpen",
        isOpen))
  {
      Serial.print("Updated isOpen = ");
      Serial.println(isOpen);
  }
  else
  {
      Serial.print("Firebase update failed: ");
      Serial.println(fbdo.errorReason());
  }
}


// ─── Check Firebase Commands ─────────────────────────
void checkFirebaseState() {

  if (!signupOK) return;

  if (Firebase.RTDB.getBool(
        &fbdo,
        "/wheelchair/isOpen"))
  {
      bool requestedState = fbdo.boolData();

      if (requestedState &&
          gateStatus != GATE_OPEN)
      {
          Serial.println(
              "Firestore requested OPEN"
          );

          openGates();
      }

      else if (!requestedState &&
               gateStatus != GATE_CLOSED)
      {
          Serial.println(
              "Firestore requested CLOSE"
          );

          closeGates();
      }
  }
}

// ─── WiFi & Firebase Setup ──────────────────────────
void setupWiFiAndFirebase() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(SSID);

  WiFi.begin(SSID, PASSWORD);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nFailed to connect to WiFi");
    return;
  }

  // Setup Firebase
  config.api_key = API_KEY;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  config.database_url = DATABASE_URL;
  config.token_status_callback = NULL;

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  // Wait for authentication
  unsigned long ms = millis();
  while (!Firebase.ready() && (millis() - ms) < 3000) {
    delay(100);
  }

  signupOK = Firebase.signUp(&config, &auth, USER_EMAIL, USER_PASSWORD);
  if (!signupOK) {
    Serial.println("Firebase sign-up failed");
  } else {
    Serial.println("Firebase connected!");
  }
}

// ─── Button Handling ─────────────────────────────────
void checkButtons() {

  static bool lastOpenState  = HIGH;
  static bool lastCloseState = HIGH;

  bool openState  = digitalRead(PIN_BTN_OPEN);
  bool closeState = digitalRead(PIN_BTN_CLOSE);

  // Detect button press (HIGH → LOW)
  if (lastOpenState == HIGH && openState == LOW) {
    openGates();
  }

  if (lastCloseState == HIGH && closeState == LOW) {
    closeGates();
  }

  lastOpenState  = openState;
  lastCloseState = closeState;
}

// ─── Setup ───────────────────────────────────────────
void setup() {

  Serial.begin(115200);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);

  servoLeft.setPeriodHertz(50);
  servoLeft.attach(Left_PIN_SERVO, 500, 2500);

  servoRight.setPeriodHertz(50);
  servoRight.attach(Right_PIN_SERVO, 500, 2500);

  pinMode(PIN_BTN_OPEN, INPUT_PULLUP);
  pinMode(PIN_BTN_CLOSE, INPUT_PULLUP);

  // Move to closed position at startup
  servoLeft.write(currentAngleLeft);
  servoRight.write(currentAngleRight);

  // Initialize WiFi and Firebase
  setupWiFiAndFirebase();
  
  delay(300);
}

// ─── Loop ────────────────────────────────────────────
void loop() {

  // Check push buttons
  checkButtons();

  // Check Firebase commands
  checkFirebaseState();

  // Publish gate status periodically
   publishGateStatus();

  // Serial commands
  if (Serial.available()) {

    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toLowerCase();

    if (cmd == "open") {
      openGates();
    }
    else if (cmd == "close") {
      closeGates();
    }
  }

  delay(10);
}