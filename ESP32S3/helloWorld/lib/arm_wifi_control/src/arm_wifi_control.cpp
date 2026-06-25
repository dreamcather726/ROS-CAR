#include "arm_wifi_control.h"

#include <WiFi.h>
#include <string.h>

static constexpr char ARM_WIFI_AP_SSID[] = "ROS-CAR-Arm";
static constexpr char ARM_WIFI_AP_PASSWORD[] = "12345678";
static constexpr uint16_t ARM_WIFI_UDP_PORT = 8888;
static constexpr uint32_t ARM_WIFI_UDP_TIMEOUT_MS = 1500;
static constexpr float NORMAL_SPEED_LIMIT_CM_S = 80.0f;

ArmWifiControl::ArmWifiControl()
    : server(80)
{
}

void ArmWifiControl::begin(ArmServo &armServo)
{
  controlledArmServo = &armServo;

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ARM_WIFI_AP_SSID, ARM_WIFI_AP_PASSWORD);

  server.on("/", HTTP_GET, [this]() { handleRoot(); });
  server.on("/status", HTTP_GET, [this]() { handleStatus(); });
  server.on("/arm/status", HTTP_GET, [this]() { handleStatus(); });
  server.on("/arm/grab", HTTP_GET, [this]() { handleGrab(); });
  server.on("/arm/reset", HTTP_GET, [this]() { handleReset(); });
  server.on("/arm/stop", HTTP_GET, [this]() { handleStop(); });
  server.on("/arm/end", HTTP_GET, [this]() { handleEndControl(); });
  server.on("/car/status", HTTP_GET, [this]() { handleCarWifiPlaceholder(); });
  server.onNotFound([this]() { handleNotFound(); });
  server.begin();
  udp.begin(ARM_WIFI_UDP_PORT);

  Serial.print("Arm WiFi SSID: ");
  Serial.println(ARM_WIFI_AP_SSID);
  Serial.print("Arm WiFi page: http://");
  Serial.println(WiFi.softAPIP());
  Serial.print("PyBullet UDP target: ");
  Serial.print(WiFi.softAPIP());
  Serial.print(":");
  Serial.println(ARM_WIFI_UDP_PORT);
}

void ArmWifiControl::update()
{
  server.handleClient();
  handleUdpPacket();
  checkUdpTimeout();
}

bool ArmWifiControl::isSpeedLimited() const
{
  return isArmControlActive || (controlledArmServo != nullptr && controlledArmServo->isBusy());
}

float ArmWifiControl::getSpeedLimitCmS() const
{
  return ARM_WIFI_SPEED_LIMIT_CM_S;
}

void ArmWifiControl::limitTargetSpeeds(float *leftSpeedCmS, float *rightSpeedCmS) const
{
  if (leftSpeedCmS != nullptr) {
    *leftSpeedCmS = constrain(*leftSpeedCmS,
                              -ARM_WIFI_SPEED_LIMIT_CM_S,
                              ARM_WIFI_SPEED_LIMIT_CM_S);
  }
  if (rightSpeedCmS != nullptr) {
    *rightSpeedCmS = constrain(*rightSpeedCmS,
                               -ARM_WIFI_SPEED_LIMIT_CM_S,
                               ARM_WIFI_SPEED_LIMIT_CM_S);
  }
}

void ArmWifiControl::handleUdpPacket()
{
  const int packetSize = udp.parsePacket();
  if (packetSize <= 0) {
    return;
  }

  const int length = udp.read(udpBuffer, sizeof(udpBuffer) - 1);
  if (length <= 0) {
    return;
  }

  udpBuffer[length] = '\0';
  lastUdpPacketMs = millis();
  isUdpSessionActive = true;
  handleUdpCommand(udpBuffer);
}

void ArmWifiControl::checkUdpTimeout()
{
  if (!isUdpSessionActive || !isArmControlActive || controlledArmServo == nullptr) {
    return;
  }

  if (millis() - lastUdpPacketMs <= ARM_WIFI_UDP_TIMEOUT_MS) {
    return;
  }

  if (!controlledArmServo->isBusy()) {
    isArmControlActive = false;
  }
}

void ArmWifiControl::handleUdpCommand(const char *command)
{
  if (strcmp(command, "ping") == 0) {
    replyUdp("pong");
    return;
  }

  if (strcmp(command, "heartbeat") == 0 || strcmp(command, "status") == 0) {
    replyUdp(buildStatusJson());
    return;
  }

  if (strcmp(command, "stop") == 0) {
    if (controlledArmServo != nullptr) {
      controlledArmServo->stop();
    }
    markArmControlActive();
    replyUdp("stopped");
    return;
  }

  if (strcmp(command, "reset") == 0) {
    if (controlledArmServo != nullptr) {
      controlledArmServo->startReset();
    }
    markArmControlActive();
    replyUdp("reset ok");
    return;
  }

  if (parseMoveJoints(command)) {
    markArmControlActive();
    replyUdp("movej ok");
    return;
  }

  replyUdp("unknown command");
}

bool ArmWifiControl::parseMoveJoints(const char *command)
{
  static constexpr char MOVE_PREFIX[] = "movej:";
  if (strncmp(command, MOVE_PREFIX, strlen(MOVE_PREFIX)) == 0) {
    return readJointArray(command + strlen(MOVE_PREFIX));
  }

  const char *joints = strstr(command, "\"joints\"");
  if (joints == nullptr) {
    joints = strstr(command, "joints");
  }
  if (joints == nullptr) {
    return false;
  }

  const char *arrayStart = strchr(joints, '[');
  if (arrayStart == nullptr) {
    return false;
  }

  return readJointArray(arrayStart + 1);
}

bool ArmWifiControl::readJointArray(const char *start)
{
  if (controlledArmServo == nullptr) {
    return false;
  }

  int targetAngles[ARM_SERVO_COUNT] = {};
  uint8_t angleCount = 0;
  const char *cursor = start;

  while (*cursor != '\0' && angleCount < ARM_SERVO_COUNT) {
    char *endPtr = nullptr;
    const long rawAngle = strtol(cursor, &endPtr, 10);
    if (cursor == endPtr) {
      cursor++;
      continue;
    }

    targetAngles[angleCount] = constrain(static_cast<int>(rawAngle), 0, 180);
    angleCount++;
    cursor = endPtr;
  }

  if (angleCount != ARM_SERVO_COUNT) {
    return false;
  }

  return controlledArmServo->setTargetAngles(targetAngles, ARM_SERVO_COUNT);
}

void ArmWifiControl::replyUdp(const String &message)
{
  udp.beginPacket(udp.remoteIP(), udp.remotePort());
  udp.print(message);
  udp.endPacket();
}

void ArmWifiControl::handleRoot()
{
  const String html =
      "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<title>ROS-CAR Arm</title></head><body>"
      "<h2>ROS-CAR Arm WiFi</h2>"
      "<p>PyBullet UDP target: 192.168.4.1:8888</p>"
      "<p>Commands: movej:90,0,180,130,90 / reset / stop / status</p>"
      "<p><a href='/status'>Status JSON</a></p>"
      "</body></html>";
  server.send(200, "text/html", html);
}

void ArmWifiControl::handleStatus()
{
  server.send(200, "application/json", buildStatusJson());
}

void ArmWifiControl::handleGrab()
{
  if (controlledArmServo != nullptr) {
    controlledArmServo->startGrab();
  }
  markArmControlActive();
  server.send(200, "text/plain", "Grab started");
}

void ArmWifiControl::handleReset()
{
  if (controlledArmServo != nullptr) {
    controlledArmServo->startReset();
  }
  markArmControlActive();
  server.send(200, "text/plain", "Reset started");
}

void ArmWifiControl::handleStop()
{
  if (controlledArmServo != nullptr) {
    controlledArmServo->stop();
  }
  markArmControlActive();
  server.send(200, "text/plain", "Arm stopped");
}

void ArmWifiControl::handleEndControl()
{
  if (controlledArmServo != nullptr && controlledArmServo->isBusy()) {
    server.send(409, "text/plain", "Arm is still moving");
    return;
  }

  isArmControlActive = false;
  server.send(200, "text/plain", "Arm mode ended");
}

void ArmWifiControl::handleCarWifiPlaceholder()
{
  server.send(200,
              "text/plain",
              "Reserved for WiFi car control. Prefer keeping chassis control in the upper-computer or ROS layer so all motion commands share one safety policy.");
}

void ArmWifiControl::handleNotFound()
{
  server.send(404, "text/plain", "Not found");
}

void ArmWifiControl::markArmControlActive()
{
  isArmControlActive = true;
}

String ArmWifiControl::buildStatusJson() const
{
  String json = "{\"armControlActive\":";
  json += isArmControlActive ? "true" : "false";
  json += ",\"busy\":";
  json += (controlledArmServo != nullptr && controlledArmServo->isBusy()) ? "true" : "false";
  json += ",\"udpActive\":";
  json += isUdpSessionActive ? "true" : "false";
  json += ",\"speedLimitCmS\":";
  json += String(isSpeedLimited() ? ARM_WIFI_SPEED_LIMIT_CM_S : NORMAL_SPEED_LIMIT_CM_S);
  json += ",\"current\":[";
  for (uint8_t channel = 0; channel < ARM_SERVO_COUNT; channel++) {
    if (channel > 0) json += ",";
    json += String(controlledArmServo == nullptr ? -1 : controlledArmServo->getCurrentAngle(channel));
  }
  json += "],\"target\":[";
  for (uint8_t channel = 0; channel < ARM_SERVO_COUNT; channel++) {
    if (channel > 0) json += ",";
    json += String(controlledArmServo == nullptr ? -1 : controlledArmServo->getTargetAngle(channel));
  }
  json += "]}";
  return json;
}
