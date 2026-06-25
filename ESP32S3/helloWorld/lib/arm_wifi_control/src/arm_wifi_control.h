#pragma once

#include <Arduino.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <arm_servo.h>

static constexpr float ARM_WIFI_SPEED_LIMIT_CM_S = 20.0f; // 0.2 m/s

class ArmWifiControl {
public:
  ArmWifiControl();

  void begin(ArmServo &armServo, const char *stationSsid, const char *stationPassword);
  void update();

  bool isSpeedLimited() const;
  float getSpeedLimitCmS() const;
  void limitTargetSpeeds(float *leftSpeedCmS, float *rightSpeedCmS) const;

private:
  void startWifi(const char *stationSsid, const char *stationPassword);
  bool connectStation(const char *stationSsid, const char *stationPassword);
  void startFallbackAccessPoint();
  void handleUdpPacket();
  void checkUdpTimeout();
  void handleUdpCommand(const char *command);
  bool parseMoveJoints(const char *command);
  bool readJointArray(const char *start);
  void replyUdp(const String &message);

  void handleRoot();
  void handleStatus();
  void handleGrab();
  void handleReset();
  void handleStop();
  void handleEndControl();
  void handleCarWifiPlaceholder();
  void handleNotFound();

  void markArmControlActive();
  String buildStatusJson() const;

  ArmServo *controlledArmServo = nullptr;
  WebServer server;
  WiFiUDP udp;
  IPAddress controlIp;
  bool isAccessPointMode = false;
  bool isArmControlActive = false;
  bool isUdpSessionActive = false;
  uint32_t lastUdpPacketMs = 0;
  char udpBuffer[256] = {};
};
