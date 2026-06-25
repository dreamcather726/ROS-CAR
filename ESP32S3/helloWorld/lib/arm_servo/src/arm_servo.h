#pragma once

#include <Arduino.h>

static constexpr uint8_t ARM_SERVO_COUNT = 5;

// PCA9685 五路机械臂控制库。
// 主程序只需要调用 begin() 初始化，并在 loop() 里持续调用 update()。
class ArmServo {
public:
  // 初始化 I2C、PCA9685 和五个舵机的初始角度。
  void begin();

  // 每次 loop() 调用一次，用小步长把当前角度慢慢追到目标角度。
  void update();

  // 切到抓取姿态，对应原 JXB.ino 里的 positionB。
  bool startGrab();

  // 切回初始/复位姿态，对应原 JXB.ino 里的 positionA。
  bool startReset();

  // 立即停止继续运动：把目标角度改成当前角度。
  bool stop();

  // 设置单个舵机目标角度，channel 范围为 0 到 4，angle 范围自动限制到 0 到 180。
  bool setTargetAngle(uint8_t channel, int angle);

  // 一次设置五个舵机目标角度，count 必须等于 ARM_SERVO_COUNT。
  bool setTargetAngles(const int angles[], uint8_t count);

  // 只要还有舵机没有到达目标角度，就认为机械臂正在运动。
  bool isBusy() const;
  bool isAtTarget() const;

  // 返回 -1 表示 channel 超出范围。
  int getCurrentAngle(uint8_t channel) const;
  int getTargetAngle(uint8_t channel) const;

private:
  void setAngle(uint8_t channel, int angle);
  bool smoothMoveServo(uint8_t channel, int targetAngle);
  void copyPresetToTarget(const int presetAngles[]);

  bool isInitialized = false;
  int currentAngles[ARM_SERVO_COUNT] = {90, 0, 180, 130, 90};
  int targetAngles[ARM_SERVO_COUNT] = {90, 0, 180, 130, 90};
};
