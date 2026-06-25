#include "arm_servo.h"

#include <Wire.h>

static constexpr uint8_t PCA9685_ADDRESS = 0x40;
static constexpr uint8_t PCA9685_MODE1 = 0x00;
static constexpr uint8_t PCA9685_PRESCALE = 0xFE;
static constexpr uint8_t PCA9685_LED0_ON_L = 0x06;
static constexpr uint8_t PCA9685_RESTART = 0x80;
static constexpr uint8_t PCA9685_SLEEP = 0x10;
static constexpr uint8_t PCA9685_ALLCALL = 0x01;
static constexpr uint16_t SERVO_MIN_PULSE = 75;
static constexpr uint16_t SERVO_MAX_PULSE = 510;
static constexpr uint8_t SERVO_STEP_SIZE = 1;
static constexpr uint16_t SERVO_PWM_FREQ_HZ = 50;
static constexpr float PCA9685_CLOCK_HZ = 25000000.0f;

// 复位位和抓取位沿用原 JXB.ino 的 positionA / positionB。
static const int ARM_RESET_POSITION[ARM_SERVO_COUNT] = {90, 0, 180, 130, 90};
static const int ARM_GRAB_POSITION[ARM_SERVO_COUNT] = {120, 30, 90, 90, 90};

static void writePca9685Register(uint8_t reg, uint8_t value);
static uint8_t readPca9685Register(uint8_t reg);
static void setPca9685PwmFreq(uint16_t frequencyHz);
static void setPca9685Pwm(uint8_t channel, uint16_t onTick, uint16_t offTick);

void ArmServo::begin()
{
  Wire.begin();

  // 这里直接配置 PCA9685，避免额外依赖 Adafruit_PWMServoDriver。
  writePca9685Register(PCA9685_MODE1, PCA9685_ALLCALL);
  delay(10);
  setPca9685PwmFreq(SERVO_PWM_FREQ_HZ);
  delay(500);

  for (uint8_t channel = 0; channel < ARM_SERVO_COUNT; channel++) {
    setAngle(channel, currentAngles[channel]);
    delay(100);
  }

  isInitialized = true;
}

void ArmServo::update()
{
  if (!isInitialized) {
    return;
  }

  // update() 不阻塞等待动作完成，而是每次只移动一步，方便主程序继续处理串口和 PID。
  for (uint8_t channel = 0; channel < ARM_SERVO_COUNT; channel++) {
    smoothMoveServo(channel, targetAngles[channel]);
  }
}

bool ArmServo::startGrab()
{
  copyPresetToTarget(ARM_GRAB_POSITION);
  return true;
}

bool ArmServo::startReset()
{
  copyPresetToTarget(ARM_RESET_POSITION);
  return true;
}

bool ArmServo::stop()
{
  for (uint8_t channel = 0; channel < ARM_SERVO_COUNT; channel++) {
    targetAngles[channel] = currentAngles[channel];
  }

  return true;
}

bool ArmServo::setTargetAngle(uint8_t channel, int angle)
{
  if (channel >= ARM_SERVO_COUNT) {
    return false;
  }

  targetAngles[channel] = constrain(angle, 0, 180);
  return true;
}

bool ArmServo::setTargetAngles(const int angles[], uint8_t count)
{
  if (angles == nullptr || count != ARM_SERVO_COUNT) {
    return false;
  }

  for (uint8_t channel = 0; channel < ARM_SERVO_COUNT; channel++) {
    targetAngles[channel] = constrain(angles[channel], 0, 180);
  }

  return true;
}

bool ArmServo::isBusy() const
{
  return !isAtTarget();
}

bool ArmServo::isAtTarget() const
{
  for (uint8_t channel = 0; channel < ARM_SERVO_COUNT; channel++) {
    if (currentAngles[channel] != targetAngles[channel]) {
      return false;
    }
  }

  return true;
}

int ArmServo::getCurrentAngle(uint8_t channel) const
{
  if (channel >= ARM_SERVO_COUNT) {
    return -1;
  }

  return currentAngles[channel];
}

int ArmServo::getTargetAngle(uint8_t channel) const
{
  if (channel >= ARM_SERVO_COUNT) {
    return -1;
  }

  return targetAngles[channel];
}

void ArmServo::setAngle(uint8_t channel, int angle)
{
  angle = constrain(angle, 0, 180);

  // PCA9685 使用 0 到 4095 的计数值表示一个 PWM 周期内的高电平宽度。
  // 这里保留原程序的 75 到 510 计数范围，避免实物舵机行程突然改变。
  const int pulse = map(angle, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  setPca9685Pwm(channel, 0, static_cast<uint16_t>(pulse));
}

bool ArmServo::smoothMoveServo(uint8_t channel, int targetAngle)
{
  int currentAngle = currentAngles[channel];

  // 每次只变化 SERVO_STEP_SIZE 度，让机械臂动作更平滑，也减少舵机瞬间电流冲击。
  if (currentAngle < targetAngle) {
    currentAngle = min(currentAngle + SERVO_STEP_SIZE, targetAngle);
  } else if (currentAngle > targetAngle) {
    currentAngle = max(currentAngle - SERVO_STEP_SIZE, targetAngle);
  }

  setAngle(channel, currentAngle);
  currentAngles[channel] = currentAngle;
  return currentAngle == targetAngle;
}

void ArmServo::copyPresetToTarget(const int presetAngles[])
{
  for (uint8_t channel = 0; channel < ARM_SERVO_COUNT; channel++) {
    targetAngles[channel] = constrain(presetAngles[channel], 0, 180);
  }
}

static void writePca9685Register(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(PCA9685_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

static uint8_t readPca9685Register(uint8_t reg)
{
  Wire.beginTransmission(PCA9685_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(PCA9685_ADDRESS, static_cast<uint8_t>(1));
  if (!Wire.available()) {
    return 0;
  }

  return Wire.read();
}

static void setPca9685PwmFreq(uint16_t frequencyHz)
{
  // PCA9685 需要先进入 sleep 模式才能安全修改预分频寄存器。
  const float prescaleValue = (PCA9685_CLOCK_HZ / (4096.0f * frequencyHz)) - 1.0f;
  const uint8_t prescale = static_cast<uint8_t>(prescaleValue + 0.5f);
  const uint8_t oldMode = readPca9685Register(PCA9685_MODE1);
  const uint8_t sleepMode = (oldMode & ~PCA9685_RESTART) | PCA9685_SLEEP;

  writePca9685Register(PCA9685_MODE1, sleepMode);
  writePca9685Register(PCA9685_PRESCALE, prescale);
  writePca9685Register(PCA9685_MODE1, oldMode);
  delay(5);
  writePca9685Register(PCA9685_MODE1, oldMode | PCA9685_RESTART);
}

static void setPca9685Pwm(uint8_t channel, uint16_t onTick, uint16_t offTick)
{
  // 每个通道占 4 个连续寄存器：ON_L、ON_H、OFF_L、OFF_H。
  const uint8_t baseRegister = PCA9685_LED0_ON_L + (4 * channel);

  Wire.beginTransmission(PCA9685_ADDRESS);
  Wire.write(baseRegister);
  Wire.write(onTick & 0xFF);
  Wire.write(onTick >> 8);
  Wire.write(offTick & 0xFF);
  Wire.write(offTick >> 8);
  Wire.endTransmission();
}
