#ifndef MOTOR_ENCODER_H
#define MOTOR_ENCODER_H

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

class MotorEncoder {
public:
    MotorEncoder(TIM_TypeDef *timer, bool reverse = false);
    MotorEncoder(uint8_t pinA, uint8_t pinB, bool reverse = false);

    void begin();
    long getCount() const;
    void reset();
    void update();
    void setReverse(bool reverse);

private:
    TIM_TypeDef *_timer;
    uint8_t _pinA;
    uint8_t _pinB;
    bool _reverse;
    mutable volatile int32_t _totalCount;
    mutable volatile uint32_t _lastRawCnt;
};

#endif // MOTOR_ENCODER_H


