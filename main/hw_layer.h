#ifndef HW_LAYER_H
#define HW_LAYER_H

#include <iostream>
#include "driver/gpio.h"
#include "PCF8563.h"
#include "MPU.hpp"

//T-WRISTband pin defenition
#define LOW                 0
#define HIGH                1
#define TP_PIN_PIN                  GPIO_NUM_33
#define TP_PWR_PIN                  GPIO_NUM_25

#define I2C0_SDA_PIN         GPIO_NUM_21
#define I2C0_SCL_PIN         GPIO_NUM_22
#define IMU_INT_PIN         GPIO_NUM_38
#define RTC_INT_PIN         GPIO_NUM_34
#define BATT_ADC_PIN        GPIO_NUM_35
#define VBUS_PIN            GPIO_NUM_36
#define LED_PIN             GPIO_NUM_4
#define CHARGE_PIN          GPIO_NUM_32
#define CLOCK_SPEED         400000
//constexpr gpio_num_t I2C0_SDA_PIN = GPIO_NUM_21;
//constexpr gpio_num_t I2C0_SCL_PIN = GPIO_NUM_22;/
//constexpr uint32_t CLOCK_SPEED = 400000;

class HWLayer
{
public:
    enum ButtonPress_t
    {
        NOT_PRESSED = 0,
        SHORT_PRESS,
        LONG_PRESS
    };

    HWLayer(HWLayer const&) = delete;
    HWLayer(HWLayer const&&) = delete;
    HWLayer& operator=(HWLayer const&) = delete;

    static HWLayer* getInstance()
    {
        static HWLayer *inst_ptr = new HWLayer;
        return inst_ptr;
    }

    bool espSetup();
    void rtcSetup();
    void tftSetup();
    void gyroSetup();

    void LedOn();
    void LedOff();
    ButtonPress_t isButtonPresssed();

    tm getTime();
    void gyroTest();

private:
    HWLayer();
    ~HWLayer();
    mpud::MPU *m_mpu;
};

#endif // HW_LAYER_H
