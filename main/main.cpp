/* T-wristband hardware usage example */
#include <iostream>
#include <thread>
#include <chrono>
#include <memory>
#include <string>
#include <sstream>
#include <cstring>
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_pthread.h>
#include <esp_system.h>
#include <driver/gpio.h>
#include <esp_heap_caps.h>
#include "hw_layer.h"
#include "tft/tftspi.h"
#include "tft/tft.h"

enum class DemoFunc: unsigned char {
    Watch = 0,
    Gyro,
    Accel,
    LastItem
};

DemoFunc& operator++(DemoFunc &_m) {
    return _m = (_m == DemoFunc::Accel) ? DemoFunc::Watch : static_cast<DemoFunc>(static_cast<unsigned char>(_m)+1);
}

using namespace std::chrono;
std::atomic<DemoFunc> current_demo;
const auto sleep_time = seconds{100};
static char tmp_buff[64];

void print_thread_info(const char *extra = nullptr)
{
    std::stringstream ss;
    if (extra) {
        ss << extra;
    }
    ss << "Core id: " << xPortGetCoreID()
       << ", prio: " << uxTaskPriorityGet(nullptr)
       << ", minimum free stack: " << uxTaskGetStackHighWaterMark(nullptr) << " bytes.";
    ESP_LOGI(pcTaskGetTaskName(nullptr), "%s", ss.str().c_str());
}

void display_time_thread()
{
    struct tm tm_time = {};
    const size_t delimetr_size = 2;
    char time_delimiter[delimetr_size] = {};
    while (true) {
        switch(current_demo.load()) {
            case DemoFunc::Watch:
                ESP_LOGI(pcTaskGetTaskName(nullptr),"Watch item");
                tm_time = HWLayer::getInstance()->getTime();
                _bg = TFT_BLACK;
                _fg = TFT_GREEN;
                TFT_fillScreen(TFT_BLACK);
                TFT_setFont(FONT_7SEG, NULL);
                strncpy(time_delimiter, tm_time.tm_sec%2==0 ? " " : ":", delimetr_size);
                sprintf(tmp_buff, "%02d%s%02d", tm_time.tm_hour, time_delimiter, tm_time.tm_min);
                TFT_print(tmp_buff, CENTER, _height-TFT_getfontheight()-20);
            break;
            case DemoFunc::Gyro:
            {
                ESP_LOGI(pcTaskGetTaskName(nullptr),"Gyro item");
                print_thread_info();
                auto gyro = HWLayer::getInstance()->getGyro();
                _bg = TFT_BLACK;
                _fg = TFT_GREEN;
                TFT_fillScreen(TFT_BLACK);
                TFT_setFont(UBUNTU16_FONT, NULL);
                sprintf(tmp_buff, "Gyro:\n x=%+7.2f\n y=%+7.2f\n z=%+7.2f\n", gyro[0], gyro[1], gyro[2]);
                TFT_print(tmp_buff, CENTER, _height-TFT_getfontheight()-50);
            }
            break;
            case DemoFunc::Accel:
            {
                ESP_LOGI(pcTaskGetTaskName(nullptr),"Accel item");
                print_thread_info();
                auto accel = HWLayer::getInstance()->getAccel();
                _bg = TFT_BLACK;
                _fg = TFT_GREEN;
                TFT_fillScreen(TFT_BLACK);
                TFT_setFont(UBUNTU16_FONT, NULL);
                sprintf(tmp_buff, "Accel:\n x=%+4.2f\n y=%+4.2f\n z=%+4.2f\n", accel.x, accel.y, accel.z);
                TFT_print(tmp_buff, CENTER, _height-TFT_getfontheight()-50);
            }
            break;
            default:
                ESP_LOGI(pcTaskGetTaskName(nullptr),"Unhandle item");
            break;
        }
        std::this_thread::sleep_for(milliseconds{500});
    }
}

void btn_test_thread()
{
    DemoFunc curent_item = DemoFunc::Watch;
    while (true) {
        HWLayer::ButtonPress_t press_type = HWLayer::getInstance()->isButtonPresssed();
        if (HWLayer::SHORT_PRESS == press_type) {
            print_thread_info("Buttor is pressed");
            current_demo.store(++curent_item);
            ESP_LOGI(pcTaskGetTaskName(nullptr),"new item %d", static_cast<int>(current_demo.load()));
        }
        else if (HWLayer::LONG_PRESS == press_type) {
            print_thread_info("Buttor long pressed");
        }
        std::this_thread::sleep_for(seconds{1});
    }
}

esp_pthread_cfg_t create_config(const char *name, int core_id, int stack, int prio)
{
    auto cfg = esp_pthread_get_default_config();
    cfg.thread_name = name;
    cfg.pin_to_core = core_id;
    cfg.stack_size = stack;
    cfg.prio = prio;
    return cfg;
}

extern "C" void app_main(void)
{
    //Init HW
    HWLayer* wristBandHw = HWLayer::getInstance();
    wristBandHw->espSetup();
    wristBandHw->tftSetup();
    wristBandHw->rtcSetup();
    wristBandHw->gyroSetup();
    current_demo.store(DemoFunc::Watch);

    // Create a thread using deafult values that can run on any core
    auto cfg = esp_pthread_get_default_config();
    cfg.thread_name = "Btn demo";
    esp_pthread_set_cfg(&cfg);
    std::thread any_core(btn_test_thread);

    // Create a thread on core 0 that display time
    cfg = create_config("Time thread", 0, 3 * 1024, 5);
    cfg.inherit_cfg = true;
    esp_pthread_set_cfg(&cfg);
    std::thread thread_1(display_time_thread);

    // Let the main task do something too
    while (true) {
        std::stringstream ss;
        ss << "core id: " << xPortGetCoreID()
           << ", prio: " << uxTaskPriorityGet(nullptr)
           << ", minimum free stack: " << uxTaskGetStackHighWaterMark(nullptr) << " bytes.";
        ESP_LOGI(pcTaskGetTaskName(nullptr), "%s", ss.str().c_str());
        std::this_thread::sleep_for(sleep_time);
    }
}
