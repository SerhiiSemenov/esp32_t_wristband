/* pthread/std::thread example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <iostream>
#include <thread>
#include <chrono>
#include <memory>
#include <string>
#include <sstream>
#include <cstring>
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

using namespace std::chrono;
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
    while (true) {
        tm_time = HWLayer::getInstance()->getTime();
        _bg = TFT_BLACK;
        _fg = TFT_GREEN;

        TFT_fillScreen(TFT_BLACK);
        TFT_setFont(COMIC24_FONT, NULL);
        sprintf(tmp_buff, "%02d:%02d:%02d", tm_time.tm_hour, tm_time.tm_min, tm_time.tm_sec);
        TFT_print(tmp_buff, CENTER, _height-TFT_getfontheight()-50);
        std::this_thread::sleep_for(seconds(1));
    }
}

void btn_test_thread()
{
    while (true) {
        HWLayer::ButtonPress_t press_type = HWLayer::getInstance()->isButtonPresssed();
        if (HWLayer::SHORT_PRESS == press_type) {
            print_thread_info("Buttor is pressed");
        }
        else if (HWLayer::LONG_PRESS == press_type) {
            print_thread_info("Buttor long pressed");
        }
        std::this_thread::sleep_for(seconds{1});
    }
}

void gyro_test_thread()
{
    while (true) {
        print_thread_info();
        HWLayer::getInstance()->gyroTest();
        std::this_thread::sleep_for(milliseconds{500});
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

    // Create a thread on core 1
    cfg = create_config("Gyro demo thread", 1, 3 * 1024, 5);
    esp_pthread_set_cfg(&cfg);
    std::thread thread_2(gyro_test_thread);

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
