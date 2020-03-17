#pragma once

#include <stdint.h>
#include <math.h>

#include "Module.h"
#include "gpio.h"
#include "mbed.h"

class RBMTest : public Module {
    public:
        void on_module_loaded();
        void on_main_loop(void *argument);
        RBMTest() {}

    private:
        GPIO latch_pin = GPIO(P0_16);
        mbed::SPI _spidac = SPI(P0_18, P0_17, P0_15);   //MOSI,MISO,SCK
        
        void initDAC8760();
        
        uint16_t data_x;
        uint16_t data_y;
        float angle = 0.0F;
};