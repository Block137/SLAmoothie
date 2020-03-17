#include "RBMTest.h"

void RBMTest::on_module_loaded()
{
    this->register_for_event(ON_MAIN_LOOP);
    latch_pin.output();
    latch_pin.clear();
    _spidac.format(8,0);   //support 4-16 bit, 8bit/mode0 is default
    initDAC8760();
    
    _spidac.frequency(7500000);
}

extern GPIO leds[];
void RBMTest::on_main_loop(void*)
{
    static uint16_t cnt = 0;
//    if(!(cnt & 0x0003)) {
//        angle = ((float)(cnt&0x03FF)) / 162.97466165F;
        latch_pin.clear();
        if((cnt & 0x0300) == 0x0000) {
            data_x = (cnt&0x00FF) << 8;
        }
        else {
            if((cnt & 0x0300) == 0x0100) {
                data_y = (cnt&0x00FF) << 8;
            }
            else {
                if((cnt & 0x0300) == 0x0200) {
                    data_x = (~(cnt&0x00FF)) << 8;     
                }
                else {
                    data_y = (~(cnt&0x00FF)) << 8;
                }
            }
        }

        
//        data_x = (uint16_t)(32763.0F + (32759.0F * cosf(angle)));
//        data_y = (uint16_t)(32763.0F + (32759.0F * sinf(angle)));
        _spidac.write(0x01);
        _spidac.write(data_y >> 8);
        _spidac.write(data_y & 0xFF);
        _spidac.write(0x01);
        _spidac.write(data_x >> 8);
        _spidac.write(data_x & 0xFF);
        latch_pin.set();
//    }
    leds[1]= (cnt++ & 0x4000) ? 0 : 1;
}

void RBMTest::initDAC8760()
{
    latch_pin.clear();
    _spidac.write(0x55);    //control, enable DaisyChain but not enabled output
    _spidac.write(0x01);
    _spidac.write(0xEA);
    leds[1] = 1;
    latch_pin.set();
    _spidac.write(0x55);
    _spidac.write(0x11);
    _spidac.write(0xEA);
    leds[1] = 0;
    latch_pin.clear();
    _spidac.write(0x55);
    _spidac.write(0x11);
    _spidac.write(0xEA);
    leds[1] = 1;
    latch_pin.set();
    _spidac.write(0x57); //config, all default
    _spidac.write(0x00);
    _spidac.write(0x00);
    leds[1] = 0;
    latch_pin.clear();
    _spidac.write(0x57);
    _spidac.write(0x00);
    _spidac.write(0x00);
    leds[1] = 1;
    latch_pin.set();
//    leds[1] = 0;
//    latch_pin = 0;
}