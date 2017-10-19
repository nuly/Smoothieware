/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Kernel.h"

#include "modules/utils/simpleshell/SimpleShell.h"
#include "modules/utils/configurator/Configurator.h"
#include "modules/utils/currentcontrol/CurrentControl.h"
#include "modules/utils/killbutton/KillButton.h"
//#include "modules/encoder/RotaryEncoder.h"
#include "libs/Network/uip/Network.h"
#include "modules/pump/Pump.h"
#include "Config.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "StepTicker.h"
#include "SlowTicker.h"
//#include "Heater.h"
#include "HT16K33.h"

// #include "libs/ChaNFSSD/SDFileSystem.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"

// Debug
#include "libs/SerialMessage.h"

#include "libs/USBDevice/USB.h"
#include "libs/USBDevice/USBMSD/USBMSD.h"
#include "libs/USBDevice/USBMSD/SDCard.h"
#include "libs/USBDevice/USBSerial/USBSerial.h"
#include "libs/USBDevice/DFU.h"
#include "libs/SDFAT.h"
#include "StreamOutputPool.h"

#include "libs/Adc.h"
#include "libs/Watchdog.h"

#include "I2C.h"

#include "version.h"
#include "system_LPC17xx.h"
#include "platform_memory.h"

#include "mbed.h"
#include "libs/constants.h"

#define second_usb_serial_enable_checksum  CHECKSUM("second_usb_serial_enable")
#define disable_msd_checksum  CHECKSUM("msd_disable")
#define dfu_enable_checksum  CHECKSUM("dfu_enable")
#define watchdog_timeout_checksum  CHECKSUM("watchdog_timeout")


// USB Stuff
SDCard sd  __attribute__ ((section ("AHBSRAM0"))) (P0_9, P0_8, P0_7, P0_6);      // this selects SPI1 as the sdcard as it is on Smoothieboard
//SDCard sd(P0_18, P0_17, P0_15, P0_16);  // this selects SPI0 as the sdcard
//SDCard sd(P0_18, P0_17, P0_15, P2_8);  // this selects SPI0 as the sdcard witrh a different sd select

USB u __attribute__ ((section ("AHBSRAM0")));
USBSerial usbserial __attribute__ ((section ("AHBSRAM0"))) (&u);
#ifndef DISABLEMSD
USBMSD msc __attribute__ ((section ("AHBSRAM0"))) (&u, &sd);
#else
USBMSD *msc= NULL;
#endif

SDFAT mounter __attribute__ ((section ("AHBSRAM0"))) ("sd", &sd);

GPIO leds[5] = {
    GPIO(P1_18),
    GPIO(P1_19),
    GPIO(P1_20),
    GPIO(P1_21),
    GPIO(P4_28)
};

#define TOP_R  4.7
#define MAX_R  10.

#define HEATINVLEN 20

float HEATINV[] = 
{
1.0, 0.79804, 0.74109, 0.69896, 0.66368, 0.63237, 0.60358, 0.57645, 0.55042, 0.52505, 0.5, 0.47495, 0.44958, 0.42355, 0.39642, 0.36763, 0.33632, 0.30104, 0.25891, 0.20196, 0.
};

float HEATINVF(float powfact) {
    int idx = powfact * HEATINVLEN;
    float di = powfact * HEATINVLEN - idx;
    return HEATINV[idx] * (1 - di) + HEATINV[idx+1] * di;
}

class PotReader {
    public:
        Pin speed_pot;
        Pin temp_pot;

    uint32_t pot_read_tick(uint32_t dummy) {
        float speed_pot_val = 1 - Adc::instance->read(&(this->speed_pot))/((float)Adc::instance->get_max_value());
        int64_t speed = 0;
        if (speed_pot_val < 0.) {
            speed_pot_val = 0.;
        } else if (speed_pot_val >= MAX_R/(TOP_R + MAX_R)) {
            speed_pot_val = MAX_R/(TOP_R + MAX_R);
        }
        speed_pot_val = 1 - speed_pot_val; // [4.7/14.7, 1]
        speed_pot_val = (TOP_R/speed_pot_val - TOP_R)/MAX_R;
        speed = speed_pot_val * QVmax;
        StepTicker::getInstance()->pump_speed(speed);

        // forward is 0, backward is 1
        float heater_pot_val = 1 - Adc::instance->read(&(this->temp_pot))/((float)Adc::instance->get_max_value());
        int32_t heater = 0;
        if (heater_pot_val < 0.) {
            heater_pot_val = 0.;
        } else if (heater_pot_val >= MAX_R/(TOP_R + MAX_R)) {
            heater_pot_val = MAX_R/(TOP_R + MAX_R);
        }
        heater_pot_val = 1 - heater_pot_val;
        heater_pot_val = (TOP_R/heater_pot_val - TOP_R) / MAX_R;

        heater_pot_val = HEATINVF(speed_pot_val * heater_pot_val);

        heater = 8333 * heater_pot_val;

//        Heater::getInstance()->set_delay_us(heater);

        return 0;
    }

    void pot_read_init() {
        this->speed_pot.from_string("0.25");
        this->temp_pot.from_string("0.24");

        Adc::instance->enable_pin(&(this->speed_pot));
        Adc::instance->enable_pin(&(this->temp_pot));
        THEKERNEL->slow_ticker->attach(20, this, &PotReader::pot_read_tick);
    }
};

//PotReader PR;

volatile int zccnt = 0;

mbed::I2C* i2c;
//HT16K33* disps[8];
SevenSeg* numdisp;
//HT16K33* disp;
BarGraph* bardisp;

void init() {
    // Default pins to low status
    for (int i = 0; i < 5; i++){
        leds[i].output();
        leds[i]= 0;
    }

    Kernel* kernel = new Kernel();

    kernel->streams->printf("Smoothie Running @%ldMHz\r\n", SystemCoreClock / 1000000);
    Version version;
    kernel->streams->printf("  Build version %s, Build date %s\r\n", version.get_build(), version.get_build_date());
    kernel->streams->printf("  Nuli Pump Build\r\n");

    bool sdok= (sd.disk_initialize() == 0);
    if(!sdok) kernel->streams->printf("SDCard failed to initialize\r\n");

    #ifdef NONETWORK
        kernel->streams->printf("NETWORK is disabled\r\n");
    #endif

#ifdef DISABLEMSD
    // attempt to be able to disable msd in config
    if(sdok && !kernel->config->value( disable_msd_checksum )->by_default(true)->as_bool()){
        // HACK to zero the memory USBMSD uses as it and its objects seem to not initialize properly in the ctor
        size_t n= sizeof(USBMSD);
        void *v = AHB0.alloc(n);
        memset(v, 0, n); // clear the allocated memory
        msc= new(v) USBMSD(&u, &sd); // allocate object using zeroed memory
    }else{
        msc= NULL;
        kernel->streams->printf("MSD is disabled\r\n");
    }
#endif

    // Create and add main modules
//    kernel->add_module( new(AHB0) CurrentControl() );
//    kernel->add_module( new(AHB0) KillButton() );
//    kernel->add_module( new(AHB0) RotaryEncoder() );
    kernel->add_module( new(AHB0) Pump() );

    // these modules can be completely disabled in the Makefile by adding to EXCLUDE_MODULES
    #ifndef NONETWORK
    kernel->add_module( new Network() );
    #endif
    // Create and initialize USB stuff
    u.init();

#ifdef DISABLEMSD
    if(sdok && msc != NULL){
        kernel->add_module( msc );
    }
#else
    kernel->add_module( &msc );
#endif

    kernel->add_module( &usbserial );
    if( kernel->config->value( second_usb_serial_enable_checksum )->by_default(false)->as_bool() ){
        kernel->add_module( new(AHB0) USBSerial(&u) );
    }

    if( kernel->config->value( dfu_enable_checksum )->by_default(false)->as_bool() ){
        kernel->add_module( new(AHB0) DFU(&u));
    }

    // 10 second watchdog timeout (or config as seconds)
    float t= kernel->config->value( watchdog_timeout_checksum )->by_default(10.0F)->as_number();
    if(t > 0.1F) {
        // NOTE setting WDT_RESET with the current bootloader would leave it in DFU mode which would be suboptimal
        kernel->add_module( new Watchdog(t*1000000, WDT_MRI)); // WDT_RESET));
        kernel->streams->printf("Watchdog enabled for %f seconds\n", t);
    }else{
        kernel->streams->printf("WARNING Watchdog is disabled\n");
    }


    kernel->add_module( &u );

    // memory before cache is cleared
    //SimpleShell::print_mem(kernel->streams);

    // clear up the config cache to save some memory
    kernel->config->config_cache_clear();

    if(kernel->is_using_leds()) {
        // set some leds to indicate status... led0 init done, led1 mainloop running, led2 idle loop running, led3 sdcard ok
        leds[0]= 1; // indicate we are done with init
        leds[3]= sdok?1:0; // 4th led indicates sdcard is available (TODO maye should indicate config was found)
    }

    if(sdok) {
        // load config override file if present
        // NOTE only Mxxx commands that set values should be put in this file. The file is generated by M500
        FILE *fp= fopen(kernel->config_override_filename(), "r");
        if(fp != NULL) {
            char buf[132];
            kernel->streams->printf("Loading config override file: %s...\n", kernel->config_override_filename());
            while(fgets(buf, sizeof buf, fp) != NULL) {
                kernel->streams->printf("  %s", buf);
                if(buf[0] == ';') continue; // skip the comments
                struct SerialMessage message= {&(StreamOutput::NullStream), buf};
                kernel->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
            }
            kernel->streams->printf("config override file executed\n");
            fclose(fp);
        }
    }

//    PR.pot_read_init();
//    Heater* heater = new Heater();

    i2c = new mbed::I2C(P0_27, P0_28);
    /*
    for (char x=0; x<8; x++) {
        disps[x] = new HT16K33(i2c, x);
        disps[x]->init();
    }
    */

    numdisp = new SevenSeg(i2c, 2);
    numdisp->init();
    bardisp = new BarGraph(i2c, 0);
    bardisp->init();

    // start the timers and interrupts
    THEKERNEL->step_ticker->start();
    THEKERNEL->slow_ticker->start();

    /*
    for (int y=0; y<10000; y++) {
        for (char x=2; x<6; x+=8) {
            disps[x]->flush_int();
        }
        sprintf(buf, "%04d", y);
        for (char x=2; x<6; x+=8) {
            disps[x]->setmem(0, numbertable[buf[0]-'0']); //
            disps[x]->setmem(2, numbertable[buf[1]-'0']); //
            disps[x]->setmem(6, numbertable[buf[2]-'0']); //
            disps[x]->setmem(8, numbertable[buf[3]-'0']); // N
            disps[x]->repaint();
        }
        for (char x=0; x<8; x++) {
            disps[x]->setmem(0, 0x00); //
            disps[x]->setmem(2, 0x00); //
            disps[x]->setmem(6, 0x00); //
            disps[x]->setmem(8, 0x37); // N
            disps[x]->repaint();
        }
        wait(0.1);
        for (char x=0; x<8; x++) {
            disps[x]->setmem(0, 0x00); //
            disps[x]->setmem(2, 0x00); //
            disps[x]->setmem(6, 0x37); // N
            disps[x]->setmem(8, 0x3E); // U
            disps[x]->repaint();
        }
        wait(0.1);
        for (char x=0; x<8; x++) {
            disps[x]->setmem(0, 0x00); //
            disps[x]->setmem(2, 0x37); // N
            disps[x]->setmem(6, 0x3E); // U
            disps[x]->setmem(8, 0x38); // L
            disps[x]->repaint();
        }
        wait(0.1);
        for (char x=0; x<8; x++) {
            disps[x]->setmem(0, 0x37); // N
            disps[x]->setmem(2, 0x3E); // U
            disps[x]->setmem(6, 0x38); // L
            disps[x]->setmem(8, 0x30); // I
            disps[x]->repaint();
        }
        wait(0.3);
        for (char x=0; x<8; x++) {
            disps[x]->setmem(0, 0x3E); // U
            disps[x]->setmem(2, 0x38); // L
            disps[x]->setmem(6, 0x30); // I
            disps[x]->setmem(8, 0x00); //
            disps[x]->repaint();
        }
        wait(0.1);
        for (char x=0; x<8; x++) {
            disps[x]->setmem(0, 0x38); // L
            disps[x]->setmem(2, 0x30); // I
            disps[x]->setmem(6, 0x00); //
            disps[x]->setmem(8, 0x00); //
            disps[x]->repaint();
        }
        wait(0.1);
        for (char x=0; x<8; x++) {
            disps[x]->setmem(0, 0x30); // I
            disps[x]->setmem(2, 0x00); //
            disps[x]->setmem(6, 0x00); //
            disps[x]->setmem(8, 0x00); //
            disps[x]->repaint();
        }
        wait(0.1);
        for (char x=0; x<8; x++) {
            disps[x]->setmem(0, 0x00); //
            disps[x]->setmem(2, 0x00); //
            disps[x]->setmem(6, 0x00); //
            disps[x]->setmem(8, 0x00); //
            disps[x]->repaint();
        }
    }
    */
}

int main() {
    init();

    uint16_t cnt= 0;
    uint32_t lcnt= 0;

    // Main loop
    while(1) {
        if(THEKERNEL->is_using_leds()) {
            // flash led 2 to show we are alive
            leds[1]= (cnt++ & 0x1000) ? 1 : 0;
        }

        if (cnt % 50000 == 0) {
            lcnt += 1; lcnt %= (51 * 3);
            // first four bits of 0th byte are the
//            disp->setmem(0, lcnt & 0xFF);         // bottom of first (red)
//            disp->setmem(2, lcnt & 0xFF);         // bottom of first (red)
//            disp->setmem(4, lcnt & 0xFF);         // bottom of first (red)
//            disp->setmem(1, (lcnt >> 8) & 0xFF);  // bottom of second (red)
//            disp->setmem(3, (lcnt >> 8) & 0xFF); //           green
//            disp->setmem(5, (lcnt >> 8) & 0xFF); //           green

            numdisp->print("    NULI    " + (lcnt % 9));
            numdisp->repaint();

            if (lcnt <= 25)
                bardisp->set_val(lcnt);
            else
                bardisp->set_val(51 - lcnt);
            bardisp->repaint();

//            disp->print((ledcnt += 100)*0.01F);
        }

        THEKERNEL->call_event(ON_MAIN_LOOP);
        THEKERNEL->call_event(ON_IDLE);
    }
}
