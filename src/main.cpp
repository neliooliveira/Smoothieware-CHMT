/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Kernel.h"

#include "modules/tools/laser/Laser.h"
#include "modules/tools/spindle/SpindleMaker.h"
#include "modules/tools/extruder/ExtruderMaker.h"
#include "modules/tools/temperaturecontrol/TemperatureControlPool.h"
#include "modules/tools/endstops/Endstops.h"
#include "modules/tools/zprobe/ZProbe.h"
#include "modules/tools/scaracal/SCARAcal.h"
#include "RotaryDeltaCalibration.h"
#include "modules/tools/switch/SwitchPool.h"
#include "modules/tools/temperatureswitch/TemperatureSwitch.h"
#include "modules/tools/drillingcycles/Drillingcycles.h"
#include "FilamentDetector.h"
#include "MotorDriverControl.h"
#include "modules/tools/flybyvision/FlyByVision.h"

#include "modules/robot/Conveyor.h"
#include "modules/utils/simpleshell/SimpleShell.h"
#include "modules/utils/configurator/Configurator.h"
#include "modules/utils/currentcontrol/CurrentControl.h"
#include "modules/utils/player/Player.h"
#include "modules/utils/killbutton/KillButton.h"
#include "modules/utils/encoder/Encoder.h"
#include "modules/utils/PlayLed/PlayLed.h"
#include "modules/utils/panel/Panel.h"
#include "Config.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "StepTicker.h"
#include "SlowTicker.h"
#include "Robot.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "libs/SerialMessage.h"
#include "StreamOutputPool.h"
#include "ToolManager.h"
#include "libs/Watchdog.h"
#include "libs/gpio.h"
#include "version.h"
#include "platform_memory.h"
#include "mbed.h"

#define second_usb_serial_enable_checksum  CHECKSUM("second_usb_serial_enable")
#define disable_msd_checksum  CHECKSUM("msd_disable")
#define dfu_enable_checksum  CHECKSUM("dfu_enable")
#define watchdog_timeout_checksum  CHECKSUM("watchdog_timeout")

#ifndef DISABLEMSD
#else
#endif

GPIO leds[] = { GPIO(PE_12), GPIO(PE_13) };

void init() {
    for (uint8_t i = 0; i < sizeof(leds)/sizeof(leds[0]); i++) { leds[i].output(); leds[i]= 0; }
    Kernel* kernel = new Kernel();

    kernel->add_module( new(AHB0) Player() );
    kernel->add_module( new(AHB0) CurrentControl() );
    kernel->add_module( new(AHB0) KillButton() );
    kernel->add_module( new(AHB0) Encoder() );
    kernel->add_module( new(AHB0) PlayLed() );
    kernel->add_module( new(AHB0) FlyByVision() );

    #ifndef NO_TOOLS_SWITCH
    SwitchPool *sp= new SwitchPool(); sp->load_tools(); delete sp;
    #endif
    #ifndef NO_TOOLS_EXTRUDER
    ExtruderMaker *em= new ExtruderMaker(); em->load_tools(); delete em;
    #endif
    #ifndef NO_TOOLS_TEMPERATURECONTROL
    TemperatureControlPool *tp= new TemperatureControlPool(); tp->load_tools(); delete tp;
    #endif
    #ifndef NO_TOOLS_ENDSTOPS
    kernel->add_module( new(AHB0) Endstops() );
    #endif
    #ifndef NO_TOOLS_LASER
    kernel->add_module( new Laser() );
    #endif
    #ifndef NO_TOOLS_SPINDLE
    SpindleMaker *sm= new SpindleMaker(); sm->load_spindle(); delete sm;
    #endif
    #ifndef NO_UTILS_PANEL
    kernel->add_module( new(AHB0) Panel() );
    #endif
    #ifndef NO_TOOLS_ZPROBE
    kernel->add_module( new(AHB0) ZProbe() );
    #endif
    #ifndef NO_TOOLS_SCARACAL
    kernel->add_module( new(AHB0) SCARAcal() );
    #endif
    #ifndef NO_TOOLS_ROTARYDELTACALIBRATION
    kernel->add_module( new(AHB0) RotaryDeltaCalibration() );
    #endif
    #ifndef NO_TOOLS_TEMPERATURESWITCH
    kernel->add_module( new(AHB0) TemperatureSwitch() );
    #endif
    #ifndef NO_TOOLS_DRILLINGCYCLES
    kernel->add_module( new(AHB0) Drillingcycles() );
    #endif
    #ifndef NO_TOOLS_FILAMENTDETECTOR
    kernel->add_module( new(AHB0) FilamentDetector() );
    #endif
    #ifndef NO_UTILS_MOTORDRIVERCONTROL
    kernel->add_module( new MotorDriverControl(0) );
    #endif

    float t= kernel->config->value( watchdog_timeout_checksum )->by_default(10.0F)->as_number();
    if(t > 0.1F) kernel->add_module( new Watchdog(t*1000000, WDT_MRI));
    else kernel->streams->printf("WARNING Watchdog is disabled\n");

    kernel->config->config_cache_clear();
    if(kernel->is_using_leds()) leds[0]= 1;

    THEKERNEL->conveyor->start(THEROBOT->get_number_registered_motors());
    THEKERNEL->step_ticker->start();
    THEKERNEL->slow_ticker->start();
}

int main()
{
    init();
    uint16_t cnt= 0;
    while(1) {
        if(THEKERNEL->is_using_leds()) leds[0]= (cnt++ & 0x1000) ? 1 : 0;
        THEKERNEL->call_event(ON_MAIN_LOOP);
        THEKERNEL->call_event(ON_IDLE);
    }
}
