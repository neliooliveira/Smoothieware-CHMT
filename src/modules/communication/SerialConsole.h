/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef SERIALCONSOLE_H
#define SERIALCONSOLE_H

#include "libs/Module.h"
#include "Serial.h" // mbed.h lib
#include "libs/Kernel.h"
#include <vector>
#include <string>
using std::string;
#include "libs/RingBuffer.h"
#include "libs/StreamOutput.h"
#include "libs/gpio.h"


#define baud_rate_setting_checksum CHECKSUM("baud_rate")

#define RX_BUFF_SIZE 512

class SerialConsole : public Module, public StreamOutput {
    public:
        SerialConsole( PinName rx_pin, PinName tx_pin, PinName rts_pin, PinName cts_pin, int baud_rate );

        void on_module_loaded();
        void on_main_loop(void * argument);

        void on_line_idle( void ) { manage_buffer(); }
        void on_buffer_half_full(void) { manage_buffer(); }
        void on_buffer_full(void) { manage_buffer(); }

        int puts(const char*);
        mbed::Serial* serial;
        GPIO *rts_signal;

        unsigned char rx_buff[RX_BUFF_SIZE];
        unsigned char *tail = rx_buff;
        
    private:
        void manage_buffer(void);
        bool rts_signal_is_set;
        bool flow_control;
};

#endif
