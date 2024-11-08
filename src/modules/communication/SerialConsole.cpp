/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/
#include <string>
#include <stdarg.h>
using std::string;
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "SerialConsole.h"
#include "libs/RingBuffer.h"
#include "libs/SerialMessage.h"
#include "libs/StreamOutput.h"
#include "libs/StreamOutputPool.h"
#include "libs/gpio.h"
#include "port_api.h"
#include "mri.h"

// Serial reading module
// Treats every received line as a command and passes it ( via event call ) to the command dispatcher.
// The command dispatcher will then ask other modules if they can do something with it
SerialConsole::SerialConsole( PinName rx_pin, PinName tx_pin, PinName rts_pin, PinName cts_pin, int baud_rate ){
    this->serial = new mbed::Serial( rx_pin, tx_pin, rts_pin, cts_pin );
    this->serial->baud(baud_rate);
    
    if ( rts_pin != NC ) {
        this->rts_signal = new GPIO(rts_pin) ;
        this->rts_signal->clear();
        flow_control = true;
    } else
        flow_control = false;

    rts_signal_is_set = false;
}

// Called when the module has just been loaded
void SerialConsole::on_module_loaded() {
    this->serial->attach(this, &SerialConsole::on_line_idle, mbed::Serial::RxIdleIrq);
    this->serial->attach(this, &SerialConsole::on_buffer_half_full, mbed::Serial::DmaHFIrq);
    this->serial->attach(this, &SerialConsole::on_buffer_full, mbed::Serial::DmaTCIrq);
    this->serial->dma_init( rx_buff, RX_BUFF_SIZE );

    // We only call the command dispatcher in the main loop, nowhere else
    this->register_for_event(ON_MAIN_LOOP);

    // Add to the pack of streams kernel can call to, for example for broadcasting
    THEKERNEL->streams->append_stream(this);
}


// This is running in interrupt context
void SerialConsole::manage_buffer( void )
{
    int free_space;
    int remain_until_buf_top = this->serial->get_dma_buffer_index();
    unsigned char *buf_start = rx_buff;
    unsigned char *buf_end = buf_start + RX_BUFF_SIZE-1;
    unsigned char *head = 1+buf_end - remain_until_buf_top;

    if (head >= tail)
        free_space = (remain_until_buf_top-1) + (tail-buf_start);
    else
        free_space = tail-head;

    if (free_space <= RX_BUFF_SIZE/2 ) { // Ask other end to relax a bit while we deal with what we already got!
        if (flow_control) {
            this->rts_signal->set();
            rts_signal_is_set = true;
        }
    }
}

void SerialConsole::on_main_loop(void * argument){
    int msglen = 0; 
    bool got_msg = false;
    unsigned char *p, *buf_start = rx_buff, *buf_end = buf_start + RX_BUFF_SIZE-1;
    int remain_until_buf_top = this->serial->get_dma_buffer_index();
    unsigned char *head = 1+buf_end - remain_until_buf_top;

    
    // Find if there's a message for us
    p = tail;
    
    do {
        if( p == head )
            break;
        if ( *p == '\r' )
        	*p = '\n';
        if ( *p == '\n' ) {
            got_msg = true;
            break;
        }

        msglen++;
        p++;
        if( p > buf_end )
            p = buf_start;
    } while ( !got_msg );

    if ( got_msg ) {
        string received;
        struct SerialMessage message;

        if ( tail > &rx_buff[0] && *(tail-1) != '\n')
            __debugbreak();

        if ( p > tail ) {
            received.append( (char*) tail, msglen); 
        } else {
            int len;
            len = 1+buf_end - tail;
            received.append( (char*)tail, len); 
            
            len = p-buf_start;
            received.append( (char*)buf_start, len); 
        }

        message.message = received;
        message.stream = this;
        THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );

        // update tail for next time
        p++;
        if( p > buf_end )
            p = buf_start;
        tail = p; 

        if (flow_control && rts_signal_is_set) {
            int free_space;

            // Get updated head, there maybe much more that have come in behind our back,
            // since the serial/dma interrupt levels are the lowest prio.
            remain_until_buf_top = this->serial->get_dma_buffer_index();
            head = 1+buf_end - remain_until_buf_top;

            if (head >= tail)
                free_space = (remain_until_buf_top-1) + (tail-buf_start);
            else
                free_space = tail-head;

            if (free_space > RX_BUFF_SIZE/2 ) {
                rts_signal_is_set = false;
                this->rts_signal->clear();
            }
        }
    }
}


int SerialConsole::puts(const char* s){
    this->serial->send_string(s);
    return 0;
}
