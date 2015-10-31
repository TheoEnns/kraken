/*
  Commander.h - Library for interfacing with arbotiX Commander
  Copyright (c) 2009-2010 Michael E. Ferguson.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef Commander_h
#define Commander_h

/* bitmasks for buttons array */
#define BUT_R1      0x01
#define BUT_R2      0x02
#define BUT_R3      0x04
#define BUT_L4      0x08
#define BUT_L5      0x10
#define BUT_L6      0x20
#define BUT_RT      0x40
#define BUT_LT      0x80

/* the Commander will send out a frame at about 30hz, this class helps decipher the output. */
class Commander
{    
  public:
	Commander(); 
    void begin(int baud);
    void UseSouthPaw();     // enable southpaw configuration
    int ReadMsgs();         // must be called regularly to clean out Serial buffer

    // joystick values are -125 to 125
    signed char walkV;      // vertical stick movement = forward speed
    signed char walkH;      // horizontal stick movement = sideways or angular speed
    signed char lookV;      // vertical stick movement = tilt    
    signed char lookH;      // horizontal stick movement = pan (when we run out of pan, turn body?)
    // 0-1023, use in extended mode    
    int pan;
    int tilt;
    
    // buttons are 0 or 1 (PRESSED), and bitmapped
    unsigned char buttons;  // 
    unsigned char ext;      // Extended function set
        
    // Hooks are used as callbacks for button presses -- NOT IMPLEMENT YET
        
  private:
    // internal variables used for reading messages
    unsigned char vals[7];  // temporary values, moved after we confirm checksum
    int index;              // -1 = waiting for new packet
    int checksum;
    unsigned char status; 
};

#define serial_Commander       Serial3

/* Constructor */
Commander::Commander(){
    index = -1;
    status = 0;
}

void Commander::begin(int baud){
    serial_Commander.begin(baud);
}

/* SouthPaw Support */
void Commander::UseSouthPaw(){
    status |= 0x01;
}

/* process messages coming from Commander 
 *  format = 0xFF RIGHT_H RIGHT_V LEFT_H LEFT_V BUTTONS EXT CHECKSUM */
int Commander::ReadMsgs(){
    while(serial_Commander.available() > 0){
        if(index == -1){         // looking for new packet
            if(serial_Commander.read() == 0xff){
                index = 0;
                checksum = 0;
            }
        }else if(index == 0){
            vals[index] = (unsigned char) serial_Commander.read();
            if(vals[index] != 0xff){            
                checksum += (int) vals[index];
                index++;
            }
        }else{
            vals[index] = (unsigned char) serial_Commander.read();
            checksum += (int) vals[index];
            index++;
            if(index == 7){ // packet complete
                if(checksum%256 != 255){
                    // packet error!
                    index = -1;
                    return 0;
                }else{
                    if((status&0x01) > 0){     // SouthPaw
                        walkV = (signed char)( (int)vals[0]-128 );
                        walkH = (signed char)( (int)vals[1]-128 );
                        lookV = (signed char)( (int)vals[2]-128 );
                        lookH = (signed char)( (int)vals[3]-128 );
                    }else{
                        lookV = (signed char)( (int)vals[0]-128 );
                        lookH = (signed char)( (int)vals[1]-128 );
                        walkV = (signed char)( (int)vals[2]-128 );
                        walkH = (signed char)( (int)vals[3]-128 );
                    }
                    pan = (vals[0]<<8) + vals[1];
                    tilt = (vals[2]<<8) + vals[3];
                    buttons = vals[4];
                    ext = vals[5];
                }
                index = -1;
                serial_Commander.flush();
                return 1;
            }
        }
    }
    return 0;
}

#endif

