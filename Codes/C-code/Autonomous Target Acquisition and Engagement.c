/********************************************************************************
 Written by: Sachitanand Malewar, NEX Robotics Pvt. Ltd.
 AVR Studio Version 4.17, Build 666

 Date: 13th January 2010
 
 Application example: Robot control over serial port

 Concepts covered:  serial communication

 Serial Port used: UART1

 There are two components to the motion control:
 1. Direction control using pins PORTA0 to PORTA3
 2. Velocity control by PWM on pins PL3 and PL4 using OC5A and OC5B.

 In this experiment for the simplicity PL3 and PL4 are kept at logic 1.
 
 Pins for PWM are kept at logic 1.
 
 Connection Details:     
                        
  Motion control:        L-1---->PA0;        L-2---->PA1;
                           R-1---->PA2;        R-2---->PA3;
                           PL3 (OC5A) ----> Logic 1;     PL4 (OC5B) ----> Logic 1;


  Serial Communication:    PORTD 2 --> RXD1 UART1 receive for RS232 serial communication
                        PORTD 3 --> TXD1 UART1 transmit for RS232 serial communication

                        PORTH 0 --> RXD2 UART 2 receive for USB - RS232 communication
                        PORTH 1 --> TXD2 UART 2 transmit for USB - RS232 communication

                        PORTE 0 --> RXD0 UART0 receive for ZigBee wireless communication
                        PORTE 1 --> TXD0 UART0 transmit for ZigBee wireless communication

                        PORTJ 0 --> RXD3 UART3 receive available on microcontroller expainsion board
                        PORTJ 1 --> TXD3 UART3 transmit available on microcontroller expainsion board

Serial communication baud rate: 9600bps
To control robot use number pad of the keyboard which is located on the right hand side of the keyboard.
Make sure that NUM lock is on.

Commands:
            Keyboard Key    HEX value    Action
                8                0x38    Forward
                2                0x32    Backward
                4                0x34    Left
                6                0x36    Right
                5                0x35    Stop
                7                0x37    Buzzer on
                9                0x39    Buzzer off

 Note:
 
 1. Make sure that in the configuration options following settings are
     done for proper operation of the code

     Microcontroller: atmega2560
     Frequency: 11059200
     Optimization: -O0 (For more information read section: Selecting proper optimization options
                        below figure 4.22 in the hardware manual)

 2. Difference between the codes for RS232 serial, USB and wireless communication is only in the serial port number.
     Rest of the things are the same.

 3. For USB communication check the Jumper 1 position on the ATMEGA2560 microcontroller adaptor board

*********************************************************************************/

/********************************************************************************

   Copyright (c) 2010, NEX Robotics Pvt. Ltd.                       -*- c -*-
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

   * Source code can be used for academic purpose.
     For commercial use permission form the author needs to be taken.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.

  Software released under Creative Commence cc by-nc-sa licence.
  For legal information refer to:
  http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode

********************************************************************************/


#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>
#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>
//!To store received data from UDR1
unsigned char data; 
//!To maintain the states of the servo moters
int PositionPanServo_1;
//!To maintain the states of the servo moters
int PositionPanServo_2;
//!To maintain the states of the servo moters
int PositionPanServo_3;
//!defined here to make sure that program works properly
#define FCPU 11059200ul 
//!Command input channel in LCD
#define RS 0
//!Write channel in LCD
#define RW 1
//!Enable in LCD
#define EN 2
//!Defining the LCD port
#define lcd_port PORTC
//! Macro defined for Setting a bit of any register.
#define sbit(reg,bit)	reg |= (1<<bit)		
//! Macro defined for Clearing a bit of any register.	
#define cbit(reg,bit)	reg &= ~(1<<bit)		
//!Upper limit for rotation of servo motor
#define upper_limit 60    
//!Lower limit for rotation of servo motor                     
#define lower_limit 0   
//!Rotation angle of the servo motors                       
#define rotation_angle  5  
//!The time to run the bot to left before stopping during both scanning and centering
#define left_right_delay 80                            
//!Initializing the ports of LCD,motors,servo motors,buzzer               
void init_ports();
//!For LCD reset ie clear screen,curser at initial position
void lcd_reset();
//!LCD initialization used in init ports
void lcd_init();
//!Function for writing a command into the lcd screen.
void lcd_wr_command(unsigned char);
//!Function for writing into the lcd screen charactar at a time
void lcd_wr_char(char);
//!Function for writing into the lcd screen in the first line
void lcd_line1();
//!Function for writing into the lcd screen in the second line
void lcd_line2();
//!Function for writing into the lcd screen in the from the current location
void lcd_string(char*);

//!temporary variable
unsigned int temp;
//!temporary variable
unsigned int unit;
//!temporary variable
unsigned int tens;
//!temporary variable
unsigned int hundred;
//!temporary variable
unsigned int thousand;
//!temporary variable
unsigned int million;

//!Function to configure LCD port
void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}
//!Function to set motion in the dc motors
void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F; 			// removing upper nibbel as it is not needed
 PortARestore = PORTA; 			// reading the PORTA's original status
 PortARestore &= 0xF0; 			// setting lower direction nibbel to 0
 PortARestore |= Direction; 	// adding lower nibbel for direction command and restoring the PORTA status
 PORTA = PortARestore; 			// setting the command to the port
}
 //!Function to move the bot forward
void forward (void) //both wheels forward
{
  motion_set(0x06);
}
 //!Function to move the bot Left
void left (void) //Left wheel backward, Right wheel forward
{
  motion_set(0x05);
}
 //!Function to move the bot backward
void back (void) //both wheels backward
{
  motion_set(0x09);
}

 //!Function to move the bot right
void right (void) //Left wheel forward, Right wheel backward
{
  motion_set(0x0A);
}
 //!Function to stop the bot
void stop (void)
{
  motion_set(0x00);
}
//!Initializing the motion pins in the bot 
void motion_pin_config (void)
{
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

//!Function to Reset LCD
void lcd_set_4bit()
{
	_delay_ms(1);

	cbit(lcd_port,RS);				//RS=0 --- Command Input
	cbit(lcd_port,RW);				//RW=0 --- Writing to LCD
	lcd_port = 0x30;				//Sending 3
	sbit(lcd_port,EN);				//Set Enable Pin
	_delay_ms(5);					//Delay
	cbit(lcd_port,EN);				//Clear Enable Pin

	_delay_ms(1);

	cbit(lcd_port,RS);				//RS=0 --- Command Input
	cbit(lcd_port,RW);				//RW=0 --- Writing to LCD
	lcd_port = 0x30;				//Sending 3
	sbit(lcd_port,EN);				//Set Enable Pin
	_delay_ms(5);					//Delay
	cbit(lcd_port,EN);				//Clear Enable Pin

	_delay_ms(1);

	cbit(lcd_port,RS);				//RS=0 --- Command Input
	cbit(lcd_port,RW);				//RW=0 --- Writing to LCD
	lcd_port = 0x30;				//Sending 3
	sbit(lcd_port,EN);				//Set Enable Pin
	_delay_ms(5);					//Delay
	cbit(lcd_port,EN);				//Clear Enable Pin

	_delay_ms(1);

	cbit(lcd_port,RS);				//RS=0 --- Command Input
	cbit(lcd_port,RW);				//RW=0 --- Writing to LCD
	lcd_port = 0x20;				//Sending 2 to initialise LCD 4-bit mode
	sbit(lcd_port,EN);				//Set Enable Pin
	_delay_ms(5);					//Delay
	cbit(lcd_port,EN);				//Clear Enable Pin

	
}

//!Function to Initialize LCD
void lcd_init()
{
	_delay_ms(1);

	lcd_wr_command(0x28);			//LCD 4-bit mode and 2 lines.
	lcd_wr_command(0x01);
	lcd_wr_command(0x06);
	lcd_wr_command(0x0E);
	lcd_wr_command(0x80);
		
}

	 
//!Function to Write Command on LCD
void lcd_wr_command(unsigned char cmd)
{
	unsigned char temp;
	temp = cmd;
	temp = temp & 0xF0;
	lcd_port &= 0x0F;
	lcd_port |= temp;
	cbit(lcd_port,RS);
	cbit(lcd_port,RW);
	sbit(lcd_port,EN);
	_delay_ms(5);
	cbit(lcd_port,EN);
	
	cmd = cmd & 0x0F;
	cmd = cmd<<4;
	lcd_port &= 0x0F;
	lcd_port |= cmd;
	cbit(lcd_port,RS);
	cbit(lcd_port,RW);
	sbit(lcd_port,EN);
	_delay_ms(5);
	cbit(lcd_port,EN);
}

//!Function to Write Data on LCD
void lcd_wr_char(char letter)
{
	char temp;
	temp = letter;
	temp = (temp & 0xF0);
	lcd_port &= 0x0F;
	lcd_port |= temp;
	sbit(lcd_port,RS);
	cbit(lcd_port,RW);
	sbit(lcd_port,EN);
	_delay_ms(5);
	cbit(lcd_port,EN);

	letter = letter & 0x0F;
	letter = letter<<4;
	lcd_port &= 0x0F;
	lcd_port |= letter;
	sbit(lcd_port,RS);
	cbit(lcd_port,RW);
	sbit(lcd_port,EN);
	_delay_ms(5);
	cbit(lcd_port,EN);
}


//!Function to bring cursor at home position
void lcd_home()
{
	lcd_wr_command(0x80);
}


//!Function to Print String on LCD
void lcd_string(char *str)
{
	while(*str != '\0')
	{
		lcd_wr_char(*str);
		str++;
	}
}

//!Position the LCD cursor at "row", "column".

void lcd_cursor (char row, char column)
{
	switch (row) {
		case 1: lcd_wr_command (0x80 + column - 1); break;
		case 2: lcd_wr_command (0xc0 + column - 1); break;
		case 3: lcd_wr_command (0x94 + column - 1); break;
		case 4: lcd_wr_command (0xd4 + column - 1); break;
		default: break;
	}
}

//!Function To Print Any input value upto the desired digit on LCD
void lcd_print (char row, char coloumn, unsigned int value, int digits)
{
	unsigned char flag=0;
	if(row==0||coloumn==0)
	{
		lcd_home();
	}
	else
	{
		lcd_cursor(row,coloumn);
	}
	if(digits==5 || flag==1)
	{
		million=value/10000+48;
		lcd_wr_char(million);
		flag=1;
	}
	if(digits==4 || flag==1)
	{
		temp = value/1000;
		thousand = temp%10 + 48;
		lcd_wr_char(thousand);
		flag=1;
	}
	if(digits==3 || flag==1)
	{
		temp = value/100;
		hundred = temp%10 + 48;
		lcd_wr_char(hundred);
		flag=1;
	}
	if(digits==2 || flag==1)
	{
		temp = value/10;
		tens = temp%10 + 48;
		lcd_wr_char(tens);
		flag=1;
	}
	if(digits==1 || flag==1)
	{
		unit = value%10 + 48;
		lcd_wr_char(unit);
	}
	if(digits>5)
	{
		lcd_wr_char('E');
	}
	
}



//!Function to configure PORTB 5 pin for servo motor 1 operation
void servo1_pin_config (void)
{
 DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
 PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}

//!Function to configure PORTB 6 pin for servo motor 2 operation
void servo2_pin_config (void)
{
 DDRB  = DDRB | 0x40;  //making PORTB 6 pin output
 PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
}

//!Function to configure PORTB 7 pin for servo motor 3 operation
void servo3_pin_config (void)
{
 DDRB  = DDRB | 0x80;  //making PORTB 7 pin output
 PORTB = PORTB | 0x80; //setting PORTB 7 pin to logic 1
}


//!TIMER1 initialization in 10 bit fast PWM mode 
//!prescale:256
//!WGM: 7) PWM 10bit fast, TOP=0x03FF
//!actual value: 42.187Hz
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01;    //Counter low value to which OCR1xH value is to be compared with
 OCR1AH = 0x03;    //Output compare eegister high value for servo 1
 OCR1AL = 0xFF;    //Output Compare Register low Value For servo 1
 OCR1BH = 0x03;    //Output compare eegister high value for servo 2
 OCR1BL = 0xFF;    //Output Compare Register low Value For servo 2
 OCR1CH = 0x03;    ///Output compare eegister high value for servo 3
 OCR1CL = 0xFF;    //Output Compare Register low Value For servo 3
 ICR1H  = 0x03;   
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
                     For Overriding normal port functionalit to OCRnA outputs.
                  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}

//!Function to rotate Servo 1 by a specified angle in the multiples of 2.25 degrees
void servo_1(unsigned char degrees) 
{
if(degrees<lower_limit ){PositionPanServo_1 +=10;return;}
if(degrees>upper_limit ){PositionPanServo_1 -=10;return;}
float PositionPanServo = ((float)degrees / 2.25)+21;
 OCR1AH = 0x00;
 OCR1AL = (unsigned char) PositionPanServo;
}

//!Function to rotate Servo 2 by a specified angle in the multiples of 2.25 degrees
void servo_2(unsigned char degrees)
{
float PositionPanServo = ((float)degrees / 2.25)+21;
 OCR1BH = 0x00;
 OCR1BL = (unsigned char) PositionPanServo;
}

//!Function to rotate Servo 3 by a specified angle in the multiples of 2.25 degrees
void servo_3(unsigned char degrees)
{
 float PositionPanServo = ((float)degrees / 2.25)+21;
 OCR1CH = 0x00;
 OCR1CL = (unsigned char) PositionPanServo;
}

//!servo_1_free functions unlocks the servo motors from the any angle and make them free by giving 100% duty cycle at the PWM. This function can be used to reduce the power consumption of the motor if it is holding load against the gravity.
void servo_1_free (void) //makes servo 1 free rotating
{
 OCR1AH = 0x03;
 OCR1AL = 0xFF; //Servo 1 off
}
//!servo_2_free functions unlocks the servo motors from the any angle and make them free by giving 100% duty cycle at the PWM. This function can be used to reduce the power consumption of the motor if it is holding load against the gravity.
void servo_2_free (void) //makes servo 2 free rotating
{
 OCR1BH = 0x03;
 OCR1BL = 0xFF; //Servo 2 off
}
//!servo_3_free functions unlocks the servo motors from the any angle and make them free by giving 100% duty cycle at the PWM. This function can be used to reduce the power consumption of the motor if it is holding load against the gravity.
void servo_3_free (void) //makes servo 3 free rotating
{
 OCR1CH = 0x03;
 OCR1CL = 0xFF; //Servo 3 off
}

//!Initializing the buzzer ports
void buzzer_pin_config (void)
{
 DDRC = DDRC | 0x08;        //Setting PORTC 3 as outpt
 PORTC = PORTC & 0xF7;        //Setting PORTC 3 logic low to turnoff buzzer
}


//!Function to initialize ports
void port_init()
{
    servo1_pin_config(); //Configure PORTB 5 pin for servo motor 1 operation
     servo2_pin_config(); //Configure PORTB 6 pin for servo motor 2 operation
     servo3_pin_config(); //Configure PORTB 7 pin for servo motor 3 operation 
    motion_pin_config();
    buzzer_pin_config();
	lcd_port_config();
}
//!Function to turn the buzzer on
void buzzer_on (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore | 0x08;
 PORTC = port_restore;
}
//!Function to turn off the buzzer.
void buzzer_off (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore & 0xF7;
 PORTC = port_restore;
}

//!Function To Initialize UART1.desired baud rate:9600.actual baud rate:9600 (error 0.0%).char size: 8 bit.parity: Disabled
void uart1_init(void)
{
 UCSR1B = 0x00; //disable while setting baud rate
 UCSR1A = 0x00;
 UCSR1C = 0x06;
 UBRR1L = 0x47; //set baud rate lo
 UBRR1H = 0x00; //set baud rate hi
 UCSR1B = 0x98;
}

/**
 \brief
Function called upon recieving a character on the serial port ie UART1.This Function analyses the recieved character and takes appropriate action.
<table>
<tr>
<td>ASCII-Code </td> 						<td>Action Taken</td>
</tr>
<tr>
<td>1							  <td>Move the bat back for 500ms
</tr>
<tr>
<td>0x30						  <td>Moves both the servo Motors to upper_limit
</tr>
<tr>
<td>0x31-0x39  					  <td>Moves camera according to the nine squares method using the servo motor 2 and moving the bot left or right.Here the rotation is of  
									  rotation_angle and in the case of Movement of the bot for the time left_right_delay
</tr>
<tr>
<td>0x35						  <td>Beeps the buzzer and fires the laser for a short amount of time which is in servo_3.
</tr>
<tr>
<td>>0x39						  <td>This moves the servo moter 2 to an angle =(ascii_code-0x40)*rotation_angle  
</tr>
</table>
*/
SIGNAL(SIG_USART1_RECV)         // ISR for receive complete interrupt
{
    //buzzer_on();

    data = UDR1;                 //making copy of data from UDR1 in 'data' variable

    UDR1 = data;                 //echo data back to PC
		if(data==0x01){
			back();
			_delay_ms(500);
			stop();
		}
		if(data == 0x30)//ASCII value of 0
		{
			//this puts the values of both positions to upper_limit this is required as the other functions are relative to position before
			PositionPanServo_1=upper_limit;
			PositionPanServo_2=upper_limit;
			servo_1(PositionPanServo_1);
			_delay_ms(60);
            servo_2(PositionPanServo_2);
			_delay_ms(60);
		}
        if(data == 0x31) //ASCII value of 1
        {
			left();
			_delay_ms(left_right_delay);
			stop();
			//decrease the camara angle of both the planes
			/*PositionPanServo_1 -=10;
			servo_1(PositionPanServo_1);
			_delay_ms(60);*/
			PositionPanServo_2 -=rotation_angle;
			servo_2(PositionPanServo_2);

            //if(PositionPanServo_1<=upper_limit)servo_1(PositionPanServo_1);
			//else{PositionPanServo_1 -=10;}
            _delay_ms(60);
		
        }

        if(data == 0x32) //ASCII value of 2
        {
			//this is to decrease the  camera angle in the horizantal plane
			left();
			_delay_ms(left_right_delay);
			stop();
			/*PositionPanServo_1 -=10;
			servo_1(PositionPanServo_1);
			_delay_ms(60);*/
        }

        if(data == 0x33) //ASCII value of 3
        {
			//this is to increase the  camera angle in the vertical plane and decrease it in horizantal plane
			left();
			_delay_ms(left_right_delay);
			stop();
			/*PositionPanServo_1 -=rotation_angle;
			servo_1(PositionPanServo_1);
			PositionPanServo_2 +=rotation_angle;
			_delay_ms(60);*/
			servo_2(PositionPanServo_2);
            //if(PositionPanServo_1<=upper_limit)servo_1(PositionPanServo_1);
			//else{PositionPanServo_1 -=10;}
            _delay_ms(60);

        }

        if(data == 0x34) //ASCII value of 4
        {
			//This is to decrease the camara angle in vertical plane
           	PositionPanServo_2 -=rotation_angle;
			servo_2(PositionPanServo_2);
            //if(PositionPanServo_1<=upper_limit)servo_1(PositionPanServo_1);
			//else{PositionPanServo_1 -=rotation_angle;}
            _delay_ms(60);
        }

        if(data == 0x35) //ASCII value of 5
        {
			//servo3_pin_config();
			//beeps the buzzer and shoots lazer using servo 3 port for 1000 iterations
            buzzer_on();
			
			servo_3(0);
			_delay_ms(1500);
			buzzer_off();
			//servo_3_free();
			

        }

        if(data == 0x36) //ASCII value of 6
        {
			//This is to increase the camara angle in vertical plane
			PositionPanServo_2 +=rotation_angle;
			servo_2(PositionPanServo_2);
            //if(PositionPanServo_1<=upper_limit)servo_1(PositionPanServo_1);
			//else{PositionPanServo_1 -=10;}
            _delay_ms(60);
           	
        }

        if(data == 0x37) //ASCII value of 7
        {
            //this is to decrease the  camera angle in the vertical plane and increase it in horizantal plane
			right();
			_delay_ms(left_right_delay);
			stop();
		/*	PositionPanServo_1 +=rotation_angle;
			servo_1(PositionPanServo_1);
			*/
			PositionPanServo_2 -=rotation_angle;
			_delay_ms(60);
			servo_2(PositionPanServo_2);
            //if(PositionPanServo_1<=upper_limit)servo_1(PositionPanServo_1);
			//else{PositionPanServo_1 -=rotation_angle;}
            _delay_ms(60);
        }
        if(data == 0x38) //ASCII value of 8
        {
			//this is to increase the  camera angle in the horizantal plane
			right();
			_delay_ms(left_right_delay);
			stop();
			/*PositionPanServo_1 +=rotation_angle;
			servo_1(PositionPanServo_1);
			_delay_ms(60);
			*/
        }
        if(data == 0x39) //ASCII value of 9
        {		
			//increase the camara angle of both the planes
			right();
			_delay_ms(left_right_delay);
			stop();
		/*PositionPanServo_1 +=rotation_angle;
			servo_1(PositionPanServo_1);
			_delay_ms(60);
			*/
			PositionPanServo_2 +=rotation_angle;
			servo_2(PositionPanServo_2);
            //if(PositionPanServo_1<=upper_limit)servo_1(PositionPanServo_1);
			//else{PositionPanServo_1 -=rotation_angle;}
            _delay_ms(60);
        }
		if((int)data>(int)0x39){
			PositionPanServo_2 =((int)(data)-(int)0x3A)*rotation_angle;
		
			servo_2(((int)(data)-(int)0x3A)*rotation_angle);
			_delay_ms(60);
		}

}


//!Function To Initialize all The Devices
void init_devices()
{
 cli(); //Clears the global interrupts
 port_init();  //Initializes all the ports
 uart1_init(); //Initailize UART1 for serial communiaction
 timer1_init();
 sei();   //Enables the global interrupts
}

//!Main Function This just initializes the ports and goes into a infinate wait
int main(void)
{
    PositionPanServo_1=0;
    PositionPanServo_2=0;
    PositionPanServo_3=0;
    init_devices();
	lcd_set_4bit();
	lcd_init();
//	servo_1(0);
//	_delay_ms(30);
//	servo_2(0);
   while(1){
   	//servo_3(0);
   }
}
