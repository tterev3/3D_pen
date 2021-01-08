#include <xc.h>

#define _XTAL_FREQ 16000000

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection Bits (INTOSC oscillator; I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer  
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = ON			// Flash Program Memory Code Protection (Program memory code protection is enabled)
//#warning("WARNING: CODE PROTECT OFF!")
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)

// CONFIG2
#pragma config WRT = HALF       // Flash Memory Self-Write Protection (000h to 7FFh write protected, 800h to FFFh may be modified by EECON control)
#pragma config PPS1WAY = ON     // PPSLOCK bit One-Way Set Enable bit (PPSLOCKED Bit Can Be Cleared & Set Once)
#pragma config PLLEN = OFF      // PLL Enable (4x PLL disabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOREN = ON     // Low Power Brown-out Reset enable bit (LPBOR is enabled)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)


#define NVM_location 0x1fc0

#define	T_SAMPLE_RATE	100
#define COLDEND_TEMP_THRESHOLD 500 //50.0C
#define COLDEND_OVERHEAT 600 //60.0C
#define PLA_TEMP 403
#define ABS_TEMP 513
#define EXTRUDER_STALL_TIME 250
#define IDLE_TIMEOUT 180 //seconds
#define RETRACT_POWER 500
#define RETRACT_TIME 300
#define CLICK_TIME 300
#define BLINK_TIME 125
#define REMOVE_FILAMENT_POWER 700
#define BROKEN_THERMISTOR_TIMEOUT 120

#define	adc_hotend	0b00001100		//an3 RA4
#define adc_coldend	0b00011000 //an6 RC2
#define green_led LATBbits.LATB7
#define red_led LATBbits.LATB6
#define blue_led LATCbits.LATC7
#define led_on 0
#define led_off 1
#define low_temperature_switch PORTAbits.RA5
#define system_enable_switch PORTAbits.RA0
//adc reference 2.5V A1
//extruder hall effect C3
//extruder forward C4
//extruder backward B5
//fan C0
//front switch C5
//back switch A2
//heater C1

//extruder speeds are about 50ms/1200rpm fastest, 200ms/300rpm slowest. 400rpm is a more realistic slowest usable rate
static const unsigned int speeds[3]=450,700,1500;

unsigned int t_sample_timer;
bit t_sample;
unsigned int hotend_temperature, coldend_temperature;
unsigned int hotend_target;
bit enable_hotend;
unsigned int heater_duty;
signed int hotend_derivative;
bit at_temperature;

unsigned char state;
unsigned int state_timer;

unsigned int milliseconds;
unsigned int idle_timer;
unsigned int extruder_rotation_timer;
unsigned int extruder_speed;
unsigned int new_extruder_sample;
bit extruder_speed_update;
unsigned int extruder_drive_duty;
unsigned int extruder_speed_target;
bit enable_speed_control;
unsigned int extruder_stall_timer;
signed int speed_delta;
signed int extruder_derivative;
bit allow_motion;
unsigned char selected_speed;

unsigned int debug_timer;
bit debug_flag;

void configure(void);
void delayms(int milliseconds);
unsigned int get_analog(void);

unsigned int get_hotend(void);
unsigned int get_coldend(void);
unsigned int temp_lookup(unsigned int adc_count);
unsigned int rpm_lookup(unsigned int period);

void drive_fan(unsigned int power);
void drive_extruder(unsigned int power, char direction);
void drive_heater(unsigned int power);

void debounce(void);
unsigned char port_copy;
unsigned char switch_count;
bit pressed, pressed_front, pressed_back;
bit new_press, new_press_front, new_press_back;
unsigned int press_timer;

void flash_write_int(unsigned char address, unsigned int value);
unsigned int flash_read_int(unsigned char address);
unsigned char flash_ram_copy[32];
unsigned char flash_read_byte(unsigned char address);
void flash_write(unsigned char address, unsigned char value);
void flash_write_row(void);
unsigned int abs(signed int input);

void ms_tasks(void);
unsigned int ms_ticks;


#define UART_BUFFER_SIZE 64
unsigned char uart_buffer[UART_BUFFER_SIZE];
unsigned char uart_index_in, uart_index_out;
unsigned char uart_pending;
bit uart_semaphore;
void UART_Write(char data);
void uart_string(const char * s);
void uart_hex(unsigned char input);
void uart_fifo(void);
void uart_send(char data);
void uart_int(int number);
void uart_char(unsigned char input);
void uart_ui_state(unsigned char state);



void interrupt isr(void)
{
	if(TMR0IF){
		TMR0IF=0;
        ms_ticks++;
        debounce();
	}

	if(IOCIF){
        IOCCF=0;
        new_extruder_sample=extruder_rotation_timer;
        extruder_rotation_timer=0;
        extruder_speed_update=1;
	}	  
}

void ms_tasks(void)
{
    t_sample_timer++;
    if(t_sample_timer>T_SAMPLE_RATE){
        t_sample=1;
        t_sample_timer=0;
    }	
    if(++debug_timer>1000){debug_timer=0; debug_flag=1;}
    if(state_timer) state_timer--;
    if(extruder_rotation_timer<EXTRUDER_STALL_TIME) extruder_rotation_timer++;
    else{
        new_extruder_sample=EXTRUDER_STALL_TIME;
        extruder_rotation_timer=0;
        extruder_speed_update=1;
    }
    if(++milliseconds>999){
        milliseconds=0;
        idle_timer++;
    }
    if(press_timer) press_timer--;
}

void main(void)
{
	configure();

    red_led=led_off;
    green_led=led_off;
    blue_led=led_off;
    drive_extruder(0,0);
    drive_fan(0);
    enable_hotend=0;
    enable_speed_control=0;
    extruder_drive_duty=200;
    idle_timer=0;
    state_timer=0;
    state=0;
    selected_speed=2;
    coldend_temperature = get_coldend();
    unsigned char blinker;
    //uart_string("\r\n reset"); 
    
	GIE=1;
	while(1){
         
        if(ms_ticks){
            ms_ticks--;
            ms_tasks();
        }
        
        uart_fifo();
        
        
        if(state_timer==0){
            switch(state){
                default:
                case 0: //system off
                    if(system_enable_switch){
                        state=1;
                        if(low_temperature_switch) hotend_target=PLA_TEMP;
                        else hotend_target=ABS_TEMP;
                        enable_hotend=1;
                        idle_timer=0;
                        red_led=led_on;
                    }
                    break;
                case 1: //heating up
                    if(at_temperature){
                        red_led=led_off;
                        green_led=led_on;
                        state=2;
                    }
                    else if(idle_timer>BROKEN_THERMISTOR_TIMEOUT){ //safety shutoff in case of broken thermistor
                        state=50;
                        enable_hotend=0;
                    }
                    else if(!system_enable_switch){
                        state=255;
                    }
                    break;
                case 2: //normal operation, idle
                    if(pressed_front){
                        //if(allow_motion){
                            extruder_speed_target=speeds[selected_speed];
                            drive_extruder(extruder_drive_duty,1);
                            enable_speed_control=1;
                            state=3;
                        //}
                        idle_timer=0;
                    }
                    else if(pressed_back){
                        state=5;
                        press_timer=CLICK_TIME;
                        idle_timer=0;
                    }
                    else if(idle_timer>IDLE_TIMEOUT){
                        state=254;
                        green_led=led_off;
                        red_led=led_off;
                        blue_led=led_on;
                        enable_hotend=0;
                        enable_speed_control=0;
                    }
                    else if(!system_enable_switch){
                        state=255;
                    }
                    else{
                        if(low_temperature_switch) hotend_target=PLA_TEMP;
                        else hotend_target=ABS_TEMP;
                        if(!at_temperature) red_led=led_on;
                        else red_led=led_off;
                    }
                    break;
                case 3: //extruding
                    if(!pressed_front){
                        enable_speed_control=0;
                        drive_extruder(RETRACT_POWER,0);
                        state_timer=RETRACT_TIME;
                        state=4; 
                    }
                    //need to check for hotend temperature failure and stop extruding?
                    break;
                case 4: //retracting
                    drive_extruder(0,0);
                    state=2;
                    break;
                case 5: //interpret press on rear switch
                    if(!pressed_back){ //short click
                        selected_speed++; 
                        if(selected_speed>2) selected_speed=0;
                        blinker=selected_speed+1;
                        state_timer=BLINK_TIME;
                        state=6;
                        green_led=led_off;
                        red_led=led_off;
                    }
                    else if(press_timer==0){ //switch hold
                        drive_extruder(REMOVE_FILAMENT_POWER,0);
                        state=8;
                    }
                    break;
                case 6:
                    green_led=led_on;
                    state_timer=BLINK_TIME;
                    state=7;
                    break;
                case 7:
                    green_led=led_off;
                    state_timer=BLINK_TIME;
                    if(--blinker==0){
                        green_led=led_on;
                        state=2;
                    }
                    else state=6;
                    break;
                case 8: //backing out filament
                    if(!pressed_back){
                        drive_extruder(0,0);
                        state=2;
                    }
                    break;
                case 50: //critical fault
                    state_timer=400;
                    red_led = !red_led;
                    if(!system_enable_switch) state=255;
                    break;
                case 254: //timed out, automatic cool off
                    if(!system_enable_switch) state=255; //switched off
                    else if(pressed){
                        blue_led=led_off;
                        state=0; //go start up
                    } 
                    break;
                case 255: //reset
                    red_led=led_off;
                    green_led=led_off;
                    blue_led=led_off;
                    drive_extruder(0,0);
                    enable_hotend=0;
                    enable_speed_control=0;
                    extruder_drive_duty=200;
                    idle_timer=0;
                    state_timer=0;
                    selected_speed=0;
                    state=0;
                    break;
            }
        }
        
        /*switch on: rise to temperature
once up at temp, allow extruder movement
front switch is momentary forward extrude
short retract after forward movement
click rear switch shifts between 3 speeds
hold rear switch backs out filament until released
safety timeout 3 minutes
broken thermistor detection*/
        
        
        if(debug_flag){
            debug_flag=0;
            //green_led = !green_led;
//            uart_int(extruder_speed); uart_string(" speed\r\n");
//            uart_int(extruder_drive_duty); uart_string(" duty\r\n");
//            uart_int(speed_delta); uart_string(" delta\r\n");
//            uart_int(hotend_temperature);
//            uart_string(" hotend (raw)\r\n");
//            uart_int(heater_duty); uart_string(" heater duty\r\n");
//            uart_int(coldend_temperature);
//            uart_string("C coldend\r\n\r\n");
        }
        
            
		if(t_sample){
			t_sample=0;
            hotend_derivative = hotend_temperature;
			hotend_temperature = get_hotend();
            hotend_derivative -= hotend_temperature;
            //coldend_temperature = get_coldend();
            unsigned int sample = get_coldend();
            if(sample>coldend_temperature) coldend_temperature++;
            if(sample<coldend_temperature) coldend_temperature--;
            if(coldend_temperature>COLDEND_OVERHEAT){
                //red_led=led_on;
                drive_fan(800);
            }
            else if(coldend_temperature>COLDEND_TEMP_THRESHOLD){
                //red_led=led_on;
                drive_fan(200);
            }
            else if(coldend_temperature<(COLDEND_TEMP_THRESHOLD-20)){ //2.0 degree hysteresis
                //red_led=led_off;
                drive_fan(0);
            }
            
            signed int temp_delta = hotend_target - hotend_temperature;
            signed int new_duty = heater_duty;

            if(abs(temp_delta)<10 && hotend_target!=0){ at_temperature=1; allow_motion=1;}
            else at_temperature=0;
            
            if(hotend_target==0 || (hotend_temperature<(hotend_target-30))){
                allow_motion=0;
            }
            
            if(enable_hotend){
                new_duty+=temp_delta/4;
                new_duty+=hotend_derivative*8;
                
                if(new_duty>1023) new_duty=1023;
                if(new_duty<0) new_duty=0;
                heater_duty=new_duty;
                drive_heater(heater_duty);
            }
            else{
                heater_duty=0;
                drive_heater(heater_duty);
            }
		}	
        
        if(extruder_speed_update){
            extruder_speed_update=0;
            extruder_derivative = extruder_speed;
            extruder_speed = rpm_lookup(new_extruder_sample);
            extruder_derivative -= extruder_speed;
            if(enable_speed_control){
                speed_delta = extruder_speed_target - extruder_speed;
                signed int new_duty = extruder_drive_duty;
                new_duty += speed_delta/8;
                new_duty += extruder_derivative/8;
                if(new_duty>1023) new_duty=1023;
                if(new_duty<100) new_duty=100;
                extruder_drive_duty=new_duty;
                drive_extruder(extruder_drive_duty,1);
            }
        }

	}
}


void drive_extruder(unsigned int power, char direction)
{
    if(power>1023) power=1023;
    if(power==0){
        LATBbits.LATB5=0; //
        PWM1DC=0; PWM1LDCONbits.LDA=1;
    }
    else{
        if(direction){
            LATBbits.LATB5=0;
            PWM1DC=power; PWM1LDCONbits.LDA=1;
        }
        else{
            LATBbits.LATB5=1;
            PWM1DC=1023-power; PWM1LDCONbits.LDA=1;
        }
    }
}
	
void drive_fan(unsigned int power)
{
	PWM2DC=power;
	PWM2LDCONbits.LDA=1;
}	

void drive_heater(unsigned int power)
{
	PWM3DC=power;
	PWM3LDCONbits.LDA=1;
}

void configure(void)
{
    LATA=0;
    LATC=0;

    ANSELA=0b00010010;
    TRISA= 0b11111111;	//
    WPUA=  0b11101101;	//

    ANSELC=0b00000100;
    TRISC= 0b01101100;
    WPUC=  0b01101000;    
    
    ANSELB=0b00000000;
    TRISB= 0b00011111;
    WPUB=  0b00011111;

//    TXSTA=0b10100000; //async, transmit enabled, low speed
//    RCSTA=0b10000000; //enable port, do not enable receive
//    BAUDCON=0b01000000; //don't invert tx data, use 8b baud rate generator
//    SPBRGL=12; //19200 baud with 8bit and low speed set
//    SPBRGH=0;
//    uart_index_in=0;
//    uart_index_out=0;
//    uart_pending=0;
//    uart_semaphore=0;
    
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0x00; // unlock PPS
	
    //RB7PPS = 0b00001001; //UART TX 
    
    RC4PPS = 0b00000011; //PWM1 out extruder
    RC0PPS = 0b00000100; //PWM2 out fan
    RC1PPS = 0b00000101; //PWM3 out heater
	
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0x01; // lock PPS

    INTCON=0b00101000;	//ioc and tmr0
    IOCAN=0b00000000;
    IOCAP=0b00000000;
	IOCCN=0b00001000;
	IOCCP=0b00000000;    
	IOCBN=0b00000000;
	IOCBP=0b00000000;

    IOCAF=0;
    IOCBF=0;
	IOCCF=0;
    PIE1=0b00000000;	//
    PIE2=0b00000000;	//
    PIE3=0b00000000;	//
    PIR1=0;
    PIR2=0;

    OSCCON=0b01111011;//0b01110011;
    OPTION_REG=0b00000011; 	//1khz tmr0 ints
    T1CON=0b00100101;	//prescale 1:4. counts at 1MHz. 

    ADCON0=0b00011001;	//AN6
	ADCON1=0b10100010;	//1/32, right justify, vref from vref pin A1
    
	PWM1TMR=0;
	PWM2TMR=0;

    PWM1CON = 0; 
    PWM1CLKCON = 0b01000000;//1:16 for 1kHz at 10bit
	PWM1PR = 1023;
	PWM1TMR=0;
	PWM1PH = 0;
	PWM1DC = 1023; //default
	PWM1OFCON = 0;
	PWM1OF = 0;
    PWM1LDCON=0;

	PWM2CON = 0;
	PWM2CLKCON = 0b01000000;//1:16 for 1kHz at 10bit
	PWM2PR = 1023;
	PWM2PH = 0;
	PWM2DC = 1023; 
	PWM2OFCON = 0;
	PWM2OF = 1023;
    PWM2LDCON=0;

	PWM3CON = 0;
	PWM3CLKCON = 0b01110000;//1:128 for 122Hz at 10bit
	PWM3PR = 1023;
	PWM3PH = 0;
	PWM3DC = 1023; 
	PWM3OFCON = 0;
	PWM3OF = 1023;
    PWM3LDCON=0;
    
    
    PWM1CON = 0b10000000; //active high standard mode
	PWM2CON = 0b10000000; //active high standard mode
	PWM3CON = 0b10000000; //active high standard mode
}

void debounce(void)
{
	unsigned char port_read=PORTA;
	port_read&=0b00000100;
    unsigned char c_temp = PORTC;
    c_temp&=0b00100000;
    port_read|=c_temp;
	if(port_read==port_copy){
		if(++switch_count>9){
            switch_count=0;
			if((port_copy&0b00100100)==0b00100100){
 				pressed=0; pressed_front=0; pressed_back=0;
			}
			else{
				if(!pressed){ new_press=1;}
				pressed=1;
                if((port_copy&0b00100000)==0){
                    if(!pressed_front) new_press_front=1;
                    pressed_front=1;
                }
                if((port_copy&0b00000100)==0){
                    if(!pressed_back) new_press_back=1;
                    pressed_back=1;
                }                
			}
		}
	}
	else{
		switch_count=0;
		port_copy=port_read;
	}
}

unsigned int abs(signed int input)
{
	if(input<0) return -input;
	return input;
}

void delayms(int milliseconds)
{
	while(milliseconds!=0){ __delay_ms(1); milliseconds--;}
}

unsigned int rpm_lookup(unsigned int period)
{
    static const unsigned int table[]=60000,60000,30000,20000,15000,12000,10000,8571,7500,6667,6000,5455,5000,4615,4286,4000,3750,3529,3333,3158,3000,2857,2727,2609,2500,2400,2308,2222,2143,2069,2000,1935,1875,1818,1765,1714,1667,1622,1579,1538,1500,1463,1429,1395,1364,1333,1304,1277,1250,1224,1200,1176,1154,1132,1111,1091,1071,1053,1034,1017,1000,984,968,952,938,923,909,896,882,870,857,845,833,822,811,800,789,779,769,759,750,741,732,723,714,706,698,690,682,674,667,659,652,645,638,632,625,619,612,606,600,594,588,583,577,571,566,561,556,550,545,541,536,531,526,522,517,513,508,504,500,496,492,488,484,480,476,472,469,465,462,458,455,451,448,444,441,438,435,432,429,426,423,420,417,414,411,408,405,403,400,397,395,392,390,387,385,382,380,377,375,373,370,368,366,364,361,359,357,355,353,351,349,347,345,343,341,339,337,335,333,331,330,328,326,324,323,321,319,317,316,314,313,311,309,308,306,305,303,302,300,299,297,296,294,293,291,290,288,287,286,284,283,282,280,279,278,276,275,274,273,271,270,269,268,267,265,264,263,262,261,260,259,258,256,255,254,253,252,251,250,249,248,247,246,245,244,243,242,241,240;
    if(period>250) return table[250];
    return table[period];
}

unsigned int get_hotend(void)
{
    ADCON1=0b10100010;	//1/32, right justify, vref from vref pin A1
	ADCON0 = adc_hotend | 0b00000001; //select channel and turn on
	__delay_us(40);
	GO_nDONE=1;
	while(GO_nDONE);
    return (1023-ADRES);
    //PLA 1.51V = 618 raw, 405 inverted
    //ABS 1.24V = 507 raw, 516 inverted
}

unsigned int get_coldend(void)
{
    ADCON1=0b10100000;	//1/32, right justify, vref from vdd
	ADCON0 = adc_coldend | 0b00000001; //select channel and turn on
	__delay_us(40);
	GO_nDONE=1;
	while(GO_nDONE);
    return temp_lookup(ADRES);
}

unsigned int temp_lookup(unsigned int adc_count)
{
	static const signed int table[]=-637,-637,-560,-511,-476,-448,-424,-403,-385,-369,-355,-341,-329,-317,-307,-296,-287,-278,-269,-261,-253,-245,-238,-231,-225,-218,-212,-206,-200,-194,-189,-183,-178,-173,-168,-163,-159,-154,-149,-145,-141,-136,-132,-128,-124,-120,-116,-113,-109,-105,-102,-98,-95,-91,-88,-84,-81,-78,-75,-72,-68,-65,-62,-59,-56,-54,-51,-48,-45,-42,-40,-37,-34,-31,-29,-26,-24,-21,-19,-16,-14,-11,-9,-6,-4,-2,1,3,5,8,10,12,14,16,19,21,23,25,27,29,31,33,36,38,40,42,44,46,48,49,51,53,55,57,59,61,63,65,67,68,70,72,74,76,77,79,81,83,84,86,88,90,91,93,95,96,98,100,101,103,105,106,108,109,111,113,114,116,117,119,121,122,124,125,127,128,130,131,133,134,136,137,139,140,142,143,145,146,148,149,150,152,153,155,156,158,159,160,162,163,165,166,167,169,170,171,173,174,176,177,178,180,181,182,184,185,186,188,189,190,191,193,194,195,197,198,199,200,202,203,204,206,207,208,209,211,212,213,214,216,217,218,219,220,222,223,224,225,227,228,229,230,231,233,234,235,236,237,239,240,241,242,243,244,246,247,248,249,250,251,253,254,255,256,257,258,260,261,262,263,264,265,266,267,269,270,271,272,273,274,275,276,278,279,280,281,282,283,284,285,286,287,289,290,291,292,293,294,295,296,297,298,299,301,302,303,304,305,306,307,308,309,310,311,312,313,314,316,317,318,319,320,321,322,323,324,325,326,327,328,329,330,331,332,333,334,335,336,338,339,340,341,342,343,344,345,346,347,348,349,350,351,352,353,354,355,356,357,358,359,360,361,362,363,364,365,366,367,368,369,370,371,372,373,374,375,376,377,378,379,380,381,382,383,384,385,386,387,388,389,390,391,392,393,394,395,396,397,398,399,400,401,402,403,404,405,406,407,408,409,410,411,412,413,414,415,416,417,418,419,420,421,422,423,424,425,426,427,428,429,430,431,432,433,434,435,436,437,438,439,440,441,442,443,444,445,446,447,448,449,450,451,452,453,454,455,456,457,458,459,460,461,462,463,464,465,466,467,468,469,470,471,472,473,474,475,476,477,478,479,480,481,482,483,484,485,486,487,488,489,490,491,492,493,494,495,496,497,498,499,500,501,502,503,504,505,506,507,508,509,510,511,512,513,514,515,516,517,518,519,520,521,522,523,524,525,526,527,528,529,530,531,532,533,534,535,536,537,538,539,540,541,542,543,544,545,546,547,548,550,551,552,553,554,555,556,557,558,559,560,561,562,563,564,565,566,567,568,569,570,571,573,574,575,576,577,578,579,580,581,582,583,584,585,586,587,588,590,591,592,593,594,595,596,597,598,599,600,601,603,604,605,606,607,608,609,610,611,612,614,615,616,617,618,619,620,621,622,624,625,626,627,628,629,630,631,633,634,635,636,637,638,639,640,642,643,644,645,646,647,649,650,651,652,653,654,655,657,658,659,660,661,663,664,665,666,667,668,670,671,672,673,674,676,677,678,679,680,682,683,684,685,687,688,689,690,691,693,694,695,696,698,699,700,701,703,704,705,707,708,709,710,712,713,714,715,717,718,719,721,722,723,725,726,727,728,730,731,732,734,735,736,738,739,740,742,743,744,746,747,749,750,751,753,754,755,757,758,760,761,762,764,765,767,768,769,771,772,774,775,777,778,779,781,782,784,785,787,788,790,791,793,794,796,797,799,800,802,803,805,806,808,809,811,812,814,816,817,819,820,822,823,825,827,828,830,831,833,835,836,838,840,841,843,845,846,848,850,851,853,855,856,858,860,861,863,865,867,868,870,872,874,875,877,879,881,883,884,886,888,890,892,894,895,897,899,901,903,905,907,909,911,913,914,916,918,920,922,924,926,928,930,932,934,936,939,941,943,945,947,949,951,953,955,958,960,962,964,966,968,971,973,975,977,980,982,984,987,989,991,994,996,998,1001,1003,1006,1008,1010,1013,1015,1018,1020,1023,1025,1028,1031,1033,1036,1038,1041,1044,1046,1049,1052,1055,1057,1060,1063,1066,1069,1071,1074,1077,1080,1083,1086,1089,1092,1095,1098,1101,1104,1107,1111,1114,1117,1120,1124,1127,1130,1133,1137,1140,1144,1147,1151,1154,1158,1161,1165,1169,1172,1176,1180,1184,1188,1191,1195,1199,1203,1207,1211,1216,1220,1224,1228,1233,1237,1241,1246,1250,1255,1260,1264,1269,1274,1279,1284,1289,1294,1299,1304,1309,1315,1320,1326,1331,1337,1343,1349,1354,1360,1367,1373,1379,1386,1392,1399,1405,1412,1419,1426,1434,1441,1449,1456,1464,1472,1480,1488,1497,1506,1514,1524,1533,1542,1552,1562,1572,1583,1593,1604,1616,1627,1640,1652,1665,1678,1691,1705,1720,1735,1751,1767,1784,1801,1820,1839,1859,1879,1901,1924,1949,1974,2002,2031,2061,2094,2130,2168,2210,2255,2305,2361,2423,2493,2574,2668,2781,2922,3103,3355,3748,4552;
    if(table[adc_count]<0) return 0;
	return table[adc_count];
}

void flash_write_int(unsigned char address, unsigned int value)
{
    unsigned char single = value&0xff;
    flash_write(address, single);
    value>>=8; value&=0xff;
    single=value;
    flash_write(address+1,single);
}
unsigned int flash_read_int(unsigned char address)
{
    unsigned int result = flash_read_byte(address+1);
    result<<=8;
    result |= flash_read_byte(address);
    return result;
}
unsigned char flash_read_byte(unsigned char address)
{
	PMADR = NVM_location+address;
#asm
	BANKSEL PMADRL
	BCF	PMCON1,6 //CFGS
	BSF	PMCON1,0 //RD
	NOP
	NOP
#endasm
	return PMDATL;
}

void flash_write(unsigned char address, unsigned char value)
{
	if(address>31) return;
	for(char i=0; i<32; i++){
		flash_ram_copy[i]=flash_read_byte(i);
	}
	flash_ram_copy[address]=value;
	flash_ram_copy[7]=calc_checksum();
	//flash_write_row(0);
	//flash_write_row(1);
	flash_write_row();
}

void flash_write_row(void)
{
	char gie_copy=0;
	if(GIE) gie_copy=1;
	GIE=0;
	PMADR = NVM_location;
#asm	//row erase
	BCF	PMCON1,6 //CFGS
	BSF	PMCON1,4 //FREE
	BSF	PMCON1,2 //WREN
	MOVLW	0x55
	MOVWF	PMCON2
	MOVLW	0xAA
	MOVWF	PMCON2
	BSF	PMCON1,1 //WR
	NOP
	NOP
	BCF	PMCON1,2 //WREN
#endasm
	PMADR = NVM_location; 
#asm
	BCF	PMCON1,6 //CFGS
	BCF	PMCON1,4 //FREE
	BSF	PMCON1,2 //WREN
	BSF	PMCON1,5 //LWLO
#endasm
	for(char i=0; i<31; i++){
		PMDATH=0x34; //retlw
		PMDATL=flash_ram_copy[i];
		#asm
			MOVLW	0x55
			MOVWF	PMCON2
			MOVLW	0xAA
			MOVWF	PMCON2
			BSF	PMCON1,1 //WR
			NOP
			NOP
			INCF	PMADRL,F
		#endasm
	}
	PMDATH=0x34; //retlw
	PMDATL=flash_ram_copy[31];
#asm
	BCF PMCON1,5 //LWLO	
	MOVLW	0x55
	MOVWF	PMCON2
	MOVLW	0xAA
	MOVWF	PMCON2
	BSF	PMCON1,1 //WR
	NOP
	NOP
	BCF	PMCON1,2 //WREN
#endasm
	if(gie_copy) GIE=1;
}


void uart_fifo(void)
{
    if(uart_pending){
        if(TXSTAbits.TRMT){	//only write if wz_rx_buffer is empty
            if(!uart_semaphore){
                uart_index_out++;
                if(uart_index_out>=UART_BUFFER_SIZE) uart_index_out=0;
                uart_pending--;
                UART_Write(uart_buffer[uart_index_out]);
            }
        }	
    }
}	
void uart_send(char data)
{
    while(uart_pending>=UART_BUFFER_SIZE) uart_fifo();
    uart_semaphore=1;
    uart_index_in++; 
    if(uart_index_in>=UART_BUFFER_SIZE) uart_index_in=0;
    uart_pending++;
    uart_buffer[uart_index_in]=data;
    uart_semaphore=0;
}
void uart_char(unsigned char input)
{
    char hundred=0;
    char ten=0;
    while(input>99){
        input-=100;
        hundred++;
    }
    while(input>9){
        input-=10;
        ten++;
    }
    uart_send(hundred+48);
    uart_send(ten+48);
    uart_send(input+48);
}

void uart_int(int number)
{
    if(number<0){
        uart_send('-');
        number=-number;
    }	
    char tthousand=0;
    char thousand=0;
    char hundred=0;
    char ten=0;
    while(number>9999){
        number-=10000;
        tthousand++;
    }
    while(number>999){
        number-=1000;
        thousand++;
    }
    while(number>99){
        number-=100;
        hundred++;
    }
    while(number>9){
        number-=10;
        ten++;
    }
    uart_send(tthousand+48);
    uart_send(thousand+48);
    uart_send(hundred+48);
    uart_send(ten+48);
    uart_send(number+48);
}	
void uart_hex(unsigned char input)
{
    unsigned char output = (input>>4)&0x0f;
    output+=48;
    if(output>57) output+=7;
    uart_send('0');uart_send('x');
    uart_send(output);
    output = (input)&0x0f;
    output+=48;
    if(output>57) output+=7;
    uart_send(output);
}	
void uart_string(const char * s)
{
    if(*s==0) return;
    while(*s){	//may consider adding length restriction 
        uart_send(*s++);
    }
}	
void UART_Write(char data)
{
  TXREG = data;
}


const char nvm_data[] @ NVM_location = { //must be at least 32byte long (min row erase)
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
};

