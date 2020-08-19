/*****************************************************
This program was produced by the
CodeWizardAVR V1.25.9 Professional
Automatic Program Generator
© Copyright 1998-2008 Pavel Haiduc, HP InfoTech s.r.l.
http://www.hpinfotech.com

Project : 
Version : 
Date    : 17.03.2010
Author  : hardlock
Company : Hardlock Company
Comments: 


Chip type           : ATtiny13
Clock frequency     : 9,600000 MHz
Memory model        : Tiny
External SRAM size  : 0            
Data Stack size     : 16
*****************************************************/

//#include <tiny13a.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/eeprom.h>

#define ADC_VREF_TYPE 0x00
#define U12V 0xDC
/*
12.5 = 0x00DF // 223
13.0 = 0x00E7 // 231
13.5 = 0x00F5 // 245
14.0 = 0x00FC // 252
*/

// default pwm frequency is (9,6MHz / 255 (Top Timer0) = 37,65 khz
// 0,0000265625s timer overflow

// when defined use (37500kHz / 255 (Top Timer0) = 146,5hz pwm
// 0,0068s timer overflow
#define SLOW_PWM

#ifdef SLOW_PWM
#define DELAY_TIMER_NORMAL 2            // 2 * 0,0068 = 0.0136s timer * 255 = 3,468 sec
#define DELAY_TIMER_MODE_OFF 4          // 6,936 sec
#define DELAY_OFF_IF_DOOR_IS_OPEN 44000 // * 0.0136s sek = 10min
#define DELAY_ON_TIME 368               // * 0,0136 = 5 sec
#define DELAY_BLINKER 18                // * 0,0136 = 0,25 sec
#define DELAY_BLINKER_PAUSE 220         // * 0,0136 = 3 sec
#define DELAY_BUTTON_PRESSED 74         // * 0,0136 = 1 sec
#else
#define DELAY_TIMER_NORMAL 376          // 376 * 0,0000265625 = 0.01s timer * 255 = 2,55 sec
#define DELAY_TIMER_MODE_OFF 1000       // 6,77 sec
#define DELAY_OFF_IF_DOOR_IS_OPEN 60000 // * 0.01 sek = 10min
#define DELAY_ON_TIME 500               // * 0,01 = 5 sec
#define DELAY_BLINKER 25                // * 0,01 = 0,25 sec
#define DELAY_BLINKER 25                // * 0,01 = 0,25 sec
#define DELAY_BLINKER_PAUSE 300         // * 0,01 = 3 sec
#define DELAY_BUTTON_PRESSED 100        // * 0,01 = 1 sec
#endif

#define BUTTON (!(PINB & _BV(PB4)))   //кнопка
#define DOOR_CLOSED (PINB & _BV(PB1)) //концевик двери

typedef enum
{
  MODE_STANDBY = 0,
  MODE_SOFT_START,
  MODE_ON,
  MODE_WAIT_BEFORE_OFF,
  MODE_SOFT_OFF,
  MODE_SETUP,
  MODE_SETUP_OK,
} tModeInfo;

EEMEM unsigned char eeMultU = 1;
EEMEM unsigned char eeOnTime5s = 2;

tModeInfo mode;

unsigned int counter;
unsigned int counter2;
unsigned char GoToSleep;
unsigned char SafeShutdown;

// Read the AD conversion result
unsigned int read_adc(unsigned char adc_input)
{
  ADMUX = adc_input | (ADC_VREF_TYPE & 0xff);
  // Delay needed for the stabilization of the ADC input voltage
  _delay_us(10);
  // Start the AD conversion
  ADCSRA |= 0x40;
  // Wait for the AD conversion to complete
  while ((ADCSRA & 0x10) == 0)
    ;
  ADCSRA |= 0x10;
  return ADCW;
}

// Pin change interrupt service routine
ISR(PCINT0_vect)
{
  // Place your code here
  if (!DOOR_CLOSED)
    if (mode == MODE_STANDBY)
    {
      OCR0A = 0;
      mode = MODE_SOFT_START;
      GoToSleep = 0;
      SafeShutdown = 0;
    }

  if (BUTTON)
  {
    //    OCR0A = 0;
    GoToSleep = 0;
    SafeShutdown = 0;
  }
}

unsigned char blinker = 0;
unsigned char max_blink = 0;

tModeInfo setupMode;

unsigned char buttonPressed = 0;

void blink(unsigned int delay)
{
  if (++counter2 < delay)
    return;
  counter2 = 0;

  if (OCR0A == 0)
  {
    OCR0A = 255;
  }
  else
  {
    blinker++;
    OCR0A = 0;
  }
}

// Timer 0 overflow interrupt service routine
ISR(TIM0_OVF_vect)
{
  // Place your code here
  unsigned int iVoltage;
  unsigned int iLevelOff;
  unsigned int delay;

  if (mode == MODE_SOFT_OFF)
  {
    delay = DELAY_TIMER_MODE_OFF; //это шоб чуть плавнее гасло...
  }
  else
  {
    delay = DELAY_TIMER_NORMAL;
  }

  counter++;
  if (counter < delay)
  {
    return;
  }
  else
  {
    counter = 0;
  }

  if (BUTTON)
  {
    if ((mode == MODE_STANDBY) || (mode == MODE_ON))
    {
      buttonPressed++;
      if (buttonPressed > DELAY_BUTTON_PRESSED)
      {
        setupMode = mode;
        blinker = 0;
        max_blink = setupMode == MODE_STANDBY ? 6 : 12;
        TCCR0A = 0x83;
        OCR0A = 0;
        counter2 = 0;
        mode = MODE_SETUP;
      }
    }

    if (mode == MODE_SETUP)
    {
      if (blinker <= max_blink)
      {
        blink((OCR0A == 0) ? (DELAY_BLINKER << 2) : DELAY_BLINKER);
      }
      else
      {
        if (++counter2 < DELAY_BLINKER_PAUSE)
          return;
        counter2 = 0;
        blinker = 0;
      }
    }
  }
  else
  {
    if (mode == MODE_SETUP)
    {
      if (blinker > 0)
      {
        if (setupMode == MODE_ON)
        {
          eeprom_write_byte(&eeOnTime5s, blinker - 1);
        }
        else
        {
          eeprom_write_byte(&eeMultU, blinker - 1);
        }
      }
      blinker = 0;
      OCR0A = 0;
      mode = MODE_SETUP_OK;
    }

    if (mode == MODE_SETUP_OK)
    {
      if (blinker < 3)
      {
        blink(DELAY_BLINKER);
      }
      else
      {
        mode = setupMode;
        counter2 = 0;
      }
    }

    buttonPressed = 0;
  }

  iVoltage = read_adc(1);                            //читаем напряжение на входе
  iLevelOff = U12V + eeprom_read_byte(&eeMultU) * 5; //вычисляем напряжение, при котором не надо ждать ПАУЗУ до гашения

  switch (mode)
  {
  case MODE_STANDBY: //выключено - ждём.
    OCR0A = 0;       //ШИМ в ноль
    TCCR0A = 0x03;   //Отключаем вывод ШИМа.
    SafeShutdown = 0;

    GoToSleep = 1; //можно идти спать.
    break;
  case MODE_SOFT_START: //зажигаем
    TCCR0A = 0x83;      //Подключаем вывод ШИМа
    if (DOOR_CLOSED && (iVoltage > iLevelOff))
    {
      mode = MODE_SOFT_OFF;
    }
    else if (OCR0A < 255) //Пока значение ШИМа меньше 255
    {
      OCR0A++; //увеличиваем его
    }
    else
    {
      mode = MODE_ON; //когда добрались до верха (255), то значить лампа полностью включилась и переходим в режим "включено"
      counter2 = 0;
    }
    break;
  case MODE_ON: //включено
    OCR0A = 255;
    counter2++;
    if (counter2 > DELAY_OFF_IF_DOOR_IS_OPEN) //если дверь открыта слишком долго, то
    {
      SafeShutdown = 1;
      mode = MODE_SOFT_OFF; //переходим к гашению...
    }

    if (DOOR_CLOSED) //если двери закрылись...
    {
      if (iVoltage > iLevelOff) //..и напряжение бортсети больше порогового, то
      {
        mode = MODE_SOFT_OFF; //сразу переходим к гашению
      }
      else
      {
        mode = MODE_WAIT_BEFORE_OFF; //иначе переходим к паузе.
        counter2 = 0;
      }
    }

    break;
  case MODE_WAIT_BEFORE_OFF: //ждём (eeOnTime5s * 5) сек перед тушением.
    if (++counter2 > (eeprom_read_byte(&eeOnTime5s) * DELAY_ON_TIME))
      mode = MODE_SOFT_OFF;

    if (!DOOR_CLOSED)
      mode = MODE_ON;

    if (DOOR_CLOSED && iVoltage > iLevelOff)
    {
      mode = MODE_SOFT_OFF;
    }
    break;
  case MODE_SOFT_OFF: //тушим
    if (OCR0A > 0)
    {
      OCR0A--;
      if (!DOOR_CLOSED)
      {
        if (!SafeShutdown)
          mode = MODE_SOFT_START;
      }
    }
    else
    {
      mode = MODE_STANDBY;
      SafeShutdown = 0;
    }
    break;
  }
}

// Declare your global variables here

int main(void)
{
  // Declare your local variables here

  // Crystal Oscillator division factor: 1
  CLKPR = 0x80;
  CLKPR = 0x00;

  // Input/Output Ports initialization
  // Port B initialization
  // Func5=In Func4=In Func3=In Func2=In Func1=In Func0=Out
  // State5=T State4=P State3=P State2=T State1=P State0=0
  PORTB = 0x1A;
  DDRB = 0x01;

#ifdef SLOW_PWM
  // Timer/Counter 0 initialization
  // Clock source: System Clock
  // Clock value: 37,500 kHz
  // Mode: Fast PWM top=0xFF
  // OC0A output: Non-Inverted PWM
  // OC0B output: Disconnected
  // Timer Period: 6,8267 ms
  // Output Pulse(s):
  // OC0A Period: 6,8267 ms Width: 0 us
  TCCR0A = 0x03;
  TCCR0B = 0x04;
#else
  // Timer/Counter 0 initialization
  // Clock source: System Clock
  // Clock value: 9600,000 kHz
  // Mode: Fast PWM top=FFh
  // OC0A output: Non-Inverted PWM
  // OC0B output: Disconnected
  TCCR0A = 0x03;
  TCCR0B = 0x01;
#endif
  //TCNT0=0x00;
  //OCR0A=0x00;
  //OCR0B=0x00;

  // External Interrupt(s) initialization
  // INT0: Off
  // Interrupt on any change on pins PCINT0-5: On
  GIMSK = 0x20;
  MCUCR = 0x30; //Sleep enabled. Power-down mod
  PCMSK = 0x12; //Pin Change interupt on PIN.1, PIN.4
  GIFR = 0x20;

  // Timer/Counter 0 Interrupt(s) initialization
  TIMSK0 = 0x02;

  // Analog Comparator initialization
  // Analog Comparator: Off
  ACSR = 0x80;
  //ADCSRB=0x00;

  // ADC initialization
  // ADC Clock frequency: 600,000 kHz
  // ADC Bandgap Voltage Reference: Off
  // ADC Auto Trigger Source: None
  // Digital input buffers on ADC0: On, ADC1: On, ADC2: On, ADC3: On
  DIDR0 &= 0x03;
  DIDR0 |= 0x00;
  ADMUX = ADC_VREF_TYPE & 0xff;
  ADCSRA = 0x84;

  mode = MODE_STANDBY;
  //if (!DOOR_CLOSED)
  //  mode = MODE_SOFT_START;
  //GoToSleep = 1;
  //SafeShutdown = 0;

  // Global enable interrupts
  asm("sei");

  while (1)
  {
    // Place your code here
    if (GoToSleep)
    {
      asm("sei");
      asm("sleep");
    }
  };
}
