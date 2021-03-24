/*
*
* original code by hardlock
*
*/

//#include <tiny13a.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <wdt.h>

// use watchdog
#define USE_WATCHDOG

#define U12V 0xDC
/*
12.5 = 0x00DF // 223
13.0 = 0x00E7 // 231
13.5 = 0x00F5 // 245
14.0 = 0x00FC // 252
*/

// default pwm frequency is (1,2MHz / 255 (Top Timer0) = 4,7khz
// 0,0002133s timer overflow
#define TIMER_RESOLUTION 0.0002133

#define DELAY_TIMER_NORMAL        (int) (3.0 / 255.0 / TIMER_RESOLUTION)                // 3 sec on time
#define DELAY_TIMER_MODE_OFF      (int) (6.0 / 255.0 / TIMER_RESOLUTION)                // 6 sec off time

#define DELAY_OFF_IF_DOOR_IS_OPEN (int) (600.0 / DELAY_TIMER_NORMAL / TIMER_RESOLUTION) // 10 min delay before off
#define DELAY_ON_TIME             (int) (5.0 / DELAY_TIMER_NORMAL / TIMER_RESOLUTION)   // 5 sec delay
#define DELAY_BLINKER             (int) (0.25 / DELAY_TIMER_NORMAL / TIMER_RESOLUTION)  // 0,25 sec blink
#define DELAY_BLINKER_PAUSE       (int) (3.0 / DELAY_TIMER_NORMAL / TIMER_RESOLUTION)   // 3 sec blink pause
#define DELAY_BUTTON_PRESSED      (int) (1.0 / DELAY_TIMER_NORMAL / TIMER_RESOLUTION)   // 1 sec button pressed

// two times faster soft on when voltage threshold reached
//#define FASTER_SOFT_ON_VOLTAGE_THRESHOLD
// two times faster soft off when voltage threshold reached
#define FASTER_SOFT_OFF_VOLTAGE_THRESHOLD

//кнопка
#define BUTTON (!(PINB & _BV(PB4)))
//концевик двери
#define DOOR_CLOSED (PINB & _BV(PB1))
//вход измерения напряжения (PB2)
#define V_IN (0<<MUX1) | (1<<MUX0)

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

tModeInfo mode, setupMode;

unsigned char counter;
unsigned int counter2;

unsigned char SafeShutdown;
volatile unsigned char GoToSleep;

unsigned char blinker = 0;
unsigned char max_blink = 0;

unsigned char buttonPressed = 0;


// Read the AD conversion result
int read_adc(unsigned char adc_input)
{
  ADMUX = adc_input | (0<<REFS0);
  
  // Delay needed for the stabilization of the ADC input voltage
  _delay_us(10);
  
  // Enable ADC and start the AD conversion
  ADCSRA = (1<<ADEN) | (1<<ADSC);
  
  // Wait for the AD conversion to complete
  while ((ADCSRA & (1<<ADIF)) == 0);

  ADCSRA |= (1<<ADIF);

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

void blink(unsigned int delay)
{
  if (++counter2 < delay)
    return;
  counter2 = 0;

  if (OCR0A == 0)
  {
    TCCR0A = 0x83;
    OCR0A = 255;
  }
  else
  {
    blinker++;
    TCCR0A = 0x03;
    OCR0A = 0;
  }
}

bool voltageThreshold;

// Timer 0 overflow interrupt service routine
ISR(TIM0_OVF_vect)
{
  // Place your code here

  if (--counter > 0)
    return;

  counter = DELAY_TIMER_NORMAL;

#ifdef USE_WATCHDOG
  wdt_reset();
#endif

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
        TCCR0A = 0x03;
        OCR0A = 0;
        counter2 = 0;
        mode = MODE_SETUP;
      } else {
        return;
      }
    }
  }
  else
  {
    buttonPressed = 0;
  }

  // переход к гашению, если дверь закрыта, напряжение больше порога паузы
  voltageThreshold = read_adc(V_IN) > U12V + eeprom_read_byte(&eeMultU) * 5;

  // и находимся в режимах MODE_ON, MODE_SOFT_START, MODE_WAIT_BEFORE_OFF
  if ((mode == MODE_ON || mode == MODE_SOFT_START || mode == MODE_WAIT_BEFORE_OFF) && DOOR_CLOSED && voltageThreshold)
  {
    mode = MODE_SOFT_OFF;
  }

  switch (mode)
  {
  case MODE_SETUP:
    if (BUTTON)
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
    else
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
    break;
  case MODE_SETUP_OK:
    if (blinker < 3)
    {
      blink(DELAY_BLINKER);
    }
    else
    {
      mode = (setupMode == MODE_ON) ? MODE_SOFT_START : setupMode;
      //mode = setupMode;
      //if (mode == MODE_ON) {
      //  TCCR0A = 0x83;      //Подключаем вывод ШИМа
      //}
      counter2 = 0;
    }
    break;
  case MODE_STANDBY: //выключено - ждём.
    OCR0A = 0;       //ШИМ в ноль
    TCCR0A = 0x03;   //Отключаем вывод ШИМа.
    SafeShutdown = 0;
    GoToSleep = 1; //можно идти спать.
    break;
  case MODE_SOFT_START: //зажигаем
    TCCR0A = 0x83;      //Подключаем вывод ШИМа
#ifdef FASTER_SOFT_ON_VOLTAGE_THRESHOLD
    if (OCR0A < 254)    //Пока значение ШИМа меньше 255
    {
      OCR0A = OCR0A + (voltageThreshold ? 2 : 1); //увеличиваем его
    }
#else
    if (OCR0A < 255)    //Пока значение ШИМа меньше 255
    {
      OCR0A++; //увеличиваем его
    }
#endif
    else
    {
      mode = MODE_ON; //когда добрались до верха (255), то значить лампа полностью включилась и переходим в режим "включено"
      counter2 = 0;
    }
    break;
  case MODE_ON: //включено
    //TCCR0A = 0x83;      //Подключаем вывод ШИМа
    OCR0A = 255;
    if (++counter2 > DELAY_OFF_IF_DOOR_IS_OPEN) //если дверь открыта слишком долго, то
    {
      SafeShutdown = 1;
      mode = MODE_SOFT_OFF; //переходим к гашению...
    }

    if (DOOR_CLOSED) //если двери закрылись...
    {
      mode = MODE_WAIT_BEFORE_OFF; //переходим к паузе.
      counter2 = 0;
    }

    break;
  case MODE_WAIT_BEFORE_OFF: //ждём (eeOnTime5s * 5) сек перед тушением.
    if (++counter2 > (eeprom_read_byte(&eeOnTime5s) * DELAY_ON_TIME))
      mode = MODE_SOFT_OFF;

    if (!DOOR_CLOSED)
      mode = MODE_ON;

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

#ifdef FASTER_SOFT_OFF_VOLTAGE_THRESHOLD   
    counter = (voltageThreshold ? DELAY_TIMER_MODE_OFF >> 1 : DELAY_TIMER_MODE_OFF);
#else    
    counter = DELAY_TIMER_MODE_OFF;
#endif

    break;
  }
}

// Declare your global variables here

int main(void)
{
  // Declare your local variables here

  // Crystal Oscillator division factor: 1
  //CLKPR=(1<<CLKPCE);
  //CLKPR=(0<<CLKPCE) | (0<<CLKPS3) | (0<<CLKPS2) | (0<<CLKPS1) | (0<<CLKPS0);

  // Input/Output Ports initialization
  // Port B initialization
  // Func5=In Func4=In Func3=In Func2=In Func1=In Func0=Out
  DDRB=(0<<DDB5) | (0<<DDB4) | (0<<DDB3) | (0<<DDB2) | (0<<DDB1) | (1<<DDB0);
  // State5=T State4=P State3=T State2=T State1=P State0=0
  PORTB=(0<<PORTB5) | (1<<PORTB4) | (0<<PORTB3) | (0<<PORTB2) | (1<<PORTB1) | (0<<PORTB0);
  
  // Timer/Counter 0 initialization
  // Clock source: System Clock
  // Clock value: 1200,000 kHz
  // Mode: Fast PWM top=0xFF
  // OC0A output: Non-Inverted PWM
  // OC0B output: Disconnected
  // Timer Period: 0,21333 ms
  // Output Pulse(s):
  // OC0A Period: 0,21333 ms Width: 0 us
  TCCR0A=(1<<COM0A1) | (0<<COM0A0) | (0<<COM0B1) | (0<<COM0B0) | (1<<WGM01) | (1<<WGM00);
  TCCR0B=(0<<WGM02) | (0<<CS02) | (0<<CS01) | (1<<CS00);
  
  // External Interrupt(s) initialization
  // INT0: Off
  // Interrupt on any change on pins PCINT0-5: On
  GIMSK=(0<<INT0) | (1<<PCIE);
  
  //Sleep enabled. Power-down mod
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); 

  //Pin Change interupt on PIN.1, PIN.4
  PCMSK = (1<<PCINT4) | (1<<PCINT1);
  
  GIFR = (1<<PCIF);
  
  // Timer/Counter 0 Interrupt(s) initialization
  TIMSK0=(0<<OCIE0B) | (0<<OCIE0A) | (1<<TOIE0);
  
  // Analog Comparator initialization
  // Analog Comparator: Off
  ACSR = (1<<ACD);
  
  // Digital input buffers on ADC0: On, ADC1: Off, ADC2: On, ADC3: On
  DIDR0 = (0<<ADC0D) | (1<<ADC1D) | (0<<ADC2D) | (0<<ADC3D) | (0<<AIN1D) | (0<<AIN0D);
  
  mode = MODE_STANDBY;

#ifdef USE_WATCHDOG
  _wdt_enable(_BV(WDE), WDTO_250MS);
#endif  

  do {
   
    // Global enable interrupts
    sei();

    // Place your code here
    if (GoToSleep)
    {
#ifdef USE_WATCHDOG
      _wdt_enable(_BV(WDTIE), WDTO_4S);
#endif
      // disable ADC
      ADCSRA = (0<<ADEN);
      sei();
      while (GoToSleep) {
        sleep_mode();
        WDTCR |= _BV(WDTIE);
      }
#ifdef USE_WATCHDOG
      _wdt_enable(_BV(WDE), WDTO_250MS);
#endif      
    }
  } while (1);
}

#ifdef USE_WATCHDOG
ISR (WDT_vect, ISR_NAKED) {
  // clear output
  PORTB &= ~_BV(PB0);
  reti();
}
#endif