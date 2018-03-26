/* TODOs */
/* when threshold reached, goto ultra low power mode */
/* improve adc measure */
/* add a LED indicator (reuse LED is possible ?) */
/* how to calibrate (ie. automatic threshold setting) */
/* test with router */


#include <stdint.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <util/delay_basic.h>

/* must be defined before uart */
#define CLK_PRESCAL (256UL)
/* #define CLK_PRESCAL (1UL) */

#define CONFIG_UART
#ifdef CONFIG_UART
#include "./uart.c"
#endif /* CONFIG_UART */


/* adc */

#define ADC_R0 15000.0
#define ADC_R1 995.0
#define ADC_VREF 1.1
#define ADC_NBITS 1024.0
#define ADC_VOLT_TO_CODE(__v) \
((uint16_t)(((__v) * ADC_R1 * ADC_NBITS) / ((ADC_R0 + ADC_R1) * ADC_VREF)))
#define ADC_THRESH ADC_VOLT_TO_CODE(11.000)

static void adc_init(void)
{
  /* 1.1V internal ref. use ADC0. */
  ADMUX = (3 << 6) | (0 << 0);
}

static uint16_t adc_read(void)
{
  /* Vx = (x * (1.1 / 2^10)) */
  /* Vx = Vin * (1 / (15 + 1)) */
  /* (x * (1.1 / 2^10)) = Vin * (1 / (15 + 1)) */
  /* Vin = (x * (1.1 / 2^10)) / (1 / (15 + 1)) */
  /* x = Vin * (1 / (15 + 1)) / (1.1 / 2^10) */

  /* in practice, resistors are 926 and 14.71K */
  /* if x = 1, Vin = 0.018139V (resolution) */

  /* offset error is around 8mV */

#define ADC_NSUM 4
#define ADC_OFF 8

  uint8_t i;
  uint16_t x;
  uint16_t sum;

  /* enable adc */
  ADCSRA |= (1 << 7);
  _delay_ms(1);

  /* first read after enable is alway buggy */
  ADCSRA |= (1 << 6);
  while (ADCSRA & (1 << 6)) ;

  sum = 0;
  for (i = 0; i != ADC_NSUM; ++i)
  {
    /* start conversion */
    ADCSRA |= (1 << 6);

    /* wait conversion done */
    while (ADCSRA & (1 << 6)) ;

    x = ADCL;
    __asm__ __volatile__ ("nop");
    x |= ((uint16_t)ADCH) << 8;
    x &= 0x3ff;
    sum += x;
  }

  /* disable adc */
  ADCSRA &= ~(1 << 7);

  sum /= ADC_NSUM;
  if (sum > ADC_OFF) sum -= ADC_OFF;
  else sum = 0;

  return sum;
}


/* mosfet */

#define FET_MASK (1 << 2)
#define FET_PORT PORTB
#define FET_DDR DDRB

static void fet_disable(void)
{
  FET_PORT &= ~FET_MASK;
}

static void fet_enable(void)
{
  FET_PORT |= FET_MASK;
}

static void fet_init(void)
{
  FET_DDR |= FET_MASK;
  fet_disable();
}


/* core clock prescaler */

static inline void clk_set_prescal(uint8_t x)
{
  /* pow2, max 8 = 256 */

  CLKPR = 1 << 7;
  CLKPR = x << 0;

  /* wait for 4 - 2 cycles */
  __asm__ __volatile__ ("nop\n");
  __asm__ __volatile__ ("nop\n");
}

static inline void clk_set_prescal_max(void)
{
  clk_set_prescal(8);
}


/* timer */

#define TIMER_PRESCAL (1024UL)

static volatile uint8_t timer_irq = 0;

ISR(TIMER1_COMPA_vect)
{
#if 0
  /* stop the timer, signal interrupt */
  TCCR1B &= ~((1 << 3) - 1);
#endif /* 0 */

  timer_irq = 1;
}

static uint32_t timer_ms100_to_counter(uint8_t ms100)
{
  static const uint32_t p = TIMER_PRESCAL * CLK_PRESCAL;
  return (F_CPU * (uint32_t)ms100) / (p * 10UL);
}

static void timer_init(uint16_t n)
{
  /* n the counter */
  
  /* stop timer */
  TCCR1B = 0;

  /* CTC mode, overflow when OCR1A reached */
  TCCR1A = 0;
  OCR1A = n;
  TCNT1 = 0;
  TCCR1C = 0;

  /* interrupt on OCIE0A match */
  TIMSK1 = 1 << 1;

  /* clear soft irq flag */
  timer_irq = 0;

  /* set mode high bits, 1024 prescal */
  TCCR1B = (1 << 3) | (5 << 0);
}


/* main */

int main(void)
{
  uint8_t i;
  uint8_t j;
  int8_t k;

  /* must be power of 2 */
#define THRESH_COUNT 4
  uint16_t v[THRESH_COUNT];

#ifdef CONFIG_UART
  uart_setup();
#else
  /* power consumption, disable uart */
  UCSR0B = 0;
  PRR |= (1 << 1);
#endif /* CONFIG_UART */

  adc_init();
  fet_init();

  /* power consumption, disable watchdog */
  wdt_reset();
  MCUSR &= ~(1 << WDRF);
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  WDTCSR = 0x00;

  /* power consumption, disable comparator */
  ACSR = 1 << 7;

#if (CLK_PRESCAL == 256UL)
  clk_set_prescal_max();
#else
#if (CLK_PRESCAL != 1UL)
# error "not implemented"
#endif /* (CLK_PRESCAL != 1UL) */
#endif /* (CLK_PRESCAL == 256UL) */

  set_sleep_mode(SLEEP_MODE_IDLE);

  for (i = 0; i != (THRESH_COUNT - 1); ++i) v[i] = adc_read();

  timer_init(timer_ms100_to_counter(10));

  while (1)
  {
    /* enable irq and put in sleep mode. disabled when awake. */
    sleep_enable();
    sleep_bod_disable();
    sei();
    sleep_cpu();
    sleep_disable();
    cli();

    if (timer_irq == 0) continue ;

    v[i] = adc_read();

#ifdef CONFIG_UART
    uart_write(uint16_to_string(ADC_THRESH), 4);
    UART_WRITE_STRING(" ");
    uart_write(uint16_to_string(v[i]), 4);
    uart_write_rn();
#endif /* CONFIG_UART */

    k = 0;
    for (j = 0; j != THRESH_COUNT; ++j)
    {
      if (v[(i + j) & (THRESH_COUNT - 1)] < ADC_THRESH) --k;
      else ++k;
    }

    if (k == -THRESH_COUNT)
    {
      fet_disable();
#ifdef CONFIG_UART
      UART_WRITE_STRING("fet_disable");
      uart_write_rn();
#endif /* CONFIG_UART */
    }
    else if (k == THRESH_COUNT)
    {
      fet_enable();
#ifdef CONFIG_UART
      UART_WRITE_STRING("fet_enable");
      uart_write_rn();
#endif /* CONFIG_UART */
    }

    /* increment modulo THRESH_COUNT */
    i = (i + 1) & (THRESH_COUNT - 1);
  }

  return 0;
}
