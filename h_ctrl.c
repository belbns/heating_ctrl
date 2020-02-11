/*
 * Author: Nikolay Belov
 *
 * Tools: Linux Mint, gcc-arm-none-eabi, OpenOCD, LIBOPENCM3, Sublime Text.
 *

    Через BLE передается 2 пакета
    1-й:    "Ann:-ttt:hhh:AP\n" - атмосферные параметры (в комнате),
                где nn - счетчик пакетов (0..99),
                ttt - температура,
                hhh - влажность,
                A[PTB]  - наличие тревоги по давлению, температуре, разряду батареи
                или NN - отсутствие проблем.
    2-й     "Bnn:-ttt:pppp:vvvv\n" - парамтры отопления,
                где nn - счетчик пакетов (тот же),
                ttt - температура,
                ppp - давление,
                vvvv - напряжение питания
*/

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/f0/nvic.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/dbgmcu.h>
#include <libopencm3/../libopencmsis/core_cm3.h>
#include <libopencm3/cm3/scb.h>

#include "rtc.h"

#define SENSOR_PORT GPIOA
#define AM2320_PIN1 GPIO5
#define AM2320_PIN2 GPIO6
#define PRESS_PIN   GPIO7
#define SPEAKER_PIN GPIO4

#define PWRC_PORT   GPIOA
#define PWRC_PIN    GPIO1 // 0 - для ввода AT команд в режиме соединения
#define STAT_PORT   GPIOB
#define STAT_PIN    GPIO1 // 1 - connected

#define PACK_SZ 24

#define TEMP_ALARM 150        // 15 градусов и ниже - авария
#define PRESS_ALARM 646       // 1 bar (при Vcc= 2.8..3V) и ниже - авария
#define VBAT_ALARM 3220       // 2.6V

typedef struct {
    union {
        uint16_t humidity;  // влажность
        uint8_t hum[2];
    };
    union {
        uint16_t temper;  // температура
        uint8_t temp[2];
    };
} am2320_s;


static void clock_setup(void);
static void systick_setup(void);
static void gpio_setup(void);
static void usart_setup(void);
static void dma_write(char * buf);
static void adc_setup(void);
static void tim14_setup(void);
static void tim16_setup(void);
static void rtc_isr_setup(void);

uint64_t millis(void);
void delay(uint64_t duration);
uint16_t get_us_value(bool state, uint16_t cnt, uint16_t pin);
char * itoa_m(int val, int base);

void make_pack(void);
uint16_t am2320_recv(am2320_s * ds, uint16_t s_pin);
void set_new_alarm(void);
void send_pack(char * sbuf);

// Команды BLE JDY-16
// пробуждение по соединению
const char ble_to_sleep[] = "AT+STARTEN1\r\n"; // - default
 // Broadcast interval = 500mS (0 - 100, 9 - 1000)
const char ble_bc_interval[] = "AT+ADVIN4\r\n"; // ADVIN2(300mS) - default 
// частота PWM (50-25KHz, 1000Hz - default);
const char ble_pwm_freq[] = "AT+PWMFRE1000\r\n";
// заполнение PWM (0-255, 10 - default)
const char ble_pwm2_pus[] = "AT+PWM2PUS10\r\n";
// включение/выключение PWM
const char ble_pwm_on[] = "AT+PWMOPEN1\r\n";
const char ble_pwm_off[] = "AT+PWMOPEN0\r\n";
const char ble_dis[] = "AT+DISC\r\n";

uint8_t channel_array[] = {7, 18};  // для АЦП: Pressure sensor, VBAT

am2320_s ds_air = {.humidity = 0, .temper =0};
am2320_s ds_bat = {.humidity = 0, .temper =0};
am2320_s ds_tmp = {.humidity = 0, .temper =0};

RTC_struct rtc_params = {.year = 20, .month = 1, .week = 3, .day = 1, 
                            .hour = 0, .minutes = 0, .seconds = 0};

char airbuf[PACK_SZ];
char batbuf[PACK_SZ];
char rxbuff[PACK_SZ];
char cmdbuff[PACK_SZ];

uint8_t pack_count = 0;

uint8_t rxindex = 0;
uint8_t flag_cmd = 0;
bool transmit = false;
bool ble_connected = false;
bool flag_alarm = 0;
uint16_t pwm_count = 0;

static volatile int16_t t_air = 0;
static volatile int16_t t_bat = 0;
static volatile int16_t t_chip = 0;
static volatile int16_t pressure = 0;
static volatile int16_t pressure1 = 0;
static volatile int16_t vcc = 0;
static volatile int16_t vbat = 0;
static volatile int16_t vcc1 = 0;

uint16_t res = 0;

int main(void)
{
    uint64_t ms1 = 0;
    uint64_t ms2 = 0;

    clock_setup();
    systick_setup();
    gpio_setup();
    gpio_set(SENSOR_PORT, AM2320_PIN1);
    gpio_set(SENSOR_PORT, AM2320_PIN2);
    gpio_set(GPIOA, GPIO1); // PWRC
    adc_setup();
    tim14_setup();
    tim16_setup();
    usart_setup();
    RTC_init();
    /*
    if( !(RTC_ISR & RTC_ISR_INITS) )  // Часы не настроены
    {
        RTC_sets(&rtc_params);
    }
    */
    RTC_sets(&rtc_params);
    rtc_isr_setup();

    gpio_set(GPIOA, GPIO2); // питание на датчики
	am2320_recv(&ds_tmp, AM2320_PIN1);
    am2320_recv(&ds_tmp, AM2320_PIN2);

    // задержка для программатора
    for (uint8_t i = 0; i < 5; i++)
    {
    	gpio_set(GPIOA, GPIO3); // LED
    	delay(200);
    	gpio_clear(GPIOA, GPIO3);
    	delay(200);
    }
    make_pack();
    set_new_alarm();

    while (1)
    {
        gpio_set(GPIOA, GPIO2); // питание на датчики
        ms2 = ms1 = millis();
        gpio_set(GPIOA, GPIO3); // LED

        // разрешаем прерывания
        nvic_enable_irq(NVIC_EXTI0_1_IRQ);
        nvic_enable_irq(NVIC_DMA1_CHANNEL2_3_DMA2_CHANNEL1_2_IRQ);
        nvic_enable_irq(NVIC_USART1_IRQ);

        if (gpio_get(GPIOB, GPIO1) != 0) // если разбудил BLE - передаем имеющийся пакет
        {
            ble_connected = true;
            // После пробуждения по STAT надо выждать 1 секунду!
            delay(1000);
            uint8_t nn = 4;
            while ((nn-- > 0) && (gpio_get(GPIOB, GPIO1) != 0)) // передаем до 4-х раз
            {
                if (flag_alarm) // beep
                {
                    timer_enable_counter(TIM14);
                }
                send_pack(batbuf);
                delay(250);

                timer_disable_counter(TIM14);

                send_pack(airbuf);
                delay(250);
            }
        }
        else                            // разбудил будильник RTC
        {
            ble_connected = false;
            while ((ms2 - ms1) < 100) // прошло ли 100 мС после подачи питания на датчики?
            {
                ms2 = millis();
            }
            // читаем предварительно датчики AM2320 - эти данные не используются
            am2320_recv(&ds_tmp, AM2320_PIN1);
            am2320_recv(&ds_tmp, AM2320_PIN2);
            ms1 = ms2;
            // реальные значения надо прочитать через 2 секунды
            // пока ждем, проверяем не произошло ли соединение по BLE
            delay(200);
            while ((ms2 - ms1) < 2000)
            {
                ms2 = millis();
                if (gpio_get(GPIOB, GPIO1) != 0)
                {
                    ble_connected = true;
					send_pack(batbuf);
                   	delay(250);
                    send_pack(airbuf);
                    delay(250);
                }
            }
            // прошло 2 секунды, проводим измерения
            make_pack();
            if (flag_alarm)
            {
                for (uint8_t i = 0; i < 5; i++)
                {
                    timer_enable_counter(TIM14);
                    delay(1000);
                    timer_disable_counter(TIM14);
                    delay(1000);
                }       
            }

			set_new_alarm();    // заводим будильник на 2 минуты
        }

        while (transmit) {};

        // BLE disconnect
        if (gpio_get(GPIOB, GPIO1) != 0)	// все еще есть соединение
        {
        	gpio_clear(GPIOA, GPIO1); // PWRC=0 - AT mode
            delay(200);
            strcpy(cmdbuff, ble_dis);
            dma_write(cmdbuff);
            delay(500);
            gpio_set(GPIOA, GPIO1); // PWRC
        }
        while (transmit) {};

        gpio_clear(GPIOA, GPIO2);
        gpio_clear(GPIOA, GPIO3);
        gpio_set(GPIOA, GPIO5);
        gpio_set(GPIOA, GPIO6);

        timer_disable_oc_output(TIM14, TIM_OC1);
        timer_disable_break_main_output(TIM14);

        nvic_disable_irq(NVIC_EXTI0_1_IRQ);
        nvic_disable_irq(NVIC_DMA1_CHANNEL2_3_DMA2_CHANNEL1_2_IRQ);
        nvic_disable_irq(NVIC_USART1_IRQ);

        exti_reset_request(EXTI0|EXTI1|EXTI2|EXTI3|EXTI4|EXTI5|EXTI6|EXTI7|
                        EXTI8|EXTI9|EXTI10|EXTI11|EXTI12|EXTI13|EXTI14|EXTI15|
                        EXTI16|EXTI17|EXTI18|EXTI19|EXTI20|EXTI21|EXTI22|EXTI23|
                        EXTI24|EXTI25|EXTI26|EXTI27|EXTI28|EXTI29|EXTI30|EXTI31);        

        //pwr_set_stop_mode();
        PWR_CR &= ~PWR_CR_PDDS;
        //pwr_clear_wakeup_flag();
        PWR_CR |= PWR_CR_CWUF;
        //pwr_voltage_regulator_low_power_in_stop();
        PWR_CR |= PWR_CR_LPDS;
        SCB_SCR |=  (SCB_SCR_SLEEPDEEP_Msk);
        __WFI();

        // Wake up
        clock_setup();
        systick_setup();
        timer_enable_counter(TIM16);
        timer_enable_oc_output(TIM14, TIM_OC1);
        timer_enable_break_main_output(TIM14);        
	}
}

// =====================================================================

void set_new_alarm(void)
{
    RTC_get(&rtc_params);
    uint8_t minutes = RTC_Bcd2ToByte(rtc_params.minutes) + 2;   // через 2 минуты
    if (minutes > 59)
    {
        minutes -= 60;
    }
    rtc_params.minutes = RTC_ByteToBcd2(minutes);

    RTC_alarm(&rtc_params, 0x0D); // только минуты
}

uint16_t get_us_value(bool state, uint16_t cnt, uint16_t pin)
{
    uint16_t tb = timer_get_counter(TIM16);
    uint16_t d = 0;
    while (((gpio_get(SENSOR_PORT, pin) != 0) == state) && (d < cnt))
    {
        uint16_t tc = timer_get_counter(TIM16);
        if (tc < tb)
        {
            d = 0xffff - tb + tc;
        }
        else
        {
            d = tc - tb;
        }
    };

    return d;
}

uint16_t am2320_recv(am2320_s * ds, uint16_t s_pin)
{   
    uint8_t buf[5];
    uint16_t tcnt = 0; 
    uint16_t result = 0;

    // Поднимем ногу датчика к питанию для включения режима 
    gpio_set(SENSOR_PORT, s_pin);
    delay(100); 
    // прижимаем пин к земле на 5mS - "Tbe"
    gpio_clear(SENSOR_PORT, s_pin);    
    delay(5); 
    // отпустим пин 
    gpio_set(SENSOR_PORT, s_pin);
    // ждем появления нуля  - "Tgo"
    tcnt = get_us_value(true, 200, s_pin); // 20..200 uS
    //Tgo = tcnt;
    if (tcnt < 200) // дождались 0 от AM2320 - Trel началось
    {
        tcnt = get_us_value(false, 200, s_pin); // ждем окончания Trel: 75..85 uS
        //Trel = tcnt;
        if (tcnt < 200) // Trel закончилось, дождались Treh
        {
            // теперь ждем пока Treh закончится
            tcnt = get_us_value(true, 200, s_pin);
            //Treh = tcnt;
            if (tcnt < 200) // 75..85 uS ?
            {
                // Treh получен 
                // получать данные - Tlow + Th[0|1] 
                // цикл по размеру массива 
                for (uint8_t b = 0; b < 5; b++)
                { 
                    // получаем 8 бит в очередной байт 
                    buf[b] = 0;
                    for (uint8_t i = 0; i < 8; i++)
                    { 
                        // измеряем Tlow
                        uint16_t cnt_l = get_us_value(false, 100, s_pin); // 48..55 uS
                        // измеряем Th[0|1]
                        uint16_t cnt_h = get_us_value(true, 150, s_pin); // 0: 22..30 uS, 1: 68..75 uS
                        if (cnt_h > cnt_l) // пришла "1" (Th1) - пишем бит в буфер
                        {
                            buf[b] |= (1 << (7 - i));
                        }
                    } 
                } 

                // контрольная сумма 
                if (buf[4] == ((buf[0] + buf[1] + buf[2] + buf[3]) & 0xff))
                {
                    ds->hum[1] = buf[0];
                    ds->hum[0] = buf[1];
                    ds->temp[1] = buf[2];
                    ds->temp[0] = buf[3];
                }
                else
                {
                    result = 14000; // не совпала контрольная сумма
                }
            }
            else
            {
                result = 13000 + tcnt;  // нет Tlow
            }
        }
        else
        {
            result = 12000 + tcnt;  // нет Treh
        }
    }
    else
    {
        result = 11000 + tcnt;  // нет Trel
    }

    return result;
} 

void  send_pack(char * sbuf)
{
    while (transmit) {};
    dma_write(sbuf);
}

void make_pack(void)
{
	// измерения
	am2320_recv(&ds_air, AM2320_PIN1);
    am2320_recv(&ds_bat, AM2320_PIN2);

    adc_start_conversion_regular(ADC1);
    while (!(adc_eoc(ADC1)));
    pressure = adc_read_regular(ADC1);
    while (!(adc_eoc(ADC1)));
    vbat = adc_read_regular(ADC1) * 2;

    if ((pressure < PRESS_ALARM) || (ds_bat.temper < TEMP_ALARM))
    // "Ann:-ttt:hhh:AP\n" - 1-й пакет
    strcpy(airbuf, "A");
    strcat(airbuf, itoa_m(pack_count, 10));
    strcat(airbuf, ":");
    int16_t t = (int16_t)(ds_air.temper & 0x7FFF);
    if ((ds_air.temper & 0x8000) != 0) // отрицательная температура
    {
        t = -t;
    }
    strcat(airbuf, itoa_m(t, 10));
    strcat(airbuf, ":");
    strcat(airbuf, itoa_m(ds_air.humidity, 10));
    strcat(airbuf, ":");
    if (pressure < PRESS_ALARM)
    {
        strcat(airbuf, "AP");
        flag_alarm = true;
    }
    else if (ds_bat.temper < TEMP_ALARM)
    {
        strcat(airbuf, "AT");   
        flag_alarm = true;
    }
    else if (vbat < VBAT_ALARM)
    {
        strcat(airbuf, "AB");   
        flag_alarm = true;
    }
    else
    {
        strcat(airbuf, "NN");   
        flag_alarm = false;
    }
    strcat(airbuf, "\n");

    // "Bnn:-ttt:pppp:vvvv\n" - 2-й пакет
    strcpy(batbuf, "B");
    strcat(batbuf, itoa_m(pack_count, 10));
    strcat(batbuf, ":");
    t = (int16_t)(ds_bat.temper & 0x7FFF);
    if ((ds_bat.temper & 0x8000) != 0) // отрицательная температура
    {
        t = -t;
    }
    strcat(batbuf, itoa_m(t, 10));
    strcat(batbuf, ":");
    strcat(batbuf, itoa_m(pressure, 10));
    strcat(batbuf, ":");
    strcat(batbuf, itoa_m(vbat, 10));
    strcat(batbuf, "\n");

    pack_count++;
    if (pack_count > 99)
    {
        pack_count = 0;
    }
}

// в стандарте C99 функции itoa нет, а sprinf слишком тяжелая
char * itoa_m(int val, int base) {
    static char buf[32] = {0};

    if (val == 0) {
        buf[0] = '0';
        buf[1] = '\0';
        return &buf[0];
    }
    else
    {
        int i = 30;
        unsigned int uval = abs(val);

        for(; uval && i ; --i, uval /= base)
        {
            buf[i] = "0123456789abcdef"[uval % base];
        }
        if (val < 0)
        {
            buf[i--] = '-';
        }

        return &buf[i+1];
    }
}

// =================================================================
static volatile uint64_t _millis = 0;

uint64_t millis(void) {
    return _millis;
}

void sys_tick_handler(void) {
    _millis++;
}

void delay(uint64_t duration) {
    const uint64_t until = millis() + duration;
    while (millis() < until) ;
}

static void clock_setup(void) {
    rcc_clock_setup_in_hsi_out_48mhz();
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_DMA);
    rcc_periph_clock_enable(RCC_ADC);
    rcc_periph_clock_enable(RCC_PWR);

    // смысл тот же, что и включение тактирования AFIO в STM32F103
    // без этого прерывание по PB1 не работает,
    // спасибо andrey_spb (https://radiokot.ru/forum/viewtopic.php?f=59&t=126219)
    RCC_APB2ENR |=RCC_APB2ENR_SYSCFGCOMPEN;// тактирование SYSCFG
}

static void systick_setup(void) {
    // Set the systick clock source to our main clock
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    // Clear the Current Value Register so that we start at 0
    STK_CVR = 0;
    systick_set_reload(rcc_ahb_frequency / 1000 - 1);
    systick_interrupt_enable();
    systick_counter_enable();
}

static void gpio_setup(void) {

    // PWRC
    gpio_set(GPIOA, GPIO1);
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1);
    //  Switch
    gpio_clear(GPIOA, GPIO2);
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO2);
    // LED
    gpio_clear(GPIOA, GPIO3);
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO3);

    // AM2320 - T_Air, H_Air
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);
    gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_MED, GPIO5);
    // AM2320 - T_Bat, H_Bat
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO6);
    gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_MED, GPIO6);

    nvic_enable_irq(NVIC_EXTI0_1_IRQ);
    // PWM input interrupt
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO0);
    exti_select_source(EXTI0, GPIOA);
    exti_set_trigger(EXTI0, EXTI_TRIGGER_RISING);
    exti_enable_request(EXTI0);
    // STAT input interrupt
    gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO1);
    exti_select_source(EXTI1, GPIOB);
    exti_set_trigger(EXTI1, EXTI_TRIGGER_BOTH);  // falling edge interrupt
    exti_enable_request(EXTI1);
    // ADC Pressure
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO7);

}

// Сигнал на динамик
static void tim14_setup(void) {
    rcc_periph_clock_enable(RCC_TIM14);

    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO4);
    gpio_set_af(GPIOA, GPIO_AF4, GPIO4);

    rcc_periph_reset_pulse(RST_TIM14);
    timer_set_mode(TIM14, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_continuous_mode(TIM14);
    timer_set_prescaler(TIM14, 48 - 1); // 1MHz
    timer_disable_preload(TIM14);
    timer_set_period(TIM14, 2500 - 1);  // 400 Hz
    timer_enable_oc_preload(TIM14, TIM_OC1);
    timer_set_oc_mode(TIM14, TIM_OC1, TIM_OCM_PWM1);

    timer_set_oc_polarity_high(TIM14, TIM_OC1);
    timer_set_oc_idle_state_set(TIM14, TIM_OC1);
    timer_set_oc_slow_mode(TIM14, TIM_OC1);
    timer_set_oc_value(TIM14, TIM_OC1, 200);
    timer_enable_oc_output(TIM14, TIM_OC1);
    timer_enable_break_main_output(TIM14);

    //timer_enable_counter(TIM14);
}

// для измерения микросекундных интервалов
static void tim16_setup(void) {
    rcc_periph_clock_enable(RCC_TIM16);

    rcc_periph_reset_pulse(RST_TIM16);
    timer_set_mode(TIM16, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_continuous_mode(TIM16);
    timer_set_prescaler(TIM16, 48 - 1);
    timer_enable_preload(TIM16);
    timer_set_period(TIM16, 0xffff);  // ARR
    timer_enable_counter(TIM16);
}

static void adc_setup(void)
{
    adc_power_off(ADC1);
    adc_set_clk_source(ADC1, ADC_CLKSOURCE_ADC);
    adc_calibrate(ADC1);
    adc_set_operation_mode(ADC1, ADC_MODE_SCAN);
    adc_disable_external_trigger_regular(ADC1);
    adc_set_right_aligned(ADC1);
    adc_enable_temperature_sensor();
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPTIME_071DOT5);
    adc_set_regular_sequence(ADC1, 2, channel_array);
    adc_set_resolution(ADC1, ADC_RESOLUTION_12BIT);
    adc_disable_analog_watchdog(ADC1);
    adc_power_on(ADC1);

    /* Wait for ADC starting up. */
    int i;
    for (i = 0; i < 800000; i++) {    /* Wait a bit. */
        __asm__("nop");
    }
}

static void usart_setup(void) {
    nvic_set_priority(NVIC_DMA1_CHANNEL2_3_DMA2_CHANNEL1_2_IRQ, 0);
    nvic_enable_irq(NVIC_DMA1_CHANNEL2_3_DMA2_CHANNEL1_2_IRQ);
    nvic_enable_irq(NVIC_USART1_IRQ);

    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
    gpio_set_af(GPIOA, GPIO_AF1, GPIO9);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10);
    gpio_set_af(GPIOA, GPIO_AF1, GPIO10);

    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_enable_rx_interrupt(USART1);
    usart_disable_tx_interrupt(USART1);

    usart_enable(USART1);
}

static void dma_write(char * buf)
{
	transmit = true;
    // channel 2 for USART1_TX
    dma_channel_reset(DMA1, DMA_CHANNEL2);

    dma_set_peripheral_address(DMA1, DMA_CHANNEL2, (uint32_t)&USART1_TDR);
    dma_set_memory_address(DMA1, DMA_CHANNEL2, (uint32_t)buf);
    dma_set_number_of_data(DMA1, DMA_CHANNEL2, strlen(buf));
    dma_set_read_from_memory(DMA1, DMA_CHANNEL2);
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL2);
    dma_set_peripheral_size(DMA1, DMA_CHANNEL2, DMA_CCR_PSIZE_8BIT);
    dma_set_memory_size(DMA1, DMA_CHANNEL2, DMA_CCR_MSIZE_8BIT);
    dma_set_priority(DMA1, DMA_CHANNEL2, DMA_CCR_PL_HIGH);
    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL2);

    dma_enable_channel(DMA1, DMA_CHANNEL2);
    usart_enable_tx_dma(USART1);
}

// USART DMA interrupt
void dma1_channel2_3_dma2_channel1_2_isr(void)
{
    if ((DMA1_ISR &DMA_ISR_TCIF2) != 0)
    {
        DMA1_IFCR |= DMA_IFCR_CTCIF2;
    }
    dma_disable_transfer_complete_interrupt(DMA1, DMA_CHANNEL2);
    usart_disable_tx_dma(USART1);
    dma_disable_channel(DMA1, DMA_CHANNEL2);
    transmit = false;
}

// USART1 RX interrupt
void usart1_isr(void)
{
    /* Check if we were called because of RXNE. */
    if (((USART_CR1(USART1) & USART_FLAG_RXNE) != 0) &&
            ((USART_ISR(USART1) & USART_FLAG_RXNE) != 0))
    {
        char data = usart_recv(USART1);
        if (rxindex < PACK_SZ)
        {
        	rxbuff[rxindex++] = data;
        	if ((data == '\n') || (rxindex == PACK_SZ))
        	{
        		rxbuff[rxindex] = '\0';
        		strcpy(cmdbuff, rxbuff);
        		flag_cmd = 1;
        		rxindex = 0;
        		rxbuff[0] = '\0';
        	}
        }
    }
}
    
void exti0_1_isr(void)
{
    if (exti_get_flag_status(EXTI1) != 0) // BLE STAT input
    {
        exti_reset_request(EXTI1);
        if ( gpio_get(GPIOB, GPIO1) != 0 )
        {
            // BLE connection is on
            ble_connected = true;
        }
        else
        {
        	ble_connected = false;
        }
    }
    else if (exti_get_flag_status(EXTI0) != 0) // BLE pwm input
    {
        exti_reset_request(EXTI0);
        if ( gpio_get(GPIOA, GPIO0) == 1 )
        {
            pwm_count++;
        }
    }
}

static void rtc_isr_setup(void)
{
    nvic_enable_irq(NVIC_RTC_IRQ);
    nvic_set_priority(NVIC_RTC_IRQ, 1);

    exti_set_trigger(EXTI17, EXTI_TRIGGER_RISING);
    exti_enable_request(EXTI17);
}

void rtc_isr(void)
{
    if(RTC_ISR & RTC_ISR_ALRAF){ // Сработал Alarm
        
        //GPIOA_ODR ^= GPIO_ODR_6;
        
        RTC_ISR &= ~(RTC_ISR_ALRAF);// Очичтим флаг прерывани ALARM
        //EXTI_PR |= EXTI_PR_PR17; // Сбросим флаг прерывания линии EXTI
        exti_reset_request(EXTI17);
    }
}
