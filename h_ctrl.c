/*
 * Author: Nikolay Belov
 *
 * Tools: Linux Mint, gcc-arm-none-eabi, OpenOCD, LIBOPENCM3, Sublime Text.
 *
*/

#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/f0/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/f0/dma.h>
#include <libopencm3/stm32/f0/usart.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/f0/nvic.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/f0/timer.h>
#include <libopencm3/stm32/adc.h>

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
static void dma_write(void);
static void adc_setup(void);
static void tim16_setup(void);

uint64_t millis(void);
void delay(uint64_t duration);

//static void adc_calc(void);
static void make_pack(void);
static uint16_t am2320_recv(am2320_s * ds, uint16_t s_pin);

uint16_t get_us_value(bool state, uint16_t cnt, uint16_t pin);

char * itoa_m(int val, int base);


#define AM2320_PORT GPIOA
#define AM2320_PIN1 GPIO5
#define AM2320_PIN2 GPIO6
#define PACK_SZ	20

am2320_s ds_air = {{0}, {0}};
am2320_s ds_bat = {{0}, {0}};
static volatile am2320_s ds_tmp = {{0}, {0}};

char sndbuf[PACK_SZ + 4];
char rxbuff[PACK_SZ + 4];
char cmdbuff[PACK_SZ + 4];
uint8_t channel_array[] = {3, 7};


uint8_t rxindex = 0;
uint8_t flag_cmd = 0;
uint8_t transmit = 0;
uint8_t ble_connected = 0;
uint16_t pwm_count = 0;
uint8_t	adc_scanning = 0;  
uint8_t adc_error = 0;  
uint8_t adc_new_val = 0;

int16_t t_air = 0;
int16_t t_bat = 0;
//int16_t t_chip = 0;
int16_t pressure = 0;
int16_t pressure1 = 0;
int16_t vcc = 0;
int16_t vcc1 = 0;

uint16_t res = 0;
static volatile uint16_t Tbe = 0;
static volatile uint16_t Tgo = 0;
static volatile uint16_t Trel = 0;
static volatile uint16_t Treh = 0;

static volatile uint16_t test1 = 0;
static volatile uint16_t test2 = 0;
static volatile uint16_t test3 = 0;

int main(void)
{
    clock_setup();
    systick_setup();
    gpio_setup();
    gpio_set(AM2320_PORT, AM2320_PIN1);
    gpio_set(AM2320_PORT, AM2320_PIN2);
    adc_setup();
    tim16_setup();
    usart_setup();
    delay(500);

    while (1)
    {
    	if (adc_scanning == 0)
    	{
    		adc_scanning = 1;
            //channel_array[0] = 3;
            //adc_power_off(ADC1);
            //adc_set_regular_sequence(ADC1, 1, channel_array);
            //adc_power_on(ADC1);
            //delay(20);
    		adc_start_conversion_regular(ADC1);
            while (!(adc_eoc(ADC1)));
            vcc = adc_read_regular(ADC1);
            //delay(20);
            //adc_start_conversion_regular(ADC1);
            while (!(adc_eoc(ADC1)));
            pressure = adc_read_regular(ADC1);
            //delay(20);
            /*
            channel_array[0] = 7;
            adc_power_off(ADC1);
            adc_set_regular_sequence(ADC1, 1, channel_array);
            adc_power_on(ADC1);
			delay(20);
            adc_start_conversion_regular(ADC1);
            while (!(adc_eoc(ADC1)));
            pressure1 = adc_read_regular(ADC1);
            delay(20);
            adc_start_conversion_regular(ADC1);
            while (!(adc_eoc(ADC1)));
            pressure = adc_read_regular(ADC1);
            */
            adc_scanning = 0;
            delay(20);
    	}
    	else
    	{
    		if ((adc_new_val == 1) && ble_connected)
    		{
    			//adc_calc();
    			make_pack();
    			while (transmit) {};
    			dma_write();
    		}
    	}

        res = am2320_recv(&ds_air, AM2320_PIN1);
        res = am2320_recv(&ds_bat, AM2320_PIN2);
        delay(200);
	}
}

uint16_t get_us_value(bool state, uint16_t cnt, uint16_t pin)
{
    uint16_t tb = timer_get_counter(TIM16);
    uint16_t d = 0;
    while (((gpio_get(AM2320_PORT, pin) != 0) == state) && (d < cnt))
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

static uint16_t am2320_recv(am2320_s * ds, uint16_t s_pin)
{   
    uint8_t buf[5];
    uint16_t tcnt = 0; 
    uint16_t result = 0;

    // Поднимем ногу датчика к питанию для включения режима 
    gpio_set(AM2320_PORT, s_pin);
    delay(100); 
    // прижимаем пин к земле на 5mS - "Tbe"
    gpio_clear(AM2320_PORT, s_pin);    
    delay(5); 
    // отпустим пин 
    gpio_set(AM2320_PORT, s_pin);
    // ждем появления нуля  - "Tgo"
    tcnt = get_us_value(true, 200, s_pin); // 20..200 uS
    Tgo = tcnt;
    if (tcnt < 200) // дождались 0 от AM2320 - Trel началось
    {
        tcnt = get_us_value(false, 200, s_pin); // ждем окончания Trel: 75..85 uS
        Trel = tcnt;
        if (tcnt < 200) // Trel закончилось, дождались Treh
        {
            // теперь ждем пока Treh закончится
            tcnt = get_us_value(true, 200, s_pin);
            Treh = tcnt;
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



/*
static void adc_calc(void)
{
	t_air = (adc_buff[1] + adc_buff[2]) >> 1;
	t_bat = (adc_buff[4] + adc_buff[5]) >> 1;
	pressure = (adc_buff[7] + adc_buff[8]) >> 1;
	vcc = (adc_buff[10] + adc_buff[11]) >> 1;
	adc_new_val = 0;
}
*/
static void make_pack(void)
{
	strcpy(sndbuf, "A");
	strcat(sndbuf, itoa_m(ds_air.temper, 10));
	strcat(sndbuf, "B");
	strcat(sndbuf, itoa_m(ds_bat.temper, 10));
	strcat(sndbuf, "P");
	strcat(sndbuf, itoa_m(pressure, 10));
	strcat(sndbuf, "V");
	strcat(sndbuf, itoa_m(vcc, 10));
	strcat(sndbuf, "\n");
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
    //  Switch
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0);
    // PWRC
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1);
    // AM2320 - T_Air, H_Air
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);
    gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_MED, GPIO5);
    // AM2320 - T_Bat, H_Bat
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO6);
    gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_MED, GPIO6);

    // STAT input
    nvic_enable_irq(NVIC_EXTI0_1_IRQ);
    gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO1);
    exti_select_source(EXTI1, GPIOB);
    exti_set_trigger(EXTI1, EXTI_TRIGGER_BOTH);  // falling edge interrupt
    exti_enable_request(EXTI1);
    // PWM input
    nvic_enable_irq(NVIC_EXTI4_15_IRQ);
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO4);
    exti_select_source(EXTI4, GPIOA);
    exti_set_trigger(EXTI4, EXTI_TRIGGER_RISING);  // falling edge interrupt
    exti_enable_request(EXTI4);

    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO7);
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO3);

}

// measuring ir signal period
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

void dma1_channel1_isr(void)
{
    if ((DMA1_ISR & DMA_IFCR_CTCIF1) != 0) // Tranfer complete flag
    {
        DMA1_IFCR |= DMA_IFCR_CTCIF1;
		adc_scanning = 0;
		adc_new_val = 1;  
    }
    
    if ((DMA1_ISR & DMA_IFCR_CTEIF1) != 0) // Tranfer error flag
    {
        DMA1_IFCR |= DMA_IFCR_CTEIF1;
		adc_scanning = 0;  
		adc_error = 1;  
    }
}

static void usart_setup(void) {
    nvic_set_priority(NVIC_DMA1_CHANNEL2_3_DMA2_CHANNEL1_2_IRQ, 0);
    nvic_enable_irq(NVIC_DMA1_CHANNEL2_3_DMA2_CHANNEL1_2_IRQ);

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

static void dma_write(void)
{
    // Using channel 2 for USART1_TX
    //Reset DMA channel
    dma_channel_reset(DMA1, DMA_CHANNEL2);

    dma_set_peripheral_address(DMA1, DMA_CHANNEL2, (uint32_t)&USART1_TDR);
    dma_set_memory_address(DMA1, DMA_CHANNEL2, (uint32_t)sndbuf);
    dma_set_number_of_data(DMA1, DMA_CHANNEL2, strlen(sndbuf));
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
    transmit = 0;
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
     if (exti_get_flag_status(EXTI1) != 0)
    {
        exti_reset_request(EXTI1);
        if ( gpio_get(GPIOB, GPIO1) == 0 )
        {
            // BLE connection is off
            ble_connected = 0;
        }
        else
        {
        	ble_connected = 1;
        }
    }
}

void exti4_15_isr(void)
{
     if (exti_get_flag_status(EXTI4) != 0)
    {
        exti_reset_request(EXTI4);
        if ( gpio_get(GPIOA, GPIO4) == 1 )
        {
            pwm_count++;
        }
    }
}
