#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/rtc.h>

#include "rtc.h"

//Настройка тактирования часов
void RTC_init(void){
	// Включим LSI
	RCC_CSR |= RCC_CSR_LSION;
	// Ждем готовности LSI
	while ( !(RCC_CSR & RCC_CSR_LSIRDY) ){}

	// Включим тактирование модуля PWR
	//RCC_APB1ENR |= RCC_APB1ENR_PWREN;
	// После сброса регистры RTC находятся в защищенном домене 
	// Для разрешения записи необходимо поставить флаг в PWR_CR
	PWR_CR |= PWR_CR_DBP;
	
  	// Выберем источник тактирования RTC( от низкоскоростного внутреннего источника LSI(40 kHz) )
	RCC_BDCR |= RCC_BDCR_RTCSEL_LSI;
	// Включим тактиование RTC
	RCC_BDCR |= RCC_BDCR_RTCEN;
	
	// Отключим защиту от записи
	RTC_WPR = 0xCA;
	RTC_WPR = 0x53;

	// Войдем в режим редактирования
	RTC_ISR |= RTC_ISR_INIT;
	// Ждем подтверждения входа в режим редактирования
	while( !(RTC_ISR & RTC_ISR_INITF) ){};	

	// Установим асинхронный предтелитель на 100(99+1).
	RTC_PRER = (uint32_t)(99 << 16);
	// Установим синхронный предделитель на 400(399+1).
	RTC_PRER |= (uint32_t)399;
	
	// Выйти из режима редактирования
	RTC_ISR &= ~(RTC_ISR_INIT);
	
	// Включим защиту от записи
	RTC_WPR = 0xFF;
}

// первоначальная установка часов
void RTC_sets(RTC_struct *value){
	uint32_t tr, dr;
	// Отключим защиту от записи
	RTC_WPR = 0xCA;
	RTC_WPR = 0x53;

	// Войдем в режим редактирования
	RTC_ISR |= RTC_ISR_INIT;
	// Ждем подтверждения входа в режим редактирования
	while( !(RTC_ISR & RTC_ISR_INITF) ){};	
	// Установим время
	tr = ( ((uint32_t)(value->hour) << 16) | ((uint32_t)(value->minutes) << 8) | ((uint32_t)value->seconds) );
	RTC_TR = tr & RTC_TR_MASK;
	// Установим дату
	dr = (uint32_t)(value->year) << 16  | (uint32_t)(value->week) << 13 | (uint32_t)(value->month) << 8 | (uint32_t)(value->day);
	RTC_DR = dr & RTC_DR_MASK;
		
	// Выйти из режима редактирования
	RTC_ISR &= ~(RTC_ISR_INIT); 
		
	// Включим защиту от записи
	RTC_WPR = 0xFF;
}

// Установка времени и даты часов реального времени. Все значения в BCD.
void RTC_change(RTC_struct *value)
{
	uint32_t tr, dr;
	
	// Отключим защиту от записи
	RTC_WPR = 0xCA;
	RTC_WPR = 0x53;

 	// Войдем в режим редактирования
	RTC_ISR |= RTC_ISR_INIT;
	
	// Ждем подтверждения входа в режим редактирования
	while( !(RTC_ISR & RTC_ISR_INITF) ){};	
	
	// Установим время
	tr = ( ((uint32_t)(value->hour) << 16) | ((uint32_t)(value->minutes) << 8) | ((uint32_t)value->seconds) );
	RTC_TR = tr & RTC_TR_MASK;
	// Установим дату
	dr = (uint32_t)(value->year) << 16  | (uint32_t)(value->week) << 13 | (uint32_t)(value->month) << 8 | (uint32_t)(value->day);
	RTC_DR = dr & RTC_DR_MASK;
	
 	// Выйти из режима редактирования
	RTC_ISR &= ~(RTC_ISR_INIT); 
	
	// Включим защиту от записи
	RTC_WPR = 0xFF;
	
	RTC_BKPXR(0) |= RTC_SET; // Значит установили часы
}

void RTC_alarm(RTC_struct *value, uint8_t msk){
	uint32_t alr;
	
	// Отключим защиту от записи
	RTC_WPR = 0xCA;
	RTC_WPR = 0x53;
	
	RTC_CR |= RTC_CR_ALRAIE; // Прерывание по alarm. Доступ только в защищенной зоне.
	
	// Отключить ALARM
	RTC_CR &= ~(RTC_CR_ALRAE); 
	// Ждем подтверждения входа в режим редактирования
	while( !(RTC_ISR & RTC_ISR_ALRAWF) ){};
	
	RTC_ALRMAR = ( (msk & 0x01) << 7 ); 	// MSK1
	RTC_ALRMAR |= ( (msk & 0x02) << 14 ); 	// MSK2
	RTC_ALRMAR |= ( (msk & 0x04) << 21 ); 	// MSK3
	RTC_ALRMAR |= ( (msk & 0x08) << 28 ); 	// MSK4

	
	alr = ( (uint32_t)(value->day) << 24) | ((uint32_t)(value->hour) << 16) 
		| ((uint32_t)(value->minutes) << 8) | ((uint32_t)(value->seconds)); 
	// установили время срабатывания
	RTC_ALRMAR |= alr;
	
	// Включить ALARM
	RTC_CR |= RTC_CR_ALRAE;
	// Включим защиту от записи
	RTC_WPR = 0xFF;
}

// Считывание значений часов реального времени. Все значения в BCD.
void RTC_get(RTC_struct * value){
	uint32_t tr, dr;
	tr = RTC_TR;
	dr = RTC_DR;
	
	// Отдадим время
	value->hour = (uint8_t)(tr >> 16) & 0x3F; 	// шесть бит
	value->minutes = (uint8_t)(tr >> 8) & 0x7F; // семь бит
	value->seconds = (uint8_t)(tr) & 0x7F;		// семь бит
	// Отдадим дату
	value->year = (uint8_t)(dr >> 16);
	value->week = (uint8_t)(dr >> 13) & 0x07;	// Три бита
	value->month = (uint8_t)(dr >> 8) & 0x1F; 	// Пять бит
	value->day = (uint8_t)(dr) & 0x3F; 			// Шесть бит
	
}

// Перевод из двоичного в BCD(двузначное).
uint8_t RTC_ByteToBcd2(uint8_t Value){
	uint8_t bcdhigh = 0;
	while (Value >= 10)	{
		bcdhigh++;
		Value -= 10;
	}
	return  ((uint8_t)(bcdhigh << 4) | Value);
}

// Перевод из BCD(двузначное) в двоичную форму
uint8_t RTC_Bcd2ToByte(uint8_t Value){
	uint8_t tmp = 0;
	tmp = ((uint8_t)(Value & (uint8_t)0xF0) >> 0x4) * 10;
	return (tmp + (Value & 0x0F));
}

/* Получить старшую часть (десятки) BCD 
uint8_t RTC_Bcd_elder(uint8_t value){
	return (value >> 4);
}

// Получить младшую часть (единицы) BCD
uint8_t RTC_Bcd_under(uint8_t value){
	return (value & (uint8_t)0x0F);
}
*/