/*
  Thanks to denigrim article (https://denigrim.livejournal.com/34580.html)
*/
#ifndef _MRTC_H_
#define _MRTC_H_

#include <stdint.h>

#define RTC_TR_MASK 0x007F7F7F
#define RTC_DR_MASK 0x00FFFF3F
#define RTC_SET 0x01


typedef struct {
     uint8_t year;
     uint8_t month;
     uint8_t week;
     uint8_t day;
     uint8_t hour;
     uint8_t minutes;
     uint8_t seconds;
} RTC_struct;


void RTC_init(void);
void RTC_set(RTC_struct *value);
void RTC_sets(RTC_struct *value);
void RTC_get(RTC_struct * value);
void RTC_change(RTC_struct *value);
void RTC_alarm(RTC_struct *value, uint8_t msk);

uint8_t RTC_ByteToBcd2(uint8_t Value);
uint8_t RTC_Bcd2ToByte(uint8_t Value);
//uint8_t RTC_Bcd_elder(uint8_t value);
//uint8_t RTC_Bcd_under(uint8_t value);


#endif