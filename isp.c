/*
 * isp.c - part of USBasp
 *
 * Autor..........: Thomas Fischl <tfischl@gmx.de>
 * Description....: Provides functions for communication/programming
 *                  over ISP interface
 * Licence........: GNU GPL v2 (see Readme.txt)
 * Creation Date..: 2005-02-23
 * Last change....: 2010-01-19
 */

#include <avr/io.h>
#include "isp.h"
#include "clock.h"
#include <util/delay.h> 
#include "usbasp.h"
#include <avr/pgmspace.h> //  дл€ авто-подбора в программной пам€ти
#include <avr/interrupt.h>
#include <stddef.h>

extern uchar prog_sck;
uchar (*ispTransmit)(uchar) = NULL;

uint8_t last_success_speed = USBASP_ISP_SCK_AUTO;


// явный массив скоростей от —јћќ… Ѕџ—“–ќ… к —јћќ… ћ≈ƒЋ≈ЌЌќ…
// ѕор€док ¬ј∆≈Ќ - от быстрой к медленной!
static const uchar isp_retry_speeds[] PROGMEM = {

    USBASP_ISP_SCK_3000,   // 3.0 MHz
    USBASP_ISP_SCK_1500,   // 1.5 MHz
    USBASP_ISP_SCK_750,    // 750 kHz
    USBASP_ISP_SCK_375,    // 375 kHz
    USBASP_ISP_SCK_187_5,  // 187.5 kHz
    USBASP_ISP_SCK_93_75,  // 93.75 kHz
    USBASP_ISP_SCK_32,     // 32 kHz
    USBASP_ISP_SCK_16,     // 16 kHz
    USBASP_ISP_SCK_8,      // 8 kHz
    USBASP_ISP_SCK_4,      // 4 kHz
    USBASP_ISP_SCK_2,      // 2 kHz
    USBASP_ISP_SCK_1,      // 1 kHz
    USBASP_ISP_SCK_0_5     // 0.5 kHz
};

#define ISP_SPEED_CNT (sizeof(isp_retry_speeds)/sizeof(isp_retry_speeds[0]))
// ћакрос дл€ удобного чтени€ из массива скоростей
#define GET_SPEED(idx) pgm_read_byte(&isp_retry_speeds[(idx)])

uchar sck_sw_delay;
uchar sck_spcr;
uchar sck_spsr;
uchar isp_hiaddr = 0xFF;   // 0xFF == ЂещЄ не инициализированої

void spiHWenable() {
	SPCR = sck_spcr;
	SPSR = sck_spsr;
}

static inline void spiHWdisable() {
    SPCR = 0;
}

void ispSetSCKOption(uchar option) {
    if (option == USBASP_ISP_SCK_AUTO)
        option = USBASP_ISP_SCK_750;

    if (option >= USBASP_ISP_SCK_93_75) {
        ispTransmit = (uchar (*)(uchar))ispTransmit_hw;
        sck_spsr = 0;
        sck_sw_delay = 1;
        switch (option) {
    	
	case USBASP_ISP_SCK_3000:   // 4.0 MHz
        	sck_spcr = (1 << SPE) | (1 << MSTR);
        	sck_spsr = 0;
        	break;

    	case USBASP_ISP_SCK_1500:   // 2.0 MHz
        	sck_spcr = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
        	sck_spsr = (1 << SPI2X);
        	break;

    	case USBASP_ISP_SCK_750:    // 1.0 MHz
          default:
		sck_spcr = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
        	sck_spsr = 0;
        	break;

    	case USBASP_ISP_SCK_375:    // 500 kHz
           
		sck_spcr = (1 << SPE) | (1 << MSTR) | (1 << SPR1);
        	sck_spsr = (1 << SPI2X);      // /32 > 500 kHz
        	break;

    	case USBASP_ISP_SCK_187_5:  // 250 kHz
        	sck_spcr = (1 << SPE) | (1 << MSTR) | (1 << SPR1);
        	sck_spsr = 0;
        	break;

    	case USBASP_ISP_SCK_93_75:  // 125 kHz
        	sck_spcr = (1 << SPE) | (1 << MSTR) | (1 << SPR1) | (1 << SPR0);
        	sck_spsr = 0;
        	break;
	}
    } else {
        ispTransmit = ispTransmit_sw;
        switch (option) {
            case USBASP_ISP_SCK_32:
                sck_sw_delay = 4;    // ~31.25 к√ц
                break;
            case USBASP_ISP_SCK_16:
                sck_sw_delay = 8;    // ~15.625 к√ц
                break;
            case USBASP_ISP_SCK_8:
                sck_sw_delay = 16;   // ~7.8 к√ц
                break;
            case USBASP_ISP_SCK_4:
                sck_sw_delay = 31;   // ~4.03 к√ц
                break;
            case USBASP_ISP_SCK_2:
                sck_sw_delay = 63;   // ~1.98 к√ц
                break;
            case USBASP_ISP_SCK_1:
                sck_sw_delay = 125;  // 1 к√ц
                break;
            case USBASP_ISP_SCK_0_5:
                sck_sw_delay = 250;  // 500 √ц
                break;
        }
    }
}
void ispDelay() {

	uint8_t starttime = TIMERVALUE;
	while ((uint8_t) (TIMERVALUE - starttime) < sck_sw_delay) {
	}
}

void ispConnect() {

	/* all ISP pins are inputs before */
	/* now set output pins */
	ISP_DDR |= (1 << ISP_RST) | (1 << ISP_SCK) | (1 << ISP_MOSI);
        ISP_DDR &= ~(1 << ISP_MISO); // MISO всегда вход

	/* reset device */
	ISP_OUT &= ~(1 << ISP_RST); /* RST low */
	ISP_OUT &= ~(1 << ISP_SCK); /* SCK low */

	/* positive reset pulse > 2 SCK (target) */
	clockWait(1); /* ~320 µs */
	ISP_OUT |= (1 << ISP_RST); /* RST high */
	clockWait(1);		   /* 320us */
	ISP_OUT &= ~(1 << ISP_RST);/* RST low */

	if (ispTransmit == (uchar (*)(uchar))ispTransmit_hw) {

	spiHWenable();
	}
	
	/* Initial extended address value */
        isp_hiaddr = 0xff;  /* ensure that even 0x00000 causes a write of the extended address byte */
}

void isp25Connect() {
	/* all ISP pins are inputs before */
	/* now set output pins */
	ISP_DDR |= (1 << ISP_RST) | (1 << ISP_SCK) | (1 << ISP_MOSI);
	
	if (ispTransmit == (uchar (*)(uchar))ispTransmit_hw) {
		spiHWenable();
	}
	isp_hiaddr = 0xff;  /* ensure that even 0x00000 causes a write of the extended address byte */
	CS_HI();
}

void ispDisconnect() {

	/* set all ISP pins inputs */
	ISP_DDR &= ~((1 << ISP_RST) | (1 << ISP_SCK) | (1 << ISP_MOSI));
	/* switch pullups off */
	ISP_OUT &= ~((1 << ISP_RST) | (1 << ISP_SCK) | (1 << ISP_MOSI));
        prog_sck = USBASP_ISP_SCK_AUTO;	
}

uchar ispTransmit_sw(uchar send_byte)
{
    uchar rec_byte = 0;

    ISP_OUT &= ~(1 << ISP_SCK);      // SCK = 0

    for (uchar bit = 8; bit; --bit) {
        /* MOSI */
        if (send_byte & 0x80)
            ISP_OUT |=  (1 << ISP_MOSI);
        else
            ISP_OUT &= ~(1 << ISP_MOSI);
        send_byte <<= 1;

        ispDelay();                  // tsetup

        /* rising edge */
        ISP_OUT |= (1 << ISP_SCK);
        ispDelay();                  // thold

        /* sample MISO */
        rec_byte = (rec_byte << 1) | ((ISP_IN >> ISP_MISO) & 1);

        /* falling edge + задержка */
        ISP_OUT &= ~(1 << ISP_SCK);
        ispDelay();                  // tlow Ч добавьте!
    }

    return rec_byte;
}

uchar ispTransmit_hw(uchar send_byte) {

	SPDR = send_byte;

	while (!(SPSR & (1 << SPIF)));

	return SPDR;
}

/* ѕопытка войти в режим программировани€ с заданной скоростью */
static uchar tryEnterProgMode(uchar speed)
{
    ispSetSCKOption(speed);

    /* јдаптивные тайминги сброса: длинные дл€ SW ( 32 к√ц), короткие дл€ HW */
    uint8_t pulse = (speed <= USBASP_ISP_SCK_32) ? 15 : 1;
    uint8_t delay = (speed <= USBASP_ISP_SCK_32) ? 250 : 63;

    for (uchar tries = 3; tries > 0; tries--) {
        ISP_OUT |= (1 << ISP_RST);
        clockWait(pulse);
        ISP_OUT &= ~(1 << ISP_RST);
        clockWait(delay);

        ispTransmit(0xAC);
        ispTransmit(0x53);
        uchar check  = ispTransmit(0);
        uchar check2 = ispTransmit(0);

        if (check == 0x53 && check2 == 0x00) {
            return 0; /* успех */
        }
        _delay_ms(5);
    }
    return 1; /* не удалось */
}

uchar ispEnterProgrammingMode(void)
{
    uchar rc;

       /* 1. —охранЄнна€ скорость - Ѕџ—“–џ… ѕ”“№ */
	if (last_success_speed != USBASP_ISP_SCK_AUTO) {
    	rc = tryEnterProgMode(last_success_speed); /* уже содержит адаптивные тайминги */
     	if (rc == 0) {
          prog_sck = last_success_speed;
          return 0;
    	 }
    	last_success_speed = USBASP_ISP_SCK_AUTO;
	}

    /* 2. –учна€ скорость (-B) */
    if (prog_sck != USBASP_ISP_SCK_AUTO) {
        return tryEnterProgMode(prog_sck);
    }

    /* 3. јвтоподбор - только если не указана €вна€ скорость */
    for (uchar i = 0; i < ISP_SPEED_CNT; i++) {
        uchar speed = GET_SPEED(i);
        rc = tryEnterProgMode(speed);
        if (rc == 0) {
            prog_sck = speed;
            last_success_speed = speed;
            return 0;
        }
    }

    /* ничего не подошло */
    prog_sck = USBASP_ISP_SCK_AUTO;
    ispSetSCKOption(USBASP_ISP_SCK_375);
    return 1;
}

void ispUpdateExtended(uint32_t address)
{
    if (address < EXTADDR_BLOCK || address >= FLASH_MAX_BYTES)
        return;

    uint8_t curr_hiaddr = (uint8_t)(address >> 17);

    if (curr_hiaddr == isp_hiaddr)
        return;

    isp_hiaddr = curr_hiaddr;

    // ќтправка команды стандартными вызовами
    ispTransmit(0x4D);
    ispTransmit(0x00);
    ispTransmit(isp_hiaddr);
    ispTransmit(0x00);
}

uchar ispReadFlashRaw(uint32_t address)
{
    ispTransmit(0x20 | ((address & 1) << 3));
    ispTransmit(address >> 9);
    ispTransmit(address >> 1);
    return ispTransmit(0);
}

uchar ispReadFlash(uint32_t address)
{
    ispUpdateExtended(address);
    return ispReadFlashRaw(address);
}

uchar ispWriteFlash(uint32_t address, uint8_t data, uint8_t pollmode)
{
    ispUpdateExtended(address);

    /* ---------- 1. —обственно загрузка байта в буфер страницы ---------- */
    ispTransmit(0x40 | ((address & 1) << 3));
    ispTransmit(address >> 9);
    ispTransmit(address >> 1);
    ispTransmit(data);

    /* если страница ещЄ не полна Ц выходим сразу */
    if (!pollmode) return 0;

    /* ---------- 2. ѕроверка Ђуже 0xFFї (только дл€ стирани€) ---------- */
    if (data == 0xFF && ispReadFlash(address) == 0xFF) return 0;

    /* ---------- 3. Poll готовности ---------- */
    for (uint8_t t = 30; t; --t) {
        clockWait(t > 20 ? 1 : t > 10 ? 2 : 4);
        if (ispReadFlash(address) == data) return 0;
    }
    return 1;                 // timeout
}

uchar ispFlushPage(uint32_t address) {

    ispUpdateExtended(address);
    
    ispTransmit(0x4C);
    ispTransmit(address >> 9);
    ispTransmit(address >> 1);
    ispTransmit(0);

    /* ¬сегда провер€ем запись */
    for (uint8_t t = 25; t > 0; t--) {
        clockWait(t > 15 ? 1 : (t > 5 ? 2 : 4));
        if (ispReadFlash(address) != 0xFF) {
            return 0; // ”спех
        }
    }
    
    return 1; // ќшибка
}

uchar ispReadEEPROM(unsigned int address) {

    ispTransmit(0xA0);
    ispTransmit(address >> 8);
    ispTransmit(address & 0xFF);
    return ispTransmit(0);
    
}

uchar ispWriteEEPROM(unsigned int address, uchar data) {

    ispTransmit(0xC0);
    ispTransmit(address >> 8);    // —тарший байт
    ispTransmit(address & 0xFF);  // ћладший байт  
    ispTransmit(data);
    // Typical Wait Delay Before Writing
    // tWD_EEPROM min 3.6ms
    clockWait(11); // wait 3,52 ms
    return 0;

}

