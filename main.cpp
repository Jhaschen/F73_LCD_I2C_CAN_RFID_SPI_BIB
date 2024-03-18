#include <avr/io.h>
#include <util/delay.h>

#include <stdio.h>
#include "ATMega32_utility_bib.h"
#include "rfid.h"
#include "can.h"
#include "lcd.h"


// UID S50 Mifare 1K Chip auslesen
uint8_t mfrc522_get_card_serial(uint8_t * serial_out)
{

    uint8_t status;
    uint8_t i;
    uint8_t serNumCheck=0;
    uint32_t unLen;
    
	mfrc522_write(BitFramingReg, 0x00);		//TxLastBists = BitFramingReg[2..0]
 
    serial_out[0] = PICC_ANTICOLL;
    serial_out[1] = 0x20;
    status = mfrc522_to_card(Transceive_CMD, serial_out, 2, serial_out, &unLen);

    if (status == CARD_FOUND)
	{
		//Check card serial number
		for (i=0; i<4; i++)
		{   
		 	serNumCheck ^= serial_out[i];
		}
		if (serNumCheck != serial_out[i])
		{   
			status = ERROR;    
		}
    }
    return status;
}




int main ()
{
	SET_BIT(LED_DDR,0);			// LED-Port LED 0: output
	SET_LED(LED_PORT,0);			// LED0  aus
	
	//USART UART(8,0,1,9600);	// USART init 8 Zeichenbits , keien Paritätsbits , 1 Stoppbit, 9600 Zeichen pro Sekunde
	 lcd_init(LCD_DISP_ON);    // init lcd and turn on
  
  lcd_puts("Hello World");  // put string from RAM to display (TEXTMODE) or buffer (GRAPHICMODE)
  lcd_gotoxy(0,1);          // set cursor to first column at line 1
  lcd_puts_p(PSTR("String from flash"));  // puts string form flash to display (TEXTMODE) or buffer (GRAPHICMODE)
  lcd_gotoxy(5,2);          // set cursor to 2 column at line 3
  lcd_puts_p(PSTR("Moin!"));  // puts string form flash to display
#if defined GRAPHICMODE   
  lcd_drawCircle(48,48,10,WHITE); // draw circle to buffer white lines
  lcd_display();                 // send buffer to display
#endif
	
char buffer[100];
	for (;;)
	{
	
	for (uint8_t i=0;i<255;i++){
     lcd_gotoxy(0,3);
     sprintf(buffer,"%d",i);
    lcd_puts(buffer);
    _delay_ms(200); 
    }	
	TGL_BIT(LED_PORT,0);
	_delay_ms(250);

	}
	return 0;
}

