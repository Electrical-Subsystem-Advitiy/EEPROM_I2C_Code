/*
 * I2C_INTERRUPR_XMEGA.c
 *
 * Created: 13-05-2018 19:14:31
 * Author : PRASHANT KURREY
 */ 

#include <avr/io.h>



#include "avr_compiler.h"
#include "twi_master_driver.h"
#include "twi_slave_driver.h"
#include "usart_driver.h"
/*! Defining an example slave address. */
#define SLAVE_ADDRESS    0xA0

/*! Defining number of bytes in buffer. */
#define NUM_BYTES        256

/*! CPU speed 2MHz, BAUDRATE 100kHz and Baudrate Register Settings */
#define CPU_SPEED       2000000
#define BAUDRATE	9600
#define TWI_BAUDSETTING TWI_BAUD(CPU_SPEED, BAUDRATE)


/* Global variables */
TWI_Master_t twiMaster;    /*!< TWI master module. */
TWI_Slave_t twiSlave;      /*!< TWI slave module. */


/*! Define that selects the Usart used in example. */
#define USART USARTC0

/*! USART data struct used in example. */
USART_data_t USART_data;




/*! Buffer with test data to send.*/
uint8_t sendBuffer[NUM_BYTES] = {0b00000000,0b00000111,'a','d','v','i','t','y','I','I','B','o','m','b','a','y'};
uint8_t sending[NUM_BYTES]={'a','d','v','i','t','y','I','I','T','B','o','m','b','a','y'};
uint8_t SendBuffer[NUM_BYTES] ={0b00000010,0b00000000};


void eeprom_page_write(uint8_t page_address, uint8_t *write_data, int length)
{
	uint8_t a[length+2];
	a[0]=page_address;
	a[1]=0b00000000;


	for(int i=2; i<(length+2); i++)
	{
		a[i]=write_data[i-2];
		//UART_TXBuffer_PutByte(&USART_data, write_data[i-2]);
		_delay_ms(100);
	}
	
	TWI_MasterWriteRead(&twiMaster,                                      //added by me
	SLAVE_ADDRESS,
	&a,
	length+2,
	0);
	while (twiMaster.status != TWIM_STATUS_READY) {
		// Wait until transaction is complete.
	}
	
}

void eeprom_page_read(uint8_t page_address, int length)
{
		uint8_t a[2];
		a[0]=page_address;
		a[1]=0b00000000;
		
		TWI_MasterWriteRead(&twiMaster,                                      //added by me
		SLAVE_ADDRESS,
		&a,
		2,
		length);
		
		while (twiMaster.status != TWIM_STATUS_READY) {
			// Wait until transaction is complete.
		}	
}




int main(void)
{
	
	/* This PORT setting is only valid to USARTC0 if other USARTs is used a
	 * different PORT and/or pins are used. */
  	/* PC3 (TXD0) as output. */
	PORTC.DIRSET   = PIN3_bm;
	/* PC2 (RXD0) as input. */
	PORTC.DIRCLR   = PIN2_bm;

	/* Use USARTC0 and initialize buffers. */
	USART_InterruptDriver_Initialize(&USART_data, &USART, USART_DREINTLVL_LO_gc);

	/* USARTC0, 8 Data bits, No Parity, 1 Stop bit. */
	USART_Format_Set(USART_data.usart, USART_CHSIZE_8BIT_gc,
                     USART_PMODE_DISABLED_gc, false);

	/* Enable RXC interrupt. */
	USART_RxdInterruptLevel_Set(USART_data.usart, USART_RXCINTLVL_LO_gc);

	/* Set Baudrate to 9600 bps:
	 * Use the default I/O clock frequency that is 2 MHz.
	 * Do not use the baudrate scale factor
	 *
	 * Baudrate select = (1/(16*(((I/O clock frequency)/Baudrate)-1)
	 *                 = 12
	 */
	USART_Baudrate_Set(&USART, 12 , 0);

	/* Enable both RX and TX. */
	USART_Rx_Enable(USART_data.usart);
	USART_Tx_Enable(USART_data.usart);


	

	/* Initialize TWI master. */
	TWI_MasterInit(&twiMaster,
	               &TWIC,
	               TWI_MASTER_INTLVL_LO_gc,
	               TWI_BAUDSETTING);


	/* Enable LO interrupt level. */
	PMIC.CTRL |= PMIC_LOLVLEN_bm;
	sei();
uint8_t BufPos = 0;

	
while(true){
	
	//TWI_MasterWriteRead(&twiMaster,                                      //added by me
	//SLAVE_ADDRESS,
	//sendBuffer,
	//16,
	//0);
	//while (twiMaster.status != TWIM_STATUS_READY) {
		//// Wait until transaction is complete.
	//}
	//	UART_TXBuffer_PutByte(&USART_data, 'a');
	eeprom_page_write(0b00000010,sending,15);
		_delay_ms(100);
	eeprom_page_read(0b00000010,15);
		
	//TWI_MasterWriteRead(&twiMaster,                                      //added by me
	//SLAVE_ADDRESS,
	//&SendBuffer,
	//2,
	//14);
	//
	//while (twiMaster.status != TWIM_STATUS_READY) {
		//// Wait until transaction is complete.
	//}

	//UART_TXBuffer_PutByte(&USART_data, 'A');	                     // send data
	_delay_ms(500);
	};
	
	}



/*! TWIC Master Interrupt vector. */
ISR(TWIC_TWIM_vect)
{
	TWI_MasterInterruptHandler(&twiMaster);
}


/*! TWIC Slave Interrupt vector. */
ISR(TWIC_TWIS_vect)
{
	TWI_SlaveInterruptHandler(&twiSlave);
	
}





/* ADDED  BY ME




*/

/*! \brief Receive complete interrupt service routine.
 *
 *  Receive complete interrupt service routine.
 *  Calls the common receive complete handler with pointer to the correct USART
 *  as argument.
 */
ISR(USARTC0_RXC_vect)
{
	int receive=0;
	USART_RXComplete(&USART_data);
	if (USART_RXBufferData_Available(&USART_data)) {                                               // modified by  me
	receive = USART_RXBuffer_GetByte(&USART_data);}                  // receive the data      // modified
	UART_TXBuffer_PutByte(&USART_data, receive);	                     // send data
	
}


/*! \brief Data register empty  interrupt service routine.
 *
 *  Data register empty  interrupt service routine.
 *  Calls the common data register empty complete handler with pointer to the
 *  correct USART as argument.
 */
ISR(USARTC0_DRE_vect)
{
	USART_DataRegEmpty(&USART_data);
}
