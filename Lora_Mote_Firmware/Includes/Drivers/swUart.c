
// Algoritmo para UART por Software

// - Usa timer;
// - Usa interrupção da entrada de RX;

// Estado na inicialização -Silencio


#include "swUart.h"
#include "stdint.h"
#include "HardwareProfile.h"

// Número de bits do data format
#define NUM_BITS 8

//Tamanho do buffer em bytes
#define NUM_BUFFER_RX 128

// Numeros de bits em alto para considerar que linha está em silêncio.
// No caso dois bytes, 2 dois bit-frame, 22 bits.
// Quando retornar ao estado de SILENT é gerado o evento de fim de Byte frame, callback ReadyByteFrame
#define NUM_BITS_SILENT 22

//	Amostragem no ":" (dois pontos)
//  ------| START_UA |---:---|   D1  |---:---|   D3  |---:---|   D5  |---:---|   D7  |---:---|---:---|---:---|---:--- ...   0xAA
//  	  |:---------|   D0  |---:---|  D2   |---:---|  D4   |---:---|  D6   |---:---|-Parity|       |       |       

// Tempo para o bitrate, em micro segundos

#define TIME_BIT 104 // us, (1/9600)*1000000
#define TIME_HALF (TIME_BIT/2) // Meio periodo do bit, para o start 1,5.

typedef enum Status_t
{
	SILENT,
	START_UA,
	N_BIT,
	PARITY,
	STOP_UA
};
enum Status_t status;

typedef enum Parity_t
{
	NONE,
	ODD,
	EVEN
};
enum Parity_t  parity;

typedef enum Stops_t
{
	STOPS1,
	STOPS2
};
enum Stops_t stops;


uint8_t n_bit;
uint8_t rx_data;
uint8_t buffer_rx[NUM_BUFFER_RX];
uint8_t cs;
uint8_t countRX;
uint8_t countBitsSilent;

void InitSwUart()
{
	status = SILENT; 
	n_bit = 0;
	rx_data = 0;
	cs = 0;
	countRX = 0;
	countBitsSilent = 0;
	
	// Inicializa pino RX
	// Interrupção de Pino RX
	// Interrupção Timer RX;
	
}


// Interrupção da porta de RX
// Ocorre quando sinal transitar de nível alto para nível baixo, Edge Fall.
void InterruptPinRX(void)
{
	if(status == SILENT)
	{
		if(SW_UART_RX_PORT == 0)
		{
			status = START_UA;
			// Carrega o timer com o tempo de um bit e meio para pegar a amostragem quando no meio do intervalo bit.
			reloadTimer(TIME_HALF + TIME_BIT);
		}
	}
	// Limpar a interupção
	clearInterruptPinRX();
}

void InterruptTimerUART(void)
{
	uint8_t mask;
	
	switch(status)
	{
	case SILENT: // Quanto a linha ficar em silêncio por 2 bit frame chama ReadyByteFrame.
	if(countBitsSilent ==  ( NUM_BITS_SILENT - 1 ) )
	{
		ReadyByteFrame(buffer_rx, countRX);
		// Apronta para a próxima recepção de bytes.
		countRX = 0;
	}
	if(countBitsSilent < NUM_BITS_SILENT)
	{
		countBitsSilent ++;
		reloadTimer(TIME_BIT);
	}
	break;
	case START_UA:
		countBitsSilent = 0;
		n_bit = 0;
		cs = 0;
		rx_data = 0;
		status = N_BIT;
		reloadTimer(TIME_BIT);
	break;
	case N_BIT:
		mask = 0x01 << n_bit;
		// Seta o bit recebido
		if( SW_UART_RX_PORT == 1 )
		{
			rx_data |= mask;
			cs ++;
		}
		else
		{
			// Redundante... 
			rx_data &= ~mask;
		}
		n_bit ++;
		if( n_bit == NUM_BITS )
		{
			status = PARITY;
		}
		reloadTimer(TIME_BIT);
	break;
	case PARITY:
		// Deve Usar cs para fazer a verificação da paridade com o bit recebido
		// No caso pode ser negligenciado no momento
		if( SW_UART_RX_PORT == 0 )
		{}
		else
		{}
		status = STOP_UA;
		reloadTimer(TIME_BIT);
	break;
	case STOP_UA:
		buffer_rx[countRX] = rx_data;
		countRX ++;
		status = SILENT;
		reloadTimer(TIME_BIT);
	break;
	default:
		countRX = 0;
		status = SILENT;
		countBitsSilent = NUM_BITS_SILENT;
	break;
	}
	clearInterruptTimerUART();
}

void clearInterruptTimerUART(void)
{
    
}

void reloadTimer(uint8_t setTimerValue)
{
    
}

void clearInterruptPinRX(void)
{
    
}