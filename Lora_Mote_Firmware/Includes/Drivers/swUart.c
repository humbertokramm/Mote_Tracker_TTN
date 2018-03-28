
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
#define TIME_ONE_AND_HALF  TIME_BIT+TIME_HALF

typedef enum Status_t
{
	SILENT,
	//START_UA,
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

//
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
			//status = START_UA;
			// Carrega o timer com o tempo de um bit e meio para pegar a amostragem quando no meio do intervalo bit.
			reloadTimer(TIME_ONE_AND_HALF);
            
            // Inicializa estado para dar entrada nos bits.
			status = N_BIT;
			countBitsSilent = 0;
			n_bit = 0;
			cs = 0;
			rx_data = 0;
			status = N_BIT;
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
	case SILENT: // Quanto a linha ficar em silêncio por 2 byte-frame chama ReadyByteFrame.
		if(countBitsSilent ==  ( NUM_BITS_SILENT - 1 ) )
		{
			ReadyByteFrame(buffer_rx, countRX);
			// Prepara a próxima recepção de bytes.
			countRX = 0;
		}
		if(countBitsSilent < NUM_BITS_SILENT)
		{
			countBitsSilent ++;
			reloadTimer(TIME_BIT);
		}
	break;
    /*
	case START_UA:
		countBitsSilent = 0;
		n_bit = 0;
		cs = 0;
		rx_data = 0;
		status = N_BIT;
		reloadTimer(TIME_BIT);
	break;
     */
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
		// Deve Usar cs para a verificação da paridade com o bit recebido
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
    TMR0_Clear();
}

void reloadTimer(uint8_t setTimerValue)
{
    switch(setTimerValue)
    {
        case TIME_BIT:
            //Timer0 Registers Prescaler= 8 - TMR0 Preset = 100 - Freq = 9615.38 Hz - Period = 0.000104 seconds
            //Por aferição será usado 110
            TMR0 = 110;//100;
            break;
        case TIME_ONE_AND_HALF:
            //Timer0 Registers Prescaler= 8 - TMR0 Preset = 25 - Freq = 6493.51 Hz - Period = 0.000154 seconds
            //Por aferição será usado 34
            TMR0 = 34;//25;
            break;
        default:
            TMR0 = 0;
            break;
    }
}

void clearInterruptPinRX(void)
{
    IOC_FLAG = 0;
}