
// Algoritmo para UART por Software

// - Usa timer;
// - Usa interrup��o da entrada de RX;

// Estado na inicializa��o -Silencio


#include "swUart.h"
#include "stdint.h"
#include "HardwareProfile.h"
#include "tmr0.h"
#include "SSD1306oLED.h"
#include "interrupt_manager.h"

// N�mero de bits do data format
#define NUM_BITS 8

//Tamanho do buffer em bytes
#define NUM_BUFFER_RX 200

// Numeros de bits em alto para considerar que linha est� em sil�ncio.
// No caso dois bytes, 2 dois bit-frame, 22 bits.
// Quando retornar ao estado de SILENT � gerado o evento de fim de Byte frame, callback ReadyByteFrame
#define NUM_BITS_SILENT 22

//	Amostragem no ":" (dois pontos)
//  ------| START_UA |---:---|   D1  |---:---|   D3  |---:---|   D5  |---:---|   D7  |---:---|---:---|---:---|---:--- ...   0xAA
//  	  |:---------|   D0  |---:---|  D2   |---:---|  D4   |---:---|  D6   |---:---|-Parity|       |       |       

// Tempo para o bitrate, em micro segundos

//Timer0 Registers Prescaler= 8 - TMR0 Preset = 100 - Freq = 9615.38 Hz - Period = 0.000104
//Por aferi��o ser� usado 110
#define TIME_BIT 110//110 //104us, (1/9600)*1000000

#define TIME_HALF (TIME_BIT/2) // Meio periodo do bit, para o start 1,5.
//Timer0 Registers Prescaler= 8 - TMR0 Preset = 25 - Freq = 6493.51 Hz - Period = 0.000154 seconds
//Por aferi��o ser� usado 34
#define TIME_ONE_AND_HALF  95//34 //TIME_BIT+TIME_HALF
/******************************************************************************/
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
uint8_t countBitsSilent;
uint8_t countOut;
uint8_t countIn;


/******************************************************************************/
void inFIFO(uint8_t data);

/******************************************************************************/
void InitSwUart()
{
	status = SILENT; 
	n_bit = 0;
	rx_data = 0;
	cs = 0;
	countBitsSilent = 0;
    
    countOut = 0;
    countIn = 0;
	
	// Inicializa pino RX
	// Interrup��o de Pino RX
	// Interrup��o Timer RX;
	
}
/******************************************************************************/
// Interrup��o da porta de RX
// Ocorre quando sinal transitar de n�vel alto para n�vel baixo, Edge Fall.
/******************************************************************************/
void InterruptPinRX(void)
{
    
	if(status == SILENT)
	{
		if(SW_UART_RX_PORT == 0)
		{
            
            LED_RED_PORT = !LED_RED_PORT;
            
            IOC_ENABLE = DISABLE;
            
			//status = START_UA;
			// Carrega o timer com o tempo de um bit e meio para pegar a amostragem quando no meio do intervalo bit.
            TMR0_Clear();
            reloadTimer(TIME_ONE_AND_HALF);
			TMR0_StartTimer();
            //reloadTimer(TIME_BIT);
            
            // Inicializa estado para dar entrada nos bits.
			//status = N_BIT; //redundante
			countBitsSilent = 0;
			n_bit = 0;
			cs = 0;
			rx_data = 0;
			status = N_BIT;
		}
	}
	// Limpar a interup��o
	clearInterruptPinRX();
}
/******************************************************************************/
void InterruptTimerUART(void)
{
	uint8_t mask, aux;
    
    reloadTimer(TIME_BIT);
    
	LED_RED_PORT = !LED_RED_PORT;
	switch(status)
	{
	case SILENT: // Quanto a linha ficar em sil�ncio por 2 byte-frame chama ReadyByteFrame.
		if(countBitsSilent ==  ( NUM_BITS_SILENT - 1 ) )
		{
            TMR0_StopTimer();
            
			ReadyByteFrame();
		}
		if(countBitsSilent < NUM_BITS_SILENT)
		{
			countBitsSilent ++;
			//reloadTimer(TIME_BIT);
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
			status = STOP_UA;
		}
		//reloadTimer(TIME_BIT);
	break;
	case PARITY:
		// Deve Usar cs para a verifica��o da paridade com o bit recebido
		// No caso pode ser negligenciado no momento
		if( SW_UART_RX_PORT == 0 )
		{}
		else
		{}
		status = STOP_UA;
		//reloadTimer(TIME_BIT);
	break;
	case STOP_UA:
        inFIFO(rx_data);
		status = SILENT;
		//reloadTimer(TIME_BIT);
        reloadTimer(0);
        aux = PORTB;
        IOC_FLAG = 0;
        IOC_ENABLE = ENABLE;
	break;
	default:
		status = SILENT;
		countBitsSilent = NUM_BITS_SILENT;
	break;
	}
    TMR0_Clear();
}

/******************************************************************************/
void reloadTimer(uint8_t setTimerValue)
{
    TMR0 = setTimerValue;
   /* switch(setTimerValue)
    {
        case TIME_BIT:
            //Timer0 Registers Prescaler= 8 - TMR0 Preset = 100 - Freq = 9615.38 Hz - Period = 0.000104 seconds
            //Por aferi��o ser� usado 110
            TMR0 = 110;//238;// 57600 //110;//100;
            break;
        case TIME_ONE_AND_HALF:
            //Timer0 Registers Prescaler= 8 - TMR0 Preset = 25 - Freq = 6493.51 Hz - Period = 0.000154 seconds
            //Por aferi��o ser� usado 34
            TMR0 = 34;//229; //34;//25;
            break;
        default:
            TMR0 = 0;
            break;
    }*/
}
/******************************************************************************/
void clearInterruptPinRX(void)
{
    bool tClearPin;
    //Faz uma leitura do Pin para limpar o IOC
    tClearPin = SW_UART_RX_PORT;
    //Limpa o Interrupt On Change
    IOC_FLAG = 0;
}
/******************************************************************************/
void printfOled(char value)
{
    static char lastValue;
    if(lastValue == value) return;
    lastValue = value;
    oled_clear();
    oled_putString(value,0,0);
}
/******************************************************************************/

void inFIFO(uint8_t data)
{
    buffer_rx[countIn] = data;
    countIn++;
    if(countIn == NUM_BUFFER_RX)
    {
        countIn = 0;
    }
    if(countOut == countIn)
    {
        countOut ++;
        if(countOut == NUM_BUFFER_RX)
        {
           countOut = 0; 
        }
    }
}

uint8_t outFIFO(void)
{
    uint8_t retVal = 0;
    INTERRUPT_GlobalInterruptDisable();
    if(countIn != countOut)
    {
        retVal = buffer_rx[countOut];
        countOut ++;
        if(countOut == NUM_BUFFER_RX){
           countOut = 0;
        }
    }
    INTERRUPT_GlobalInterruptEnable();
    return retVal;
}


uint8_t emptyFIFO(void)
{
    if(countIn == countOut)
    {
        return true;
    }
    return false;
}

uint8_t silentRX(void)
{
   if(status == SILENT)
   {
       return true;
   }
   return false;
}