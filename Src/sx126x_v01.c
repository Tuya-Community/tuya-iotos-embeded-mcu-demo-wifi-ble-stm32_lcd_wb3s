/* 
Description: Generic SX126x driver implementation
*/
#include <math.h>
#include <string.h> 
#include "sx126x_v01.h"



/*!
 * \brief Holds the internal operating mode of the radio
 */
static RadioOperatingModes_t OperatingMode;

/*!
 * \brief Stores the current packet type set in the radio
 */
static RadioPacketTypes_t PacketType;

/*!
 * \brief Stores the last frequency error measured on LoRa received packet
 */
volatile uint32_t FrequencyError = 0;

/*!
 * \brief Hold the status of the Image calibration
 */
static bool ImageCalibrated = false;

uint8_t MaxPayloadLength = 0xFF;
const RadioLoRaBandwidths_t Bandwidths[] = { LORA_BW_125, LORA_BW_250, LORA_BW_500,LORA_BW_020,LORA_BW_031,LORA_BW_062,LORA_BW_041 };

/*!
 * Precomputed FSK bandwidth registers values
 */
const FskBandwidth_t FskBandwidths[] =
{
    { 4800  , 0x1F },
    { 5800  , 0x17 },
    { 7300  , 0x0F },
    { 9700  , 0x1E },
    { 11700 , 0x16 },
    { 14600 , 0x0E },
    { 19500 , 0x1D },
    { 23400 , 0x15 },
    { 29300 , 0x0D },
    { 39000 , 0x1C },
    { 46900 , 0x14 },
    { 58600 , 0x0C },
    { 78200 , 0x1B },
    { 93800 , 0x13 },
    { 117300, 0x0B },
    { 156200, 0x1A },
    { 187200, 0x12 },
    { 234300, 0x0A },
    { 312000, 0x19 },
    { 373600, 0x11 },
    { 467000, 0x09 },
    { 500000, 0x00 }, // Invalid Bandwidth
};
/*!
 * Radio hardware and global parameters
 */
SX126x_t SX126x;

extern SPI_HandleTypeDef hspi1;
bool RxContinuous = true;//设置为连续接收
uint32_t TxTimeout = 0;
uint32_t RxTimeout = 0;

#define CHIP_TYPE   SX1262    //SX1261-SX1261  SX1262-SX1262/SX1268  

//#define USE_TCXO

#ifdef USE_TCXO
    /*!
     * Radio c
omplete Wake-up Time with TCXO stabilisation time
     */
    #define RADIO_TCXO_SETUP_TIME                       5 // [ms]
#else
    /*!
     * Radio complete Wake-up Time with TCXO stabilisation time
     */
    #define RADIO_TCXO_SETUP_TIME                       0 // No Used
#endif


//////////RADIO 层 ///////
void RadioInit(void)
{
    //RadioEvents = events;//这里进行了函数的初始化
#ifdef USE_TCXO
	printf("USE TCXO\n");
#else
	printf("USE CRYSTAL\n");
#endif
    SX126xInit();////中断的回调函后续在其他地方去定义
    SX126xSetStandby( STDBY_RC );
    SX126xSetRegulatorMode( USE_DCDC);//USE_LDO//USE_DCDC

    SX126xSetBufferBaseAddress( 0x00, 0x00 );
    SX126xSetTxParams( 0, RADIO_RAMP_200_US );
    //DIO_0的中断MASK全部打开，在RadioSend()会再继续细分中断
    SX126xSetDioIrqParams( IRQ_RADIO_ALL, IRQ_RADIO_ALL, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
}
void RadioSetTxContinuousWave( uint32_t freq, int8_t power, uint16_t time )
{
    SX126xSetRfFrequency( freq );
    SX126xSetRfTxPower( power );
    SX126xSetTxContinuousWave( );
}
void RadioStandby( void )
{
    SX126xSetStandby( STDBY_RC );
}



void RadioSetModem( RadioModems_t modem )
{
    switch( modem )
    {
    default:
    case MODEM_FSK:
        SX126xSetPacketType( PACKET_TYPE_GFSK );
        // When switching to GFSK mode the LoRa SyncWord register value is reset
        // Thus, we also reset the RadioPublicNetwork variable
        RadioPublicNetwork.Current = false;
        break;
    case MODEM_LORA:
        SX126xSetPacketType( PACKET_TYPE_LORA );
        // Public/Private network register is reset when switching modems
        if( RadioPublicNetwork.Current != RadioPublicNetwork.Previous )
        {
            RadioPublicNetwork.Current = RadioPublicNetwork.Previous;
            RadioSetPublicNetwork( RadioPublicNetwork.Current );
        }
        break;
    }
}

void RadioSetPublicNetwork( bool enable )
{
	RadioPublicNetwork.Current = RadioPublicNetwork.Previous = enable;

	RadioSetModem( MODEM_LORA );
	if( enable == true )
	{
		// Change LoRa modem SyncWord
		SX126xWriteRegister( REG_LR_SYNCWORD, ( LORA_MAC_PUBLIC_SYNCWORD >> 8 ) & 0xFF );
		SX126xWriteRegister( REG_LR_SYNCWORD + 1, LORA_MAC_PUBLIC_SYNCWORD & 0xFF );
	}
	else
	{
		// Change LoRa modem SyncWord
		SX126xWriteRegister( REG_LR_SYNCWORD, ( LORA_MAC_PRIVATE_SYNCWORD >> 8 ) & 0xFF );
		SX126xWriteRegister( REG_LR_SYNCWORD + 1, LORA_MAC_PRIVATE_SYNCWORD & 0xFF );
	}
}

//timeoout是ms单位
void RadioRx( uint32_t timeout )
{

//for lierda switcher
	HAL_GPIO_WritePin(SW_CTL1_GPIO_Port, SW_CTL1_Pin, GPIO_PIN_SET);	
	HAL_GPIO_WritePin(SW_CTL2_GPIO_Port, SW_CTL2_Pin, GPIO_PIN_RESET);		
	
//end




	SX126xSetDioIrqParams( IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT |IRQ_CRC_ERROR,
						   IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT |IRQ_CRC_ERROR,
						   IRQ_RADIO_NONE,
						   IRQ_RADIO_NONE );
	if( RxContinuous == true )
	{
		SX126xSetRx( 0xFFFFFF ); // Rx Continuous
	}
	else
	{
		SX126xSetRx( timeout << 6 );//ms为单位
		printf("RXTimeout=%d\n",timeout<<6);
	}
}



	
void RadioSend( uint8_t *buffer, uint8_t size )
{
//for lierda switcher
	HAL_GPIO_WritePin(SW_CTL1_GPIO_Port, SW_CTL1_Pin, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(SW_CTL2_GPIO_Port, SW_CTL2_Pin, GPIO_PIN_SET);	

//end

	SX126xSetDioIrqParams( IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
						   IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
						   IRQ_RADIO_NONE,
						   IRQ_RADIO_NONE );

	if( SX126xGetPacketType( ) == PACKET_TYPE_LORA )
	{
		SX126x.PacketParams.Params.LoRa.PayloadLength = size;
	}
	else
	{
		SX126x.PacketParams.Params.Gfsk.PayloadLength = size;
	}
	SX126xSetPacketParams( &SX126x.PacketParams );

	SX126xSendPayload( buffer, size, 0 );//timeout 为0时不带timeout功能
}


/*!
 * Returns the known FSK bandwidth registers value
 *
 * \param [IN] bandwidth Bandwidth value in Hz
 * \retval regValue Bandwidth register value.
 */
static uint8_t RadioGetFskBandwidthRegValue( uint32_t bandwidth )
{
	uint8_t i;

	if( bandwidth == 0 )
	{
		return( 0x1F );
	}

	for( i = 0; i < ( sizeof( FskBandwidths ) / sizeof( FskBandwidth_t ) ) - 1; i++ )
	{
		if( ( bandwidth >= FskBandwidths[i].bandwidth ) && ( bandwidth < FskBandwidths[i + 1].bandwidth ) )
		{
			printf("FSK BW=%d\n",FskBandwidths[i+1]);
			return FskBandwidths[i+1].RegValue;
		}
	}
	// ERROR: Value not found
	while( 1 );
}




void RadioSetTxConfig( RadioModems_t modem, int8_t power, uint32_t fdev,
                        uint32_t bandwidth, uint32_t datarate,
                        uint8_t coderate, uint16_t preambleLen,
                        bool fixLen, bool crcOn, bool freqHopOn,
                        uint8_t hopPeriod, bool iqInverted, uint32_t timeout )
{

    switch( modem )
    {
        case MODEM_FSK:
            SX126x.ModulationParams.PacketType = PACKET_TYPE_GFSK;
            SX126x.ModulationParams.Params.Gfsk.BitRate = datarate;

            SX126x.ModulationParams.Params.Gfsk.ModulationShaping = MOD_SHAPING_G_BT_1;//MOD_SHAPING_G_BT_03
            SX126x.ModulationParams.Params.Gfsk.Bandwidth = RadioGetFskBandwidthRegValue( bandwidth );
            SX126x.ModulationParams.Params.Gfsk.Fdev = fdev;

            SX126x.PacketParams.PacketType = PACKET_TYPE_GFSK;
            SX126x.PacketParams.Params.Gfsk.PreambleLength = ( preambleLen << 3 ); // convert byte into bit
            SX126x.PacketParams.Params.Gfsk.PreambleMinDetect = RADIO_PREAMBLE_DETECTOR_08_BITS;
            SX126x.PacketParams.Params.Gfsk.SyncWordLength = 3 << 3 ; // convert byte into bit
            SX126x.PacketParams.Params.Gfsk.AddrComp = RADIO_ADDRESSCOMP_FILT_OFF;
            SX126x.PacketParams.Params.Gfsk.HeaderType = ( fixLen == true ) ? RADIO_PACKET_FIXED_LENGTH : RADIO_PACKET_VARIABLE_LENGTH;

            if( crcOn == true )
            {
                SX126x.PacketParams.Params.Gfsk.CrcLength = RADIO_CRC_2_BYTES_CCIT;
            }
            else
            {
                SX126x.PacketParams.Params.Gfsk.CrcLength = RADIO_CRC_OFF;
            }
            SX126x.PacketParams.Params.Gfsk.DcFree = RADIO_DC_FREEWHITENING;

            RadioStandby( );
            RadioSetModem( ( SX126x.ModulationParams.PacketType == PACKET_TYPE_GFSK ) ? MODEM_FSK : MODEM_LORA );
            SX126xSetModulationParams( &SX126x.ModulationParams );
            SX126xSetPacketParams( &SX126x.PacketParams );
            SX126xSetSyncWord( ( uint8_t[] ){ 0xC1, 0x94, 0xC1, 0x00, 0x00, 0x00, 0x00, 0x00 } );
            SX126xSetWhiteningSeed( 0x01FF );
            break;

        case MODEM_LORA:
            SX126x.ModulationParams.PacketType = PACKET_TYPE_LORA;
            SX126x.ModulationParams.Params.LoRa.SpreadingFactor = ( RadioLoRaSpreadingFactors_t ) datarate;
            SX126x.ModulationParams.Params.LoRa.Bandwidth =  Bandwidths[bandwidth];
            SX126x.ModulationParams.Params.LoRa.CodingRate= ( RadioLoRaCodingRates_t )coderate;

            if( ( ( bandwidth == 0 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
            ( ( bandwidth == 1 ) && ( datarate == 12 ) ) )
            {
                SX126x.ModulationParams.Params.LoRa.LowDatarateOptimize = 0x01;
            }
            else
            {
                SX126x.ModulationParams.Params.LoRa.LowDatarateOptimize = 0x00;
            }

            SX126x.PacketParams.PacketType = PACKET_TYPE_LORA;

            if( ( SX126x.ModulationParams.Params.LoRa.SpreadingFactor == LORA_SF5 ) ||
                ( SX126x.ModulationParams.Params.LoRa.SpreadingFactor == LORA_SF6 ) )
            {
                if( preambleLen < 12 )
                {
                    SX126x.PacketParams.Params.LoRa.PreambleLength = 12;
                }
                else
                {
                    SX126x.PacketParams.Params.LoRa.PreambleLength = preambleLen;
                }
            }
            else
            {
                SX126x.PacketParams.Params.LoRa.PreambleLength = preambleLen;
            }

            SX126x.PacketParams.Params.LoRa.HeaderType = ( RadioLoRaPacketLengthsMode_t )fixLen;
            SX126x.PacketParams.Params.LoRa.PayloadLength = MaxPayloadLength;
            SX126x.PacketParams.Params.LoRa.CrcMode = ( RadioLoRaCrcModes_t )crcOn;
            SX126x.PacketParams.Params.LoRa.InvertIQ = ( RadioLoRaIQModes_t )iqInverted;

            RadioStandby( );
            RadioSetModem( ( SX126x.ModulationParams.PacketType == PACKET_TYPE_GFSK ) ? MODEM_FSK : MODEM_LORA );
            SX126xSetModulationParams( &SX126x.ModulationParams );
            SX126xSetPacketParams( &SX126x.PacketParams );
            break;
    }
    SX126xSetRfTxPower( power );
    //TxTimeout = timeout;
}

void RadioSetRxConfig( RadioModems_t modem, uint32_t bandwidth,
                         uint32_t datarate, uint8_t coderate,
                         uint32_t bandwidthAfc, uint16_t preambleLen,
                         uint16_t symbTimeout, bool fixLen,
                         uint8_t payloadLen,
                         bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                         bool iqInverted, bool rxContinuous )
{

    RxContinuous = rxContinuous;

    if( fixLen == true )
    {
        MaxPayloadLength = payloadLen;
    }
    else
    {
        MaxPayloadLength = 0xFF;
    }

    switch( modem )
    {
        case MODEM_FSK:
            SX126xSetStopRxTimerOnPreambleDetect( false );
            SX126x.ModulationParams.PacketType = PACKET_TYPE_GFSK;

            SX126x.ModulationParams.Params.Gfsk.BitRate = datarate;
            SX126x.ModulationParams.Params.Gfsk.ModulationShaping = MOD_SHAPING_G_BT_1;
            SX126x.ModulationParams.Params.Gfsk.Bandwidth = RadioGetFskBandwidthRegValue( bandwidth );

            SX126x.PacketParams.PacketType = PACKET_TYPE_GFSK;
            SX126x.PacketParams.Params.Gfsk.PreambleLength = ( preambleLen << 3 ); // convert byte into bit
            SX126x.PacketParams.Params.Gfsk.PreambleMinDetect = RADIO_PREAMBLE_DETECTOR_08_BITS;
            SX126x.PacketParams.Params.Gfsk.SyncWordLength = 4 << 3; // convert byte into bit
            SX126x.PacketParams.Params.Gfsk.AddrComp = RADIO_ADDRESSCOMP_FILT_OFF;
            SX126x.PacketParams.Params.Gfsk.HeaderType = ( fixLen == true ) ? RADIO_PACKET_FIXED_LENGTH : RADIO_PACKET_VARIABLE_LENGTH;
            SX126x.PacketParams.Params.Gfsk.PayloadLength = MaxPayloadLength;
            if( crcOn == true )
            {
                SX126x.PacketParams.Params.Gfsk.CrcLength = RADIO_CRC_2_BYTES;//RADIO_CRC_2_BYTES_CCIT;
            }
            else
            {
                SX126x.PacketParams.Params.Gfsk.CrcLength = RADIO_CRC_OFF;
            }
            SX126x.PacketParams.Params.Gfsk.DcFree = RADIO_DC_FREE_OFF;//RADIO_DC_FREEWHITENING;

            RadioStandby( );
            RadioSetModem( ( SX126x.ModulationParams.PacketType == PACKET_TYPE_GFSK ) ? MODEM_FSK : MODEM_LORA );
            SX126xSetModulationParams( &SX126x.ModulationParams );
            SX126xSetPacketParams( &SX126x.PacketParams );
            //SX126xSetSyncWord( ( uint8_t[] ){ 0xC1, 0x94, 0xC1, 0x00, 0x00, 0x00, 0x00, 0x00 } );
						SX126xSetSyncWord( ( uint8_t[] ){ 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0 } );
            SX126xSetWhiteningSeed( 0x01FF );

            RxTimeout = ( uint32_t )( symbTimeout * ( ( 1.0 / ( double )datarate ) * 8.0 ) * 1000 );
            break;

        case MODEM_LORA:
            SX126xSetStopRxTimerOnPreambleDetect( false );
            SX126xSetLoRaSymbNumTimeout( symbTimeout );
            SX126x.ModulationParams.PacketType = PACKET_TYPE_LORA;
            SX126x.ModulationParams.Params.LoRa.SpreadingFactor = ( RadioLoRaSpreadingFactors_t )datarate;
            SX126x.ModulationParams.Params.LoRa.Bandwidth = Bandwidths[bandwidth];
            SX126x.ModulationParams.Params.LoRa.CodingRate = ( RadioLoRaCodingRates_t )coderate;

            if( ( ( bandwidth == 0 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
            ( ( bandwidth == 1 ) && ( datarate == 12 ) ) )
            {
                SX126x.ModulationParams.Params.LoRa.LowDatarateOptimize = 0x01;
            }
            else
            {
                SX126x.ModulationParams.Params.LoRa.LowDatarateOptimize = 0x00;
            }

            SX126x.PacketParams.PacketType = PACKET_TYPE_LORA;

            if( ( SX126x.ModulationParams.Params.LoRa.SpreadingFactor == LORA_SF5 ) ||
                ( SX126x.ModulationParams.Params.LoRa.SpreadingFactor == LORA_SF6 ) )
            {
                if( preambleLen < 12 )
                {
                    SX126x.PacketParams.Params.LoRa.PreambleLength = 12;
                }
                else
                {
                    SX126x.PacketParams.Params.LoRa.PreambleLength = preambleLen;
                }
            }
            else
            {
                SX126x.PacketParams.Params.LoRa.PreambleLength = preambleLen;
            }

            SX126x.PacketParams.Params.LoRa.HeaderType = ( RadioLoRaPacketLengthsMode_t )fixLen;

            SX126x.PacketParams.Params.LoRa.PayloadLength = MaxPayloadLength;
            SX126x.PacketParams.Params.LoRa.CrcMode = ( RadioLoRaCrcModes_t )crcOn;
            SX126x.PacketParams.Params.LoRa.InvertIQ = ( RadioLoRaIQModes_t )iqInverted;


			//public/private 设置
            RadioSetModem( ( SX126x.ModulationParams.PacketType == PACKET_TYPE_GFSK ) ? MODEM_FSK : MODEM_LORA );
            SX126xSetModulationParams( &SX126x.ModulationParams );
            SX126xSetPacketParams( &SX126x.PacketParams );

            // Timeout Max, Timeout handled directly in SetRx function
			//超时时间设置为最大
            RxTimeout = 0xFFFF;

            break;
    }
}



///SX126x层
void SX126xSetWhiteningSeed( uint16_t seed )
{
    uint8_t regValue = 0;
    switch( SX126xGetPacketType( ) )
    {
        case PACKET_TYPE_GFSK:
            regValue = SX126xReadRegister( REG_LR_WHITSEEDBASEADDR_MSB ) & 0xFE;
            regValue = ( ( seed >> 8 ) & 0x01 ) | regValue;
            SX126xWriteRegister( REG_LR_WHITSEEDBASEADDR_MSB, regValue ); // only 1 bit.
            SX126xWriteRegister( REG_LR_WHITSEEDBASEADDR_LSB, ( uint8_t )seed );
            break;

        default:
            break;
    }

}

uint8_t SX126xSetSyncWord( uint8_t *syncWord )
{
    SX126xWriteRegisters( REG_LR_SYNCWORDBASEADDRESS, syncWord, 8 );
    return 0;

}

void SX126xSetModulationParams( ModulationParams_t *modulationParams )
{
    uint8_t n;
    uint32_t tempVal = 0;
    uint8_t buf[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    // Check if required configuration corresponds to the stored packet type
    // If not, silently update radio packet type
    if( PacketType != modulationParams->PacketType )
    {
        SX126xSetPacketType( modulationParams->PacketType );
    }

    switch( modulationParams->PacketType )
    {
    case PACKET_TYPE_GFSK:
        n = 8;
        tempVal = ( uint32_t )( 32 * ( ( double )XTAL_FREQ / ( double )modulationParams->Params.Gfsk.BitRate ) );
        buf[0] = ( tempVal >> 16 ) & 0xFF;
        buf[1] = ( tempVal >> 8 ) & 0xFF;
        buf[2] = tempVal & 0xFF;
        buf[3] = modulationParams->Params.Gfsk.ModulationShaping;
        buf[4] = modulationParams->Params.Gfsk.Bandwidth;
        tempVal = ( uint32_t )( ( double )modulationParams->Params.Gfsk.Fdev / ( double )FREQ_STEP );
        buf[5] = ( tempVal >> 16 ) & 0xFF;
        buf[6] = ( tempVal >> 8 ) & 0xFF;
        buf[7] = ( tempVal& 0xFF );
        break;
    case PACKET_TYPE_LORA:
        n = 4;
        switch( modulationParams->Params.LoRa.Bandwidth )
        {
            case LORA_BW_500:
                 modulationParams->Params.LoRa.LowDatarateOptimize = 0x00;
                break;
            case LORA_BW_250:
                if( modulationParams->Params.LoRa.SpreadingFactor == 12 )
                {
                    modulationParams->Params.LoRa.LowDatarateOptimize = 0x01;
                }
                else
                {
                    modulationParams->Params.LoRa.LowDatarateOptimize = 0x00;
                }
                break;
            case LORA_BW_125:
                if( modulationParams->Params.LoRa.SpreadingFactor >= 11 )
                {
                    modulationParams->Params.LoRa.LowDatarateOptimize = 0x01;
                }
                else
                {
                    modulationParams->Params.LoRa.LowDatarateOptimize = 0x00;
                }
                break;
            case LORA_BW_062:
                if( modulationParams->Params.LoRa.SpreadingFactor >= 10 )
                {
                    modulationParams->Params.LoRa.LowDatarateOptimize = 0x01;
                }
                else
                {
                    modulationParams->Params.LoRa.LowDatarateOptimize = 0x00;
                }
                break;
            case LORA_BW_041:
                if( modulationParams->Params.LoRa.SpreadingFactor >= 9 )
                {
                    modulationParams->Params.LoRa.LowDatarateOptimize = 0x01;
                }
                else
                {
                    modulationParams->Params.LoRa.LowDatarateOptimize = 0x00;
                }
                break;
            case LORA_BW_031:
            case LORA_BW_020:
            case LORA_BW_015:
            case LORA_BW_010:
            case LORA_BW_007:
                    modulationParams->Params.LoRa.LowDatarateOptimize = 0x01;
                break;
            default:
                break;
        }
        buf[0] = modulationParams->Params.LoRa.SpreadingFactor;
        buf[1] = modulationParams->Params.LoRa.Bandwidth;
        buf[2] = modulationParams->Params.LoRa.CodingRate;
        buf[3] = modulationParams->Params.LoRa.LowDatarateOptimize;
        break;
    default:
    case PACKET_TYPE_NONE:
        return;
    }
    printf("LowDatarateOptimize=%d\n",modulationParams->Params.LoRa.LowDatarateOptimize);
    SX126xWriteCommand( RADIO_SET_MODULATIONPARAMS, buf, n );
}



void SX126xSendPayload( uint8_t *payload, uint8_t size, uint32_t timeout )
{
    SX126xSetPayload( payload, size );
    SX126xSetTx( timeout );
}

void SX126xSetPayload( uint8_t *payload, uint8_t size )
{
    SX126xWriteBuffer( 0x00, payload, size );
}

uint8_t SX126xGetPayload( uint8_t *buffer, uint8_t *size,  uint8_t maxSize )
{
    uint8_t offset = 0;
    SX126xGetRxBufferStatus( size, &offset );
    if( *size > maxSize )
    {
        return 1;
    }
    SX126xReadBuffer( offset, buffer, *size );
    return 0;
}
RadioStatus_t SX126xGetStatus( void )
{
    uint8_t stat = 0;
    RadioStatus_t status;
    SX126xReadCommand( RADIO_GET_STATUS, ( uint8_t * )&stat, 1 );
    status.Value = stat;
    return status;
}

void SX126xGetPacketStatus( PacketStatus_t *pktStatus )
{
		uint8_t status[3];
	
		SX126xReadCommand( RADIO_GET_PACKETSTATUS, status, 3 );
	
		pktStatus->packetType = SX126xGetPacketType( );
		switch( pktStatus->packetType )
		{
			case PACKET_TYPE_GFSK:
				pktStatus->Params.Gfsk.RxStatus = status[0];
				pktStatus->Params.Gfsk.RssiSync = -status[1] >> 1;
				pktStatus->Params.Gfsk.RssiAvg = -status[2] >> 1;
				pktStatus->Params.Gfsk.FreqError = 0;
				break;
	
			case PACKET_TYPE_LORA:
				pktStatus->Params.LoRa.RssiPkt = -status[0] >> 1;
				( status[1] < 128 ) ? ( pktStatus->Params.LoRa.SnrPkt = status[1] >> 2 ) : ( pktStatus->Params.LoRa.SnrPkt = ( ( status[1] - 256 ) >> 2 ) );
				pktStatus->Params.LoRa.SignalRssiPkt = -status[2] >> 1;
				pktStatus->Params.LoRa.FreqError = FrequencyError;
				break;
	
			default:
			case PACKET_TYPE_NONE:
				// In that specific case, we set everything in the pktStatus to zeros
				// and reset the packet type accordingly
				memset( pktStatus, 0, sizeof( PacketStatus_t ) );
				pktStatus->packetType = PACKET_TYPE_NONE;
				break;
		}
}

void SX126xSetStopRxTimerOnPreambleDetect( bool enable )
{
    SX126xWriteCommand( RADIO_SET_STOPRXTIMERONPREAMBLE, ( uint8_t* )&enable, 1 );
}
void SX126xSetLoRaSymbNumTimeout( uint8_t SymbNum )
{
    SX126xWriteCommand( RADIO_SET_LORASYMBTIMEOUT, &SymbNum, 1 );
}

	
void SX126xGetRxBufferStatus( uint8_t *payloadLength, uint8_t *rxStartBufferPointer )
{
	uint8_t status[2];
	SX126xReadCommand( RADIO_GET_RXBUFFERSTATUS, status, 2 );

	// In case of LORA fixed header, the payloadLength is obtained by reading
	// the register REG_LR_PAYLOADLENGTH
	if( ( SX126xGetPacketType( ) == PACKET_TYPE_LORA ) && ( SX126xReadRegister( REG_LR_PACKETPARAMS ) >> 7 == 1 ) )
	{
		*payloadLength = SX126xReadRegister( REG_LR_PAYLOADLENGTH );
	}
	else
	{
		*payloadLength = status[0];
	}
	*rxStartBufferPointer = status[1];
}


//超时时间为  timeout*15.625us

void SX126xSetTx( uint32_t timeout )
{
    uint8_t buf[3];
    OperatingMode = MODE_TX;
    buf[0] = ( uint8_t )( ( timeout >> 16 ) & 0xFF );
    buf[1] = ( uint8_t )( ( timeout >> 8 ) & 0xFF );
    buf[2] = ( uint8_t )( timeout & 0xFF );
    SX126xWriteCommand( RADIO_SET_TX, buf, 3 );
}


void SX126xSetRx( uint32_t timeout )
{
    uint8_t buf[3];

    OperatingMode = MODE_RX;

    buf[0] = ( uint8_t )( ( timeout >> 16 ) & 0xFF );
    buf[1] = ( uint8_t )( ( timeout >> 8 ) & 0xFF );
    buf[2] = ( uint8_t )( timeout & 0xFF );
    SX126xWriteCommand( RADIO_SET_RX, buf, 3 );
}



void SX126xSetPacketParams( PacketParams_t *packetParams )
{
    uint8_t n;
    uint8_t crcVal = 0;
    uint8_t buf[9] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

    // Check if required configuration corresponds to the stored packet type
    // If not, silently update radio packet type
    if( PacketType != packetParams->PacketType )
    {
        SX126xSetPacketType( packetParams->PacketType );
    }

    switch( packetParams->PacketType )
    {
    case PACKET_TYPE_GFSK:
        if( packetParams->Params.Gfsk.CrcLength == RADIO_CRC_2_BYTES_IBM )
        {
            SX126xSetCrcSeed( CRC_IBM_SEED );
            SX126xSetCrcPolynomial( CRC_POLYNOMIAL_IBM );
            crcVal = RADIO_CRC_2_BYTES;
        }
        else if( packetParams->Params.Gfsk.CrcLength == RADIO_CRC_2_BYTES_CCIT )
        {
            SX126xSetCrcSeed( CRC_CCITT_SEED );
            SX126xSetCrcPolynomial( CRC_POLYNOMIAL_CCITT );
            crcVal = RADIO_CRC_2_BYTES_INV;
        }
        else
        {
            crcVal = packetParams->Params.Gfsk.CrcLength;
        }
        n = 9;
        buf[0] = ( packetParams->Params.Gfsk.PreambleLength >> 8 ) & 0xFF;
        buf[1] = packetParams->Params.Gfsk.PreambleLength;
        buf[2] = packetParams->Params.Gfsk.PreambleMinDetect;
        buf[3] = ( packetParams->Params.Gfsk.SyncWordLength /*<< 3*/ ); // convert from byte to bit
        buf[4] = packetParams->Params.Gfsk.AddrComp;
        buf[5] = packetParams->Params.Gfsk.HeaderType;
        buf[6] = packetParams->Params.Gfsk.PayloadLength;
        buf[7] = crcVal;
        buf[8] = packetParams->Params.Gfsk.DcFree;
        break;
    case PACKET_TYPE_LORA:
        n = 6;
        buf[0] = ( packetParams->Params.LoRa.PreambleLength >> 8 ) & 0xFF;
        buf[1] = packetParams->Params.LoRa.PreambleLength;
        buf[2] = packetParams->Params.LoRa.HeaderType;
        buf[3] = packetParams->Params.LoRa.PayloadLength;
        buf[4] = packetParams->Params.LoRa.CrcMode;
        buf[5] = packetParams->Params.LoRa.InvertIQ;
        break;
    default:
    case PACKET_TYPE_NONE:
        return;
    }
    SX126xWriteCommand( RADIO_SET_PACKETPARAMS, buf, n );
}



void SX126xSetCrcSeed( uint16_t seed )
{
    uint8_t buf[2];
    buf[0] = ( uint8_t )( ( seed >> 8 ) & 0xFF );
    buf[1] = ( uint8_t )( seed & 0xFF );
    switch( SX126xGetPacketType( ) )
    {
        case PACKET_TYPE_GFSK:
            SX126xWriteRegisters( REG_LR_CRCSEEDBASEADDR, buf, 2 );
            break;

        default:
            break;
    }
}



void SX126xSetCrcPolynomial( uint16_t polynomial )
{
    uint8_t buf[2];
    buf[0] = ( uint8_t )( ( polynomial >> 8 ) & 0xFF );
    buf[1] = ( uint8_t )( polynomial & 0xFF );
    switch( SX126xGetPacketType( ) )
    {
        case PACKET_TYPE_GFSK:
            SX126xWriteRegisters( REG_LR_CRCPOLYBASEADDR, buf, 2 );
            break;

        default:
            break;
    }
}


void SX126xSetPacketType( RadioPacketTypes_t packetType )
{
    // Save packet type internally to avoid questioning the radio
    PacketType = packetType;
    
    if( packetType == PACKET_TYPE_GFSK )
    {
    	//fsk的时候要清除掉
        SX126xWriteRegister( REG_BIT_SYNC, 0x00 );
    }
    SX126xWriteCommand( RADIO_SET_PACKETTYPE, ( uint8_t* )&packetType, 1 );
}


//返回当前发包的类型
RadioPacketTypes_t SX126xGetPacketType( void )
{
    return PacketType;
}


void SX126xSetTxInfinitePreamble( void )
{
    SX126xWriteCommand( RADIO_SET_TXCONTINUOUSPREAMBLE, 0, 0 );
}



void SX126xSetPaConfig( uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut )
{
    uint8_t buf[4];
    buf[0] = paDutyCycle;
    buf[1] = hpMax;
    buf[2] = deviceSel;
    buf[3] = paLut;
    SX126xWriteCommand( RADIO_SET_PACONFIG, buf, 4 );
}


uint8_t SX126xGetPaSelect( uint32_t channel )
{
	//始终返回1262  这样可以发射22dBm的功率
	if (CHIP_TYPE==SX1261)
	{
		printf("CHIP is SX1261\n");
		return SX1261;
	}
	else
	{
		printf("CHIP is SX1262 or SX1268\n");
		return SX1262;
	}
    
}


void SX126xSetTxParams( int8_t power, RadioRampTimes_t rampTime )
{
    uint8_t buf[2];

    if( SX126xGetPaSelect( 0 ) == SX1261 )
    {
        if( power == 15 )
        {
            SX126xSetPaConfig( 0x06, 0x00, 0x01, 0x01 );
        }
        else
        {
            SX126xSetPaConfig( 0x04, 0x00, 0x01, 0x01 );
        }
        if( power >= 14 )
        {
            power = 14;
        }
        else if( power < -3 )
        {
            power = -3;
        }
        SX126xWriteRegister( REG_OCP, 0x18 ); // current max is 80 mA for the whole device
    }
    else // sx1262
    {
        SX126xSetPaConfig( 0x04, 0x07, 0x00, 0x01 );
        if( power > 22 )
        {
            power = 22;
        }
        else if( power < -3 )
        {
            power = -3;
        }
        SX126xWriteRegister( REG_OCP, 0x38 ); // current max 160mA for the whole device
    }
    buf[0] = power;
    buf[1] = ( uint8_t )rampTime;
    SX126xWriteCommand( RADIO_SET_TXPARAMS, buf, 2 );
}

void SX126xSetRegulatorMode( RadioRegulatorMode_t mode )
{
    SX126xWriteCommand( RADIO_SET_REGULATORMODE, ( uint8_t* )&mode, 1 );
}


void SX126xSetTxContinuousWave( void )
{
    SX126xWriteCommand( RADIO_SET_TXCONTINUOUSWAVE, 0, 0 );
}

void SX126xSetRfFrequency( uint32_t frequency )
{
    uint8_t buf[4];
    uint32_t freq = 0;

    if( ImageCalibrated == false )
    {
        SX126xCalibrateImage( frequency );
        ImageCalibrated = true;
    }
	printf("Frequenc= %d\n",frequency);
    freq = ( uint32_t )( ( double )frequency / ( double )FREQ_STEP );
    buf[0] = ( uint8_t )( ( freq >> 24 ) & 0xFF );
    buf[1] = ( uint8_t )( ( freq >> 16 ) & 0xFF );
    buf[2] = ( uint8_t )( ( freq >> 8 ) & 0xFF );
    buf[3] = ( uint8_t )( freq & 0xFF );
    SX126xWriteCommand( RADIO_SET_RFFREQUENCY, buf, 4 );
}





void SX126xSetDioIrqParams( uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask )
{
    uint8_t buf[8];

    buf[0] = ( uint8_t )( ( irqMask >> 8 ) & 0x00FF );
    buf[1] = ( uint8_t )( irqMask & 0x00FF );
    buf[2] = ( uint8_t )( ( dio1Mask >> 8 ) & 0x00FF );
    buf[3] = ( uint8_t )( dio1Mask & 0x00FF );
    buf[4] = ( uint8_t )( ( dio2Mask >> 8 ) & 0x00FF );
    buf[5] = ( uint8_t )( dio2Mask & 0x00FF );
    buf[6] = ( uint8_t )( ( dio3Mask >> 8 ) & 0x00FF );
    buf[7] = ( uint8_t )( dio3Mask & 0x00FF );
    SX126xWriteCommand( RADIO_CFG_DIOIRQ, buf, 8 );
}


void SX126xSetRfTxPower( int8_t power )
{
    SX126xSetTxParams( power, RADIO_RAMP_800_US );
}


void SX126xSetBufferBaseAddress( uint8_t txBaseAddress, uint8_t rxBaseAddress )
{
    uint8_t buf[2];

    buf[0] = txBaseAddress;
    buf[1] = rxBaseAddress;
    SX126xWriteCommand( RADIO_SET_BUFFERBASEADDRESS, buf, 2 );
}



/////CAD/////
void SX126xSetCad( void )
{
    SX126xWriteCommand( RADIO_SET_CAD, 0, 0 );
    OperatingMode = MODE_CAD;
}
void SX126xSetCadParams( RadioLoRaCadSymbols_t cadSymbolNum, uint8_t cadDetPeak, uint8_t cadDetMin, RadioCadExitModes_t cadExitMode, uint32_t cadTimeout )
{
    uint8_t buf[7];

    buf[0] = ( uint8_t )cadSymbolNum;
    buf[1] = cadDetPeak;
    buf[2] = cadDetMin;
    buf[3] = ( uint8_t )cadExitMode;
    buf[4] = ( uint8_t )( ( cadTimeout >> 16 ) & 0xFF );
    buf[5] = ( uint8_t )( ( cadTimeout >> 8 ) & 0xFF );
    buf[6] = ( uint8_t )( cadTimeout & 0xFF );
    SX126xWriteCommand( RADIO_SET_CADPARAMS, buf, 7 );
    OperatingMode = MODE_CAD;
}

	
	
void SX126xInit(void)
{
	SX126xReset( );

	//SX126xIoIrqInit( dioIrq );//中断回调在其他地方定义

	SX126xWakeup( );
	SX126xSetStandby( STDBY_RC );

#ifdef USE_TCXO
	CalibrationParams_t calibParam;

	SX126xSetDio3AsTcxoCtrl( TCXO_CTRL_3_0V, RADIO_TCXO_SETUP_TIME << 6 ); // convert from ms to SX126x time base
	calibParam.Value = 0x7F;
	SX126xCalibrate( calibParam );
#endif

	SX126xSetDio2AsRfSwitchCtrl( true );
	OperatingMode = MODE_STDBY_RC;
}



void SX126xSetDio3AsTcxoCtrl( RadioTcxoCtrlVoltage_t tcxoVoltage, uint32_t timeout )
{
    uint8_t buf[4];

    buf[0] = tcxoVoltage & 0x07;
    buf[1] = ( uint8_t )( ( timeout >> 16 ) & 0xFF );
    buf[2] = ( uint8_t )( ( timeout >> 8 ) & 0xFF );
    buf[3] = ( uint8_t )( timeout & 0xFF );

    SX126xWriteCommand( RADIO_SET_TCXOMODE, buf, 4 );
}


void SX126xCalibrateImage( uint32_t freq )
{
    uint8_t calFreq[2];

    if( freq > 900000000 )
    {
        calFreq[0] = 0xE1;
        calFreq[1] = 0xE9;
    }
    else if( freq > 850000000 )
    {
        calFreq[0] = 0xD7;
        calFreq[1] = 0xD8;
    }
    else if( freq > 770000000 )
    {
        calFreq[0] = 0xC1;
        calFreq[1] = 0xC5;
    }
    else if( freq > 460000000 )
    {
        calFreq[0] = 0x75;
        calFreq[1] = 0x81;
    }
    else if( freq > 425000000 )
    {
        calFreq[0] = 0x6B;
        calFreq[1] = 0x6F;
    }else if( freq > 410000000 )
    {
        calFreq[0] = 0x65;
        calFreq[1] = 0x68;
			printf("SX126xCalibrateImage 0x%x,0x%x\n",calFreq[0],calFreq[1]);
    }else if( freq > 407000000 )
    {
        calFreq[0] = 0x6B;
        calFreq[1] = 0x6F;
        printf("SX126xCalibrateImage 0x%x,0x%x\n",calFreq[0],calFreq[1]);
    }else if( freq > 400000000 )
    {
        calFreq[0] = 0x6B;
        calFreq[1] = 0x6F;
        printf("SX126xCalibrateImage 0x%x,0x%x\n",calFreq[0],calFreq[1]);
    }
    
    SX126xWriteCommand( RADIO_CALIBRATEIMAGE, calFreq, 2 );
}

void SX126xCalibrate( CalibrationParams_t calibParam )
{
    SX126xWriteCommand( RADIO_CALIBRATE, ( uint8_t* )&calibParam, 1 );
}

void SX126xReset( void )
{
   	HAL_Delay(10);
   	HAL_GPIO_WritePin(NRESET_GPIO_Port,NRESET_Pin,GPIO_PIN_RESET);
   	HAL_Delay(20);
   	HAL_GPIO_WritePin(NRESET_GPIO_Port,NRESET_Pin,GPIO_PIN_SET);
  	HAL_Delay(10);
}

void SX126xSetStandby( RadioStandbyModes_t standbyConfig )
{
    SX126xWriteCommand( RADIO_SET_STANDBY, ( uint8_t* )&standbyConfig, 1 );
    if( standbyConfig == STDBY_RC )
    {
        OperatingMode = MODE_STDBY_RC;
    }
    else
    {
        OperatingMode = MODE_STDBY_XOSC;
    }
}

void SX126xWaitOnBusy( void )
{
	while(HAL_GPIO_ReadPin(BUSY_GPIO_Port,BUSY_Pin)==GPIO_PIN_SET);

}
/*!
 * Nested interrupt counter.
 *
 * \remark Interrupt should only be fully disabled once the value is 0
 */
static uint8_t IrqNestLevel = 0;

void BoardDisableIrq( void )
{
    __disable_irq( );
    IrqNestLevel++;
}

void BoardEnableIrq( void )
{
  	IrqNestLevel--;
    if( IrqNestLevel == 0 )
    {
        __enable_irq( );
    }
}
void SX126xWakeup( void )
{
  BoardDisableIrq( );
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port,SPI_CS_Pin,GPIO_PIN_RESET);
	HW_SPI_InOut(RADIO_GET_STATUS);
	HW_SPI_InOut(0);
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port,SPI_CS_Pin,GPIO_PIN_SET);
  SX126xWaitOnBusy( );
  BoardEnableIrq( );
}

void SX126xAntSwOn( void )
{
	HAL_GPIO_WritePin(ANT_SWITCH_POWER_GPIO_Port,ANT_SWITCH_POWER_Pin,GPIO_PIN_SET);
}

void SX126xAntSwOff( void )
{
   HAL_GPIO_WritePin(ANT_SWITCH_POWER_GPIO_Port,ANT_SWITCH_POWER_Pin,GPIO_PIN_RESET);
}

RadioOperatingModes_t SX126xGetOperatingMode( void )
{
    return OperatingMode;
}
void SX126xCheckDeviceReady( void )
{
    if( ( SX126xGetOperatingMode( ) == MODE_SLEEP ) || ( SX126xGetOperatingMode( ) == MODE_RX_DC ) )
    {
        SX126xWakeup( );
        SX126xAntSwOn( );
    }
    SX126xWaitOnBusy( );
}


void SX126xSetDio2AsRfSwitchCtrl( uint8_t enable )
{
    SX126xWriteCommand( RADIO_SET_RFSWITCHMODE, &enable, 1 );
}




void SX126xSetSleep( SleepParams_t sleepConfig )
{
    SX126xAntSwOff( );
    SX126xWriteCommand( RADIO_SET_SLEEP, &sleepConfig.Value, 1 );
    OperatingMode = MODE_SLEEP;
}




void SX126xClearIrqStatus( uint16_t irq )
{
    uint8_t buf[2];
    buf[0] = ( uint8_t )( ( ( uint16_t )irq >> 8 ) & 0x00FF );
    buf[1] = ( uint8_t )( ( uint16_t )irq & 0x00FF );
    SX126xWriteCommand( RADIO_CLR_IRQSTATUS, buf, 2 );
}



uint16_t SX126xGetIrqStatus( void )
{
    uint8_t irqStatus[2];
    SX126xReadCommand( RADIO_GET_IRQSTATUS, irqStatus, 2 );
    return ( irqStatus[0] << 8 ) | irqStatus[1];
}

void SX126xSetRxTxFallbackMode( uint8_t fallbackMode )
{
    SX126xWriteCommand( RADIO_SET_TXFALLBACKMODE, &fallbackMode, 1 );
}


/////////////底层实现

void SX126xWriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    SX126xCheckDeviceReady( );
	  HAL_GPIO_WritePin(SPI_CS_GPIO_Port,SPI_CS_Pin,GPIO_PIN_RESET);
    HW_SPI_InOut(RADIO_WRITE_BUFFER);//就是WriteBuffer() 这个命令
	  HW_SPI_InOut(offset );
    for( uint16_t i = 0; i < size; i++ )
    {
		  HW_SPI_InOut( buffer[i] );
    }
  	HAL_GPIO_WritePin(SPI_CS_GPIO_Port,SPI_CS_Pin,GPIO_PIN_SET);
    SX126xWaitOnBusy( );
}

void SX126xReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    SX126xCheckDeviceReady( );
	  HAL_GPIO_WritePin(SPI_CS_GPIO_Port,SPI_CS_Pin,GPIO_PIN_RESET);
    HW_SPI_InOut(RADIO_READ_BUFFER);//就是ReadBuffer() 这个命令
	  HW_SPI_InOut(offset );
	  HW_SPI_InOut(0 ); 
  for( uint16_t i = 0; i < size; i++ )
   {
		   buffer[i] = HW_SPI_InOut( 0 );   
   }
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port,SPI_CS_Pin,GPIO_PIN_SET);
  SX126xWaitOnBusy( );
}
void SX126xWriteCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
	uint16_t i=0;
  SX126xCheckDeviceReady( );
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port,SPI_CS_Pin,GPIO_PIN_RESET);
	HW_SPI_InOut(( uint8_t )command);
	for( i = 0; i < size; i++ )
	{
		HW_SPI_InOut( buffer[i] );
	}
 HAL_GPIO_WritePin(SPI_CS_GPIO_Port,SPI_CS_Pin,GPIO_PIN_SET);
	if( command != RADIO_SET_SLEEP )
	{
			SX126xWaitOnBusy( );
	}
}

void SX126xReadCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
	uint16_t i;
  SX126xCheckDeviceReady( );
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port,SPI_CS_Pin,GPIO_PIN_RESET);
	HW_SPI_InOut(( uint8_t )command);
	HW_SPI_InOut(0x00);  
	for(  i= 0; i < size; i++ )
    {
        buffer[i] = HW_SPI_InOut( 0 );
    }
   	HAL_GPIO_WritePin(SPI_CS_GPIO_Port,SPI_CS_Pin,GPIO_PIN_SET);
    
	SX126xWaitOnBusy( );
}
//只是写一个reg
void SX126xWriteRegister( uint16_t address, uint8_t value )
{
    SX126xWriteRegisters( address, &value, 1 );
}

//可以连续写任意个寄存器，NSS在最后一个byte写完后拉高
void SX126xWriteRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
	uint16_t i;
  SX126xCheckDeviceReady( );
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port,SPI_CS_Pin,GPIO_PIN_RESET);
	HW_SPI_InOut(RADIO_WRITE_REGISTER);
	HW_SPI_InOut(( address & 0xFF00 ) >> 8);
	HW_SPI_InOut(address & 0x00FF);
	for( i = 0; i < size; i++ )
	{
			HW_SPI_InOut( buffer[i] );
	}
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port,SPI_CS_Pin,GPIO_PIN_SET);
  SX126xWaitOnBusy( );
}
//只读一个寄存器
uint8_t SX126xReadRegister( uint16_t address )
{
    uint8_t data;
    SX126xReadRegisters( address, &data, 1 );
    return data;
}

//可以连续读很多寄存器，NSS在最后一个byte写完后拉高
void SX126xReadRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
	uint16_t i;
	SX126xCheckDeviceReady( );
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port,SPI_CS_Pin,GPIO_PIN_RESET);
	HW_SPI_InOut(RADIO_READ_REGISTER);
	HW_SPI_InOut(( address & 0xFF00 ) >> 8);
	HW_SPI_InOut(address & 0x00FF);
	HW_SPI_InOut(0);
  for(i = 0; i < size; i++ )
   {
      buffer[i] = HW_SPI_InOut(0);
   }
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port,SPI_CS_Pin,GPIO_PIN_SET);
  SX126xWaitOnBusy( );
}

uint16_t HW_SPI_InOut( uint16_t txData )
{
  uint16_t rxData ;
  HAL_SPI_TransmitReceive( &hspi1, ( uint8_t * ) &txData, ( uint8_t* ) &rxData, 1, HAL_MAX_DELAY);	
  return rxData;
}




