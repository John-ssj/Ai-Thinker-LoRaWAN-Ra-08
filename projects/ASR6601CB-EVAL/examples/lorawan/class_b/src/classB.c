/*!
 * \file      main.c
 *
 * \brief     LoRaMac classB device implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */

#include <stdio.h>
#include "utilities.h"
#include "LoRaMac.h"
#include "Commissioning.h"

#ifndef ACTIVE_REGION

#warning "No active region defined, LORAMAC_REGION_CN470 will be used as default."

#define ACTIVE_REGION LORAMAC_REGION_CN470

#endif

/*!
 * Defines the application data transmission duty cycle. 30s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            30000

/*!
 * Defines a random delay for application data transmission duty cycle. 5s,
 * value in [ms].
 */
#define APP_TX_DUTYCYCLE_RND                        5000

/*!
 * Default datarate
 */
#define LORAWAN_DEFAULT_DATARATE                    DR_0

/*!
 * Default ping slots periodicity
 *
 * \remark periodicity is equal to 2^LORAWAN_DEFAULT_PING_SLOT_PERIODICITY seconds
 *         example: 2^3 = 8 seconds. The end-device will open an Rx slot every 8 seconds.
 */
#define LORAWAN_DEFAULT_PING_SLOT_PERIODICITY       0

/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_CONFIRMED_MSG_ON                    false

/*!
 * LoRaWAN Adaptive Data Rate
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_ON                              1

/*!
 * LoRaWAN application port
 */
#define LORAWAN_APP_PORT                            2
   
// static uint8_t DevEui[] = LORAWAN_DEVICE_EUI;
// static uint8_t AppEui[] = LORAWAN_APPLICATION_EUI;
// static uint8_t AppKey[] = LORAWAN_APPLICATION_KEY;

static uint8_t DevEui[] = {                            \
        0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xD0, 0x20 \
    };
static uint8_t AppEui[] = {                            \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 \
    };
static uint8_t AppKey[] = {                            \
        0x52, 0x58, 0xCF, 0x37, 0x80, 0x5D, 0xFD, 0x3B, 0x7E, 0xA7, 0x24, 0x91, 0xAF, 0x3D, 0x60, 0x23 \
    };

#if( OVER_THE_AIR_ACTIVATION == 0 )

static uint8_t NwkSKey[] = LORAWAN_NWKSKEY;
static uint8_t AppSKey[] = LORAWAN_APPSKEY;
static uint32_t DevAddr = LORAWAN_DEVICE_ADDRESS;

#endif

/*!
 * Application port
 */
static uint8_t AppPort = LORAWAN_APP_PORT;

/*!
 * User application data size
 */
static uint8_t AppDataSize = 4;
/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_MAX_SIZE                           16

/*!
 * User application data
 */
static uint8_t AppData[LORAWAN_APP_DATA_MAX_SIZE];

/*!
 * Indicates if the node is sending confirmed or unconfirmed messages
 */
static uint8_t IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;

/*!
 * Defines the application data transmission duty cycle
 */
static uint32_t TxDutyCycleTime = APP_TX_DUTYCYCLE;

/*!
 * Timer to handle the application data transmission duty cycle
 */
static TimerEvent_t TxNextPacketTimer;

/*!
 * Indicates if a new packet can be sent
 */
static bool NextTx = true;

/*!
 * Device states
 */
static enum eDeviceState
{
    // DEVICE_STATE_RESTORE,
    // DEVICE_STATE_START,
    DEVICE_STATE_INIT,
    DEVICE_STATE_JOIN,
    DEVICE_STATE_SEND,
    DEVICE_STATE_REQ_DEVICE_TIME,
    DEVICE_STATE_REQ_PINGSLOT_ACK,
    DEVICE_STATE_REQ_BEACON_TIMING,
    DEVICE_STATE_BEACON_ACQUISITION,
    DEVICE_STATE_SWITCH_CLASS,
    DEVICE_STATE_CYCLE,
    DEVICE_STATE_SLEEP
}DeviceState, WakeUpState;


const char* DeviceStateStrings[] =
{
    "INIT",
    "JOIN",
    "SEND",
    "REQ_DEVICE_TIME",
    "REQ_PINGSLOT_ACK",
    "REQ_BEACON_TIMING",
    "BEACON_ACQUISITION",
    "SWITCH_CLASS",
    "CYCLE",
    "SLEEP"
};

/*!
 * Prints the provided buffer in HEX
 * 
 * \param buffer Buffer to be printed
 * \param size   Buffer size to be printed
 */
void PrintHexBuffer( uint8_t *buffer, uint8_t size )
{
    uint8_t newline = 0;

    for( uint8_t i = 0; i < size; i++ )
    {
        if( newline != 0 )
        {
            printf( "\r\n" );
            newline = 0;
        }

        printf( "%02X ", buffer[i] );

        if( ( ( i + 1 ) % 16 ) == 0 )
        {
            newline = 1;
        }
    }
    printf( "\r\n" );
}

/*!
 * \brief   Prepares the payload of the frame
 */
static void PrepareTxFrame( uint8_t port )
{
#ifdef MY_DEBUG1
    printf("PrepareTxFrame \r\n");
#endif
    AppDataSize = 4;
    AppData[0] = 0x00;
    AppData[1] = 0x01;
    AppData[2] = 0x02;
    AppData[3] = 0x03;
}

/*!
 * \brief   Prepares the payload of the frame
 *
 * \retval  [0: frame could be send, 1: error]
 */
static bool SendFrame( void )
{
#ifdef MY_DEBUG1
    printf( "SendFrame\r\n");
#endif
    McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;

    if( LoRaMacQueryTxPossible( AppDataSize, &txInfo ) != LORAMAC_STATUS_OK )
    {
        // Send empty frame in order to flush MAC commands
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fBuffer = NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;
        mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
    }
    else
    {
        if( IsTxConfirmed == false )
        {
            mcpsReq.Type = MCPS_UNCONFIRMED;
            mcpsReq.Req.Unconfirmed.fPort = AppPort;
            mcpsReq.Req.Unconfirmed.fBuffer = AppData;
            mcpsReq.Req.Unconfirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
        else
        {
            mcpsReq.Type = MCPS_CONFIRMED;
            mcpsReq.Req.Confirmed.fPort = AppPort;
            mcpsReq.Req.Confirmed.fBuffer = AppData;
            mcpsReq.Req.Confirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Confirmed.NbTrials = 8;
            mcpsReq.Req.Confirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
    }

    if( LoRaMacMcpsRequest( &mcpsReq ) == LORAMAC_STATUS_OK )
    {
        return false;
    }
    
    
    return true;
}

/*!
 * \brief Function executed on TxNextPacket Timeout event
 */
static void OnTxNextPacketTimerEvent( void )
{
#ifdef MY_DEBUG1
    printf( "OnTxNextPacketTimerEvent , DeviceState : %d %s\r\n", DeviceState, DeviceStateStrings[DeviceState]);
#endif
    MibRequestConfirm_t mibReq;
    LoRaMacStatus_t status;

    TimerStop( &TxNextPacketTimer );

    mibReq.Type = MIB_NETWORK_JOINED;
    status = LoRaMacMibGetRequestConfirm( &mibReq );

    if( status == LORAMAC_STATUS_OK )
    {
        if( mibReq.Param.IsNetworkJoined == true )
        {
            DeviceState = WakeUpState;
            NextTx = true;
        }
        else
        {
            // Network not joined yet. Try to join again
            MlmeReq_t mlmeReq;
            mlmeReq.Type = MLME_JOIN;
            mlmeReq.Req.Join.DevEui = DevEui;
            mlmeReq.Req.Join.AppEui = AppEui;
            mlmeReq.Req.Join.AppKey = AppKey;

            if( LoRaMacMlmeRequest( &mlmeReq ) == LORAMAC_STATUS_OK )
            {
                DeviceState = DEVICE_STATE_SLEEP;
            }
            else
            {
                DeviceState = DEVICE_STATE_CYCLE;
            }
        }
    }
}

/*!
 * \brief   MCPS-Confirm event function
 *
 * \param   [IN] mcpsConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void McpsConfirm( McpsConfirm_t *mcpsConfirm )
{
#ifdef MY_DEBUG1
    printf( "\r\n###### ===== McpsConfirm ==== ######\r\n" );
    printf( "Status : %d\r\n", mcpsConfirm->Status);
    printf( "DeviceState : %d %s\r\n", DeviceState, DeviceStateStrings[DeviceState]);
#endif
    if( mcpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
    {
        switch( mcpsConfirm->McpsRequest )
        {
            case MCPS_UNCONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                break;
            }
            case MCPS_CONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                // Check AckReceived
                // Check NbTrials
                break;
            }
            case MCPS_PROPRIETARY:
            {
                break;
            }
            default:
                break;
        }
    }
    NextTx = true;
}

/*!
 * \brief   MCPS-Indication event function
 *
 * \param   [IN] mcpsIndication - Pointer to the indication structure,
 *               containing indication attributes.
 */
static void McpsIndication( McpsIndication_t *mcpsIndication )
{
#ifdef MY_DEBUG1
    printf( "\r\n###### ===== McpsIndication ==== ######\r\n" );
    printf( "Status : %d\r\n", mcpsIndication->Status);
    printf( "DeviceState : %d %s\r\n", DeviceState, DeviceStateStrings[DeviceState]);
#endif
    if( mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK )
    {
        return;
    }

#ifdef MY_DEBUG1
    printf( "receive data: rssi = %d, snr = %d, datarate = %d\r\n", mcpsIndication->Rssi, (int)mcpsIndication->Snr,
                 (int)mcpsIndication->RxDatarate);
    printf("\r\n");
#endif
    switch( mcpsIndication->McpsIndication )
    {
        case MCPS_UNCONFIRMED:
        {
            break;
        }
        case MCPS_CONFIRMED:
        {
            break;
        }
        case MCPS_PROPRIETARY:
        {
            break;
        }
        case MCPS_MULTICAST:
        {
            break;
        }
        default:
            break;
    }

    // Check Multicast
    // Check Port
    // Check Datarate
    // Check FramePending
    if( mcpsIndication->FramePending == true )
    {
        // The server signals that it has pending data to be sent.
        // We schedule an uplink as soon as possible to flush the server.
        OnTxNextPacketTimerEvent( );
    }
    // Check Buffer
    // Check BufferSize
    // Check Rssi
    // Check Snr
    // Check RxSlot
    if( mcpsIndication->RxData == true )
    {
    }
}

/*!
 * \brief   MLME-Confirm event function
 *
 * \param   [IN] mlmeConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void MlmeConfirm( MlmeConfirm_t *mlmeConfirm )
{
    MibRequestConfirm_t mibReq;

#ifdef MY_DEBUG1
    printf( "\r\n###### ===== MLME-Confirm ==== ######\r\n" );
    printf( "STATUS      : %d\r\n", mlmeConfirm->Status);
    printf( "DeviceState : %d %s\r\n", DeviceState, DeviceStateStrings[DeviceState]);
#endif    
    switch( mlmeConfirm->MlmeRequest )
    {
        case MLME_JOIN:
        {
#ifdef MY_DEBUG1
            printf("MlmeConfirm -- MLME_JOIN\r\n");
#endif
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
#ifdef MY_DEBUG1
                printf("joined\r\n");
#endif
                // Status is OK, node has joined the network
#if defined( USE_BEACON_TIMING )
                DeviceState = DEVICE_STATE_REQ_BEACON_TIMING;
#else
                DeviceState = DEVICE_STATE_REQ_DEVICE_TIME;
#endif
            }
            else
            {
                MlmeReq_t mlmeReq;
#ifdef MY_DEBUG1
                printf("join failed\r\n");
#endif
                // Join was not successful. Try to join again
                mlmeReq.Type = MLME_JOIN;
                mlmeReq.Req.Join.DevEui = DevEui;
                mlmeReq.Req.Join.AppEui = AppEui;
                mlmeReq.Req.Join.AppKey = AppKey;
                mlmeReq.Req.Join.NbTrials = 8;

                if( LoRaMacMlmeRequest( &mlmeReq ) == LORAMAC_STATUS_OK )
                {
                    DeviceState = DEVICE_STATE_SLEEP;
                }
                else
                {
                    DeviceState = DEVICE_STATE_CYCLE;
                }
            }
            break;
        }
        case MLME_LINK_CHECK:
        {
#ifdef MY_DEBUG1
            printf("MlmeConfirm -- MLME_LINK_CHECK\r\n");
#endif
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                // Check DemodMargin
                // Check NbGateways
            }
            break;
        }
        case MLME_DEVICE_TIME:
        {
#ifdef MY_DEBUG1
            printf("MlmeConfirm -- MLME_DEVICE_TIME\r\n");
#endif
            // Setup the WakeUpState to DEVICE_STATE_SEND. This allows the
            // application to initiate MCPS requests during a beacon acquisition
            WakeUpState = DEVICE_STATE_SEND;
            // Switch to the next state immediately
            DeviceState = DEVICE_STATE_BEACON_ACQUISITION;
            NextTx = true;
            break;
        }
        case MLME_BEACON_TIMING:
        {
#ifdef MY_DEBUG1
            printf("MlmeConfirm -- MLME_BEACON_TIMING\r\n");
#endif
            // Setup the WakeUpState to DEVICE_STATE_SEND. This allows the
            // application to initiate MCPS requests during a beacon acquisition
            WakeUpState = DEVICE_STATE_SEND;
            // Switch to the next state immediately
            DeviceState = DEVICE_STATE_BEACON_ACQUISITION;
            NextTx = true;
            break;
        }
        case MLME_BEACON_ACQUISITION:
        {
#ifdef MY_DEBUG1
            printf("MlmeConfirm -- MLME_BEACON_ACQUISITION\r\n");
#endif
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
#ifdef MY_DEBUG1
                printf("\r\nWakeUpState: DEVICE_STATE_REQ_PINGSLOT_ACK\r\n\r\n");
#endif
                WakeUpState = DEVICE_STATE_REQ_PINGSLOT_ACK;
            }
            else
            {
#if defined( USE_BEACON_TIMING )
#ifdef MY_DEBUG1
                printf("\r\nWakeUpState: DEVICE_STATE_REQ_BEACON_TIMING\r\n\r\n");
#endif
                WakeUpState = DEVICE_STATE_REQ_BEACON_TIMING;
#else
#ifdef MY_DEBUG1
                printf("\r\nWakeUpState: DEVICE_STATE_REQ_DEVICE_TIME\r\n\r\n");
#endif
                WakeUpState = DEVICE_STATE_REQ_DEVICE_TIME;
#endif
            }
            break;
        }
        case MLME_PING_SLOT_INFO:
        {
#ifdef MY_DEBUG1
            printf("MlmeConfirm -- MLME_PING_SLOT_INFO\r\n");
            printf("mlmeConfirm->Status: %d\r\n", mlmeConfirm->Status);
#endif
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                mibReq.Type = MIB_DEVICE_CLASS;
                mibReq.Param.Class = CLASS_B;
                LoRaMacMibSetRequestConfirm( &mibReq );
#ifdef MY_DEBUG1
                printf( "\r\n\r\n###### ===== Switch to Class B done. ==== ######\r\n\r\n" );
#endif
                WakeUpState = DEVICE_STATE_SEND;
                DeviceState = WakeUpState;
                NextTx = true;
            }
            else
            {
#ifdef MY_DEBUG1
                printf("\r\nWakeUpState: DEVICE_STATE_REQ_PINGSLOT_ACK\r\n\r\n");
#endif
                WakeUpState = DEVICE_STATE_REQ_PINGSLOT_ACK;
            }
            break;
        }
        default:
            break;
    }
    NextTx = true;
}

/*!
 * \brief   MLME-Indication event function
 *
 * \param   [IN] mlmeIndication - Pointer to the indication structure.
 */
static void MlmeIndication( MlmeIndication_t *mlmeIndication )
{
    MibRequestConfirm_t mibReq;

    if( mlmeIndication->Status != LORAMAC_EVENT_INFO_STATUS_BEACON_LOCKED )
    {
#ifdef MY_DEBUG1
        printf( "\r\n###### ===== MLME-Indication ==== ######\r\n" );
        printf( "STATUS      : %d\r\n", mlmeIndication->Status);
        printf( "DeviceState : %d %s\r\n", DeviceState, DeviceStateStrings[DeviceState]);
#endif
    }
    switch( mlmeIndication->MlmeIndication )
    {
        case MLME_SCHEDULE_UPLINK:
        {// The MAC signals that we shall provide an uplink as soon as possible
            printf("MlmeIndication -- MLME_SCHEDULE_UPLINK\r\n");
            OnTxNextPacketTimerEvent( );
            break;
        }
        case MLME_BEACON_LOST:
        {
#ifdef MY_DEBUG1
            printf("MlmeIndication -- MLME_BEACON_LOST\r\n");
#endif
            mibReq.Type = MIB_DEVICE_CLASS;
            mibReq.Param.Class = CLASS_A;
            LoRaMacMibSetRequestConfirm( &mibReq );
#ifdef MY_DEBUG1
            printf( "\r\n\r\n###### ===== Switch to Class A done. ==== ######\r\n\r\n" );
#endif
            // Switch to class A again
#if defined( USE_BEACON_TIMING )
#ifdef MY_DEBUG1
            printf("\r\nWakeUpState: DEVICE_STATE_REQ_BEACON_TIMING\r\n\r\n");
#endif
            WakeUpState = DEVICE_STATE_REQ_BEACON_TIMING;
#else
#ifdef MY_DEBUG1
            printf("\r\nWakeUpState: DEVICE_STATE_REQ_DEVICE_TIME\r\n\r\n");
#endif
            WakeUpState = DEVICE_STATE_REQ_DEVICE_TIME;
#endif
            // TimerStop( &LedBeaconTimer );
#ifdef MY_DEBUG1
            printf( "\r\n###### ===== BEACON LOST ==== ######\r\n" );
#endif
            break;
        }
        case MLME_BEACON:
        {
#ifdef MY_DEBUG1
            printf("MlmeIndication -- MLME_BEACON\r\n");
#endif
            if( mlmeIndication->Status == LORAMAC_EVENT_INFO_STATUS_BEACON_LOCKED )
            {
                // TimerStart( &LedBeaconTimer );
#ifdef MY_DEBUG1
                printf( "\r\n###### ===== BEACON %lu ==== ######\r\n", mlmeIndication->BeaconInfo.Time );
                printf( "GW DESC     : %d\r\n", mlmeIndication->BeaconInfo.GwSpecific.InfoDesc );
                printf( "GW INFO     : " );
                PrintHexBuffer( mlmeIndication->BeaconInfo.GwSpecific.Info, 6 );
                printf( "\r\n" );
                printf( "FREQ        : %lu\r\n", mlmeIndication->BeaconInfo.Frequency );
                printf( "DATA RATE   : DR_%d\r\n", mlmeIndication->BeaconInfo.Datarate );
                printf( "RX RSSI     : %d\r\n", mlmeIndication->BeaconInfo.Rssi );
                printf( "RX SNR      : %d\r\n", mlmeIndication->BeaconInfo.Snr );
                printf( "\r\n" );
#endif
            }
            else
            {
                // TimerStop( &LedBeaconTimer );
#ifdef MY_DEBUG1
                printf( "\r\n###### ===== BEACON NOT RECEIVED ==== ######\r\n" );
#endif
            }
            break;
        }
        default:
            break;
    }
}

static void lwan_dev_params_update( void )
{
    MibRequestConfirm_t mibReq;
    uint16_t channelsMaskTemp[6];
    channelsMaskTemp[0] = 0x00FF;
    channelsMaskTemp[1] = 0x0000;
    channelsMaskTemp[2] = 0x0000;
    channelsMaskTemp[3] = 0x0000;
    channelsMaskTemp[4] = 0x0000;
    channelsMaskTemp[5] = 0x0000;

    mibReq.Type = MIB_CHANNELS_DEFAULT_MASK;
    mibReq.Param.ChannelsDefaultMask = channelsMaskTemp;
    LoRaMacMibSetRequestConfirm(&mibReq);
    mibReq.Type = MIB_CHANNELS_MASK;
    mibReq.Param.ChannelsMask = channelsMaskTemp;
    LoRaMacMibSetRequestConfirm(&mibReq);
}

uint8_t BoardGetBatteryLevel( void )
{
#ifdef MY_DEBUG1
    printf("BoardGetBatteryLevel\r\n");
#endif
    return 0;
}

float BoardGetTemperatureLevel( void )
{
#ifdef MY_DEBUG1
    printf("BoardGetTemperatureLevel\r\n");
#endif
    return 25;
}

/**
 * Main application entry point.
 */
int app_start( void )
{
    LoRaMacPrimitives_t LoRaMacPrimitives;
    LoRaMacCallback_t LoRaMacCallbacks;
    MibRequestConfirm_t mibReq;

    DeviceState = DEVICE_STATE_INIT;
    WakeUpState = DEVICE_STATE_INIT;

    printf("ClassB app start\r\n");
    while( 1 )
    {
        if( DeviceState != 9 ) {
            printf( "main cycle , DeviceState : %d %s\r\n", DeviceState, DeviceStateStrings[DeviceState]);
        }
    
        switch( DeviceState )
        {
            case DEVICE_STATE_INIT:
            {
                LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
                LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
                LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;
                LoRaMacPrimitives.MacMlmeIndication = MlmeIndication;
                LoRaMacCallbacks.GetBatteryLevel = BoardGetBatteryLevel;
                LoRaMacCallbacks.GetTemperatureLevel = BoardGetTemperatureLevel;
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, ACTIVE_REGION );

                TimerInit( &TxNextPacketTimer, OnTxNextPacketTimerEvent );

                mibReq.Type = MIB_ADR;
                mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_PUBLIC_NETWORK;
                mibReq.Param.EnablePublicNetwork = LORAWAN_PUBLIC_NETWORK;
                LoRaMacMibSetRequestConfirm( &mibReq );
                
                lwan_dev_params_update();
                
                DeviceState = DEVICE_STATE_JOIN;
                break;
            }
            case DEVICE_STATE_JOIN:
            {
#if( OVER_THE_AIR_ACTIVATION != 0 )
                MlmeReq_t mlmeReq;

                // Initialize LoRaMac device unique ID
                //BoardGetUniqueId( DevEui );

                mlmeReq.Type = MLME_JOIN;

                mlmeReq.Req.Join.DevEui = DevEui;
                mlmeReq.Req.Join.AppEui = AppEui;
                mlmeReq.Req.Join.AppKey = AppKey;
                mlmeReq.Req.Join.NbTrials = 8;

                if( LoRaMacMlmeRequest( &mlmeReq ) == LORAMAC_STATUS_OK )
                {
                    DeviceState = DEVICE_STATE_SLEEP;
                }
                else
                {
                    DeviceState = DEVICE_STATE_CYCLE;
                }
#else

                mibReq.Type = MIB_NET_ID;
                mibReq.Param.NetID = LORAWAN_NETWORK_ID;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_DEV_ADDR;
                mibReq.Param.DevAddr = DevAddr;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_NWK_SKEY;
                mibReq.Param.NwkSKey = NwkSKey;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_APP_SKEY;
                mibReq.Param.AppSKey = AppSKey;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_NETWORK_JOINED;
                mibReq.Param.IsNetworkJoined = true;
                LoRaMacMibSetRequestConfirm( &mibReq );

#if defined( USE_BEACON_TIMING )
                DeviceState = DEVICE_STATE_REQ_BEACON_TIMING;
#else
                DeviceState = DEVICE_STATE_REQ_DEVICE_TIME;
#endif
#endif
                break;
            }
            case DEVICE_STATE_REQ_DEVICE_TIME:
            {
                MlmeReq_t mlmeReq;

                if( NextTx == true )
                {
                    mlmeReq.Type = MLME_DEVICE_TIME;

                    if( LoRaMacMlmeRequest( &mlmeReq ) == LORAMAC_STATUS_OK )
                    {
                        WakeUpState = DEVICE_STATE_SEND;
                    }
                }
                DeviceState = DEVICE_STATE_SEND;
                break;
            }
            case DEVICE_STATE_REQ_BEACON_TIMING:
            {
                MlmeReq_t mlmeReq;

                if( NextTx == true )
                {
                    mlmeReq.Type = MLME_BEACON_TIMING;

                    if( LoRaMacMlmeRequest( &mlmeReq ) == LORAMAC_STATUS_OK )
                    {
                        WakeUpState = DEVICE_STATE_SEND;
                    }
                }
                DeviceState = DEVICE_STATE_SEND;
                break;
            }
            case DEVICE_STATE_BEACON_ACQUISITION:
            {
                MlmeReq_t mlmeReq;

                if( NextTx == true )
                {
                    mlmeReq.Type = MLME_BEACON_ACQUISITION;

                    LoRaMacMlmeRequest( &mlmeReq );
                    NextTx = false;
                }
                DeviceState = DEVICE_STATE_SEND;
                break;
            }
            case DEVICE_STATE_REQ_PINGSLOT_ACK:
            {
                MlmeReq_t mlmeReq;

                if( NextTx == true )
                {
                    mlmeReq.Type = MLME_LINK_CHECK;
                    LoRaMacMlmeRequest( &mlmeReq );

                    mlmeReq.Type = MLME_PING_SLOT_INFO;
                    mlmeReq.Req.PingSlotInfo.PingSlot.Fields.Periodicity = LORAWAN_DEFAULT_PING_SLOT_PERIODICITY;
                    mlmeReq.Req.PingSlotInfo.PingSlot.Fields.RFU = 0;

                    if( LoRaMacMlmeRequest( &mlmeReq ) == LORAMAC_STATUS_OK )
                    {
                        WakeUpState = DEVICE_STATE_SEND;
                    }
                }
                DeviceState = DEVICE_STATE_SEND;
                break;
            }
            case DEVICE_STATE_SEND:
            {
                if( NextTx == true )
                {
                    PrepareTxFrame( AppPort );

                    NextTx = SendFrame( );
                }
                
                // Schedule next packet transmission
                TxDutyCycleTime = APP_TX_DUTYCYCLE + randr( 0, APP_TX_DUTYCYCLE_RND );
                DeviceState = DEVICE_STATE_CYCLE;
                break;
            }
            case DEVICE_STATE_CYCLE:
            {
                DeviceState = DEVICE_STATE_SLEEP;

                // Schedule next packet transmission
                TimerSetValue( &TxNextPacketTimer, TxDutyCycleTime );
                TimerStart( &TxNextPacketTimer );
                break;
            }
            case DEVICE_STATE_SLEEP:
            {
                // Wake up through events
                TimerLowPowerHandler( );
                
                // Process Radio IRQ
                Radio.IrqProcess( );
                break;
            }
            default:
            {
                DeviceState = DEVICE_STATE_INIT;
                break;
            }
        }
    }
}
