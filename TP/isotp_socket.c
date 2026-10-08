#include "isotp_socket.h"
#include "can.h"
#include "isotp.h"
#include <stdint.h>
#include "uds_services.h"


/** Stores the System given Communication Socket */
static ISocket_t s_systemComSocket = {NULL};
/** Charon Session and Communication Control Receive Buffer */
static uint8_t s_receiveBuffer[RCV_BUFFER_SIZE];
/** Charon Session and Communication Control Transmit Buffer */
static uint8_t s_sendBuffer[TX_BUFFER_SIZE];


#define ISOTP_RCV_BUFFER_SIZE     512u
#define ISOTP_TX_BUFFER_SIZE      512u
#define CAN_RX_QUEUE_SIZE         16u

typedef struct
{
    uint32_t id;
    uint8_t len;
    uint8_t data[8];
} CanRxFrame_t;

static int32_t numAvailableBytes(void);
static int32_t receive(uint8_t *buf, uint32_t len);
static int32_t transmit(const uint8_t *buf, uint32_t len);
static bool enqueueCanFrame(uint32_t id, const uint8_t *data, uint8_t len);
static bool dequeueCanFrame(uint32_t *id, uint8_t *data, uint8_t *len);
static void processReceivedMessage (uint8_t *pBuffer, uint32_t length);

static IsoTpLink g_phylink;;
static uint8_t g_isotpPhyRecvBuf[ISOTP_RCV_BUFFER_SIZE];
static uint8_t g_isotpPhySendBuf[ISOTP_TX_BUFFER_SIZE];
static CanRxFrame_t g_canRxQueue[CAN_RX_QUEUE_SIZE];
static volatile uint8_t  g_canRxHead = 0u;
static volatile uint8_t  g_canRxTail = 0u;
static volatile uint32_t g_canRxDropped = 0u;
static volatile uint32_t g_canTxErrors = 0u;

void isotp_socket_init(void)
{
    g_canRxHead = 0u;
    g_canRxTail = 0u;
    g_canRxDropped = 0u;
    g_canTxErrors = 0u;
    isotp_init_link(
        &g_phylink,
        TX_ID,
        g_isotpPhySendBuf,
        sizeof(g_isotpPhySendBuf),
        g_isotpPhyRecvBuf,
        sizeof(g_isotpPhyRecvBuf));
}

void isotp_socket_task(void)
{
    uint32_t id;
    uint8_t len;
    uint8_t data[8];

    while (dequeueCanFrame(&id, data, &len))
    {
        /*
         * Physical requests may be multi-frame. Functional requests are only
         * accepted as single frames because ISO-TP flow control is physical.
         */
        if ((id == RX_ID) ||
            ((id == 0x7DFu) && ((data[0] & 0xF0u) == 0u)))
        {
            isotp_on_can_message(&g_phylink, data, len);
        }
    }

    isotp_poll(&g_phylink);
}

const ISocket_t isotp_socket =
{
    .numAvailableBytes = numAvailableBytes,
    .receive = receive,
    .transmit = transmit
};

static int32_t numAvailableBytes(void)
{
    if (g_phylink.receive_status == ISOTP_RECEIVE_STATUS_FULL)
    {
        return (int32_t)g_phylink.receive_size;
    }
    return 0;
}

static int32_t receive(uint8_t *buf, uint32_t len)
{
    uint32_t outSize = 0u;
    int result = isotp_receive(&g_phylink, buf, len, &outSize);
    return (result == ISOTP_RET_OK) ? (int32_t)outSize : (int32_t)result;
}

static int32_t transmit(const uint8_t *buf, uint32_t len)
{
    int result = isotp_send(&g_phylink, buf, len);
    if (result != ISOTP_RET_OK)
    {
        g_canTxErrors++;
    }
    return (result == ISOTP_RET_OK) ? (int32_t)len : (int32_t)result;
}

static bool enqueueCanFrame(uint32_t id, const uint8_t *data, uint8_t len)
{
    uint8_t nextHead = (uint8_t)((g_canRxHead + 1u) % CAN_RX_QUEUE_SIZE);

    if ((data == NULL) || (len > 8u))
    {
        return false;
    }

    if (nextHead == g_canRxTail)
    {
        g_canRxDropped++;
        return false;
    }

    g_canRxQueue[g_canRxHead].id = id;
    g_canRxQueue[g_canRxHead].len = len;
    memcpy(g_canRxQueue[g_canRxHead].data, data, len);
    __DMB();
    g_canRxHead = nextHead;
    return true;
}

static bool dequeueCanFrame(uint32_t *id, uint8_t *data, uint8_t *len)
{
    uint8_t tail = g_canRxTail;

    if ((tail == g_canRxHead) || (id == NULL) || (data == NULL) || (len == NULL))
    {
        return false;
    }

    *id = g_canRxQueue[tail].id;
    *len = g_canRxQueue[tail].len;
    memcpy(data, g_canRxQueue[tail].data, *len);
    __DMB();
    g_canRxTail = (uint8_t)((tail + 1u) % CAN_RX_QUEUE_SIZE);
    return true;
}

uint32_t TpGetDroppedFrameCount(void)
{
    return g_canRxDropped;
}

uint32_t TpGetTransmitErrorCount(void)
{
    return g_canTxErrors;
}

void TpRecordCanTxError(void)
{
    g_canTxErrors++;
}





/* Receive FIFO 0 message pending interrupt management */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, data) == HAL_OK)
    {
        if ((rxHeader.StdId == RX_ID) || (rxHeader.StdId == 0x7DFu))
        {
			(void)enqueueCanFrame(rxHeader.StdId, data, (uint8_t)rxHeader.DLC);
		}
    }

}

static void processReceivedMessage (uint8_t *pBuffer, uint32_t length)
{
	uint8_t response[3];
	// Identifier le service UDS bas� sur le premier octet du message
	switch (pBuffer[0]) {
		case UDS_DIAGNOSTIC_SESSION_CONTROL:
			// Appeler la fonction pour le service Diagnostic Session Control
			uds_diagnostic_session_control(pBuffer[1]);
			break;
	
		case UDS_ECU_RESET:
			// Appeler la fonction pour le service ECU Reset avec le resetType
			uds_ecu_reset(pBuffer[1]);
			break;
		case UDS_SECURITY_ACCESS:
			// Appeler directement la fonction pour le service Security Access
			uds_security_access(pBuffer[1], &pBuffer[2], length - 2);
			break;
		case UDS_COMMUNICATION_CONTROL:
			uds_communication_control(pBuffer[1]);
			break;
		case UDS_TESTER_PRESENT:
			// Appeler la fonction pour le service TesterPresent
			uds_tester_present(pBuffer[1]);
			break;
		case UDS_ACCESS_TIMING_PARAMETER:
			// Appeler la fonction pour le service Access Timing Parameter
			uds_access_timing_parameter(pBuffer[1], &pBuffer[2], length - 2);
			break;
		case UDS_SECURED_DATA_TRANSMISSION:
			// Appeler la fonction pour le service Secured Data Transmission
			uds_secured_data_transmission(&pBuffer[1], length - 1);
			break;
		case UDS_CONTROL_DTC_SETTING:
			// Appeler la fonction pour le service ControlDTCSetting
			uds_control_dtc_setting(pBuffer[1]);
			break;
		case UDS_RESPONSE_ON_EVENT:
			// Appeler la fonction pour le service ResponseOnEvent
			uds_response_on_event(pBuffer[1], &pBuffer[2], length - 2);
			break;
		case UDS_LINK_CONTROL:
			// Appeler la fonction pour le service LinkControl
			uds_link_control(pBuffer[1], &pBuffer[2], length - 2);
			break;
		case UDS_READ_DATA_BY_IDENTIFIER:
			// Appeler la fonction pour le service ReadDataByIdentifier
			uds_read_data_by_identifier(&pBuffer[1], length - 1);
			break;
		case UDS_READ_DATA_BY_PERIODIC_IDENTIFIER:
			// Appeler la fonction pour le service ReadDataByPeriodicIdentifier
			uds_read_data_by_periodic_identifier(&pBuffer[1], length - 1);
			break;
		case UDS_DYNAMICAL_DEFINE_DATA_IDENTIFIER:
			// Appeler la fonction pour le service DynamicallyDefineDataIdentifier
			uds_dynamically_define_data_identifier(pBuffer[1], &pBuffer[2], length - 2);
			break;
		case UDS_WRITE_DATA_BY_IDENTIFIER:
			// Appeler la fonction pour le service WriteDataByIdentifier
			uds_write_data_by_identifier(&pBuffer[1], length - 1);
			break;
		case UDS_CLEAR_DIAGNOSTIC_INFORMATION:
			// Appeler la fonction pour le service ClearDiagnosticInformation
			uds_clear_diagnostic_information(&pBuffer[1], length - 1);
			break;
		case UDS_READ_DTC_INFORMATION:
			// Appeler la fonction pour g�rer le service ReadDTCInformation
			uds_read_dtc_information(pBuffer[1], &pBuffer[2], length - 2);
			break;
		case UDS_INPUT_OUTPUT_CONTROL_BY_IDENTIFIER:
			uds_input_output_control_by_identifier((IOControlRequest_t *)&pBuffer[1], NULL);
			break;
		case UDS_ROUTINE_CONTROL:
			uds_routine_control((RoutineControlRequest_t *)&pBuffer[1], NULL);
			break;
		case UDS_REQUEST_DOWNLOAD:
			uds_request_download((RequestDownload_t *)&pBuffer[1]);
			break;
		case UDS_REQUEST_UPLOAD:
			uds_request_upload((RequestUpload_t *)&pBuffer[1]);
			break;
		case UDS_TRANSFER_DATA:
			uds_transfer_data((RequestTransferData_t *)&pBuffer[1]);
			break;
		case UDS_REQUEST_TRANSFER_EXIT:
			uds_request_transfer_exit((RequestTransferExit_t *)&pBuffer[1], NULL);
			break;
		case UDS_REQUEST_FILE_TRANSFER:
			uds_request_file_transfer((RequestFileTransfer_t *)&pBuffer[1]);
			 break;
	
		default:
		  // Remplir le message de r�ponse n�gative pour un service non support�
		  response[0] = UDS_NEGATIVE_RESPONSE;		   // R�ponse n�gative g�n�rique
		  response[1] = pBuffer[0];  // Service non support�
		  response[2] = NRC_SERVICE_NOT_SUPPORTED;		   // Code NRC (ServiceNotSupported)
	
		  // Envoyer le message de r�ponse n�gative via CAN
		  send_can_message(response, 3);
		 break;
	}
}

void uds_task(void)
{
    int32_t length;

    length = s_systemComSocket.receive(s_receiveBuffer, sizeof(s_receiveBuffer));

    /* Process Received Message if one was gotten and de-crypted */
    if(length > 0)
    {
        processReceivedMessage(s_receiveBuffer, (uint32_t)length);
    }
}


void sendMessage (const uint8_t * pUdsMessage, uint32_t length)
{
    uint32_t txLength;

    if(length >= TX_BUFFER_SIZE)     
     {
        txLength = TX_BUFFER_SIZE;
    }
    else
    {
        txLength = length;
    }

    /* Copy to Buffer and start transfer */
    memcpy(s_sendBuffer, pUdsMessage, txLength);
    (void)s_systemComSocket.transmit(s_sendBuffer, txLength);
}

void iso_init(ISocket_t sscComSocket)
{
    s_systemComSocket = sscComSocket;
}

