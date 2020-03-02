#include "can_comm.h"
#include "can.h"
#include "stdbool.h"

#define CAN_HEARTBEAT_ID    0x700
#define CAN_SDO_TX_ID       0x600
#define CAN_SDO_RX_ID       0x580

uint8_t CAN_TARGETNODEID   = 1;     /**!< ���ID�� */

uint32_t get_position = 0;          /**!< ���ö�ȡλ�ýӿڶ�ȡ����λ�� */
uint32_t get_velocity = 0;          /**!< ���ö�ȡ�ٶȽӿڶ�ȡ�����ٶ� */
float get_current = 0;              /**!< ���ö�ȡ�����ӿڶ�ȡ���ĵ��� */


typedef struct
{
    CAN_RxHeaderTypeDef RxHead;     /**!< canͨ��Э��ͷ */
    uint8_t data[8];                /**!< canͨ�Ž��յ������ݰ� */
    uint8_t len;                    /**!< canͨ�Ž��յ������ݰ�����*/
    bool refeshFlag;
}CanRxMessage_t;

static CanRxMessage_t CanRxMsg;

/**
  * @brief  CAN�ӿ��˲�����
  * @param
  * @retval 
  */
static void _CanFilter(void)
{
    CAN_FilterTypeDef   sCAN_Filter;
    
    sCAN_Filter.FilterBank = 0;                         /* ָ��������ʼ���Ĺ����� */  
    sCAN_Filter.FilterMode = CAN_FILTERMODE_IDMASK;     /* ����ģʽΪ����λģʽ */
    sCAN_Filter.FilterScale = CAN_FILTERSCALE_32BIT;    /* ָ���˲����Ĺ�ģ */
    sCAN_Filter.FilterIdHigh = 0;
    sCAN_Filter.FilterIdLow = 0;             
    sCAN_Filter.FilterMaskIdHigh = 0;
    sCAN_Filter.FilterMaskIdLow = 0;
    sCAN_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sCAN_Filter.FilterActivation = ENABLE;              /* ���û���ù����� */
    sCAN_Filter.SlaveStartFilterBank = 14;              /* ѡ�������ӹ������� */
    
    HAL_CAN_ConfigFilter(&hcan, &sCAN_Filter);
}

/**
  * @brief  CAN�ӿڳ�ʼ��
  * @param
  * @retval 
  */
void CanComm_Init(void)
{
    _CanFilter();
    HAL_CAN_Start(&hcan);               /* ����CANͨ�� */  
    HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING);    /* ���������ж����� */
}


/**
  * @brief  CAN��������
  * @param
  * @retval 
  */
static int8_t CAN_Transmit(uint32_t nodeid, uint32_t data_size, uint8_t* data)
{
    CAN_TxHeaderTypeDef TxHeader;     /**!< canͨ�ŷ���Э��ͷ */
    uint32_t canTxMailbox;
    
    if((data_size >8) || (data == NULL))    return -1;
    
	TxHeader.StdId  = CAN_SDO_TX_ID + nodeid;
    TxHeader.IDE    = CAN_ID_STD;               /* ָ����Ҫ������Ϣ�ı�ʶ������ */
    TxHeader.RTR    = CAN_RTR_DATA;             /* ָ����Ϣ����֡���� */
	TxHeader.DLC    = data_size;

	// check whether there is available mailbox
	if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
	{
		return -2;
	}

	HAL_CAN_AddTxMessage(& hcan, &TxHeader, data, &canTxMailbox);

	return 0;
}

/* Read and Write position, speed and status
   All the nodeid of following functions is target can node id, not the source can node id.
*/

/**
  * @brief  ����CAN����ģʽ
  * @param  
  * @retval 
  */
void CANComm_SetMode(uint32_t nodeid, uint8_t mode)
{
	uint8_t cmd[8];
	
	/* set the position mode to be relative */
	/* donot use absolute position! */
	cmd[0] = 0x2F;          /* дһ���ֽ� */
	cmd[1] = 0x60;          /* ��������0x6060 */
	cmd[2] = 0x60;
	cmd[3] = 0x00;          /* ������0x00    */
	if (mode == 0)
    {
		cmd[4] = 0x01;      /* position mode */
    }
	else
    {
		cmd[4] = 0x03;      /* velocity mode */
    }
	cmd[5] = 0x00;
	cmd[6] = 0x00;
	cmd[7] = 0x00;
	
	CAN_Transmit(nodeid, 8, cmd);
}


void CANComm_SetPostion(uint32_t nodeid, int position)
{
	uint8_t cmd[8];
	
	/* set the position */
	cmd[0] = 0x23;          /* д4���ֽ� */
	cmd[1] = 0x7A;          /* ��������0x607A */
	cmd[2] = 0x60;
	cmd[3] = 0x00;          /* ������0x00 */
	cmd[4] = (uint8_t)((uint32_t)position&0xFF);
	cmd[5] = (uint8_t)((uint32_t)(position>>8)&0xFF);
	cmd[6] = (uint8_t)((uint32_t)(position>>16)&0xFF);
	cmd[7] = (uint8_t)((uint32_t)(position>>24)&0xFF);
	
	CAN_Transmit(nodeid, 8, cmd);
}


void CANComm_SetVelocity(uint32_t nodeid, int velocity)
{
	uint8_t cmd[8];
	
	/* set the velocity */
	if (velocity == 0)
	{
		/* send the stop command */
		cmd[0] = 0x2B;      /* д2���ֽ� */
		cmd[1] = 0x40;      /* ��������ox6040 */
		cmd[2] = 0x60;
		cmd[3] = 0x00;      /* ������0x00 */
		cmd[4] = 0x44;      
		cmd[5] = 0x00;
		cmd[6] = 0x00;
		cmd[7] = 0x00;

	}
	else {
		cmd[0] = 0x23;      /* д4���ֽ� */
		cmd[1] = 0xFF;      /* ��������0x60FF */
		cmd[2] = 0x60;
		cmd[3] = 0x00;      /* ������0x00 */
		cmd[4] = (uint8_t)((uint32_t)velocity&0xFF);
		cmd[5] = (uint8_t)((uint32_t)(velocity>>8)&0xFF);
		cmd[6] = (uint8_t)((uint32_t)(velocity>>16)&0xFF);
		cmd[7] = (uint8_t)((uint32_t)(velocity>>24)&0xFF);
	}
	
	CAN_Transmit(nodeid, 8, cmd);
}


void CANComm_GetPosition(uint32_t nodeid)
{
	uint8_t cmd[8];

	/* set the velocity mode to be relative */
	cmd[0] = 0x40;      /* ������ */
	cmd[1] = 0x64;      /* ��������0x6064 */
	cmd[2] = 0x60;
	cmd[3] = 0x00;      /* ������0x00 */
	cmd[4] = 0x00;
	cmd[5] = 0x00;
	cmd[6] = 0x00;
	cmd[7] = 0x00;
	
	CAN_Transmit(nodeid, 8, cmd);
}


void CANComm_GetVelocity(uint32_t nodeid)
{
	uint8_t cmd[8];
    
	/* set the velocity mode to be relative */
	cmd[0] = 0x40;      /* ������ */
	cmd[1] = 0x6C;      /* ��������0x606C */
	cmd[2] = 0x60;
	cmd[3] = 0x00;      /* ������0x00 */
	cmd[4] = 0x00;
	cmd[5] = 0x00;
	cmd[6] = 0x00;
	cmd[7] = 0x00;
	
	CAN_Transmit(nodeid, 8, cmd);
}


void CANComm_GetCurrent(uint32_t nodeid)
{
	uint8_t cmd[8];

	/* set the velocity mode to be relative */
	cmd[0] = 0x40;      /* ������ */
	cmd[1] = 0x78;      /* ��������0x6078 */
	cmd[2] = 0x60;      
	cmd[3] = 0x00;      /* ������0x00 */
	cmd[4] = 0x00;
	cmd[5] = 0x00;
	cmd[6] = 0x00;
	cmd[7] = 0x00;
	
	CAN_Transmit(nodeid, 8, cmd);
}


static void CanComm_Process(void)
{
    /* CAN_SDO_RX_ID Process */
    if(CanRxMsg.refeshFlag)
    {
        if ((CanRxMsg.RxHead.StdId == CAN_SDO_RX_ID + CAN_TARGETNODEID) && (CanRxMsg.RxHead.IDE == CAN_ID_STD))
        {
            switch(CanRxMsg.data[2]<<8 | CanRxMsg.data[1])
            {
                case 0x6064:    /* ��ȡ��λ�� */
                    get_position = (uint32_t)(*(&CanRxMsg.data[4]));
                	get_position |= ((uint32_t)(CanRxMsg.data[4]))&0xFF;
                    get_position |= (((uint32_t)CanRxMsg.data[5])&0xFF)<<8;
                    get_position |= (((uint32_t)CanRxMsg.data[6])&0xFF)<<16;
                    get_position |= (((uint32_t)CanRxMsg.data[7])&0xFF)<<24;
                    break;
                case 0x606c:    /* ��ȡ���ٶ� */
                    get_velocity = (uint32_t)(*(&CanRxMsg.data[4]));
                	get_velocity |= ((uint32_t)(CanRxMsg.data[4]))&0xFF;
                    get_velocity |= (((uint32_t)CanRxMsg.data[5])&0xFF)<<8;
                    get_velocity |= (((uint32_t)CanRxMsg.data[6])&0xFF)<<16;
                    get_velocity |= (((uint32_t)CanRxMsg.data[7])&0xFF)<<24;
                    break;
                case 0x6078:    /* ��ȡ�ĵ��� */
                    get_current = 0.01f *((CanRxMsg.data[5] | CanRxMsg.data[4]));
                    break;
                default:
                    break;
            }
        }
        CanRxMsg.refeshFlag = false;
    }
}

/**
  * @brief  CAN�жϽ�������
  * @param
  * @retval 
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    /* Get RX message */
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CanRxMsg.RxHead, CanRxMsg.data);
    CanRxMsg.refeshFlag = true;
    
    CanComm_Process();
}


