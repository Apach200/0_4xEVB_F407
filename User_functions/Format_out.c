/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Format_out.c
  * @brief          : User Format output functions
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/

//#define CO_Aliex_Disco407green	0x3A



/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
//uint32_t* pMessage SDO_abortCode_to_String(CO_SDO_abortCode_t Code);


uint32_t* pMessage SDO_abortCode_to_String(CO_SDO_abortCode_t Code)
{
	/// SDO abort codes
const char Message_SDO_AB_NONE[]="CO_SDO_AB_NONE = 0x00000000UL  /* No abort */";	
const char Message_SDO_AB_TOGGLE_BIT[]="CO_SDO_AB_TOGGLE_BIT = 0x05030000UL /* Toggle bit not altered */";	
const char Message_SDO_AB_TIMEOUT[]="CO_SDO_AB_TIMEOUT = 0x05040000UL /* SDO protocol timed out */";	
const char Message_SDO_AB_CMD[]="CO_SDO_AB_CMD = 0x05040001UL  /*Command specifier not valid or unknown */";	
const char Message_SDO_AB_BLOCK_SIZE[]="CO_SDO_AB_BLOCK_SIZE = 0x05040002UL /* Invalid block size in block mode */";	
const char Message_SDO_AB_SEQ_NUM[]="CO_SDO_AB_SEQ_NUM = 0x05040003UL /* Invalid sequence number in block mode */";	
const char Message_SDO_AB_CRC[]="CO_SDO_AB_CRC = 0x05040004UL /*  CRC error (block mode only) */";	
const char Message_SDO_AB_OUT_OF_MEM[]="CO_SDO_AB_OUT_OF_MEM = 0x05040005UL /* Out of memory */";	
const char Message_SDO_AB_UNSUPPORTED_ACCESS[]="CO_SDO_AB_UNSUPPORTED_ACCESS = 0x06010000UL /* Unsupported access to an object */";	
const char Message_SDO_AB_WRITEONLY[]="CO_SDO_AB_WRITEONLY = 0x06010001UL /*  Attempt to read a write only object */";	
const char Message_SDO_AB_READONLY[]="CO_SDO_AB_READONLY = 0x06010002UL /* Attempt to write a read only object */";	
const char Message_SDO_AB_NOT_EXIST[]="CO_SDO_AB_NOT_EXIST = 0x06020000UL /* Object does not exist in the object dictionary */";	
const char Message_SDO_AB_NO_MAP[]="CO_SDO_AB_NO_MAP = 0x06040041UL /*  Object cannot be mapped to the PDO */";	
const char Message_SDO_AB_MAP_LEN[]="CO_SDO_AB_MAP_LEN = 0x06040042UL /* Number and length of object to be mapped exceeds PDO length does not match */*/";	
const char Message_SDO_AB_PRAM_INCOMPAT[]="CO_SDO_AB_PRAM_INCOMPAT = 0x06040043UL /* General parameter incompatibility reasons */";	
const char Message_SDO_AB_DEVICE_INCOMPAT[]="CO_SDO_AB_DEVICE_INCOMPAT = 0x06040047UL /* General internal incompatibility in device */";	
const char Message_SDO_AB_HW[]="CO_SDO_AB_HW = 0x06060000UL /* Access failed due to hardware error */";	
const char Message_SDO_AB_TYPE_MISMATCH[]="CO_SDO_AB_TYPE_MISMATCH = 0x06070010UL /* Data type does not match, length of service parameter";	
const char Message_SDO_AB_DATA_LONG[]="CO_SDO_AB_DATA_LONG = 0x06070012UL /* Data type does not match, length of service parameter too high */";	
const char Message_SDO_AB_DATA_SHORT[]="CO_SDO_AB_DATA_SHORT = 0x06070013UL /* Data type does not match, length of service parameter too short */";	
const char Message_SDO_AB_SUB_UNKNOWN[]="CO_SDO_AB_SUB_UNKNOWN = 0x06090011UL /* Sub index does not exist */";	
const char Message_SDO_AB_INVALID_VALUE[]="CO_SDO_AB_INVALID_VALUE = 0x06090030UL /* Invalid value for parameter (download only). */";	
const char Message_SDO_AB_VALUE_HIGH[]="CO_SDO_AB_VALUE_HIGH = 0x06090031UL /* Value range of parameter written too high */";	
const char Message_SDO_AB_VALUE_LOW[]="CO_SDO_AB_VALUE_LOW = 0x06090032UL /* Value range of parameter written too low */";	
const char Message_SDO_AB_MAX_LESS_MIN[]="CO_SDO_AB_MAX_LESS_MIN = 0x06090036UL /* Maximum value is less than minimum value. */";	
const char Message_SDO_AB_NO_RESOURCE[]="CO_SDO_AB_NO_RESOURCE = 0x060A0023UL /* Resource not available: SDO connection */";
const char Message_SDO_AB_GENERAL[]="CO_SDO_AB_GENERAL = 0x08000000UL /* General error */";	
const char Message_SDO_AB_DATA_TRANSF[]="CO_SDO_AB_DATA_TRANSF = 0x08000020UL /* Data cannot be transferred or stored to application */";	
const char Message_SDO_AB_DATA_LOC_CTRL[]="CO_SDO_AB_DATA_LOC_CTRL = 0x08000021UL /* Data cannot be transferred or stored to application because of local control */";
const char Message_SDO_AB_DATA_DEV_STATE[]="CO_SDO_AB_DATA_DEV_STATE = 0x08000022UL /*Data cannot be transferred or stored to application because of present device state */";	
const char Message_SDO_AB_DATA_OD[]="CO_SDO_AB_DATA_OD = 0x08000023UL /* Object dictionary not present or dynamic generation fails */";	
const char Message_SDO_AB_NO_DATA[]="CO_SDO_AB_NO_DATA = 0x08000024UL /* No data available */";

//The abort codes not listed above are reserved.
const char Message_Default[]="UNKNOUN CODE";
	

	
switch((uint32_t )Code)
{
case CO_SDO_AB_NONE:				return  Message_SDO_AB_NONE;				break;
case CO_SDO_AB_TOGGLE_BIT:			return  Message_SDO_AB_TOGGLE_BIT;			break;
case CO_SDO_AB_TIMEOUT:				return  Message_SDO_AB_TIMEOUT;				break;
case CO_SDO_AB_CMD:					return  Message_SDO_AB_CMD;					break;
case CO_SDO_AB_BLOCK_SIZE:			return  Message_SDO_AB_BLOCK_SIZE;			break;
case CO_SDO_AB_SEQ_NUM:				return  Message_SDO_AB_SEQ_NUM;				break;
case CO_SDO_AB_CRC:					return  Message_SDO_AB_CRC;					break;
case CO_SDO_AB_OUT_OF_MEM:			return  Message_SDO_AB_OUT_OF_MEM;			break;
case CO_SDO_AB_UNSUPPORTED_ACCESS:	return  Message_SDO_AB_UNSUPPORTED_ACCESS;	break;
case CO_SDO_AB_WRITEONLY:			return  Message_SDO_AB_WRITEONLY;			break;
case CO_SDO_AB_READONLY:			return  Message_SDO_AB_READONLY;			break;
case CO_SDO_AB_NOT_EXIST:			return  Message_SDO_AB_NOT_EXIST;			break;
case CO_SDO_AB_NO_MAP:				return  Message_SDO_AB_NO_MAP;				break;
case CO_SDO_AB_MAP_LEN:				return  Message_SDO_AB_MAP_LEN;				break;
case CO_SDO_AB_PRAM_INCOMPAT:		return  Message_SDO_AB_PRAM_INCOMPAT;		break;
case CO_SDO_AB_DEVICE_INCOMPAT:		return  Message_SDO_AB_DEVICE_INCOMPAT;		break;
case CO_SDO_AB_HW:					return  Message_SDO_AB_HW;					break;
case CO_SDO_AB_TYPE_MISMATCH:		return  Message_SDO_AB_TYPE_MISMATCH;		break;
case CO_SDO_AB_DATA_LONG:			return  Message_SDO_AB_DATA_LONG;			break;
case CO_SDO_AB_DATA_SHORT:			return  Message_SDO_AB_DATA_SHORT;			break;
case CO_SDO_AB_SUB_UNKNOWN:			return  Message_SDO_AB_SUB_UNKNOWN;			break;
case CO_SDO_AB_INVALID_VALUE:		return  Message_SDO_AB_INVALID_VALUE;		break;
case CO_SDO_AB_VALUE_HIGH:			return  Message_SDO_AB_VALUE_HIGH;			break;
case CO_SDO_AB_VALUE_LOW:			return  Message_SDO_AB_VALUE_LOW;			break;
case CO_SDO_AB_MAX_LESS_MIN:		return  Message_SDO_AB_MAX_LESS_MIN;		break;
case CO_SDO_AB_NO_RESOURCE:			return  Message_SDO_AB_NO_RESOURCE;			break;
case CO_SDO_AB_GENERAL:				return  Message_SDO_AB_GENERAL;				break;
case CO_SDO_AB_DATA_TRANSF:			return  Message_SDO_AB_DATA_TRANSF;			break;
case CO_SDO_AB_DATA_LOC_CTRL:		return  Message_SDO_AB_DATA_LOC_CTRL;		break;
case CO_SDO_AB_DATA_DEV_STATE:		return  Message_SDO_AB_DATA_DEV_STATE;		break;
case CO_SDO_AB_DATA_OD:				return  Message_SDO_AB_DATA_OD;				break; 
case CO_SDO_AB_NO_DATA:				return  Message_SDO_AB_NO_DATA;				break;
default:							return  Message_Default;					break;  			
}
	
}//end_of_SDO_abortCode_to_String(CO_SDO_abortCode_t Code)

