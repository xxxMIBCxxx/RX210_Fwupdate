/***********************************************************************/
/*                                                                     */
/*  FILE        :Main.c or Main.cpp                                    */
/*  DATE        :Tue, Oct 31, 2006                                     */
/*  DESCRIPTION :Main Program                                          */
/*  CPU TYPE    :                                                      */
/*                                                                     */
/*  NOTE:THIS IS A TYPICAL EXAMPLE.                                    */
/*                                                                     */
/***********************************************************************/
//#include "typedefine.h"
#include "iodefine.h"
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <machine.h>
#include "ff.h"	

#ifdef __cplusplus
//#include <ios>                        // Remove the comment when you use ios
//_SINT ios_base::Init::init_cnt;       // Remove the comment when you use ios
#endif

void main(void);
#ifdef __cplusplus
extern "C" {
void abort(void);
}
#endif


// �ԐFLED(P3-3�FCN1-5)
#define RED_LED_GPIO_PIN					( PORT3.PODR.BIT.B3 )
#define RED_LED( on )						( RED_LED_GPIO_PIN = on )
#define GET_RED_LED( )						( RED_LED_GPIO_PIN )

// �ΐFLED(P3-1�FCN1-7)
#define GERRN_LED_GPIO_PIN					( PORT3.PODR.BIT.B1 )
#define GREEN_LED( on )						( GERRN_LED_GPIO_PIN = on )
#define GET_GREEN_LED( )					( GERRN_LED_GPIO_PIN )


// RX210�v���O���������ݗ̈�
#define	PROGRAM_START_ADDR					( 0xFFFFFFFF )		// �v���O���������݊J�n�A�h���X
#define	PROGRAM_END_ADDR					( 0xFFF00000 )		// �v���O���������ݏI���A�h���X

#define MOTOROLA_S_TYPE_RECODE_HEADER_SIZE ( 4 )				// ���g���[��S ���R�[�h�w�b�_�[�T�C�Y


//-----------------------------------------------
// �������ʎ��
//-----------------------------------------------
typedef enum
{
	FW_UPDATE_SUCCESS 						= 0,
	FW_UPDATE_ERR_NOT_FW_FILE				= 0xE0001,			// �t�@�[���E�F�A�t�@�C�������݂��Ȃ�
	FW_UPDATE_ERR_FW_FILE					= 0xE0002,			// �t�@�[���E�F�A�t�@�C���ُ�
	FW_UPDATE_ERR_FILE_READ_BUFFER_SIZE		= 0xE0003,			// �t�@�[���E�F�A�i�[�p�o�b�t�@�T�C�Y�ُ�
	
	FW_UPDATE_ERR_ANALYZE_RECODE_SIZE		= 0xE0002,			// �t�@�[���E�F�A��̓G���[�i���R�[�h�T�C�Y�ُ�j
	FW_UPDATE_ERR_ANALYZE_NOT_RECODE		= 0xE0003,			// �t�@�[���E�F�A��̓G���[�i���R�[�h�f�[�^�ُ�j
	FW_UPDATE_ERR_ANALYZE_RECODE_TYPE		= 0xE0004,			// �t�@�[���E�F�A��̓G���[�i���R�[�h�^�C�v�ُ�j
	FW_UPDATE_ERR_ANALYZE_PROGRAM_RANGE		= 0xE0005,			// �t�@�[���E�F�A��̓G���[�i�v���O���������ݗ̈�ُ͈͈�j
	FW_UPDATE_ERR_ANALYZE_INPUT_BUFFER_SIZE	= 0xE0006,			// �t�@�[���E�F�A��̓G���[�i���̓f�[�^�T�C�Y�ُ�j
	FW_UPDATE_ERR_ANALYZE_CHECK_SUM			= 0xE0007,			// �t�@�[���E�F�A��̓G���[�i�`�F�b�N�E�T���G���[�j
	FW_UPDATE_ERR_ANALYZE_RECODE_EOF		= 0xE0008,			// �t�@�[���E�F�A��̓G���[�i���R�[�h�I�[�ُ�j

	FW_UPDATE_ERR_SYSTEM					= 0xE9999,			// �V�X�e���ُ�
	
	
	
} FW_UPDATE_RESULT;



//-----------------------------------------------
// LED�_�����[�h���
//-----------------------------------------------
typedef enum
{
	LED_MODE_START,												// FW�J�n�i�ΐFLED�F�_�� / �ԐFLED�F�_���j
	LED_MODE_PROCESS,											// �������i�ΐFLED�F�_��(500ms) / �ԐFLED�F�����j
	LED_MODE_PROCESS_PROGRAM,									// ������_�����ݒ��i�ΐFLED�F�_��(200ms) / �ԐFLED�F�����j
	LED_MODE_ERROR,												// �G���[�i�ΐFLED�F���� / �ԐFLED�F�_���j
	LED_MODE_SUCCESS,											// ����I���i�ΐFLED�F�_�� / �ԐFLED�F�_�Łj	
} LED_MODE_ENUM;



//-----------------------------------------------
// ���g���[��S�t�@�C���E���R�[�h�^�C�v���
//-----------------------------------------------
typedef enum
{
	S0_RECODE_TYPE,
	S1_RECODE_TYPE,
	S2_RECODE_TYPE,
	S3_RECODE_TYPE,
	S7_RECODE_TYPE,
	S8_RECODE_TYPE,
	S9_RECODE_TYPE,
} S_RECODE_TYPE_ENUM;


//-----------------------------------------------
// ���g���[��S�t�@�C�����\����
//-----------------------------------------------
typedef struct
{
	S_RECODE_TYPE_ENUM			eRecodeType;			// ���R�[�h�^�C�v
	
	DWORD						Address;				// ���[�h�E�A�h���X(S1�F16Bit / S2�F24Bit / S3�F32Bit)
	BYTE						CodeSize;				// �R�[�h�T�C�Y
	BYTE						Code[ 256 ];			// �R�[�h(S1 / S2 / S3) or �t�@�C����(S0) 
	BYTE						CheckSum;				// �`�F�b�N�E�T��
} MOTOROLA_S_TYPE_DATA_INFO_TABLE;






//=============================================================================
// �v���g�^�C�v�錾
//=============================================================================
void InitClock( void );																// �N���b�N�ݒ�
void InitCMT0( void );																// CMT0�������i1ms�^�C�}�[�𐶐��j
void SoftwareDelay( DWORD delay_ms );												// �\�t�g�E�F�A�x��
void Delay_IntervalTimer( void );													// Delay 1ms�^�C�}�[����
void LED_IntervalTimer( void );														// LED 1ms�^�C�}�[
FW_UPDATE_RESULT Get_Motorola_S_Data( FIL *pFile, BYTE *pData, WORD DataSize );		// ���g���[���ES�^�C�v�t�@�C����1�����擾����
FW_UPDATE_RESULT Analyze_Motorola_S_Data( BYTE *pData, WORD DataSize, MOTOROLA_S_TYPE_DATA_INFO_TABLE *ptStypeInfo );				// ���g���[���ES�^�C�v�t�@�C������͂���





//=============================================================================
// �O���[�o���ϐ�
//=============================================================================
typedef struct 
{
	FW_UPDATE_RESULT									eResult;					// ��������
	LED_MODE_ENUM										eLedMode;					// LED���[�h
	
	DWORD												DelayTimer;					// �x���^�C�}�[(ms)
	BYTE												Buffer[ 512 ];				// �t�@�[���E�F�A�f�[�^�ǂݏo���p�o�b�t�@
	
} GROBAL_VARIABLE_TABLE;

GROBAL_VARIABLE_TABLE						g_Grobal;	

	

void main(void)
{
	FW_UPDATE_RESULT					eRet = FW_UPDATE_ERR_SYSTEM;
	bool								bRet = false;
	FATFS								Fs;
	FRESULT								FsResult = FR_OK;
	FIL									File;
	MOTOROLA_S_TYPE_DATA_INFO_TABLE 	tStypeInfo;
	
	
	// �N���b�N�ݒ�
	InitClock( );
	
	// �ԐFLED(P3-3�FCN1-5) , �ΐFLED(P3-1�FCN1-7)
	PORT3.PDR.BIT.B3 = 1;
	PORT3.PDR.BIT.B1=  1;
	RED_LED( 1 );
	GREEN_LED( 1 );

	// CMT0�������i1ms�^�C�}�[�𐶐��j
	InitCMT0( );
	
	// 1�b�҂�
	SoftwareDelay( 1000 );

	// SD�J�[�h���}�E���g����
	FsResult = f_mount( &Fs, "", 1 );
	if (FsResult == FR_OK)
	{
		// FW�t�@�C���I�[�v��
		FsResult = f_open( &File, "Fw.mot", FA_READ );
		if (FsResult == FR_OK)
		{
			while( f_eof( &File) != 1 )
			{
				// �P�����擾
				memset( g_Grobal.Buffer, 0x00, sizeof(g_Grobal.Buffer) );
				eRet = Get_Motorola_S_Data( &File, g_Grobal.Buffer, sizeof(g_Grobal.Buffer) );
				if( eRet != FW_UPDATE_SUCCESS )
				{
					g_Grobal.eLedMode = LED_MODE_ERROR;			// LED���[�h�F�G���[
					break;
				}
				
				// ���
				memset( &tStypeInfo, 0x00, sizeof(tStypeInfo) );
				eRet = Analyze_Motorola_S_Data( g_Grobal.Buffer, sizeof(g_Grobal.Buffer), &tStypeInfo );
				if( eRet != FW_UPDATE_SUCCESS )
				{
					g_Grobal.eLedMode = LED_MODE_ERROR;			// LED���[�h�F�G���[
					break;
				}
			}
			
			// ����
			eRet = FW_UPDATE_SUCCESS;
			g_Grobal.eLedMode = LED_MODE_SUCCESS;			// LED���[�h�F����I��

		}
		else
		{
			// �t�@�[���E�F�A�t�@�C�������݂��Ȃ�
			eRet = FW_UPDATE_ERR_NOT_FW_FILE;
			g_Grobal.eLedMode = LED_MODE_ERROR;				// LED���[�h�F�G���[
		}
	}
	else
	{
		// �V�X�e���ُ�
		eRet = FW_UPDATE_ERR_SYSTEM;
		g_Grobal.eLedMode = LED_MODE_ERROR;				// LED���[�h�F�G���[
	}	
	
	
	while( 1 )
	{
		nop( );
	}
	
	
	

}

#ifdef __cplusplus
void abort(void)
{

}
#endif




//=============================================================================
// �N���b�N�ݒ�
//=============================================================================
void InitClock( void )
{
    // �v���e�N�g�����i�N���b�N������H�֘A / ���샂�[�h�A����d�͒ጸ�@�\�A�\�t�g�E�F�A���Z�b�g�֘A / VRCR�j
    SYSTEM.PRCR.WORD = 0xA507;
     
//  // �d�����M�����[�^���䃌�W�X�^(�V�X�e���N���b�N��ύX���邽�߁A"00h"���������ށj
//  SYSTEM.VRCR = 0x00;
     
//  RX210�ɂ̓��C���N���b�N���Ȃ��̂�
//  // ���C���N���b�N���U����J�n
//  SYSTEM.MOSCWTCR.BYTE = 0x0D;        		// ���C���N���b�N���U���莞�ԁF131072�T�C�N��
//  SYSTEM.MOSCCR.BIT.MOSTP = 0;        		// ���C���N���b�N���U����J�n
//  while( SYSTEM.MOSCCR.BIT.MOSTP != 0 );  	// �ݒ芮���҂�
     
    // PLL���U����J�n
    SYSTEM.PLLCR.WORD = 0x0901;     			// 20HMz / 2 * 10 = 100MHz
    SYSTEM.PLLWTCR.BYTE = 0x0D;     			// PLL���U���莞�ԁF1048576�T�C�N��
    SYSTEM.PLLCR2.BIT.PLLEN = 0;        		// PLL���U����J�n
    while( SYSTEM.PLLCR2.BIT.PLLEN != 0 );  	// �ݒ芮���҂�
     
// // �������샂�[�h�ɑJ��
//  SYSTEM.OPCCR.BIT.OPCM = 0;      			// �������샂�[�h
//  while( SYSTEM.OPCCR.BIT.OPCMTSF != 0 ); 	// �J�ڊ����܂ő҂�
     
    // ���Ӄ��W���[���N���b�N(PCLKD)�F50MHz(100MHz / 2)
    // ���Ӄ��W���[���N���b�N(PCLKB)�F25MHz(100MHz / 4)
    // �O���o�X�N���b�N(BCLK)       �F25MHz(100Mhz / 4)
    // �V�X�e���N���b�N(ICLK)       �F50MHz(100MHz / 2)
    // FlashIF�N���b�N(FCLK)        �F25MHz(100MHz / 4)
    SYSTEM.SCKCR.LONG = 0x21021212;
    while( SYSTEM.SCKCR.LONG != 0x21021212 ); 	// �ݒ芮���҂�
         
    // PLL��H�I��
    SYSTEM.SCKCR3.WORD = 0x0400;        		// PLL��H�I��
    while( SYSTEM.SCKCR3.WORD != 0x0400 );  	// �ݒ芮���҂�
     
    // �����I���`�b�v�I�V���[�^��~
    SYSTEM.HOCOCR.BYTE = 0x01;      			// HOCO��~
    while( SYSTEM.HOCOCR.BYTE != 0x01 );    	// �ݒ芮���҂�
     
    // �ᑬ�I���`�b�v�I�V���[�^��~
    SYSTEM.LOCOCR.BYTE = 0x01;      			// LOCO��~
    while( SYSTEM.LOCOCR.BYTE != 0x01 );    	// �ݒ芮���҂�
     
    // �T�u�N���b�N���U���~
    SYSTEM.SOSCCR.BYTE = 0x01;      			// �T�u�N���b�N���U���~
    while( SYSTEM.SOSCCR.BYTE != 0x01 );    	// �ݒ芮���҂�
    RTC.RCR3.BYTE = 0x0C;           			// RTC[�T�u�N���b�N���U���~, �W��CL�p�h���C�u�\��]
    while( RTC.RCR3.BYTE != 0x0C );     		// �ݒ芮���҂�

    // �v���e�N�g�ݒ�
    SYSTEM.PRCR.WORD = 0xA500;  
}


//=============================================================================
// CMT0�������i1ms�^�C�}�[�𐶐��j
//=============================================================================
void InitCMT0( void )
{
    // �v���e�N�g�����i���샂�[�h�A����d�͒ጸ�@�\�A�\�t�g�E�F�A���Z�b�g�֘A�j
    SYSTEM.PRCR.WORD = 0xA502;
     
    // �R���y�A�}�b�`�^�C�}(���j�b�g0)���W���[���X�g�b�v�ݒ�r�b�g
    MSTP(CMT0) = 0;             	// ��ԉ���
     
    // CMI0�����ݗv������
    IEN( CMT0, CMI0 ) = 1;
     
    // CMI0�����݂̊����ݗD�惌�x����1�ɐݒ�
    IPR( CMT0, CMI0 ) = 1;
     
    // �v���e�N�g�ݒ�
    SYSTEM.PRCR.WORD = 0xA500;  
 
     
    CMT0.CMCR.WORD = 0x00C0;   	 	// �R���y�A�}�b�`�����݋���, �N���b�N�I���r�b�g(PCLKB(25MHz) / 8)
    CMT0.CMCOR = 3125 - 1;      	// 1ms�̃J�E���g
    CMT0.CMCNT = 0;         		// �R���y�A�}�b�`�^�C�}�J�E���^������
    CMT.CMSTR0.BIT.STR0 = 1;    	// CMT0.CMCNT�J�E���^�̃J�E���g����J�n
}   


//=============================================================================
// �\�t�g�E�F�A�x��
//=============================================================================
void SoftwareDelay( DWORD delay_ms )
{
	g_Grobal.DelayTimer = delay_ms;
	
	while ( g_Grobal.DelayTimer );
}


//=============================================================================
// Delay 1ms�^�C�}�[����
//=============================================================================
void Delay_IntervalTimer( void )
{
	DWORD		time;
	
	// DelayTimer
	time = g_Grobal.DelayTimer;
	if (time) g_Grobal.DelayTimer = --time;
}
	

//=============================================================================
// LED 1ms�^�C�}�[����
//=============================================================================
void LED_IntervalTimer( void )
{
	static volatile LED_MODE_ENUM			ePrevLedMode = LED_MODE_START;						// �O���LED���[�h
	static volatile DWORD					LedTimer = 0;										// �_�Ń^�C�}�[(ms)
	static volatile DWORD					BackupLedTimer = 0;									// �_�Ń^�C�}�[(ms)�o�b�N�A�b�v�p
	
	
	// LED���[�h���O���LED���[�h�ƈႤ�ꍇ
	if (g_Grobal.eLedMode != ePrevLedMode)
	{
		// FW�J�n�i�ΐFLED�F�_�� / �ԐFLED�F�_���j
		if (g_Grobal.eLedMode == LED_MODE_START)
		{
			GREEN_LED( 1 );
			RED_LED( 1 );
		}
		// �������i�ΐFLED�F�_��(500ms) / �ԐFLED�F�����j
		else if (g_Grobal.eLedMode == LED_MODE_PROCESS)
		{
			LedTimer = BackupLedTimer = 500;
			GREEN_LED( 1 );
			RED_LED( 0 );
		}
		// ������_�����ݒ��i�ΐFLED�F�_��(200ms) / �ԐFLED�F�����j
		else if (g_Grobal.eLedMode == LED_MODE_PROCESS_PROGRAM)
		{
			LedTimer = BackupLedTimer = 200;
			GREEN_LED( 1 );
			RED_LED( 0 );
		}
		// �G���[�i�ΐFLED�F���� / �ԐFLED�F�_���j
		else if (g_Grobal.eLedMode == LED_MODE_ERROR)
		{
			GREEN_LED( 0 );
			RED_LED( 1 );
		}
		// ����I���i�ΐFLED�F�_�� / �ԐFLED�F�_�Łj
		else if (g_Grobal.eLedMode == LED_MODE_SUCCESS)		
		{
			GREEN_LED( 1 );
			RED_LED( 0 );
		}
		
		// �O���LED���[�h��ύX
		ePrevLedMode = g_Grobal.eLedMode;		
	}
	else
	{
		// ������ or ������_�����ݒ��i�����Ƃ��ΐFLED�_�łȂ̂Łc�j
		if ( (g_Grobal.eLedMode == LED_MODE_PROCESS) || (g_Grobal.eLedMode == LED_MODE_PROCESS_PROGRAM) )
		{
			LedTimer--;
			if (LedTimer == 0)
			{
				LedTimer = BackupLedTimer;
				(GET_GREEN_LED() == 0) ? GREEN_LED( 1 ) : GREEN_LED( 0 );
			}
		}
	}
}


//=============================================================================
// ���g���[���ES�^�C�v�t�@�C����1�����擾����
//=============================================================================
FW_UPDATE_RESULT Get_Motorola_S_Data( FIL *pFile, BYTE *pData, WORD DataSize )
{
	FRESULT			eFsResult = FR_OK;
	BYTE			Buff[ 2 ] = { 0x00, 0x00 };
	UINT			ByteRead = 0;
	WORD			i = 0;
	bool			bCr = false;
	
	
	// �f�[�^���Ȃ��Ȃ�܂Ń��[�v
	while ( !f_eof( pFile ) )
	{
		// 1�����ǂݍ���
		eFsResult = f_read( pFile, Buff, 1, &ByteRead );
		if (eFsResult != FR_OK)
		{
			// �t�@�[���E�F�A�t�@�C���ُ�
			return FW_UPDATE_ERR_FW_FILE;
		}
		
		// �擾�f�[�^�i�[
		pData[ i ] = Buff[ 0 ];
		
		// ��؂�`�F�b�N
		if (Buff[ 0 ] == 0x0D)
		{
			bCr = true;
		}
		else if ( (bCr == true) && (Buff[ 0 ] == 0x0A) )
		{
			// �P�����̃f�[�^��������
			break;
		}
		else
		{
			bCr = false;
		}
		
		// �i�[�T�C�Y�`�F�b�N
		i++;
		if ( DataSize <= i )
		{
			// 1�����̃f�[�^���i�[�ł��Ȃ����߁A�G���[�Ƃ���
			// �t�@�[���E�F�A�i�[�p�o�b�t�@�T�C�Y�ُ�
			return FW_UPDATE_ERR_FILE_READ_BUFFER_SIZE;
		}
	}
		
	return FW_UPDATE_SUCCESS;
}


//=============================================================================
// ���g���[���ES�^�C�v�t�@�C������͂���
//=============================================================================
FW_UPDATE_RESULT Analyze_Motorola_S_Data( BYTE *pData, WORD DataSize, MOTOROLA_S_TYPE_DATA_INFO_TABLE *ptStypeInfo )
{
	BYTE				strHex[ 3 ];
	BYTE				Length = 0x00;
	BYTE				Sum = 0x00;
	BYTE				Bin = 0x00;
	WORD				i = 0;
	WORD				DataNum = 0;
	WORD				AddrEndPos = 0;
	
	
	// �T�C�Y�`�F�b�N�i�Ƃ肠�����f�[�^���܂ł���H�j
	if ( DataSize < MOTOROLA_S_TYPE_RECODE_HEADER_SIZE )
	{
		// �t�@�[���E�F�A��̓G���[�i���R�[�h�T�C�Y�ُ�j
		return FW_UPDATE_ERR_ANALYZE_RECODE_SIZE;
	}
	
	// ���R�[�h�̍ŏ���'S'
	if ( (pData[ 0 ] != 'S') && (pData[ 0 ] != 's') )
	{
		// �t�@�[���E�F�A��̓G���[�i���R�[�h�f�[�^�ُ�j
		return FW_UPDATE_ERR_ANALYZE_NOT_RECODE;
	}
	
	// �^�C�v���擾
	switch ( pData[ 1 ] ) {
	case '0':
		ptStypeInfo->eRecodeType = S0_RECODE_TYPE;
		AddrEndPos = 4;			// "0000"�Œ肾���ǁc
		break;
	case '1':
		ptStypeInfo->eRecodeType = S1_RECODE_TYPE;
		AddrEndPos = 4;			// ���[�h�E�A�h���X(16bit)
		break;
	case '2':
		ptStypeInfo->eRecodeType = S2_RECODE_TYPE;
		AddrEndPos = 6;			// ���[�h�E�A�h���X(24bit)
		break;
	case '3':
		ptStypeInfo->eRecodeType = S3_RECODE_TYPE;
		AddrEndPos = 8;			// ���[�h�E�A�h���X(32bit)
		break;
	case '7':
		ptStypeInfo->eRecodeType = S7_RECODE_TYPE;
		AddrEndPos = 8;			// �G���g���E�|�C���g�E�A�h���X(32bit)
		break;
	case '8':
		ptStypeInfo->eRecodeType = S8_RECODE_TYPE;
		AddrEndPos = 6;			// �G���g���E�|�C���g�E�A�h���X(24bit)
		break;
	case '9':
		ptStypeInfo->eRecodeType = S9_RECODE_TYPE;
		AddrEndPos = 4;			// �G���g���E�|�C���g�E�A�h���X(16bit)
		break;
	default:
		// �t�@�[���E�F�A��̓G���[�i���R�[�h�^�C�v�ُ�j
		return FW_UPDATE_ERR_ANALYZE_RECODE_TYPE;
	}

	// ���R�[�h�����擾�i�`�F�b�N�E�T���Ώہj
	memset( strHex, 0x00, sizeof(strHex) );
	strHex[ 0 ] = pData[ 2 ];
	strHex[ 1 ] = pData[ 3 ];
	Length = (BYTE)strtoul( (const char *)strHex, NULL, 16 );
	Sum += Length;

	// ���R�[�h���������������߁A�f�[�^�T�C�Y�����R�[�h�����i�[�ł��邩���`�F�b�N����
	if ( ((Length * 2) + MOTOROLA_S_TYPE_RECODE_HEADER_SIZE) > DataSize )
	{
		// �t�@�[���E�F�A��̓G���[�i���̓f�[�^�T�C�Y�ُ�j
		return FW_UPDATE_ERR_ANALYZE_INPUT_BUFFER_SIZE;
	}
	
	// �A�h���X���擾�i�`�F�b�N�E�T���Ώہj
	memset( strHex, 0x00, sizeof(strHex) );
	ptStypeInfo->Address = 0;
	DataNum = 0;
	for ( i = MOTOROLA_S_TYPE_RECODE_HEADER_SIZE ; i < (AddrEndPos + MOTOROLA_S_TYPE_RECODE_HEADER_SIZE) ; i++ )
	{
		strHex[ DataNum ] = pData[ i ];
		DataNum++;
		
		// HEX�ϊ��ɕK�v�ȂQ���������擾������
		if ( (DataNum % 2) == 0 )
		{
			Bin = (BYTE)strtoul( (const char *)strHex, NULL, 16 );
			Sum += Bin;
			
			ptStypeInfo->Address <<= 8;
			ptStypeInfo->Address |=  (DWORD)Bin;
			
			memset( strHex, 0x00, sizeof(strHex) );
			DataNum = 0;
		}
	}
	
	// �v���O���������ݗ̈�͈̓`�F�b�N�iS0���R�[�h�^�C�v�ȊO)
	if ( ptStypeInfo->eRecodeType != S0_RECODE_TYPE)
	{
		if ( (ptStypeInfo->Address > PROGRAM_START_ADDR)  || (ptStypeInfo->Address < PROGRAM_END_ADDR) )
		{
			// �t�@�[���E�F�A��̓G���[�i�v���O���������ݗ̈�ُ͈͈�j
			return FW_UPDATE_ERR_ANALYZE_PROGRAM_RANGE;
		}
	}
	
	// �f�[�^��́i�`�F�b�N�E�T���Ώہj
	// �E�ϐ�i�̏����l�ɂ��āF�A�h���X�擾�Ŏg�p�����Ƃ��̏�Ԃ������l�Ƃ���
	// �ELength - 1�F�`�F�b�N�E�T�����Ȃ�����
	DataNum = 0;
	for( ; i < (MOTOROLA_S_TYPE_RECODE_HEADER_SIZE + ((Length - 1) * 2)) ; i+=2 )
	{
		strHex[ 0 ] = pData[ i ];
		strHex[ 1 ] = pData[ (i + 1) ];
		Bin = (BYTE)strtoul( (const char *)strHex, NULL, 16 );
		ptStypeInfo->Code[ DataNum ] = Bin;
		Sum += Bin;
		DataNum++;
	}
	ptStypeInfo->CodeSize = DataNum;
	
	// �`�F�b�N�E�T����
	strHex[ 0 ] = pData[ i ];
	strHex[ 1 ] = pData[ (i + 1) ];
	Bin = (BYTE)strtoul( (const char *)strHex, NULL, 16 );
	ptStypeInfo->CheckSum = Bin;
	Sum += Bin;

	// �`�F�b�N�E�T��
	if ( Sum != 0xFF )
	{
		// �t�@�[���E�F�A��̓G���[�i�`�F�b�N�E�T���G���[�j
		return FW_UPDATE_ERR_ANALYZE_CHECK_SUM;
	}
	
	// ���R�[�h�I�[�`�F�b�N
	if ( (pData[ i + 2 ] != 0x0D) && (pData[ i + 3 ] != 0x0A) )
	{
		// �t�@�[���E�F�A��̓G���[�i���R�[�h�I�[�ُ�j
		return FW_UPDATE_ERR_ANALYZE_RECODE_EOF;
	}

	return FW_UPDATE_SUCCESS;
}
