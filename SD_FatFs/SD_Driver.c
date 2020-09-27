//*************************************************************************************************
// SD�h���C�o
// ��GPIO��PB-0��CS�Ƃ��Ďg�p���܂�
//*************************************************************************************************
#include "iodefine.h"
#include <stdbool.h>
#include <stdlib.h>
#include <machine.h>
#include "diskio.h"													// FATFS - Common include file for ff.c and disk layer







#ifdef __LIT
#define LDDW(x) revl(x)												// Little endian: swap bytes
#else
#define LDDW(x) x													// Big endian: no swap
#endif


#define SdCS_GPIO_PIN						( PORTB.PODR.BIT.B7 )	// CS�[�q
#define CS_HIGH( )							( SdCS_GPIO_PIN = 1 )	
#define CS_LOW( )							( SdCS_GPIO_PIN = 0 )

#define	SPRI								IR( RSPI0, SPRI0 )

#define F_PCLK								( 25000000UL )			// PCLK frequency (configured by SCKCR.PCKB)
#define SCLK_FAST							( 16000000UL )			// SCLK frequency (R/W)
#define	SCLK_SLOW							(   400000UL )			// SCLK frequency (Init)



// SD�R�}���h
#define CMD0	(0)					// �\�t�g�E�F�A���Z�b�g 		- [GO_IDLE_STATE]
#define CMD1	(1)					// �������J�n 					- [SEND_OP_COND (MMC)]
#define	ACMD41	(0x80+41)			// �������J�n(SDC��p) 			- [SEND_OP_COND (SDC)]
#define CMD8	(8)					// SDC V2��p�B����d���m�F 	- [SEND_IF_COND]
#define CMD9	(9)					// CSD�ǂݏo�� 					- [SEND_CSD]
#define CMD10	(10)				// CID�ǂݏo��					- [SEND_CID]
#define CMD12	(12)				// ���[�h�����~				- [STOP_TRANSMISSION]
#define ACMD13	(0x80+13)			// ��Ԏ擾(SDC��p)			- [SD_STATUS (SDC)]
#define CMD16	(16)				// R/W�u���b�N���ύX			- [SET_BLOCKLEN]
#define CMD17	(17)				// �V���O���u���b�N�ǂݏo��		- [READ_SINGLE_BLOCK]
#define CMD18	(18)				// �}���`�u���b�N�ǂݏo��		- [READ_MULTIPLE_BLOCK]
#define CMD23	(23)				// ���̃}���`�u���b�N�ǂݏo��/�������݃R�}���h�ł̓]���u���b�N����ݒ�(MMC��p)	- [SET_BLOCK_COUNT (MMC)]
#define	ACMD23	(0x80+23)			// ���̃}���`�u���b�N�������݃R�}���h�ł̃v�������u���b�N����ݒ�(SDC��p)		- [SET_WR_BLK_ERASE_COUNT (SDC)]
#define CMD24	(24)				// �V���O���u���b�N��������		- [WRITE_BLOCK]
#define CMD25	(25)				// �}���`�u���b�N��������		- [WRITE_MULTIPLE_BLOCK]
#define CMD32	(32)				// ERASE_ER_BLK_START
#define CMD33	(33)				// ERASE_ER_BLK_END
#define CMD38	(38)				// ERASE
#define CMD55	(55)				// �A�v���P�[�V���������R�}���h	- [APP_CMD]
#define CMD58	(58)				// OCR�ǂݏo��					- [READ_OCR]



// SD���e�[�u���\����
typedef struct _SD_INFO_TABLE
{
	BYTE										CardType;								// �J�[�h�^�C�v
	DSTATUS 									Stat;									// Physical drive status

	DWORD										Timer1;
	DWORD										Timer2;
	
} SD_INFO_TABLE;





// �O���[�o���ϐ�
static SD_INFO_TABLE							g_tSdInfo =	{ 0, STA_NOINIT, 0, 0 };	// SD���e�[�u��

	


// �v���g�^�C�v�錾
static BYTE send_cmd( BYTE cmd, DWORD arg );											// SD�փR�}���h�𑗐M
static UINT rcvr_datablock( BYTE *pBuff, UINT ReadSize );								// Send a data packet to the MMC

#if DISK_USE_WRITE
static UINT xmit_datablock( const BYTE *pBuff, UINT Token );							// Send a command packet to the MMC
#endif	// #if DISK_USE_WRITE

static UINT wait_ready( DWORD Timeout );												// SD�J�[�h���������`�F�b�N
static void sd_deselect( void );														// CS:SD����
static UINT sd_select( void );															// CS:SD�I��
static BYTE xchg_spi( BYTE SendData );													// 1�o�C�g����M

#if DISK_USE_WRITE
static bool xmit_spi_multi( const BYTE *pSendBuff, UINT SendSize );						// �}���`�o�C�g���M
#endif	// #if DISK_USE_WRITE

static bool rcvr_spi_multi( const BYTE *pRecvBuff, UINT RecvSize );						// �}���`�o�C�g��M
static void sd_spimode( void );															// SD��SPI���[�h�ɂ���
static void rspi_enable( void );														// RSPI���g�p�ł���悤�ɂ���
static BYTE calc_spbr( DWORD bps_target );												// RSPI �r�b�g���[�g���W�X�^�ɐݒ肷��l���v�Z����
static void rspi_interrupts_enable( bool enabled );										// RSPI�����ݗL���^����
static void rspi_spsr_clear( void );													// SPSR���W�X�^�N���A
static void rspi_ir_clear( void );														// RSPI IR ���W�X�^�N���A


//=============================================================================
// RSPI 1ms�^�C�}�[����
//=============================================================================
void RSPI_IntervalTimer( void )
{
	DWORD		time;
//	BYTE		status;

	
	// Time1
	time = g_tSdInfo.Timer1;
	if (time) g_tSdInfo.Timer1 == --time;
	
	// Time2
	time = g_tSdInfo.Timer2;
	if (time) g_tSdInfo.Timer2 == --time;
	
#if 0
//	status = g_Stat;
//	if (WPRT)
//	{
//		status |= STA_PROTECT;		// Write protected
//	}
//	else
//	{
//		status &= ~STA_PROTECT;		// Write enabled
//	}
//	if (INS)
//	{
//		status &= ~STA_NODISK;		// Card is in socket
//	}
//	else
//	{
//		status |= (STA_NODISK | STA_NOINIT);	// Socket empty
//	}
//	g_Stat = status;
#endif		
}


//=============================================================================
// SD������
//=============================================================================
DSTATUS disk_initialize( BYTE Drv )
{
	BYTE					Response = 0x00;
	BYTE					ocr[ 4 ] = { 0x00, 0x00, 0x00, 0x00 };
	BYTE					i = 0;
	BYTE					card_type = 0;
	BYTE					cmd = 0;
	
	
	// �h���C�u0�ȊO�̓G���[
	if (Drv != 0)
	{
		g_tSdInfo.Stat |= STA_NOINIT;
		return STA_NOINIT;
	}
	
	// RSPI���g�p�ł���悤�ɂ���
	rspi_enable( );

	// SD��SPI���[�h�ɂ���
	sd_spimode( );

	// �\�t�g�E�F�A���Z�b�g
	card_type = 0;
	Response = send_cmd( CMD0, 0 );
	if (Response == 1)
	{
		g_tSdInfo.Timer1 = 1000;
		
		// SDC V2��p - ����d���m�F
		Response = send_cmd( CMD8, 0x1AA );
		if (Response == 1)
		{
			// R7�̉������擾
			for ( i = 0 ; i < 4 ; i++ )
			{
				ocr[ i ] = xchg_spi( 0xFF );
			}

			if ( (ocr[ 2 ] == 0x01) && (ocr[ 3 ] == 0xAA ) )
			{
				// SD������(SDv2)
				while( g_tSdInfo.Timer1 && send_cmd( ACMD41, 1UL << 30 ) );				// Wait for end of initialization with ACMD41(HCS)
				if (g_tSdInfo.Timer1 && send_cmd( CMD58, 0 ) == 0)						// Check CCS bit in the OCR
				{
					// R7�̉������擾
					for ( i = 0 ; i < 4 ; i++ )
					{
						ocr[ i ] = xchg_spi( 0xFF );
					}
					card_type = (ocr[ 0 ] & 0x40) ? (CT_SD2 | CT_BLOCK) : CT_SD2;		// Check if the card is SDv2
				}
			}
			else
			{
				// SDv2�łȂ��ꍇ
				Response = send_cmd( ACMD41, 0 );
				if( Response <= 1 )
				{
					card_type = CT_SD1;
					cmd = ACMD41;
				}
				else
				{
					card_type = CT_MMC;
					cmd = CMD1;
				}
		
				// SD������(MMC or SDv1)
				while ( g_tSdInfo.Timer1 && send_cmd( cmd, 0 ) != 0 );					// Wait for end of initialization 					

				// �u���b�N����512�ɕύX
				if( !g_tSdInfo.Timer1 || (send_cmd( CMD16, 512 ) != 0) )
				{
					// �ύX�Ɏ��s�����ꍇ�̓G���[�Ƃ��邽�߁A�J�[�h�^�C�v��0�ɂ���
					card_type = 0;
				}				
			}
		}
	}
	
	// �J�[�h�^�C�v���Z�b�g
	g_tSdInfo.CardType = card_type;
	
	sd_deselect( );		
	
	if (g_tSdInfo.CardType == 0)
	{
		g_tSdInfo.Stat |= STA_NOINIT;
		return STA_NOINIT;
	}
		
	// RSPI�̒ʐM���x���グ��(16MHz)
	RSPI0.SPCR.BIT.SPE = 0;
	RSPI0.SPBR = calc_spbr( SCLK_FAST );
	RSPI0.SPCR.BIT.SPE = 1;

	
	g_tSdInfo.Stat &= ~STA_NOINIT;
	
	return g_tSdInfo.Stat;
}


//=============================================================================
// SD�̏�Ԏ擾
//=============================================================================
DSTATUS disk_status( BYTE Drv )
{
	// �h���C�u0�ȊO�̓G���[
	if (Drv != 0)
	{
		return STA_NOINIT;
	}
	
	return g_tSdInfo.Stat;
}


//=============================================================================
// SD�Ǎ���
//   Drv      : Physical drive number (0)
//   pBuff 	  : Pointer to the data buffer to store read data
//   Sector   : Start sector number (LBA)
//   Count    : Number of sectors to read (1..128)
//=============================================================================
DRESULT disk_read( BYTE Drv, BYTE *pBuff, DWORD Sector, UINT Count )
{
	BYTE					Response = 0x00;
	
	
	// �p�����[�^�`�F�b�N
	if (Drv || !Count)
	{
		return RES_PARERR;
	}
	
	// SD�̏�ԁH
	if (g_tSdInfo.Stat & STA_NOINIT)
	{
		return RES_NOTRDY;
	}
	
	// �u���b�N�A�h���X�ɑΉ����Ă��Ȃ�SD�J�[�h�̏ꍇ
	if ( !(g_tSdInfo.CardType & CT_BLOCK) )
	{
		// LBA ot BA conversion (byte addressing cards)
		Sector *= 512;
	}
	
	// Single sector read
	if (Count == 1)
	{
		// �V���O���u���b�N�ǂݏo���v��
		Response = send_cmd( CMD17, Sector );
		if (Response == 0)
		{
			// �V���O���u���b�N���[�h
			if (rcvr_datablock( pBuff, 512 ) == 1)
			{
				Count = 0;
			}
		}
	}
	// Multiple sector read
	else
	{
		// �}���`�u���b�N�ǂݏo��
		Response = send_cmd( CMD18, Sector );
		if (Response == 0)
		{
			// �Z�N�^�[���A�f�[�^��ǂݍ���
			do
			{
				// Timeout?
				if (rcvr_datablock( pBuff, 512 ) == 0)
				{
					break;
				}
				pBuff += 512;							
			} while ( --Count );
			
			// ���[�h�����~
			send_cmd( CMD12, 0 );
		}
	}
		
	sd_deselect( );
			
	return ( (Count == 0) ? RES_OK : RES_ERROR );
}


//=============================================================================
// SD������
//   Drv      : Physical drive number (0)
//   pBuff 	  : Pointer to the data buffer to store read data
//   Sector   : Start sector number (LBA)
//   Count    : Number of sectors to read (1..128)
//=============================================================================
#if DISK_USE_WRITE
DRESULT disk_write( BYTE Drv, const BYTE *pBuff, DWORD Sector, UINT Count )
{
	BYTE					Response = 0x00;
	
	
	// �p�����[�^�`�F�b�N
	if (Drv || !Count)
	{
		return RES_PARERR;
	}
	
	// SD�̏�ԁH
	if (g_tSdInfo.Stat & STA_NOINIT)
	{
		return RES_NOTRDY;
	}
	
	if (g_tSdInfo.Stat & STA_PROTECT)
	{
		return RES_WRPRT;
	}
	
	// �u���b�N�A�h���X�ɑΉ����Ă��Ȃ�SD�J�[�h�̏ꍇ
	if ( !(g_tSdInfo.CardType & CT_BLOCK) )
	{
		// LBA ot BA conversion (byte addressing cards)
		Sector *= 512;
	}
	
	//  Single sector write
	if (Count == 1)
	{
		// �V���O���u���b�N�������ݗv��
		Response = send_cmd( CMD24, Sector );
		if (Response == 0)
		{
			// �V���O�� �u���b�N ���C�g
			if ( xmit_datablock( pBuff, 0xFE ) == 1 )
			{
				Count = 0;
			}
		}
	}
	// Multiple sector write
	else
	{
		// SD�̏ꍇ
		if ( g_tSdInfo.CardType & CT_SDC )
		{
			// ���̃}���`�u���b�N�������݃R�}���h�ł̃v�������u���b�N����ݒ�(SDC��p)
			send_cmd( ACMD23, Count );
		}
		
		// �}���`�u���b�N��������
		Response = send_cmd( CMD25, Sector );
		if (Response == 0)
		{
			// �Z�N�^�[���A�f�[�^����������
			do
			{
				// Timeout?
				if ( xmit_datablock( pBuff, 0xFC ) == 0 )
				{
					break;
				}
				pBuff += 512;		
			} while ( --Count );
			
			// STOP_TRAN Token
			if ( xmit_datablock( pBuff, 0xFD ) == 0 )
			{
				Count = 1;
			}
		}
	}
				
	sd_deselect( );
			
	return ( (Count == 0) ? RES_OK : RES_ERROR );
}
#endif	// #if DISK_USE_WRITE


//=============================================================================
// Miscellaneous drive controls other than data read/write
//   Drv      : Physical drive number (0)
//   Ctrl 	  : Control command code
//   pBuff    : Pointer to the conrtol data
//=============================================================================
#if DISK_USE_IOCTL
DRESULT disk_ioctl( BYTE Drv, BYTE Ctrl, void *pBuff )
{
	DRESULT				result = RES_ERROR;
	BYTE 				n = 0;
	BYTE				csd[ 16 ];
	BYTE				*ptr = (BYTE *)pBuff;
	DWORD 				*dp, st, ed, csz;
	
	
	// �p�����[�^�`�F�b�N
	if ( Drv )
	{
		return RES_PARERR;
	}
	
	// SD�̏�ԁH
	if ( g_tSdInfo.Stat & STA_NOINIT )
	{
		return RES_NOTRDY;
	}
	
	// Control Command ?
	switch ( Ctrl ) {
	case CTRL_SYNC:					// Wait for end of internal write process of the drive
		if ( sd_select() == 1 )
		{
			result = RES_OK;
		}
		break;
	
	case GET_SECTOR_COUNT:			// Get drive capacity in unit of sector (DWORD)
		if ( (send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16) ) 
		{
			// SDC ver 2.00
			if ((csd[0] >> 6) == 1) 
			{	
				csz = csd[9] + ((WORD)csd[8] << 8) + ((DWORD)(csd[7] & 63) << 16) + 1;
				*(DWORD*)pBuff = csz << 10;
			}
			// SDC ver 1.XX or MMC ver 3
			else 
			{
				n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
				csz = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
				*(DWORD*)pBuff = csz << (n - 9);
			}
			result = RES_OK;
		}
		break;
		
	case GET_BLOCK_SIZE :			// Get erase block size in unit of sector (DWORD)
		// SDC ver 2.00
		if ( g_tSdInfo.CardType & CT_SD2 ) 
		{
			// 	Read SD status
			if ( send_cmd(ACMD13, 0) == 0 ) 
			{
				// Discard 2nd byte of R2 resp.
				xchg_spi(0xFF);									
			
				// Read partial block
				if ( rcvr_datablock( csd, 16 ) ) 
				{					
					for ( n = 64 - 16; n; n--) xchg_spi( 0xFF );		// Purge trailing data
					*(DWORD*)pBuff = 16UL << (csd[10] >> 4);
					result = RES_OK;
				}
			}
		}
		// SDC ver 1.XX or MMC
		else 
		{
			// Read CSD
			if ( (send_cmd( CMD9, 0 ) == 0) && rcvr_datablock( csd, 16) ) 
			{
				// SDC ver 1.XX
				if (g_tSdInfo.CardType & CT_SD1)
				{
					*(DWORD*)pBuff = (((csd[10] & 63) << 1) + ((WORD)(csd[11] & 128) >> 7) + 1) << ((csd[13] >> 6) - 1);
				}
				// MMC
				else 
				{					
					*(DWORD*)pBuff = ((WORD)((csd[10] & 124) >> 2) + 1) * (((csd[11] & 3) << 3) + ((csd[11] & 224) >> 5) + 1);
				}
				result = RES_OK;
			}
		}
		break;
		
#if DISK_USE_WRITE
	case CTRL_TRIM:					// Erase a block of sectors (used when FF_USE_TRIM == 1)
		if (!(g_tSdInfo.CardType & CT_SDC)) break;				// Check if the card is SDC
		if (disk_ioctl(Drv, MMC_GET_CSD, csd)) break;			// Get CSD
		if (!(csd[0] >> 6) && !(csd[10] & 0x40)) break;			// Check if sector erase can be applied to the card
		dp = pBuff; st = dp[0]; ed = dp[1];						// Load sector block
		if (!(g_tSdInfo.CardType & CT_BLOCK)) 
		{
			st *= 512; ed *= 512;
		}
		
		// Erase sector block
		if (send_cmd(CMD32, st) == 0 && send_cmd(CMD33, ed) == 0 && send_cmd(CMD38, 0) == 0 && wait_ready(30000)) 
		{	
			result = RES_OK;
		}
		break;
#endif
	// Following command are not used by FatFs module

	case MMC_GET_TYPE:				// Get MMC/SDC type (BYTE)
		*ptr = g_tSdInfo.CardType;
		result = RES_OK;
		break;

	case MMC_GET_CSD:				// Read CSD (16 bytes)
		// READ_CSD
		if (send_cmd(CMD9, 0) == 0 && rcvr_datablock(ptr, 16)) 
		{	
			result = RES_OK;
		}
		break;

	case MMC_GET_CID:				// Read CID (16 bytes)
		// READ_CID
		if (send_cmd(CMD10, 0) == 0 && rcvr_datablock(ptr, 16))
		{
			result = RES_OK;
		}
		break;

	case MMC_GET_OCR:				// Read OCR (4 bytes)
		// READ_OCR
		if (send_cmd(CMD58, 0) == 0) 
		{
			for (n = 4; n; n--) *ptr++ = xchg_spi(0xFF);
			result = RES_OK;
		}
		break;

	case MMC_GET_SDSTAT :			// Read SD status (64 bytes)
		// SD_STATUS
		if (send_cmd(ACMD13, 0) == 0) 
		{
			xchg_spi(0xFF);
			if (rcvr_datablock(ptr, 64)) result = RES_OK;
		}
		break;

	default:
		result = RES_PARERR;
	}

	sd_deselect();

	return result;
}
#endif	// #if DISK_USE_IOCTL


//=============================================================================
// SD�փR�}���h�𑗐M
//=============================================================================
static BYTE send_cmd( BYTE cmd, DWORD arg )
{
	BYTE					Response = 0x00;
	BYTE					SendData = 0x00;
	BYTE					cnt = 0;
	
	
	// �R�}���h���uCMD55�v or �uACMD�v�̏ꍇ
	if (cmd & 0x80)
	{
		cmd &= 0x7F;
		Response = send_cmd( CMD55, 0 );
		if (Response > 1)
		{
			return Response;
		}
	}
	
	// CMD12�i���[�h��~����j�ȊO�A�J�[�h�`�F�b�N
	if (cmd != CMD12)
	{
		sd_deselect( );
		
		// �^�C���A�E�g�iSD�J�[�h���牞�����Ԃ��Ă��Ȃ������ꍇ�j
		if(sd_select( ) == 0)
		{
			return 0xFF;
		}
	}
	
	// �R�}���h���M
	xchg_spi( 0x40 | cmd );						// Start + command index
	xchg_spi( (BYTE)(arg >> 24) );				// Aggument[31..24]
	xchg_spi( (BYTE)(arg >> 16) );				// Aggument[23..16]
	xchg_spi( (BYTE)(arg >>  8) );				// Aggument[15.. 8]
	xchg_spi( (BYTE) arg );						// Aggument[ 7.. 0]
	
	SendData = 0x01;							// CRC + Stop
	if (cmd == CMD0) SendData = 0x95;			// Valid CRC for CMD0(0)
	if (cmd == CMD8) SendData = 0x87;			// Valid CRC for CMD8(0x1AA)
	xchg_spi( SendData );
	
	// CMD12�i���[�h��~����j�̏ꍇ
	if (cmd == CMD12)
	{
		xchg_spi( 0xFF );
	}
		
	// �����R�[�h����M�icnt�o�C�g���f�[�^�𑗂��āA������҂�)
	cnt = 30;
	do
	{
		Response = xchg_spi( 0xFF );		
	} while( (Response & 0x80) && --cnt );
	
	return Response;
}	


//=============================================================================
// Send a data packet to the MMC
//  1:Ready , 0:Timeout
//=============================================================================
static UINT rcvr_datablock( BYTE *pBuff, UINT ReadSize )
{
	BYTE			token = 0;
	
	
	// �f�[�^�p�P�b�g������̂�҂�
	g_tSdInfo.Timer1 = 200;
	do
	{
		token = xchg_spi( 0xFF );
	} while ( (token == 0xFF) && g_tSdInfo.Timer1 );

	// �f�[�^�p�P�b�g�̃f�[�^�g�[�N����(Data token for CMD17/18/24)��0xFE ?
	if (token != 0xFE)
	{
		return 0;			// Timeout
	}
	
	// Store trailing data to the buffer
	rcvr_spi_multi( pBuff, ReadSize );
	
	// Discard CRC
	xchg_spi( 0xFF ); 
	xchg_spi( 0xFF );

	return 1;				// Success
}


//=============================================================================
// Send a command packet to the MMC
//  1:Ready , 0:Timeout
//=============================================================================
#if DISK_USE_WRITE
static UINT xmit_datablock( const BYTE *pBuff, UINT Token )
{
	BYTE					Response = 0x00;
	
	
	// �R�}���h���X�|���X�ƃf�[�^�p�P�b�g�̊ԂɂP�o�C�g�ȏ゠���Ȃ���΂Ȃ�Ȃ��̂œK���Ȓl�𑗐M
	if ( wait_ready( 500 ) == 0 )
	{
		return 0;
	}
	
	// �g�[�N��(Data Token or Stop Tran Token)�𑗐M
	xchg_spi( Token ); 
	
	// Stop Tran Token(0xFD)�𑗐M�����ꍇ�́A�I��������̂Ŗ{�֐��𔲂���
	if ( Token == 0xFD )
	{
		return 1;				// OK
	}
	
	// �w��o�C�g�����A�����݃f�[�^�𑗐M
	xmit_spi_multi( pBuff, 512 );

	// 1�u���b�N���Ńf�[�^�𑗐M�����������}�ƂȂ�CRC(2Byte)�𑗐M
	xchg_spi( 0xFF ); 
	xchg_spi( 0xFF );
	
	// �f�[�^���X�|���X����M
	Response = xchg_spi( 0xFF );
	
	return ( ((Response & 0x1F) == 0x05) ? 1 : 0 );
}
#endif	// #if DISK_USE_WRITE

//=============================================================================
// SD�J�[�h���������`�F�b�N
//  1:Ready , 0:Timeout
//=============================================================================
static UINT wait_ready( DWORD Timeout )
{
	BYTE					Response = 0x00;
	
	// �^�C���A�E�g�^�C�}�[���Z�b�g
	g_tSdInfo.Timer2 = Timeout;
	
	do
	{
		Response = xchg_spi( 0xFF );
		if (Response == 0xFF)
		{
			return 1;
		}
	} while ( g_tSdInfo.Timer2 );
	
	return 0;
}

	
//=============================================================================
// CS:SD����
//=============================================================================
static void sd_deselect( void )
{
	CS_HIGH( );				// Set CS# high
	xchg_spi( 0xFF );		// Dummy clock (force DO hi-z for multiple slave SPI)
}


//=============================================================================
// CS:SD�I��
//  1:OK , 0:Timeout
//=============================================================================
static UINT sd_select( void )
{
	CS_LOW( );				// Set CS# low
	xchg_spi( 0xFF );		// Dummy clock (force DO enabled)
	
	// SD�J�[�h���������H
	if (wait_ready( 500 ) == 1)
	{
		return 1;
	}
	
	// CS:SD����
	sd_deselect( );
	
	return 0;
}


//=============================================================================
// 1�o�C�g����M
//=============================================================================
static BYTE xchg_spi( BYTE SendData )
{
#if 0
	rspi_interrupts_enable( false );	
	
	// �����݃X�e�[�^�X�t���O�������݂Ȃ��ɂ���
	ICU.IR[ 45 ].BIT.IR = 0;
	
	// ���M�f�[�^���Z�b�g
	RSPI0.SPDR.LONG = SendData;
	
	// ���M���������݂��ʒm�����܂ő҂�
	while( ICU.IR[ 45 ].BIT.IR != 1 )
	{
		nop( );
	}
		
	// ��M�f�[�^��Ԃ�
	return RSPI0.SPDR.LONG;
#endif

	// RSPI IR���W�X�^�N���A
	rspi_ir_clear( );
	
	// ���M�f�[�^���Z�b�g
	RSPI0.SPDR.LONG = SendData;
	
	// ���M���������݂��ʒm�����܂ő҂�
	while( ICU.IR[ IR_RSPI0_SPRI0 ].BYTE != 1 )
	{
		nop( );
	}
	
	// ��M�f�[�^��Ԃ�
	ICU.IR[ IR_RSPI0_SPRI0 ].BYTE = 0;
	return RSPI0.SPDR.LONG;
}
	

//=============================================================================
// �}���`�o�C�g���M
// ��SendSize��4�Ŋ���؂��T�C�Y�ɂ��邱��
//=============================================================================
#if DISK_USE_WRITE
static bool xmit_spi_multi( const BYTE *pSendBuff, UINT SendSize )
{
	const DWORD 			*lp = (const DWORD *)pSendBuff;
	
	// RSPI�f�[�^���ݒ�r�b�g��32bit�ɐݒ�
	RSPI0.SPCMD0.BIT.SPB = 3;
	
	do
	{
		// �����݃X�e�[�^�X�t���O�������݂Ȃ��ɂ���
		SPRI = 0;
		
		// ���M�f�[�^���Z�b�g
		RSPI0.SPDR.LONG = LDDW(*lp);
		lp++;
		
		// ���M���������݂��ʒm�����܂ő҂�
		while( !SPRI ); 	
		
		// ��M�f�[�^��j��
		RSPI0.SPDR.LONG;

	} while( SendSize -= 4 );

	// RSPI�f�[�^���ݒ�r�b�g��8bit�ɐݒ�
	RSPI0.SPCMD0.BIT.SPB = 7;
	
	return true;
}
#endif	// #if DISK_USE_WRITE
	
//=============================================================================
// �}���`�o�C�g��M
// ��RecvSize��4�Ŋ���؂��T�C�Y�ɂ��邱��
//=============================================================================
static bool rcvr_spi_multi( const BYTE *pRecvBuff, UINT RecvSize )
{
	DWORD 					*lp = (DWORD *)pRecvBuff;
	
	// RSPI�f�[�^���ݒ�r�b�g��32bit�ɐݒ�
	RSPI0.SPCMD0.BIT.SPB = 3;
	
	do
	{
		// �����݃X�e�[�^�X�t���O�������݂Ȃ��ɂ���
		SPRI = 0;
		
		// ���M�f�[�^���Z�b�g�i�_�~�[�j
		RSPI0.SPDR.LONG = 0xFFFFFFFF;
		
		// ���M���������݂��ʒm�����܂ő҂�
		while( !SPRI ); 	
	
		// ��M�f�[�^���擾
		*lp = LDDW( RSPI0.SPDR.LONG );
		lp++;
		
	} while( RecvSize -= 4 );
		
	return true;
}

	
//=============================================================================
// SD��SPI���[�h�ɂ���
// �˓d��ON�ɂ��A�J�[�h�͂���{���̃l�C�e�B�u��(SPI���[�h�łȂ�)���샂�[�h�ɓ���܂��B
//   �d���d�����K��͈̔�(2.7�`3.6V)�ɒB�������Ə��Ȃ��Ƃ�1ms�҂��ADI,CS��H���x���ɂ���SCLK��74�N���b�N�ȏ�����ƃR�}���h���󂯕t���鏀�����ł��܂��B
//=============================================================================
static void sd_spimode( void )
{
	UINT			i = 0;
	
	for ( i = 0 ; i < 10 ; i++ )
	{
		xchg_spi( 0xFF );
	}
}
		

//=============================================================================
// RSPI���g�p�ł���悤�ɂ���
//=============================================================================
static void rspi_enable( void )
{
	//---------------------
	// RSPI��L���ɂ���
	//---------------------
	SYSTEM.PRCR.WORD = 0xA502;						// ���샂�[�h�A����d�͒ጸ�@�\�A�\�t�g�E�F�A���Z�b�g�֘A���W�X�^�ւ̏������݋��F����
	MSTP_RSPI0 = 0;									// �V���A���y���t�F�����C���^�t�F�[�X0���W���[���X�g�b�v�ݒ�r�b�g(SYSTEM.MSTPCRB.BIT.MSTPB17)
													//   0�F���W���[���X�g�b�v��Ԃ̉���
													//   1�F���W���[���X�g�b�v��Ԃ֑J��
	SYSTEM.PRCR.WORD = 0xA500;						// ���샂�[�h�A����d�͒ጸ�@�\�A�\�t�g�E�F�A���Z�b�g�֘A���W�X�^�ւ̏������݋��F����
	if (SYSTEM.PRCR.WORD == 0xA500)
	{
		nop( );
	}
	
	//---------------------
	// RSPI�̐ݒ�
	//---------------------
	RSPI0.SPCR.BYTE = 0x00;							// RSPI�@�\���~
	while( RSPI0.SPCR.BYTE != 0x00 );
	
	// ---[ RSPI �[�q���䃌�W�X�^�iSPPCR�j]---	
	RSPI0.SPPCR.BYTE = 0x00;						// RSPI���[�v�o�b�N�r�b�g(SPLP)  �� 0:�ʏ탂�[�h 
													//   0�F�ʏ탂�[�h
													//   1�F���[�v�o�b�N���[�h�i���M�f�[�^�̔��]����M�f�[�^�j
													//
													// RSPI���[�v�o�b�N2�r�b�g(SPLP2) �� 0:�ʏ탂�[�h 
													//   0�F�ʏ탂�[�h
													//   1�F���[�v�o�b�N���[�h�i���M�f�[�^����M�f�[�^�j
													//
													// MOSI�A�C�h���Œ�l�r�b�g(MOIFV) �� 0:MOSI�A�C�h������MOSIA�[�q�̏o�͒l��Low
													//   0�FMOSI�A�C�h������MOSIA�[�q�̏o�͒l��Low
													//   1�FMOSI�A�C�h������MOSIA�[�q�̏o�͒l��High
													//
													// MOSI�A�C�h���l�Œ苖�r�b�g(MOIFE) �� 0�FMOSI�o�͒l�͑O��]���̍ŏI�f�[�^
													//   0�FMOSI�o�͒l�͑O��]���̍ŏI�f�[�^
													//   1�FMOSI�o�͒l��MOIFV�r�b�g�̐ݒ�l

	// ---[ RSPI �r�b�g���[�g���W�X�^�iSPBR�j]---
	RSPI0.SPBR = calc_spbr( SCLK_SLOW );			// 400KHz

	// ---[ RSPI �f�[�^�R���g���[�����W�X�^�iSPDCR�j]---													
	RSPI0.SPDCR.BYTE = 0x20;						// �t���[�����ݒ�r�b�g(SPFC[1:0]) �� 0 0�F1�t���[��
													//   0 0�F1�t���[��
													//   0 1�F2�t���[��
													//   1 0�F3�t���[��
													//   1 1�F4�t���[��
													//
													// RSPI��M/���M�f�[�^�I���r�b�g(SPRDTD) �� 0�FSPDR�͎�M�o�b�t�@��ǂݏo��
													//   0�FSPDR�͎�M�o�b�t�@��ǂݏo��
													//   1�FSPDR�͑��M�o�b�t�@��ǂݏo���i�������A���M�o�b�t�@����̂Ƃ��j
													//
													// RSPI�����O���[�h�A�N�Z�X/���[�h�A�N�Z�X�ݒ�r�b�g(SPLW) �� 1�FSPDR���W�X�^�ւ̓����O���[�h�A�N�Z�X
													//   0�FSPDR���W�X�^�ւ̓��[�h�A�N�Z�X , 
													//   1�FSPDR���W�X�^�ւ̓����O���[�h�A�N�Z�X

	// ---[ RSPI �N���b�N�x�����W�X�^�iSPCKD�j ]---
	RSPI0.SPCKD.BYTE = 0x00;						// RSPCK�x���ݒ�r�b�g(SCKDL[2:0]) �� 0 0 0�F1RSPCK
													//   0 0 0�F1RSPCK
													//   0 0 1�F2RSPCK
													//   0 1 0�F3RSPCK
													//   0 1 1�F4RSPCK
													//   1 0 0�F5RSPCK
													//   1 0 1�F6RSPCK
													//   1 1 0�F7RSPCK
													//   1 1 1�F8RSPCK	

	// ---[ RSPI �X���[�u�Z���N�g�l�Q�[�g�x�����W�X�^�iSSLND�j ]---
	RSPI0.SSLND.BYTE = 0x00;						// SSL�l�Q�[�g�x���ݒ�r�b�g(SLNDL[2:0]) �� 0 0 0�F1RSPCK
													//   0 0 0�F1RSPCK
													//   0 0 1�F2RSPCK
													//   0 1 0�F3RSPCK
													//   0 1 1�F4RSPCK
													//   1 0 0�F5RSPCK
													//   1 0 1�F6RSPCK
													//   1 1 0�F7RSPCK
													//   1 1 1�F8RSPCK
													
	// ---[ RSPI ���A�N�Z�X�x�����W�X�^�iSPND�j ]---
	RSPI0.SPND.BYTE = 0x00;							// RSPI���A�N�Z�X�x���ݒ�r�b�g(SPNDL[2:0]) �� 0 0 0�F1RSPCK�{2PCLK
													//   0 0 0�F1RSPCK�{2PCLK
													//   0 0 1�F2RSPCK�{2PCLK
													//   0 1 0�F3RSPCK�{2PCLK
													//   0 1 1�F4RSPCK�{2PCLK
													//   1 0 0�F5RSPCK�{2PCLK
													//   1 0 1�F6RSPCK�{2PCLK
													//   1 1 0�F7RSPCK�{2PCLK
													//   1 1 1�F8RSPCK�{2PCLK	
	
	// ---[ RSPI ���䃌�W�X�^2�iSPCR2�j ]---
	RSPI0.SPCR2.BYTE = 0x00;						// �p���e�B���r�b�g(SPPE)
													//   0�F���M�f�[�^�p���e�B�r�b�g��t�����Ȃ�
													//      ��M�f�[�^�̃p���e�B�`�F�b�N���s��Ȃ�
													//   1�F���M�f�[�^�Ƀp���e�B�r�b�g��t�����A��M�f�[�^�̃p��
													//      �e�B�`�F�b�N���s���iSPCR.TXMD=0�̂Ƃ��j
													//      ���M�f�[�^�Ƀp���e�B�r�b�g��t�����邪�A��M�f�[�^�̃p���e�B�`�F�b�N�͍s��Ȃ��iSPCR.TXMD=1�̂Ƃ��j
													//
													// �p���e�B���[�h�r�b�g(SPOE) �� 0�F�����p���e�B�ő���M
													//   0�F�����p���e�B�ő���M
													//   1�F��p���e�B�ő���M
													//
													// RSPI�A�C�h�����荞�݋��r�b�g(SPIIE) �� 0�F�A�C�h�����荞�ݗv���̔������֎~
													//   0�F�A�C�h�����荞�ݗv���̔������֎~
													//   1�F�A�C�h�����荞�ݗv���̔���������
													//
													// �p���e�B���Ȕ��f�r�b�g(PTE) �� 0�F�p���e�B��H���Ȑf�f�@�\�͖���
													//   0�F�p���e�B��H���Ȑf�f�@�\�͖���
													//   1�F�p���e�B��H���Ȑf�f�@�\���L��
													
	// ---[ RSPI �V�[�P���X���䃌�W�X�^�iSPSCR�j]---												
	RSPI0.SPSCR.BYTE = 0x00;						// RSPI�V�[�P���X���ݒ�r�b�g(SPSLN) ��  0 0 0�F�@�@1
													//   0 0 0�F�@�@1�@�@�@�@�@�@0��0���c
													//   0 0 1�F�@�@2�@�@�@�@�@�@0��1��0���c
													//   0 1 0�F�@�@3�@�@�@�@�@�@0��1��2��0���c
													//   0 1 1�F�@�@4�@�@�@�@�@�@0��1��2��3��0���c
													//   1 0 0�F�@�@5�@�@�@�@�@�@0��1��2��3��4��0���c
													//   1 0 1�F�@�@6�@�@�@�@�@�@0��1��2��3��4��5��0���c
													//   1 1 0�F�@�@7�@�@�@�@�@�@0��1��2��3��4��5��6��0���c
													//   1 1 1�F�@�@8�@�@�@�@�@�@0��1��2��3��4��5��6��7��0���c	
													// ��RSPI �R�}���h���W�X�^0 �` 7�̎Q�Ə��Ԃ�ύX
	
													
	// ---[ RSPI �R�}���h���W�X�^0 �` 7�iSPCMD0 �` SPCMD7�j]---
	RSPI0.SPCMD0.WORD = 0x0700;						// RSPCK�ʑ��ݒ�r�b�g(CPHA) �� 0�F��G�b�W�Ńf�[�^�T���v���A�����G�b�W�Ńf�[�^�ω�
													//   0�F��G�b�W�Ńf�[�^�T���v���A�����G�b�W�Ńf�[�^�ω�
													//   1�F��G�b�W�Ńf�[�^�ω��A�����G�b�W�Ńf�[�^�T���v��
													//
													// RSPCK�ɐ��ݒ�r�b�g(CPOL) �� 0�F�A�C�h������RSPCK��Low
													//   0�F�A�C�h������RSPCK��Low
													//   1�F�A�C�h������RSPCK��High
													//
													// �r�b�g���[�g�����ݒ�r�b�g(BRDV[1:0]) �� 0 0�F�x�[�X�̃r�b�g���[�g��I��
													//   0 0�F�x�[�X�̃r�b�g���[�g��I��
													//   0 1�F�x�[�X�̃r�b�g���[�g��2������I��
													//   1 0�F�x�[�X�̃r�b�g���[�g��4������I��
													//   1 1�F�x�[�X�̃r�b�g���[�g��8������I��
													//
													// SSL�M���A�T�[�g�ݒ�r�b�g(SSLA[2:0]) �� 0 0 0�FSSL0
													//   0 0 0�FSSL0
													//   0 0 1�FSSL1
													//   0 1 0�FSSL2
													//   0 1 1�FSSL3
													//   1 x x�F�ݒ肵�Ȃ��ł�������
													//
													// SSL�M�����x���ێ��r�b�g(SSLKP)
													//   0�F�]���I�����ɑSSSL�M�����l�Q�[�g
													//   1�F�]���I���ォ�玟�A�N�Z�X�J�n�܂�SSL�M�����x����ێ�
													//
													// RSPI�f�[�^���ݒ�r�b�g(SPB[3:0]) �� 0 1 1 1 : 8�r�b�g
													//  0100 �` 0111 �F8�r�b�g
													//  1 0 0 0      �F9�r�b�g
													//  1 0 0 1      �F10�r�b�g
													//  1 0 1 0      �F11�r�b�g
													//  1 0 1 1      �F12�r�b�g
													//  1 1 0 0      �F13�r�b�g
													//  1 1 0 1      �F14�r�b�g
													//  1 1 1 0      �F15�r�b�g
													//  1 1 1 1      �F16�r�b�g
													//  0 0 0 0      �F20�r�b�g
													//  0 0 0 1      �F24�r�b�g
													//  0010�A0011   �F32�r�b�g
													//
													// RSPI LSB�t�@�[�X�g�r�b�g(LSBF) �� 0�FMSB�t�@�[�X�g
													//   0�FMSB�t�@�[�X�g
													//   1�FLSB�t�@�[�X�g
													// 
													// RSPI���A�N�Z�X�x�����r�b�g(SPNDEN) �� 0�F���A�N�Z�X�x����1RSPCK�{2PCLK
													//   0�F���A�N�Z�X�x����1RSPCK�{2PCLK
													//   1�F���A�N�Z�X�x����RSPI���A�N�Z�X�x�����W�X�^�iSPND�j�̐ݒ�l
													// 
													// SSL�l�Q�[�g�x���ݒ苖�r�b�g(SLNDEN) �� 0�FSSL�l�Q�[�g�x����1RSPCK
													//   0�FSSL�l�Q�[�g�x����1RSPCK
													//   1�FSSL�l�Q�[�g�x����RSPI�X���[�u�Z���N�g�l�Q�[�g�x�����W�X�^�iSSLND�j�̐ݒ�l
													// RSPCK�x���ݒ苖�r�b�g(SCKDEN) �� 0�FRSPCK�x����1RSPCK
													//   0�FRSPCK�x����1RSPCK
													//   1�FRSPCK�x����RSPI�N���b�N�x�����W�X�^�iSPCKD�j�̐ݒ�l
													
	
	// ---[RSPI ���䃌�W�X�^�iSPCR�j]---
	RSPI0.SPCR.BYTE = 0xC9;							// RSPI���[�h�I���r�b�g(SPMS) �� 1�F�N���b�N����������i3�����j
													//   0�FSPI����i4�����j
													//   1�F�N���b�N����������i3�����j
													//
													// �ʐM���샂�[�h�I���r�b�g(TXMD) �� 0�F�S��d�������V���A���ʐM
													//   0�F�S��d�������V���A���ʐM
													//   1�F���M����݂̂̃V���A���ʐM
													//
													// ���[�h�t�H���g�G���[���o���r�b�g(MODFEN) �� 0�F���[�h�t�H���g�G���[���o���֎~
													//   0�F���[�h�t�H���g�G���[���o���֎~
													//   1�F���[�h�t�H���g�G���[���o������
													//
													// RSPI�}�X�^/�X���[�u���[�h�I���r�b�g(MSTR) �� 1�F�}�X�^���[�h
													//   0�F�X���[�u���[�h
													//   1�F�}�X�^���[�h
													//
													// RSPI�G���[���荞�݋��r�b�g(SPEIE) �� 0�FRSPI�G���[���荞�ݗv���̔������֎~
													//   0�FRSPI�G���[���荞�ݗv���̔������֎~
													//   1�FRSPI�G���[���荞�ݗv���̔���������
													//   
													// RSPI���M���荞�݋��r�b�g(SPTIE) �� 0�FRSPI���M���荞�ݗv���̔������֎~
													//   0�FRSPI���M���荞�ݗv���̔������֎~
													//   1�FRSPI���M���荞�ݗv���̔���������
													//
													// RSPI�@�\���r�b�g(SPE) �� 1�FRSPI�@�\�͗L��
													//   0�FRSPI�@�\�͖���
													//   1�FRSPI�@�\�͗L��
													//
													// RSPI��M���荞�݋��r�b�g(SPRIE) �� 1�FRSPI��M���荞�ݗv���̔���������
													//   0�FRSPI��M���荞�ݗv���̔������֎~
													//   1�FRSPI��M���荞�ݗv���̔���������
	if (RSPI0.SPCR.BYTE == 0xC9)
	{
		nop( );
	}

	//---------------------
	// RSPI�֘A�̒[�q�ݒ�
	//---------------------
//  MPC.PWPR.BYTE = 0x40;							// Unlock MPC
    /* Enable writing of PFSWE bit. */
    MPC.PWPR.BIT.B0WI = 0;
    /* Enable writing to PFS registers. */ 
    MPC.PWPR.BIT.PFSWE = 1;

	
    // Set RSPCKA pin								// CLK[OUT] - PA5
    MPC.PA5PFS.BYTE = 0x0DU;						// RSPI�Ƃ��Ďg�p
    PORTA.PMR.BIT.B5 = 1U;							// 1:���Ӌ@��Ƃ��Ďg�p

    // Set MOSIA pin 								// SMI[IN]  - PA6
    MPC.PA6PFS.BYTE = 0x0DU;						// RSPI�Ƃ��Ďg�p
    PORTA.PMR.BIT.B6 = 1U;							// 1:���Ӌ@��Ƃ��Ďg�p

    // Set MISOA pin 								// SDO[OUT] - PA7
    MPC.PA7PFS.BYTE = 0x0DU;						// RSPI�Ƃ��Ďg�p
    PORTA.PMR.BIT.B7 = 1U;							// 1:���Ӌ@��Ƃ��Ďg�p
	PORTA.PCR.BIT.B7 = 1;							// A pull-up on MISO

	// Set CS pin									// SC[OUT] - PB7
	PORTB.PODR.BIT.B7 = 1;
	CS_HIGH( );	

//	MPC.PWPR.BYTE = 0x80;							// Lock MPC
	MPC.PWPR.BIT.B0WI = 0;     
	/* Disable writing to PFS registers. */ 
	MPC.PWPR.BIT.PFSWE = 0;    
	/* Disable writing of PFSWE bit. */
	MPC.PWPR.BIT.B0WI = 1;     
 	
}	
	

//=============================================================================
// RSPI �r�b�g���[�g���W�X�^�ɐݒ肷��l���v�Z����
//=============================================================================
static BYTE calc_spbr( DWORD bps_target )
{
	BYTE					spbr_result = 0;
	DWORD					f = F_PCLK;					// Frequency
	signed long				n = 0;						// n term in equation
	signed long				N = 0;						// N term in equation
	signed long				calc = 0;
	
	
	// 2�������Ȃ��ꍇ
	calc = f / bps_target;
	if ( calc < 2 )
	{
		return 0;
	}
	
	//////////////////////////////////////////////////////////////
	// From Hardware manual: Bit rate = f / (2(n + 1)(2^N))
    // where:
    //      f = PCLK, n = SPBR setting, N = BRDV bits
    // Solving for n:
    //      n = (((f/(2^N))/2) / bps) - 1
	//////////////////////////////////////////////////////////////
    
	// Only calculate for BRDV value of 0 (div/1) to get SPBR setting for the board PCLK.
    // BRDV setting will be done during write/read operations.
	N = 0;
	n = ((f >> (N+1)) / (signed long)bps_target) - 1;
	if ( !((n >= 0) && (n <= 0xFF)) )
	{
		return 0;
	}
	
	calc = (DWORD)(f / (2 * ((n + 1) << N)));
	if (calc > bps_target)
	{
		n += 1;
		if ( n > 0xFF )
		{
			return 0;
		}
	}
	spbr_result = n;
	
	return spbr_result;
}
	

//=============================================================================
// RSPI�����ݗL���^����
//=============================================================================
static void rspi_interrupts_enable( bool enabled )
{
	// Disable or enable receive buffer full interrupt
	IEN(RSPI0, SPRI0) = enabled;
	
	// Disable  or enable transmit buffer empty interrupt
	IEN(RSPI0, SPTI0) = enabled;
	
	// Disable or enable error interrupt
	IEN(RSPI0, SPEI0) = enabled;
	
    if (0 == IEN(RSPI0, SPTI0))
    {
        nop();
    }
}
	

//=============================================================================
// SPSR���W�X�^�N���A
//=============================================================================
static void rspi_spsr_clear( void )
{
	// �I�[�o�[�����G���[�E���[�h�t�H���g�G���[�E�p���e�B�G���[�H
	if (RSPI0.SPSR.BYTE & 0x0D)
	{
		// // �I�[�o�[�����G���[�^���[�h�t�H���g�G���[�^�p���e�B�G���[�t���O���N���A
		RSPI0.SPSR.BYTE = 0xA0;
		
		while( (RSPI0.SPSR.BYTE & 0x0D) != 0x00 );
	}
}	


//=============================================================================
// RSPI IR ���W�X�^�N���A
//=============================================================================
static void rspi_ir_clear( void )
{
//	// RSPI�@�\���L���̏ꍇ
//	if (RSPI0.SPCR.BIT.SPE == 1)
//	{
//		// RSPI�@�\�𖳌��ɂ���
//		RSPI0.SPCR.BIT.SPE = 0;
//		while( RSPI0.SPCR.BIT.SPE != 0 );
//	}
	
	// �I�[�o�[�����G���[�^���[�h�t�H���g�G���[�^�p���e�B�G���[�t���O���N���A
	RSPI0.SPSR.BYTE = 0xA0;
	while( (RSPI0.SPSR.BYTE & 0x0D) != 0x00 );
	
	// ���M�f�[�^�G���v�e�B���N���A
	ICU.IR[ IR_RSPI0_SPTI0 ].BYTE = 0;
	
	// ��M�f�[�^�t�����N���A
	ICU.IR[ IR_RSPI0_SPRI0 ].BYTE = 0;
	if (ICU.IR[ IR_RSPI0_SPRI0 ].BYTE == 0)
	{
		nop( );
	}
}
		 
	

