//*************************************************************************************************
// SDドライバ
// ※GPIOのPB-0をCSとして使用します
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


#define SdCS_GPIO_PIN						( PORTB.PODR.BIT.B7 )	// CS端子
#define CS_HIGH( )							( SdCS_GPIO_PIN = 1 )	
#define CS_LOW( )							( SdCS_GPIO_PIN = 0 )

#define	SPRI								IR( RSPI0, SPRI0 )

#define F_PCLK								( 25000000UL )			// PCLK frequency (configured by SCKCR.PCKB)
#define SCLK_FAST							( 16000000UL )			// SCLK frequency (R/W)
#define	SCLK_SLOW							(   400000UL )			// SCLK frequency (Init)



// SDコマンド
#define CMD0	(0)					// ソフトウェアリセット 		- [GO_IDLE_STATE]
#define CMD1	(1)					// 初期化開始 					- [SEND_OP_COND (MMC)]
#define	ACMD41	(0x80+41)			// 初期化開始(SDC専用) 			- [SEND_OP_COND (SDC)]
#define CMD8	(8)					// SDC V2専用。動作電圧確認 	- [SEND_IF_COND]
#define CMD9	(9)					// CSD読み出し 					- [SEND_CSD]
#define CMD10	(10)				// CID読み出し					- [SEND_CID]
#define CMD12	(12)				// リード動作停止				- [STOP_TRANSMISSION]
#define ACMD13	(0x80+13)			// 状態取得(SDC専用)			- [SD_STATUS (SDC)]
#define CMD16	(16)				// R/Wブロック長変更			- [SET_BLOCKLEN]
#define CMD17	(17)				// シングルブロック読み出し		- [READ_SINGLE_BLOCK]
#define CMD18	(18)				// マルチブロック読み出し		- [READ_MULTIPLE_BLOCK]
#define CMD23	(23)				// 次のマルチブロック読み出し/書き込みコマンドでの転送ブロック数を設定(MMC専用)	- [SET_BLOCK_COUNT (MMC)]
#define	ACMD23	(0x80+23)			// 次のマルチブロック書き込みコマンドでのプレ消去ブロック数を設定(SDC専用)		- [SET_WR_BLK_ERASE_COUNT (SDC)]
#define CMD24	(24)				// シングルブロック書き込み		- [WRITE_BLOCK]
#define CMD25	(25)				// マルチブロック書き込み		- [WRITE_MULTIPLE_BLOCK]
#define CMD32	(32)				// ERASE_ER_BLK_START
#define CMD33	(33)				// ERASE_ER_BLK_END
#define CMD38	(38)				// ERASE
#define CMD55	(55)				// アプリケーション特化コマンド	- [APP_CMD]
#define CMD58	(58)				// OCR読み出し					- [READ_OCR]



// SD情報テーブル構造体
typedef struct _SD_INFO_TABLE
{
	BYTE										CardType;								// カードタイプ
	DSTATUS 									Stat;									// Physical drive status

	DWORD										Timer1;
	DWORD										Timer2;
	
} SD_INFO_TABLE;





// グローバル変数
static SD_INFO_TABLE							g_tSdInfo =	{ 0, STA_NOINIT, 0, 0 };	// SD情報テーブル

	


// プロトタイプ宣言
static BYTE send_cmd( BYTE cmd, DWORD arg );											// SDへコマンドを送信
static UINT rcvr_datablock( BYTE *pBuff, UINT ReadSize );								// Send a data packet to the MMC

#if DISK_USE_WRITE
static UINT xmit_datablock( const BYTE *pBuff, UINT Token );							// Send a command packet to the MMC
#endif	// #if DISK_USE_WRITE

static UINT wait_ready( DWORD Timeout );												// SDカード準備完了チェック
static void sd_deselect( void );														// CS:SD解除
static UINT sd_select( void );															// CS:SD選択
static BYTE xchg_spi( BYTE SendData );													// 1バイト送受信

#if DISK_USE_WRITE
static bool xmit_spi_multi( const BYTE *pSendBuff, UINT SendSize );						// マルチバイト送信
#endif	// #if DISK_USE_WRITE

static bool rcvr_spi_multi( const BYTE *pRecvBuff, UINT RecvSize );						// マルチバイト受信
static void sd_spimode( void );															// SDをSPIモードにする
static void rspi_enable( void );														// RSPIを使用できるようにする
static BYTE calc_spbr( DWORD bps_target );												// RSPI ビットレートレジスタに設定する値を計算する
static void rspi_interrupts_enable( bool enabled );										// RSPI割込み有効／無効
static void rspi_spsr_clear( void );													// SPSRレジスタクリア
static void rspi_ir_clear( void );														// RSPI IR レジスタクリア


//=============================================================================
// RSPI 1msタイマー処理
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
// SD初期化
//=============================================================================
DSTATUS disk_initialize( BYTE Drv )
{
	BYTE					Response = 0x00;
	BYTE					ocr[ 4 ] = { 0x00, 0x00, 0x00, 0x00 };
	BYTE					i = 0;
	BYTE					card_type = 0;
	BYTE					cmd = 0;
	
	
	// ドライブ0以外はエラー
	if (Drv != 0)
	{
		g_tSdInfo.Stat |= STA_NOINIT;
		return STA_NOINIT;
	}
	
	// RSPIを使用できるようにする
	rspi_enable( );

	// SDをSPIモードにする
	sd_spimode( );

	// ソフトウェアリセット
	card_type = 0;
	Response = send_cmd( CMD0, 0 );
	if (Response == 1)
	{
		g_tSdInfo.Timer1 = 1000;
		
		// SDC V2専用 - 動作電圧確認
		Response = send_cmd( CMD8, 0x1AA );
		if (Response == 1)
		{
			// R7の応答を取得
			for ( i = 0 ; i < 4 ; i++ )
			{
				ocr[ i ] = xchg_spi( 0xFF );
			}

			if ( (ocr[ 2 ] == 0x01) && (ocr[ 3 ] == 0xAA ) )
			{
				// SD初期化(SDv2)
				while( g_tSdInfo.Timer1 && send_cmd( ACMD41, 1UL << 30 ) );				// Wait for end of initialization with ACMD41(HCS)
				if (g_tSdInfo.Timer1 && send_cmd( CMD58, 0 ) == 0)						// Check CCS bit in the OCR
				{
					// R7の応答を取得
					for ( i = 0 ; i < 4 ; i++ )
					{
						ocr[ i ] = xchg_spi( 0xFF );
					}
					card_type = (ocr[ 0 ] & 0x40) ? (CT_SD2 | CT_BLOCK) : CT_SD2;		// Check if the card is SDv2
				}
			}
			else
			{
				// SDv2でない場合
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
		
				// SD初期化(MMC or SDv1)
				while ( g_tSdInfo.Timer1 && send_cmd( cmd, 0 ) != 0 );					// Wait for end of initialization 					

				// ブロック長を512に変更
				if( !g_tSdInfo.Timer1 || (send_cmd( CMD16, 512 ) != 0) )
				{
					// 変更に失敗した場合はエラーとするため、カードタイプを0にする
					card_type = 0;
				}				
			}
		}
	}
	
	// カードタイプをセット
	g_tSdInfo.CardType = card_type;
	
	sd_deselect( );		
	
	if (g_tSdInfo.CardType == 0)
	{
		g_tSdInfo.Stat |= STA_NOINIT;
		return STA_NOINIT;
	}
		
	// RSPIの通信速度を上げる(16MHz)
	RSPI0.SPCR.BIT.SPE = 0;
	RSPI0.SPBR = calc_spbr( SCLK_FAST );
	RSPI0.SPCR.BIT.SPE = 1;

	
	g_tSdInfo.Stat &= ~STA_NOINIT;
	
	return g_tSdInfo.Stat;
}


//=============================================================================
// SDの状態取得
//=============================================================================
DSTATUS disk_status( BYTE Drv )
{
	// ドライブ0以外はエラー
	if (Drv != 0)
	{
		return STA_NOINIT;
	}
	
	return g_tSdInfo.Stat;
}


//=============================================================================
// SD読込み
//   Drv      : Physical drive number (0)
//   pBuff 	  : Pointer to the data buffer to store read data
//   Sector   : Start sector number (LBA)
//   Count    : Number of sectors to read (1..128)
//=============================================================================
DRESULT disk_read( BYTE Drv, BYTE *pBuff, DWORD Sector, UINT Count )
{
	BYTE					Response = 0x00;
	
	
	// パラメータチェック
	if (Drv || !Count)
	{
		return RES_PARERR;
	}
	
	// SDの状態？
	if (g_tSdInfo.Stat & STA_NOINIT)
	{
		return RES_NOTRDY;
	}
	
	// ブロックアドレスに対応していないSDカードの場合
	if ( !(g_tSdInfo.CardType & CT_BLOCK) )
	{
		// LBA ot BA conversion (byte addressing cards)
		Sector *= 512;
	}
	
	// Single sector read
	if (Count == 1)
	{
		// シングルブロック読み出し要求
		Response = send_cmd( CMD17, Sector );
		if (Response == 0)
		{
			// シングルブロックリード
			if (rcvr_datablock( pBuff, 512 ) == 1)
			{
				Count = 0;
			}
		}
	}
	// Multiple sector read
	else
	{
		// マルチブロック読み出し
		Response = send_cmd( CMD18, Sector );
		if (Response == 0)
		{
			// セクター分、データを読み込む
			do
			{
				// Timeout?
				if (rcvr_datablock( pBuff, 512 ) == 0)
				{
					break;
				}
				pBuff += 512;							
			} while ( --Count );
			
			// リード動作停止
			send_cmd( CMD12, 0 );
		}
	}
		
	sd_deselect( );
			
	return ( (Count == 0) ? RES_OK : RES_ERROR );
}


//=============================================================================
// SD書込み
//   Drv      : Physical drive number (0)
//   pBuff 	  : Pointer to the data buffer to store read data
//   Sector   : Start sector number (LBA)
//   Count    : Number of sectors to read (1..128)
//=============================================================================
#if DISK_USE_WRITE
DRESULT disk_write( BYTE Drv, const BYTE *pBuff, DWORD Sector, UINT Count )
{
	BYTE					Response = 0x00;
	
	
	// パラメータチェック
	if (Drv || !Count)
	{
		return RES_PARERR;
	}
	
	// SDの状態？
	if (g_tSdInfo.Stat & STA_NOINIT)
	{
		return RES_NOTRDY;
	}
	
	if (g_tSdInfo.Stat & STA_PROTECT)
	{
		return RES_WRPRT;
	}
	
	// ブロックアドレスに対応していないSDカードの場合
	if ( !(g_tSdInfo.CardType & CT_BLOCK) )
	{
		// LBA ot BA conversion (byte addressing cards)
		Sector *= 512;
	}
	
	//  Single sector write
	if (Count == 1)
	{
		// シングルブロック書き込み要求
		Response = send_cmd( CMD24, Sector );
		if (Response == 0)
		{
			// シングル ブロック ライト
			if ( xmit_datablock( pBuff, 0xFE ) == 1 )
			{
				Count = 0;
			}
		}
	}
	// Multiple sector write
	else
	{
		// SDの場合
		if ( g_tSdInfo.CardType & CT_SDC )
		{
			// 次のマルチブロック書き込みコマンドでのプレ消去ブロック数を設定(SDC専用)
			send_cmd( ACMD23, Count );
		}
		
		// マルチブロック書き込み
		Response = send_cmd( CMD25, Sector );
		if (Response == 0)
		{
			// セクター分、データを書き込む
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
	
	
	// パラメータチェック
	if ( Drv )
	{
		return RES_PARERR;
	}
	
	// SDの状態？
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
// SDへコマンドを送信
//=============================================================================
static BYTE send_cmd( BYTE cmd, DWORD arg )
{
	BYTE					Response = 0x00;
	BYTE					SendData = 0x00;
	BYTE					cnt = 0;
	
	
	// コマンドが「CMD55」 or 「ACMD」の場合
	if (cmd & 0x80)
	{
		cmd &= 0x7F;
		Response = send_cmd( CMD55, 0 );
		if (Response > 1)
		{
			return Response;
		}
	}
	
	// CMD12（リード停止動作）以外、カードチェック
	if (cmd != CMD12)
	{
		sd_deselect( );
		
		// タイムアウト（SDカードから応答が返ってこなかった場合）
		if(sd_select( ) == 0)
		{
			return 0xFF;
		}
	}
	
	// コマンド送信
	xchg_spi( 0x40 | cmd );						// Start + command index
	xchg_spi( (BYTE)(arg >> 24) );				// Aggument[31..24]
	xchg_spi( (BYTE)(arg >> 16) );				// Aggument[23..16]
	xchg_spi( (BYTE)(arg >>  8) );				// Aggument[15.. 8]
	xchg_spi( (BYTE) arg );						// Aggument[ 7.. 0]
	
	SendData = 0x01;							// CRC + Stop
	if (cmd == CMD0) SendData = 0x95;			// Valid CRC for CMD0(0)
	if (cmd == CMD8) SendData = 0x87;			// Valid CRC for CMD8(0x1AA)
	xchg_spi( SendData );
	
	// CMD12（リード停止動作）の場合
	if (cmd == CMD12)
	{
		xchg_spi( 0xFF );
	}
		
	// 応答コードを受信（cntバイト分データを送って、応答を待つ)
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
	
	
	// データパケットが来るのを待つ
	g_tSdInfo.Timer1 = 200;
	do
	{
		token = xchg_spi( 0xFF );
	} while ( (token == 0xFF) && g_tSdInfo.Timer1 );

	// データパケットのデータトークンが(Data token for CMD17/18/24)の0xFE ?
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
	
	
	// コマンドレスポンスとデータパケットの間に１バイト以上あけなければならないので適当な値を送信
	if ( wait_ready( 500 ) == 0 )
	{
		return 0;
	}
	
	// トークン(Data Token or Stop Tran Token)を送信
	xchg_spi( Token ); 
	
	// Stop Tran Token(0xFD)を送信した場合は、終了させるので本関数を抜ける
	if ( Token == 0xFD )
	{
		return 1;				// OK
	}
	
	// 指定バイト数分、書込みデータを送信
	xmit_spi_multi( pBuff, 512 );

	// 1ブロック分でデータを送信完了した合図となるCRC(2Byte)を送信
	xchg_spi( 0xFF ); 
	xchg_spi( 0xFF );
	
	// データレスポンスを受信
	Response = xchg_spi( 0xFF );
	
	return ( ((Response & 0x1F) == 0x05) ? 1 : 0 );
}
#endif	// #if DISK_USE_WRITE

//=============================================================================
// SDカード準備完了チェック
//  1:Ready , 0:Timeout
//=============================================================================
static UINT wait_ready( DWORD Timeout )
{
	BYTE					Response = 0x00;
	
	// タイムアウトタイマーをセット
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
// CS:SD解除
//=============================================================================
static void sd_deselect( void )
{
	CS_HIGH( );				// Set CS# high
	xchg_spi( 0xFF );		// Dummy clock (force DO hi-z for multiple slave SPI)
}


//=============================================================================
// CS:SD選択
//  1:OK , 0:Timeout
//=============================================================================
static UINT sd_select( void )
{
	CS_LOW( );				// Set CS# low
	xchg_spi( 0xFF );		// Dummy clock (force DO enabled)
	
	// SDカード準備完了？
	if (wait_ready( 500 ) == 1)
	{
		return 1;
	}
	
	// CS:SD解除
	sd_deselect( );
	
	return 0;
}


//=============================================================================
// 1バイト送受信
//=============================================================================
static BYTE xchg_spi( BYTE SendData )
{
#if 0
	rspi_interrupts_enable( false );	
	
	// 割込みステータスフラグを割込みなしにする
	ICU.IR[ 45 ].BIT.IR = 0;
	
	// 送信データをセット
	RSPI0.SPDR.LONG = SendData;
	
	// 送信完了割込みが通知されるまで待つ
	while( ICU.IR[ 45 ].BIT.IR != 1 )
	{
		nop( );
	}
		
	// 受信データを返す
	return RSPI0.SPDR.LONG;
#endif

	// RSPI IRレジスタクリア
	rspi_ir_clear( );
	
	// 送信データをセット
	RSPI0.SPDR.LONG = SendData;
	
	// 送信完了割込みが通知されるまで待つ
	while( ICU.IR[ IR_RSPI0_SPRI0 ].BYTE != 1 )
	{
		nop( );
	}
	
	// 受信データを返す
	ICU.IR[ IR_RSPI0_SPRI0 ].BYTE = 0;
	return RSPI0.SPDR.LONG;
}
	

//=============================================================================
// マルチバイト送信
// ※SendSizeは4で割り切れるサイズにすること
//=============================================================================
#if DISK_USE_WRITE
static bool xmit_spi_multi( const BYTE *pSendBuff, UINT SendSize )
{
	const DWORD 			*lp = (const DWORD *)pSendBuff;
	
	// RSPIデータ長設定ビットを32bitに設定
	RSPI0.SPCMD0.BIT.SPB = 3;
	
	do
	{
		// 割込みステータスフラグを割込みなしにする
		SPRI = 0;
		
		// 送信データをセット
		RSPI0.SPDR.LONG = LDDW(*lp);
		lp++;
		
		// 送信完了割込みが通知されるまで待つ
		while( !SPRI ); 	
		
		// 受信データを破棄
		RSPI0.SPDR.LONG;

	} while( SendSize -= 4 );

	// RSPIデータ長設定ビットを8bitに設定
	RSPI0.SPCMD0.BIT.SPB = 7;
	
	return true;
}
#endif	// #if DISK_USE_WRITE
	
//=============================================================================
// マルチバイト受信
// ※RecvSizeは4で割り切れるサイズにすること
//=============================================================================
static bool rcvr_spi_multi( const BYTE *pRecvBuff, UINT RecvSize )
{
	DWORD 					*lp = (DWORD *)pRecvBuff;
	
	// RSPIデータ長設定ビットを32bitに設定
	RSPI0.SPCMD0.BIT.SPB = 3;
	
	do
	{
		// 割込みステータスフラグを割込みなしにする
		SPRI = 0;
		
		// 送信データをセット（ダミー）
		RSPI0.SPDR.LONG = 0xFFFFFFFF;
		
		// 送信完了割込みが通知されるまで待つ
		while( !SPRI ); 	
	
		// 受信データを取得
		*lp = LDDW( RSPI0.SPDR.LONG );
		lp++;
		
	} while( RecvSize -= 4 );
		
	return true;
}

	
//=============================================================================
// SDをSPIモードにする
// ⇒電源ONにより、カードはそれ本来のネイティブな(SPIモードでない)動作モードに入ります。
//   電源電圧が規定の範囲(2.7～3.6V)に達したあと少なくとも1ms待ち、DI,CSをHレベルにしてSCLKを74クロック以上入れるとコマンドを受け付ける準備ができます。
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
// RSPIを使用できるようにする
//=============================================================================
static void rspi_enable( void )
{
	//---------------------
	// RSPIを有効にする
	//---------------------
	SYSTEM.PRCR.WORD = 0xA502;						// 動作モード、消費電力低減機能、ソフトウェアリセット関連レジスタへの書き込み許可：許可
	MSTP_RSPI0 = 0;									// シリアルペリフェラルインタフェース0モジュールストップ設定ビット(SYSTEM.MSTPCRB.BIT.MSTPB17)
													//   0：モジュールストップ状態の解除
													//   1：モジュールストップ状態へ遷移
	SYSTEM.PRCR.WORD = 0xA500;						// 動作モード、消費電力低減機能、ソフトウェアリセット関連レジスタへの書き込み許可：許可
	if (SYSTEM.PRCR.WORD == 0xA500)
	{
		nop( );
	}
	
	//---------------------
	// RSPIの設定
	//---------------------
	RSPI0.SPCR.BYTE = 0x00;							// RSPI機能を停止
	while( RSPI0.SPCR.BYTE != 0x00 );
	
	// ---[ RSPI 端子制御レジスタ（SPPCR）]---	
	RSPI0.SPPCR.BYTE = 0x00;						// RSPIループバックビット(SPLP)  ⇒ 0:通常モード 
													//   0：通常モード
													//   1：ループバックモード（送信データの反転＝受信データ）
													//
													// RSPIループバック2ビット(SPLP2) ⇒ 0:通常モード 
													//   0：通常モード
													//   1：ループバックモード（送信データ＝受信データ）
													//
													// MOSIアイドル固定値ビット(MOIFV) ⇒ 0:MOSIアイドル時のMOSIA端子の出力値はLow
													//   0：MOSIアイドル時のMOSIA端子の出力値はLow
													//   1：MOSIアイドル時のMOSIA端子の出力値はHigh
													//
													// MOSIアイドル値固定許可ビット(MOIFE) ⇒ 0：MOSI出力値は前回転送の最終データ
													//   0：MOSI出力値は前回転送の最終データ
													//   1：MOSI出力値はMOIFVビットの設定値

	// ---[ RSPI ビットレートレジスタ（SPBR）]---
	RSPI0.SPBR = calc_spbr( SCLK_SLOW );			// 400KHz

	// ---[ RSPI データコントロールレジスタ（SPDCR）]---													
	RSPI0.SPDCR.BYTE = 0x20;						// フレーム数設定ビット(SPFC[1:0]) ⇒ 0 0：1フレーム
													//   0 0：1フレーム
													//   0 1：2フレーム
													//   1 0：3フレーム
													//   1 1：4フレーム
													//
													// RSPI受信/送信データ選択ビット(SPRDTD) ⇒ 0：SPDRは受信バッファを読み出す
													//   0：SPDRは受信バッファを読み出す
													//   1：SPDRは送信バッファを読み出す（ただし、送信バッファが空のとき）
													//
													// RSPIロングワードアクセス/ワードアクセス設定ビット(SPLW) ⇒ 1：SPDRレジスタへはロングワードアクセス
													//   0：SPDRレジスタへはワードアクセス , 
													//   1：SPDRレジスタへはロングワードアクセス

	// ---[ RSPI クロック遅延レジスタ（SPCKD） ]---
	RSPI0.SPCKD.BYTE = 0x00;						// RSPCK遅延設定ビット(SCKDL[2:0]) ⇒ 0 0 0：1RSPCK
													//   0 0 0：1RSPCK
													//   0 0 1：2RSPCK
													//   0 1 0：3RSPCK
													//   0 1 1：4RSPCK
													//   1 0 0：5RSPCK
													//   1 0 1：6RSPCK
													//   1 1 0：7RSPCK
													//   1 1 1：8RSPCK	

	// ---[ RSPI スレーブセレクトネゲート遅延レジスタ（SSLND） ]---
	RSPI0.SSLND.BYTE = 0x00;						// SSLネゲート遅延設定ビット(SLNDL[2:0]) ⇒ 0 0 0：1RSPCK
													//   0 0 0：1RSPCK
													//   0 0 1：2RSPCK
													//   0 1 0：3RSPCK
													//   0 1 1：4RSPCK
													//   1 0 0：5RSPCK
													//   1 0 1：6RSPCK
													//   1 1 0：7RSPCK
													//   1 1 1：8RSPCK
													
	// ---[ RSPI 次アクセス遅延レジスタ（SPND） ]---
	RSPI0.SPND.BYTE = 0x00;							// RSPI次アクセス遅延設定ビット(SPNDL[2:0]) ⇒ 0 0 0：1RSPCK＋2PCLK
													//   0 0 0：1RSPCK＋2PCLK
													//   0 0 1：2RSPCK＋2PCLK
													//   0 1 0：3RSPCK＋2PCLK
													//   0 1 1：4RSPCK＋2PCLK
													//   1 0 0：5RSPCK＋2PCLK
													//   1 0 1：6RSPCK＋2PCLK
													//   1 1 0：7RSPCK＋2PCLK
													//   1 1 1：8RSPCK＋2PCLK	
	
	// ---[ RSPI 制御レジスタ2（SPCR2） ]---
	RSPI0.SPCR2.BYTE = 0x00;						// パリティ許可ビット(SPPE)
													//   0：送信データパリティビットを付加しない
													//      受信データのパリティチェックを行わない
													//   1：送信データにパリティビットを付加し、受信データのパリ
													//      ティチェックを行う（SPCR.TXMD=0のとき）
													//      送信データにパリティビットを付加するが、受信データのパリティチェックは行わない（SPCR.TXMD=1のとき）
													//
													// パリティモードビット(SPOE) ⇒ 0：偶数パリティで送受信
													//   0：偶数パリティで送受信
													//   1：奇数パリティで送受信
													//
													// RSPIアイドル割り込み許可ビット(SPIIE) ⇒ 0：アイドル割り込み要求の発生を禁止
													//   0：アイドル割り込み要求の発生を禁止
													//   1：アイドル割り込み要求の発生を許可
													//
													// パリティ自己判断ビット(PTE) ⇒ 0：パリティ回路自己診断機能は無効
													//   0：パリティ回路自己診断機能は無効
													//   1：パリティ回路自己診断機能が有効
													
	// ---[ RSPI シーケンス制御レジスタ（SPSCR）]---												
	RSPI0.SPSCR.BYTE = 0x00;						// RSPIシーケンス長設定ビット(SPSLN) ⇒  0 0 0：　　1
													//   0 0 0：　　1　　　　　　0→0→…
													//   0 0 1：　　2　　　　　　0→1→0→…
													//   0 1 0：　　3　　　　　　0→1→2→0→…
													//   0 1 1：　　4　　　　　　0→1→2→3→0→…
													//   1 0 0：　　5　　　　　　0→1→2→3→4→0→…
													//   1 0 1：　　6　　　　　　0→1→2→3→4→5→0→…
													//   1 1 0：　　7　　　　　　0→1→2→3→4→5→6→0→…
													//   1 1 1：　　8　　　　　　0→1→2→3→4→5→6→7→0→…	
													// ※RSPI コマンドレジスタ0 ～ 7の参照順番を変更
	
													
	// ---[ RSPI コマンドレジスタ0 ～ 7（SPCMD0 ～ SPCMD7）]---
	RSPI0.SPCMD0.WORD = 0x0700;						// RSPCK位相設定ビット(CPHA) ⇒ 0：奇数エッジでデータサンプル、偶数エッジでデータ変化
													//   0：奇数エッジでデータサンプル、偶数エッジでデータ変化
													//   1：奇数エッジでデータ変化、偶数エッジでデータサンプル
													//
													// RSPCK極性設定ビット(CPOL) ⇒ 0：アイドル時のRSPCKがLow
													//   0：アイドル時のRSPCKがLow
													//   1：アイドル時のRSPCKがHigh
													//
													// ビットレート分周設定ビット(BRDV[1:0]) ⇒ 0 0：ベースのビットレートを選択
													//   0 0：ベースのビットレートを選択
													//   0 1：ベースのビットレートの2分周を選択
													//   1 0：ベースのビットレートの4分周を選択
													//   1 1：ベースのビットレートの8分周を選択
													//
													// SSL信号アサート設定ビット(SSLA[2:0]) ⇒ 0 0 0：SSL0
													//   0 0 0：SSL0
													//   0 0 1：SSL1
													//   0 1 0：SSL2
													//   0 1 1：SSL3
													//   1 x x：設定しないでください
													//
													// SSL信号レベル保持ビット(SSLKP)
													//   0：転送終了時に全SSL信号をネゲート
													//   1：転送終了後から次アクセス開始までSSL信号レベルを保持
													//
													// RSPIデータ長設定ビット(SPB[3:0]) ⇒ 0 1 1 1 : 8ビット
													//  0100 ～ 0111 ：8ビット
													//  1 0 0 0      ：9ビット
													//  1 0 0 1      ：10ビット
													//  1 0 1 0      ：11ビット
													//  1 0 1 1      ：12ビット
													//  1 1 0 0      ：13ビット
													//  1 1 0 1      ：14ビット
													//  1 1 1 0      ：15ビット
													//  1 1 1 1      ：16ビット
													//  0 0 0 0      ：20ビット
													//  0 0 0 1      ：24ビット
													//  0010、0011   ：32ビット
													//
													// RSPI LSBファーストビット(LSBF) ⇒ 0：MSBファースト
													//   0：MSBファースト
													//   1：LSBファースト
													// 
													// RSPI次アクセス遅延許可ビット(SPNDEN) ⇒ 0：次アクセス遅延は1RSPCK＋2PCLK
													//   0：次アクセス遅延は1RSPCK＋2PCLK
													//   1：次アクセス遅延はRSPI次アクセス遅延レジスタ（SPND）の設定値
													// 
													// SSLネゲート遅延設定許可ビット(SLNDEN) ⇒ 0：SSLネゲート遅延は1RSPCK
													//   0：SSLネゲート遅延は1RSPCK
													//   1：SSLネゲート遅延はRSPIスレーブセレクトネゲート遅延レジスタ（SSLND）の設定値
													// RSPCK遅延設定許可ビット(SCKDEN) ⇒ 0：RSPCK遅延は1RSPCK
													//   0：RSPCK遅延は1RSPCK
													//   1：RSPCK遅延はRSPIクロック遅延レジスタ（SPCKD）の設定値
													
	
	// ---[RSPI 制御レジスタ（SPCR）]---
	RSPI0.SPCR.BYTE = 0xC9;							// RSPIモード選択ビット(SPMS) ⇒ 1：クロック同期式動作（3線式）
													//   0：SPI動作（4線式）
													//   1：クロック同期式動作（3線式）
													//
													// 通信動作モード選択ビット(TXMD) ⇒ 0：全二重同期式シリアル通信
													//   0：全二重同期式シリアル通信
													//   1：送信動作のみのシリアル通信
													//
													// モードフォルトエラー検出許可ビット(MODFEN) ⇒ 0：モードフォルトエラー検出を禁止
													//   0：モードフォルトエラー検出を禁止
													//   1：モードフォルトエラー検出を許可
													//
													// RSPIマスタ/スレーブモード選択ビット(MSTR) ⇒ 1：マスタモード
													//   0：スレーブモード
													//   1：マスタモード
													//
													// RSPIエラー割り込み許可ビット(SPEIE) ⇒ 0：RSPIエラー割り込み要求の発生を禁止
													//   0：RSPIエラー割り込み要求の発生を禁止
													//   1：RSPIエラー割り込み要求の発生を許可
													//   
													// RSPI送信割り込み許可ビット(SPTIE) ⇒ 0：RSPI送信割り込み要求の発生を禁止
													//   0：RSPI送信割り込み要求の発生を禁止
													//   1：RSPI送信割り込み要求の発生を許可
													//
													// RSPI機能許可ビット(SPE) ⇒ 1：RSPI機能は有効
													//   0：RSPI機能は無効
													//   1：RSPI機能は有効
													//
													// RSPI受信割り込み許可ビット(SPRIE) ⇒ 1：RSPI受信割り込み要求の発生を許可
													//   0：RSPI受信割り込み要求の発生を禁止
													//   1：RSPI受信割り込み要求の発生を許可
	if (RSPI0.SPCR.BYTE == 0xC9)
	{
		nop( );
	}

	//---------------------
	// RSPI関連の端子設定
	//---------------------
//  MPC.PWPR.BYTE = 0x40;							// Unlock MPC
    /* Enable writing of PFSWE bit. */
    MPC.PWPR.BIT.B0WI = 0;
    /* Enable writing to PFS registers. */ 
    MPC.PWPR.BIT.PFSWE = 1;

	
    // Set RSPCKA pin								// CLK[OUT] - PA5
    MPC.PA5PFS.BYTE = 0x0DU;						// RSPIとして使用
    PORTA.PMR.BIT.B5 = 1U;							// 1:周辺機器として使用

    // Set MOSIA pin 								// SMI[IN]  - PA6
    MPC.PA6PFS.BYTE = 0x0DU;						// RSPIとして使用
    PORTA.PMR.BIT.B6 = 1U;							// 1:周辺機器として使用

    // Set MISOA pin 								// SDO[OUT] - PA7
    MPC.PA7PFS.BYTE = 0x0DU;						// RSPIとして使用
    PORTA.PMR.BIT.B7 = 1U;							// 1:周辺機器として使用
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
// RSPI ビットレートレジスタに設定する値を計算する
//=============================================================================
static BYTE calc_spbr( DWORD bps_target )
{
	BYTE					spbr_result = 0;
	DWORD					f = F_PCLK;					// Frequency
	signed long				n = 0;						// n term in equation
	signed long				N = 0;						// N term in equation
	signed long				calc = 0;
	
	
	// 2分周分ない場合
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
// RSPI割込み有効／無効
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
// SPSRレジスタクリア
//=============================================================================
static void rspi_spsr_clear( void )
{
	// オーバーランエラー・モードフォルトエラー・パリティエラー？
	if (RSPI0.SPSR.BYTE & 0x0D)
	{
		// // オーバーランエラー／モードフォルトエラー／パリティエラーフラグをクリア
		RSPI0.SPSR.BYTE = 0xA0;
		
		while( (RSPI0.SPSR.BYTE & 0x0D) != 0x00 );
	}
}	


//=============================================================================
// RSPI IR レジスタクリア
//=============================================================================
static void rspi_ir_clear( void )
{
//	// RSPI機能が有効の場合
//	if (RSPI0.SPCR.BIT.SPE == 1)
//	{
//		// RSPI機能を無効にする
//		RSPI0.SPCR.BIT.SPE = 0;
//		while( RSPI0.SPCR.BIT.SPE != 0 );
//	}
	
	// オーバーランエラー／モードフォルトエラー／パリティエラーフラグをクリア
	RSPI0.SPSR.BYTE = 0xA0;
	while( (RSPI0.SPSR.BYTE & 0x0D) != 0x00 );
	
	// 送信データエンプティをクリア
	ICU.IR[ IR_RSPI0_SPTI0 ].BYTE = 0;
	
	// 受信データフルをクリア
	ICU.IR[ IR_RSPI0_SPRI0 ].BYTE = 0;
	if (ICU.IR[ IR_RSPI0_SPRI0 ].BYTE == 0)
	{
		nop( );
	}
}
		 
	

