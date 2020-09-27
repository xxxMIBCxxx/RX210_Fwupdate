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


// 赤色LED(P3-3：CN1-5)
#define RED_LED_GPIO_PIN					( PORT3.PODR.BIT.B3 )
#define RED_LED( on )						( RED_LED_GPIO_PIN = on )
#define GET_RED_LED( )						( RED_LED_GPIO_PIN )

// 緑色LED(P3-1：CN1-7)
#define GERRN_LED_GPIO_PIN					( PORT3.PODR.BIT.B1 )
#define GREEN_LED( on )						( GERRN_LED_GPIO_PIN = on )
#define GET_GREEN_LED( )					( GERRN_LED_GPIO_PIN )


// RX210プログラム書込み領域
#define	PROGRAM_START_ADDR					( 0xFFFFFFFF )		// プログラム書込み開始アドレス
#define	PROGRAM_END_ADDR					( 0xFFF00000 )		// プログラム書込み終了アドレス

#define MOTOROLA_S_TYPE_RECODE_HEADER_SIZE ( 4 )				// モトローラS レコードヘッダーサイズ


//-----------------------------------------------
// 処理結果種別
//-----------------------------------------------
typedef enum
{
	FW_UPDATE_SUCCESS 						= 0,
	FW_UPDATE_ERR_NOT_FW_FILE				= 0xE0001,			// ファームウェアファイルが存在しない
	FW_UPDATE_ERR_FW_FILE					= 0xE0002,			// ファームウェアファイル異常
	FW_UPDATE_ERR_FILE_READ_BUFFER_SIZE		= 0xE0003,			// ファームウェア格納用バッファサイズ異常
	
	FW_UPDATE_ERR_ANALYZE_RECODE_SIZE		= 0xE0002,			// ファームウェア解析エラー（レコードサイズ異常）
	FW_UPDATE_ERR_ANALYZE_NOT_RECODE		= 0xE0003,			// ファームウェア解析エラー（レコードデータ異常）
	FW_UPDATE_ERR_ANALYZE_RECODE_TYPE		= 0xE0004,			// ファームウェア解析エラー（レコードタイプ異常）
	FW_UPDATE_ERR_ANALYZE_PROGRAM_RANGE		= 0xE0005,			// ファームウェア解析エラー（プログラム書込み領域範囲異常）
	FW_UPDATE_ERR_ANALYZE_INPUT_BUFFER_SIZE	= 0xE0006,			// ファームウェア解析エラー（入力データサイズ異常）
	FW_UPDATE_ERR_ANALYZE_CHECK_SUM			= 0xE0007,			// ファームウェア解析エラー（チェック・サムエラー）
	FW_UPDATE_ERR_ANALYZE_RECODE_EOF		= 0xE0008,			// ファームウェア解析エラー（レコード終端異常）

	FW_UPDATE_ERR_SYSTEM					= 0xE9999,			// システム異常
	
	
	
} FW_UPDATE_RESULT;



//-----------------------------------------------
// LED点灯モード種別
//-----------------------------------------------
typedef enum
{
	LED_MODE_START,												// FW開始（緑色LED：点灯 / 赤色LED：点灯）
	LED_MODE_PROCESS,											// 処理中（緑色LED：点滅(500ms) / 赤色LED：消灯）
	LED_MODE_PROCESS_PROGRAM,									// 処理中_書込み中（緑色LED：点滅(200ms) / 赤色LED：消灯）
	LED_MODE_ERROR,												// エラー（緑色LED：消灯 / 赤色LED：点灯）
	LED_MODE_SUCCESS,											// 正常終了（緑色LED：点灯 / 赤色LED：点滅）	
} LED_MODE_ENUM;



//-----------------------------------------------
// モトローラSファイル・レコードタイプ種別
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
// モトローラSファイル情報構造体
//-----------------------------------------------
typedef struct
{
	S_RECODE_TYPE_ENUM			eRecodeType;			// レコードタイプ
	
	DWORD						Address;				// ロード・アドレス(S1：16Bit / S2：24Bit / S3：32Bit)
	BYTE						CodeSize;				// コードサイズ
	BYTE						Code[ 256 ];			// コード(S1 / S2 / S3) or ファイル名(S0) 
	BYTE						CheckSum;				// チェック・サム
} MOTOROLA_S_TYPE_DATA_INFO_TABLE;






//=============================================================================
// プロトタイプ宣言
//=============================================================================
void InitClock( void );																// クロック設定
void InitCMT0( void );																// CMT0初期化（1msタイマーを生成）
void SoftwareDelay( DWORD delay_ms );												// ソフトウェア遅延
void Delay_IntervalTimer( void );													// Delay 1msタイマー処理
void LED_IntervalTimer( void );														// LED 1msタイマー
FW_UPDATE_RESULT Get_Motorola_S_Data( FIL *pFile, BYTE *pData, WORD DataSize );		// モトローラ・Sタイプファイルを1件分取得する
FW_UPDATE_RESULT Analyze_Motorola_S_Data( BYTE *pData, WORD DataSize, MOTOROLA_S_TYPE_DATA_INFO_TABLE *ptStypeInfo );				// モトローラ・Sタイプファイルを解析する





//=============================================================================
// グローバル変数
//=============================================================================
typedef struct 
{
	FW_UPDATE_RESULT									eResult;					// 処理結果
	LED_MODE_ENUM										eLedMode;					// LEDモード
	
	DWORD												DelayTimer;					// 遅延タイマー(ms)
	BYTE												Buffer[ 512 ];				// ファームウェアデータ読み出し用バッファ
	
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
	
	
	// クロック設定
	InitClock( );
	
	// 赤色LED(P3-3：CN1-5) , 緑色LED(P3-1：CN1-7)
	PORT3.PDR.BIT.B3 = 1;
	PORT3.PDR.BIT.B1=  1;
	RED_LED( 1 );
	GREEN_LED( 1 );

	// CMT0初期化（1msタイマーを生成）
	InitCMT0( );
	
	// 1秒待つ
	SoftwareDelay( 1000 );

	// SDカードをマウントする
	FsResult = f_mount( &Fs, "", 1 );
	if (FsResult == FR_OK)
	{
		// FWファイルオープン
		FsResult = f_open( &File, "Fw.mot", FA_READ );
		if (FsResult == FR_OK)
		{
			while( f_eof( &File) != 1 )
			{
				// １件分取得
				memset( g_Grobal.Buffer, 0x00, sizeof(g_Grobal.Buffer) );
				eRet = Get_Motorola_S_Data( &File, g_Grobal.Buffer, sizeof(g_Grobal.Buffer) );
				if( eRet != FW_UPDATE_SUCCESS )
				{
					g_Grobal.eLedMode = LED_MODE_ERROR;			// LEDモード：エラー
					break;
				}
				
				// 解析
				memset( &tStypeInfo, 0x00, sizeof(tStypeInfo) );
				eRet = Analyze_Motorola_S_Data( g_Grobal.Buffer, sizeof(g_Grobal.Buffer), &tStypeInfo );
				if( eRet != FW_UPDATE_SUCCESS )
				{
					g_Grobal.eLedMode = LED_MODE_ERROR;			// LEDモード：エラー
					break;
				}
			}
			
			// 正常
			eRet = FW_UPDATE_SUCCESS;
			g_Grobal.eLedMode = LED_MODE_SUCCESS;			// LEDモード：正常終了

		}
		else
		{
			// ファームウェアファイルが存在しない
			eRet = FW_UPDATE_ERR_NOT_FW_FILE;
			g_Grobal.eLedMode = LED_MODE_ERROR;				// LEDモード：エラー
		}
	}
	else
	{
		// システム異常
		eRet = FW_UPDATE_ERR_SYSTEM;
		g_Grobal.eLedMode = LED_MODE_ERROR;				// LEDモード：エラー
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
// クロック設定
//=============================================================================
void InitClock( void )
{
    // プロテクト解除（クロック発生回路関連 / 動作モード、消費電力低減機能、ソフトウェアリセット関連 / VRCR）
    SYSTEM.PRCR.WORD = 0xA507;
     
//  // 電圧レギュレータ制御レジスタ(システムクロックを変更するため、"00h"を書き込む）
//  SYSTEM.VRCR = 0x00;
     
//  RX210にはメインクロックがないので
//  // メインクロック発振動作開始
//  SYSTEM.MOSCWTCR.BYTE = 0x0D;        		// メインクロック発振安定時間：131072サイクル
//  SYSTEM.MOSCCR.BIT.MOSTP = 0;        		// メインクロック発振動作開始
//  while( SYSTEM.MOSCCR.BIT.MOSTP != 0 );  	// 設定完了待ち
     
    // PLL発振動作開始
    SYSTEM.PLLCR.WORD = 0x0901;     			// 20HMz / 2 * 10 = 100MHz
    SYSTEM.PLLWTCR.BYTE = 0x0D;     			// PLL発振安定時間：1048576サイクル
    SYSTEM.PLLCR2.BIT.PLLEN = 0;        		// PLL発振動作開始
    while( SYSTEM.PLLCR2.BIT.PLLEN != 0 );  	// 設定完了待ち
     
// // 高速動作モードに遷移
//  SYSTEM.OPCCR.BIT.OPCM = 0;      			// 高速動作モード
//  while( SYSTEM.OPCCR.BIT.OPCMTSF != 0 ); 	// 遷移完了まで待つ
     
    // 周辺モジュールクロック(PCLKD)：50MHz(100MHz / 2)
    // 周辺モジュールクロック(PCLKB)：25MHz(100MHz / 4)
    // 外部バスクロック(BCLK)       ：25MHz(100Mhz / 4)
    // システムクロック(ICLK)       ：50MHz(100MHz / 2)
    // FlashIFクロック(FCLK)        ：25MHz(100MHz / 4)
    SYSTEM.SCKCR.LONG = 0x21021212;
    while( SYSTEM.SCKCR.LONG != 0x21021212 ); 	// 設定完了待ち
         
    // PLL回路選択
    SYSTEM.SCKCR3.WORD = 0x0400;        		// PLL回路選択
    while( SYSTEM.SCKCR3.WORD != 0x0400 );  	// 設定完了待ち
     
    // 高速オンチップオシレータ停止
    SYSTEM.HOCOCR.BYTE = 0x01;      			// HOCO停止
    while( SYSTEM.HOCOCR.BYTE != 0x01 );    	// 設定完了待ち
     
    // 低速オンチップオシレータ停止
    SYSTEM.LOCOCR.BYTE = 0x01;      			// LOCO停止
    while( SYSTEM.LOCOCR.BYTE != 0x01 );    	// 設定完了待ち
     
    // サブクロック発振器停止
    SYSTEM.SOSCCR.BYTE = 0x01;      			// サブクロック発振器停止
    while( SYSTEM.SOSCCR.BYTE != 0x01 );    	// 設定完了待ち
    RTC.RCR3.BYTE = 0x0C;           			// RTC[サブクロック発振器停止, 標準CL用ドライブ能力]
    while( RTC.RCR3.BYTE != 0x0C );     		// 設定完了待ち

    // プロテクト設定
    SYSTEM.PRCR.WORD = 0xA500;  
}


//=============================================================================
// CMT0初期化（1msタイマーを生成）
//=============================================================================
void InitCMT0( void )
{
    // プロテクト解除（動作モード、消費電力低減機能、ソフトウェアリセット関連）
    SYSTEM.PRCR.WORD = 0xA502;
     
    // コンペアマッチタイマ(ユニット0)モジュールストップ設定ビット
    MSTP(CMT0) = 0;             	// 状態解除
     
    // CMI0割込み要求許可
    IEN( CMT0, CMI0 ) = 1;
     
    // CMI0割込みの割込み優先レベルを1に設定
    IPR( CMT0, CMI0 ) = 1;
     
    // プロテクト設定
    SYSTEM.PRCR.WORD = 0xA500;  
 
     
    CMT0.CMCR.WORD = 0x00C0;   	 	// コンペアマッチ割込み許可, クロック選択ビット(PCLKB(25MHz) / 8)
    CMT0.CMCOR = 3125 - 1;      	// 1msのカウント
    CMT0.CMCNT = 0;         		// コンペアマッチタイマカウンタ初期化
    CMT.CMSTR0.BIT.STR0 = 1;    	// CMT0.CMCNTカウンタのカウント動作開始
}   


//=============================================================================
// ソフトウェア遅延
//=============================================================================
void SoftwareDelay( DWORD delay_ms )
{
	g_Grobal.DelayTimer = delay_ms;
	
	while ( g_Grobal.DelayTimer );
}


//=============================================================================
// Delay 1msタイマー処理
//=============================================================================
void Delay_IntervalTimer( void )
{
	DWORD		time;
	
	// DelayTimer
	time = g_Grobal.DelayTimer;
	if (time) g_Grobal.DelayTimer = --time;
}
	

//=============================================================================
// LED 1msタイマー処理
//=============================================================================
void LED_IntervalTimer( void )
{
	static volatile LED_MODE_ENUM			ePrevLedMode = LED_MODE_START;						// 前回のLEDモード
	static volatile DWORD					LedTimer = 0;										// 点滅タイマー(ms)
	static volatile DWORD					BackupLedTimer = 0;									// 点滅タイマー(ms)バックアップ用
	
	
	// LEDモードが前回のLEDモードと違う場合
	if (g_Grobal.eLedMode != ePrevLedMode)
	{
		// FW開始（緑色LED：点灯 / 赤色LED：点灯）
		if (g_Grobal.eLedMode == LED_MODE_START)
		{
			GREEN_LED( 1 );
			RED_LED( 1 );
		}
		// 処理中（緑色LED：点滅(500ms) / 赤色LED：消灯）
		else if (g_Grobal.eLedMode == LED_MODE_PROCESS)
		{
			LedTimer = BackupLedTimer = 500;
			GREEN_LED( 1 );
			RED_LED( 0 );
		}
		// 処理中_書込み中（緑色LED：点滅(200ms) / 赤色LED：消灯）
		else if (g_Grobal.eLedMode == LED_MODE_PROCESS_PROGRAM)
		{
			LedTimer = BackupLedTimer = 200;
			GREEN_LED( 1 );
			RED_LED( 0 );
		}
		// エラー（緑色LED：消灯 / 赤色LED：点灯）
		else if (g_Grobal.eLedMode == LED_MODE_ERROR)
		{
			GREEN_LED( 0 );
			RED_LED( 1 );
		}
		// 正常終了（緑色LED：点灯 / 赤色LED：点滅）
		else if (g_Grobal.eLedMode == LED_MODE_SUCCESS)		
		{
			GREEN_LED( 1 );
			RED_LED( 0 );
		}
		
		// 前回のLEDモードを変更
		ePrevLedMode = g_Grobal.eLedMode;		
	}
	else
	{
		// 処理中 or 処理中_書込み中（両方とも緑色LED点滅なので…）
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
// モトローラ・Sタイプファイルを1件分取得する
//=============================================================================
FW_UPDATE_RESULT Get_Motorola_S_Data( FIL *pFile, BYTE *pData, WORD DataSize )
{
	FRESULT			eFsResult = FR_OK;
	BYTE			Buff[ 2 ] = { 0x00, 0x00 };
	UINT			ByteRead = 0;
	WORD			i = 0;
	bool			bCr = false;
	
	
	// データがなくなるまでループ
	while ( !f_eof( pFile ) )
	{
		// 1文字読み込む
		eFsResult = f_read( pFile, Buff, 1, &ByteRead );
		if (eFsResult != FR_OK)
		{
			// ファームウェアファイル異常
			return FW_UPDATE_ERR_FW_FILE;
		}
		
		// 取得データ格納
		pData[ i ] = Buff[ 0 ];
		
		// 区切りチェック
		if (Buff[ 0 ] == 0x0D)
		{
			bCr = true;
		}
		else if ( (bCr == true) && (Buff[ 0 ] == 0x0A) )
		{
			// １件分のデータを見つけた
			break;
		}
		else
		{
			bCr = false;
		}
		
		// 格納サイズチェック
		i++;
		if ( DataSize <= i )
		{
			// 1件分のデータを格納できないため、エラーとする
			// ファームウェア格納用バッファサイズ異常
			return FW_UPDATE_ERR_FILE_READ_BUFFER_SIZE;
		}
	}
		
	return FW_UPDATE_SUCCESS;
}


//=============================================================================
// モトローラ・Sタイプファイルを解析する
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
	
	
	// サイズチェック（とりあえずデータ長まである？）
	if ( DataSize < MOTOROLA_S_TYPE_RECODE_HEADER_SIZE )
	{
		// ファームウェア解析エラー（レコードサイズ異常）
		return FW_UPDATE_ERR_ANALYZE_RECODE_SIZE;
	}
	
	// レコードの最初は'S'
	if ( (pData[ 0 ] != 'S') && (pData[ 0 ] != 's') )
	{
		// ファームウェア解析エラー（レコードデータ異常）
		return FW_UPDATE_ERR_ANALYZE_NOT_RECODE;
	}
	
	// タイプを取得
	switch ( pData[ 1 ] ) {
	case '0':
		ptStypeInfo->eRecodeType = S0_RECODE_TYPE;
		AddrEndPos = 4;			// "0000"固定だけど…
		break;
	case '1':
		ptStypeInfo->eRecodeType = S1_RECODE_TYPE;
		AddrEndPos = 4;			// ロード・アドレス(16bit)
		break;
	case '2':
		ptStypeInfo->eRecodeType = S2_RECODE_TYPE;
		AddrEndPos = 6;			// ロード・アドレス(24bit)
		break;
	case '3':
		ptStypeInfo->eRecodeType = S3_RECODE_TYPE;
		AddrEndPos = 8;			// ロード・アドレス(32bit)
		break;
	case '7':
		ptStypeInfo->eRecodeType = S7_RECODE_TYPE;
		AddrEndPos = 8;			// エントリ・ポイント・アドレス(32bit)
		break;
	case '8':
		ptStypeInfo->eRecodeType = S8_RECODE_TYPE;
		AddrEndPos = 6;			// エントリ・ポイント・アドレス(24bit)
		break;
	case '9':
		ptStypeInfo->eRecodeType = S9_RECODE_TYPE;
		AddrEndPos = 4;			// エントリ・ポイント・アドレス(16bit)
		break;
	default:
		// ファームウェア解析エラー（レコードタイプ異常）
		return FW_UPDATE_ERR_ANALYZE_RECODE_TYPE;
	}

	// レコード長を取得（チェック・サム対象）
	memset( strHex, 0x00, sizeof(strHex) );
	strHex[ 0 ] = pData[ 2 ];
	strHex[ 1 ] = pData[ 3 ];
	Length = (BYTE)strtoul( (const char *)strHex, NULL, 16 );
	Sum += Length;

	// レコード長が判明したため、データサイズがレコード長が格納できるかをチェックする
	if ( ((Length * 2) + MOTOROLA_S_TYPE_RECODE_HEADER_SIZE) > DataSize )
	{
		// ファームウェア解析エラー（入力データサイズ異常）
		return FW_UPDATE_ERR_ANALYZE_INPUT_BUFFER_SIZE;
	}
	
	// アドレスを取得（チェック・サム対象）
	memset( strHex, 0x00, sizeof(strHex) );
	ptStypeInfo->Address = 0;
	DataNum = 0;
	for ( i = MOTOROLA_S_TYPE_RECODE_HEADER_SIZE ; i < (AddrEndPos + MOTOROLA_S_TYPE_RECODE_HEADER_SIZE) ; i++ )
	{
		strHex[ DataNum ] = pData[ i ];
		DataNum++;
		
		// HEX変換に必要な２文字分を取得した時
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
	
	// プログラム書込み領域範囲チェック（S0レコードタイプ以外)
	if ( ptStypeInfo->eRecodeType != S0_RECODE_TYPE)
	{
		if ( (ptStypeInfo->Address > PROGRAM_START_ADDR)  || (ptStypeInfo->Address < PROGRAM_END_ADDR) )
		{
			// ファームウェア解析エラー（プログラム書込み領域範囲異常）
			return FW_UPDATE_ERR_ANALYZE_PROGRAM_RANGE;
		}
	}
	
	// データ解析（チェック・サム対象）
	// ・変数iの初期値について：アドレス取得で使用したときの状態を初期値とする
	// ・Length - 1：チェック・サムを省くため
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
	
	// チェック・サム分
	strHex[ 0 ] = pData[ i ];
	strHex[ 1 ] = pData[ (i + 1) ];
	Bin = (BYTE)strtoul( (const char *)strHex, NULL, 16 );
	ptStypeInfo->CheckSum = Bin;
	Sum += Bin;

	// チェック・サム
	if ( Sum != 0xFF )
	{
		// ファームウェア解析エラー（チェック・サムエラー）
		return FW_UPDATE_ERR_ANALYZE_CHECK_SUM;
	}
	
	// レコード終端チェック
	if ( (pData[ i + 2 ] != 0x0D) && (pData[ i + 3 ] != 0x0A) )
	{
		// ファームウェア解析エラー（レコード終端異常）
		return FW_UPDATE_ERR_ANALYZE_RECODE_EOF;
	}

	return FW_UPDATE_SUCCESS;
}
