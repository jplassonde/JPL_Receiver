Project:      output/JPL_Receiver
Build Number: 0
Date:         Thu Apr 05 22:24:40 2018
Compiler:     ICCM8C
Version:      7.05.00


Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             lit   0150   005A =     90. bytes (rel,con,rom,lit)

       Addr  Global Symbol
      -----  --------------------------------
       0150  LoadConfigTBL_jpl_receiver_Bank0
       0181  LoadConfigTBL_jpl_receiver_Bank1
       01AA  __lit_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                     psoc_config   01AA   012F =    303. bytes (rel,con,rom,code)

       Addr  Global Symbol
      -----  --------------------------------
       01AA  LoadConfigTBL_jpl_receiver_Ordered
       026F  LoadConfigInit
       026F  _LoadConfigInit
       0282  _LoadConfig_jpl_receiver
       0282  LoadConfig_jpl_receiver
       02D9  __psoc_config_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                     UserModules   02D9   021A =    538. bytes (rel,con,rom,code)

       Addr  Global Symbol
      -----  --------------------------------
       02D9  _SPIM_ISR
       02DA  SPIM_EnableInt
       02DA  _SPIM_EnableInt
       02DE  SPIM_DisableInt
       02DE  _SPIM_DisableInt
       02E2  _SPIM_Start
       02E2  SPIM_Start
       02E7  SPIM_Stop
       02E7  _SPIM_Stop
       02EB  _SPIM_SendTxData
       02EB  SPIM_SendTxData
       02EE  bSPIM_ReadRxData
       02EE  SPIM_bReadRxData
       02EE  _SPIM_bReadRxData
       02EE  _bSPIM_ReadRxData
       02F1  bSPIM_ReadStatus
       02F1  _SPIM_bReadStatus
       02F1  _bSPIM_ReadStatus
       02F1  SPIM_bReadStatus
       02F4  _SleepTimer_ISR
       0308  SleepTimer_EnableInt
       0308  _SleepTimer_EnableInt
       030C  SleepTimer_DisableInt
       030C  _SleepTimer_DisableInt
       0310  SleepTimer_Start
       0310  _SleepTimer_Start
       031B  _SleepTimer_Stop
       031B  SleepTimer_Stop
       031C  _SleepTimer_SetInterval
       031C  SleepTimer_SetInterval
       032E  _SleepTimer_TickWait
       032E  SleepTimer_TickWait
       033D  SleepTimer_SyncWait
       033D  _SleepTimer_SyncWait
       034B  _SleepTimer_bGetTickCntr
       034B  SleepTimer_bGetTickCntr
       034E  _SleepTimer_SetTimer
       034E  SleepTimer_SetTimer
       0351  SleepTimer_bGetTimer
       0351  _SleepTimer_bGetTimer
       0354  LCD_PrCString
       0354  _LCD_PrCString
       0366  LCD_HEX_STR
       0376  _LCD_PrHexByte
       0376  LCD_PrHexByte
       0389  LCD_PrHexInt
       0389  _LCD_PrHexInt
       0390  _LCD_PrString
       0390  LCD_PrString
       039A  LCD_WriteData
       039A  _LCD_WriteData
       039A  LCD_Write_Data
       039A  _LCD_Write_Data
       03AE  _LCD_Control
       03AE  LCD_Control
       0455  LCD_Init
       0455  LCD_Start
       0455  _LCD_Init
       0455  _LCD_Start
       04C1  LCD_ROW_OFFSET
       04C5  _LCD_Position
       04C5  LCD_Position
       04D1  LCD_Delay50uTimes
       04D1  _LCD_Delay50uTimes
       04D7  LCD_Delay50u_Table
       04DF  _LCD_Delay50u
       04DF  LCD_Delay50u
       04F3  __UserModules_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                            text   04F3   029A =    666. bytes (rel,con,rom,code)

       Addr  Global Symbol
      -----  --------------------------------
       04F3  _Delay10msTimes
       04F3  Delay10msTimes
       04F3  __text_start
       0500  _Delay50uTimes
       0500  Delay50uTimes
       0506  Delay50u_Table
       050E  _Delay50u
       050E  Delay50u
       0526  _main
       078A  PSoC_GPIO_ISR
       078D  __text_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                           table   078D   0006 =      6. bytes (rel,con,rom,code)

       Addr  Global Symbol
      -----  --------------------------------
       0793  __table_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
               virtual_registers   0000   0001 =      1. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0000  __r0
       0001  __virtual_registers_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                    InterruptRAM   0001   0008 =      8. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0001  SleepTimer_fTick
       0001  _SleepTimer_fTick
       0002  _SleepTimer_bTimerValue
       0002  SleepTimer_bTimerValue
       0003  SleepTimer_bCountDown
       0003  _SleepTimer_bCountDown
       0004  SleepTimer_TickCount
       0004  _SleepTimer_TickCount
       0005  Port_0_Data_SHADE
       0005  NO_SHADOW
       0005  _NO_SHADOW
       0005  _Port_0_Data_SHADE
       0006  _Port_2_Data_SHADE
       0006  Port_2_Data_SHADE
       0007  _Port_2_DriveMode_0_SHADE
       0007  Port_2_DriveMode_0_SHADE
       0008  Port_2_DriveMode_1_SHADE
       0008  _Port_2_DriveMode_1_SHADE
       0009  __InterruptRAM_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             bss   0009   0041 =     65. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0009  __bss_start
       0009  rxrdy_flag
       004A  __ramareas_end
       004A  __bss_end

Area                               Start  End    Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             TOP   0000   00D7 =    215. bytes (abs,ovr,rom,code)

       Addr  Global Symbol
      -----  --------------------------------
       0068  __Start
       00D4  _bGetPowerSetting
       00D4  bGetPowerSetting

Files Linked      [ module(s) ]

./obj/boot.o	[ boot.asm ]
obj/delay.o	[ delay.asm ]
obj/main.o	[ main.asm ]
<library>	[ spimint.asm, spim.asm, sleeptimerint.asm, sleeptimer.asm, psocgpioint.asm, psocconfigtbl.asm ]
[ psocconfig.asm, lcd.asm, r0.asm ]

Area Definitions

-blit:0x150.0x3fff
-bdata:0.0xff
-rwh:SSCParmBlk:0xf7.0xff

