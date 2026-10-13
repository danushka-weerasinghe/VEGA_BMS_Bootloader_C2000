//###########################################################################
//
// FILE:    main.c
//
// TITLE:   BMS CAN Flash Kernel V1.6
//
// Created on: 8th August 2024
//
// Author: Ashan Sandanayake
//
// CPUTimer update for CAN timeout incomplete | up to backup creation complete
//
//###########################################################################

//
// Included Files
//
#include "DSP28x_Project.h"
#include "Flash2806x_API_Library.h"
#include "stdbool.h"
#ifndef NULL
#define NULL 0
#endif

#define LOAD_BACKUP 1
#define RELOAD_BACKUP 2
#define PROG_BUFFER_LENGTH 0x08

typedef enum
{
    FLASH_FLAG,
    JUMP_FLAG,
    BACKUP_FLAG,
    PACKET_FLAG
}flags;

typedef enum
{
    NULL_STATE,
    FULL_STATE
}state;

//STATE FLAGS
#define BOOT_INIT 0xFFFF
#define BOOTLOADER 0xAAAA
#define BOOT_CONFIG 0x5555
#define APPLICATION 0x4444
#define BACKUP 0xBBBB
#define BOOT_ERROR 0xEEEE


//MEMORY ADDRESSES
#define FLASH_FLAG_ADDRESS 0x3E4000
#define JUMP_FLAG_ADDRESS 0x3E4001
#define BACKUP_FLAG_ADDRESS 0x3E4002
#define PACKET_FLAG_ADDRESS 0x3E4003
#define APPLICATION_BEGIN_ADDRESS 0x3F3FF6
#define APPLICATION_ADDRESS_MAIN_FUNCS 0x3EC000
#define APPLICATION_COPY_ADDRESS_MAIN_FUNCS 0x3DC000
#define APPLICATION_ADDRESS_RAM_FUNCS 0x3E8000
#define APPLICATION_COPY_ADDRESS_RAM_FUNCS 0x3D8000
#define FLASH_FLAG_LENGTH 4
#define COPY_BUFFER_LENGTH_MAIN_FUNCS 0X8000
#define COPY_BUFFER_LENGTH_RAM_FUNCS 0X4000

//Can Messages
#define BEGIN_FLASH 0x01010101
#define TERMINATE_FLASH 0x02020202
#define END_FLASH 0x05050505
#define READ_BEGIN 0x11111111
#define READ_END 0x12121212
#define BACKUP_WRITE_START 0x06060606
#define BACKUP_WRITE_ERROR 0x07070707
#define BACKUP_WRITE_END 0x08080808
#define FLASH_ERROR 0x09090909
#define LOAD_COMPLETE 0x10101010

//Message timeouts
#define CAN_TIMEOUT_PERIOD 5000000
#define INITIAL_DELAY 100
#define BEGIN_DELAY 100000L

//
// Globals
//
Uint16 CAN_GetWordData(void);
Uint16 CAN_GetWordStart(void);
Uint16 CAN_GetSize();
Uint16 calcrc1(Uint8 *ptr, int count);
void dataWriteBack(Uint16 *writeBackWord);
void CAN_Init(void);
void init_bootloader(void);
void begin_flash_kernal(void);
void end_flashing(void);
void sendCANMessage(Uint32 can_msg);
bool check_packet_count(void);
void sendENDMessage(void);
void resetBMS();

volatile struct ECAN_REGS ECanaShadow;

Uint32 packet_count = 0;
Uint16 test_crc = 0;
Uint32 CAN_word_count = 0;
Uint16 size_of_update = 0;
typedef Uint16 (*uint16fptr)();
uint16fptr GetWordData;

Uint16 dataBuffer[6];

Uint16 copy_buffer[COPY_BUFFER_LENGTH_MAIN_FUNCS];

Uint32 GetLongData();
void   CopyData(void);
void CopyBackup(Uint8 backup_state);
void ReadReservedFn(void);
//Uint16 read_OTA_flag(void);
//Uint16 get_jump_flag(void);
//Uint16 get_backup_flag(void);
Uint16 read_flag(Uint8 flag_status);
void init_jump_to_BMS(void);
void load_application(void);

//Ramfuncs
#pragma CODE_SECTION(CopyData, "ramfuncs");
#pragma CODE_SECTION(CopyBackup, "ramfuncs");
#pragma CODE_SECTION(resetBMS, "ramfuncs");
#pragma CODE_SECTION(sendCANMessage, "ramfuncs");
#pragma CODE_SECTION(CAN_GetWordData, "ramfuncs");
//#pragma CODE_SECTION(read_OTA_flag, "ramfuncs");
//#pragma CODE_SECTION(get_jump_flag, "ramfuncs");
//#pragma CODE_SECTION(get_backup_flag, "ramfuncs");
#pragma CODE_SECTION(read_flag, "ramfuncs");
#pragma CODE_SECTION(init_jump_to_BMS, "ramfuncs");
#pragma CODE_SECTION(load_application, "ramfuncs");

//
// Flash Status Structure
//
FLASH_ST FlashStatus;

extern Uint32 Flash_CPUScaleFactor;
extern void (*Flash_CallbackPtr) (void);

//
// Main
//
void main(void)
{
    init_bootloader();

    if (read_flag(FLASH_FLAG) == BOOT_INIT || read_flag(FLASH_FLAG) == BOOTLOADER)
    {
        DELAY_US(BEGIN_DELAY);
        begin_flash_kernal();
        CopyData();
        end_flashing();
        resetBMS();
    }
    else if (read_flag(FLASH_FLAG) == BOOT_CONFIG)
    {
        init_jump_to_BMS();
    }
    else if (read_flag(FLASH_FLAG) == BACKUP)
    {
        CopyBackup(RELOAD_BACKUP);
        resetBMS();
    }
    else if (read_flag(FLASH_FLAG) == APPLICATION)
    {
        load_application();
    }
    else if (read_flag(FLASH_FLAG) == BOOT_ERROR)
    {
        CopyBackup(LOAD_BACKUP);
        resetBMS();
    }

    for(;;);
}

void init_bootloader(void)
{
    test_crc = 0;
    CAN_word_count = 0;
    //packet_count = 0;

    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (Uint32) &RamfuncsLoadSize);
    InitFlash();

    DINT; // Disable CPU interrupts
    InitPieCtrl(); //PIE control registers to their default state
    //
    // Disable CPU interrupts and clear all CPU interrupt flags
    //
    IER = 0x0000;
    IFR = 0x0000;

    //
    // Setup sysctl and pll
    //
    DisableDog();
    XtalOscSel();
    InitPll(9,2);
    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer0, 90, CAN_TIMEOUT_PERIOD);

    DELAY_US(INITIAL_DELAY);

    if(SysCtrlRegs.PLLSTS.bit.MCLKSTS == 1){
        for(;;);
    }

    CAN_Init();
}

void begin_flash_kernal(void)
{
    while(CAN_GetWordStart() != 0x3269) {};

    size_of_update = CAN_GetSize();

    ECanaRegs.CANRMP.all = 0xFFFFFFFF;

    GetWordData = CAN_GetWordData;

    sendCANMessage(TERMINATE_FLASH);

    sendCANMessage(BEGIN_FLASH);

    if (CAN_GetWordData() != 0x08AA) ((void(*)())APPLICATION_BEGIN_ADDRESS)();

    ReadReservedFn();

    Uint32 EntryAddr = GetLongData();
}

void CAN_Init()
{
   struct ECAN_REGS ECanaShadow;

   EALLOW;

   SysCtrlRegs.PCLKCR0.bit.ECANAENCLK=1;    // Enable CAN clock

   GpioCtrlRegs.GPAMUX2.all |= 0x50000000;  // Configure eCAN-A pins using GPIO regs

   GpioCtrlRegs.GPAPUD.all &= 0x3FFFFFFF;   //Enable internal pullups for the CAN pins

   GpioCtrlRegs.GPAQSEL2.bit.GPIO30 = 3;    // Asynch Qual

   //
   // Configure eCAN RX and TX pins for CAN operation using eCAN regs
   //
    ECanaShadow.CANTIOC.all = ECanaRegs.CANTIOC.all;
    ECanaShadow.CANTIOC.bit.TXFUNC = 1;
    ECanaRegs.CANTIOC.all = ECanaShadow.CANTIOC.all;

    ECanaShadow.CANRIOC.all = ECanaRegs.CANRIOC.all;
    ECanaShadow.CANRIOC.bit.RXFUNC = 1;
    ECanaRegs.CANRIOC.all = ECanaShadow.CANRIOC.all;

    //
    // all bits (including reserved bits) of MSGCTRL to zero
    //
    ECanaMboxes.MBOX1.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX2.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX3.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX4.MSGCTRL.all = 0x00000000;
    ECanaMboxes.MBOX5.MSGCTRL.all = 0x00000000;

    ECanaRegs.CANRMP.all = 0xFFFFFFFF;  // Clear all RMPn, GIFn bits

    //
    // Clear all interrupt flag bits
    //
    ECanaRegs.CANGIF0.all = 0xFFFFFFFF;
    ECanaRegs.CANGIF1.all = 0xFFFFFFFF;

    //
    // Configure bit timing parameters for eCANA
    //
    ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
    ECanaShadow.CANMC.bit.CCR = 1 ;
    ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;

    ECanaShadow.CANES.all = ECanaRegs.CANES.all;

    //
    // Wait for CCE bit to be set..
    //
    do
    {
        ECanaShadow.CANES.all = ECanaRegs.CANES.all;
    } while(ECanaShadow.CANES.bit.CCE != 1 );

    ECanaShadow.CANBTC.all = 0;

    ECanaShadow.CANBTC.bit.BRPREG = 9; //9 = 250kbps, 4 = 500kbps
    ECanaShadow.CANBTC.bit.TSEG2REG = 2;
    ECanaShadow.CANBTC.bit.TSEG1REG = 13;

    ECanaShadow.CANBTC.bit.SAM = 1;

    ECanaRegs.CANBTC.all = ECanaShadow.CANBTC.all;

    ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
    ECanaShadow.CANMC.bit.CCR = 0 ;
    ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;

    ECanaShadow.CANES.all = ECanaRegs.CANES.all;

    //
    // Wait for CCE bit to be  cleared..
    //
    do
    {
       ECanaShadow.CANES.all = ECanaRegs.CANES.all;
    } while(ECanaShadow.CANES.bit.CCE != 0 );

    ECanaRegs.CANME.all = 0;    // Disable all Mailboxes

    ECanaMboxes.MBOX1.MSGID.all = 0x80047B84;   // Assign MSGID to MBOX1 (Extended ID of 1, Acceptance mask disabled)

    ECanaMboxes.MBOX2.MSGID.all = 0x80067B84;   // Assign MSGID to MBOX2 (Extended ID of 2, Acceptance mask disabled)

    ECanaMboxes.MBOX3.MSGID.all = 0x80027B84;   // Assign MSGID to MBOX3 (Extended ID of 3, Acceptance mask disabled)

    ECanaMboxes.MBOX4.MSGID.all = 0x80057B84;   // Assign MSGID to MBOX4 (Extended ID of 4, Acceptance mask disabled)

    ECanaMboxes.MBOX5.MSGID.all = 0x80037B84;   // Assign MSGID to MBOX4 (Extended ID of 4, Acceptance mask disabled)

    ECanaRegs.CANMD.bit.MD1 = 1;    // Configure MBOX1 to be a receive MBOX

    ECanaRegs.CANMD.bit.MD2 = 0;    // Configure MBOX2 to be a transmit MBOX

    ECanaRegs.CANMD.bit.MD3 = 1;    // Configure MBOX3 to be a receive MBOX

    ECanaRegs.CANMD.bit.MD4 = 0;    // Configure MBOX4 to be a transmit MBOX

    ECanaRegs.CANMD.bit.MD5 = 1;    // Configure MBOX3 to be a receive MBOX

    ECanaRegs.CANME.bit.ME1 = 1;    // Enable MBOX1

    ECanaRegs.CANME.bit.ME2 = 1;    // Enable MBOX2

    ECanaRegs.CANME.bit.ME3 = 1;    // Enable MBOX3

    ECanaRegs.CANME.bit.ME4 = 1;    // Enable MBOX4

    ECanaRegs.CANME.bit.ME5 = 1;    // Enable MBOX4

    EDIS;

    return;
}

Uint16 CAN_GetWordStart()
{
   Uint16 startWord = 0;
   Uint16 startByte = 0;

   while(ECanaRegs.CANRMP.bit.RMP3 == 0 && CAN_word_count == 0) {};

   startWord = (Uint16)ECanaMboxes.MBOX3.MDL.byte.BYTE0;
   startByte = (Uint16)ECanaMboxes.MBOX3.MDL.byte.BYTE1;

   startWord |= (startByte << 8);

   return startWord;
}

Uint16 CAN_GetSize()
{
   Uint16 sizeWord = 0;
   Uint16 sizeByte = 0;

   while(ECanaRegs.CANRMP.bit.RMP5 == 0) {};

   sizeWord = (Uint16)ECanaMboxes.MBOX5.MDL.byte.BYTE0;
   sizeByte = (Uint16)ECanaMboxes.MBOX5.MDL.byte.BYTE1;

   sizeWord |= (sizeByte << 8);

   return sizeWord;
}

Uint16 CAN_GetWordData()
{
   Uint16 recieve_word;
   Uint16 word_num;
   Uint8 recieved_packet_num;
   Uint16 byteData[4];
   Uint16 wordData[4];
   Uint8 crc_data[6];

   recieve_word = 0x0000;

   if (CAN_word_count == 0)
   {
       sendCANMessage(READ_BEGIN);
       recieved_packet_num = 0;

       while(recieved_packet_num < 2)
       {
           CpuTimer0Regs.TCR.bit.TRB = 1;  // Reload the timer
           CpuTimer0Regs.TCR.bit.TSS = 0;  // Start the timer

           while(ECanaRegs.CANRMP.bit.RMP1 == 0) {

               if (CpuTimer0Regs.TCR.bit.TIF == 1)
               {
                   CpuTimer0Regs.TCR.bit.TIF = 1;  // Clear the interrupt flag
                   resetBMS();  // Timer has reached the period reset bms
               }
           }
           CpuTimer0Regs.TCR.bit.TSS = 1;  // Stop the timer

           for(word_num =0; word_num<4; word_num++)
           {
               byteData[word_num] = 0x0000;
               wordData[word_num] = 0x0000;
           }
           for(word_num =0; word_num<6; word_num++)
           {
               crc_data[word_num] = 0x0000;
           }
           // Fetch the LSBs
           wordData[0] =  (Uint16)ECanaMboxes.MBOX1.MDL.byte.BYTE0;   // LS byte1
           wordData[1] =  (Uint16)ECanaMboxes.MBOX1.MDL.byte.BYTE2;   // LS byte2
           wordData[2] =  (Uint16)ECanaMboxes.MBOX1.MDH.byte.BYTE4;   // LS byte3
           wordData[3] =  (Uint16)ECanaMboxes.MBOX1.MDH.byte.BYTE6;   // LS byte4

           // Fetch the MSBs
           byteData[0] =  (Uint16)ECanaMboxes.MBOX1.MDL.byte.BYTE1;    // MS byte1
           byteData[1] =  (Uint16)ECanaMboxes.MBOX1.MDL.byte.BYTE3;    // MS byte2
           byteData[2] =  (Uint16)ECanaMboxes.MBOX1.MDH.byte.BYTE5;    // MS byte3
           byteData[3] =  (Uint16)ECanaMboxes.MBOX1.MDH.byte.BYTE7;    // MS byte4

           crc_data[0] = wordData[0] & 0xFF;
           crc_data[1] = byteData[0] & 0xFF;
           crc_data[2] = wordData[1] & 0xFF;
           crc_data[3] = byteData[1] & 0xFF;
           crc_data[4] = wordData[2] & 0xFF;
           crc_data[5] = byteData[2] & 0xFF;

           // form the wordData from the MSB:LSB
           for(word_num =0; word_num<4; word_num++)
           {
               wordData[word_num] |= (byteData[word_num] << 8);
           }
           test_crc = calcrc1(crc_data, 6);

           if (calcrc1(crc_data, 6) == wordData[3])
           {
               packet_count++;
               recieved_packet_num++;
               if(recieved_packet_num == 1)
               {
                   ECanaRegs.CANRMP.bit.RMP1 = 1;
                   dataBuffer[0] = wordData[0];
                   dataBuffer[1] = wordData[1];
                   dataBuffer[2] = wordData[2];
               }
               else
               {
                   dataBuffer[3] = wordData[0];
                   dataBuffer[4] = wordData[1];
                   dataBuffer[5] = wordData[2];
               }
           }
           else{
               sendCANMessage(TERMINATE_FLASH);
               sendCANMessage(calcrc1(crc_data, 6));
               resetBMS();
           }
       }
       CAN_word_count++;
       recieve_word = dataBuffer[0];
   }
   else if(CAN_word_count == 1)
   {
       CAN_word_count++;
       recieve_word = dataBuffer[1];
   }
   else if(CAN_word_count == 2)
   {
       CAN_word_count++;
       recieve_word = dataBuffer[2];
   }
   else if(CAN_word_count == 3)
   {
       CAN_word_count++;
       recieve_word = dataBuffer[3];
   }
   else if(CAN_word_count == 4)
   {
       CAN_word_count++;
       recieve_word = dataBuffer[4];
   }
   else if(CAN_word_count == 5)
   {
       sendCANMessage(READ_END);
       CAN_word_count = 0;
       ECanaRegs.CANRMP.bit.RMP1 = 1;   //Clear all RMPn bits
       recieve_word = dataBuffer[5];
   }

   return recieve_word;
}

Uint16 calcrc1(Uint8 *ptr, int count) {

    Uint16  crc;
    int i;
    crc = 0;
    while (--count >= 0)
    {
      crc = crc ^ (Uint16) * ptr++ << 8;
      i = 8;
      do
      {
        if (crc & 0x8000)
          crc = (crc << 1) ^ 0x1021;
        else
          crc = crc << 1;
      } while (--i);
    }
    return (crc);
}

Uint32 indication;

void sendCANMessage(Uint32 can_msg) {
    EALLOW; // Enable write access to protected registers

    // Configure mailbox 0 for transmission
    ECanaMboxes.MBOX2.MSGID.all = 0x80067B84; // Set message ID
    ECanaMboxes.MBOX2.MSGCTRL.bit.DLC = 8;    // Set data length code (8 bytes)
    ECanaMboxes.MBOX2.MDL.all = can_msg;   // Set data (lower 4 bytes)
    ECanaMboxes.MBOX2.MDH.all = can_msg;   // Set data (upper 4 bytes)

    // Request transmission
    ECanaRegs.CANTRS.bit.TRS2 = 1; // Set TRS for mailbox 2

    // Wait for transmission to complete
    while (ECanaRegs.CANTA.bit.TA2 != 1) {}

    // Clear the transmission flag
    ECanaRegs.CANTA.bit.TA2 = 0x00000001;

    EDIS; // Disable write access to protected registers
    indication++; //place the break point here
}

void sendENDMessage(void) {
    EALLOW; // Enable write access to protected registers

    static Uint16 tempVar = 0;

    if(check_packet_count())
    {
        tempVar = 0xAB;
    }
    else
    {
        tempVar = 0xFF;
    }

    // Configure mailbox 0 for transmission
    ECanaMboxes.MBOX4.MSGID.all = 0x80057B84; // Set message ID
    ECanaMboxes.MBOX4.MSGCTRL.bit.DLC = 8;    // Set data length code (8 bytes)
    ECanaMboxes.MBOX4.MDL.byte.BYTE0 = packet_count*6 & 0xFF;   // Set data (lower 4 bytes)
    ECanaMboxes.MBOX4.MDL.byte.BYTE1 = (packet_count*6 >> 8) & 0xFF;
    ECanaMboxes.MBOX4.MDL.byte.BYTE2 = (size_of_update) & 0xFF;
    ECanaMboxes.MBOX4.MDL.byte.BYTE3 = ((size_of_update) >> 8) & 0xFF;
    ECanaMboxes.MBOX4.MDH.byte.BYTE4 = tempVar;   // Set data (upper 4 bytes)
    ECanaMboxes.MBOX4.MDH.byte.BYTE5 = check_packet_count();
    ECanaMboxes.MBOX4.MDH.byte.BYTE6 = read_flag(PACKET_FLAG) & 0xFF;
    ECanaMboxes.MBOX4.MDH.byte.BYTE7 = (read_flag(PACKET_FLAG) >> 8) & 0xFF;

    // Request transmission
    ECanaRegs.CANTRS.bit.TRS4 = 1; // Set TRS for mailbox 2

    // Wait for transmission to complete
    while (ECanaRegs.CANTA.bit.TA4 != 1) {}

    // Clear the transmission flag
    ECanaRegs.CANTA.bit.TA4 = 0x00000001;

    EDIS; // Disable write access to protected registers
    indication++; //place the break point here
}

void dataWriteBack(Uint16 *writeBackWord){
    EALLOW; // Enable write access to protected registers

    // Configure mailbox 0 for transmission
    ECanaMboxes.MBOX4.MSGID.all = 0x80057B84; // Set message ID
    ECanaMboxes.MBOX4.MSGCTRL.bit.DLC = 8;    // Set data length code (8 bytes)

    ECanaMboxes.MBOX4.MDL.byte.BYTE0 = (writeBackWord[2]) & 0xFF;
    ECanaMboxes.MBOX4.MDL.byte.BYTE1 = (writeBackWord[2]) >> 8;
    ECanaMboxes.MBOX4.MDL.byte.BYTE2 = (writeBackWord[3]) & 0xFF;
    ECanaMboxes.MBOX4.MDL.byte.BYTE3 = (writeBackWord[3]) >> 8;
    ECanaMboxes.MBOX4.MDH.byte.BYTE4 = (writeBackWord[4]) & 0xFF;
    ECanaMboxes.MBOX4.MDH.byte.BYTE5 = (writeBackWord[4]) >> 8;
    ECanaMboxes.MBOX4.MDH.byte.BYTE6 = (writeBackWord[5]) & 0xFF;
    ECanaMboxes.MBOX4.MDH.byte.BYTE7 = (writeBackWord[5]) >> 8;

    // Request transmission
    ECanaRegs.CANTRS.bit.TRS4 = 1; // Set TRS for mailbox 2

    // Wait for transmission to complete
    while (ECanaRegs.CANTA.bit.TA4 != 1) {}

    // Clear the transmission flag
    ECanaRegs.CANTA.bit.TA4 = 0x00000001;

    EDIS; // Disable write access to protected registers
    indication++; //place the break point here
}

void
CopyData(void)
{
    Uint16 progBuf[PROG_BUFFER_LENGTH]; // Programming Buffer
    Uint16 flag_buff[FLASH_FLAG_LENGTH];
    flag_buff[FLASH_FLAG] = BOOT_CONFIG; //OTA flag
    flag_buff[JUMP_FLAG] = NULL_STATE; //jump flag

    if(read_flag(FLASH_FLAG) == BOOT_INIT)
    {
        flag_buff[BACKUP_FLAG] = BOOT_INIT; // backup flag
        flag_buff[PACKET_FLAG] = NULL_STATE; //packet count flag
    }
    else
    {
        flag_buff[BACKUP_FLAG] = read_flag(BACKUP_FLAG); //backup flag
        flag_buff[PACKET_FLAG] = read_flag(PACKET_FLAG); //packet count flag
    }

    struct HEADER
    {
        Uint16 BlockSize;
        Uint32 DestAddr;
        Uint32 ProgBuffAddr;
    } BlockHeader;

    Uint16 wordData;
    Uint16 status;
    Uint16 i,j;
    Uint16 fail = 0;

    CsmUnlock();    //code security is disabled

    EALLOW;
    Flash_CPUScaleFactor = SCALE_FACTOR;
    Flash_CallbackPtr = NULL;
    EDIS;

    BlockHeader.BlockSize = (*GetWordData)();   // Get the size in words of the first block

//    sendCANMessage(0, BlockHeader.BlockSize);

    status = Flash_Erase((SECTORB | SECTORC | SECTORD),
                         &FlashStatus);
    if(status != STATUS_SUCCESS)
    {
        sendCANMessage(TERMINATE_FLASH);
        resetBMS();
    }

    //
    // While the block size is > 0 copy the data to the DestAddr
    //
    while(BlockHeader.BlockSize != (Uint16)0x0000)
    {
        if(BlockHeader.BlockSize > PROG_BUFFER_LENGTH)
        {
            BlockHeader.DestAddr = GetLongData();   // Block is to big to fit into our buffer so we must program it in chunks

            //
            // Program full buffers
            //
            for(j = 0; j < (BlockHeader.BlockSize / PROG_BUFFER_LENGTH); j++)
            {
                BlockHeader.ProgBuffAddr = (Uint32)progBuf;
                for(i = 1; i <= PROG_BUFFER_LENGTH; i++)
                {
                    wordData = (*GetWordData)();
                    *(Uint16 *)BlockHeader.ProgBuffAddr++ = wordData;
                }

                status = Flash_Program((Uint16 *) BlockHeader.DestAddr,
                         (Uint16 *)progBuf, PROG_BUFFER_LENGTH, &FlashStatus);

                if(status != STATUS_SUCCESS)
                {

                    sendCANMessage(TERMINATE_FLASH);
                    resetBMS();
                }

                BlockHeader.DestAddr += PROG_BUFFER_LENGTH;
            }


            //
            // Program the leftovers
            //
            BlockHeader.ProgBuffAddr = (Uint32)progBuf;
            for(i = 1; i <= (BlockHeader.BlockSize % PROG_BUFFER_LENGTH); i++)
            {
                wordData = (*GetWordData)();
                *(Uint16 *)BlockHeader.ProgBuffAddr++ = wordData;
            }

            status = Flash_Program((Uint16 *) BlockHeader.DestAddr,
                     (Uint16 *)progBuf, (BlockHeader.BlockSize %
                     PROG_BUFFER_LENGTH), &FlashStatus);

            if(status != STATUS_SUCCESS)
            {
                sendCANMessage(TERMINATE_FLASH);
                resetBMS();
            }

        }

        else
        {
            BlockHeader.DestAddr = GetLongData();
            BlockHeader.ProgBuffAddr = (Uint32)progBuf;

            for(i = 1; i <= BlockHeader.BlockSize; i++)
            {
                wordData = (*GetWordData)();
                *(Uint16 *)BlockHeader.ProgBuffAddr++ = wordData;
            }

            status = Flash_Program((Uint16 *) BlockHeader.DestAddr,
                     (Uint16 *)progBuf, BlockHeader.BlockSize, &FlashStatus);
            if(status != STATUS_SUCCESS)
            {
                sendCANMessage(TERMINATE_FLASH);
                resetBMS();
            }

        }

        BlockHeader.BlockSize = (*GetWordData)();   // Get the size of the next block
    }

    status = Flash_Erase(SECTORE,
                         &FlashStatus);
    if(status != STATUS_SUCCESS)
    {
        sendCANMessage(TERMINATE_FLASH);
        fail++;
        return;
    }

    status = Flash_Program((Uint16 *) FLASH_FLAG_ADDRESS,
             (Uint16 *)flag_buff, FLASH_FLAG_LENGTH, &FlashStatus);
    if(status != STATUS_SUCCESS)
    {
        sendCANMessage(TERMINATE_FLASH);
        fail++;
        return ;
    }

    sendCANMessage(TERMINATE_FLASH);

    sendCANMessage(END_FLASH);

//    sendENDMessage();

    return;
}

//
// GetLongData - This routine fetches a 32-bit value from the peripheral input stream.
//
Uint32
GetLongData(void)
{
    Uint32 longData;

    longData = ( (Uint32)(*GetWordData)() << 16);   // Fetch the upper 1/2 of the 32-bit value

    longData |= (Uint32)(*GetWordData)();   // Fetch the lower 1/2 of the 32-bit value

    return longData;
}

//
// Read_ReservedFn1 - This function reads 8 reserved words in the header.
//
void
ReadReservedFn(void)
{
    Uint16 i;

    for(i = 1; i <= 8; i++)
    {
        GetWordData();   // Read and discard the 8 reserved words.
    }
    return;
}

//
// InitPieCtrl - This function initializes the PIE control registers to a known state.
//
void
InitPieCtrl(void)
{

    DINT;   // Disable Interrupts at the CPU level

    //
    // Disable the PIE
    //
    PieCtrlRegs.PIECTRL.bit.ENPIE = 0;

    //
    // Clear all PIEIER registers
    //
    PieCtrlRegs.PIEIER1.all = 0;
    PieCtrlRegs.PIEIER2.all = 0;
    PieCtrlRegs.PIEIER3.all = 0;
    PieCtrlRegs.PIEIER4.all = 0;
    PieCtrlRegs.PIEIER5.all = 0;
    PieCtrlRegs.PIEIER6.all = 0;
    PieCtrlRegs.PIEIER7.all = 0;
    PieCtrlRegs.PIEIER8.all = 0;
    PieCtrlRegs.PIEIER9.all = 0;
    PieCtrlRegs.PIEIER10.all = 0;
    PieCtrlRegs.PIEIER11.all = 0;
    PieCtrlRegs.PIEIER12.all = 0;

    //
    // Clear all PIEIFR registers
    //
    PieCtrlRegs.PIEIFR1.all = 0;
    PieCtrlRegs.PIEIFR2.all = 0;
    PieCtrlRegs.PIEIFR3.all = 0;
    PieCtrlRegs.PIEIFR4.all = 0;
    PieCtrlRegs.PIEIFR5.all = 0;
    PieCtrlRegs.PIEIFR6.all = 0;
    PieCtrlRegs.PIEIFR7.all = 0;
    PieCtrlRegs.PIEIFR8.all = 0;
    PieCtrlRegs.PIEIFR9.all = 0;
    PieCtrlRegs.PIEIFR10.all = 0;
    PieCtrlRegs.PIEIFR11.all = 0;
    PieCtrlRegs.PIEIFR12.all = 0;
}

Uint16 read_flag(Uint8 flag_status)
{
    Uint16 flag_val;
    if(flag_status == FLASH_FLAG)
    {
        flag_val = *(Uint16 *)FLASH_FLAG_ADDRESS;
    }
    else if(flag_status == JUMP_FLAG)
    {
        flag_val = *(Uint16 *)JUMP_FLAG_ADDRESS;
    }
    else if(flag_status == BACKUP_FLAG)
    {
        flag_val = *(Uint16 *)BACKUP_FLAG_ADDRESS;
    }
    else if(flag_status == PACKET_FLAG)
    {
        flag_val = *(Uint16 *)PACKET_FLAG_ADDRESS;
    }

    return flag_val;
}

void CopyBackup(Uint8 backup_state)
{
    Uint16 fail = 0;
    Uint16 status;
    Uint16 i;
    Uint16 *flashCopyPtrMainfuncs;
    Uint16 *flashCopyPtrRamfuncs;

    if(backup_state == LOAD_BACKUP)
    {
        flashCopyPtrMainfuncs = (Uint16 *)APPLICATION_COPY_ADDRESS_MAIN_FUNCS;
        flashCopyPtrRamfuncs = (Uint16 *)APPLICATION_COPY_ADDRESS_RAM_FUNCS;
    }
    else if(backup_state == RELOAD_BACKUP)
    {
        flashCopyPtrMainfuncs = (Uint16 *)APPLICATION_ADDRESS_MAIN_FUNCS;
        flashCopyPtrRamfuncs = (Uint16 *)APPLICATION_ADDRESS_RAM_FUNCS;
    }
    Uint16 flag_buff[FLASH_FLAG_LENGTH];

    DINT;
    InitPieCtrl();
    //
    // Disable CPU interrupts and clear all CPU interrupt flags
    //
    IER = 0x0000;
    IFR = 0x0000;

    CsmUnlock();

    EALLOW;
    Flash_CPUScaleFactor = SCALE_FACTOR;
    Flash_CallbackPtr = NULL;
    EDIS;

    sendCANMessage(BACKUP_WRITE_START);

    if(backup_state == LOAD_BACKUP)
    {
        status = Flash_Erase((SECTORB | SECTORC | SECTORD),
                             &FlashStatus);
    }
    else if(backup_state == RELOAD_BACKUP)
    {
        status = Flash_Erase((SECTORH | SECTORG | SECTORF),
                             &FlashStatus);
    }

    if(status != STATUS_SUCCESS)
    {
        sendCANMessage(BACKUP_WRITE_ERROR);
        fail++;
        return;
    }

    sendCANMessage(BACKUP_WRITE_END);

    for(i = 0; i < COPY_BUFFER_LENGTH_MAIN_FUNCS; i++){
            copy_buffer[i] = *(flashCopyPtrMainfuncs + i);
    }

    sendCANMessage(BACKUP_WRITE_START);

    if(backup_state == LOAD_BACKUP)
    {
        status = Flash_Program((Uint16 *) APPLICATION_ADDRESS_MAIN_FUNCS,
                     (Uint16 *)copy_buffer, COPY_BUFFER_LENGTH_MAIN_FUNCS, &FlashStatus);
    }
    else if(backup_state == RELOAD_BACKUP)
    {
        status = Flash_Program((Uint16 *) APPLICATION_COPY_ADDRESS_MAIN_FUNCS,
                     (Uint16 *)copy_buffer, COPY_BUFFER_LENGTH_MAIN_FUNCS, &FlashStatus);
    }

    if(status != STATUS_SUCCESS)
        {
            sendCANMessage(BACKUP_WRITE_ERROR);
            fail++;
            return ;
        }

    sendCANMessage(BACKUP_WRITE_END);

    for(i = 0; i < COPY_BUFFER_LENGTH_MAIN_FUNCS; i++){
           copy_buffer[i] = 0;
    }
//
    for(i = 0; i < COPY_BUFFER_LENGTH_RAM_FUNCS; i++){
            copy_buffer[i] = *(flashCopyPtrRamfuncs + i);
    }

    sendCANMessage(BACKUP_WRITE_START);

    if(backup_state == LOAD_BACKUP)
    {
        status = Flash_Program((Uint16 *) APPLICATION_ADDRESS_RAM_FUNCS,
                     (Uint16 *)copy_buffer, COPY_BUFFER_LENGTH_RAM_FUNCS, &FlashStatus);
    }
    else if(backup_state == RELOAD_BACKUP)
    {
        status = Flash_Program((Uint16 *) APPLICATION_COPY_ADDRESS_RAM_FUNCS,
                         (Uint16 *)copy_buffer, COPY_BUFFER_LENGTH_RAM_FUNCS, &FlashStatus);
    }

    if(status != STATUS_SUCCESS)
        {
            sendCANMessage(BACKUP_WRITE_ERROR);
            fail++;
            return ;
        }

    sendCANMessage(BACKUP_WRITE_END);

    for(i = 0; i < COPY_BUFFER_LENGTH_MAIN_FUNCS; i++){
           copy_buffer[i] = 0;
    }

    if(backup_state == LOAD_BACKUP)
    {
        flag_buff[FLASH_FLAG] = APPLICATION;
        flag_buff[JUMP_FLAG] = FULL_STATE;
        flag_buff[BACKUP_FLAG] = FULL_STATE;
        flag_buff[PACKET_FLAG] = NULL_STATE;
    }
    else if(backup_state == RELOAD_BACKUP)
    {
        flag_buff[FLASH_FLAG] = BOOT_CONFIG;
        flag_buff[JUMP_FLAG] = read_flag(JUMP_FLAG);
        flag_buff[BACKUP_FLAG] = NULL_STATE;
        flag_buff[PACKET_FLAG] = read_flag(PACKET_FLAG);
    }

    sendCANMessage(BACKUP_WRITE_START);

    status = Flash_Erase(SECTORE,
                         &FlashStatus);
    if(status != STATUS_SUCCESS)
    {
        fail++;
        sendCANMessage(BACKUP_WRITE_ERROR);
        return;
    }

    sendCANMessage(BACKUP_WRITE_END);

    sendCANMessage(BACKUP_WRITE_START);

    status = Flash_Program((Uint16 *) FLASH_FLAG_ADDRESS,
             (Uint16 *)flag_buff, FLASH_FLAG_LENGTH, &FlashStatus);
    if(status != STATUS_SUCCESS)
    {
        sendCANMessage(BACKUP_WRITE_ERROR);
        fail++;
        return ;
    }

    sendCANMessage(BACKUP_WRITE_END);
    sendCANMessage(LOAD_COMPLETE);

    return;

}

void init_jump_to_BMS()
{
    Uint16 status;
    Uint16 flag_buff[FLASH_FLAG_LENGTH];

    CsmUnlock();

    EALLOW;
    Flash_CPUScaleFactor = SCALE_FACTOR;
    Flash_CallbackPtr = NULL;
    EDIS;

//    sendENDMessage();
    if(read_flag(PACKET_FLAG))
    {
//        sendENDMessage();
        if(read_flag(PACKET_FLAG) > 2) // LOAD BACKUP
        {
            flag_buff[FLASH_FLAG] = BOOT_ERROR;
            flag_buff[JUMP_FLAG] = read_flag(JUMP_FLAG);
            flag_buff[BACKUP_FLAG] = FULL_STATE;
            flag_buff[PACKET_FLAG] = NULL_STATE;
        }
        else
        {
            flag_buff[FLASH_FLAG] = BOOTLOADER;
            flag_buff[JUMP_FLAG] = read_flag(JUMP_FLAG);
            flag_buff[BACKUP_FLAG] = read_flag(BACKUP_FLAG);
            flag_buff[PACKET_FLAG] = read_flag(PACKET_FLAG);
        }
        status = Flash_Erase(SECTORE,
                             &FlashStatus);
        if(status != STATUS_SUCCESS)
        {
            resetBMS();
        }

        status = Flash_Program((Uint16 *) FLASH_FLAG_ADDRESS,
                 (Uint16 *)flag_buff, FLASH_FLAG_LENGTH, &FlashStatus);
        if(status != STATUS_SUCCESS)
        {
            resetBMS();
        }
        sendCANMessage(FLASH_ERROR);
        resetBMS();
    }
    else if(read_flag(JUMP_FLAG) > 2)
    {
//        sendENDMessage();
        if(read_flag(BACKUP_FLAG) != BOOT_INIT && read_flag(BACKUP_FLAG) > 2) // LOAD BACKUP
        {
            flag_buff[FLASH_FLAG] = BOOT_ERROR;
            flag_buff[JUMP_FLAG] = read_flag(JUMP_FLAG);
            flag_buff[BACKUP_FLAG] = FULL_STATE;
            flag_buff[PACKET_FLAG] = NULL_STATE;
        }
        else
        {
            flag_buff[FLASH_FLAG] = BOOTLOADER;
            flag_buff[JUMP_FLAG] = read_flag(JUMP_FLAG);
            flag_buff[BACKUP_FLAG] = (read_flag(BACKUP_FLAG)) + FULL_STATE;
            flag_buff[PACKET_FLAG] = read_flag(PACKET_FLAG);
        }
        status = Flash_Erase(SECTORE,
                             &FlashStatus);
        if(status != STATUS_SUCCESS)
        {
            resetBMS();
        }

        status = Flash_Program((Uint16 *) FLASH_FLAG_ADDRESS,
                 (Uint16 *)flag_buff, FLASH_FLAG_LENGTH, &FlashStatus);
        if(status != STATUS_SUCCESS)
        {
            resetBMS();
        }
        sendCANMessage(FLASH_ERROR);
        resetBMS();
    }
    else
    {
//        sendENDMessage();
        flag_buff[FLASH_FLAG] = read_flag(FLASH_FLAG);
        flag_buff[JUMP_FLAG] = (read_flag(JUMP_FLAG)) + FULL_STATE;
        flag_buff[BACKUP_FLAG] = read_flag(BACKUP_FLAG);
        flag_buff[PACKET_FLAG] = read_flag(PACKET_FLAG);
        status = Flash_Erase(SECTORE,
                             &FlashStatus);
        if(status != STATUS_SUCCESS)
        {
            resetBMS();
        }

        status = Flash_Program((Uint16 *) FLASH_FLAG_ADDRESS,
                 (Uint16 *)flag_buff, FLASH_FLAG_LENGTH, &FlashStatus);
        if(status != STATUS_SUCCESS)
        {
            resetBMS();
        }
        load_application();
    }
}

bool check_packet_count()
{
    bool packet_status;

    if(packet_count*6 == size_of_update)
    {
        packet_status = true;
    }
    else
    {
        packet_status = false;
    }
    return packet_status;
}

void end_flashing()
{
    Uint16 status;
    Uint16 flag_buff[FLASH_FLAG_LENGTH];
    CsmUnlock();

    EALLOW;
    Flash_CPUScaleFactor = SCALE_FACTOR;
    Flash_CallbackPtr = NULL;
    EDIS;

    if(check_packet_count())
    {
        flag_buff[FLASH_FLAG] = read_flag(FLASH_FLAG);
        flag_buff[JUMP_FLAG] = read_flag(JUMP_FLAG);
        flag_buff[BACKUP_FLAG] = read_flag(BACKUP_FLAG);
        flag_buff[PACKET_FLAG] = NULL_STATE;
    }
    else
    {
        flag_buff[FLASH_FLAG] = read_flag(FLASH_FLAG);
        flag_buff[JUMP_FLAG] = read_flag(JUMP_FLAG);
        flag_buff[BACKUP_FLAG] = read_flag(BACKUP_FLAG);
        flag_buff[PACKET_FLAG] = read_flag(PACKET_FLAG) + FULL_STATE;
    }
    status = Flash_Erase(SECTORE,
                         &FlashStatus);
    if(status != STATUS_SUCCESS)
    {
        resetBMS();
    }

    status = Flash_Program((Uint16 *) FLASH_FLAG_ADDRESS,
             (Uint16 *)flag_buff, FLASH_FLAG_LENGTH, &FlashStatus);
    if(status != STATUS_SUCCESS)
    {
        resetBMS();
    }
    sendENDMessage();
}

void load_application()
{
    ((void(*)())APPLICATION_BEGIN_ADDRESS)();
}

void resetBMS(){

    EALLOW;  // Enable write access to protected registers
    SysCtrlRegs.WDCR = 0;  // Write an incorrect value to the WDCR register
    EDIS;    // Disable write access to protected registers
}

//
// End of File
//

