/*
 * flash_al.c
 *
 * FLASH Abstraction Layer
 *
 *  Created on: Jun 5, 2015
 *      Author: Angus Galloway
 */

#include "flash_al.h"

/* This is a FLASH Abstraction Layer that provides
   functions that can be called from vscp external helper functions */

pFLASHCOMMANDSEQUENCE g_FlashLaunchCommand = (pFLASHCOMMANDSEQUENCE)0xFFFFFFFF;		/*! flash command sequence as defined by C90TFS driver */

/* Flash Standard Software Driver Structure. */
FLASH_SSD_CONFIG flashSSDConfig =
{
    FTFx_REG_BASE,          /*! FTFx control register base */
    P_FLASH_BASE,           /*! Base address of PFlash block */
    P_FLASH_SIZE,           /*! Size of PFlash block */
    FLEXNVM_BASE,           /*! Base address of DFlash block */
    0,                      /*! Size of DFlash block */
    EERAM_BASE,             /*! Base address of EERAM block */
    0,                      /*! Size of EEE block */
    DEBUGENABLE,            /*! Background debug mode enable bit */
    NULL_CALLBACK           /*! Pointer to callback function */
};


/* VSCP_FLASH_BASE is defined near the end of flash space as
 * an offset so that the following VSCP addresses can stay the same.
 * PFlashSize = 0x10_0000 = 1MB
 * VSCP_FLASH_BASE = 0xf_a000 = 0x10_0000 - 6 * (0x1000 = 4KB sectors)
 * Result = 24KB of space for VSCP
 * vscp data space was reserved in linker (MK64FN1M0xxx12_flash.ld)
 * m_vscp_data			(RX)  : ORIGIN = 0x000FA000, LENGTH = 0x00006000 */

const uint32_t VSCP_FLASH_BASE = 0xfa000;

/*!
    @brief initialize flash and perform checks
 */
void init_flash(void){

	uint8_t  securityStatus = 0;   /*! Return protection status */
	uint32_t result = FlashInit(&flashSSDConfig);

	if (FTFx_OK != result)
	{
		error_trap();
	}
	// print welcome message
	printf("\n\r Flash Example Start \r\n");
	// Print flash information - PFlash.
	printf("\n\r Flash Information: ");
	printf("\n\r Total Flash Size:\t%d KB, Hex: (0x%x)", (P_FLASH_SIZE/ONE_KB), P_FLASH_SIZE);
	printf("\n\r Flash Sector Size:\t%d KB, Hex: (0x%x) ", (FTFx_PSECTOR_SIZE/ONE_KB), FTFx_PSECTOR_SIZE);
	// Check if DFlash exist on this device.
	if (flashSSDConfig.DFlashSize)
	{
		printf("\n\r Data Flash Size:\t%d KB,\tHex: (0x%x)", (int)(flashSSDConfig.DFlashSize/ONE_KB), (unsigned int)flashSSDConfig.DFlashSize);
		printf("\n\r Data Flash Base Address:\t0x%x", (unsigned int)flashSSDConfig.DFlashBase);
	}
	else
	{
		printf("\n\r There is no D-Flash (FlexNVM) on this Device.");
	}
	// Check if FlexMemory exist on this device.
	if (flashSSDConfig.EEESize)
	{
		printf("\n\r Enhanced EEPROM (EEE) Block Size:\t%d KB,\tHex: (0x%x)", (int)(flashSSDConfig.EEESize/ONE_KB), (unsigned int)flashSSDConfig.EEESize);
		printf("\n\r Enhanced EEPROM (EEE) Base Address:\t0x%x", (unsigned int)flashSSDConfig.EERAMBase);
	}
	else
	{
		printf("\n\r There is no Enhanced EEPROM (EEE) on this Device.");
	}

	// Check security status.
	result = FlashGetSecurityState(&flashSSDConfig, &securityStatus);
	if (FTFx_OK != result)
	{
		error_trap();
	}
	// Print security status.
	switch(securityStatus)
	{
	case FLASH_NOT_SECURE:
		printf("\n\r Flash is UNSECURE!");
		break;
	case FLASH_SECURE_BACKDOOR_ENABLED:
		printf("\n\r Flash is SECURE, BACKDOOR is ENABLED!");
		break;
	case FLASH_SECURE_BACKDOOR_DISABLED:
		printf("\n\r Flash is SECURE, BACKDOOR is DISABLED!");
		break;
	default:
		break;
	}
	printf("\n\r");
}

/*!
    read a word from flash
    @param address of memory to be read.
    @return byte read.
 */
uint32_t readFLASH(uint32_t address)
{
	return *(address + (uint32_t *) VSCP_FLASH_BASE);
}

/*!
    write a byte of data to flash
    @param address of memory to be written.
    @return byte read.
 */
void writeFLASH(uint32_t address, uint8_t *data)
{

	uint32_t result;
	uint16_t number;               /*! Number of longword or phrase to be verified */
	uint32_t marginReadLevel;      /*! 0=normal, 1=user, 2=factory - margin read for reading */

	/* Use this line when passing in an offset */
	//uint32_t effectiveAddress = (address * 4) + (uint32_t) VSCP_FLASH_BASE; //byte addressing, word fetching

	// Set command to RAM.
	g_FlashLaunchCommand = (pFLASHCOMMANDSEQUENCE)RelocateFunction((uint32_t)ramFunc , LAUNCH_CMD_SIZE ,(uint32_t)FlashCommandSequence);

	// Erase a sector from destAdrss.
	//destAdrss = flashSSDConfig.PFlashBase + (flashSSDConfig.PFlashSize - 6*FTFx_PSECTOR_SIZE);  //write 6 sectors in from end of flash

	result = FlashEraseSector(&flashSSDConfig, address, FTFx_PSECTOR_SIZE, g_FlashLaunchCommand);
	//result = FlashEraseSector(&flashSSDConfig, VSCP_FLASH_BASE, FTFx_PSECTOR_SIZE, g_FlashLaunchCommand);

	if (FTFx_OK != result)
	{
		error_trap();
	}

	// Verify sector if it's been erased.
	// Number of long to verify.
	number = FTFx_PSECTOR_SIZE/FSL_FEATURE_FLASH_PFLASH_SECTION_CMD_ADDRESS_ALIGMENT;
	for(marginReadLevel = 0; marginReadLevel <=0x2; marginReadLevel++)
	{
		result = FlashVerifySection(&flashSSDConfig, address, number, marginReadLevel, g_FlashLaunchCommand);
		//result = FlashVerifySection(&flashSSDConfig, VSCP_FLASH_BASE, number, marginReadLevel, g_FlashLaunchCommand);
		if (FTFx_OK != result)
		{
			error_trap();
		}
	}
	// Print message for user.
	printf("\n\r Successfully Erased Sector 0x%x -> 0x%x\r\n", (unsigned int)address, (unsigned int)(address+FTFx_PSECTOR_SIZE));

	//result = FlashProgram(&flashSSDConfig, VSCP_FLASH_BASE, BUFFER_SIZE_BYTE, data, g_FlashLaunchCommand);
	result = FlashProgram(&flashSSDConfig, address, BUFFER_SIZE_BYTE, data, g_FlashLaunchCommand);

	if (FTFx_ERR_ACCERR == result)
	{
		printf("\n\r FlashProgram Error FTFx_ERR_ACCERR\r\n");
		error_trap();
	}
	else if (FTFx_ERR_PVIOL == result)
	{
		printf("\n\r FlashProgram Error FTFx_ERR_PVIOL\r\n");
		error_trap();
	}
	else if (FTFx_ERR_SIZE == result)
	{
		printf("\n\r FlashProgram Error FTFx_ERR_SIZE\r\n");
		error_trap();
	}
	else if (FTFx_ERR_MGSTAT0 == result)
	{
		printf("\n\r FlashProgram Error FTFx_ERR_MGSTAT0\r\n");
		error_trap();
	}

	printf("\n\r FlashProgram success.\r\n");

	return;
}


/*
* @brief Gets called when an error occurs.
* Print error message and trap forever.
*/
void error_trap(void)
{
    printf("\n\n\n\r\t---- HALTED DUE TO FLASH ERROR! ----");
    while (1)
    {}
}

