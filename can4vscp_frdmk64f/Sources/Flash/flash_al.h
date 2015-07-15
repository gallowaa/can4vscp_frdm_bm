/*
 * flash_al.h
 *
 *  Created on: Jun 9, 2015
 *      Author: Angus
 */

#ifndef SOURCES_FLASH_AL_H_
#define SOURCES_FLASH_AL_H_

#include <stdio.h>
#include "SSD_FTFx.h"


extern const uint32_t VSCP_FLASH_BASE; //= 0xfa000;									/*! This is assigned in flash_al.c */

extern FLASH_SSD_CONFIG flashSSDConfig;												/*! Standard Software Driver flash configuration struct */


// size of array to copy__Launch_Command function to
// It should be at least equal to actual size of __Launch_Command func
// User can change this value based on RAM size availability and actual size of __Launch_Command function
#define LAUNCH_CMD_SIZE           0x100

#define BUFFER_SIZE_BYTE          0x80

#define DEBUGENABLE               0x00

#define READ_NORMAL_MARGIN        0x00
#define READ_USER_MARGIN          0x01
#define READ_FACTORY_MARGIN       0x02

#define ONE_KB                    1024

#define FTFx_REG_BASE             0x40020000
#define P_FLASH_BASE              0x00000000

#define BACKDOOR_KEY_LOCATION         0x400

// Program flash IFR
#if (FSL_FEATURE_FLASH_IS_FTFE == 1)
#define PFLASH_IFR                0x3C0

#else // FSL_FEATURE_FLASH_IS_FTFL == 1 or FSL_FEATURE_FLASH_IS_FTFA = =1
#define PFLASH_IFR                0xC0
#endif
// Program Flash block information
#define P_FLASH_SIZE            (FSL_FEATURE_FLASH_PFLASH_BLOCK_SIZE * FSL_FEATURE_FLASH_PFLASH_BLOCK_COUNT)
#define P_BLOCK_NUM             FSL_FEATURE_FLASH_PFLASH_BLOCK_COUNT
#define P_SECTOR_SIZE           FSL_FEATURE_FLASH_PFLASH_BLOCK_SECTOR_SIZE
// Data Flash block information
#define FLEXNVM_BASE            FSL_FEATURE_FLASH_FLEX_NVM_START_ADDRESS
#define FLEXNVM_SECTOR_SIZE     FSL_FEATURE_FLASH_FLEX_NVM_BLOCK_SECTOR_SIZE
#define FLEXNVM_BLOCK_SIZE      FSL_FEATURE_FLASH_FLEX_NVM_BLOCK_SIZE
#define FLEXNVM_BLOCK_NUM       FSL_FEATURE_FLASH_FLEX_NVM_BLOCK_COUNT

// Flex Ram block information
#define EERAM_BASE              FSL_FEATURE_FLASH_FLEX_RAM_START_ADDRESS
#define EERAM_SIZE              FSL_FEATURE_FLASH_FLEX_RAM_SIZE


// Has flash cache control in MCM module
#if (FSL_FEATURE_FLASH_HAS_MCM_FLASH_CACHE_CONTROLS == 1)
#define CACHE_DISABLE             MCM_BWR_PLACR_DFCS(MCM_BASE_PTR, 1);
// Has flash cache control in FMC module
#elif (FSL_FEATURE_FLASH_HAS_FMC_FLASH_CACHE_CONTROLS == 1)
#if defined(FMC_PFB1CR) && defined(FMC_PFB1CR_B1SEBE_MASK)
#define CACHE_DISABLE     FMC_PFB0CR &= ~(FMC_PFB0CR_B0SEBE_MASK | FMC_PFB0CR_B0IPE_MASK | FMC_PFB0CR_B0DPE_MASK | FMC_PFB0CR_B0ICE_MASK | FMC_PFB0CR_B0DCE_MASK);\
                                  FMC_PFB1CR &= ~(FMC_PFB1CR_B1SEBE_MASK | FMC_PFB1CR_B1IPE_MASK | FMC_PFB1CR_B1DPE_MASK | FMC_PFB1CR_B1ICE_MASK | FMC_PFB1CR_B1DCE_MASK);
#else
#define CACHE_DISABLE     FMC_PFB0CR &= ~(FMC_PFB0CR_B0SEBE_MASK | FMC_PFB0CR_B0IPE_MASK | FMC_PFB0CR_B0DPE_MASK | FMC_PFB0CR_B0ICE_MASK | FMC_PFB0CR_B0DCE_MASK);
#endif
#else
// No cache in the device
#define CACHE_DISABLE
#endif


extern pFLASHCOMMANDSEQUENCE g_FlashLaunchCommand;									/*! flash command sequence as defined by C90TFS driver */

uint16_t ramFunc[LAUNCH_CMD_SIZE/2];												/*! Array to copy __Launch_Command func to RAM.
 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	program flash from ram to avoid read-while-write error.
 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	the k64f has 2x 512KB blocks, so there is a high risk that
 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	the code which writes to flash is located in the same block
 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	as is being written */

/*!
    @brief initialize flash and perform checks
 */
void init_flash(void);

/*!
    read a byte of data from flash
    @param address of memory to be read.
    @return byte read.
 */
uint32_t readFLASH(uint32_t address);


/*!
    write a byte of data to flash
    @param address of memory to be written.
    @return byte read.
 */
void writeFLASH(uint32_t address, uint8_t *data);




/*
* @brief Gets called when an error occurs.
* Print error message and trap forever.
*/
void error_trap(void);


#endif /* SOURCES_FLASH_AL_H_ */
