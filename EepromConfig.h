/**************************************************************************

Copyright (c) 2016, Intel Corporation

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Intel Corporation nor the names of its contributors
      may be used to endorse or promote products derived from this software
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

***************************************************************************/
#ifndef EEPROM_CONFIG_H_
#define EEPROM_CONFIG_H_

#include "E1000.h"

typedef enum {
  ADAPTER_PF_STATE_DISABLED = 0,
  ADAPTER_PF_STATE_ENABLED = 1
} ADAPTER_PF_STATE;


#ifndef BIT
#define BIT(a) (1UL << (a))
#endif /* BIT */

// EEPROM power management bit definitions
#define E1000_INIT_CONTROL_WORD1          0x0A
#define E1000_PME_ENABLE_BIT              0x0008

#define E1000_INIT_CONTROL_WORD2          0x0F
#define E1000_APM_PME_ENABLE_BIT          0x8000

#define E1000_LEGACY_APM_ENABLE_BIT       0x0004
#define E1000_LEGACY_FLASH_DISABLE_BIT    0x0100

#define PCH_APM_INIT_CONTROL_WORD         10
#define PCH_APM_ENABLE_BIT                0x0004

#define PCH2_APM_INIT_CONTROL_WORD        0x1A
#define PCH2_APM_ENABLE_BIT               0x0001

#define IOV_CONTROL_WORD_OFFSET           0x25
#define IOV_CONTROL_WORD_IOVENABLE_SHIFT  0
#define IOV_CONTROL_WORD_IOVENABLE_MASK   0x0001
#define IOV_CONTROL_WORD_MAXVFS_SHIFT     5
#define IOV_CONTROL_WORD_MAXVFS_MASK      0x00E0
#define IOV_CONTROL_WORD_MAXVFS_MAX       7

#define E1000_INIT_CONTROL_WORD3          0x24
#define E1000_INIT_CONTROL_WORD3_LANB     0x14
#define E1000_FLASH_DISABLE_BIT           0x0800
#define E1000_FLASH_DISABLE_BIT_ZOAR      0x0080
#define E1000_APM_ENABLE_BIT              0x0400

#define E1000_FLASH_SIZE_WORD_HARTW       0xF
#define E1000_NVM_TYPE_BIT_HARTW          0x1000

#define E1000_HARTW_FLASH_LAN_ADDRESS     0x21
#define E1000_HARTW_EXP_ROM_DISABLE       0x80  /* bit 7 */

#define E1000_SDP_CONTROL                 0x20
#define E1000_SDP_CONTROL_LAN_DIS_BIT     BIT (11)

#define LAN1_BASE_ADDRESS_82580           0x80
#define LAN2_BASE_ADDRESS_82580           0xC0
#define LAN3_BASE_ADDRESS_82580           0x100

/** Reads SR buffer.

   @param[in]   UndiPrivateData  Points to the driver information.
   @param[in]   Offset           Offset in words from module start.
   @param[in]   Length           Number of words to read.
   @param[out]  Data             Pointer to location with data to be read to.

   @retval    EFI_SUCCESS            Buffer successfully read.
   @retval    EFI_INVALID_PARAMETER  UndiPrivateData or Data is NULL.
   @retval    EFI_DEVICE_ERROR       Failed to read buffer.
**/
EFI_STATUS
ReadSrBuffer16 (
  IN  UNDI_PRIVATE_DATA  *UndiPrivateData,
  IN  UINT16             Offset,
  IN  UINT16             Length,
  OUT UINT16             *Data
  );

/** Reads SR word.

   @param[in]   UndiPrivateData  Points to the driver information.
   @param[in]   Offset           Offset in words from module start.
   @param[out]  Data             Pointer to location with data to be read to.

   @retval    EFI_SUCCESS     Word successfully read.
   @retval    !EFI_SUCCESS    Word not read, failure of underlying function.
**/
EFI_STATUS
ReadSr16 (
  IN  UNDI_PRIVATE_DATA  *UndiPrivateData,
  IN  UINT16             Offset,
  OUT UINT16             *Data
  );

/** Writes SR buffer.

   @param[in]   UndiPrivateData  Points to the driver information.
   @param[in]   Offset           Offset in words from module start.
   @param[in]   Length           Number of words to write.
   @param[in]   Data             Pointer to location with words to be written.

   @retval    EFI_SUCCESS            Buffer successfully written.
   @retval    EFI_INVALID_PARAMETER  UndiPrivateData or Data is NULL.
   @retval    EFI_DEVICE_ERROR       Failed to write buffer.
**/
EFI_STATUS
WriteSrBuffer16 (
  IN  UNDI_PRIVATE_DATA  *UndiPrivateData,
  IN  UINT16             Offset,
  IN  UINT16             Length,
  IN  UINT16             *Data
  );

/** Writes SR word.

   @param[in]   UndiPrivateData  Points to the driver information.
   @param[in]   Offset           Offset in words from module start.
   @param[in]   Data             Word to be written.

   @retval    EFI_SUCCESS    Word successfully written.
   @retval    !EFI_SUCCESS   Word not written, failure of underlying function.
**/
EFI_STATUS
WriteSr16 (
  IN  UNDI_PRIVATE_DATA  *UndiPrivateData,
  IN  UINT16             Offset,
  IN  UINT16             Data
  );



/** Gets LAN speed setting for port

   @param[in]   UndiPrivateData    Pointer to adapter structure

   @retval   LINK_SPEED_AUTO_NEG   We do not support speed settings
   @retval   LINK_SPEED_AUTO_NEG   Default Auto-Negotiation settings
   @retval   LINK_SPEED_100FULL    LAN speed 100 MBit Full duplex
   @retval   LINK_SPEED_10FULL     LAN speed 10 MBit Full duplex
   @retval   LINK_SPEED_100HALF    LAN speed 100 MBit Half duplex
   @retval   LINK_SPEED_10HALF     LAN speed 10 MBit Half duplex
**/
UINTN
EepromGetLanSpeedStatus (
  UNDI_PRIVATE_DATA *UndiPrivateData
  );

/** Sets LAN speed setting for port

   @param[in]   UndiPrivateData    Driver private data structure
   @param[in]   LanSpeed           Desired LAN speed

   @retval   EFI_SUCCESS    LAN speed set successfully
**/
EFI_STATUS
EepromSetLanSpeed (
  UNDI_PRIVATE_DATA *UndiPrivateData,
  UINT8              LanSpeed
  );

/** Sets WOL (enable/disable) setting

   @param[in]   UndiPrivateData    Driver private data structure
   @param[in]   Enable             Enable/Disable boolean.

   @return WOL enabled/disabled according to Enable value.
**/
VOID
EepromSetWol (
  UNDI_PRIVATE_DATA *UndiPrivateData,
  IN UINT8           Enable
  );

/** Sets the override MAC address back to FF-FF-FF-FF-FF-FF to disable
   (or in 82580-like case) restores the factory default MAC address.

   @param[in]   UndiPrivateData   Driver private data structure

   @retval   EFI_UNSUPPORTED   Invalid offset for alternate MAC address
   @retval   EFI_SUCCESS       Default MAC address set successfully
**/
EFI_STATUS
EepromMacAddressDefault (
  IN UNDI_PRIVATE_DATA *UndiPrivateData
  );



/** Programs the port with an alternate MAC address, and (in 82580-like case)
   backs up the factory default MAC address.

   @param[in]   UndiPrivateData   Pointer to driver private data structure
   @param[in]   MacAddress        Value to set the MAC address to.

   @retval   EFI_UNSUPPORTED   Invalid offset for alternate MAC address
   @retval   EFI_SUCCESS       Default MAC address set successfully
**/
EFI_STATUS
EepromMacAddressSet (
  IN UNDI_PRIVATE_DATA *UndiPrivateData,
  IN UINT16 *           MacAddress
  );

/** Reads the currently assigned MAC address and factory default MAC address.

   @param[in]    UndiPrivateData   Driver private data structure
   @param[out]   DefaultMacAddress   Factory default MAC address of the adapter
   @param[out]   AssignedMacAddress  CLP Assigned MAC address of the adapter,
                                     or the factory MAC address if an alternate MAC
                                     address has not been assigned.

   @retval   EFI_SUCCESS   MAC addresses successfully read.
**/
EFI_STATUS
EepromMacAddressGet (
  IN  UNDI_PRIVATE_DATA *UndiPrivateData,
  OUT UINT16 *           DefaultMacAddress,
  OUT UINT16 *           AssignedMacAddress
  );


#define EEPROM_CAPABILITIES_WORD 0x33
#define EEPROM_CAPABILITIES_SIG  0x4000

/** Returns EEPROM capabilities word (0x33) for current adapter

   @param[in]    UndiPrivateData   Points to the driver instance private data
   @param[out]   CapabilitiesWord   EEPROM capabilities word (0x33) for current adapter

   @retval   EFI_SUCCESS   Function completed successfully,
   @retval   !EFI_SUCCESS  Failed to read EEPROM capabilities word
**/
EFI_STATUS
EepromGetCapabilitiesWord (
  IN  UNDI_PRIVATE_DATA *UndiPrivateData,
  OUT UINT16 *           CapabilitiesWord
  );

/** Checks if it is LOM device

   @param[in]   UndiPrivateData   Points to the driver instance private data

   @retval   TRUE     It is LOM device
   @retval   FALSE    It is not LOM device
   @retval   FALSE    Failed to read NVM word
**/
BOOLEAN
EepromIsLomDevice (
  IN  UNDI_PRIVATE_DATA *UndiPrivateData
  );

/** Updates NVM checksum

   @param[in]   UndiPrivateData   Pointer to driver private data structure

   @retval      EFI_SUCCESS       Checksum successfully updated
   @retval      EFI_DEVICE_ERROR  Failed to update NVM checksum
**/
EFI_STATUS
EepromUpdateChecksum (
  IN  UNDI_PRIVATE_DATA *UndiPrivateData
  );


#endif /* EEPROM_CONFIG_H_ */
