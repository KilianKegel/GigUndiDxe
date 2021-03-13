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
#include "EepromConfig.h"
#include "wol.h"
#include "DeviceSupport.h"



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
  )
{
  UINTN  Active;
  UINT16 SetupOffset;
  UINT16 ConfigOffset;
  UINT16 SetupWord;
  UINT16 CustomConfigWord;

  struct e1000_hw *hw = &UndiPrivateData->NicInfo.Hw;

  // For media other than copper we do not support speed settings
  if (hw->phy.media_type != e1000_media_type_copper) {
    return LINK_SPEED_AUTO_NEG;
  }

  if (hw->mac.type == e1000_i211) {

    // I211 is OTP device. On init default configuration read from ROM.
    // HII or UNDI init command can changes the configuration only for current
    // power cycle, but it cannot be preserved for reboot. In case user enters/exits
    // the HII browser many times we want to display current configuration.

    // Assume auto negotiation is configured.
    Active = LINK_SPEED_AUTO_NEG;

    // If autonegotiation is not configured,
    // decode from enums used by HII to bits used in shared code.
    if (!hw->mac.autoneg) {
      UINTN i;

      for (i = LINK_SPEED_10HALF; i <= LINK_SPEED_100FULL; ++i) {
        if (hw->mac.forced_speed_duplex & (1U << (i - 1))) {
          Active = i;
          break;
        }
      }
    }

    return Active;
  }

  // If the device is a dual port device then we need to use the EEPROM settings
  // for the second adapter port
  switch (UndiPrivateData->NicInfo.Function) {
  case 0:
    ConfigOffset = CONFIG_CUSTOM_WORD;
    SetupOffset  = SETUP_OPTIONS_WORD;
    break;
  case 1:
    ConfigOffset = CONFIG_CUSTOM_WORD_LANB;
    SetupOffset  = SETUP_OPTIONS_WORD_LANB;
    break;
  case 2:
    ConfigOffset = CONFIG_CUSTOM_WORD_LANC;
    SetupOffset  = SETUP_OPTIONS_WORD_LANC;
    break;
  case 3:
    ConfigOffset = CONFIG_CUSTOM_WORD_LAND;
    SetupOffset  = SETUP_OPTIONS_WORD_LAND;
    break;
  default:
    ConfigOffset = CONFIG_CUSTOM_WORD;
    SetupOffset  = SETUP_OPTIONS_WORD;
    break;
  }

  e1000_read_nvm (hw, SetupOffset, 1, &SetupWord);

  // If the boot agent EEPROM signature is not set properly then we will return
  // default link speed value which is Auto-negotiate.
  e1000_read_nvm (hw, ConfigOffset, 1, &CustomConfigWord);

  if ((CustomConfigWord & SIG_MASK) != SIG) {
    Active = LINK_SPEED_AUTO_NEG;
  } else {

    // The signature bits are set so get the speed duplex settings
    // Mask of the speed and duplex setting bits so that we can determine
    // what the settings are
    switch (SetupWord & (FSP_MASK | FDP_FULL_DUPLEX_BIT)) {
    case (FDP_FULL_DUPLEX_BIT | FSP_100MBS):
      hw->mac.autoneg              = 0;
      hw->mac.forced_speed_duplex  = ADVERTISE_100_FULL;
      Active = LINK_SPEED_100FULL;
      break;
    case (FDP_FULL_DUPLEX_BIT | FSP_10MBS):
      hw->mac.autoneg              = 0;
      hw->mac.forced_speed_duplex  = ADVERTISE_10_FULL;
      Active = LINK_SPEED_10FULL;
      break;
    case (FSP_100MBS):
      hw->mac.autoneg              = 0;
      hw->mac.forced_speed_duplex  = ADVERTISE_100_HALF;
      Active = LINK_SPEED_100HALF;
      break;
    case (FSP_10MBS):
      hw->mac.autoneg              = 0;
      hw->mac.forced_speed_duplex  = ADVERTISE_10_HALF;
      Active = LINK_SPEED_10HALF;
      break;
    default:
      hw->mac.autoneg              = 1;
      Active = LINK_SPEED_AUTO_NEG;
      break;
    }
  }

  return Active;
}

/** Sets LAN speed setting for port

   @param[in]   UndiPrivateData    Driver private data structure
   @param[in]   LanSpeed           Desired LAN speed

   @retval   EFI_SUCCESS    LAN speed set successfully
**/
EFI_STATUS
EepromSetLanSpeed (
  UNDI_PRIVATE_DATA *UndiPrivateData,
  UINT8              LanSpeed
  )
{
  UINT16 SetupOffset;
  UINT16 ConfigOffset;
  UINT16 SetupWord;
  UINT16 OldSetupWord;
  UINT16 CustomConfigWord;
  UINT8  ReceiveStarted;

  struct e1000_hw *hw = &UndiPrivateData->NicInfo.Hw;

  // Configure offsets depending on function number
  switch (UndiPrivateData->NicInfo.Function) {
  case 0:
    ConfigOffset = CONFIG_CUSTOM_WORD;
    SetupOffset  = SETUP_OPTIONS_WORD;
    break;
  case 1:
    ConfigOffset = CONFIG_CUSTOM_WORD_LANB;
    SetupOffset  = SETUP_OPTIONS_WORD_LANB;
    break;
  case 2:
    ConfigOffset = CONFIG_CUSTOM_WORD_LANC;
    SetupOffset  = SETUP_OPTIONS_WORD_LANC;
    break;
  case 3:
    ConfigOffset = CONFIG_CUSTOM_WORD_LAND;
    SetupOffset  = SETUP_OPTIONS_WORD_LAND;
    break;
  default:
    ConfigOffset = CONFIG_CUSTOM_WORD;
    SetupOffset  = SETUP_OPTIONS_WORD;
    break;
  }

  if (e1000_read_nvm (hw, SetupOffset, 1, &SetupWord) != E1000_SUCCESS) {
    return EFI_DEVICE_ERROR;
  }
  if (e1000_read_nvm (hw, ConfigOffset, 1, &CustomConfigWord) != E1000_SUCCESS) {
    return EFI_DEVICE_ERROR;
  }
  // If the boot agent EEPROM signature is not set properly then we will initialize
  // the words to default values and assume a default autonegotiation setting
  if ((CustomConfigWord & SIG_MASK) != SIG) {
    CustomConfigWord = SIG;
    SetupWord = DISPLAY_SETUP_MESSAGE;
  }

  OldSetupWord = SetupWord;

  DEBUGPRINT (HII, ("Setting speed %d\n", LanSpeed));

  switch (LanSpeed) {
  case LINK_SPEED_AUTO_NEG:

    // Speed mask has already been cleared
    SetupWord &= ~FSP_MASK;
    SetupWord |= (FSP_AUTONEG);
    hw->mac.autoneg  = 1;
    break;
  case LINK_SPEED_100FULL:
    SetupWord &= ~FSP_MASK;
    SetupWord |= (FDP_FULL_DUPLEX_BIT | FSP_100MBS);
    hw->mac.autoneg  = 0;
    hw->mac.forced_speed_duplex  = ADVERTISE_100_FULL;
    break;
  case LINK_SPEED_100HALF:
    SetupWord &= ~(FSP_MASK | FDP_FULL_DUPLEX_BIT);
    SetupWord |= FSP_100MBS;
    hw->mac.autoneg  = 0;
    hw->mac.forced_speed_duplex  = ADVERTISE_100_HALF;
    break;
  case LINK_SPEED_10FULL:
    SetupWord &= ~FSP_MASK;
    SetupWord |= (FDP_FULL_DUPLEX_BIT | FSP_10MBS);
    hw->mac.autoneg  = 0;
    hw->mac.forced_speed_duplex  = ADVERTISE_10_FULL;
    break;
  case LINK_SPEED_10HALF:
    SetupWord &= ~(FSP_MASK | FDP_FULL_DUPLEX_BIT);
    SetupWord |= FSP_10MBS;
    hw->mac.forced_speed_duplex  = ADVERTISE_10_HALF;
    hw->mac.autoneg  = 0;
    break;
  default:
    break;
  }

  DEBUGPRINT (HII, ("SetupWord: %x\n", SetupWord));

  // Only write the EEPROM if the speed/duplex value has changed
  if (SetupWord != OldSetupWord) {
    if (e1000_write_nvm (hw, ConfigOffset, 1, &CustomConfigWord) == E1000_SUCCESS) {
      if (e1000_write_nvm (hw, SetupOffset, 1, &SetupWord) == E1000_SUCCESS) {
        // Success
        DEBUGPRINT (HII, ("Checksum\n"));
        e1000_update_nvm_checksum (hw);
      }
    }

    // After speed/duplex setting completes we need to perform a full reset of the adapter.
    // If the adapter was initialized on entry then force a full reset of the adapter.
    // Also re-enable the receive unit if it was enabled before we started the PHY loopback test.
    if (UndiPrivateData->NicInfo.UndiEnabled) {
      ReceiveStarted = UndiPrivateData->NicInfo.ReceiveStarted;

      e1000_reset_hw (&UndiPrivateData->NicInfo.Hw);
      UndiPrivateData->NicInfo.HwInitialized = FALSE;
      if (UndiPrivateData->NicInfo.State == PXE_STATFLAGS_GET_STATE_INITIALIZED) {
        E1000Inititialize (&UndiPrivateData->NicInfo);
        DEBUGPRINT (HII, ("E1000Inititialize complete\n"));

        //  Restart the receive unit if it was running on entry
        if (ReceiveStarted) {
          DEBUGPRINT (HII, ("RESTARTING RU\n"));
          E1000ReceiveStart (&UndiPrivateData->NicInfo);
        }
      }
    }
    DEBUGPRINT (HII, ("ADAPTER RESET COMPLETE\n"));
  }
  return EFI_SUCCESS;
}

/** Sets WOL (enable/disable) setting

   @param[in]   UndiPrivateData    Driver private data structure
   @param[in]   Enable             Enable/Disable boolean.

   @return WOL enabled/disabled according to Enable value.
**/
VOID
EepromSetWol (
  UNDI_PRIVATE_DATA *UndiPrivateData,
  IN UINT8           Enable
  )
{
  UndiPrivateData->Configuration.WolEnable = WolEnableWakeOnLanEx (UndiPrivateData, Enable);
}

/** Reads the currently assigned MAC address and factory default MAC address.

   @param[in]    AdapterInfo          Pointer to adapter structure
   @param[in]    LanFunction          Required to calculate LAN function offset
                                      where factory MAC address is located
   @param[out]   DefaultMacAddress    Factory default MAC address of the adapter
   @param[out]   AssignedMacAddress   CLP Assigned MAC address of the adapter,
                                      or the factory MAC address if an alternate
                                      MAC address has not been assigned.

   @retval   EFI_SUCCESS   MAC addresses successfully read.
**/
EFI_STATUS
_EepromMacAddressGet82580 (
  IN  DRIVER_DATA *AdapterInfo,
  IN  UINT32       LanFunction,
  OUT UINT16      *DefaultMacAddress,
  OUT UINT16      *AssignedMacAddress
  )
{
  UINT16 BackupMacOffset;
  UINT16 FactoryMacOffset;
  UINT16 BackupMacAddress[3];

  // Read in the currently assigned address from the default MAC address location
  // Factory MAC address is located at the start of the LAN configuration block in EEPROM
  DEBUGPRINT (CLP, ("EepromMacAddressGet_82580.\n"));

  FactoryMacOffset = NVM_82580_LAN_FUNC_OFFSET (LanFunction);

  DEBUGPRINT (CLP, ("Factory MAC address at offset %X\n", FactoryMacOffset));

  e1000_read_nvm (&AdapterInfo->Hw, FactoryMacOffset, 3, &AssignedMacAddress[0]);

  // Check to see if the backup MAC address location is being used, otherwise the
  // factory MAC address location will be the default.
  e1000_read_nvm (&AdapterInfo->Hw, NVM_ALT_MAC_ADDR_PTR, 1, &BackupMacOffset);

  if (BackupMacOffset == 0xFFFF
    || BackupMacOffset == 0x0000)
  {
    DEBUGPRINT (CLP, ("Alt MAC Address feature not enabled.\n"));
    e1000_read_nvm (&AdapterInfo->Hw, FactoryMacOffset, 3, &DefaultMacAddress[0]);
  } else {

    // Adjust the MAC address offset if this is the second port (function 1)
    BackupMacOffset = BackupMacOffset + (UINT16) (3 * AdapterInfo->Function);
    DEBUGPRINT (CLP, ("MAC addresses at offset %X\n", BackupMacOffset));

    // Check if MAC address is backed up
    e1000_read_nvm (&AdapterInfo->Hw, BackupMacOffset, 1, &BackupMacAddress[0]);
    if (BackupMacAddress[0] == 0xFFFF) {

      // In this case the factory MAC address is not in the backup location, so the factory
      // default MAC address is the same as the address we read in from the EEPROM CORE 0/1
      // locations.
      DefaultMacAddress[0] = AssignedMacAddress[0];
      DefaultMacAddress[1] = AssignedMacAddress[1];
      DefaultMacAddress[2] = AssignedMacAddress[2];
    } else {

      // Read in the factory default Mac address.
      e1000_read_nvm (&AdapterInfo->Hw, BackupMacOffset, 3, &DefaultMacAddress[0]);
    }
  }
  return EFI_SUCCESS;
}


/** Programs the port with an alternate MAC address, and backs up the factory default
   MAC address.

   @param[in]   AdapterInfo   Pointer to adapter structure
   @param[in]   MacAddress    Value to set the MAC address to.

   @retval   EFI_UNSUPPORTED   Alternate MAC Address feature not enabled
   @retval   EFI_SUCCESS       Alternate MAC Address set successfully
**/
EFI_STATUS
_EepromMacAddressSet82580 (
  IN DRIVER_DATA *AdapterInfo,
  IN UINT16      *MacAddress
  )
{
  UINT16 BackupMacOffset;
  UINT16 FactoryMacOffset;
  UINT16 BackupMacAddress[3];

  // Read the address where the override MAC address is stored.
  e1000_read_nvm (&AdapterInfo->Hw, NVM_ALT_MAC_ADDR_PTR, 1, &BackupMacOffset);

  if (BackupMacOffset == 0xFFFF
    || BackupMacOffset == 0x0000)
  {
    DEBUGPRINT (CLP, ("Alt MAC Address feature not enabled.\n"));
    return EFI_UNSUPPORTED;
  }

  // Adjust the MAC address offset if this is the second port (function 1)
  BackupMacOffset = BackupMacOffset + (UINT16) (3 * AdapterInfo->Function);
  DEBUGPRINT (CLP, ("MAC addresses at offset %X\n", BackupMacOffset));

  // Factory MAC address is located at the start of the LAN configuration block in EEPROM
  FactoryMacOffset = NVM_82580_LAN_FUNC_OFFSET (AdapterInfo->LanFunction);

  DEBUGPRINT (CLP, ("Factory MAC address at offset %X\n", FactoryMacOffset));

  // Check if MAC address is backed up
  e1000_read_nvm (&AdapterInfo->Hw, BackupMacOffset, 1, &BackupMacAddress[0]);
  if (BackupMacAddress[0] == 0xFFFF) {

    // Read in the factory MAC address
    e1000_read_nvm (&AdapterInfo->Hw, FactoryMacOffset, 3, &BackupMacAddress[0]);

    // Now back it up
    e1000_write_nvm (&AdapterInfo->Hw, BackupMacOffset, 3, &BackupMacAddress[0]);
  }

  // At this point the factory MAC address should be in the backup location.  Now
  // write the new CLP assigned MAC address into the original factory location.
  e1000_write_nvm (&AdapterInfo->Hw, FactoryMacOffset, 3, &MacAddress[0]);

  e1000_update_nvm_checksum (&AdapterInfo->Hw);

  return EFI_SUCCESS;
}

/** Restores the factory default MAC address.

   @param[in]   AdapterInfo   Pointer to adapter structure

   @retval   EFI_UNSUPPORTED   Alternate MAC Address feature not enabled
   @retval   EFI_SUCCESS       Factory default MAC address restored
**/
EFI_STATUS
_EepromMacAddressDefault82580 (
  IN DRIVER_DATA *AdapterInfo
  )
{
  UINT16 BackupMacOffset;
  UINT16 FactoryMacOffset;
  UINT16 BackupMacAddress[3];

  DEBUGPRINT (CLP, ("EepromMacAddressDefault_82580 .\n"));

  // Read the address where the override MAC address is stored.
  e1000_read_nvm (&AdapterInfo->Hw, NVM_ALT_MAC_ADDR_PTR, 1, &BackupMacOffset);

  if (BackupMacOffset == 0xFFFF
    || BackupMacOffset == 0x0000)
  {
    DEBUGPRINT (CLP, ("Alt MAC Address feature not enabled.\n"));
    return EFI_UNSUPPORTED;
  }

  // Adjust the MAC address offset if this is the second port (function 1)
  BackupMacOffset = BackupMacOffset + (UINT16) (3 * AdapterInfo->Function);
  DEBUGPRINT (CLP, ("MAC addresses at offset %X\n", BackupMacOffset));

  // Check if MAC address is backed up
  e1000_read_nvm (&AdapterInfo->Hw, BackupMacOffset, 3, &BackupMacAddress[0]);
  if (BackupMacAddress[0] == 0xFFFF) {
    DEBUGPRINT (CRITICAL, ("No backup MAC addresses\n"));
    return EFI_SUCCESS;
  }

  // Restore the factory MAC address
  // Factory MAC address is located at the start of the LAN configuration block in EEPROM
  FactoryMacOffset = NVM_82580_LAN_FUNC_OFFSET (AdapterInfo->LanFunction);

  DEBUGPRINT (CLP, ("Factory MAC address at offset %X\n", FactoryMacOffset));

  e1000_write_nvm (&AdapterInfo->Hw, FactoryMacOffset, 3, &BackupMacAddress[0]);

  e1000_update_nvm_checksum (&AdapterInfo->Hw);

  return EFI_SUCCESS;
}


/** Reads the currently assigned MAC address and factory default MAC address.

   @param[in]    AdapterInfo          Pointer to adapter structure
   @param[out]   DefaultMacAddress   Factory default MAC address of the adapter
   @param[out]   AssignedMacAddress  CLP Assigned MAC address of the adapter,
                                     or the factory MAC address if an alternate MAC
                                     address has not been assigned.

   @retval   EFI_SUCCESS   MAC addresses returned successfully
**/
EFI_STATUS
_EepromMacAddressGetGeneric (
  IN  DRIVER_DATA *AdapterInfo,
  OUT UINT16      *DefaultMacAddress,
  OUT UINT16      *AssignedMacAddress
  )
{
  UINT16 AlternateMacOffset;
  UINT16 FactoryMacOffset;
  UINT16 AlternateMacAddress[3];

  // Read in the currently assigned address from the default MAC address location
  // Factory MAC address is located at the start of the LAN configuration block in EEPROM
  FactoryMacOffset = 0x0000;

  DEBUGPRINT (CLP, ("Factory MAC address at offset %X\n", FactoryMacOffset));

  e1000_read_nvm (&AdapterInfo->Hw, FactoryMacOffset, 3, &AssignedMacAddress[0]);
  DEBUGPRINT (CLP, ("Adjusting MAC address for PCI function %d\n", (CHAR8) AdapterInfo->Function));
  ((CHAR8 *) AssignedMacAddress)[5] ^= (CHAR8) AdapterInfo->Function;

  // Check to see if alternate MAC address is available, otherwise the
  // factory MAC address location will be the default.
  e1000_read_nvm (&AdapterInfo->Hw, NVM_ALT_MAC_ADDR_PTR, 1, &AlternateMacOffset);

  if (AlternateMacOffset == 0xFFFF
    || AlternateMacOffset == 0x0000)
  {
    DEBUGPRINT (CLP, ("Alt MAC Address feature not enabled.\n"));
    DefaultMacAddress[0] = AssignedMacAddress[0];
    DefaultMacAddress[1] = AssignedMacAddress[1];
    DefaultMacAddress[2] = AssignedMacAddress[2];
  } else {

    // Adjust the MAC address offset if this is the second port (function 1)
    AlternateMacOffset = AlternateMacOffset + (UINT16) (3 * AdapterInfo->LanFunction);
    DEBUGPRINT (CLP, ("MAC addresses at offset %X\n", AlternateMacOffset));

    // Check if MAC address is set up
    e1000_read_nvm (&AdapterInfo->Hw, AlternateMacOffset, 1, &AlternateMacAddress[0]);
    if (AlternateMacAddress[0] == 0xFFFF) {

      // In this case the alternate MAC address is not set, so the alternate
      // MAC address is the same as the factory MAC address.
      DefaultMacAddress[0] = AssignedMacAddress[0];
      DefaultMacAddress[1] = AssignedMacAddress[1];
      DefaultMacAddress[2] = AssignedMacAddress[2];
    } else {

      // Read in the alternate MAC address.
      e1000_read_nvm (&AdapterInfo->Hw, AlternateMacOffset, 3, &DefaultMacAddress[0]);
    }
  }
  return EFI_SUCCESS;
}

/** Reads the currently assigned MAC address and factory default MAC address.

   @param[in]   UndiPrivateData   Driver private data structure
   @param[in]   LanFunction       Required to calculate LAN function offset
                                  where factory MAC address is located
   @param[out]   DefaultMacAddress   Factory default MAC address of the adapter
   @param[out]   AssignedMacAddress  CLP Assigned MAC address of the adapter,
                                     or the factory MAC address if an alternate MAC
                                     address has not been assigned.

   @retval   EFI_SUCCESS   MAC addresses successfully read.
**/
EFI_STATUS
_EepromMacAddressGet (
  IN  UNDI_PRIVATE_DATA *UndiPrivateData,
  IN  UINT32             LanFunction,
  OUT UINT16 *           DefaultMacAddress,
  OUT UINT16 *           AssignedMacAddress
  )
{
  if ((UndiPrivateData->NicInfo.Hw.mac.type  == e1000_82580)
    || (UndiPrivateData->NicInfo.Hw.mac.type  == e1000_i350)
    || (UndiPrivateData->NicInfo.Hw.mac.type == e1000_i354)
    || (UndiPrivateData->NicInfo.Hw.mac.type  == e1000_i210)
    || (UndiPrivateData->NicInfo.Hw.mac.type  == e1000_i211)
    )
  {
    return _EepromMacAddressGet82580 (
             &UndiPrivateData->NicInfo,
             LanFunction,
             DefaultMacAddress,
             AssignedMacAddress
           );
  } else {
    return _EepromMacAddressGetGeneric (
             &UndiPrivateData->NicInfo,
             DefaultMacAddress,
             AssignedMacAddress
           );
  }
}

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
  )
{
  return _EepromMacAddressGet (
           UndiPrivateData,
           UndiPrivateData->NicInfo.LanFunction,
           DefaultMacAddress,
           AssignedMacAddress
         );
}


/** Sets the override MAC address to the default value.

   @param[in]   AdapterInfo   Pointer to adapter structure
   @param[in]   MacAddress    Value to set the MAC address to.

   @retval   EFI_UNSUPPORTED   Invalid offset for alternate MAC address
   @retval   EFI_SUCCESS       Default MAC address set successfully
**/
EFI_STATUS
_EepromMacAddressSetGeneric (
  IN DRIVER_DATA *AdapterInfo,
  IN UINT16      *MacAddress
  )
{
  UINT16 MacOffset;
  UINT16 OldMacAddress[3];
#if (DBG_LVL)
  UINT8 *MacAddr = (UINT8 *) MacAddress;

  DEBUGPRINT (
    CLP, ("MAC=%X-%X-%X-%X-%X-%X\n", MacAddr[0], MacAddr[1], MacAddr[2],
    MacAddr[3], MacAddr[4], MacAddr[5])
  );
#endif /* (DBG_LVL) */

  // Determine the location of the card.
  AdapterInfo->PciIo->GetLocation (
                        AdapterInfo->PciIo,
                        &AdapterInfo->Segment,
                        &AdapterInfo->Bus,
                        &AdapterInfo->Device,
                        &AdapterInfo->Function
                      );

  // Read the address where the override MAC address is stored.
  e1000_read_nvm (&AdapterInfo->Hw, NVM_ALT_MAC_ADDR_PTR, 1, &MacOffset);

  if (MacOffset == 0xFFFF) {
    DEBUGPRINT (CLP, ("Invalid offset for alt MAC address\n"));
    return EFI_UNSUPPORTED;
  }

  DEBUGPRINT (CLP, ("MAC addresses at offset %X\n", MacOffset));

  MacOffset = MacOffset + (UINT16) (3 * AdapterInfo->Function);

  // Read the current stored MAC address to see if it needs to be changed
  e1000_read_nvm (&AdapterInfo->Hw, MacOffset, 3, OldMacAddress);

  if ((MacAddress[0] != OldMacAddress[0])
    || (MacAddress[1] != OldMacAddress[1])
    || (MacAddress[2] != OldMacAddress[2]))
  {
    DEBUGPRINT (CLP, ("Updating MAC addresses in EEPROM\n"));
    e1000_write_nvm (&AdapterInfo->Hw, MacOffset, 3, MacAddress);
  } else {
    DEBUGPRINT (CLP, ("No need to update MAC addresses in EEPROM\n"));
  }
  return EFI_SUCCESS;
}



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
  )
{
  if ((UndiPrivateData->NicInfo.Hw.mac.type  == e1000_82580)
    || (UndiPrivateData->NicInfo.Hw.mac.type  == e1000_i350)
    || (UndiPrivateData->NicInfo.Hw.mac.type == e1000_i354)
    || (UndiPrivateData->NicInfo.Hw.mac.type  == e1000_i210)
    || (UndiPrivateData->NicInfo.Hw.mac.type  == e1000_i211)
    )
  {
    return _EepromMacAddressSet82580 (&UndiPrivateData->NicInfo, MacAddress);
  } else {
    return _EepromMacAddressSetGeneric (&UndiPrivateData->NicInfo, MacAddress);
  }
}

/** Sets the override MAC address back to FF-FF-FF-FF-FF-FF to disable.

   @param[in]   AdapterInfo   Pointer to adapter structure

   @retval   EFI_UNSUPPORTED   Invalid offset for alternate MAC address
   @retval   EFI_SUCCESS       Default MAC address set successfully
**/
EFI_STATUS
_EepromMacAddressDefaultGeneric (
  IN DRIVER_DATA *AdapterInfo
  )
{
  UINT16 MacOffset;
  UINT16 MacAddress[3];
  UINT16 MacDefault[3] = {0xFFFF, 0xFFFF, 0xFFFF};

  // Determine the location of the card.
  AdapterInfo->PciIo->GetLocation (
                        AdapterInfo->PciIo,
                        &AdapterInfo->Segment,
                        &AdapterInfo->Bus,
                        &AdapterInfo->Device,
                        &AdapterInfo->Function
                      );

  // Read the address where the override MAC address is stored.
  e1000_read_nvm (&AdapterInfo->Hw, NVM_ALT_MAC_ADDR_PTR, 1, &MacOffset);

  if (MacOffset == 0xFFFF) {
    DEBUGPRINT (CLP, ("Invalid offset for alt MAC address\n"));
    return EFI_UNSUPPORTED;
  }

  DEBUGPRINT (CLP, ("MAC addresses at offset %X\n", MacOffset));

  // Adjust the MAC address offset if this is the second port (function 1)
  MacOffset = MacOffset + (UINT16) (3 * AdapterInfo->Function);

  // Read the current stored MAC address to see if it needs to be changed
  e1000_read_nvm (&AdapterInfo->Hw, MacOffset, 3, (UINT16 *) MacAddress);

  if ((MacAddress[0] != 0xFFFF)
    || (MacAddress[1] != 0xFFFF)
    || (MacAddress[2] != 0xFFFF))
  {
    DEBUGPRINT (CLP, ("Setting default MAC addresses in EEPROM\n"));
    e1000_write_nvm (&AdapterInfo->Hw, MacOffset, 3, MacDefault);
  } else {
    DEBUGPRINT (CLP, ("No need to update MAC addresses in EEPROM\n"));
  }
  return EFI_SUCCESS;

}


/** Sets the override MAC address back to FF-FF-FF-FF-FF-FF to disable
   (or in 82580-like case) restores the factory default MAC address.

   @param[in]   UndiPrivateData   Driver private data structure

   @retval   EFI_UNSUPPORTED   Invalid offset for alternate MAC address
   @retval   EFI_SUCCESS       Default MAC address set successfully
**/
EFI_STATUS
EepromMacAddressDefault (
  IN UNDI_PRIVATE_DATA *UndiPrivateData
  )
{
  if ((UndiPrivateData->NicInfo.Hw.mac.type  == e1000_82580)
    || (UndiPrivateData->NicInfo.Hw.mac.type  == e1000_i350)
    || (UndiPrivateData->NicInfo.Hw.mac.type == e1000_i354)
    || (UndiPrivateData->NicInfo.Hw.mac.type  == e1000_i210)
    || (UndiPrivateData->NicInfo.Hw.mac.type  == e1000_i211)
    )
  {
    return _EepromMacAddressDefault82580 (&UndiPrivateData->NicInfo);
  } else {
    return _EepromMacAddressDefaultGeneric (&UndiPrivateData->NicInfo);
  }
}

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
  )
{
  UINT16     Word;
  EFI_STATUS Status;

  Status = e1000_read_nvm (&UndiPrivateData->NicInfo.Hw, EEPROM_CAPABILITIES_WORD, 1, &Word);
  Word &= ~EEPROM_CAPABILITIES_SIG;
  *CapabilitiesWord = Word;

  return Status;
}

/** Checks if it is LOM device

   @param[in]   UndiPrivateData   Points to the driver instance private data

   @retval   TRUE     It is LOM device
   @retval   FALSE    It is not LOM device
   @retval   FALSE    Failed to read NVM word
**/
BOOLEAN
EepromIsLomDevice (
  IN  UNDI_PRIVATE_DATA *UndiPrivateData
  )
{
  EFI_STATUS Status;
  UINT16     SetupWord;

  Status = e1000_read_nvm (&UndiPrivateData->NicInfo.Hw, COMPATIBILITY_WORD, 1, &SetupWord);
  if (Status != E1000_SUCCESS) {
    return FALSE;
  }

  if ((SetupWord & COMPATABILITY_LOM_BIT) == COMPATABILITY_LOM_BIT) {
    return TRUE;
  }
  return FALSE;
}

/** Updates NVM checksum

   @param[in]   UndiPrivateData   Pointer to driver private data structure

   @retval      EFI_SUCCESS       Checksum successfully updated
   @retval      EFI_DEVICE_ERROR  Failed to update NVM checksum
**/
EFI_STATUS
EepromUpdateChecksum (
  IN  UNDI_PRIVATE_DATA *UndiPrivateData
  )
{
  if (e1000_update_nvm_checksum (&UndiPrivateData->NicInfo.Hw) != E1000_SUCCESS) {
    return EFI_DEVICE_ERROR;
  }
  return EFI_SUCCESS;
}

