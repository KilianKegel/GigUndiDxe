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
#include "E1000.h"

EFI_GUID gEfiVlanProtocolGuid = EFI_VLAN_GUID;

/** Enables or disables 802.3Q VLAN tagging on the specified network interface.

   @param[in]   ControllerHandle  The handle to the network interface to configure VLAN tagging.
   @param[in]   VlanEnable        Enable or disable 802.3Q ethernet header and VLAN tag insertion
   @param[in]   VlanId            Vlan tag to insert into each 802.3Q packet.
   @param[in]   VlanPriority      Vlan priority
   @param[in]   VlanCfi           Vlan canonical form indication

   @retval   EFI_SUCCESS             Vlan tag set successfully
   @retval   EFI_INVALID_PARAMETER   VlanId parameter out of range ( > 4095 ).
   @retval   EFI_INVALID_PARAMETER   VlanPriority parameter out of range.
   @retval   EFI_INVALID_PARAMETER   VlanCfi parameter out of range.
   @retval   !EFI_SUCCESS            Failed to open NII Pointer Protocol
**/
EFI_STATUS
GigUndiSetVlanTag (
  IN      EFI_HANDLE ControllerHandle,
  IN      BOOLEAN    VlanEnable,
  IN      UINT16     VlanId,
  IN      UINT16     VlanPriority,
  IN      BOOLEAN    VlanCfi
  )
{
  EFI_STATUS                Status;
  UNDI_PRIVATE_DATA *       UndiPrivateData;
  EFI_NII_POINTER_PROTOCOL *NiiPointerProtocol;
  UINT32                    Reg;

  if (VlanId > 0x0FFF) {
    DEBUGPRINT (VLAN, ("VlanId parameter out of range.\n"));
    return EFI_INVALID_PARAMETER;
  }

  if (VlanPriority > 7) {
    DEBUGPRINT (VLAN, ("VlanPriority parameter out of range.\n"));
    return EFI_INVALID_PARAMETER;
  }

  if (VlanCfi > 1) {
    DEBUGPRINT (VLAN, ("VlanCfi parameter out of range.\n"));
    return EFI_INVALID_PARAMETER;
  }

  //  Open an instance for the Network Interface Identifier Protocol so we can check
  // if the child handle interface is actually valid.
  DEBUGPRINT (VLAN, ("Open an instance for the Network Interface Identifier Protocol\n"));
  Status = gBS->OpenProtocol (
                  ControllerHandle,
                  &gEfiNiiPointerGuid,
                  (VOID **) &NiiPointerProtocol,
                  gUndiDriverBinding.DriverBindingHandle,
                  NULL,
                  EFI_OPEN_PROTOCOL_GET_PROTOCOL
                );

  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("2. OpenProtocol returns %r\n", Status));
    DEBUGWAIT (CRITICAL);
    return Status;
  }

  UndiPrivateData = UNDI_PRIVATE_DATA_FROM_THIS (NiiPointerProtocol->NiiProtocol31);

  DEBUGPRINT (VLAN, ("Subsystem Vendor ID = %X\n", UndiPrivateData->NicInfo.Hw.subsystem_vendor_id));
  DEBUGPRINT (VLAN, ("Vendor ID = %X\n", UndiPrivateData->NicInfo.Hw.vendor_id));
  DEBUGPRINT (VLAN, ("Device ID = %X\n", UndiPrivateData->NicInfo.Hw.device_id));

  Reg = E1000_READ_REG (&UndiPrivateData->NicInfo.Hw, E1000_CTRL);
  if (VlanEnable) {
    UndiPrivateData->NicInfo.VlanEnable = TRUE;
    UndiPrivateData->NicInfo.VlanTag = VlanId | (VlanCfi << 12) | (VlanPriority << 13);
    DEBUGPRINT (VLAN, ("VlanTag = %X\n", UndiPrivateData->NicInfo.VlanTag));
    Reg |= E1000_CTRL_VME;
    DEBUGPRINT (VLAN, ("VME in CTRL register enabled\n"));
  } else {
    UndiPrivateData->NicInfo.VlanEnable = FALSE;
    UndiPrivateData->NicInfo.VlanTag = 0;
    Reg &= ~E1000_CTRL_VME;
    DEBUGPRINT (VLAN, ("VME in CTRL register disabled\n"));
  }
  E1000_WRITE_REG (&UndiPrivateData->NicInfo.Hw, E1000_CTRL, Reg);
  DEBUGPRINT (VLAN, ("E1000_CTRL=%X\n", Reg));
  DEBUGPRINT (VLAN, ("Vlan setting complete.\n"));

  Reg = E1000_READ_REG (&UndiPrivateData->NicInfo.Hw, E1000_VET);
  DEBUGPRINT (VLAN, ("E1000_VET=%X\n", Reg));

  return Status;
}

/* Protocol structure definition and initialization */
EFI_VLAN_PROTOCOL gGigUndiVlanData = {
  1,
  GigUndiSetVlanTag
};

