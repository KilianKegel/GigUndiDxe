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
#ifndef VLAN_H_
#define VLAN_H_

#define EFI_VLAN_GUID \
  { \
    0xe1ad94a, 0xdcf4, 0x11db, \
    { \
      0x97, 0x5, 0x0, 0xe0, 0x81, 0x61, 0x16, 0x5f \
    } \
  }

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
typedef
EFI_STATUS
(EFIAPI *EFI_SET_VLAN_TAG) (
  IN      EFI_HANDLE              ControllerHandle,
  IN      BOOLEAN                 VlanEnable,
  IN      UINT16                  VlanId,
  IN      UINT16                  VlanPriority,
  IN      BOOLEAN                 VlanCfi
  );

typedef struct {
  UINT16             Version;
  EFI_SET_VLAN_TAG   SetVlanTag;
} EFI_VLAN_PROTOCOL;

#endif /* VLAN_H_ */
