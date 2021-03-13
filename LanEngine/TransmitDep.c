/******************************************************************************
**                                                                           **
** INTEL CONFIDENTIAL                                                        **
**                                                                           **
** Copyright 2020 Intel Corporation All Rights Reserved.                     **
**                                                                           **
** The source code contained or described herein and all documents related   **
** to the source code ("Material") are owned by Intel Corporation or its     **
** suppliers or licensors.  Title to the Material remains with Intel         **
** Corporation or its suppliers and licensors.  The Material contains trade  **
** secrets and proprietary and confidential information of Intel or its      **
** suppliers and licensors.  The Material is protected by worldwide          **
** copyright and trade secret laws and treaty provisions.  No part of the    **
** Material may be used, copied, reproduced, modified, published, uploaded,  **
** posted, transmitted, distributed, or disclosed in any way without Intel's **
** prior express written permission.                                         **
**                                                                           **
** No license under any patent, copyright, trade secret or other             **
** intellectual property right is granted to or conferred upon you by        **
** disclosure or delivery of the Materials, either expressly, by             **
** implication, inducement, estoppel or otherwise.  Any license under such   **
** intellectual property rights must be express and approved by Intel in     **
** writing.                                                                  **
**                                                                           **
******************************************************************************/

#include "CommonDriver.h"

/**
  Check whether adapter has finished processing specific Tx descriptor.

  @param[in]   TxDesc             Pointer to Tx descriptor

  @retval      TRUE               Descriptor has been processed.
  @retval      FALSE              Descriptor has not been processed.

**/
BOOLEAN
TransmitIsDescriptorDone (
  IN  TRANSMIT_DESCRIPTOR    *TxDesc
  )
{
  ASSERT (TxDesc != NULL);
  return BIT_TEST (TxDesc->upper.fields.status, E1000_TXD_STAT_DD);
}

/**
  Setup descriptor to be ready for processing by NIC.

  @param[in]   AdapterInfo        Pointer to the NIC data structure.
  @param[in]   TxDesc             Pointer to Tx descriptor.
  @param[in]   Packet             Physical address of the buffer holding packet
                                  to be sent.
  @param[in]   PacketLength       Length of the packet to be sent.

**/
VOID
TransmitSetupDescriptor (
  IN  DRIVER_DATA            *AdapterInfo,
  IN  TRANSMIT_DESCRIPTOR    *TxDesc,
  IN  EFI_PHYSICAL_ADDRESS   Packet,
  IN  UINT16                 PacketLength
  )
{
  ASSERT (AdapterInfo != NULL);
  ASSERT (TxDesc != NULL);
  ASSERT (Packet != 0);
  ASSERT (PacketLength != 0);

  TxDesc->buffer_addr         = Packet;
  TxDesc->upper.fields.status = 0;
  TxDesc->lower.flags.length  = PacketLength;


  TxDesc->lower.data |= (E1000_TXD_CMD_EOP |
                         E1000_TXD_CMD_IFCS |
                         E1000_TXD_CMD_RS);
}

/**
  Update Tx ring tail register with Tx descriptor index.

  @param[in]   AdapterInfo        Pointer to the NIC data structure
  @param[in]   Index              Tx descriptor index.

**/
VOID
TransmitUpdateRingTail (
  IN  DRIVER_DATA   *AdapterInfo,
  IN  UINT8         Index
  )
{
  ASSERT (AdapterInfo != NULL);

  TransmitLockIo (AdapterInfo, TRUE);
  E1000_WRITE_REG (&AdapterInfo->Hw, E1000_TDT (0), Index);
  TransmitLockIo (AdapterInfo, FALSE);
}

/**
  Reset Tx descriptor to a valid (unused) state.

  @param[in]   TxDesc             Pointer to Tx descriptor.

**/
VOID
TransmitResetDescriptor (
  IN  TRANSMIT_DESCRIPTOR    *TxDesc
  )
{
  ASSERT (TxDesc != NULL);

  TxDesc->upper.fields.status = 0;
}

/**
  Configure NIC to be ready to use initialized Tx queue.

  @param[in]   AdapterInfo        Pointer to the NIC data structure

  @retval EFI_SUCCESS             NIC successfully configured.
  @retval EFI_DEVICE_ERROR        NIC operation failure.

**/
EFI_STATUS
TransmitConfigureQueue (
  IN  DRIVER_DATA   *AdapterInfo
  )
{
  TRANSMIT_RING   *TxRing;
  UINT64          MemAddr;
  UINT32          *MemPtr;
  UINT32          TempReg;

  ASSERT (AdapterInfo != NULL);

  // Set the transmit tail equal to the head pointer.
  // Makes the adapter see there is no work to be done.
  TempReg = E1000_READ_REG (&AdapterInfo->Hw, E1000_TDH (0));
  E1000_WRITE_REG (&AdapterInfo->Hw, E1000_TDT (0), TempReg);

  TxRing  = TX_RING_FROM_ADAPTER (AdapterInfo);
  MemAddr = TxRing->Descriptors.PhysicalAddress;
  MemPtr  = (UINT32 *) &MemAddr;

  E1000_WRITE_REG (&AdapterInfo->Hw, E1000_TDBAL (0), MemPtr[0]);
  E1000_WRITE_REG (&AdapterInfo->Hw, E1000_TDBAH (0), MemPtr[1]);
  E1000_WRITE_REG (
    &AdapterInfo->Hw,
    E1000_TDLEN (0),
    sizeof (TRANSMIT_DESCRIPTOR) * TxRing->BufferCount
    );

  E1000PciFlush (&AdapterInfo->Hw);

  return EFI_SUCCESS;
}

/**
  Enable Tx queue on the NIC.

  @param[in]   AdapterInfo        Pointer to the NIC data structure

  @retval EFI_SUCCESS             Tx queue has been enabled.

**/
EFI_STATUS
TransmitEnableQueue (
  IN DRIVER_DATA *AdapterInfo
  )
{
  UINT32    TempReg;
  UINTN     i;

#ifndef NO_82580_SUPPORT
  switch (AdapterInfo->Hw.mac.type) {
  case e1000_82580:
  case e1000_i350:
  case e1000_i354:
  case e1000_i210:
  case e1000_i211:

#define E1000_TXDCTL_ENABLE_TIMEOUT   1000

    E1000SetRegBits (AdapterInfo, E1000_TXDCTL (0), E1000_TXDCTL_QUEUE_ENABLE);

    for (i = 0; i < E1000_TXDCTL_ENABLE_TIMEOUT; i++) {
      TempReg = E1000_READ_REG (&AdapterInfo->Hw, E1000_TXDCTL (0));
      if ((TempReg & E1000_TXDCTL_QUEUE_ENABLE) != 0) {
        DEBUGPRINT (E1000, ("TX queue enabled, after attempt i = %d\n", i));
        break;
      }

      DelayInMicroseconds (AdapterInfo, 1);
    }

    if (i >= E1000_TXDCTL_ENABLE_TIMEOUT) {
      DEBUGPRINT (CRITICAL, ("Enable TX queue failed!\n"));
    }

    break;

  default:
    break;
  }
#endif /* !NO_82580_SUPPORT */

  E1000SetRegBits (AdapterInfo, E1000_TCTL, E1000_TCTL_EN | E1000_TCTL_PSP);
  E1000PciFlush (&AdapterInfo->Hw);

  return EFI_SUCCESS;
}

/**
  Disable Tx queue on the NIC.

  @param[in]   AdapterInfo        Pointer to the NIC data structure

  @retval EFI_SUCCESS             Tx queue has been disabled.

**/
EFI_STATUS
TransmitDisableQueue (
  IN DRIVER_DATA *AdapterInfo
  )
{
  UINTN     i;
  UINT32    TxdCtl;


  switch (AdapterInfo->Hw.mac.type) {
#ifndef NO_82575_SUPPORT
  case e1000_82575:
  case e1000_82576:
#endif /* NO_82575_SUPPORT */
#ifndef NO_82580_SUPPORT
  case e1000_82580:
#endif /* NO_82580_SUPPORT */
  case e1000_i350:
  case e1000_i354:
  case e1000_i210:
  case e1000_i211:

#define MAX_QUEUE_DISABLE_TIME  200

    E1000ClearRegBits (AdapterInfo, E1000_TXDCTL (0), E1000_TXDCTL_QUEUE_ENABLE);
    i = 0;
    do {
      gBS->Stall (1);
      TxdCtl = E1000_READ_REG (&AdapterInfo->Hw, E1000_TXDCTL (0));
    } while ((++i < MAX_QUEUE_DISABLE_TIME)
      && (BIT_TEST (TxdCtl, E1000_TXDCTL_QUEUE_ENABLE)));
    DEBUGPRINT (E1000, ("Tx disabled\n"));
    break;
  default:
    break;
  }

  E1000ClearRegBits (AdapterInfo, E1000_TCTL, E1000_TCTL_EN);
  E1000PciFlush (&AdapterInfo->Hw);
  return EFI_SUCCESS;
}

/**
  Perform actions before transmit ring resources are freed.

  @param[in]   AdapterInfo        Pointer to the NIC data structure

  @retval EFI_SUCCESS             Tx queue has been dismantled.

**/
EFI_STATUS
TransmitDismantleQueue (
  IN  DRIVER_DATA   *AdapterInfo
  )
{
  return EFI_SUCCESS;
}
