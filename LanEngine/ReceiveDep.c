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
  Write physical address of the Rx buffer to a specific field within
  Rx descriptor.

  @param[in]   RxDesc             Pointer to Rx descriptor
  @param[in]   RxBuffer           Physical address of Rx buffer

**/
VOID
ReceiveAttachBufferToDescriptor (
  IN  RECEIVE_DESCRIPTOR    *RxDesc,
  IN  EFI_PHYSICAL_ADDRESS  RxBuffer
  )
{
  ASSERT (RxDesc != NULL);
  ASSERT (RxBuffer != 0);

  ZeroMem (RxDesc, sizeof (*RxDesc));
  RxDesc->buffer_addr = (UINT64) RxBuffer;
  RxDesc->status = E1000_RXD_STAT_IXSM;
}

/**
  Check whether adapter has finished processing specific Rx descriptor.
  Optional parameters can be provided to fill in additional information on
  received packet.

  @param[in]   RxDesc             Pointer to Rx descriptor.
  @param[out]  PacketLength       On output, length of received packet.
  @param[out]  HeaderLength       On output, length of received packet's header.
  @param[out]  RxError            On output, descriptor's RXERROR field content.
  @param[out]  PacketType         On output, descriptor's PTYPE field content.

  @retval      TRUE               Descriptor has been processed.
  @retval      FALSE              Descriptor has not been processed.

**/
BOOLEAN
ReceiveIsDescriptorDone (
  IN  RECEIVE_DESCRIPTOR  *RxDesc,
  OUT UINT16              *PacketLength   OPTIONAL,
  OUT UINT16              *HeaderLength   OPTIONAL,
  OUT UINT8               *RxError        OPTIONAL,
  OUT UINT8               *PacketType     OPTIONAL
  )
{
  ASSERT (RxDesc != NULL);

  if (!BIT_TEST (RxDesc->status, E1000_RXD_STAT_EOP | E1000_RXD_STAT_DD)) {
    return FALSE;
  }

  if (PacketLength != NULL) {
    *PacketLength = RxDesc->length;
  }
  if (HeaderLength != NULL) {
    // No header length in legacy descriptors.
    *HeaderLength = 0;
  }
  if (RxError != NULL) {
    *RxError = RxDesc->errors;
  }
  if (PacketType != NULL) {
    // No packet type in legacy descriptors.
    *PacketType = 0;
  }

  return TRUE;
}

/**
  Update device's Rx tail register with a given descriptor ID

  @param[in]   AdapterInfo        Pointer to the NIC data structure.
  @param[in]   DescId             Descriptor ID to be written.

**/
VOID
ReceiveUpdateTail (
  IN  DRIVER_DATA   *AdapterInfo,
  IN  UINT16        DescId
  )
{
  ASSERT (AdapterInfo != NULL);
  ASSERT (DescId < (RX_RING_FROM_ADAPTER (AdapterInfo))->BufferCount);
  E1000_WRITE_REG (&AdapterInfo->Hw, E1000_RDT (0), DescId);
}

/**
  Configure NIC to be ready to use initialized Rx queue.

  @param[in]   AdapterInfo        Pointer to the NIC data structure

  @retval EFI_SUCCESS             NIC successfully configured.

**/
EFI_STATUS
ReceiveConfigureQueue (
  IN DRIVER_DATA *AdapterInfo
  )
{
  RECEIVE_RING      *RxRing;
  UINT64            MemAddr;
  UINT32            *MemPtr;
  UINT32            TempReg;

  ASSERT (AdapterInfo != NULL);

  RxRing = RX_RING_FROM_ADAPTER (AdapterInfo);

  MemAddr = RxRing->Descriptors.PhysicalAddress;
  MemPtr  = (UINT32 *) &MemAddr;

  // Setup the RDBA, RDLEN
  // Write physical address of Rx descriptor buffer for HW to use
  E1000_WRITE_REG (&AdapterInfo->Hw, E1000_RDBAL (0), MemPtr[0]);
  E1000_WRITE_REG (&AdapterInfo->Hw, E1000_RDBAH (0), MemPtr[1]);

  // Setup RDLEN - size of descriptor buffer
  E1000_WRITE_REG (
    &AdapterInfo->Hw,
    E1000_RDLEN (0),
    sizeof (RECEIVE_DESCRIPTOR) * RxRing->BufferCount
    );

  // Reset RDT to RDH value (meaning no free Rx descriptors for the adapter).
  TempReg = E1000_READ_REG (&AdapterInfo->Hw, E1000_RDH (0));
  E1000_WRITE_REG (&AdapterInfo->Hw, E1000_RDT (0), TempReg);

#ifndef NO_82575_SUPPORT
  switch (AdapterInfo->Hw.mac.type) {
  case e1000_82575:
  case e1000_82576:
#ifndef NO_82580_SUPPORT
  case e1000_82580:
#endif /* !NO_82580_SUPPORT */
  case e1000_i350:
  case e1000_i354:
  case e1000_i210:
  case e1000_i211:
    E1000_WRITE_REG (
      &AdapterInfo->Hw,
      E1000_SRRCTL (0),
      E1000_SRRCTL_DESCTYPE_LEGACY
      );
  default:
    break;
  }
#endif /* !NO_82575_SUPPORT */

  E1000_WRITE_REG (&AdapterInfo->Hw, E1000_MRQC, 0);


  E1000PciFlush (&AdapterInfo->Hw);
  return EFI_SUCCESS;
}

/**
  Enable Rx queue on the NIC.

  @param[in]   AdapterInfo        Pointer to the NIC data structure

  @retval EFI_SUCCESS             Rx queue has been enabled.

**/
EFI_STATUS
ReceiveEnableQueue (
  IN DRIVER_DATA    *AdapterInfo
  )
{
  RECEIVE_RING      *RxRing;
  UINT32            TempReg;
  UINTN             i;

  ASSERT (AdapterInfo != NULL);

  RxRing = RX_RING_FROM_ADAPTER (AdapterInfo);

  switch (AdapterInfo->Hw.mac.type) {
#ifndef NO_82575_SUPPORT
  case e1000_82575:
    e1000_rx_fifo_flush_base (&AdapterInfo->Hw);

#ifndef NO_82576_SUPPORT
  case e1000_82576:
#endif /* !NO_82576_SUPPORT */
#ifndef NO_82580_SUPPORT
  case e1000_82580:
#endif /* !NO_82580_SUPPORT */
  case e1000_i350:
  case e1000_i354:
  case e1000_i210:
  case e1000_i211:
    E1000SetRegBits (AdapterInfo, E1000_RXDCTL (0), E1000_RXDCTL_QUEUE_ENABLE);

    i = 0;
    do {
      gBS->Stall (1);
      TempReg = E1000_READ_REG (&AdapterInfo->Hw, E1000_RXDCTL (0));

      i++;
      if (i >= MAX_QUEUE_ENABLE_TIME) {
        break;
      }
    } while (!BIT_TEST (TempReg, E1000_RXDCTL_QUEUE_ENABLE));

    E1000_WRITE_REG (&AdapterInfo->Hw, E1000_RDH (0), 0);
    // Note: RxRing->NextToUse is by default reset to 0.
    break;
#endif /* !NO_82575_SUPPORT */
  default:
    RxRing->NextToUse = (UINT16) E1000_READ_REG (&AdapterInfo->Hw, E1000_RDH (0));
  }

  E1000_WRITE_REG (&AdapterInfo->Hw, E1000_RDT (0), RxRing->BufferCount - 1);
  E1000SetRegBits (AdapterInfo, E1000_RCTL, E1000_RCTL_EN | E1000_RCTL_BAM);

  return EFI_SUCCESS;
}

/**
  Disable Rx queue on the NIC.

  @param[in]   AdapterInfo        Pointer to the NIC data structure

  @retval EFI_SUCCESS             Rx queue has been disabled.

**/
EFI_STATUS
ReceiveDisableQueue (
  IN DRIVER_DATA *AdapterInfo
  )
{
  RECEIVE_RING    *RxRing;
  UINT32          TempReg;
  UINTN           i;

  ASSERT (AdapterInfo != NULL);

  RxRing = RX_RING_FROM_ADAPTER (AdapterInfo);

  switch (AdapterInfo->Hw.mac.type) {
#ifndef NO_82575_SUPPORT
  case e1000_82575:
#ifndef NO_82576_SUPPORT
  case e1000_82576:
#endif /* !NO_82576_SUPPORT */
#ifndef NO_82580_SUPPORT
  case e1000_82580:
#endif /* !NO_82580_SUPPORT */
  case e1000_i350:
  case e1000_i354:
  case e1000_i210:
  case e1000_i211:
    E1000ClearRegBits (AdapterInfo, E1000_RXDCTL (0), E1000_RXDCTL_QUEUE_ENABLE);

    i = 0;
    do {
      gBS->Stall (1);
      TempReg = E1000_READ_REG (&AdapterInfo->Hw, E1000_RXDCTL (0));

      i++;
      if (i >= MAX_QUEUE_ENABLE_TIME) {
        break;
      }
    } while (BIT_TEST (TempReg, E1000_RXDCTL_QUEUE_ENABLE));

    E1000_WRITE_REG (&AdapterInfo->Hw, E1000_RDT (0), 0);
    E1000_WRITE_REG (&AdapterInfo->Hw, E1000_RDH (0), 0);
    break;
#endif /* !NO_82575_SUPPORT */
  default:
    TempReg = E1000_READ_REG (&AdapterInfo->Hw, E1000_RDH (0));
    E1000_WRITE_REG (&AdapterInfo->Hw, E1000_RDT (0), TempReg);
    break;
  }

  E1000ClearRegBits (AdapterInfo, E1000_RCTL, E1000_RCTL_EN);

  return EFI_SUCCESS;
}

/**
  Perform actions before receive ring resources are freed.

  @param[in]   AdapterInfo        Pointer to the NIC data structure

  @retval EFI_SUCCESS             Rx queue has been dismantled.

**/
EFI_STATUS
ReceiveDismantleQueue (
  IN  DRIVER_DATA   *AdapterInfo
  )
{
  return EFI_SUCCESS;
}
