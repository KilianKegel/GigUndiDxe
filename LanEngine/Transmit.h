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

#ifndef TRANSMIT_H_
#define TRANSMIT_H_

#include "CommonDriver.h"
#include "Dma.h"

#define TRANSMIT_RING_SIGNATURE       0x80865478    /* Intel vendor + 'Tx' */

typedef enum _TRANSMIT_BUFFER_STATE {
  TRANSMIT_BUFFER_STATE_FREE = 0,
  TRANSMIT_BUFFER_STATE_IN_QUEUE,
  TRANSMIT_BUFFER_STATE_UNMAPPED
} TRANSMIT_BUFFER_STATE;

typedef struct _TRANSMIT_BUFFER_ENTRY {
  TRANSMIT_BUFFER_STATE   State;
  UNDI_DMA_MAPPING        Mapping;
} TRANSMIT_BUFFER_ENTRY;

typedef struct _TRANSMIT_RING {
  UINT32                Signature;
  BOOLEAN               IsRunning;
  UINT8                 BufferCount;
  UNDI_DMA_MAPPING      Descriptors;
  TRANSMIT_BUFFER_ENTRY *BufferEntries;
  UINT16                NextToUse;
  UINT16                NextToUnmap;
  UINT16                NextToFree;
} TRANSMIT_RING;

/** Check whether Tx ring structure is in initialized state.

   @param[in]   ring  Tx ring pointer

   @return    TRUE if ring is initialized, FALSE otherwise
 */
#define IS_TX_RING_INITIALIZED(r)   ((r)->Signature == TRANSMIT_RING_SIGNATURE)

/**
  Initialize Tx ring structure of LAN engine.
  This function will allocate and initialize all the necessary resources.
  It also assumes (and checks) that TX_RING_FROM_ADAPTER (AdapterInfo)
  is zeroed.

  @param[in]   AdapterInfo        Pointer to the NIC data structure.
  @param[in]   BufferCount        Number of buffers Tx ring will accommodate.

  @retval EFI_SUCCESS             Tx ring initialized.
  @retval EFI_INVALID_PARAMETER   Parameters were NULL/invalid.
  @retval EFI_VOLUME_CORRUPTED    Tx ring structure was not zeroed.
  @retval EFI_OUT_OF_RESOURCES    Could not allocate necessary resources.
  @retval Others                  Other internal function error.

**/
EFI_STATUS
TransmitInitialize (
  IN DRIVER_DATA  *AdapterInfo,
  IN UINT8        BufferCount
  );

/**
  Start Tx ring.

  @param[in]   AdapterInfo        Pointer to the NIC data structure.

  @retval EFI_SUCCESS             Tx ring started.
  @retval EFI_INVALID_PARAMETER   Parameters were NULL/invalid.
  @retval EFI_VOLUME_CORRUPTED    Tx ring was not initialized.
  @retval EFI_ALREADY_STARTED     Tx ring was already started before.
  @retval EFI_DEVICE_ERROR        NIC operation failure.
  @retval Others                  Underlying function failure.

**/
EFI_STATUS
TransmitStart (
  IN DRIVER_DATA  *AdapterInfo
  );

/**
  Stop Tx ring.

  @param[in]   AdapterInfo        Pointer to the NIC data structure.

  @retval EFI_SUCCESS             Tx ring stopped.
  @retval EFI_INVALID_PARAMETER   Parameters were NULL/invalid.
  @retval EFI_VOLUME_CORRUPTED    Tx ring was not initialized.
  @retval EFI_NOT_STARTED         Tx ring was already stopped before.
  @retval EFI_DEVICE_ERROR        NIC operation failure.
  @retval Others                  Underlying function failure.

**/
EFI_STATUS
TransmitStop (
  IN DRIVER_DATA  *AdapterInfo
  );

/**
  Reset the transmit ring. Ring must be stopped first to call this function.

  @param[in]   AdapterInfo        Pointer to the NIC data structure.

  @retval EFI_SUCCESS             Tx ring has been reset.
  @retval EFI_VOLUME_CORRUPTED    Tx ring was not initialized.
  @retval EFI_ACCESS_DENIED       Tx ring is still running.
  @retval EFI_INVALID_PARAMETER   Parameters were NULL/invalid.
  @retval Others                  Underlying function error.

**/
EFI_STATUS
TransmitReset (
  IN  DRIVER_DATA   *AdapterInfo
  );

/**
  Clean up Tx ring structure of LAN engine.
  This function will release all the resources used by Tx ring.

  @param[in]   AdapterInfo        Pointer to the NIC data structure.

  @retval EFI_SUCCESS             Tx ring cleaned up.
  @retval EFI_INVALID_PARAMETER   Parameters were NULL/invalid.
  @retval EFI_VOLUME_CORRUPTED    Tx ring was not initialized.
  @retval EFI_ACCESS_DENIED       Tx ring is still running.
  @retval EFI_ACCESS_DENIED       Tx ring still holds unreleased packets.
  @retval Others                  Underlying function error.

**/
EFI_STATUS
TransmitCleanup (
  IN DRIVER_DATA  *AdapterInfo
  );

/**
  Enqueue the packet in Tx queue.

  @param[in]   AdapterInfo        Pointer to the NIC data structure.
  @param[in]   Packet             Address of packet buffer
  @param[in]   PacketLength       Length of packet to be sent
  @param[in]   IsBlocking         Control whether function should wait for
                                  Tx operation completion.

  @retval EFI_SUCCESS             Packet successfully enqueued/sent.
  @retval EFI_INVALID_PARAMETER   Parameters were NULL/invalid.
  @retval EFI_VOLUME_CORRUPTED    Tx ring was not initialized.
  @retval EFI_NOT_STARTED         Tx ring was not started.
  @retval Others                  Underlying function error.

**/
EFI_STATUS
TransmitSend (
  IN  DRIVER_DATA           *AdapterInfo,
  IN  EFI_VIRTUAL_ADDRESS   Packet,
  IN  UINT16                PacketLength,
  IN  BOOLEAN               IsBlocking
  );

/**
  Traverse from NextToUnmap to NextToUse in order to find descriptors indicating
  finished Tx operation. If found, unmaps the packet buffer associated with that
  descriptor.

  @param[in]   AdapterInfo        Pointer to the NIC data structure.

  @retval EFI_SUCCESS             Some buffers were unmapped.
  @retval EFI_INVALID_PARAMETER   Parameters were NULL/invalid.
  @retval EFI_VOLUME_CORRUPTED    Tx ring was not initialized.
  @retval EFI_NOT_READY           No buffers were unmapped.
  @retval Others                  Unmap operation failure.

**/
EFI_STATUS
TransmitScanDescriptors (
  IN    DRIVER_DATA   *AdapterInfo
  );

/**
  Retrieve Tx buffer for which Tx operations were completed, from Tx ring.

  @param[in]   AdapterInfo        Pointer to the NIC data structure.
  @param[out]  Buffer             Pointer to memory to hold Tx buffer address

  @retval EFI_SUCCESS             Packet has been retrieved.
  @retval EFI_INVALID_PARAMETER   Parameters were NULL/invalid.
  @retval EFI_NOT_READY           No packets to retrieve.

**/
EFI_STATUS
TransmitReleaseBuffer (
  IN    DRIVER_DATA           *AdapterInfo,
  OUT   EFI_VIRTUAL_ADDRESS   *Buffer
  );

/** Blocking function called to assure that we are not swapped out from
   the queue while moving TX ring tail pointer.

   @param[in]   AdapterInfo   Pointer to the NIC data structure information
                              the UNDI driver is layering on
   @param[in]   Flag          Block flag

   @return   According to Flag setting (TRUE/FALSE) we're acquiring or releasing EFI lock
**/
VOID
TransmitLockIo (
  IN DRIVER_DATA    *AdapterInfo,
  IN UINT32         Flag
  );

#endif /* TRANSMIT_H_ */
