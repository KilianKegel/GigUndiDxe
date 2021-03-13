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

#ifndef RECEIVE_H_
#define RECEIVE_H_

#include "CommonDriver.h"
#include "Dma.h"

#define RECEIVE_RING_SIGNATURE       0x80865278    /* Intel vendor + 'Rx' */

typedef struct _RECEIVE_RING {
  UINT32              Signature;
  BOOLEAN             IsRunning;
  UINT8               BufferCount;
  UINT16              BufferSize;
  UNDI_DMA_MAPPING    Descriptors;
  UNDI_DMA_MAPPING    Buffers;
  UINT16              NextToUse;
} RECEIVE_RING;

/** Check whether Rx ring structure is in initialized state.

   @param[in]   ring  Rx ring pointer

   @return    TRUE if ring is initialized, FALSE otherwise
 */
#define IS_RX_RING_INITIALIZED(r)   ((r)->Signature == RECEIVE_RING_SIGNATURE)

#define MIN_ETHERNET_PACKET_LENGTH  60

/**
  Initialize Rx ring structure of LAN engine.
  This function will allocate and initialize all the necessary resources.
  It also assumes (and checks) that RX_RING_FROM_ADAPTER (AdapterInfo)
  is zeroed.

  @param[in]   AdapterInfo        Pointer to the NIC data structure.
  @param[in]   BufferCount        Number of Rx buffers to be allocated within
                                  the ring.
  @param[in]   BufferSize         Target Rx buffer size.

  @retval EFI_SUCCESS             Rx ring initialized.
  @retval EFI_INVALID_PARAMETER   Parameters were NULL/invalid.
  @retval EFI_VOLUME_CORRUPTED    Rx ring structure was not zeroed.
  @retval EFI_OUT_OF_RESOURCES    Could not allocate necessary resources.
  @retval Others                  Underlying function error.

**/
EFI_STATUS
ReceiveInitialize (
  IN DRIVER_DATA  *AdapterInfo,
  IN UINT8        BufferCount,
  IN UINT16       BufferSize
  );

/**
  Start Rx ring.

  @param[in]   AdapterInfo        Pointer to the NIC data structure.

  @retval EFI_SUCCESS             Rx ring started.
  @retval EFI_INVALID_PARAMETER   Parameters were NULL/invalid.
  @retval EFI_VOLUME_CORRUPTED    Rx ring was not initialized.
  @retval EFI_ALREADY_STARTED     Rx ring was already started.
  @retval EFI_DEVICE_ERROR        NIC operation failure.
  @retval Others                  Underlying function failure.

**/
EFI_STATUS
ReceiveStart (
  IN DRIVER_DATA  *AdapterInfo
  );

/**
  Stop Rx ring.

  @param[in]   AdapterInfo        Pointer to the NIC data structure.

  @retval EFI_SUCCESS             Rx ring stopped.
  @retval EFI_INVALID_PARAMETER   Parameters were NULL/invalid.
  @retval EFI_VOLUME_CORRUPTED    Rx ring was not initialized.
  @retval EFI_NOT_STARTED         Rx ring was already stopped.
  @retval EFI_DEVICE_ERROR        NIC operation failure.
  @retval Others                  Underlying function failure.

**/
EFI_STATUS
ReceiveStop (
  IN DRIVER_DATA  *AdapterInfo
  );

/**
  Reset the receive ring. Ring must be stopped first to call this function.

  @param[in]   AdapterInfo        Pointer to the NIC data structure.

  @retval EFI_SUCCESS             Rx ring has been reset.
  @retval EFI_VOLUME_CORRUPTED    Rx ring was not initialized.
  @retval EFI_ACCESS_DENIED       Rx ring is still running.
  @retval EFI_INVALID_PARAMETER   Parameters were NULL/invalid.
  @retval Others                  Underlying function error.

**/
EFI_STATUS
ReceiveReset (
  IN  DRIVER_DATA   *AdapterInfo
  );

/**
  Clean up Rx ring structure of LAN engine.
  This function will release all the resources used by Rx ring.

  @param[in]   AdapterInfo        Pointer to the NIC data structure.

  @retval EFI_SUCCESS             Rx ring cleaned up.
  @retval EFI_INVALID_PARAMETER   Parameters were NULL/invalid.
  @retval EFI_VOLUME_CORRUPTED    Rx ring was not initialized.
  @retval EFI_ACCESS_DENIED       Rx ring is still running.
  @retval Others                  Underlying function error.

**/
EFI_STATUS
ReceiveCleanup (
  IN DRIVER_DATA  *AdapterInfo
  );

/**
  Check whether Rx ring has a packet ready to be obtained.

  @param[in]   AdapterInfo        Pointer to the NIC data structure.
  @param[out]  PacketLength       On output, length of received packet.
  @param[out]  HeaderLength       On output, length of received packet's header.
  @param[out]  RxError            On output, descriptor's RXERROR field content.
  @param[out]  PacketType         On output, descriptor's PTYPE field content.

  @retval EFI_SUCCESS             Packet received and ready to be obtained.
  @retval EFI_NOT_READY           No packet has been received.
  @retval EFI_INVALID_PARAMETER   Parameters were NULL/invalid.
  @retval EFI_VOLUME_CORRUPTED    Rx ring was not initialized.
  @retval EFI_NOT_STARTED         Rx ring was not started.

**/
EFI_STATUS
ReceiveIsPacketReady (
  IN  DRIVER_DATA   *AdapterInfo,
  OUT UINT16        *PacketLength   OPTIONAL,
  OUT UINT16        *HeaderLength   OPTIONAL,
  OUT UINT8         *RxError        OPTIONAL,
  OUT UINT8         *PacketType     OPTIONAL
  );

/**
  Try to obtain the packet from Rx ring.
  If no buffer is provided, ring will cycle through one descriptor.
  If provided buffer cannot hold the whole packet, data that could not be
  copied to that buffer will be lost. To identify this case, PacketLength value
  can be compared with BufferSize.

  @param[in]      AdapterInfo     Pointer to the NIC data structure.
  @param[out]     Buffer          Buffer to hold received packet.
  @param[in,out]  BufferSize      On input, length of provided buffer.
                                  On output, number of bytes transferred
                                  from packet to target buffer.
  @param[out]     PacketLength    On output, full length of received packet.

  @retval EFI_SUCCESS             Packet received and ready to be obtained.
  @retval EFI_INVALID_PARAMETER   Parameters were NULL/invalid.
  @retval EFI_VOLUME_CORRUPTED    Rx ring was not initialized.
  @retval EFI_NOT_STARTED         Rx ring was not started.
  @retval EFI_NOT_READY           No packet has been received.
  @retval EFI_DEVICE_ERROR        Error has been reported via Rx descriptor.
  @retval Others                  Underlying function error.

**/
EFI_STATUS
ReceiveGetPacket (
  IN      DRIVER_DATA   *AdapterInfo,
  OUT     UINT8         *Buffer,
  IN OUT  UINT16        *BufferSize,
  OUT     UINT16        *PacketLength
  );

#endif /* RECEIVE_H_ */
