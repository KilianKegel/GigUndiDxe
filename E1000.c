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
#include "DeviceSupport.h"




/* Global variables for blocking IO */
STATIC BOOLEAN  mInitializeLock = TRUE;
STATIC EFI_LOCK mLock;

EFI_TIME gTime;





/** Checks if alternate MAC address is supported

   @param[in]   UndiPrivateData    Driver instance private data structure

   @retval   TRUE    Alternate MAC address is supported
   @retval   FALSE   Alternate MAC address is not supported
**/
BOOLEAN
IsAltMacAddrSupported (
  UNDI_PRIVATE_DATA *UndiPrivateData
  )
{
  UINT16 BackupMacPointer;

  // Check to see if the backup MAC address location pointer is set
  e1000_read_nvm (&UndiPrivateData->NicInfo.Hw, NVM_ALT_MAC_ADDR_PTR, 1, &BackupMacPointer);

  if (BackupMacPointer == 0xFFFF
    || BackupMacPointer == 0x0000)
  {
    //  Alternate Mac Address not supported if 0x37 pointer is not initialized to a value
    //  other than 0x0000 or 0xffff
    return FALSE;
  } else {
    return TRUE;
  }

}




/** Implements IO blocking when reading DMA memory.

   @param[in]   AdapterInfo   Pointer to the NIC data structure information
                              which the UNDI driver is layering on.
   @param[in]   Flag          Block flag

   @return    Lock is acquired or released according to Flag
**/
VOID
E1000BlockIt (
  IN DRIVER_DATA *AdapterInfo,
  IN UINT32       Flag
  )
{
  if (AdapterInfo->Block != NULL) {
    (*AdapterInfo->Block) (AdapterInfo->UniqueId, Flag);
  } else {
    if (mInitializeLock) {
      EfiInitializeLock (&mLock, TPL_NOTIFY);
      mInitializeLock = FALSE;
    }

    if (Flag != 0) {
      EfiAcquireLock (&mLock);
    } else {
      EfiReleaseLock (&mLock);
    }
  }
}

/** Maps a virtual address to a physical address.  This is necessary for runtime functionality and also
   on some platforms that have a chipset that cannot allow DMAs above 4GB.

   @param[in]   AdapterInfo      Pointer to the NIC data structure information
                                 which the UNDI driver is layering on.
   @param[in]   VirtualAddress   Virtual Address of the data buffer to map.
   @param[in]   Size             Minimum size of the buffer to map.
   @param[out]  MappedAddress    Pointer to store the address of the mapped buffer.

   @return    Virtual address mapped to physical
**/
VOID
E1000MapMem (
  IN  DRIVER_DATA *AdapterInfo,
  IN  UINT64       VirtualAddress,
  IN  UINT32       Size,
  OUT UINTN       *MappedAddress
  )
{
  if (AdapterInfo->MapMem != NULL) {
    (*AdapterInfo->MapMem) (
                    AdapterInfo->UniqueId,
                    VirtualAddress,
                    Size,
                    TO_DEVICE,
                    (UINT64) (UINTN) MappedAddress
                  );

    if (*MappedAddress == 0) {
      *((UINTN *) MappedAddress) = (UINTN) VirtualAddress;
    }
  } else {
    *((UINTN *) MappedAddress) = (UINTN) VirtualAddress;
  }
}

/** UnMaps a virtual address to a physical address.  This is necessary for runtime functionality and also
   on some platforms that have a chipset that cannot allow DMAs above 4GB.

   @param[in]   AdapterInfo   Pointer to the NIC data structure information
                             which the UNDI driver is layering on.
   @param[in]   VirtualAddress   Virtual Address of the data buffer to map.
   @param[in]   Size             Minimum size of the buffer to map.
   @param[in]   MappedAddress   Pointer to store the address of the mapped buffer.

   @return    Physical address unmapped
**/
VOID
E1000UnMapMem (
  IN DRIVER_DATA *AdapterInfo,
  IN UINT64       VirtualAddress,
  IN UINT32       Size,
  IN UINT64       MappedAddress
  )
{
  if (AdapterInfo->UnMapMem != NULL) {
    (*AdapterInfo->UnMapMem) (
                    AdapterInfo->UniqueId,
                    VirtualAddress,
                    Size,
                    TO_DEVICE,
                    (UINT64) MappedAddress
                  );
  }
}

/** This is the drivers copy function so it does not need to rely on the BootServices
   copy which goes away at runtime.

   This copy function allows 64-bit or 32-bit copies
   depending on platform architecture.  On Itanium we must check that both addresses
   are naturally aligned before attempting a 64-bit copy.

   @param[in]   Dest     Destination memory pointer to copy data to.
   @param[in]   Source   Source memory pointer.
   @param[in]   Count    Number of bytes to copy

   @return    Memory copied from source to destination
**/
VOID
E1000MemCopy (
  IN UINT8* Dest,
  IN UINT8* Source,
  IN UINT32 Count
  )
{
  UINT32 BytesToCopy;
  UINT32 IntsToCopy;
  UINTN* SourcePtr;
  UINTN* DestPtr;
  UINT8* SourceBytePtr;
  UINT8* DestBytePtr;

  IntsToCopy = Count / sizeof (UINTN);
  BytesToCopy = Count % sizeof (UINTN);

  SourcePtr = (UINTN *) Source;
  DestPtr = (UINTN *) Dest;

  while (IntsToCopy > 0) {
    *DestPtr = *SourcePtr;
    SourcePtr++;
    DestPtr++;
    IntsToCopy--;
  }

  // Copy the leftover bytes.
  SourceBytePtr = (UINT8 *) SourcePtr;
  DestBytePtr = (UINT8 *) DestPtr;
  while (BytesToCopy > 0) {
    *DestBytePtr = *SourceBytePtr;
    SourceBytePtr++;
    DestBytePtr++;
    BytesToCopy--;
  }
}

/** Wait for up to 15 seconds for two pair downshift to complete

   @param[in]   AdapterInfo   Pointer to the NIC data structure information
                             which the UNDI driver is layering on..

   @retval   TRUE    Two pair downshift was successful and link was established
   @retval   FALSE   Otherwise
**/
BOOLEAN
E1000DownShift (
  IN DRIVER_DATA *AdapterInfo
  )
{
  UINTN  i;
  UINT32 Status;

  DEBUGPRINT (E1000, ("E1000DownShift: Attempting downshift\n"));

  i = 0;
  for (i = 0; i < 15; i++) {
    DELAY_IN_MILLISECONDS (1000);
    Status = E1000_READ_REG (&AdapterInfo->Hw, E1000_STATUS);
    DEBUGPRINT (E1000, ("Status = %x\n", Status));
    if ((Status & E1000_STATUS_LU) != 0) {
      DEBUGPRINT (E1000, ("Successfully established link\n"));
      return TRUE;
    }
  }

  return FALSE;
}

/** Copies the stats from our local storage to the protocol storage.

   It means it will read our read and clear numbers, so some adding is required before
   we copy it over to the protocol.

   @param[in]   AdapterInfo   Pointer to the NIC data structure information
                             which the UNDI driver is layering on..
   @param[in]   DbAddr   The data Block address
   @param[in]   DbSize   The data Block size

   @retval   PXE_STATCODE_SUCCESS  Statistics copied successfully
**/
UINTN
E1000Statistics (
  IN DRIVER_DATA *AdapterInfo,
  IN UINT64       DbAddr,
  IN UINT16       DbSize
  )
{
  PXE_DB_STATISTICS *    DbPtr;
  struct e1000_hw *      Hw;
  struct e1000_hw_stats *St;
  UINTN                  Stat;

  Hw  = &AdapterInfo->Hw;
  St  = &AdapterInfo->Stats;

  {
    UPDATE_OR_RESET_STAT (crcerrs, E1000_CRCERRS);
  }

  UPDATE_OR_RESET_STAT (gprc, E1000_GPRC);
  UPDATE_OR_RESET_STAT (bprc, E1000_BPRC);
  UPDATE_OR_RESET_STAT (mprc, E1000_MPRC);
  UPDATE_OR_RESET_STAT (roc, E1000_ROC);
  UPDATE_OR_RESET_STAT (prc64, E1000_PRC64);
  UPDATE_OR_RESET_STAT (prc127, E1000_PRC127);
  UPDATE_OR_RESET_STAT (prc255, E1000_PRC255);
  UPDATE_OR_RESET_STAT (prc511, E1000_PRC511);
  UPDATE_OR_RESET_STAT (prc1023, E1000_PRC1023);
  UPDATE_OR_RESET_STAT (prc1522, E1000_PRC1522);

  UPDATE_OR_RESET_STAT (symerrs, E1000_SYMERRS);
  UPDATE_OR_RESET_STAT (mpc, E1000_MPC);
  UPDATE_OR_RESET_STAT (scc, E1000_SCC);
  UPDATE_OR_RESET_STAT (ecol, E1000_ECOL);
  UPDATE_OR_RESET_STAT (mcc, E1000_MCC);
  UPDATE_OR_RESET_STAT (latecol, E1000_LATECOL);
  UPDATE_OR_RESET_STAT (dc, E1000_DC);
  UPDATE_OR_RESET_STAT (sec, E1000_SEC);
  UPDATE_OR_RESET_STAT (rlec, E1000_RLEC);
  UPDATE_OR_RESET_STAT (xonrxc, E1000_XONRXC);
  UPDATE_OR_RESET_STAT (xontxc, E1000_XONTXC);
  UPDATE_OR_RESET_STAT (xoffrxc, E1000_XOFFRXC);
  UPDATE_OR_RESET_STAT (xofftxc, E1000_XOFFTXC);
  UPDATE_OR_RESET_STAT (fcruc, E1000_FCRUC);
  UPDATE_OR_RESET_STAT (gptc, E1000_GPTC);
  UPDATE_OR_RESET_STAT (rnbc, E1000_RNBC);
  UPDATE_OR_RESET_STAT (ruc, E1000_RUC);
  UPDATE_OR_RESET_STAT (rfc, E1000_RFC);
  UPDATE_OR_RESET_STAT (rjc, E1000_RJC);
  UPDATE_OR_RESET_STAT (tpr, E1000_TPR);
  UPDATE_OR_RESET_STAT (ptc64, E1000_PTC64);
  UPDATE_OR_RESET_STAT (ptc127, E1000_PTC127);
  UPDATE_OR_RESET_STAT (ptc255, E1000_PTC255);
  UPDATE_OR_RESET_STAT (ptc511, E1000_PTC511);
  UPDATE_OR_RESET_STAT (ptc1023, E1000_PTC1023);
  UPDATE_OR_RESET_STAT (ptc1522, E1000_PTC1522);
  UPDATE_OR_RESET_STAT (mptc, E1000_MPTC);
  UPDATE_OR_RESET_STAT (bptc, E1000_BPTC);

  // used for adaptive IFS
  Hw->mac.tx_packet_delta = E1000_READ_REG (Hw, E1000_TPT);
  St->tpt                 = DbAddr ? St->tpt + Hw->mac.tx_packet_delta : 0;
  Hw->mac.collision_delta = E1000_READ_REG (Hw, E1000_COLC);
  St->colc                = DbAddr ? St->colc + Hw->mac.collision_delta : 0;

  {
    UPDATE_OR_RESET_STAT (algnerrc, E1000_ALGNERRC);
    UPDATE_OR_RESET_STAT (rxerrc, E1000_RXERRC);
    UPDATE_OR_RESET_STAT (tncrs, E1000_TNCRS);
    UPDATE_OR_RESET_STAT (cexterr, E1000_CEXTERR);
    UPDATE_OR_RESET_STAT (tsctc, E1000_TSCTC);
    UPDATE_OR_RESET_STAT (tsctfc, E1000_TSCTFC);
  }

  if (!DbAddr) {
    return PXE_STATCODE_SUCCESS;
  }

  DbPtr = (PXE_DB_STATISTICS *) (UINTN) DbAddr;

  // Fill out the OS statistics structure
  // To Add/Subtract stats, include/delete the lines in pairs.
  // E.g., adding a new stat would entail adding these two lines:
  // stat = PXE_STATISTICS_NEW_STAT_XXX;         SET_SUPPORT;
  //     DbPtr->Data[stat] = st->xxx;
  DbPtr->Supported = 0;

  {
    UPDATE_EFI_STAT (RX_TOTAL_FRAMES, tpr);
  }

  UPDATE_EFI_STAT (RX_GOOD_FRAMES, gprc);
  UPDATE_EFI_STAT (RX_UNDERSIZE_FRAMES, ruc);
  UPDATE_EFI_STAT (RX_OVERSIZE_FRAMES, roc);
  UPDATE_EFI_STAT (RX_DROPPED_FRAMES, rnbc);
  SET_SUPPORT (RX_UNICAST_FRAMES);
  DbPtr->Data[Stat] = (St->gprc - St->bprc - St->mprc);
  UPDATE_EFI_STAT (RX_BROADCAST_FRAMES, bprc);
  UPDATE_EFI_STAT (RX_MULTICAST_FRAMES, mprc);
  SET_SUPPORT (RX_CRC_ERROR_FRAMES);
  DbPtr->Data[Stat] = (St->crcerrs + St->algnerrc);
  UPDATE_EFI_STAT (TX_TOTAL_FRAMES, tpt);
  UPDATE_EFI_STAT (TX_GOOD_FRAMES, gptc);
  SET_SUPPORT (TX_UNICAST_FRAMES);
  DbPtr->Data[Stat] = (St->gptc - St->bptc - St->mptc);
  UPDATE_EFI_STAT (TX_BROADCAST_FRAMES, bptc);
  UPDATE_EFI_STAT (TX_MULTICAST_FRAMES, mptc);
  UPDATE_EFI_STAT (COLLISIONS, colc);

  return PXE_STATCODE_SUCCESS;
}

/** Takes a command Block pointer (cpb) and sends the frame.  Takes either one fragment or many
   and places them onto the wire.  Cleanup of the send happens in the function UNDI_Status in DECODE.C

   @param[in]   AdapterInfo   Pointer to the instance data
   @param[in]   Cpb       The command parameter Block address.  64 bits since this is Itanium(tm)
                          processor friendly
   @param[in]   OpFlags   The operation flags, tells if there is any special sauce on this transmit

   @retval   PXE_STATCODE_SUCCESS        If the frame goes out
   @retval   PXE_STATCODE_QUEUE_FULL     Transmit buffers aren't freed by upper layer
   @retval   PXE_STATCODE_DEVICE_FAILURE Frame failed to go out
   @retval   PXE_STATCODE_BUSY           If they need to call again later
**/
UINTN
E1000Transmit (
  IN DRIVER_DATA *AdapterInfo,
  IN UINT64       Cpb,
  IN UINT16       OpFlags
  )
{
  PXE_CPB_TRANSMIT_FRAGMENTS *TxFrags;
  PXE_CPB_TRANSMIT           *TxBuffer;
  E1000_TRANSMIT_DESCRIPTOR  *TransmitDescriptor;
  UINT32                      i;
  INT16                       WaitMsec;
  EFI_STATUS                  Status;
  UNDI_DMA_MAPPING           *TxBufMapping;
#if (DBG_LVL & TX)
  UINT8                      *UnmappedTxBuffer;
#endif /* (DBG_LVL & TX) */

  TxBufMapping = &AdapterInfo->TxBufferMappings[AdapterInfo->CurTxInd];



  // Transmit buffers must be freed by the upper layer before we can transmit any more.
  if (TxBufMapping->PhysicalAddress != 0) {
    DEBUGPRINT (CRITICAL, ("TX buffers have all been used! cur_tx=%d\n", AdapterInfo->CurTxInd));
    for (i = 0; i < DEFAULT_TX_DESCRIPTORS; i++) {
      DEBUGPRINT (CRITICAL, ("%x ", AdapterInfo->TxBufferMappings[i]));
    }
    DEBUGWAIT (CRITICAL);

    // According to UEFI spec we should return PXE_STATCODE_BUFFER_FULL,
    // but SNP is not implemented to recognize this callback.
    return PXE_STATCODE_QUEUE_FULL;
  }

  // Make some short cut pointers so we don't have to worry about typecasting later.
  // If the TX has fragments we will use the
  // tx_tpr_f pointer, otherwise the tx_ptr_l (l is for linear)
  TxBuffer  = (PXE_CPB_TRANSMIT *) (UINTN) Cpb;
  TxFrags   = (PXE_CPB_TRANSMIT_FRAGMENTS *) (UINTN) Cpb;

  // quicker pointer to the next available Tx descriptor to use.
  TransmitDescriptor = E1000_TX_DESC (&AdapterInfo->TxRing, AdapterInfo->CurTxInd);

  // Opflags will tell us if this Tx has fragments
  // So far the linear case (the no fragments case, the else on this if) is the majority
  // of all frames sent.
  if (OpFlags & PXE_OPFLAGS_TRANSMIT_FRAGMENTED) {

    // this count cannot be more than 8;
    DEBUGPRINT (E1000, ("Fragments %x\n", TxFrags->FragCnt));

    // for each fragment, give it a descriptor, being sure to keep track of the number used.
    for (i = 0; i < TxFrags->FragCnt; i++) {

      // Put the size of the fragment in the descriptor
      E1000MapMem (
        AdapterInfo,
        TxFrags->FragDesc[i].FragAddr,
        TxFrags->FragDesc[i].FragLen,
        (UINTN *) &TransmitDescriptor->buffer_addr
      );

#if (DBG_LVL & TX)
      UnmappedTxBuffer = (UINT8 *) TxFrags->FragDesc[i].FragAddr;
#endif /* (DBG_LVL & TX) */

      TransmitDescriptor->lower.flags.length  = (UINT16) TxFrags->FragDesc[i].FragLen;
      TransmitDescriptor->lower.data = (E1000_TXD_CMD_IFCS | E1000_TXD_CMD_RS);


      // If this is the last fragment we must also set the EOP bit
      if ((i + 1) == TxFrags->FragCnt) {
        TransmitDescriptor->lower.data |= E1000_TXD_CMD_EOP;
      }

      // move our software counter passed the frame we just used, watching for wrapping
      DEBUGPRINT (E1000, ("Advancing TX pointer %x\n", AdapterInfo->CurTxInd));
      AdapterInfo->CurTxInd++;
      if (AdapterInfo->CurTxInd >= DEFAULT_TX_DESCRIPTORS) {
        AdapterInfo->CurTxInd = 0;
      }
      TransmitDescriptor = E1000_TX_DESC (&AdapterInfo->TxRing, AdapterInfo->CurTxInd);
    }
  } else {
    DEBUGPRINT (E1000, ("No Fragments\n"));

    TxBufMapping->UnmappedAddress = TxBuffer->FrameAddr;
    TxBufMapping->Size = TxBuffer->DataLen + TxBuffer->MediaheaderLen;

    // Make the Tx buffer accessible for adapter over DMA
    Status = UndiDmaMapMemoryRead (
               AdapterInfo->PciIo,
               TxBufMapping
               );

    if (EFI_ERROR (Status)) {
      DEBUGPRINT (CRITICAL, ("Failed to map Tx buffer: %r\n", Status));
      DEBUGWAIT (CRITICAL);
      return PXE_STATCODE_DEVICE_FAILURE;
    }

    TransmitDescriptor->buffer_addr = TxBufMapping->PhysicalAddress;
    DEBUGPRINT (E1000, ("Packet buffer at %x\n", TransmitDescriptor->buffer_addr));

#if (DBG_LVL & TX)
    UnmappedTxBuffer = (UINT8 *) TxBufMapping->UnmappedAddress;
#endif /* (DBG_LVL & TX) */

    // Set the proper bits to tell the chip that this is the last descriptor in the send,
    // and be sure to tell us when its done.
    // EOP - End of packet
    // IFCs - Insert FCS (Ethernet CRC)
    // RS - Report Status
    TransmitDescriptor->lower.data = (E1000_TXD_CMD_IFCS | E1000_TXD_CMD_RS);
    TransmitDescriptor->upper.fields.status = 0;
    TransmitDescriptor->lower.data |= E1000_TXD_CMD_EOP;
    TransmitDescriptor->lower.flags.length  = (UINT16) ((UINT16) TxBuffer->DataLen +
                                                                 TxBuffer->MediaheaderLen);

    DEBUGPRINT (E1000, ("BuffAddr=%x, ", TransmitDescriptor->buffer_addr));
    DEBUGPRINT (E1000, ("Cmd=%x,", TransmitDescriptor->lower.flags.cmd));
    DEBUGPRINT (E1000, ("Cso=%x,", TransmitDescriptor->lower.flags.cso));
    DEBUGPRINT (E1000, ("Len=%x,", TransmitDescriptor->lower.flags.length));
    DEBUGPRINT (E1000, ("Status=%x,", TransmitDescriptor->upper.fields.status));
    DEBUGPRINT (E1000, ("Special=%x,", TransmitDescriptor->upper.fields.special));
    DEBUGPRINT (E1000, ("Css=%x\n", TransmitDescriptor->upper.fields.css));

    // In the zero fragment case, we need to add the header size to the payload size
    // to accurately tell the hw how big is the packet.
    // Move our software counter passed the frame we just used, watching for wrapping
    AdapterInfo->CurTxInd++;
    if (AdapterInfo->CurTxInd >= DEFAULT_TX_DESCRIPTORS) {
      AdapterInfo->CurTxInd = 0;
    }
  }

#if (DBG_LVL & TX)
  DEBUGPRINT (TX, ("Packet length %d, data:", TransmitDescriptor->lower.flags.length));
  for (i = 0; i < TransmitDescriptor->lower.flags.length; i++) {
    if (i % 16 == 0) {
      DEBUGDUMP (TX, ("\n%p ", &UnmappedTxBuffer[i]));
    }

    DEBUGDUMP (TX, ("%.2X ", UnmappedTxBuffer[i]));
  }
  DEBUGDUMP (TX, ("\n"))
#endif /* (DBG_LVL & TX) */

  // Turn on the blocking function so we don't get swapped out
  // Then move the Tail pointer so the HW knows to start processing the TX we just setup.
  DEBUGWAIT (E1000);
  E1000BlockIt (AdapterInfo, TRUE);
  E1000_WRITE_REG (&AdapterInfo->Hw, E1000_TDT (0), AdapterInfo->CurTxInd);
  E1000BlockIt (AdapterInfo, FALSE);

  // If the OpFlags tells us to wait for the packet to hit the wire, we will wait.
  if ((OpFlags & PXE_OPFLAGS_TRANSMIT_BLOCK) != 0) {
    WaitMsec = 1000;

    while ((TransmitDescriptor->upper.fields.status & E1000_TXD_STAT_DD) == 0) {
      DELAY_IN_MILLISECONDS (10);
      WaitMsec -= 10;
      if (WaitMsec <= 0) {
        break;
      }
    }

    // If we waited for a while, and it didn't finish then the HW must be bad.
    if ((TransmitDescriptor->upper.fields.status & E1000_TXD_STAT_DD) == 0) {
      DEBUGPRINT (CRITICAL, ("ERROR: Network device transmit failure\n"));
      return PXE_STATCODE_DEVICE_FAILURE;
    } else {
      DEBUGPRINT (E1000, ("Transmit success\n"));
    }
  }

  return PXE_STATCODE_SUCCESS;
}

/** Copies the frame from our internal storage ring (As pointed to by AdapterInfo->rx_ring)
   to the command Block passed in as part of the cpb parameter.

   The flow:
   Ack the interrupt, setup the pointers, find where the last Block copied is, check to make
   sure we have actually received something, and if we have then we do a lot of work.
   The packet is checked for errors, size is adjusted to remove the CRC, adjust the amount
   to copy if the buffer is smaller than the packet, copy the packet to the EFI buffer,
   and then figure out if the packet was targetted at us, broadcast, multicast
   or if we are all promiscuous.  We then put some of the more interesting information
   (protocol, src and dest from the packet) into the db that is passed to us.
   Finally we clean up the frame, set the return value to _SUCCESS, and inc the cur_rx_ind, watching
   for wrapping.  Then with all the loose ends nicely wrapped up, fade to black and return.

   @param[in]   AdapterInfo   Pointer to the driver data
   @param[in]   Cpb           Pointer (Ia-64 friendly) to the command parameter
                              block. The frame will be placed inside of it.
   @param[out]  Db            The data buffer. The out of band method of passing
                              pre-digested information to the protocol.

   @retval   PXE_STATCODE_NO_DATA If there is no data
   @retval   PXE_STATCODE_SUCCESS If we passed the goods to the protocol.
**/
UINTN
E1000Receive (
  IN  DRIVER_DATA *AdapterInfo,
  IN  UINT64       Cpb,
  OUT UINT64       Db
  )
{
  PXE_CPB_RECEIVE *         CpbReceive;
  PXE_DB_RECEIVE *          DbReceive;
  PXE_FRAME_TYPE            PacketType;
  E1000_RECEIVE_DESCRIPTOR *ReceiveDescriptor;
  ETHER_HEADER *            EtherHeader;
  PXE_STATCODE              StatCode;
  UINT16                    DataLength;
#if (DBG_LVL & RX)
  UINT8 *                   PacketPtr;
  UINT16                    i;
#endif /* (DBG_LVL & RX) */
#if (DBG_LVL & CRITICAL) && (DBG_LVL & RX)
  VOID *                    PhysicalRxBuffer;
  UINT32                    Rdh;
  UINT32                    Rdt;
#endif /* (DBG_LVL & CRITICAL) && (DBG_LVL & RX) */

  PacketType  = PXE_FRAME_TYPE_NONE;
  StatCode    = PXE_STATCODE_NO_DATA;

  // acknowledge the interrupts
  E1000_READ_REG (&AdapterInfo->Hw, E1000_ICR);


  // DEBUGPRINT(E1000, ("E1000Receive\n"));
  // DEBUGPRINT(E1000, ("RCTL=%X ", E1000_READ_REG(&AdapterInfo->Hw, E1000_RCTL)));
  // DEBUGPRINT(E1000, ("RDH0=%x ", (UINT16) E1000_READ_REG (&AdapterInfo->Hw, E1000_RDH(0))));
  // DEBUGPRINT(E1000, ("RDT0=%x\n", (UINT16) E1000_READ_REG (&AdapterInfo->Hw, E1000_RDT(0))));
  // DEBUGPRINT(E1000, ("RDBAL=%x desc=%X\n",  E1000_READ_REG (
  //                                            &AdapterInfo->Hw, E1000_RDBAL),
  //                                            (UINTN) AdapterInfo->rx_ring)
  //                                          );
  // rar_low = E1000_READ_REG_ARRAY(&AdapterInfo->Hw, E1000_RA, 0);
  // rar_high = E1000_READ_REG_ARRAY(&AdapterInfo->Hw, E1000_RA, 1);
  // DEBUGPRINT(E1000, ("Receive Addr = %X %X\n", rar_high, rar_low));

  // for (i = 0; i < DEFAULT_RX_DESCRIPTORS; i++) {
  //   DEBUGPRINT(E1000, ("buffer=%X ", AdapterInfo->rx_ring[i].buffer_addr));
  //   DEBUGPRINT(E1000, ("csum=%X ", AdapterInfo->rx_ring[i].csum));
  //   DEBUGPRINT(E1000, ("special=%X ", AdapterInfo->rx_ring[i].special));
  //   DEBUGPRINT(E1000, ("status=%X ", AdapterInfo->rx_ring[i].status));
  //   DEBUGPRINT(E1000, ("len=%X ", AdapterInfo->rx_ring[i].length));
  //   DEBUGPRINT(E1000, ("err=%X\n", AdapterInfo->rx_ring[i].errors));
  // }


  // Make quick copies of the buffer pointers so we can use them without fear of corrupting the originals
  CpbReceive  = (PXE_CPB_RECEIVE *) (UINTN) Cpb;
  DbReceive   = (PXE_DB_RECEIVE *) (UINTN) Db;

  // Get a pointer to the buffer that should have a rx in it, IF one is really there.
  ReceiveDescriptor = E1000_RX_DESC (&AdapterInfo->RxRing, AdapterInfo->CurRxInd);

#if (DBG_LVL & CRITICAL) && (DBG_LVL & RX)
  PhysicalRxBuffer = E1000_RX_BUFFER_ADDR (
                       AdapterInfo->RxBufferMapping.PhysicalAddress,
                       AdapterInfo->CurRxInd
                     );

  if (ReceiveDescriptor->buffer_addr != PhysicalRxBuffer) {
    DEBUGPRINT (
      CRITICAL, ("RX buffer address mismatch on desc 0x%.2X: expected %X, actual %X\n",
      AdapterInfo->CurRxInd, PhysicalRxBuffer, ReceiveDescriptor->buffer_addr)
    );
  }

  Rdt = E1000_READ_REG (&AdapterInfo->Hw, E1000_RDT (0));
  Rdh = E1000_READ_REG (&AdapterInfo->Hw, E1000_RDH (0));
  if (Rdt == Rdh) {
    DEBUGPRINT (CRITICAL, ("Receive ERROR: RX Buffers Full!\n"));
  }
#endif /* (DBG_LVL & CRITICAL) && (DBG_LVL & RX) */

  if ((ReceiveDescriptor->status & (E1000_RXD_STAT_EOP | E1000_RXD_STAT_DD)) != 0) {

    // Just to make sure we don't try to copy a zero length, only copy a positive sized packet.
    if ((ReceiveDescriptor->length != 0) && (ReceiveDescriptor->errors == 0)) {

      // If the buffer passed us is smaller than the packet, only copy the size of the buffer.
      DataLength = MIN (ReceiveDescriptor->length, (UINT16) CpbReceive->BufferLen);

      // Copy the packet from our list to the EFI buffer.
      E1000MemCopy (
        (UINT8 *) (UINTN) CpbReceive->BufferAddr,
        (UINT8 *) E1000_RX_BUFFER_ADDR (
                    AdapterInfo->RxBufferMapping.UnmappedAddress,
                    AdapterInfo->CurRxInd
                  ),
        DataLength
      );

#if (DBG_LVL & RX)
      PacketPtr = (UINT8 *) (UINTN) CpbReceive->BufferAddr;

      DEBUGPRINT (RX, ("Packet Data \n"));
      for (i = 0; i < TempLen; i++) {
        DEBUGPRINT (RX, ("%x ", PacketPtr[i]));
      }
      DEBUGPRINT (RX, ("\n"));
#endif /* (DBG_LVL & RX) */

      // Fill the DB with needed information
      DbReceive->FrameLen       = ReceiveDescriptor->length;  // includes header
      DbReceive->MediaHeaderLen = PXE_MAC_HEADER_LEN_ETHER;

      EtherHeader = (ETHER_HEADER *) E1000_RX_BUFFER_ADDR (
                                       AdapterInfo->RxBufferMapping.UnmappedAddress,
                                       AdapterInfo->CurRxInd
                                     );

      // Figure out if the packet was meant for us, was a broadcast, multicast or we
      // recieved a frame in promiscuous mode.
      if (E1000_COMPARE_MAC (EtherHeader->DestAddr, AdapterInfo->Hw.mac.addr) == 0) {
        PacketType = PXE_FRAME_TYPE_UNICAST;
        DEBUGPRINT (E1000, ("unicast packet for us.\n"));
      } else if (E1000_COMPARE_MAC (EtherHeader->DestAddr, AdapterInfo->BroadcastNodeAddress) == 0) {
        PacketType = PXE_FRAME_TYPE_BROADCAST;
        DEBUGPRINT (E1000, ("broadcast packet.\n"));
      } else {

        // That leaves multicast or we must be in promiscuous mode.
        // Check for the Mcast bit in the address. otherwise its a promiscuous receive.
        if ((EtherHeader->DestAddr[0] & 1) == 1) {
          PacketType = PXE_FRAME_TYPE_MULTICAST;
          DEBUGPRINT (E1000, ("multicast packet.\n"));
        } else {
          PacketType = PXE_FRAME_TYPE_PROMISCUOUS;
          DEBUGPRINT (E1000, ("unicast promiscuous.\n"));
        }
      }
      DbReceive->Type = PacketType;
      DEBUGPRINT (E1000, ("PacketType %x\n", PacketType));

      // Put the protocol (UDP, TCP/IP) in the data buffer.
      DbReceive->Protocol = EtherHeader->Type;
      DEBUGPRINT (E1000, ("protocol %x\n", EtherHeader->Type));

      E1000_COPY_MAC (DbReceive->SrcAddr, EtherHeader->SrcAddr);
      E1000_COPY_MAC (DbReceive->DestAddr, EtherHeader->DestAddr);

      StatCode = PXE_STATCODE_SUCCESS;
    } else {
      DEBUGPRINT (CRITICAL, ("ERROR: Received zero sized packet or receive error!\n"));
    }

    // Clean up the packet
    ReceiveDescriptor->status = 0;
    ReceiveDescriptor->length = 0;

    // Move the current cleaned buffer pointer, being careful to wrap it as needed.
    // Then update the hardware, so it knows that an additional buffer can be used.
    E1000_WRITE_REG (&AdapterInfo->Hw, E1000_RDT (0), AdapterInfo->CurRxInd);
    AdapterInfo->CurRxInd++;
    if (AdapterInfo->CurRxInd == DEFAULT_RX_DESCRIPTORS) {
      AdapterInfo->CurRxInd = 0;
    }
  }

  return StatCode;
}

/** Allows the protocol to control our interrupt behaviour.

   @param[in]   AdapterInfo   Pointer to the driver structure

   @retval   PXE_STATCODE_SUCCESS   Interrupt state set successfully
**/
UINTN
E1000SetInterruptState (
  IN DRIVER_DATA *AdapterInfo
  )
{
  UINT32 SetIntMask;

  SetIntMask = 0;

  DEBUGPRINT (E1000, ("E1000SetInterruptState\n"));

  // Start with no Interrupts.
  E1000_WRITE_REG (&AdapterInfo->Hw, E1000_IMC, 0xFFFFFFFF);

  // Mask the RX interrupts
  if (AdapterInfo->IntMask & PXE_OPFLAGS_INTERRUPT_RECEIVE) {
    SetIntMask = (E1000_ICR_RXT0 |
                  E1000_ICR_RXSEQ |
                  E1000_ICR_RXDMT0 |
                  E1000_ICR_RXO |
                  E1000_ICR_RXCFG |
                  SetIntMask);
    DEBUGPRINT (E1000, ("Mask the RX interrupts\n"));
  }

  // Mask the TX interrupts
  if (AdapterInfo->IntMask & PXE_OPFLAGS_INTERRUPT_TRANSMIT) {
    SetIntMask = (E1000_ICR_TXDW | E1000_ICR_TXQE | SetIntMask);
    DEBUGPRINT (E1000, ("Mask the TX interrupts\n"));
  }

  // Now we have all the Ints we want, so let the hardware know.
  E1000_WRITE_REG (&AdapterInfo->Hw, E1000_IMS, SetIntMask);

  return PXE_STATCODE_SUCCESS;
}

/** Stop the hardware and put it all (including the PHY) into a known good state.

   @param[in]   AdapterInfo   Pointer to the driver structure

   @retval   PXE_STATCODE_SUCCESS    Hardware stopped
**/
UINTN
E1000Shutdown (
  IN DRIVER_DATA *AdapterInfo
  )
{
  UINT32 Reg;

  DEBUGPRINT (E1000, ("E1000Shutdown - adapter stop\n"));

  // Disable the transmit and receive DMA
  E1000ReceiveStop (AdapterInfo);
  E1000TransmitDisable (AdapterInfo);

  Reg = E1000_READ_REG (&AdapterInfo->Hw, E1000_TCTL);
  Reg = (Reg & ~E1000_TCTL_EN);
  E1000_WRITE_REG (&AdapterInfo->Hw, E1000_TCTL, Reg);

  // Disable the receive unit so the hardware does not continue to DMA packets to memory.
  // Also release the software semaphore.
  E1000_WRITE_REG (&AdapterInfo->Hw, E1000_RCTL, 0);
  E1000_WRITE_REG (&AdapterInfo->Hw, E1000_SWSM, 0);
  E1000PciFlush (&AdapterInfo->Hw);

  // This delay is to ensure in flight DMA and receive descriptor flush
  // have time to complete.
  DELAY_IN_MILLISECONDS (10);

  AdapterInfo->ReceiveStarted = FALSE;
  AdapterInfo->RxFilter = 0;

  return PXE_STATCODE_SUCCESS;
}

/** Resets the hardware and put it all (including the PHY) into a known good state.

   @param[in]   AdapterInfo   The pointer to our context data
   @param[in]   OpFlags      The information on what else we need to do.

   @retval   PXE_STATCODE_SUCCESS        Successfull hardware reset
   @retval   PXE_STATCODE_NOT_STARTED    Hardware init failed
**/
UINTN
E1000Reset (
  IN DRIVER_DATA *AdapterInfo,
  IN UINT16       OpFlags
  )
{
  UINT32 TempReg;

  DEBUGPRINT (E1000, ("E1000Reset\n"));

  TempReg = E1000_READ_REG (&AdapterInfo->Hw, E1000_STATUS);

  AdapterInfo->Hw.phy.reset_disable = TRUE;

  if ((TempReg & E1000_STATUS_LU) != 0) {
    if (((TempReg & E1000_STATUS_FD) == 0)
      && ((TempReg & E1000_STATUS_SPEED_MASK) == E1000_STATUS_SPEED_1000))
    {
      DEBUGPRINT (E1000, ("BAD LINK - 1Gig/Half - Enabling PHY reset\n"));
      AdapterInfo->Hw.phy.reset_disable = FALSE;

      // Since link is in a bad state we also need to make sure that we do a full reset down below
      AdapterInfo->HwInitialized = FALSE;
    }
  }

  // Put the E1000 into a known state by resetting the transmit
  // and receive units of the E1000 and masking/clearing all
  // interrupts.
  // If the hardware has already been started then don't bother with a reset
  // We want to make sure we do not have to restart autonegotiation and two-pair
  // downshift.
  if (!AdapterInfo->HwInitialized) {
    e1000_reset_hw (&AdapterInfo->Hw);

    // Now that the structures are in place, we can configure the hardware to use it all.
    if (e1000_init_hw (&AdapterInfo->Hw) == 0) {
      DEBUGPRINT (E1000, ("e1000_init_hw success\n"));
    } else {
      DEBUGPRINT (CRITICAL, ("Hardware Init failed\n"));
      return PXE_STATCODE_NOT_STARTED;
    }
  } else {
    DEBUGPRINT (E1000, ("Skipping adapter reset\n"));
  }

  if ((OpFlags & PXE_OPFLAGS_RESET_DISABLE_FILTERS) == 0) {
    UINT16 SaveFilter;

    SaveFilter = AdapterInfo->RxFilter;

    // if we give the filter same as Rx_Filter, this routine will not set mcast list
    // (it thinks there is no change)
    // to force it, we will reset that flag in the Rx_Filter
    AdapterInfo->RxFilter &= (~PXE_OPFLAGS_RECEIVE_FILTER_FILTERED_MULTICAST);
    E1000SetFilter (AdapterInfo, SaveFilter, (UINT64) 0, (UINT32) 0);
  }

  if (OpFlags & PXE_OPFLAGS_RESET_DISABLE_INTERRUPTS) {
    AdapterInfo->IntMask = 0; // disable the interrupts
  }

  E1000SetInterruptState (AdapterInfo);

  return PXE_STATCODE_SUCCESS;
}

/** PCIe function to LAN port mapping.

   @param[in,out]   AdapterInfo   Pointer to adapter structure

   @return   LAN function set accordingly
**/
VOID
E1000LanFunction (
  IN OUT DRIVER_DATA *AdapterInfo
  )
{
  AdapterInfo->LanFunction = (E1000_READ_REG (&AdapterInfo->Hw, E1000_STATUS) &
                            E1000_STATUS_LAN_ID_MASK)
                            >> E1000_STATUS_LAN_ID_OFFSET;
  DEBUGPRINT (INIT, ("PCI function %d is LAN port %d \n", AdapterInfo->Function, AdapterInfo->LanFunction));
  DEBUGWAIT (INIT);
}

/** Sets the force speed and duplex settings for the adapter based on
   EEPROM settings and data set by the SNP

   @param[in]   AdapterInfo   Pointer to adapter structure

   @return    Speed duplex settings set
**/
VOID
E1000SetSpeedDuplex (
  IN DRIVER_DATA *AdapterInfo
  )
{
  UINT16 SetupOffset;
  UINT16 ConfigOffset;
  UINT16 SetupWord;
  UINT16 CustomConfigWord;

  DEBUGPRINT (E1000, ("E1000SetSpeedDuplex\n"));

  // Copy forced speed and duplex settings to shared code structure
  if ((AdapterInfo->LinkSpeed == 10)
    && (AdapterInfo->DuplexMode == PXE_FORCE_HALF_DUPLEX))
  {
    DEBUGPRINT (E1000, ("Force 10-Half\n"));
    AdapterInfo->Hw.phy.reset_disable    = FALSE;
    AdapterInfo->Hw.mac.autoneg              = 0;
    AdapterInfo->Hw.mac.forced_speed_duplex  = ADVERTISE_10_HALF;
    AdapterInfo->HwInitialized = FALSE;
  }

  if ((AdapterInfo->LinkSpeed == 100)
    && (AdapterInfo->DuplexMode == PXE_FORCE_HALF_DUPLEX))
  {
    DEBUGPRINT (E1000, ("Force 100-Half\n"));
    AdapterInfo->Hw.phy.reset_disable    = FALSE;
    AdapterInfo->Hw.mac.autoneg              = 0;
    AdapterInfo->Hw.mac.forced_speed_duplex  = ADVERTISE_100_HALF;
    AdapterInfo->HwInitialized = FALSE;
  }

  if ((AdapterInfo->LinkSpeed == 10)
    && (AdapterInfo->DuplexMode == PXE_FORCE_FULL_DUPLEX))
  {
    DEBUGPRINT (E1000, ("Force 10-Full\n"));
    AdapterInfo->Hw.phy.reset_disable    = FALSE;
    AdapterInfo->Hw.mac.autoneg              = 0;
    AdapterInfo->Hw.mac.forced_speed_duplex  = ADVERTISE_10_FULL;
    AdapterInfo->HwInitialized = FALSE;
  }

  if ((AdapterInfo->LinkSpeed == 100)
    && (AdapterInfo->DuplexMode == PXE_FORCE_FULL_DUPLEX))
  {
    DEBUGPRINT (E1000, ("Force 100-Full\n"));
    AdapterInfo->Hw.phy.reset_disable    = FALSE;
    AdapterInfo->Hw.mac.autoneg              = 0;
    AdapterInfo->Hw.mac.forced_speed_duplex  = ADVERTISE_100_FULL;
    AdapterInfo->HwInitialized = FALSE;
  }

  // Check for forced speed and duplex
  // The EEPROM settings will override settings passed by the SNP

  // Configure offsets depending on function number
  switch (AdapterInfo->Function) {
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

  e1000_read_nvm (&AdapterInfo->Hw, SetupOffset, 1, &SetupWord);
  e1000_read_nvm (&AdapterInfo->Hw, ConfigOffset, 1, &CustomConfigWord);

  if ((CustomConfigWord & SIG_MASK) == SIG) {
    switch (SetupWord & (FSP_MASK | FDP_FULL_DUPLEX_BIT)) {
    case (FDP_FULL_DUPLEX_BIT | FSP_100MBS):
      DEBUGPRINT (E1000, ("Forcing 100 Full from EEPROM\n"));
      AdapterInfo->Hw.phy.reset_disable = FALSE;
      AdapterInfo->Hw.mac.autoneg = 0;
      AdapterInfo->Hw.mac.forced_speed_duplex = ADVERTISE_100_FULL;
      AdapterInfo->HwInitialized = FALSE;
      break;
    case (FDP_FULL_DUPLEX_BIT | FSP_10MBS):
      DEBUGPRINT (E1000, ("Forcing 10 Full from EEPROM\n"));
      AdapterInfo->Hw.phy.reset_disable = FALSE;
      AdapterInfo->Hw.mac.autoneg = 0;
      AdapterInfo->Hw.mac.forced_speed_duplex = ADVERTISE_10_FULL;
      AdapterInfo->HwInitialized = FALSE;
      break;
    case (FSP_100MBS):
      DEBUGPRINT (E1000, ("Forcing 100 Half from EEPROM\n"));
      AdapterInfo->Hw.phy.reset_disable = FALSE;
      AdapterInfo->Hw.mac.autoneg = 0;
      AdapterInfo->Hw.mac.forced_speed_duplex = ADVERTISE_100_HALF;
      AdapterInfo->HwInitialized = FALSE;
      break;
    case (FSP_10MBS):
      DEBUGPRINT (E1000, ("Forcing 10 Half from EEPROM\n"));
      AdapterInfo->Hw.phy.reset_disable = FALSE;
      AdapterInfo->Hw.mac.autoneg = 0;
      AdapterInfo->Hw.mac.forced_speed_duplex = ADVERTISE_10_HALF;
      AdapterInfo->HwInitialized = FALSE;
      break;
    default:
      AdapterInfo->Hw.mac.autoneg = 1;
      break;
    }
  }
}

/** Initializes the transmit and receive resources for the adapter.

   @param[in]   AdapterInfo   Pointer to adapter structure

   @return   TX/RX resources configured and initialized
**/
VOID
E1000TxRxConfigure (
  IN DRIVER_DATA *AdapterInfo
  )
{
  UINT32                    TempReg;
  UINT64                    MemAddr;
  UINT32                   *MemPtr;
  UINT16                    i;
  E1000_RECEIVE_DESCRIPTOR *RxDesc;
  UINT8                    *PhysicalRxBuffer;
  LOCAL_RX_BUFFER          *VirtualRxBuffer;

  DEBUGPRINT (E1000, ("E1000TxRxConfigure\n"));

  E1000ReceiveStop (AdapterInfo);

  DEBUGPRINT (
    E1000, ("Rx Ring %x Tx Ring %X  RX size %X \n",
    E1000_RX_DESC (&AdapterInfo->RxRing, 0),
    E1000_TX_DESC (&AdapterInfo->TxRing, 0),
    sizeof (E1000_RECEIVE_DESCRIPTOR) * DEFAULT_RX_DESCRIPTORS)
  );

  ZeroMem (AdapterInfo->TxBufferMappings, sizeof (AdapterInfo->TxBufferMappings));

  PhysicalRxBuffer = (UINT8 *) (UINTN) AdapterInfo->RxBufferMapping.PhysicalAddress;
  VirtualRxBuffer = (LOCAL_RX_BUFFER *) (UINTN) AdapterInfo->RxBufferMapping.UnmappedAddress;

  DEBUGPRINT (
    E1000, ("Local RX Buffer: 0x%p (Physical) / 0x%p (Virtual) / 0x%X (Size)\n",
    PhysicalRxBuffer, VirtualRxBuffer,
    sizeof (E1000_TRANSMIT_DESCRIPTOR) * DEFAULT_TX_DESCRIPTORS)
  );

  // now to link the RX Ring to the local buffers
  for (i = 0; i < DEFAULT_RX_DESCRIPTORS; i++) {
    RxDesc = E1000_RX_DESC (&AdapterInfo->RxRing, i);
    RxDesc->buffer_addr = (UINT64) E1000_RX_BUFFER_ADDR (PhysicalRxBuffer, i);
    RxDesc->status = E1000_RXD_STAT_IXSM;

    DEBUGPRINT (
      E1000, ("RX Desc 0x%.2X -> 0x%p (Physical) / 0x%p (Virtual)\n",
      i, E1000_RX_BUFFER_ADDR (PhysicalRxBuffer, i), E1000_RX_BUFFER_ADDR ((UINT8 *) VirtualRxBuffer, i))
    );
  }

  // Setup the RDBAL (lower 32 bits of the buffer addr), RDBAH (higher 32 bits)
  // EFI_PHYSICAL/VIRTUAL_ADDRESS is always UINT64. We cast it down to UINT32.
  E1000_WRITE_REG (&AdapterInfo->Hw, E1000_RDBAL (0), (UINT32) AdapterInfo->RxRing.PhysicalAddress);
  E1000_WRITE_REG (&AdapterInfo->Hw, E1000_RDBAH (0), (UINT32) (AdapterInfo->RxRing.PhysicalAddress >> 32));

  // Setup the RDLEN (amount of memory for the RX buffers that hardware can't go over)
  E1000_WRITE_REG (&AdapterInfo->Hw, E1000_RDLEN (0), (sizeof (E1000_RECEIVE_DESCRIPTOR) * DEFAULT_RX_DESCRIPTORS));

  DEBUGPRINT (E1000, ("RDBAL0 %X\n", (UINT32) E1000_READ_REG (&AdapterInfo->Hw, E1000_RDBAL (0))));
  DEBUGPRINT (E1000, ("RDBAH0 %X\n", (UINT32) E1000_READ_REG (&AdapterInfo->Hw, E1000_RDBAH (0))));
  DEBUGPRINT (E1000, ("Rx Ring @ %p (Physical)\n", AdapterInfo->RxRing.PhysicalAddress));

  // Set the transmit tail equal to the head pointer (we do not want hardware to try to
  // transmit packets yet).
  AdapterInfo->CurTxInd = (UINT16) E1000_READ_REG (&AdapterInfo->Hw, E1000_TDH (0));
  E1000_WRITE_REG (&AdapterInfo->Hw, E1000_TDT (0), AdapterInfo->CurTxInd);
  AdapterInfo->XmitDoneHead = AdapterInfo->CurTxInd;

  AdapterInfo->CurRxInd = (UINT16) E1000_READ_REG (&AdapterInfo->Hw, E1000_RDH (0));
  E1000_WRITE_REG (&AdapterInfo->Hw, E1000_RDT (0), AdapterInfo->CurRxInd);

  if (AdapterInfo->Hw.mac.type != e1000_82575 &&
    AdapterInfo->Hw.mac.type != e1000_82576 &&
    AdapterInfo->Hw.mac.type != e1000_82580
    && AdapterInfo->Hw.mac.type != e1000_i350
    && AdapterInfo->Hw.mac.type  != e1000_i354
    && AdapterInfo->Hw.mac.type != e1000_i210
    && AdapterInfo->Hw.mac.type != e1000_i211
    )
  {
    E1000_WRITE_REG (&AdapterInfo->Hw, E1000_SRRCTL (0), E1000_SRRCTL_DESCTYPE_LEGACY);

    E1000SetRegBits (AdapterInfo, E1000_RXDCTL (0), E1000_RXDCTL_QUEUE_ENABLE);
    i = 0;
    do {
      TempReg = E1000_READ_REG (&AdapterInfo->Hw, E1000_RXDCTL (0));
      i++;
      if ((TempReg & E1000_RXDCTL_QUEUE_ENABLE) != 0) {
        DEBUGPRINT (E1000, ("RX queue enabled, after attempt i = %d\n", i));
        break;
      }

      DelayInMicroseconds (AdapterInfo, 1);
    } while (i < 1000);

    if (i >= 1000) {
      DEBUGPRINT (CRITICAL, ("Enable RX queue failed!\n"));
    }
  }


#ifndef NO_82575_SUPPORT
  if (AdapterInfo->Hw.mac.type != e1000_82575 &&
    AdapterInfo->Hw.mac.type != e1000_82576 &&
    AdapterInfo->Hw.mac.type != e1000_82580
    && AdapterInfo->Hw.mac.type != e1000_i350
    && AdapterInfo->Hw.mac.type  != e1000_i354
    && AdapterInfo->Hw.mac.type != e1000_i210
    && AdapterInfo->Hw.mac.type != e1000_i211
    )
#endif /* NO_82575_SUPPORT */
  {
    // Set the software tail pointer just behind head to give hardware the entire ring
    if (AdapterInfo->CurRxInd == 0) {
      E1000_WRITE_REG (&AdapterInfo->Hw, E1000_RDT (0), DEFAULT_RX_DESCRIPTORS - 1);
    } else {
      E1000_WRITE_REG (&AdapterInfo->Hw, E1000_RDT (0), AdapterInfo->CurRxInd - 1);
    }
  }

  // Zero out PSRCTL to use default packet size settings in RCTL
  E1000_WRITE_REG (&AdapterInfo->Hw, E1000_PSRCTL, 0);
  E1000_WRITE_REG (&AdapterInfo->Hw, E1000_MRQC, 0);

  E1000_WRITE_REG (&AdapterInfo->Hw, E1000_TDBAL (0), (UINT32) (UINTN) (AdapterInfo->TxRing.PhysicalAddress));
  MemAddr = (UINT64) (UINTN) AdapterInfo->TxRing.PhysicalAddress;
  MemPtr  = (UINT32 *) &MemAddr;
  MemPtr++;
  E1000_WRITE_REG (&AdapterInfo->Hw, E1000_TDBAH (0), *MemPtr);
  DEBUGPRINT (E1000, ("TdBah0 %X\n", *MemPtr));
  DEBUGWAIT (E1000);
  E1000_WRITE_REG (
    &AdapterInfo->Hw,
    E1000_TDLEN (0),
    (sizeof (E1000_TRANSMIT_DESCRIPTOR) * DEFAULT_TX_DESCRIPTORS)
  );

  if (AdapterInfo->Hw.mac.type == e1000_82580
    || AdapterInfo->Hw.mac.type == e1000_i350
    || AdapterInfo->Hw.mac.type  == e1000_i354
    || AdapterInfo->Hw.mac.type == e1000_i210
    || AdapterInfo->Hw.mac.type == e1000_i211
    )
  {
    E1000SetRegBits (AdapterInfo, E1000_TXDCTL (0), E1000_TXDCTL_QUEUE_ENABLE);

    for (i = 0; i < 1000; i++) {
      TempReg = E1000_READ_REG (&AdapterInfo->Hw, E1000_TXDCTL (0));
      if ((TempReg & E1000_TXDCTL_QUEUE_ENABLE) != 0) {
        DEBUGPRINT (E1000, ("TX queue enabled, after attempt i = %d\n", i));
        break;
      }

      DelayInMicroseconds (AdapterInfo, 1);
    }
    if (i >= 1000) {
      DEBUGPRINT (CRITICAL, ("Enable TX queue failed!\n"));
    }
    AdapterInfo->CurTxInd = 0;
    AdapterInfo->XmitDoneHead = 0;
  }


  TempReg = E1000_READ_REG (&AdapterInfo->Hw, E1000_TCTL);
  TempReg = (TempReg | E1000_TCTL_EN | E1000_TCTL_PSP);
  E1000_WRITE_REG (&AdapterInfo->Hw, E1000_TCTL, TempReg);

  E1000PciFlush (&AdapterInfo->Hw);
}
/** This function performs PCI-E initialization for the device.

   @param[in]   AdapterInfo   Pointer to adapter structure

   @retval   EFI_SUCCESS            PCI-E initialized successfully
   @retval   EFI_UNSUPPORTED        Failed to get supported PCI command options
   @retval   EFI_UNSUPPORTED        Failed to set PCI command options
   @retval   EFI_OUT_OF_RESOURCES   The memory pages for transmit and receive resources could
                                    not be allocated
**/
EFI_STATUS
E1000PciInit (
  IN DRIVER_DATA *AdapterInfo
  )
{
  EFI_STATUS Status;
  UINT64     Result = 0;
  BOOLEAN    PciAttributesSaved = FALSE;

  // Save original PCI attributes
  Status = AdapterInfo->PciIo->Attributes (
                                AdapterInfo->PciIo,
                                EfiPciIoAttributeOperationGet,
                                0,
                                &AdapterInfo->OriginalPciAttributes
                               );

  if (EFI_ERROR (Status)) {
    goto PciIoError;
  }
  PciAttributesSaved = TRUE;

  // Get the PCI Command options that are supported by this controller.
  Status = AdapterInfo->PciIo->Attributes (
                                AdapterInfo->PciIo,
                                EfiPciIoAttributeOperationSupported,
                                0,
                                &Result
                              );

  DEBUGPRINT (INIT, ("Attributes supported %x\n", Result));

  if (!EFI_ERROR (Status)) {

    // Set the PCI Command options to enable device memory mapped IO,
    // port IO, and bus mastering.
    Status = AdapterInfo->PciIo->Attributes (
                                  AdapterInfo->PciIo,
                                  EfiPciIoAttributeOperationEnable,
                                  Result & (EFI_PCI_DEVICE_ENABLE |
                                            EFI_PCI_IO_ATTRIBUTE_DUAL_ADDRESS_CYCLE),
                                  NULL
                                );
  }

  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("Attributes returns %r\n", Status));
    goto PciIoError;
  }

  // Allocate common DMA buffer for Tx descriptors
  AdapterInfo->TxRing.Size = TX_RING_SIZE;

  Status = UndiDmaAllocateCommonBuffer (
             AdapterInfo->PciIo,
             &AdapterInfo->TxRing
             );

  if (EFI_ERROR (Status)) {
    goto OnAllocError;
  }

  // Allocate common DMA buffer for Rx descriptors
  AdapterInfo->RxRing.Size = RX_RING_SIZE;

  Status = UndiDmaAllocateCommonBuffer (
             AdapterInfo->PciIo,
             &AdapterInfo->RxRing
             );

  if (EFI_ERROR (Status)) {
    goto OnAllocError;
  }

  // Allocate common DMA buffer for Rx buffers
  AdapterInfo->RxBufferMapping.Size = RX_BUFFERS_SIZE;

  Status = UndiDmaAllocateCommonBuffer (
             AdapterInfo->PciIo,
             &AdapterInfo->RxBufferMapping
             );

  if (EFI_ERROR (Status)) {
    goto OnAllocError;
  }

  return EFI_SUCCESS;

OnAllocError:
      if (AdapterInfo->TxRing.Mapping != NULL) {
        UndiDmaFreeCommonBuffer (AdapterInfo->PciIo, &AdapterInfo->TxRing);
      }

      if (AdapterInfo->RxRing.Mapping != NULL) {
        UndiDmaFreeCommonBuffer (AdapterInfo->PciIo, &AdapterInfo->RxRing);
      }

      if (AdapterInfo->RxBufferMapping.Mapping != NULL) {
        UndiDmaFreeCommonBuffer (AdapterInfo->PciIo, &AdapterInfo->RxBufferMapping);
      }

PciIoError:
  if (PciAttributesSaved) {

    // Restore original PCI attributes
    AdapterInfo->PciIo->Attributes (
                         AdapterInfo->PciIo,
                         EfiPciIoAttributeOperationSet,
                         AdapterInfo->OriginalPciAttributes,
                         NULL
                       );
  }

    return Status;
}
/** This function is called as early as possible during driver start to ensure the
   hardware has enough time to autonegotiate when the real SNP device initialize call is made.

   @param[in]   AdapterInfo   Pointer to adapter structure

   @retval   EFI_SUCCESS        Hardware init success
   @retval   EFI_DEVICE_ERROR   Hardware init failed
   @retval   EFI_UNSUPPORTED    Unsupported MAC type
   @retval   EFI_UNSUPPORTED    e1000_setup_init_funcs failed
   @retval   EFI_UNSUPPORTED    Could not read bus information
   @retval   EFI_UNSUPPORTED    Could not read MAC address
   @retval   EFI_ACCESS_DENIED  iSCSI Boot detected on port
   @retval   EFI_DEVICE_ERROR   Failed to reset hardware
**/
EFI_STATUS
E1000FirstTimeInit (
  IN DRIVER_DATA *AdapterInfo
  )
{
  PCI_CONFIG_HEADER *PciConfigHeader;
  UINT32 *           TempBar;
  UINT8              BarIndex;
  EFI_STATUS         Status;
  UINT32             ScStatus;
  UINT32             Reg;
  UINT16             i;

  DEBUGPRINT (E1000, ("E1000FirstTimeInit\n"));

  AdapterInfo->DriverBusy = FALSE;

  // Read all the registers from the device's PCI Configuration space
  AdapterInfo->PciIo->Pci.Read (
                           AdapterInfo->PciIo,
                           EfiPciIoWidthUint32,
                           0,
                           MAX_PCI_CONFIG_LEN,
                           AdapterInfo->PciConfig
                         );

  PciConfigHeader = (PCI_CONFIG_HEADER *) AdapterInfo->PciConfig;

  // Enumerate through the PCI BARs for the device to determine which one is
  // the IO BAR.  Save the index of the BAR into the adapter info structure.
  TempBar = &PciConfigHeader->BaseAddressReg0;
  for (BarIndex = 0; BarIndex <= 5; BarIndex++) {
    DEBUGPRINT (E1000, ("BAR = %X\n", *TempBar));
    if ((*TempBar & PCI_BAR_MEM_MASK) == PCI_BAR_MEM_64BIT) {

      // This is a 64-bit memory bar, skip this and the
      // next bar as well.
      TempBar++;
    }

    // Find the IO BAR and save it's number into IoBar
    if ((*TempBar & PCI_BAR_IO_MASK) == PCI_BAR_IO_MODE) {

      // Here is the IO Bar - save it to the Gigabit adapter struct.
      AdapterInfo->IoBarIndex = BarIndex;
      break;
    }

    // Advance the pointer to the next bar in PCI config space
    TempBar++;
  }

  AdapterInfo->PciIo->GetLocation (
                       AdapterInfo->PciIo,
                       &AdapterInfo->Segment,
                       &AdapterInfo->Bus,
                       &AdapterInfo->Device,
                       &AdapterInfo->Function
                     );

  DEBUGPRINT (INIT, ("AdapterInfo->IoBarIndex = %X\n", AdapterInfo->IoBarIndex));
  DEBUGPRINT (INIT, ("PCI Command Register = %X\n", PciConfigHeader->Command));
  DEBUGPRINT (INIT, ("PCI Status Register = %X\n", PciConfigHeader->Status));
  DEBUGPRINT (INIT, ("PCI VendorID = %X\n", PciConfigHeader->VendorId));
  DEBUGPRINT (INIT, ("PCI DeviceID = %X\n", PciConfigHeader->DeviceId));
  DEBUGPRINT (INIT, ("PCI SubVendorID = %X\n", PciConfigHeader->SubVendorId));
  DEBUGPRINT (INIT, ("PCI SubSystemID = %X\n", PciConfigHeader->SubSystemId));
  DEBUGPRINT (INIT, ("PCI Segment = %X\n", AdapterInfo->Segment));
  DEBUGPRINT (INIT, ("PCI Bus = %X\n", AdapterInfo->Bus));
  DEBUGPRINT (INIT, ("PCI Device = %X\n", AdapterInfo->Device));
  DEBUGPRINT (INIT, ("PCI Function = %X\n", AdapterInfo->Function));

  ZeroMem (AdapterInfo->BroadcastNodeAddress, PXE_MAC_LENGTH);
  SetMem (AdapterInfo->BroadcastNodeAddress, PXE_HWADDR_LEN_ETHER, 0xFF);

  // Initialize all parameters needed for the shared code
  AdapterInfo->Hw.hw_addr                = (UINT8 *) (UINTN) PciConfigHeader->BaseAddressReg0;
  AdapterInfo->Hw.back                   = AdapterInfo;
  AdapterInfo->Hw.vendor_id              = PciConfigHeader->VendorId;
  AdapterInfo->Hw.device_id              = PciConfigHeader->DeviceId;
  AdapterInfo->Hw.subsystem_vendor_id    = PciConfigHeader->SubVendorId;
  AdapterInfo->Hw.subsystem_device_id    = PciConfigHeader->SubSystemId;
  AdapterInfo->Hw.revision_id            = PciConfigHeader->RevId;

  AdapterInfo->Hw.mac.autoneg            = TRUE;
  AdapterInfo->Hw.fc.current_mode        = e1000_fc_full;
  AdapterInfo->Hw.fc.requested_mode      = e1000_fc_full;

  AdapterInfo->Hw.phy.autoneg_wait_to_complete = FALSE;
  AdapterInfo->Hw.phy.reset_disable      = FALSE;
  AdapterInfo->Hw.phy.autoneg_advertised = E1000_ALL_SPEED_DUPLEX;
  AdapterInfo->Hw.phy.autoneg_mask       = AUTONEG_ADVERTISE_SPEED_DEFAULT;

  AdapterInfo->PciClass       = PciConfigHeader->ClassIdMain;
  AdapterInfo->PciSubClass    = PciConfigHeader->ClassIdSubclass;
  AdapterInfo->PciClassProgIf = PciConfigHeader->ClassIdProgIf;

  // We need to set the IO bar to zero for the shared code because the EFI PCI protocol
  // gets the BAR for us.
  AdapterInfo->Hw.io_base               = 0;

  //  This variable is set only to make the flash shared code work on ICH8.
  //  Set to 1 because the flash BAR will always be BAR 1.
  AdapterInfo->Hw.flash_address         = (UINT8 *) ((UINTN) 1);

  if (e1000_set_mac_type (&AdapterInfo->Hw) != E1000_SUCCESS) {
    DEBUGPRINT (CRITICAL, ("Unsupported MAC type!\n"));
    return EFI_UNSUPPORTED;
  }

  if (e1000_setup_init_funcs (&AdapterInfo->Hw, TRUE) != E1000_SUCCESS) {
    DEBUGPRINT (CRITICAL, ("e1000_setup_init_funcs failed!\n"));
    return EFI_UNSUPPORTED;
  }

  E1000LanFunction (AdapterInfo);


  DEBUGPRINT (E1000, ("Calling e1000_get_bus_info\n"));
  if (e1000_get_bus_info (&AdapterInfo->Hw) != E1000_SUCCESS) {
    DEBUGPRINT (CRITICAL, ("Could not read bus information\n"));
    return EFI_UNSUPPORTED;
  }

  DEBUGPRINT (E1000, ("Calling e1000_read_mac_addr\n"));
  if (e1000_read_mac_addr (&AdapterInfo->Hw) != E1000_SUCCESS) {
    DEBUGPRINT (CRITICAL, ("Could not read MAC address\n"));
    return EFI_UNSUPPORTED;
  }

  DEBUGPRINT (INIT, ("MAC Address: "));
  for (i = 0; i < 6; i++) {
    DEBUGPRINT (INIT, ("%2x ", AdapterInfo->Hw.mac.perm_addr[i]));
  }
  DEBUGPRINT (INIT, ("\n"));

  Reg = E1000_READ_REG (&AdapterInfo->Hw, E1000_CTRL_EXT);
  if ((Reg & E1000_CTRL_EXT_DRV_LOAD) != 0) {
    DEBUGPRINT (CRITICAL, ("iSCSI Boot detected on port!\n"));
    return EFI_ACCESS_DENIED;
  }


  ScStatus = e1000_reset_hw (&AdapterInfo->Hw);
  if (ScStatus != E1000_SUCCESS) {
    DEBUGPRINT (CRITICAL, ("e1000_reset_hw returns %d\n", ScStatus));
    return EFI_DEVICE_ERROR;
  }

  // Now that the structures are in place, we can configure the hardware to use it all.
  ScStatus = e1000_init_hw (&AdapterInfo->Hw);
  if (ScStatus == E1000_SUCCESS) {
    DEBUGPRINT (E1000, ("e1000_init_hw success\n"));
    Status = EFI_SUCCESS;
    AdapterInfo->HwInitialized = TRUE;
  } else {
    DEBUGPRINT (CRITICAL, ("Hardware Init failed status=%x\n", ScStatus));
    AdapterInfo->HwInitialized = FALSE;
    Status = EFI_DEVICE_ERROR;
  }


  E1000_WRITE_REG (&AdapterInfo->Hw, E1000_RDH (0), 0);
  E1000_WRITE_REG (&AdapterInfo->Hw, E1000_TDH (0), 0);
  AdapterInfo->CurTxInd = 0;
  AdapterInfo->XmitDoneHead = 0;
  AdapterInfo->CurRxInd = 0;

#ifndef NO_82571_SUPPORT

  // On 82571 based adapters if either port is reset then the
  // MAC address will be loaded into the EEPROM If the user overrides the default MAC
  // address using the StnAddr command then the 82571 will reset the MAC address
  // the next time either port is reset.  This check resets the MAC
  // address to the default value specified by the user.
  if (AdapterInfo->Hw.mac.type == e1000_82571
    && AdapterInfo->MacAddrOverride)
  {
    DEBUGPRINT (E1000, ("RESETING STATION ADDRESS\n"));
    e1000_rar_set (&AdapterInfo->Hw, AdapterInfo->Hw.mac.addr, 0);
  }
#endif /* NO_82571_SUPPORT */

  Reg = E1000_READ_REG (&AdapterInfo->Hw, E1000_CTRL_EXT);
  Reg |= E1000_CTRL_EXT_DRV_LOAD;
  E1000_WRITE_REG (&AdapterInfo->Hw, E1000_CTRL_EXT, Reg);

  return Status;
}

/** Initializes the gigabit adapter, setting up memory addresses, MAC Addresses,
   Type of card, etc.

   @param[in]   AdapterInfo   Pointer to adapter structure

   @retval   PXE_STATCODE_SUCCESS       Initialization succeeded
   @retval   PXE_STATCODE_NOT_STARTED   Hardware Init failed
**/
PXE_STATCODE
E1000Inititialize (
  IN DRIVER_DATA *AdapterInfo
  )
{
  PXE_STATCODE PxeStatcode = PXE_STATCODE_SUCCESS;

  DEBUGPRINT (E1000, ("E1000Inititialize\n"));

  ZeroMem (
    (VOID *) (UINTN) AdapterInfo->RxRing.UnmappedAddress,
    RX_RING_SIZE
    );

  ZeroMem (
    (VOID *) (UINTN) AdapterInfo->TxRing.UnmappedAddress,
    TX_RING_SIZE
    );

  ZeroMem (
    (VOID *) (UINTN) AdapterInfo->RxBufferMapping.UnmappedAddress,
    RX_BUFFERS_SIZE
    );


  DEBUGWAIT (E1000);

  E1000SetSpeedDuplex (AdapterInfo);

  // If the hardware has already been initialized then don't bother with a reset
  // We want to make sure we do not have to restart autonegotiation and two-pair
  // downshift.
  if (!AdapterInfo->HwInitialized) {
    DEBUGPRINT (E1000, ("Initializing hardware!\n"));

    if (e1000_init_hw (&AdapterInfo->Hw) == 0) {
      DEBUGPRINT (E1000, ("e1000_init_hw success\n"));
      PxeStatcode = PXE_STATCODE_SUCCESS;
      AdapterInfo->HwInitialized      = TRUE;
    } else {
      DEBUGPRINT (CRITICAL, ("Hardware Init failed\n"));
      PxeStatcode = PXE_STATCODE_NOT_STARTED;
    }
  } else {
    DEBUGPRINT (E1000, ("Skipping adapter reset\n"));
    PxeStatcode = PXE_STATCODE_SUCCESS;
  }

  if (PxeStatcode == PXE_STATCODE_SUCCESS) {
    E1000TxRxConfigure (AdapterInfo);
  }

  // Re-read the MAC address.  The CLP configured MAC address is being reset by
  // hardware to the factory address after init, so we need to reset it here.
  if (e1000_read_mac_addr (&AdapterInfo->Hw) != E1000_SUCCESS) {
    DEBUGPRINT (CRITICAL, ("Could not read MAC address.\n"));
  }

  DEBUGWAIT (E1000);

  return PxeStatcode;
}

/** Enables Rx unit.

   @param[in]   AdapterInfo   Pointer to the adapter structure

   @return   RX unit enabled
**/
VOID
RxEnable (
  IN DRIVER_DATA *AdapterInfo
  )
{
  UINT32 RctlReg;

  RctlReg = E1000_READ_REG (&AdapterInfo->Hw, E1000_RCTL);
  RctlReg |= E1000_RCTL_EN;
  E1000_WRITE_REG (&AdapterInfo->Hw, E1000_RCTL, RctlReg);
}

/** Disables Rx unit.

   @param[in]   AdapterInfo   Pointer to the adapter structure

   @return   RX unit disabled or not depending on device MAC type
**/
VOID
RxDisable (
  IN DRIVER_DATA *AdapterInfo
  )
{
  UINT32 RctlReg;

  // No need to disable RX on the following devices
  // when changing receive filters
  if ((AdapterInfo->Hw.mac.type == e1000_82574)
    || (AdapterInfo->Hw.mac.type == e1000_82575)
    || (AdapterInfo->Hw.mac.type == e1000_82583)
    || (AdapterInfo->Hw.mac.type == e1000_i350)
    || (AdapterInfo->Hw.mac.type == e1000_i354)
    || (AdapterInfo->Hw.mac.type == e1000_i210)
    || (AdapterInfo->Hw.mac.type == e1000_i211))
  {
    return;
  }

  RctlReg = E1000_READ_REG (&AdapterInfo->Hw, E1000_RCTL);
  RctlReg &= ~E1000_RCTL_EN;
  E1000_WRITE_REG (&AdapterInfo->Hw, E1000_RCTL, RctlReg);
}

/** Changes filter settings

   @param[in]   AdapterInfo  Pointer to the NIC data structure information which the
                            UNDI driver is layering on..
   @param[in]   NewFilter   A PXE_OPFLAGS bit field indicating what filters to use.
   @param[in]   Cpb         The command parameter Block address.  64 bits since this is Itanium(tm)
                            processor friendly
   @param[in]   CpbSize     Command parameter Block size

   @retval   0   Filters changed according to NewFilter settings
**/
UINTN
E1000SetFilter (
  IN DRIVER_DATA *AdapterInfo,
  IN UINT16       NewFilter,
  IN UINT64       Cpb,
  IN UINT32       CpbSize
  )
{
  PXE_CPB_RECEIVE_FILTERS *CpbReceiveFilter;
  UINT32                   UpdateRCTL;
  UINT16                   CfgFilter;
  UINT16                   OldFilter;
  UINT16                   MulticastCount;
  UINT16                   i;
  UINT16                   j;

  DEBUGPRINT (E1000, ("E1000SetFilter\n"));

  CpbReceiveFilter = (PXE_CPB_RECEIVE_FILTERS *) (UINTN) Cpb;
  OldFilter = AdapterInfo->RxFilter;

  // only these bits need a change in the configuration
  // actually change in bcast requires configure but we ignore that change
  CfgFilter = PXE_OPFLAGS_RECEIVE_FILTER_PROMISCUOUS |
              PXE_OPFLAGS_RECEIVE_FILTER_BROADCAST |
              PXE_OPFLAGS_RECEIVE_FILTER_ALL_MULTICAST;

  if ((OldFilter & CfgFilter) != (NewFilter & CfgFilter)) {

    UpdateRCTL = E1000_READ_REG (&AdapterInfo->Hw, E1000_RCTL);

    if (NewFilter & PXE_OPFLAGS_RECEIVE_FILTER_PROMISCUOUS) {

      // add the UPE bit to the variable to be written to the RCTL
      UpdateRCTL |= E1000_RCTL_UPE;
    }


    if (NewFilter & PXE_OPFLAGS_RECEIVE_FILTER_ALL_MULTICAST) {

      // add the MPE bit to the variable to be written to the RCTL
      UpdateRCTL |= E1000_RCTL_MPE;
    }

    // Put the card into the proper mode...
    // Rx unit needs to be disabled and re-enabled
    // while changing filters.
    if (AdapterInfo->ReceiveStarted) {
      RxDisable (AdapterInfo);
    }

    UpdateRCTL |= E1000_RCTL_BAM;
    AdapterInfo->RxFilter = NewFilter;
    E1000_WRITE_REG (&AdapterInfo->Hw, E1000_RCTL, UpdateRCTL);

    if (AdapterInfo->ReceiveStarted) {
      RxEnable (AdapterInfo);
    }
  }

  // check if mcast setting changed
  if (((NewFilter & PXE_OPFLAGS_RECEIVE_FILTER_FILTERED_MULTICAST) !=
    (OldFilter & PXE_OPFLAGS_RECEIVE_FILTER_FILTERED_MULTICAST))
    || (CpbReceiveFilter != NULL))
  {

    // copy the list
    if (CpbReceiveFilter != NULL) {
      UINT8 McAddrList[MAX_MCAST_ADDRESS_CNT][ETH_ADDR_LEN];

      MulticastCount = AdapterInfo->McastList.Length = (UINT16) (CpbSize / PXE_MAC_LENGTH);
      DEBUGPRINT (E1000, ("E1000: MulticastCount=%d\n", MulticastCount));

      ZeroMem (AdapterInfo->McastList.McAddr, MAX_MCAST_ADDRESS_CNT * PXE_MAC_LENGTH);
      CopyMem (
        AdapterInfo->McastList.McAddr,
        (VOID *) (UINTN) CpbReceiveFilter->MCastList,
        CpbSize
      );

      // Copy the multicast address list into a form that can be accepted by the
      // shared code.
      for (i = 0; (i < MulticastCount && i < MAX_MCAST_ADDRESS_CNT); i++) {
        DEBUGPRINT (E1000, ("E1000: MulticastAddress %d:", i));
        for (j = 0; j < ETH_ADDR_LEN; j++) {
          McAddrList[i][j] = AdapterInfo->McastList.McAddr[i][j];
          DEBUGPRINT (E1000, ("%02x", CpbReceiveFilter->MCastList[i][j]));
        }
        DEBUGPRINT (E1000, ("\n"));
      }

      E1000BlockIt (AdapterInfo, TRUE);

      //Rx unit needs to be disabled while changing filters.
      if (AdapterInfo->ReceiveStarted) {
        RxDisable (AdapterInfo);
      }
      e1000_update_mc_addr_list (
        &AdapterInfo->Hw,
        &McAddrList[0][0],
        MulticastCount
      );
      if (AdapterInfo->ReceiveStarted) {
        RxEnable (AdapterInfo);
      }
      E1000BlockIt (AdapterInfo, FALSE);
    }

    // are we setting the list or resetting??
    if ((NewFilter & PXE_OPFLAGS_RECEIVE_FILTER_FILTERED_MULTICAST) != 0) {
      DEBUGPRINT (E1000, ("E1000: Creating new multicast list.\n"));
      AdapterInfo->RxFilter |= PXE_OPFLAGS_RECEIVE_FILTER_FILTERED_MULTICAST;
    } else {
      DEBUGPRINT (E1000, ("E1000: Disabling multicast list.\n"));
      AdapterInfo->RxFilter &= (~PXE_OPFLAGS_RECEIVE_FILTER_FILTERED_MULTICAST);
    }
  }

  if (NewFilter != 0) {

    // Enable unicast and start the RU
    AdapterInfo->RxFilter |= (NewFilter | PXE_OPFLAGS_RECEIVE_FILTER_UNICAST);
    E1000ReceiveStart (AdapterInfo);
  } else {

    // may be disabling everything!
    AdapterInfo->RxFilter = NewFilter;
    E1000ReceiveStop (AdapterInfo);
  }

  return 0;
}

/** Stops the receive unit.

   @param[in]   AdapterInfo   Pointer to the NIC data structure information
                             which the UNDI driver is layering on..

   @return   Receive unit stopped
**/
VOID
E1000ReceiveStop (
  IN DRIVER_DATA *AdapterInfo
  )
{
  E1000_RECEIVE_DESCRIPTOR *ReceiveDesc;
  UINT32                    TempReg;
  UINTN                     i;
  UINT32                    RxdCtl;

  DEBUGPRINT (E1000, ("E1000ReceiveStop\n"));

  if (!AdapterInfo->ReceiveStarted) {
    DEBUGPRINT (CRITICAL, ("Receive unit already disabled!\n"));
    return;
  }

  if (AdapterInfo->Hw.mac.type == e1000_82571) {
    TempReg = E1000_READ_REG (&AdapterInfo->Hw, E1000_RCTL);
    TempReg &= ~E1000_RCTL_EN;
    E1000_WRITE_REG (&AdapterInfo->Hw, E1000_RCTL, TempReg);
  }

  // On I82575 the ring must be reset when the recieve unit is disabled.
  if (AdapterInfo->Hw.mac.type == e1000_82575
    || AdapterInfo->Hw.mac.type == e1000_82576
#ifndef NO_82580_SUPPORT
    || AdapterInfo->Hw.mac.type == e1000_82580
#endif /* NO_82580_SUPPORT */
    || AdapterInfo->Hw.mac.type == e1000_i350
    || AdapterInfo->Hw.mac.type  == e1000_i354
    || AdapterInfo->Hw.mac.type == e1000_i210
    || AdapterInfo->Hw.mac.type == e1000_i211
    )
  {
    E1000ClearRegBits (AdapterInfo, E1000_RXDCTL (0), E1000_RXDCTL_QUEUE_ENABLE);

    i = 0;
    do {
      gBS->Stall (1);
      RxdCtl = E1000_READ_REG (&AdapterInfo->Hw, E1000_RXDCTL (0));

      i++;
      if (i >= MAX_QUEUE_DISABLE_TIME) {
        break;
      }
    } while ((RxdCtl & E1000_RXDCTL_QUEUE_ENABLE) != 0);
    DEBUGPRINT (E1000, ("Receiver Disabled\n"));

    E1000_WRITE_REG (&AdapterInfo->Hw, E1000_RDH (0), 0);
    E1000_WRITE_REG (&AdapterInfo->Hw, E1000_RDT (0), 0);
    AdapterInfo->CurRxInd = 0;
  }

  if (AdapterInfo->Hw.mac.type == e1000_82575
    || AdapterInfo->Hw.mac.type == e1000_82576
#ifndef NO_82580_SUPPORT
    || AdapterInfo->Hw.mac.type == e1000_82580
#endif /* NO_82580_SUPPORT */
    || AdapterInfo->Hw.mac.type == e1000_i350
    || AdapterInfo->Hw.mac.type  == e1000_i354
    || AdapterInfo->Hw.mac.type == e1000_i210
    || AdapterInfo->Hw.mac.type == e1000_i211
    || AdapterInfo->Hw.mac.type == e1000_82571
    )
  {
    // Clean up any left over packets
    ReceiveDesc = E1000_RX_DESC (&AdapterInfo->RxRing, 0);
    for (i = 0; i < DEFAULT_RX_DESCRIPTORS; i++) {
      ReceiveDesc->length = 0;
      ReceiveDesc->status = 0;
      ReceiveDesc->errors = 0;
      ReceiveDesc++;
    }
  }

  AdapterInfo->ReceiveStarted = FALSE;
  return;
}

/** Starts the receive unit.

   @param[in]   AdapterInfo   Pointer to the NIC data structure information
                             which the UNDI driver is layering on..

   @return   Receive unit started
**/
VOID
E1000ReceiveStart (
  IN DRIVER_DATA *AdapterInfo
  )
{
  UINT32 TempReg;
  UINTN  i;

  DEBUGPRINT (E1000, ("E1000ReceiveStart\n"));

  if (AdapterInfo->ReceiveStarted) {
    DEBUGPRINT (CRITICAL, ("Receive unit already started!\n"));
    return;
  }

  TempReg = E1000_READ_REG (&AdapterInfo->Hw, E1000_RCTL);
  TempReg |= (E1000_RCTL_EN | E1000_RCTL_BAM);
  E1000_WRITE_REG (&AdapterInfo->Hw, E1000_RCTL, TempReg);

  // Move the tail descriptor to begin receives on I82575
#ifndef NO_82575_SUPPORT
  if (AdapterInfo->Hw.mac.type == e1000_82575
#ifndef NO_82576_SUPPORT
    || AdapterInfo->Hw.mac.type == e1000_82576
#ifndef NO_82580_SUPPORT
    || AdapterInfo->Hw.mac.type == e1000_82580
#endif /* NO_82580_SUPPORT */
    || AdapterInfo->Hw.mac.type == e1000_i350
    || AdapterInfo->Hw.mac.type == e1000_i354
    || AdapterInfo->Hw.mac.type == e1000_i210
    || AdapterInfo->Hw.mac.type == e1000_i211
#endif /* NO_82576_SUPPORT */
    )
  {
    if (AdapterInfo->Hw.mac.type == e1000_82575) {
      e1000_rx_fifo_flush_base (&AdapterInfo->Hw);
    }

    E1000SetRegBits (AdapterInfo, E1000_RXDCTL (0), E1000_RXDCTL_QUEUE_ENABLE);

    i = 0;
    do {
      gBS->Stall (1);
      TempReg = E1000_READ_REG (&AdapterInfo->Hw, E1000_RXDCTL (0));

      i++;
      if (i >= MAX_QUEUE_ENABLE_TIME) {
        break;
      }
    } while ((TempReg & E1000_RXDCTL_QUEUE_ENABLE) == 0);

    E1000_WRITE_REG (&AdapterInfo->Hw, E1000_RDT (0), DEFAULT_RX_DESCRIPTORS - 1);
    E1000_WRITE_REG (&AdapterInfo->Hw, E1000_RDH (0), 0);
    AdapterInfo->CurRxInd = (UINT16) E1000_READ_REG (&AdapterInfo->Hw, E1000_RDH (0));

  }
#endif /* NO_82575_SUPPORT */

  AdapterInfo->ReceiveStarted = TRUE;
}

/** Stops the transmit unit.

   @param[in]   AdapterInfo   Pointer to the NIC data structure information
                             which the UNDI driver is layering on..

   @retval   Transmit unit disabled
**/
VOID
E1000TransmitDisable (
  IN DRIVER_DATA *AdapterInfo
  )
{
  UINTN  i;
  UINT32 TxdCtl;

  DEBUGPRINT (E1000, ("E1000TransmitDisable\n"));


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
    E1000ClearRegBits (AdapterInfo, E1000_TXDCTL (0), E1000_TXDCTL_QUEUE_ENABLE);
    i = 0;
    do {
      gBS->Stall (1);
      TxdCtl = E1000_READ_REG (&AdapterInfo->Hw, E1000_TXDCTL (0));
    } while ((++i < MAX_QUEUE_DISABLE_TIME)
      && ((TxdCtl & E1000_TXDCTL_QUEUE_ENABLE) != 0));
    DEBUGPRINT (E1000, ("Transmitter Disabled\n"));
    break;
  default:
    break;
  }
}

/** This routine blocks until auto-negotiation completes or times out (after 4.5 seconds).

   @param[in]   AdapterInfo   Pointer to the NIC data structure information
                             which the UNDI driver is layering on..

   @retval   TRUE   Auto-negotiation completed successfully,
   @retval   FALSE  Auto-negotiation did not complete (i.e., timed out)
**/
BOOLEAN
E1000WaitForAutoNeg (
  IN DRIVER_DATA *AdapterInfo
  )
{
  BOOLEAN AutoNegComplete;
  UINTN   i;
  UINT16  Reg;
  UINT32  Status;

  AutoNegComplete   = FALSE;
  Status            = 0;

  DEBUGPRINT (E1000, ("E1000WaitForAutoNeg\n"));

  if (!AdapterInfo->CableDetect) {

    // Caller specified not to detect cable, so we return true.
    DEBUGPRINT (E1000, ("Cable detection disabled.\n"));
    return TRUE;
  }


  for (i = 0; i < 500; i++) {
    Status = E1000_READ_REG (&AdapterInfo->Hw, E1000_STATUS);
    if ((Status & E1000_STATUS_LU) != 0) {
      if ((E1000_DEV_ID_I210_COPPER == AdapterInfo->Hw.device_id) ||
        (E1000_DEV_ID_I210_COPPER_FLASHLESS == AdapterInfo->Hw.device_id) ||
        (E1000_DEV_ID_I211_COPPER == AdapterInfo->Hw.device_id))
      {
        DELAY_IN_MILLISECONDS (1000);
      }
      DEBUGPRINT (E1000, ("Successfully established link on retry %d\n", i));
      return TRUE;
    }
    DELAY_IN_MILLISECONDS (10);
  }
  DEBUGPRINT (E1000, ("Link up not detected\n"));

  if (AdapterInfo->Hw.phy.type == e1000_phy_igp) {
    DEBUGPRINT (E1000, ("IGP PHY\n"));
    for (i = 5; i != 0; i--) {
      e1000_read_phy_reg (&AdapterInfo->Hw, PHY_1000T_STATUS, &Reg);
      if (Reg != 0) {
        AutoNegComplete = E1000DownShift (AdapterInfo);
        break;
      }
      DELAY_IN_MILLISECONDS (1000);
      Status = E1000_READ_REG (&AdapterInfo->Hw, E1000_STATUS);
      if ((Status & E1000_STATUS_LU) != 0) {
        AutoNegComplete = TRUE;
        break;
      }
    }

  } else if (AdapterInfo->Hw.phy.type == e1000_phy_m88) {

    // We are on a Marvel PHY that supports 2-pair downshift
    // Check the real time link status bit to see if there is actually a cable connected
    // If so then we will attempt to downshift, if not then we will report failure
    // Wait for up to 1 second for real time link detected
    for (i = 100; i != 0; i--) {
      DEBUGPRINT (E1000, ("."));
      e1000_read_phy_reg (&AdapterInfo->Hw, M88E1000_PHY_SPEC_STATUS, &Reg);
      if ((Reg & M88E1000_PSSR_LINK) != 0) {
        DEBUGPRINT (E1000, ("E1000DownShift - Real Time Link Detected\n"));
        AutoNegComplete = E1000DownShift (AdapterInfo);
        break;
      }

      DELAY_IN_MILLISECONDS (10);
    }
  }

  DEBUGPRINT (E1000, ("Return %d\n", AutoNegComplete));
  DEBUGWAIT (E1000);
  return AutoNegComplete;
}

/** Free TX buffers that have been transmitted by the hardware.

   @param[in]   AdapterInfo   Pointer to the NIC data structure information
                             which the UNDI driver is layering on.
   @param[in]   NumEntries   Number of entries in the array which can be freed.
   @param[out]  TxBuffer     Array to pass back free TX buffer

   @return   Number of TX buffers written.
**/
UINT16
E1000FreeTxBuffers (
  IN  DRIVER_DATA *AdapterInfo,
  IN  UINT16       NumEntries,
  OUT UINT64      *TxBuffer
  )
{
  E1000_TRANSMIT_DESCRIPTOR *TransmitDescriptor;
  UINT32                     Tdh;
  UINT16                     i;
  UNDI_DMA_MAPPING          *TxBufMapping;

  DEBUGPRINT (E1000, ("E1000FreeTxBuffers\n"));

  // Read the TX head posistion so we can see which packets have been sent out on the wire.
  Tdh = E1000_READ_REG (&AdapterInfo->Hw, E1000_TDH (0));
  DEBUGPRINT (E1000, ("TDH = %d, AdapterInfo->XmitDoneHead = %d\n", Tdh, AdapterInfo->XmitDoneHead));

  // If Tdh does not equal xmit_done_head then we will fill all the transmitted buffer
  // addresses between Tdh and xmit_done_head into the completed buffers array
  i = 0;
  do {
    if (i >= NumEntries) {
      DEBUGPRINT (E1000, ("Exceeded number of DB entries, i=%d, NumEntries=%d\n", i, NumEntries));
      break;
    }

    TransmitDescriptor = E1000_TX_DESC (&AdapterInfo->TxRing, AdapterInfo->XmitDoneHead);
    TxBufMapping = &AdapterInfo->TxBufferMappings[AdapterInfo->XmitDoneHead];

    if ((TransmitDescriptor->upper.fields.status & E1000_TXD_STAT_DD) != 0) {

      if (TxBufMapping->UnmappedAddress == 0) {
        DEBUGPRINT (CRITICAL, ("ERROR: TX buffer complete without being marked used!\n"));
        break;
      }

      DEBUGPRINT (E1000, ("Writing buffer address %d, %x\n", i, TxBuffer[i]));
      UndiDmaUnmapMemory (AdapterInfo->PciIo, TxBufMapping);

      TxBuffer[i] = TxBufMapping->UnmappedAddress;
      i++;


      ZeroMem (TxBufMapping, sizeof (UNDI_DMA_MAPPING));
      TransmitDescriptor->upper.fields.status = 0;

      AdapterInfo->XmitDoneHead++;
      if (AdapterInfo->XmitDoneHead >= DEFAULT_TX_DESCRIPTORS) {
        AdapterInfo->XmitDoneHead = 0;
      }
    } else {
      DEBUGPRINT (E1000, ("TX Descriptor %d not done\n", AdapterInfo->XmitDoneHead));
      break;
    }
  } while (Tdh != AdapterInfo->XmitDoneHead);
  return i;
}

/** Sets specified bits in a device register

   @param[in]   AdapterInfo   Pointer to the device instance
   @param[in]   Register     Register to write
   @param[in]   BitMask      Bits to set

   @return   Returns the value read from the PCI register.
**/
UINT32
E1000SetRegBits (
  IN DRIVER_DATA *AdapterInfo,
  IN UINT32       Register,
  IN UINT32       BitMask
  )
{
  UINT32 TempReg;

  TempReg = E1000_READ_REG (&AdapterInfo->Hw, Register);
  TempReg |= BitMask;
  E1000_WRITE_REG (&AdapterInfo->Hw, Register, TempReg);

  return TempReg;
}

/** Clears specified bits in a device register

   @param[in]   AdapterInfo   Pointer to the device instance
   @param[in]   Register     Register to write
   @param[in]   BitMask      Bits to clear

   @return    Returns the value read from the PCI register.
**/
UINT32
E1000ClearRegBits (
  IN DRIVER_DATA *AdapterInfo,
  IN UINT32       Register,
  IN UINT32       BitMask
  )
{
  UINT32 TempReg;

  TempReg = E1000_READ_REG (&AdapterInfo->Hw, Register);
  TempReg &= ~BitMask;
  E1000_WRITE_REG (&AdapterInfo->Hw, Register, TempReg);

  return TempReg;
}

/** Checks if link is up

   @param[in]   AdapterInfo   Pointer to the NIC data structure information
                             which the UNDI driver is layering on.

   @retval   TRUE   Link is up
   @retval   FALSE  Link is down
**/
BOOLEAN
IsLinkUp (
  IN DRIVER_DATA *AdapterInfo
  )
{
  UINT32 Reg;

  Reg = E1000_READ_REG (&AdapterInfo->Hw, E1000_STATUS);
  if ((Reg & E1000_STATUS_LU) == 0) {
    return FALSE;
  } else {
    return TRUE;
  }
}

/** Gets current link speed and duplex from shared code and converts it to UNDI
   driver format

   @param[in]   AdapterInfo   Pointer to the device instance

   @retval    LINK_SPEED_10FULL    10 MBit full duplex
   @retval    LINK_SPEED_100FULL   100 MBit full duplex
   @retval    LINK_SPEED_1000FULL  1 GBit full duplex
   @retval    LINK_SPEED_10HALF    10 MBit half duplex
   @retval    LINK_SPEED_100HALF   100 MBit half duplex
   @retval    LINK_SPEED_1000HALF  1 GBit half duplex
   @retval    LINK_SPEED_UNKNOWN    Unknown link speed
**/
UINT8
GetLinkSpeed (
  IN DRIVER_DATA *AdapterInfo
  )
{
  UINT16 Speed     = 0;
  UINT16 Duplex    = 0;
  UINT8  LinkSpeed = LINK_SPEED_UNKNOWN;

  e1000_get_speed_and_duplex (&AdapterInfo->Hw, &Speed, &Duplex);
  switch (Speed) {
  case SPEED_10:
    if (Duplex == FULL_DUPLEX) {
      LinkSpeed = LINK_SPEED_10FULL;
    } else {
      LinkSpeed = LINK_SPEED_10HALF;
    }
    break;
  case SPEED_100:
    if (Duplex == FULL_DUPLEX) {
      LinkSpeed = LINK_SPEED_100FULL;
    } else {
      LinkSpeed = LINK_SPEED_100HALF;
    }
    break;
  case SPEED_1000:
    if (Duplex == FULL_DUPLEX) {
      LinkSpeed = LINK_SPEED_1000FULL;
    } else {
      LinkSpeed = LINK_SPEED_1000HALF;
    }
    break;
  default:
    LinkSpeed = LINK_SPEED_UNKNOWN;
    break;
  }
  DEBUGPRINT (HII, ("Link Speed Status %x\n", LinkSpeed));
  return LinkSpeed;
}

/** Blinks LED on a port for time given in seconds

   @param[in]   AdapterInfo   Pointer to the device instance
   @param[in]   Seconds      Seconds to blink

   @return    LED is blinking for Seconds seconds
**/
VOID
BlinkLeds (
  IN DRIVER_DATA *AdapterInfo,
  IN UINT32       Seconds
  )
{
  e1000_setup_led (&AdapterInfo->Hw);

  switch (AdapterInfo->Hw.mac.type) {
#ifndef NO_82574_SUPPORT
  case e1000_82574:
#endif /* NO_82574_SUPPORT */
    {
      UINT32 Miliseconds = Seconds * 1000;

      while (Miliseconds > 0) {
        e1000_led_on (&AdapterInfo->Hw);
        DelayInMicroseconds (AdapterInfo, 200 * 1000);
        Miliseconds -= 200;
        e1000_led_off (&AdapterInfo->Hw);

        // Do not wait last 200ms with LEDs off.
        if (Miliseconds > 200) {
          DelayInMicroseconds (AdapterInfo, 200 * 1000);
          Miliseconds -= 200;
        } else {

          // Break here when the Seconds is an odd number.
          // Break on 'while (Miliseconds > 0)' when the Seconds is an even number.
          break;
        }
      }
    }
    break;

  default:
    e1000_blink_led (&AdapterInfo->Hw);
    DelayInMicroseconds (AdapterInfo, Seconds * 1000 * 1000);
    break;
  }

  e1000_cleanup_led (&AdapterInfo->Hw);
}

/** Reads PBA string from NVM

   @param[in]       AdapterInfo     Pointer to the device instance
   @param[in,out]   PbaNumber      Pointer to buffer for PBA string
   @param[in]       PbaNumberSize  Size of PBA string

   @retval   EFI_SUCCESS            PBA string successfully read
   @retval   EFI_DEVICE_ERROR       Failed to read PBA string
**/
EFI_STATUS
ReadPbaString (
  IN     DRIVER_DATA *AdapterInfo,
  IN OUT UINT8       *PbaNumber,
  IN     UINT32       PbaNumberSize
  )
{
  if (e1000_read_pba_string (&AdapterInfo->Hw, PbaNumber, PbaNumberSize) == E1000_SUCCESS) {
    return EFI_SUCCESS;
  } else {
    return EFI_DEVICE_ERROR;
  }
}

/** Detects surprise removal device status in PCI controller register

   @param[in]   AdapterInfo   Pointer to the device instance

   @retval   TRUE    Surprise removal has been detected
   @retval   FALSE   Surprise removal has not been detected
**/
BOOLEAN
IsSurpriseRemoval (
  IN DRIVER_DATA *AdapterInfo
  )
{
  UINT32 Results;

  if (AdapterInfo->SurpriseRemoval) {
    return TRUE;
  }

  MemoryFence ();
  AdapterInfo->PciIo->Mem.Read (
                            AdapterInfo->PciIo,
                            EfiPciIoWidthUint32,
                            0,
                            E1000_STATUS,
                            1,
                            (VOID *) (&Results)
                          );
  MemoryFence ();

  if (Results == INVALID_STATUS_REGISTER_VALUE) {

    if ((AdapterInfo->Hw.mac.type >= e1000_i210)
      && !( e1000_get_flash_presence_i210 (&AdapterInfo->Hw)))
    {
      // during e1000_pll_workaround_i210 flow, flashless springville adapter
      // may be put in D3 power state for a milisecond. In order to avoid
      // SurpriseRemoval state need to check if the device is in D3 power state
      UINT16 PciWord;
      MemoryFence ();
      AdapterInfo->PciIo->Pci.Read (
                                AdapterInfo->PciIo,
                                EfiPciIoWidthUint16,
                                E1000_PCI_PMCSR,
                                1,
                                (VOID *) &PciWord
                              );
      MemoryFence ();

      if (PciWord == 0xFFFF) {
        // if we've read an 0xFFFF from PCI config space value
        // it still means that we're it's a surprise removal state
        AdapterInfo->SurpriseRemoval = TRUE;
        return TRUE;
      }

      PciWord &= E1000_PCI_PMCSR_D3;
      if (PciWord == E1000_PCI_PMCSR_D3) {
        return FALSE;
      }

    }
    AdapterInfo->SurpriseRemoval = TRUE;
    return TRUE;
  }
  return FALSE;
}

/** Delay a specified number of microseconds

   @param[in]   AdapterInfo    Pointer to the NIC data structure information
                               which the UNDI driver is layering on..
   @param[in]   MicroSeconds   Time to delay in Microseconds.

   @return   Execution of code delayed
**/
VOID
DelayInMicroseconds (
  IN DRIVER_DATA *AdapterInfo,
  IN UINTN        MicroSeconds
  )
{
  if (AdapterInfo->Delay != NULL) {
    (*AdapterInfo->Delay) (AdapterInfo->UniqueId, MicroSeconds);
  } else {
    gBS->Stall (MicroSeconds);
  }
}

