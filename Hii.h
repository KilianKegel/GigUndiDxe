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
#ifndef HII_H_
#define HII_H_

/* This is the generated IFR binary data for each formset defined in VFR.
  This data array is ready to be used as input of PreparePackageList() to
  create a packagelist (which contains Form packages, String packages, etc). */
extern UINT8 InventoryBin[];

/* This is the generated String package data for all .UNI files.
  This data array is ready to be used as input of PreparePackageList() to
  create a packagelist (which contains Form packages, String packages, etc). */
extern UINT8 GigUndiDxeStrings[];

/* This seems to be enough for all string operations in HII code.
  May be adjusted in the future if needed */
#define HII_STRING_LEN                    1024

#define HII_INVALID_DELL_LANG             "x-RIS-UEFI"

#define MAX_PBA_STR_LENGTH                15 // normally it is 10 chars string

/* Variables are used for determination of default number of advertised VFs */

/** Samples Gig driver private data structure from protocol instance

   @param[in]  a   Protocol instance

   @return   UNDI_PRIVATE_DATA structure instance is retrieved
**/
#define DRIVER_SAMPLE_PRIVATE_FROM_THIS(a)  CR (a, UNDI_PRIVATE_DATA, ConfigAccess, GIG_UNDI_DEV_SIGNATURE)



// Check whether address is multicast, little-endian specific check.

#define IS_MULTICAST(address) (BOOLEAN)(((UINT8 *)(address))[0] & ((UINT8)0x01))





/** Installs the HII user interface screen in the UEFI device manager.

   @param[in]   UndiPrivateData   Points to the driver instance private data.

   @retval   EFI_SUCCESS     HII interface installed correctly
   @retval   !EFI_SUCCESS    Failed to install HII interface
**/
EFI_STATUS
EFIAPI
HiiInit (
  UNDI_PRIVATE_DATA *UndiPrivateData
  );

/** HII uninstalls the HII user interface screen in the UEFI device manager.

   @param[in]   UndiPrivateData   Points to the driver instance private data.

   @retval   EFI_SUCCESS    HII interface uninstalled correctly
   @retval   !EFI_SUCCESS   Failed to uninstall HII interface
**/
EFI_STATUS
EFIAPI
HiiUnload (
  UNDI_PRIVATE_DATA *UndiPrivateData
  );

#endif /* HII_H_ */
