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
#include <Uefi.h>
#include <Library/PrintLib.h>
#include <Library/UefiRuntimeServicesTableLib.h>
#include <Library/HiiLib.h>
#include <Guid/MdeModuleHii.h>
#include <Protocol/HiiConfigRouting.h>
#include <Protocol/FormBrowser2.h>
#include <Protocol/HiiConfigAccess.h>
#include <Protocol/HiiDatabase.h>
#include <Protocol/HiiString.h>
#include "NVDataStruc.h"
#include "wol.h"


// This is the auto-generated header file that includes definitions of string IDs for HII
#include "GigUndiDxeStrDefs.h"

#include "EepromConfig.h"





#include "ComponentName.h"
#include "HiiInternalLib.h"
#include "Hii.h"

/* Global and module variables */

/* Protocol structure tentative definition */
EFI_HII_CONFIG_ACCESS_PROTOCOL gUndiHiiConfigAccess;

EFI_GUID mHiiFormGuid    = E1000_HII_FORM_GUID;
EFI_GUID mHiiDataGuid    = E1000_HII_DATA_GUID;


CHAR16 mVariableName[]    = L"UndiNVData";

STATIC BOOLEAN mBlinkLedsCalled = FALSE;

UINT32 gGuidInstance = 0;




/** This function allows a caller to extract the current configuration for one
   or more named elements from the target driver.

   @param[in]   This       Points to the EFI_HII_CONFIG_ACCESS_PROTOCOL.
   @param[in]   Request    A null-terminated Unicode string in <ConfigRequest> format.
   @param[out]  Progress   On return, points to a character in the Request string.
                           Points to the string's null terminator if request was successful.
                           Points to the most recent '&' before the first failing name/value
                           pair (or the beginning of the string if the failure is in the
                           first name/value pair) if the request was not successful.
   @param[out]   Results   A null-terminated Unicode string in <ConfigAltResp> format which
                           has all values filled in for the names in the Request string.
                           String to be allocated by the called function.

   @retval   EFI_SUCCESS            The Results is filled with the requested values.
   @retval   EFI_OUT_OF_RESOURCES   Not enough memory to store the results.
   @retval   EFI_INVALID_PARAMETER  Progress is NULL
   @retval   EFI_INVALID_PARAMETER  Results is NULL
   @retval   EFI_INVALID_PARAMETER  Request is NULL, illegal syntax, or unknown name.
   @retval   EFI_DEVICE_ERROR       Failed to construct <ConfigHdr> template.
   @retval   EFI_NOT_FOUND          Routing data doesn't match any storage in this driver.
   @retval   EFI_DEVICE_ERROR       Failed to extract iSCSI configuration
   @retval   EFI_DEVICE_ERROR       Failed to extract FCoE configuration
**/
EFI_STATUS
EFIAPI
ExtractConfig (
  IN  CONST EFI_HII_CONFIG_ACCESS_PROTOCOL *This,
  IN  CONST EFI_STRING                      Request,
  OUT EFI_STRING *                          Progress,
  OUT EFI_STRING *                          Results
  )
{
  EFI_STATUS         Status;
  UNDI_PRIVATE_DATA *UndiPrivateData;
  UINT8              AltMac[6];
  EFI_STRING         TmpString;
  UINTN              ElementOffset;
  UINTN              ElementWidth;
  UINTN              LastElementWidth;
  EFI_STRING         ConfigRequestHdr;
  EFI_STRING         ConfigRequest;
  BOOLEAN            AllocatedRequest;
  UINTN              Size;
  UINTN              BufferSize;

  if (Progress == NULL
    || Results == NULL)
  {
    return EFI_INVALID_PARAMETER;
  }

  Status           = EFI_INVALID_PARAMETER;
  ConfigRequestHdr = NULL;
  ConfigRequest    = NULL;
  AllocatedRequest = FALSE;
  Size             = 0;

  if (Request != NULL) {
    // UINTN RequestStringLength = StrSize(Request); // Format string: %u
    DEBUGPRINT (HII, ("=== ExtractConfig: Incoming Request  ===\n"));
    DEBUGPRINT (HII, ("%s\n", Request));
    DEBUGPRINT (HII, ("=== End Request ===\n"));
  }


  *Progress = Request;
  if ((Request != NULL)
    && !HiiIsConfigHdrMatch (Request, &mHiiDataGuid, mVariableName))
  {
    DEBUGPRINT (HII, ("HiiIsConfigHdrMatch failed\n"));
    DEBUGWAIT (HII);
    return EFI_NOT_FOUND;
  }

  UndiPrivateData = DRIVER_SAMPLE_PRIVATE_FROM_THIS (This);


  ConfigRequest = Request;
  BufferSize = sizeof (UNDI_DRIVER_CONFIGURATION);
  if ((Request == NULL)
    || (StrStr (Request, L"OFFSET") == NULL))
  {

    // Request has no request element, construct full request string.
    // Allocate and fill a buffer large enough to hold the <ConfigHdr> template
    // followed by "&OFFSET=0&WIDTH=WWWWWWWWWWWWWWWW" followed by a Null-terminator.
    ConfigRequestHdr = HiiConstructConfigHdr (&mHiiDataGuid, mVariableName, UndiPrivateData->HiiInstallHandle);
    if (ConfigRequestHdr == NULL) {
      DEBUGPRINT(HII, ("Failed to construct <ConfigHdr> template\n"));
      return EFI_DEVICE_ERROR;
    }

    Size = (StrLen (ConfigRequestHdr) + 32 + 1) * sizeof (CHAR16);
    ConfigRequest = AllocateZeroPool (Size);
    if (ConfigRequest == NULL) {
      DEBUGPRINT (CRITICAL, ("Failed to allocate ConfigRequest!\n"));
      return EFI_OUT_OF_RESOURCES;
    }

    AllocatedRequest = TRUE;
    UnicodeSPrint (ConfigRequest, Size, L"%s&OFFSET=0&WIDTH=%016LX", ConfigRequestHdr, (UINT64) BufferSize);
    FreePool (ConfigRequestHdr);
  }

  // We already know the Request has a correct header. Jump over the header and
  // start parsing requested parameters
  TmpString = SkipConfigHeader (ConfigRequest);

  LastElementWidth = 0;
  ElementOffset = 0;
  ElementWidth = 0;

  // Don't read all parameters from NVM, only those included in the Request string.
  // On TM UEFI calls ExtractConfig for each individual parameter, so it would
  // cost a lot of time to read all parameters during each ExtractConfig execution.
  // Each element consists of a OFFSET, WIDTH pair.
  for (; ; ) {
    if (ElementWidth > LastElementWidth) {
      ElementWidth -= LastElementWidth;
      ElementOffset += LastElementWidth;
    } else {
      TmpString = GetNextRequestElement (TmpString, &ElementOffset, &ElementWidth);
      if (TmpString == NULL) {

        // No more elements in the Request string
        // If Request string is empty (contains only header) then
        // we exit the loop with error status
        break;
      }
    }

    // Assume success
    Status = EFI_SUCCESS;
    // DEBUGPRINT(HII, ("ElementOffset=%d, ElementWidth=%d\n", ElementOffset, ElementWidth));

    if (ElementOffset == UNDI_CONFIG_OFFSET (LinkSpeed)) {
      UndiPrivateData->Configuration.LinkSpeed = (UINT8) EepromGetLanSpeedStatus (UndiPrivateData);
      DEBUGPRINT (HII, ("GetLinkSpeed %d\n", UndiPrivateData->Configuration.LinkSpeed));
      LastElementWidth = UNDI_CONFIG_WIDTH (LinkSpeed);
      continue;
    }
    if (ElementOffset == UNDI_CONFIG_OFFSET (DefaultWolEnable)) {
      if (WolIsWakeOnLanSupported (UndiPrivateData)) {
        UndiPrivateData->Configuration.DefaultWolEnable = WOL_ENABLE;
      } else {
        UndiPrivateData->Configuration.DefaultWolEnable = WOL_NA;
      }
      LastElementWidth = UNDI_CONFIG_WIDTH (DefaultWolEnable);
      continue;
    }
    if (ElementOffset == UNDI_CONFIG_OFFSET (WolEnable)) {
      UndiPrivateData->Configuration.WolEnable = WolGetWakeOnLanStatusEx (UndiPrivateData);
      LastElementWidth = UNDI_CONFIG_WIDTH (WolEnable);
      continue;
    }

    if (ElementOffset == UNDI_CONFIG_OFFSET (AltMacAddr)) {
      if (UndiPrivateData->AltMacAddrSupported) {
        UINT8 MacAddr[6];

        // Copy Alternate MAC Address
        EepromMacAddressGet (UndiPrivateData, (UINT16 *) &MacAddr[0], (UINT16 *) &AltMac[0]);

        if ((AltMac[0] == MacAddr[0]) &&
          (AltMac[1] == MacAddr[1]) &&
          (AltMac[2] == MacAddr[2]) &&
          (AltMac[3] == MacAddr[3]) &&
          (AltMac[4] == MacAddr[4]) &&
          (AltMac[5] == MacAddr[5]))
        {
          ZeroMem (AltMac, sizeof (AltMac));
        }

        // Convert it to a MAC string
        UnicodeSPrint (
          UndiPrivateData->Configuration.AltMacAddr,
          sizeof (UndiPrivateData->Configuration.AltMacAddr),
          L"%02x:%02x:%02x:%02x:%02x:%02x",
          AltMac[0],
          AltMac[1],
          AltMac[2],
          AltMac[3],
          AltMac[4],
          AltMac[5]
        );
      }
      LastElementWidth = UNDI_CONFIG_WIDTH (AltMacAddr);
      continue;
    }
    if (ElementOffset == UNDI_CONFIG_OFFSET (LinkStatus)) {
      if (IsLinkUp (&UndiPrivateData->NicInfo)) {
        UndiPrivateData->Configuration.LinkStatus = LINK_CONNECTED;
      } else {
        UndiPrivateData->Configuration.LinkStatus = LINK_DISCONNECTED;
      }
      LastElementWidth = UNDI_CONFIG_WIDTH (LinkStatus);
      continue;
    }
    if (ElementOffset == UNDI_CONFIG_OFFSET (BlinkLed)) {

      // Set LED blinking time to default value of 0 (don't blink)
      UndiPrivateData->Configuration.BlinkLed = 0;
      LastElementWidth = UNDI_CONFIG_WIDTH (BlinkLed);
      continue;
    }
    if (ElementOffset == UNDI_CONFIG_OFFSET (AltMacAddrSupport)) {
      UndiPrivateData->Configuration.AltMacAddrSupport = UndiPrivateData->AltMacAddrSupported;
      LastElementWidth = UNDI_CONFIG_WIDTH (AltMacAddrSupport);
      continue;
    }
    if (ElementOffset == UNDI_CONFIG_OFFSET (LinkSpeedSettingsSupported)) {
      UndiPrivateData->Configuration.LinkSpeedSettingsSupported = UndiPrivateData->LinkSpeedSettingsSupported;
      LastElementWidth = UNDI_CONFIG_WIDTH (LinkSpeedSettingsSupported);
      continue;
    }
    if (ElementOffset == UNDI_CONFIG_OFFSET (WolSettingsSupported)) {
      if (WolIsWakeOnLanSupported (UndiPrivateData)) {
        UndiPrivateData->Configuration.WolSettingsSupported = WOL_SETTINGS_SUPPORTED;
      } else {
        UndiPrivateData->Configuration.WolSettingsSupported = WOL_SETTINGS_NOT_SUPPORTED;
      }
      LastElementWidth = UNDI_CONFIG_WIDTH (WolSettingsSupported);
      continue;
    }

    // If we got there that means the element's offset is not on the list.
    // Assume the width is one and search again
    LastElementWidth = 1;
    continue;
  }

  if (EFI_ERROR (Status)) {

    // Error ocurred while reading NVM settings.
    DEBUGPRINT (CRITICAL, ("Error ocurred while reading NVM settings %r\n", Status));
    DEBUGWAIT (CRITICAL);
    goto ExitExtractError;
  }

  // Parameters have been correctly read from NVM. Create Result strings using
  // helper function from BIOS and exit.
  Status = UndiPrivateData->HiiConfigRouting->BlockToConfig (
                                                UndiPrivateData->HiiConfigRouting,
                                                ConfigRequest,
                                                (UINT8 *) &UndiPrivateData->Configuration,
                                                sizeof (UNDI_DRIVER_CONFIGURATION),
                                                Results,
                                                Progress
                                              );
  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("BlockToConfig failed with %r\n", Status));
    DEBUGWAIT (CRITICAL);
  }

  // Set Progress string to the original request string.
  if (Request == NULL) {
    *Progress = NULL;
  } else if (StrStr (Request, L"OFFSET") == NULL) {
    *Progress = Request + StrLen (Request);
  }

ExitExtractError:

  // Free the allocated config request string.
  if (AllocatedRequest) {
    FreePool (ConfigRequest);
    ConfigRequest = NULL;
  }

  return Status;
}

/** This function processes the results of changes in configuration.

   @param[in]   This           Points to the EFI_HII_CONFIG_ACCESS_PROTOCOL.
   @param[in]   Configuration  A null-terminated Unicode string in <ConfigResp> format.
   @param[out]  Progress       A pointer to a string filled in with the offset of the most
                               recent '&' before the first failing name/value pair (or the
                               beginning of the string if the failure is in the first
                               name/value pair) or the terminating NULL if all was successful.

   @retval   EFI_SUCCESS            The Results is processed successfully.
   @retval   EFI_INVALID_PARAMETER  Configuration is NULL.
   @retval   EFI_INVALID_PARAMETER  Progress is NULL.
   @retval   EFI_NOT_FOUND          Routing data doesn't match any storage in this driver.
**/
EFI_STATUS
EFIAPI
RouteConfig (
  IN  CONST EFI_HII_CONFIG_ACCESS_PROTOCOL *This,
  IN  CONST EFI_STRING                      Configuration,
  OUT EFI_STRING *                          Progress
  )
{
  EFI_STATUS                       Status;
  UINTN                            BufferSize;
  UNDI_PRIVATE_DATA                *UndiPrivateData;

  if ((Configuration == NULL)
    || (Progress == NULL))
  {
    return EFI_INVALID_PARAMETER;
  }

  // UINTN ConfigurationStringLength = StrSize(Configuration); // Format string: %u
  DEBUGPRINT (HII, ("=== RouteConfig: Incoming Configuration ===\n"));
  DEBUGPRINT (HII, ("%s\n", Configuration));
  DEBUGPRINT (HII, ("=== End Configuration ===\n"));

  if (!HiiIsConfigHdrMatch (Configuration, &mHiiDataGuid, mVariableName)) {
    DEBUGPRINT (HII, ("RouteConfig: Configuration didn't match ConfigHdr...\n"));
    *Progress = Configuration;
    return EFI_NOT_FOUND;
  }

  UndiPrivateData = DRIVER_SAMPLE_PRIVATE_FROM_THIS (This);



  // Convert <ConfigResp> to buffer data by helper function ConfigToBlock()
  BufferSize = sizeof (UNDI_DRIVER_CONFIGURATION);
  Status = UndiPrivateData->HiiConfigRouting->ConfigToBlock (
                                                UndiPrivateData->HiiConfigRouting,
                                                Configuration,
                                                (UINT8 *) &UndiPrivateData->Configuration,
                                                &BufferSize,
                                                Progress
                                              );
  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("ConfigToBlock returns %r\n", Status));
    DEBUGWAIT (CRITICAL);
    goto ExitRouteError;
  }
  DEBUGPRINT (HII, ("Set WakeOnLan %d\n", UndiPrivateData->Configuration.WolEnable));

  // General Parameters
  UndiPrivateData->Configuration.WolEnable = WolEnableWakeOnLanEx (
                                               UndiPrivateData,
                                               UndiPrivateData->Configuration.WolEnable
                                             );

  DEBUGPRINT (HII, ("Set Link Speed %d\n", UndiPrivateData->Configuration.LinkSpeed));
  EepromSetLanSpeed (
    UndiPrivateData,
    UndiPrivateData->Configuration.LinkSpeed
  );

  if (UndiPrivateData->Configuration.BlinkLed > 15) {

    //  Report invalid parametrer value
    SetProgressString (
      Configuration,
      STRUCT_OFFSET (UNDI_DRIVER_CONFIGURATION, BlinkLed),
      Progress
    );
    goto ExitRouteError;
  } else if (UndiPrivateData->Configuration.BlinkLed > 0) {
    BlinkLeds (&UndiPrivateData->NicInfo, UndiPrivateData->Configuration.BlinkLed);
  }






ExitRouteError:
  EepromUpdateChecksum (UndiPrivateData);

  DEBUGPRINT (HII, ("RouteConfig finishes with status %r\n", Status));

  return Status;
}

/** Locates HII protocols (Database, String, FormBrowser2 and ConfigRouting)

   @param[in]   UndiPrivateData   Driver private data structure

   @retval   EFI_SUCCESS     All protocols located successfully
   @retval   EFI_NOT_FOUND   No instances of one of above protocols were found
**/
EFI_STATUS
HiiOpenProtocol (
  UNDI_PRIVATE_DATA *UndiPrivateData
  )
{
  EFI_STATUS                      Status;

  // Initialize the Hii Database handle to NULL so we can check later
  // to see whether it was installed.
  UndiPrivateData->HiiDatabase = NULL;
  UndiPrivateData->HiiString = NULL;
  UndiPrivateData->FormBrowser2 = NULL;
  UndiPrivateData->HiiConfigRouting = NULL;

  DEBUGPRINT (HII, ("Locate HII Protocol\n"));

  // Locate Hii Database protocol
  Status = gBS->LocateProtocol (
                  &gEfiHiiDatabaseProtocolGuid,
                  NULL,
                  (VOID **) &UndiPrivateData->HiiDatabase
                );
  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("Error finding HII protocol: %r\n", Status));
    DEBUGWAIT (CRITICAL);
    return Status;
  }

  // Locate HiiString protocol
  DEBUGPRINT (HII, ("Locate HII String Protocol\n"));
  Status = gBS->LocateProtocol (
                  &gEfiHiiStringProtocolGuid,
                  NULL,
                  (VOID **) &UndiPrivateData->HiiString
                );
  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("Error finding HII String protocol: %r\n", Status));
    DEBUGWAIT (CRITICAL);
    return Status;
  }

  // Locate Formbrowser2 protocol
  DEBUGPRINT (HII, ("Locate HII Form Browser Protocol\n"));
  Status = gBS->LocateProtocol (
                  &gEfiFormBrowser2ProtocolGuid,
                  NULL,
                  (VOID **) &UndiPrivateData->FormBrowser2
                );
  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("Error finding HII form browser protocol: %r\n", Status));
    DEBUGWAIT (CRITICAL);
    return Status;
  }

  // Locate ConfigRouting protocol
  DEBUGPRINT (HII, ("Locate HII ConfigRouting Protocol\n"));
  Status = gBS->LocateProtocol (
                  &gEfiHiiConfigRoutingProtocolGuid,
                  NULL,
                  (VOID **) &UndiPrivateData->HiiConfigRouting
                );
  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("Error finding HII ConfigRouting protocol: %r\n", Status));
    DEBUGWAIT (CRITICAL);
    return Status;
  }

  return Status;
}

/** Sets MAC ID string according to MAC type

   @param[in]   UndiPrivateData   Driver private data structure
   @param[out]  String            Pointer to string buffer for resulting MAC ID

   @return   Matching MAC ID string written to String buffer, or "unknown"
**/
VOID
HiiSetMacIdString (
  IN  UNDI_PRIVATE_DATA *UndiPrivateData,
  OUT CHAR16 *           String
  )
{
  switch (UndiPrivateData->NicInfo.Hw.mac.type) {

#ifndef NO_82571_SUPPORT
  case e1000_82571:
    UnicodeSPrint (String, 80, L"Intel 82571");
    break;
  case e1000_82572:
    UnicodeSPrint (String, 80, L"Intel 82572");
    break;
  case e1000_82573:
    UnicodeSPrint (String, 80, L"Intel 82573");
    break;
#ifndef NO_82574_SUPPORT
  case e1000_82574:
    UnicodeSPrint (String, 80, L"Intel 82574");
    break;
  case e1000_82583:
    UnicodeSPrint (String, 80, L"Intel 82583V");
    break;
#endif /* NO_82574_SUPPORT */
#endif /* NO_82571_SUPPORT */
#ifndef NO_80003ES2LAN_SUPPORT
  case e1000_80003es2lan:
    UnicodeSPrint (String, 80, L"Intel 80003ES2LAN");
    break;
#endif /* NO_80003ES2LAN_SUPPORT */
#ifndef NO_82575_SUPPORT
  case e1000_82575:
    UnicodeSPrint (String, 80, L"Intel 82575");
    break;
  case e1000_82576:
    UnicodeSPrint (String, 80, L"Intel 82576");
    break;
#endif /* NO_82575_SUPPORT */
#ifndef NO_82580_SUPPORT
  case e1000_82580:
    UnicodeSPrint (String, 80, L"Intel 82580");
    break;
#endif /* NO_82580_SUPPORT */
  case e1000_i350:
    UnicodeSPrint (String, 80, L"Intel i350");
    break;
  case e1000_i354:
    UnicodeSPrint (String, 80, L"Intel i354");
    break;
  case e1000_i210:
    UnicodeSPrint (String, 80, L"Intel i210");
    break;
  case e1000_i211:
    UnicodeSPrint (String, 80, L"Intel i211");
    break;
  default:
    UnicodeSPrint (String, 80, L"unknown");
    break;
  }
}


/** This function performs required initialization of strings in String Package

   @param[in]   UndiPrivateData   Driver private data structure
   @param[in]   Lang              Language of the strings to retrieve and set

   @retval EFI_SUCCESS            All tasks have finished succesfully
   @retval EFI_DEVICE_ERROR       Processing error ocurred
**/
EFI_STATUS
HiiSetMenuStrings (
  UNDI_PRIVATE_DATA *UndiPrivateData,
  CHAR8 *            Lang
  )
{
  EFI_STATUS    Status = EFI_SUCCESS;
  CHAR16        *BrandString;
  CHAR16        String[HII_STRING_LEN];
  CHAR16        SubString[HII_STRING_LEN];
  UINT8         MacAddr[6];
  UINT8         AltMacAddr[6];
  EFI_STRING    SubStringTmp;
  EFI_STRING_ID StringId;

  // Do not modify x-UEFI langugage strings
  if (Lang[0] != 'x') {

    // Take the branding string of this device
    // Branding strings are not localized, use the default English branding
    // string.
    BrandString = UndiPrivateData->Brand;

    UnicodeSPrint (
      SubString,
      HII_STRING_LEN,
      L"%s",
      BrandString
    );

    UnicodeSPrint (
      SubString,
      HII_STRING_LEN,
      L"%s - %02x:%02x:%02x:%02x:%02x:%02x",
      BrandString,
      UndiPrivateData->NicInfo.Hw.mac.addr[0],
      UndiPrivateData->NicInfo.Hw.mac.addr[1],
      UndiPrivateData->NicInfo.Hw.mac.addr[2],
      UndiPrivateData->NicInfo.Hw.mac.addr[3],
      UndiPrivateData->NicInfo.Hw.mac.addr[4],
      UndiPrivateData->NicInfo.Hw.mac.addr[5]
    );
    StringId = HiiSetString (
               UndiPrivateData->HiiHandle,
               STRING_TOKEN (STR_INV_FORM_SET_TITLE),
               SubString,
               Lang
             );
    if (StringId == 0) {
      DEBUGPRINT (CRITICAL, ("IfrLibSetString error: %r\n", Status));
      DEBUGWAIT (CRITICAL);
      return EFI_DEVICE_ERROR;
    }

    // Set Factory Default MAC address
    DEBUGPRINT (HII, ("Setting MAC address\n"));

    EepromMacAddressGet (
      UndiPrivateData,
      (UINT16 *) &MacAddr[0],
      (UINT16 *) &AltMacAddr[0]
    );

    SubStringTmp = HiiGetString (
                     UndiPrivateData->HiiHandle,
                     STRING_TOKEN (STR_MAC_ADDR_TEXT),
                     Lang
                   );

    if (SubStringTmp == NULL) {
      DEBUGPRINT (CRITICAL, ("HiiGetString error\n"));
      DEBUGWAIT (CRITICAL);
      return Status;
    }

    // Fill with MAC address data
    UnicodeSPrint (
      String,
      HII_STRING_LEN,
      SubStringTmp,
      MacAddr[0],
      MacAddr[1],
      MacAddr[2],
      MacAddr[3],
      MacAddr[4],
      MacAddr[5]
    );

    // Need to free memory previously allocated by HiiGetString
    FreePool (SubStringTmp);

    StringId = HiiSetString (
                 UndiPrivateData->HiiHandle,
                 STRING_TOKEN (STR_MAC_ADDR_TEXT),
                 String,
                 Lang
               );
    if (StringId == 0) {
      DEBUGPRINT (CRITICAL, ("1:SetString error: %r\n", Status));
      DEBUGWAIT (CRITICAL);
      return EFI_DEVICE_ERROR;
    }

    // Set PCI Address
    {
      UnicodeSPrint (
        String,
        HII_STRING_LEN,
        L"%02x:%02x:%02x",
        UndiPrivateData->NicInfo.Bus,
        UndiPrivateData->NicInfo.Device,
        UndiPrivateData->NicInfo.Function
      );
    }
    StringId = HiiSetString (
                 UndiPrivateData->HiiHandle,
                 STRING_TOKEN (STR_PCI_BUS_DEV_FUNC_TEXT),
                 String,
                 Lang
               );

    if (StringId == 0) {
      DEBUGPRINT (CRITICAL, ("HiiSetString error\n"));
      DEBUGWAIT (CRITICAL);
      return EFI_DEVICE_ERROR;
    }

    // Set UEFI Driver Branding String
    ComponentNameGetDriverName (NULL, "eng", &BrandString);

    SubStringTmp = HiiGetString (
                     UndiPrivateData->HiiHandle,
                     STRING_TOKEN (STR_EFI_DRIVER_VER_TEXT),
                     Lang
                   );

    if (SubStringTmp == NULL) {
      DEBUGPRINT (CRITICAL, ("HiiGetString error\n"));
      DEBUGWAIT (CRITICAL);
      return Status;
    }

    UnicodeSPrint (String, HII_STRING_LEN, SubStringTmp, BrandString);

    FreePool (SubStringTmp);

    StringId = HiiSetString (
                 UndiPrivateData->HiiHandle,
                 STRING_TOKEN (STR_EFI_DRIVER_VER_TEXT),
                 String,
                 Lang
               );

    if (StringId == 0) {
      DEBUGPRINT (CRITICAL, ("HiiSetString error\n"));
      DEBUGWAIT (CRITICAL);
      return EFI_DEVICE_ERROR;
    }

    // Set Device Name String
    UnicodeSPrint (
      String,
      HII_STRING_LEN,
      L"%s",
      UndiPrivateData->Brand
    );

    StringId = HiiSetString (
                 UndiPrivateData->HiiHandle,
                 STRING_TOKEN (STR_DEVICE_NAME_TEXT),
                 String,
                 Lang
               );
    if (StringId == 0) {
      DEBUGPRINT (CRITICAL, ("HiiSetString error\n"));
      DEBUGWAIT (CRITICAL);
      return EFI_DEVICE_ERROR;
    }

    // Set PCI Device ID
    SubStringTmp = HiiGetString (
                     UndiPrivateData->HiiHandle,
                     STRING_TOKEN (STR_DEVICE_ID_TEXT),
                     Lang
                   );

    if (SubStringTmp == NULL) {
      DEBUGPRINT (CRITICAL, ("HiiGetString error\n"));
      DEBUGWAIT (CRITICAL);
      return Status;
    }

    UnicodeSPrint (String, HII_STRING_LEN, SubStringTmp, UndiPrivateData->NicInfo.Hw.device_id);

    FreePool (SubStringTmp);

    StringId = HiiSetString (
                 UndiPrivateData->HiiHandle,
                 STRING_TOKEN (STR_DEVICE_ID_TEXT),
                 String,
                 Lang
               );
    if (StringId == 0) {
      DEBUGPRINT (CRITICAL, ("HiiSetString error\n"));
      DEBUGWAIT (CRITICAL);
      return EFI_DEVICE_ERROR;
    }

    // Set MAC ID String, not localized, use English string.
    HiiSetMacIdString (UndiPrivateData, String);
    StringId = HiiSetString (
                 UndiPrivateData->HiiHandle,
                 STRING_TOKEN (STR_CONTROLER_ID_TEXT),
                 String,
                 Lang
               );
    if (StringId == 0) {
      DEBUGPRINT (CRITICAL, ("HiiSetString error\n"));
      DEBUGWAIT (CRITICAL);
      return EFI_DEVICE_ERROR;
    }

    // Set PBA number
    {
      CHAR8  PBAString8[MAX_PBA_STR_LENGTH];
      CHAR16 PBAString[MAX_PBA_STR_LENGTH];

      Status = ReadPbaString (
                 &UndiPrivateData->NicInfo,
                 (UINT8 *)PBAString8,
                 MAX_PBA_STR_LENGTH
               );
      if (Status == EFI_SUCCESS) {

        // Convert CHAR8 to CHAR16 for use with SPrint
        PBAString8[MAX_PBA_STR_LENGTH - 1] = 0;
        Status = AsciiStrToUnicodeStrWrapper (
                   PBAString8,
                   PBAString,
                   sizeof (PBAString) / sizeof (CHAR16)
                 );
        if (EFI_ERROR (Status)) {
          DEBUGPRINT (CRITICAL, ("AsciiStrToUnicodeStrWrapper error\n"));
          DEBUGWAIT (CRITICAL);
          return Status;
        }
      } else if (UndiPrivateData->NicInfo.Hw.mac.type == e1000_i211) {
        StrCpyS (
          PBAString,
          HII_STRING_LEN,
          L"N/A"
        );
      } else {
        DEBUGPRINT (CRITICAL, ("ReadPbaString error\n"));
        DEBUGWAIT (CRITICAL);
        return EFI_DEVICE_ERROR;
      }
      StringId = HiiSetString (
                   UndiPrivateData->HiiHandle,
                   STRING_TOKEN (STR_ADAPTER_PBA_TEXT),
                   PBAString,
                   Lang
                 );
      if (StringId == 0) {
        DEBUGPRINT (CRITICAL, ("HiiSetString error\n"));
        DEBUGWAIT (CRITICAL);
        return EFI_DEVICE_ERROR;
      }
    }
  }


  return Status;
}

/** Sets Link speed support and WOL settings.

   @param[in,out]   UndiPrivateData   Points to the driver instance private data.

   @retval   EFI_SUCCESS  Settings successfully applied
**/
EFI_STATUS
HiiConfigureStandardFeaturesSupport (
  IN OUT UNDI_PRIVATE_DATA *UndiPrivateData
  )
{

  // Decide about link speed options depending on link capabilities
  // of the adapter

  // Speed settings are supported for copper only
  if (UndiPrivateData->NicInfo.Hw.phy.media_type != e1000_media_type_copper) {
    UndiPrivateData->LinkSpeedSettingsSupported = FALSE;
  } else {
    UndiPrivateData->LinkSpeedSettingsSupported = TRUE;
  }
  UndiPrivateData->Configuration.WolEnable = (UINT8) WolGetWakeOnLanStatusEx (UndiPrivateData);

  return EFI_SUCCESS;
}


/** Initializes HII inventory packages

   @param[in]   UndiPrivateData   Driver private data structure

   @retval   EFI_SUCCESS            Function returned successfully
   @retval   EFI_OUT_OF_RESOURCES   Not enough resources for HII packages
   @retval   !EFI_SUCCESS           Failed to initialize HII inventory
**/
EFI_STATUS
HiiInventoryPackage (
  UNDI_PRIVATE_DATA *UndiPrivateData
  )
{
  EFI_STATUS                      Status;
  EFI_HII_PACKAGE_LIST_HEADER     *PackageList = NULL;
  CHAR8                           Lang[HII_STRING_LEN];
  CHAR8                           SubLang[HII_STRING_LEN];
  UINTN                           Size;
  UINTN                           i, j;
  UINT8                           *NGigUndiDxeStringsPtr;
  NGigUndiDxeStringsPtr = GigUndiDxeStrings;


  DEBUGPRINT (HII, ("InventoryPackage\n"));


    PackageList = HiiAddPackages (
                    &mHiiFormGuid,
                    UndiPrivateData->HiiInstallHandle,
                    NGigUndiDxeStringsPtr,
                    InventoryBin,
                    NULL
                  );
  if (PackageList == NULL) {
    DEBUGPRINT (CRITICAL, ("PreparePackageList, out of resource.\n"));
    DEBUGWAIT (CRITICAL);
    return EFI_OUT_OF_RESOURCES;
  }

  UndiPrivateData->HiiHandle = PackageList;

  Status = HiiConfigureStandardFeaturesSupport (UndiPrivateData);
  if (EFI_ERROR (Status)) {
    return Status;
  }


  // Prepare HII strings for all supported languages
  Size = HII_STRING_LEN;
  Status = UndiPrivateData->HiiString->GetLanguages (
                                         UndiPrivateData->HiiString,
                                         UndiPrivateData->HiiHandle,
                                         Lang,
                                         &Size
                                       );

  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("GetLanguages returns %r\n", Status));
    DEBUGWAIT (CRITICAL);
    return Status;
  }

  i = 0;
  while (i < Size) {
    j = 0;
    do {
      if (Lang[i] == ';') {
        SubLang[j] = '\0';
        i++;
        j++;
        break;
      } else {
        SubLang[j] = Lang[i];
        i++;
        j++;
      }

    } while (Lang[i] != '\0');

    SubLang[j] = '\0';

    // Setup strings for this language
    Status = HiiSetMenuStrings (UndiPrivateData, SubLang);

    if (EFI_ERROR (Status)) {
      DEBUGPRINT (CRITICAL, ("SetMenuStrings returns %r\n", Status));
      DEBUGWAIT (CRITICAL);
      break;
    }

    // This was the last supported language so we can exit the loop
    if (Lang[i] == '\0') {
      break;
    }
  }

  return Status;
}

/** Callback to handle request for LED blinking

   @param[in]   This        HII config access protocol current instance
   @param[in]   Action      Type of browser action
   @param[in]   QuestionId  Question ID related to specific HII content
   @param[in]   Type        Type specifying question value
   @param[in]   Value       Question value

   @retval   EFI_SUCCESS       LED blinking handled successfully
   @retval   EFI_UNSUPPORTED   Failed to get/set HII browser data
**/
EFI_STATUS
HiiBlinkLedsCallback (
  IN CONST EFI_HII_CONFIG_ACCESS_PROTOCOL *This,
  IN  EFI_BROWSER_ACTION                   Action,
  IN EFI_QUESTION_ID                       QuestionId,
  IN UINT8                                 Type,
  IN EFI_IFR_TYPE_VALUE *                  Value
  )
{
  EFI_STATUS                Status;
  UNDI_PRIVATE_DATA         *UndiPrivateData;
  UNDI_DRIVER_CONFIGURATION HiiDrvConfig;

  Status = EFI_SUCCESS;

  if ((Action != EFI_BROWSER_ACTION_CHANGING) &&
    (Action != EFI_BROWSER_ACTION_CHANGED)) {
    return Status;
  }

  UndiPrivateData = DRIVER_SAMPLE_PRIVATE_FROM_THIS (This);

  ZeroMem (&HiiDrvConfig, sizeof (HiiDrvConfig));
  if (!HiiGetBrowserData (NULL, NULL, sizeof (HiiDrvConfig), (UINT8 *) &HiiDrvConfig)) {
    return EFI_UNSUPPORTED;
  }

  switch (QuestionId) {
  case QUESTION_ID_BLINK_LED:
    if (Type == EFI_IFR_TYPE_NUM_SIZE_16) {
      if ((Action == EFI_BROWSER_ACTION_CHANGING) ||
        ((Action == EFI_BROWSER_ACTION_CHANGED)
        && (!mBlinkLedsCalled)))
      {
#if defined(u16)
#undef u16
#endif /* defined(u16) */

        BlinkLeds (&UndiPrivateData->NicInfo, Value->u16);
        if (Action == EFI_BROWSER_ACTION_CHANGING) {
          mBlinkLedsCalled = TRUE;
        } else {
          mBlinkLedsCalled = FALSE;
        }
      }

      // After blinking the LED, always clear the Blink LEDs question back to 0.
      Value->u16 = 0;
      HiiDrvConfig.BlinkLed = 0;

      if (!HiiSetBrowserData (NULL, NULL, sizeof (HiiDrvConfig), (UINT8 *) &HiiDrvConfig, NULL)) {
        Status = EFI_UNSUPPORTED;
      }
    } else {
      Status = EFI_UNSUPPORTED;
    }
    break;
  default:
    break;
  }

  return Status;
}

/** This function processes the results of changes in configuration.

   @param[in]   This            Points to the EFI_HII_CONFIG_ACCESS_PROTOCOL.
   @param[in]   Action          Specifies the type of action taken by the browser.
   @param[in]   QuestionId      A unique value which is sent to the original exporting driver
                                so that it can identify the type of data to expect.
   @param[in]   Type            The type of value for the question.
   @param[in]   Value           A pointer to the data being sent to the original exporting driver.
   @param[out]  ActionRequest   On return, points to the action requested by the callback function.

   @retval   EFI_SUCCESS           The callback successfully handled the action.
   @retval   EFI_OUT_OF_RESOURCES  Not enough storage is available to hold the variable and its data.
   @retval   EFI_DEVICE_ERROR      The variable could not be saved.
   @retval   EFI_UNSUPPORTED       The specified Action is not supported by the callback.
**/
EFI_STATUS
EFIAPI
DriverCallback (
  IN  CONST EFI_HII_CONFIG_ACCESS_PROTOCOL *This,
  IN  EFI_BROWSER_ACTION                    Action,
  IN  EFI_QUESTION_ID                       QuestionId,
  IN  UINT8                                 Type,
  IN  EFI_IFR_TYPE_VALUE *                  Value,
  OUT EFI_BROWSER_ACTION_REQUEST *          ActionRequest
  )
{
  EFI_STATUS Status;

  DEBUGPRINT (HII, ("Action=%d, QuestionId=%d, Type=%d\n", Action, QuestionId, Type));

  if ((Value == NULL)
    || (ActionRequest == NULL))
  {
    return EFI_INVALID_PARAMETER;
  }

  *ActionRequest = EFI_BROWSER_ACTION_REQUEST_NONE;

  switch (Action) {
  case EFI_BROWSER_ACTION_CHANGING:
    Status = HiiBlinkLedsCallback (This, Action, QuestionId, Type, Value);
    if (EFI_ERROR (Status)) {
      break;
    }
    break;
  case EFI_BROWSER_ACTION_CHANGED:
    Status = HiiBlinkLedsCallback (This, Action, QuestionId, Type, Value);
    if (EFI_ERROR (Status)) {
      break;
    }
    break;
  case EFI_BROWSER_ACTION_FORM_OPEN:
  case EFI_BROWSER_ACTION_FORM_CLOSE:
    Status = EFI_SUCCESS;
    break;
  case EFI_BROWSER_ACTION_DEFAULT_STANDARD:
  case EFI_BROWSER_ACTION_DEFAULT_MANUFACTURING:
  case EFI_BROWSER_ACTION_DEFAULT_SAFE:
  case EFI_BROWSER_ACTION_DEFAULT_PLATFORM:
  case EFI_BROWSER_ACTION_DEFAULT_HARDWARE:
  case EFI_BROWSER_ACTION_DEFAULT_FIRMWARE:
    DEBUGPRINT (HII, ("DriverCallback: Attempting to set defaults, unsupported.\n"));

    switch (Type) {
      case EFI_IFR_TYPE_NUM_SIZE_8:
        DEBUGPRINT (HII, ("Proposed default (UINT8): %01x\n", *((UINT8 *) Value)));
        break;
      case EFI_IFR_TYPE_NUM_SIZE_16:
        DEBUGPRINT (HII, ("Proposed default (UINT16): %02x\n", *((UINT16 *) Value)));
        break;
      case EFI_IFR_TYPE_NUM_SIZE_32:
        DEBUGPRINT (HII, ("Proposed default (UINT32): %04x\n", *((UINT32 *) Value)));
        break;
      case EFI_IFR_TYPE_NUM_SIZE_64:
        DEBUGPRINT (HII, ("Proposed default (UINT64): %08x\n", *((UINT64 *) Value)));
        break;
      case EFI_IFR_TYPE_BOOLEAN:
        DEBUGPRINT (HII, ("Proposed default (BOOLEAN): %01x\n", *((BOOLEAN *) Value)));
        break;
      case EFI_IFR_TYPE_STRING:
        DEBUGPRINT (HII, ("Proposed default (STRING_ID): %02x\n", *((EFI_STRING_ID *) Value)));
        break;
      default:
        DEBUGPRINT (HII, ("Proposed default type: %01x (can't display...)\n", Type));
        break;
    }

    Status = EFI_UNSUPPORTED;
    break;
  default:
    Status = EFI_UNSUPPORTED;
    break;
  }

  DEBUGPRINT (HII, ("DriverCallback finished with status: %d\n", Status));
  DEBUGWAIT (HII);
  return Status;
}

/** Installs the HII user interface screen in the UEFI device manager.

   @param[in]   UndiPrivateData   Points to the driver instance private data.

   @retval   EFI_SUCCESS     HII interface installed correctly
   @retval   !EFI_SUCCESS    Failed to install HII interface
**/
EFI_STATUS
EFIAPI
HiiInit (
  IN UNDI_PRIVATE_DATA *UndiPrivateData
  )
{
  EFI_STATUS                      Status;
  EFI_SCREEN_DESCRIPTOR           Screen;

  DEBUGPRINT (HII, ("HiiInit\n"));


  // Try to open the HII protocols first.  If they are not present in the system
  // get out.
  Status = HiiOpenProtocol (UndiPrivateData);
  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("HiiOpenProtocol returns: %x\n", Status));
    DEBUGWAIT (CRITICAL);
    return Status;
  }

  // Initialize screen dimensions for SendForm().
  // Remove 3 characters from top and bottom
  ZeroMem (&Screen, sizeof (EFI_SCREEN_DESCRIPTOR));
  gST->ConOut->QueryMode (gST->ConOut, gST->ConOut->Mode->Mode, &Screen.RightColumn, &Screen.BottomRow);

  Screen.TopRow     = 3;
  Screen.BottomRow  = Screen.BottomRow - 3;

  UndiPrivateData->ConfigAccess = gUndiHiiConfigAccess;

  UndiPrivateData->HiiInstallHandle = UndiPrivateData->DeviceHandle;

  Status = gBS->InstallProtocolInterface (
                  &UndiPrivateData->HiiInstallHandle,
                  &gEfiHiiConfigAccessProtocolGuid,
                  EFI_NATIVE_INTERFACE,
                  &UndiPrivateData->ConfigAccess
                );
  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("InstallProtocolInterface error: %r\n", Status));
    DEBUGWAIT (CRITICAL);
  }

  Status = HiiInventoryPackage (UndiPrivateData);
  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("InventoryPackage returns: %r\n", Status));
    DEBUGWAIT (CRITICAL);
    return Status;
  }

  // Initialize configuration data
  DEBUGPRINT (HII, ("initialize configuration data\n"));
  ZeroMem (&UndiPrivateData->Configuration, sizeof (UNDI_DRIVER_CONFIGURATION));

  DEBUGPRINT (HII, ("HiiInit is complete\n"));

  return EFI_SUCCESS;
}

/** HII uninstalls the HII user interface screen in the UEFI device manager.

   @param[in]   UndiPrivateData   Points to the driver instance private data.

   @retval   EFI_SUCCESS    HII interface uninstalled correctly
   @retval   !EFI_SUCCESS   Failed to uninstall HII interface
**/
EFI_STATUS
EFIAPI
HiiUnload (
  IN UNDI_PRIVATE_DATA *UndiPrivateData
  )
{
  EFI_STATUS                      Status;

  if ((UndiPrivateData->HiiDatabase == NULL) ||
    (UndiPrivateData->HiiString == NULL) ||
    (UndiPrivateData->FormBrowser2 == NULL) ||
    (UndiPrivateData->HiiConfigRouting == NULL))
  {
    DEBUGPRINT (HII, ("HII Not initialized, returning.\n"));
    return EFI_SUCCESS;
  }

  DEBUGPRINT (HII, ("Calling RemovePackageList: %X\n", UndiPrivateData->HiiDatabase));
  Status = UndiPrivateData->HiiDatabase->RemovePackageList (
                                           UndiPrivateData->HiiDatabase,
                                           UndiPrivateData->HiiHandle
                                         );
  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("RemovePackageList error: %r\n", Status));
    DEBUGWAIT (CRITICAL);
    return Status;
  }

  Status = gBS->UninstallProtocolInterface (
                  UndiPrivateData->HiiInstallHandle,
                  &gEfiHiiConfigAccessProtocolGuid,
                  &UndiPrivateData->ConfigAccess
                );
  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("UninstallProtocolInterface error: %r\n", Status));
    DEBUGWAIT (CRITICAL);
  }

  return Status;
}

/* Protocol structure definition and initialization */

EFI_HII_CONFIG_ACCESS_PROTOCOL gUndiHiiConfigAccess = {
  ExtractConfig,
  RouteConfig,
  DriverCallback
};
