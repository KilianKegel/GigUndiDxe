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
#include "CommonDriver.h"



/** Retrieves the health status of a controller in the platform.

   @param[in]   This               Driver health protocol instance
   @param[in]   ControllerHandle   Controller to retrieve the health status on
   @param[in]   ChildHandle        Child to retrieve the health status on
   @param[out]  HealthStatus       Pointer to resultant health status
   @param[out]  MessageList        List of resultant error messages
   @param[out]  FormHiiHandle      Hii handle containing HII form used when
                                   configuration is required

   @retval      EFI_SUCCESS       Health status successfully retrieved
   @retval      EFI_UNSUPPORTED   HealthStatus is NULL
   @retval      !EFI_SUCCESS      Failure to retrieve health status
**/
EFI_STATUS
EFIAPI
GetHealthStatus (
  IN  EFI_DRIVER_HEALTH_PROTOCOL *     This,
  IN  EFI_HANDLE                       ControllerHandle, OPTIONAL
  IN  EFI_HANDLE                       ChildHandle,      OPTIONAL
  OUT EFI_DRIVER_HEALTH_STATUS *       HealthStatus,
  OUT EFI_DRIVER_HEALTH_HII_MESSAGE ** MessageList,    OPTIONAL
  OUT EFI_HII_HANDLE *                 FormHiiHandle   OPTIONAL
  )
{
  EFI_STATUS Status;

  if (HealthStatus == NULL) {
    DEBUGPRINT (CRITICAL, ("HealthStatus is NULL\n"));
    return EFI_INVALID_PARAMETER;
  }

  // Assume the message list is empty
  if (MessageList != NULL) {
    *MessageList = NULL;
  }

  // Assume no Form HII Handle is returned by the function
  if (FormHiiHandle != NULL) {
    *FormHiiHandle = NULL;
  }

  // Check if ControllerHandle is valid: should be NULL or
  // value of ControllerHandle managed by our driver.
  // The same for ChildHandle
  if (ControllerHandle != NULL) {
    DEBUGPRINT (HEALTH, ("%a, %d\n", __FUNCTION__, __LINE__));
    Status = EfiTestManagedDevice (
               ControllerHandle,
               gUndiDriverBinding.DriverBindingHandle,
               &gEfiDevicePathProtocolGuid
             );
    if (EFI_ERROR (Status)) {
      return Status;
    }

    if (ChildHandle != NULL) {
      Status = EfiTestChildHandle (
                 ControllerHandle,
                 ChildHandle,
                 &gEfiPciIoProtocolGuid
               );
      if (EFI_ERROR (Status)) {
        DEBUGPRINT (HEALTH, ("EfiTestChildHandle returned - %r\n", Status));
        return Status;
      }
      DEBUGPRINT (HEALTH, ("ControllerHandle and ChildHandle match\n"));
    }

    // 1G adapters always return status Healthy
    *HealthStatus = EfiDriverHealthStatusHealthy;
  } else {
    DEBUGPRINT (HEALTH, ("Get cumulative health status for driver\n"));

    // 1G adapters always return status Healthy
    *HealthStatus = EfiDriverHealthStatusHealthy;
  }

  return EFI_SUCCESS;
}

/** Functionality is unsupported

   @param[in]   This                  Driver Health Protocol instance
   @param[in]   ControllerHandle      Controller to repair handle
   @param[in]   ChildHandle           Child to repair handle
   @param[in]   ProgressNotification  Notification function to report progress

   @retval   EFI_UNSUPPORTED   This function is unsupported
**/
EFI_STATUS
EFIAPI
Repair (
  IN  EFI_DRIVER_HEALTH_PROTOCOL                *This,
  IN  EFI_HANDLE                                ControllerHandle,
  IN  EFI_HANDLE                                ChildHandle,          OPTIONAL
  IN  EFI_DRIVER_HEALTH_REPAIR_NOTIFY           ProgressNotification  OPTIONAL
  )
{
  DEBUGPRINT (HEALTH, ("Called\n"));
  return EFI_UNSUPPORTED;
}

EFI_DRIVER_HEALTH_PROTOCOL gUndiDriverHealthProtocol = {
  GetHealthStatus,
  Repair
};
