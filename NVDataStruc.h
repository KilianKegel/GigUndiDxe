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
#ifndef NV_DATA_STRUC_H_
#define NV_DATA_STRUC_H_

#include <Guid/HiiPlatformSetupFormset.h>


#define E1000_HII_FORM_GUID \
  { \
    0x77f2ea2f, 0x4312, 0x4569, 0x85, 0xc4, 0x58, 0x3a, 0xcd, 0x8d, 0xb7, 0xe2 \
  }

#define E1000_HII_DATA_GUID \
  { \
    0xa31abb16, 0xc627, 0x475b, 0x98, 0x8e, 0x7e, 0xe0, 0x77, 0x67, 0x40, 0xf3 \
  }


// #define VAR_EQ_TEST_NAME  0x100

#define STORAGE_VARIABLE_ID                   0x1234

#define FORM_MAIN                             0x0001
#define FORM_NIC                              0x1235


#define LINK_SPEED_AUTO_NEG                   0x00
#define LINK_SPEED_10HALF                     0x01
#define LINK_SPEED_10FULL                     0x02
#define LINK_SPEED_100HALF                    0x03
#define LINK_SPEED_100FULL                    0x04
#define LINK_SPEED_1000HALF                   0x05
#define LINK_SPEED_1000FULL                   0x06
#define LINK_SPEED_10000HALF                  0x07
#define LINK_SPEED_10000FULL                  0x08
#define LINK_SPEED_20000                      0x09
#define LINK_SPEED_25000                      0x0A
#define LINK_SPEED_40000                      0x0B
#define LINK_SPEED_NO_CONFIGURE_AUTO          0x10
#define LINK_SPEED_UNKNOWN                    0x20

#define WOL_DISABLE                           0x00
#define WOL_ENABLE                            0x01
#define WOL_NA                                0x02
#define WOL_SETTINGS_NOT_SUPPORTED            0
#define WOL_SETTINGS_SUPPORTED                1

#define OROM_DISABLE                          0x00
#define OROM_ENABLE                           0x01

#define LINK_DISCONNECTED                     0x00
#define LINK_CONNECTED                        0x01




#pragma pack(1)
typedef struct {
  UINT8  LinkSpeedSettingsSupported;
  UINT8  LinkSpeed;
  UINT8  WolSettingsSupported;
  UINT8  WolEnable;
  UINT8  DefaultWolEnable;
  UINT8  AltMacAddrSupport;
  UINT8  LinkStatus;
  UINT8  Padding1;
  UINT16 AltMacAddr[18];
  UINT16 BlinkLed;

} UNDI_DRIVER_CONFIGURATION;
#pragma pack()


/** Returns offset of field in Driver Configuration structure

   @param[in]   Field    Specific field from structure

   @return   Offset of given field is returned
**/
#define UNDI_CONFIG_OFFSET(Field) STRUCT_OFFSET (UNDI_DRIVER_CONFIGURATION, Field)

/** Returns width of field in Driver Configuration structure

   @param[in]   Field    Specific field from structure

   @return   Width of given field is returned
**/
#define UNDI_CONFIG_WIDTH(Field) sizeof (UndiPrivateData->Configuration. ## Field ## )

// General parameters
#define     QUESTION_ID_EFI_DRIVER_VER                          0x1100
#define     QUESTION_ID_ADAPTER_PBA                             0x1101
#define     QUESTION_ID_CONTROLER_ID                            0x1102
#define     QUESTION_ID_PCI_BUS_DEV_FUNC                        0x1103
#define     QUESTION_ID_LINK_STATUS                             0x1104
#define     QUESTION_ID_MAC_ADDR                                0x1105
#define     QUESTION_ID_ALT_MAC_ADDR                            0x1106
#define     QUESTION_ID_LINK_SPD_STATUS                         0x110C
#define     QUESTION_ID_LINK_SPEED                              0x1108
#define     QUESTION_ID_WOL                                     0x1109
#define     QUESTION_ID_BLINK_LED                               0x110A
#define     QUESTION_ID_DEVICE_ID                               0x110B
#define     QUESTION_ID_DEVICE_NAME                             0x111F
#define     QUESTION_ID_DEFAULT_WOL                             0x1182

// Menu references - used for goto opcodes
#define     QUESTION_ID_NIC_CONFIG_MENU                         0x1130

#define       QUESTION_ID_TMP_SUPPORT_ALT_MAC_ADDR              0x1407
#define       QUESTION_ID_TMP_SUPPORT_LINK_SPD_STATUS           0x1408
#endif /* NV_DATA_STRUC_H_ */
