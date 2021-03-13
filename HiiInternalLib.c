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
#include <Library/BaseLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/UefiRuntimeServicesTableLib.h>
#include <Library/HiiLib.h>
#include <Library/PrintLib.h>
#include <Library/UefiLib.h>

#include "NVDataStruc.h"


/** Return the pointer to the section of the Request string that begins just
   after the header.

   @param[in]   Request   A request string starting with "GUID="

   @return     Pointer to request string section after header or NULL if Request
               string ends before reaching it
**/
EFI_STRING
SkipConfigHeader (
  IN EFI_STRING Request
  )
{
  EFI_STRING StringPtr;

  if (Request == NULL) {
    return NULL;
  }

  StringPtr = Request;

  // Requets string must start with "GUID="
  if (StrnCmp (StringPtr, L"GUID=", StrLen (L"GUID=")) != 0) {
    return NULL;
  }

  while ((*StringPtr != 0)
    && StrnCmp (StringPtr, L"PATH=", StrLen (L"PATH=")) != 0)
  {
    StringPtr++;
  }
  if (*StringPtr == 0) {
    return NULL;
  }

  while ((*StringPtr != L'&')
    && (*StringPtr != 0))
  {
    StringPtr++;
  }
  if (*StringPtr == 0) {
    return NULL;
  }
  
  // Skip '&'
  StringPtr++;

  return StringPtr;
}

/** Get the value of <Number> in <BlockConfig> format, i.e. the value of OFFSET
  or WIDTH or VALUE.
  
  <BlockConfig> ::= 'OFFSET='<Number>&'WIDTH='<Number>&'VALUE'=<Number>
  This is a internal function.

  @param[in]  StringPtr         String in <BlockConfig> format and points to the
                                first character of <Number>.
  @param[in]  Number            The output value. Caller takes the responsibility
                                to free memory.
  @param[out]  Len              Length of the <Number>, in characters.

  @retval EFI_OUT_OF_RESOURCES   Insufficient resources to store neccessary
                                 structures.
  @retval EFI_SUCCESS            Value of <Number> is outputted in Number
                                 successfully.
**/
EFI_STATUS
GetValueOfNumber (
  IN EFI_STRING StringPtr,
  OUT UINT8 **  Number,
  OUT UINTN *   Len
  )
{
  EFI_STRING TmpPtr;
  UINTN      Length;
  EFI_STRING Str;
  UINT8 *    Buf;
  EFI_STATUS Status;
  UINT8      DigitUint8;
  UINTN      Index;
  CHAR16     TemStr[2];

  ASSERT (StringPtr != NULL 
    && Number != NULL 
    && Len != NULL);
  
  ASSERT (*StringPtr != L'\0');

  Buf = NULL;

  TmpPtr = StringPtr;
  while ((*StringPtr != L'\0')
    && (*StringPtr != L'&'))
  {
    StringPtr++;
  }
  *Len   = StringPtr - TmpPtr;
  Length = *Len + 1;

  Str = (EFI_STRING) AllocateZeroPool (Length * sizeof (CHAR16));
  if (Str == NULL) {
    Status = EFI_OUT_OF_RESOURCES;
    goto Exit;
  }
  CopyMem (Str, TmpPtr, (*Len) * sizeof (CHAR16));
  *(Str + *Len) = L'\0';

  Length = (Length + 1) / 2;
  Buf = (UINT8 *) AllocateZeroPool (Length);
  if (Buf == NULL) {
    Status = EFI_OUT_OF_RESOURCES;
    goto Exit;
  }

  Length = *Len;
  ZeroMem (TemStr, sizeof (TemStr));
  for (Index = 0; Index < Length; Index++) {
    TemStr[0] = Str[Length - Index - 1];
    DigitUint8 = (UINT8) StrHexToUint64 (TemStr);
    if ((Index & 1) == 0) {
      Buf [Index / 2] = DigitUint8;
    } else {
      Buf [Index / 2] = (UINT8) ((DigitUint8 << 4) + Buf [Index / 2]);
    }
  }

  *Number = Buf;
  Status  = EFI_SUCCESS;

Exit:
  if (Str != NULL) {
    FreePool (Str);
  }

  return Status;
}

/** Find next element in the Request string and return its parameters.

   The Request string is formatted like so, with A being an address and
   B being the buffer size: "OFFSET=A1&WIDTH=B1&OFFSET=A2&WIDTH=B2&...".

   @param[in]    Request   a request string
   @param[out]   ElementOffset   the offset of element
   @param[out]   ElementWidth   the width of the element

   @return       EFI_STRING on current position in the Request string
**/
EFI_STRING
GetNextRequestElement (
  IN EFI_STRING Request,
  OUT UINTN *    ElementOffset,
  OUT UINTN *    ElementWidth
  )
{
  EFI_STRING StringPtr;
  EFI_STRING TmpPtr;
  EFI_STATUS Status;
  UINTN      Length;
  UINT8 *    TmpBuffer;
  UINTN      Offset;
  UINTN      Width;

  if (Request == NULL) {
    return NULL;
  }

  StringPtr = Request;

  // If we currently point at the '&' character, this means we've received a
  // pointer to a string with multiple OFFSET/WIDTH pairs that we've previously
  // handled here and moved the pointer after the WIDTH=... (if it contains
  // just a single pair, the first char will be \0, caught below).
  if (StringPtr[0] == '&') {
    StringPtr++;
  }

  // If we've reached the end of the request or point at something unexpected,
  // just stop further parsing.
  if ((*StringPtr == 0 )
    || StrnCmp (StringPtr, L"OFFSET=", StrLen (L"OFFSET=")) != 0)
  {
    return NULL;
  }

  // Back up the header of one <BlockName>
  TmpPtr = StringPtr;

  StringPtr += StrLen (L"OFFSET=");

  // Get Offset
  Status = GetValueOfNumber (StringPtr, &TmpBuffer, &Length);
  if (Status == EFI_OUT_OF_RESOURCES) {
    return NULL;
  }

  Offset = 0;
  CopyMem (
    &Offset,
    TmpBuffer,
    (((Length + 1) / 2) < sizeof (UINTN)) ? ((Length + 1) / 2) : sizeof (UINTN)
  );
  FreePool (TmpBuffer);

  StringPtr += Length;

  if (StrnCmp (StringPtr, L"&WIDTH=", StrLen (L"&WIDTH=")) != 0) {
    return NULL;
  }
  StringPtr += StrLen (L"&WIDTH=");

  // Get Width
  Status = GetValueOfNumber (StringPtr, &TmpBuffer, &Length);
  if (Status == EFI_OUT_OF_RESOURCES) {
    return NULL;
  }
  Width = 0;
  CopyMem (
    &Width,
    TmpBuffer,
    (((Length + 1) / 2) < sizeof (UINTN)) ? ((Length + 1) / 2) : sizeof (UINTN)
  );
  FreePool (TmpBuffer);

  StringPtr += Length;

  *ElementWidth = Width;
  *ElementOffset = Offset;

  return StringPtr;
}

/** Returns number of values present in Config Resp String

   @param[in]   ConfigRespString   String in <ConfigResp> format

   @return    Number of value entries
**/

/** Determines if a Unicode character is a hexadecimal digit.
   The test is case insensitive.

   @param[in]   Digit  Pointer to byte that receives the value of the hex character.   
   @param[in]   Char   Unicode character to test.
   
   @retval   TRUE     If the character is a hexadecimal digit.
   @retval   FALSE    If the character is not a hexadecimal digit.
**/
BOOLEAN
IsHexDigit (
  OUT UINT8 *Digit,
  IN  CHAR16 Char
  )
{
  if ((Char >= L'0') 
    && (Char <= L'9'))
  {
    *Digit = (UINT8) (Char - L'0');
    return TRUE;
  }

  if ((Char >= L'A')
    && (Char <= L'F'))
  {
    *Digit = (UINT8) (Char - L'A' + 0x0A);
    return TRUE;
  }

  if ((Char >= L'a')
    && (Char <= L'f'))
  {
    *Digit = (UINT8) (Char - L'a' + 0x0A);
    return TRUE;
  }

  return FALSE;
}


/** Converts Unicode string to binary buffer. The conversion may be partial.
   
   The first character in the string that is not hex digit stops the conversion.
   At a minimum, any blob of data could be represented as a hex string.

   @param[in,out]   Buf   Pointer to buffer that receives the data.
   @param[in,out]   Len   Length in bytes of the buffer to hold converted data.
                          If routine return with EFI_SUCCESS, containing length of converted data.
                          If routine return with EFI_BUFFER_TOO_SMALL, containg length of buffer desired.
   @param[in]       Str               String to be converted from.
   @param[out]      ConvertedStrLen   Length of the Hex String consumed. 

   @retval   EFI_SUCCESS            Routine Success.
   @retval   EFI_BUFFER_TOO_SMALL   The buffer is too small to hold converted data.
**/
EFI_STATUS
HexStringToBuf (
  IN OUT UINT8 *                    Buf,
  IN OUT UINTN *                    Len,
  IN     CHAR16 *                   Str,
  OUT    UINTN                     *ConvertedStrLen OPTIONAL
  )
{
  UINTN HexCnt;
  UINTN Idx;
  UINTN BufferLength;
  UINT8 Digit;
  UINT8 Byte;

  // Find out how many hex characters the string has.
  for (Idx = 0, HexCnt = 0; IsHexDigit (&Digit, Str[Idx]); Idx++, HexCnt++) {
    ;
  }

  if (HexCnt == 0) {
    *Len = 0;
    return EFI_SUCCESS;
  }
  
  // Two Unicode characters make up 1 buffer byte. Round up.
  BufferLength = (HexCnt + 1) / 2;

  // Test if  buffer is passed enough.
  if (BufferLength > (*Len)) {
    *Len = BufferLength;
    return EFI_BUFFER_TOO_SMALL;
  }

  *Len = BufferLength;

  for (Idx = 0; Idx < HexCnt; Idx++) {

    IsHexDigit (&Digit, Str[HexCnt - 1 - Idx]);

    // For odd charaters, write the lower nibble for each buffer byte,
    // and for even characters, the upper nibble.
    if ((Idx & 1) == 0) {
      Byte = Digit;
    } else {
      Byte = Buf[Idx / 2];
      Byte &= 0x0F;
      Byte |= Digit << 4;
    }

    Buf[Idx / 2] = Byte;
  }

  if (ConvertedStrLen != NULL) {
    *ConvertedStrLen = HexCnt;
  }

  return EFI_SUCCESS;
}

/** Search "&OFFSET=<Offset>&WIDTH=<Width>" string in the input string that matches
   the Offset value. Set OutString to point to the BlockName that was found.

   @param[in,out]   InString   The string to search in.
   @param[in]       Offset     Offset value of searched BlockName.
   @param[out]      OutString  OFFSET= string that was found 

   @retval   TRUE         BlockName was found.
   @retval   FALSE        BlockName not found.
**/
BOOLEAN
SetProgressString (
  IN OUT EFI_STRING InString,
  UINTN             Offset,
  OUT EFI_STRING *  OutString
  )
{
  EFI_STATUS Status;
  UINTN      Data;
  UINTN      BufferSize;
  UINTN      ConvertedStrLen;
  EFI_STRING TmpString;

  while ((InString = StrStr (InString, L"&OFFSET=")) != NULL) {
    TmpString = InString;
    
    // Jump over '&OFFSET='
    InString = InString + 8;

    Data = 0;
    BufferSize = sizeof (UINTN);
    Status = HexStringToBuf ((UINT8 *) &Data, &BufferSize, InString, &ConvertedStrLen);
    if (EFI_ERROR (Status)) {
      return FALSE;
    }
    InString = InString + ConvertedStrLen;

    if (Data == Offset) {
      *OutString = TmpString;
      return TRUE;
    }

    InString = InString + ConvertedStrLen;
  }

  return FALSE;
}

/** Wrapper for UDK function AsciiStrToUnicodeStr(), checks whether Destination
    points to a buffer of length greater or equal to Source string length.

   @param[in]   Source           A pointer to a Null-terminated ASCII string.
   @param[out]  Destination      A pointer to a Null-terminated Unicode string.
   @param[in]   DestMax          The maximum number of Destination Unicode char,
                                 including terminating null char.

   @retval   EFI_SUCCESS           String converted succesfully.
   @retval   EFI_INVALID_PARAMETER If Destination is NULL
                                   If Source is NULL
   @retval   EFI_BUFFER_TOO_SMALL  If DestMax is NOT greater than StrLen(Source).
   @retval   EFI_ACCESS_DENIED     If Source and Destination overlap.
**/
EFI_STATUS
AsciiStrToUnicodeStrWrapper (
  IN  CONST CHAR8 *Source,
  OUT CHAR16      *Destination,
  IN  UINTN        DestMax
  )
{
  UINTN SourceLen;

  if (((UINTN) Destination & BIT0) != 0) {
    return EFI_INVALID_PARAMETER;
  }
  if (Destination == NULL || Source == NULL) {
    return EFI_INVALID_PARAMETER;
  }
  SourceLen = AsciiStrnLenS (Source, DestMax);

  // Source and Destination should not overlap
  if ((((UINTN)Source >= (UINTN)Destination) && ((UINTN)Source < (UINTN)Destination + DestMax)) ||
    (((UINTN)Destination >= (UINTN)Source) && ((UINTN)Destination < (UINTN)Source + SourceLen))) {
    return EFI_ACCESS_DENIED;
  }
  if (DestMax > SourceLen) {
    AsciiStrToUnicodeStr (Source, Destination);
    return EFI_SUCCESS;
  } else {
    return EFI_BUFFER_TOO_SMALL;
  }
}
