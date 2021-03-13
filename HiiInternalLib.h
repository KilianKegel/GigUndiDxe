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
#ifndef HII_INTERNAL_LIB_H_
#define HII_INTERNAL_LIB_H_

#define BIT0     0x00000001


/** Return the pointer to the section of the Request string that begins just
   after the header.

   @param[in]   Request   A request string starting with "GUID="

   @return     Pointer to request string section after header or NULL if Request
               string ends before reaching it
**/
EFI_STRING
SkipConfigHeader (
  OUT EFI_STRING Request
  );

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
  );

/** Find next element in the Request string and return its parameters.

   @param[in]    Request   a request string
   @param[out]   ElementOffset   the offset of element
   @param[out]   ElementWidth   the width of the element

   @return       EFI_STRING on current position in the Request string
**/
EFI_STRING
GetNextRequestElement (
  EFI_STRING Request,
  UINTN *    ElementOffset,
  UINTN *    ElementWidth
  );


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
  );

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
  IN OUT UINT8 *                                    Buf,
  IN OUT UINTN *                                    Len,
  IN     CHAR16 *                                   Str,
  OUT    UINTN                     *ConvertedStrLen OPTIONAL
  );

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
  );

/** Wrapper for the UDK AsciiStrToUnicodeStrS() function with an additional
    runtime check for destination address alignment.

   @param[in]   Source        A pointer to a Null-terminated ASCII string.
   @param[out]  Destination   A pointer to a Null-terminated Unicode string.
   @param[in]   DestMax       The maximum number of Destination Unicode char,
                              including the terminating null char.

   @retval   EFI_SUCCESS           String converted succesfully.
   @retval   EFI_INVALID_PARAMETER Source or Destination is NULL, or DestMax is 0.
   @retval   EFI_BUFFER_TOO_SMALL  DestMax is NOT greater than StrLen(Source).
   @retval   EFI_ACCESS_DENIED     Source and Destination overlap.
**/
EFI_STATUS
AsciiStrToUnicodeStrWrapper (
  IN CONST CHAR8 *Source,
  OUT CHAR16     *Destination,
  IN UINTN        DestMax
  );

#endif /* HII_INTERNAL_LIB_H_ */
