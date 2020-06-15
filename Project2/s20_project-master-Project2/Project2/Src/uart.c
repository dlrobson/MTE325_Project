/**
 * COPYRIGHT(c) 2014 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

#include "uart.h"

/**
 * @brief  Send a text string via USART.
 * @param  huart       pointer to a UART_HandleTypeDef structure that contains
 *                     the configuration information for the specified UART module.
 * @param  TextString  The text string to be sent.
 * @note It use the HAL_UART_Transmit function.
 */
void USART_Transmit(UART_HandleTypeDef* huart, uint8_t* TextString) {
	uint8_t TextStringLength;

	/* Calculate the length of the text string to be sent */
	TextStringLength = 0;
	while (TextString[TextStringLength++] != '\0')
		;
	TextStringLength--;

	/* Use the HAL function to send the text string via USART */
	HAL_UART_Transmit(huart, TextString, TextStringLength, 10);
}

/**
 * @brief  Convert a number nbr into a string str with 7 characters.
 * @param  nbr The number to be converted.
 * @param  str The container of the converted number into a text in decimal
 *         format.
 * @note   The decimal digits of the number must be maximum 7 so str has to be
 *         able to store at least 7 characters plus '\0'.
 */
void num2str(uint32_t nbr, uint8_t *str) {
	uint8_t k;
	uint8_t *pstrbuff;
	uint32_t divisor;

	pstrbuff = str;

	/* Reset the text string */
	for (k = 0; k < 7; k++)
		*(pstrbuff + k) = '\0';

	divisor = 1000000;

	if (nbr) // if nbr is different from zero then it is processed
	{
		while (!(nbr / divisor)) {
			divisor /= 10;
		}

		while (divisor >= 10) {
			k = nbr / divisor;
			*pstrbuff++ = '0' + k;
			nbr = nbr - (k * divisor);
			divisor /= 10;
		}
	}

	*pstrbuff++ = '0' + nbr;
	*pstrbuff++ = '\0';
}

/**
 * @brief  Convert an integer number into hexadecimal format.
 *
 * @param  num         The integer number to convert.
 * @param  HexFormat   The output format about hexadecimal number.
 *
 * @retval uint8_t*    The address of the string text for the converted hexadecimal number.
 */
uint8_t* num2hex(uint32_t num, eHexFormat HexFormat) {
	static uint8_t HexValue[8 + 1];
	uint8_t i;
	uint8_t dummy;
	uint8_t HexDigits = 0;

	switch (HexFormat) {
	case HALFBYTE_F:
		HexDigits = 1;
		break;
	case BYTE_F:
		HexDigits = 2;
		break;
	case WORD_F:
		HexDigits = 4;
		break;
	case DOUBLEWORD_F:
		HexDigits = 8;
		break;
	}

	for (i = 0; i < HexDigits; i++) {
		HexValue[i] = '\0';
		dummy = (num & (0x0F << (((HexDigits - 1) - i) * 4)))
				>> (((HexDigits - 1) - i) * 4);
		if (dummy < 0x0A) {
			HexValue[i] = dummy + '0';
		} else {
			HexValue[i] = (dummy - 0x0A) + 'A';
		}
	}
	HexValue[i] = '\0';

	return HexValue;
}
