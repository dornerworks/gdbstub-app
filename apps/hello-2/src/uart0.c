/*
 * Copyright 2015, DornerWorks
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, 
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * This data was produced by DornerWorks, Ltd. of Grand Rapids, MI, USA under
 * a DARPA SBIR, Contract Number D15PC00163.
 *
 * Expiration of SBIR Data Rights Period: 15 Jul 2021
 *
 * The Government's rights to use, modify, reproduce, release, perform,
 * display, or disclose technical data or computer software marked with this
 * legend are restricted during the period shown as provided in paragraph
 * (b)(4) of the Rights in Noncommercial Technical Data and Computer
 * Software–Small Business Innovative Research (SBIR) Program clause contained
 * in the above identified contract. No restrictions apply after the expiration
 * date shown above. Any reproduction of technical data, computer software, or
 * portions thereof marked with this legend must also reproduce the markings.
 *
 * Approved for Public Release, Distribution Unlimited.
 *
 */
 
/*
 * Description: 
 *  Simple UART driver for the Zynq7000.
 */
 
#include <simple/simple.h>

// some #defines that were borrowed from kernel/src/plat/zynq7000/machine/io.c
// see Zynq7000 TRM (UG585) B.33, "UART Controller (UART)" for details about these registers
#define UART_CONTROL                 0x00
#define UART_MODE                    0x04
#define UART_BAUD_RATE_GEN           0x18
#define UART_RCVR_FIFO_TRIGGER_LEVEL 0x20
#define UART_CHANNEL_STS             0x2C
#define UART_TX_RX_FIFO              0x30
#define UART_BAUD_RATE_DIVIDER       0x34

#define UART_CHANNEL_STS_TXEMPTY     (1U << 3)
#define UART_CHANNEL_STS_RXEMPTY	 (1U << 1)

extern seL4_Word uart0_vaddr;

#define UART_REG(x) ((volatile uint32_t *)(uart0_vaddr + (x)))

void uart0_putChar(char c)
{
    while (!(*UART_REG(UART_CHANNEL_STS) & UART_CHANNEL_STS_TXEMPTY));
    *UART_REG(UART_TX_RX_FIFO) = c;
}

static int uart0_rdyChar(void)
{
	if(	0 == (*UART_REG(UART_CHANNEL_STS) & UART_CHANNEL_STS_RXEMPTY))
		return 1;
	return 0;
}

char uart0_getChar(void)
{
	char rx;

	// wait until a character is received
    while(	0  != (*UART_REG(UART_CHANNEL_STS) & UART_CHANNEL_STS_RXEMPTY))
    	;

    rx = *UART_REG(UART_TX_RX_FIFO);

    return rx;
}

void uart0_puts(char* s)
{
	for(;*s;s++)
		uart0_putChar(*s);

}

void uart0_initChar(void)
{
	// see UG585 19.3 "Programming Guide"

	// disable Rx FIFO trigger level
	*UART_REG(UART_RCVR_FIFO_TRIGGER_LEVEL) = 0;
	*UART_REG(UART_BAUD_RATE_DIVIDER) = 0x06;
	*UART_REG(UART_BAUD_RATE_GEN) = 0x3e;
	*UART_REG(UART_CONTROL) = 0x17;
	*UART_REG(UART_MODE) = 0x20;	//TODO, try putting this before enabling in reg 0

	// purge/flush input FIFO
	while(uart0_rdyChar())
	{
		uart0_getChar();
	}

#if 0
	// a little diagnostic in case you're worried that the rx isn't working right
	{
		char rx;

		uart0_puts("Waiting for input:\r\n");
		rx = uart0_getChar();
		uart0_puts("\tread: #");
		uart0_putChar(rx);
		uart0_puts("#\r\n");
	}
#endif

}
