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
 *
 * Adapted from sparc-stub.c, offered to public domain by HP. 
 * https://sourceware.org/git/gitweb.cgi?p=binutils-gdb.git;a=blob;f=gdb/stubs/sparc-stub.c;h=c12d4360a4bb413f9e3052061ace07f2b15a0d30;hb=HEAD
 *
 */
 
/*
 * Description: 
 *  This is a GDB debugging stub for seL4 applications running on a ARMv7a 
 *  processors. This stub provides implements the target side Remote Serial 
 *  Protocol, allowing the host GDB to control the target.
 *  https://sourceware.org/gdb/onlinedocs/gdb/Remote-Protocol.html
 *
 *  For seL4, an exception handling thread has to be created which, when an
 *  exception is caught, calls handle_exception().
 * 
 *  Also, an initial call to breakpoint() is needed in the application being
 *  debugged to pass control to GDB to allow for setting of other breakpoints.
 *
 */
 
#include <stdio.h>
#include <string.h>
#include <sel4/sel4.h>
#include <assert.h>

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned long u32;
typedef enum {R0,R1,R2,R3,R4,R5,R6,R7,R8,R9,R10,R11,R12,SP,LR,PC,NUMREGS} ARM_REG;     // Other architectures will have different register sets 
typedef enum { false, true } bool;
#define CPSR NUMREGS

extern void uart0_putChar(char);			/* write a single character      */
extern char uart0_getChar(void);			/* read and return a single char */
extern void breakinst(void);

#define putDebugChar(x) uart0_putChar(x)
#define getDebugChar 	uart0_getChar

#define BUFMAX 			2048
#define BAD_ARG			0xFF

static const char hexchars[16]="0123456789abcdef";

static char remcomInBuffer[BUFMAX];
static char remcomOutBuffer[BUFMAX];

/* variables for single step */
static u32* stepOpPtr;
static u32  nextCode = 0;
static bool stepped = false;


#define TRAP 	0xfedeffe7
#define NUM_BP 50
typedef struct{u32 opcode; u32 pc;} break_point_t;
break_point_t breakpoints[NUM_BP] = {{0}};

#undef DEBUG

#ifdef DEUBG
#define dbg_printf(...) printf(__VA_ARGS__)
#else
#define dbg_printf(...)
#endif


// flush the prefetch engine
static inline void prefetch_flush(void)
{
	asm volatile("mcr p15, 0, %0, c7, c5, 4" :: "r"(0) : "memory");
}

// memory barrier to ensure things are synced up
static inline void mb(void)
{
	asm("DSB\n");	// allow all previous instructions to complete
	asm("DMB\n");	// allow all previous memory transactions to compelte
}

// clear out caches to ensure that any changes to memory will impact future execution
static inline void icache_sync(unsigned long addr, size_t len)
{
	int error;

	prefetch_flush();

	// have to rely on seL4 mechanisms to flush the cache because this thread only runs with user permissions,
	//   and you need supervisor rights to run cache flush opcodes
	error = seL4_ARM_PageDirectory_CleanInvalidate_Data(seL4_CapInitThreadVSpace,  addr, addr+len);
	assert(error == 0);
	error = seL4_ARM_PageDirectory_Unify_Instruction(seL4_CapInitThreadVSpace, addr, addr+len);
	assert(error == 0);

	mb();               // memory barrier
    asm("ISB\n");       // flush instruction pipeline
}


// remove the breakpoint, restore original opcode
static int del_bp(u32 pc)
{
	int i;
	u32* pc_ptr;
	for(i = 0; i < NUM_BP; i++)
	{
		if(pc == breakpoints[i].pc)
		{
			pc_ptr = (u32*)pc;
			*pc_ptr = breakpoints[i].opcode;

			breakpoints[i].pc = 0;
			breakpoints[i].opcode = 0;
			return 0;
		}
	}
	return -1;
}

// look to see if a breakpoint has been set at this address and what the opcode is
// returns 0xFFFFFFFF if no breakpoint found for the address
static u32 find_bp(u32 pc)
{
	int i;

	for(i = 0; i < NUM_BP; i++)
	{
		if(pc == breakpoints[i].pc)
		{
			u32 opcode = breakpoints[i].opcode;
			return opcode;
		}
	}
	return 0xFFFFFFFF;
}

// set a breakpoint at the address in PC
int set_bp(u32 pc, u32 opcode)
{
	int i;
	int slot = NUM_BP;
	for(i = 0; i < NUM_BP; i++)
	{
		if(breakpoints[i].opcode == 0 && breakpoints[i].pc == 0)
			slot = i;
		else if(breakpoints[i].pc == pc)
		{
			//breakpoint at this PC was found, should have the TRAP opcode already set
			return 0;
		}
	}
	if(slot < NUM_BP)
	{
		breakpoints[slot].pc = pc;
		breakpoints[slot].opcode = opcode;
		return 0;
	}
	return -1;
}

/* Convert ch from a hex digit to a numeric value, valid outputs are 0 through 15 */
static u8 hex (char ch)
{
	if (ch >= 'a' && ch <= 'f')
		return ch-'a'+10;
	if (ch >= '0' && ch <= '9')
		return ch-'0';
	if (ch >= 'A' && ch <= 'F')
		return ch-'A'+10;

	return BAD_ARG;
}

//converts a series of bytes (as integers) from memory into a string)
static char* mem2hex (u8* mem, char* string, int count)
{
	unsigned char ch;

	// other stub implementations install a handler for mem execptions here
	while (count-- > 0)
	{
		ch = *mem++;
		*string++ = hexchars[ch >> 4];
		*string++ = hexchars[ch & 0xf];
	}
	// other stub implementations uninstall the mem execption handler here

	*string = 0;


	return string;
}

//converts a string into a series of bytes (as integers) in memory
static u8* hex2mem (char *string, u8* mem, int count)
{
	int i;
	unsigned int ch;

	// other stub implementations install a handler for mem execptions here
	for (i=0; i<count && 0 != *string; i++)
	{
		ch = hex(*string++) << 4;
		ch |= hex(*string++);
		*mem++ = (unsigned char)ch&0xFF;
	}
	// other stub implementations uninstall the mem execption handler here

	return mem;
}

// walk a string, converting each character into a number and building up a total value
static int hexToInt(char** ptr, u32* intValue)
{
	int numChars = 0;
	u8 hexValue;

	*intValue = 0;

	while (**ptr)
	{
		hexValue = hex(**ptr);
		if (BAD_ARG == hexValue)
			break;

		*intValue = (*intValue << 4) | hexValue;
		numChars++;

		(*ptr)++;
	}

	return (numChars);
}

// assembles incoming characters into a packet, does the checksum validation over it
static char* getpacket (void)
{
	char *buffer = remcomInBuffer;
	u8 checksum;
	u8 xmitcsum;
	int count;
	char ch;

	while (1)
	{
		/* wait around for the start character, ignore all other characters */
		while ((ch = getDebugChar ()) != '$')
			;

		retry:
		checksum = 0;
		xmitcsum = -1;
		count = 0;

		/* now, read until a # or end of buffer is found */
		while (count < BUFMAX - 1)
		{
			ch = getDebugChar ();
			if (ch == '$')
				goto retry;
			if (ch == '#')
				break;
			checksum = checksum + ch;
			buffer[count] = ch;
			count = count + 1;
		}
		buffer[count] = 0;

		if (ch == '#')
		{
			ch = getDebugChar ();
			xmitcsum = hex (ch) << 4;
			ch = getDebugChar ();
			xmitcsum += hex (ch);

			if (checksum != xmitcsum)
			{
				putDebugChar ('-');	/* failed checksum */
			}
			else
			{
				putDebugChar ('+');	/* successful transfer */

				/* if a sequence char is present, reply the sequence ID */
				if (buffer[2] == ':')
				{
					putDebugChar (buffer[0]);
					putDebugChar (buffer[1]);
					dbg_printf("<<$%s#\n",buffer+3);
					return &buffer[3];
				}
				dbg_printf("<<$%s#\n",buffer);
				return &buffer[0];
			}
		}
	}

	return NULL; // can't get here
}

/* calculates checksum of message in buffer and sends it to host */
static void putpacket (char *buffer)
{
	u8 checksum;
	int count;
    char ch;

	/*  $<packet info>#<checksum>. */
	do
	{
		putDebugChar('$');
		checksum = 0;
		count = 0;

		while (0 != (ch = buffer[count]))
		{
			putDebugChar(ch);
			checksum += ch;
			count += 1;
		}

		putDebugChar('#');
		putDebugChar(hexchars[checksum >> 4]);
		putDebugChar(hexchars[checksum & 0xf]);

	}
	while (getDebugChar() != '+');

	dbg_printf(">>$%s#\n",buffer);
}

/* puts the register offset and value in that register in the buffer */
static void put_reg_value(int reg, void* p_value, char** ptr, int size)
{
	**ptr = hexchars[reg >> 4];
	(*ptr)++;
	**ptr = hexchars[reg & 0xf];
	(*ptr)++;
	**ptr = ':';
	(*ptr)++;
	*ptr = mem2hex((u8*)p_value, *ptr, size);
}

// This is the main function for handling GDB packets from the host
// Will loop in this function expecting GDB packets unless commanded to continue 'c' or step 's'
void handle_exception (unsigned int* registers, int number, int code)
{
	int sigval;
	u32 addr;
	u32 length;
	char *ptr;
	sigval = 5;	//todo, figure out what code/numbers are for the other things

	dbg_printf("handling exception PC: %x SP: %x LR: %x CPSR: %x\n",registers[PC],registers[SP],registers[LR], registers[CPSR]);

	ptr = remcomOutBuffer;

	if(stepped)
	{
		u32* opPtr;

		// sanity check
		opPtr = (u32*)registers[PC];
		if(opPtr != stepOpPtr)	// this shouldn't happen, but if it does, pop a message so someone knows something goofy is going ton
			printf("!GDB-STUB!, PC != opPtr %x %lx,  ", registers[PC], (u32)stepOpPtr);

		// if just single stepped, need to restore the TRAP with the opcode it replaced
		*stepOpPtr = nextCode;
		icache_sync((u32)stepOpPtr, 4);

		stepped = false;
		*ptr++ = 'S';
		*ptr++ = hexchars[sigval >> 4];
		*ptr++ = hexchars[sigval & 0xf];
	}
	else
	{
		u32* pc_ptr = (u32*)registers[PC];
		u32 opcode = find_bp(registers[PC]);
		if(0xFFFFFFFF != opcode)
		{
			*pc_ptr = opcode;
    		icache_sync((u32)stepOpPtr, 4);
		}
		*ptr++ = 'T';
		*ptr++ = hexchars[sigval >> 4];
		*ptr++ = hexchars[sigval & 0xf];

		put_reg_value(PC, &registers[PC], &ptr, 4);
		*ptr++ = ';';
		put_reg_value(SP, &registers[SP], &ptr, 4);
		*ptr++ = ';';
	}
	*ptr++ = 0;

	putpacket(remcomOutBuffer);

	while (1)	// stay in this loop, forever... or until a single step or continue command returns out of it
	{
		remcomOutBuffer[0] = 0;

		ptr = getpacket();
		switch (*ptr++)
		{
		case '?':
			remcomOutBuffer[0] = 'S';
			remcomOutBuffer[1] = hexchars[sigval >> 4];
			remcomOutBuffer[2] = hexchars[sigval & 0xf];
			remcomOutBuffer[3] = 0;
			break;

		case 'p':  // read a register value
            mb();
			if(hexToInt(&ptr, &addr))
			{
				if(addr == 25)
					mem2hex((u8*)&registers[CPSR],remcomOutBuffer,4);
				else if(addr < NUMREGS)
					mem2hex((u8*)&registers[addr],remcomOutBuffer,4);
				else
					strcpy(remcomOutBuffer,"xxxxxxxx");

				break;
			}
			strcpy(remcomOutBuffer,"E01");

			break;

		case 'v':
			if(0 == strncmp(remcomInBuffer,"vCont?",6))
				;//strcpy(remcomOutBuffer,"vCont;c");
			break;

		case 'H':

			if(0 == strncmp(remcomInBuffer,"Hc-1",4)
				|| 0 == strncmp(remcomInBuffer,"Hg-1",4)
				|| 0 == strncmp(remcomInBuffer,"Hc0",3)
				|| 0 == strncmp(remcomInBuffer,"Hg0",3))
				strcpy(remcomOutBuffer,"OK");
			else
				strcpy(remcomOutBuffer,"E01");
			break;

		case 'q':

			if('C' == *ptr)
				;
			else if(0 == strncmp(remcomInBuffer,"qSupported",10))
				;
			else if(0 == strncmp(remcomInBuffer,"qAttached",9))
				strcpy(remcomOutBuffer,"1");
			else if(0 == strncmp(remcomInBuffer,"qTStatus",8))
				;//strcpy(remcomOutBuffer,"T0;tnotrun:0");
			else if(0 == strncmp(remcomInBuffer,"qOffsets",8))
				; //strcpy(remcomOutBuffer,"Text=0;Data=0;Bss=0;");
			else if(0 == strncmp(remcomInBuffer,"qSymbol::",9))
				strcpy(remcomOutBuffer,"OK");
			else
				strcpy(remcomOutBuffer,"E01");

			break;

		case 'g':		/* return the value of the CPU registers */
            mb();
            mem2hex ((u8*) registers, remcomOutBuffer, (NUMREGS)*4);

			break;

		case 'z':
			if ((*ptr++ =='0') && (*ptr++ ==','))
			{
				if (hexToInt(&ptr, &addr))
					if(*ptr++ == ',')
						if(*ptr++ == '4')
						{
							if(del_bp(addr))
							{
								strcpy(remcomOutBuffer,"E03");	//todo, figure out what the proper code for out of breakpoints is
								break;
							}
							icache_sync(addr, 4);

							strcpy(remcomOutBuffer,"OK");
							break;
						}
			}
			strcpy(remcomOutBuffer,"E01");

			break;

		case 'Z':/* Z0,<addr>,4 */
			if ((*ptr++ =='0') && (*ptr++ ==','))
			{
				if (hexToInt(&ptr, &addr))
					if(*ptr++ == ',')
						if(*ptr++ == '4')
						{
							u32* pc_ptr = (u32*)addr;

							// ignore breakpoints set of current PC, already one there or we wouldn't be here
                            if(addr != registers[PC])
                            {
							    if(set_bp((u32)pc_ptr, *pc_ptr))
							    {
								    strcpy(remcomOutBuffer,"E03");	//todo, figure out what the proper code for out of breakpoints is
								    break;
							    }
							    *pc_ptr = TRAP;
                                icache_sync(addr, 4);
                            }

							strcpy(remcomOutBuffer,"OK");
							break;
						}
			}
			strcpy(remcomOutBuffer,"E01");

			break;

		case 'm':	  /* mAA..AA,LLLL  Read LLLL bytes at address AA..AA */
			/* Try to read %x,%x.  */
            mb();
			if (hexToInt(&ptr, &addr)
					&& *ptr++ == ','
							&& hexToInt(&ptr, &length))
			{
			    // hard coded address checks to prevent virtual memory exceptions, this would need to be updated if the example application changes
				if((addr >= 0x8000) && (addr < 0x13a000) && ((addr+length) < 0x13a000))
					if (mem2hex((u8*)addr, remcomOutBuffer, length))
						break;

				strcpy (remcomOutBuffer, "E03");
			}
			else
				strcpy(remcomOutBuffer,"E01");

			break;

		case 'M': /* MAA..AA,LLLL: Write LLLL bytes at address AA.AA return OK */
			/* Try to write '%x,%x:'.  */
			if (hexToInt(&ptr, &addr)
					&& *ptr++ == ','
							&& hexToInt(&ptr, &length)
							&& *ptr++ == ':')
			{
			    // hard coded address checks to prevent virtual memory exceptions, this would need to be updated if the example application changes
				if((addr >= 0x8000) && (addr < 0x13a000) && ((addr+length) < 0x13a000))
					if (hex2mem(ptr, (u8*)addr, length))
					{
						icache_sync(addr, length);
						strcpy(remcomOutBuffer, "OK");
						break;
					}
				strcpy(remcomOutBuffer, "E03");
			}
			else
				strcpy(remcomOutBuffer, "E02");
			break;

		case 's':
		{
			u32* opPtr;

			// when single stepping, save off the next opcode and replace it with the TRAP
			// todo, this doesn't always work, e.g., when the next opcode isn't going to be executed next because of branch instructions
			opPtr = (u32*)(registers[PC]+4);
			stepOpPtr= opPtr;
			nextCode = *opPtr;
			*opPtr = TRAP;

			if(registers[PC] == (u32)breakinst)	//if already sitting at a breakpoint, have to advance PC to next instruction
				registers[PC]+=4;

			icache_sync((u32)opPtr, 4);

			stepped = true;

			// break out of the while(1) loop and run free
			return;
		}

		case 'c':    /* cAA..AA    Continue at address AA..AA(optional) */
			/* try to read optional parameter, pc unchanged if no parm */
			if (hexToInt(&ptr, &addr))
			{
				registers[PC] = addr;
			}
			else
			{
				if(registers[PC] == (u32)breakinst)
					registers[PC]+=4;
			}

			// break out of the while(1) loop and run free
			return;

		case 'G':	   /* set the value of the CPU registers - return OK */
			//break;
		case 'd':		/* toggle debug flag */
			//break;
		case 'k' :		/* Kill the program */
			//break;
		case 'r':		/* Reset */
			//break;
		default:
			dbg_printf("'%s' Not implemented!\n",remcomInBuffer);
		}/* end switch */

		/* reply to the request */
		putpacket(remcomOutBuffer);
	}
}


/* initial breakpoint that has to be called from the application when using gdb-stubs */
void breakpoint(void)
{
	asm(".global breakinst");
	asm("breakinst:");
	asm(".word 0xfedeffe7\n");
}
