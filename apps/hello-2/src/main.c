/*
 * Copyright 2015, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
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
 * seL4 tutorial part 2: create and run a new thread
 */

/* Include Kconfig variables. */
#include <autoconf.h>

#include <stdio.h>
#include <assert.h>

#include <sel4/sel4.h>

#include <simple/simple.h>
#include <simple-default/simple-default.h>

#include <vka/object.h>

#include <allocman/allocman.h>
#include <allocman/bootstrap.h>
#include <allocman/vka.h>

#include <vspace/vspace.h>

#include <sel4utils/vspace.h>
#include <sel4utils/mapping.h>
#include <sel4utils/process.h>


#define XUART0_BASE 0xE0000000

/* global environment variables */
seL4_Word uart0_vaddr;


/* seL4_BootInfo defined in bootinfo.h
 * https://github.com/seL4/seL4/blob/master/libsel4/include/sel4/bootinfo.h#L50 */
seL4_BootInfo *info;

/* simple_t defined in simple.h
 * https://github.com/seL4/libsel4simple/blob/master/include/simple/simple.h#L201 */
simple_t simple;

/* vka_t defined in vka.h
 * https://github.com/seL4/libsel4vka/blob/master/include/vka/vka.h#L93 */
vka_t vka;
vspace_t vspace;

/* allocaman_t defined in allocman.h
 * https://github.com/seL4/libsel4allocman/blob/master/include/allocman/allocman.h#L105 */
allocman_t *allocman;

/* static memory for virtual memory bootstrapping */
UNUSED static sel4utils_alloc_data_t data;
seL4_CPtr irq;
vka_object_t ep_object = {0};

/* static memory for the allocator to bootstrap with */
#define ALLOCATOR_STATIC_POOL_SIZE ((1 << seL4_PageBits) * 10)
UNUSED static char allocator_mem_pool[ALLOCATOR_STATIC_POOL_SIZE];

/* stack for the new thread */
#define THREAD_2_STACK_SIZE 512
static uint64_t thread_2_stack[THREAD_2_STACK_SIZE];
static volatile unsigned int key = 0;


/* defined in uart0.c */
extern void uart0_initChar(void);

/* GDB stub functions, defined in gdb-stub.c */
extern void breakpoint(void);
extern void handle_exception (unsigned int* registers, int code, int number);

/* convenience function in util.c:
 * https://github.com/sel4-projects/sel4-tutorials/blob/master/apps/hello-2/src/util.c#L33
 */
extern void name_thread(seL4_CPtr tcb, char *name);

/* Handler to catch the breakpoint exception and call the gdb-stub handler */
void exception_handler(void)
{
	UNUSED int error;

	while(1)
	{
	    seL4_Word sender;
	    seL4_IPCBuffer *ipcbuf = (seL4_IPCBuffer *)0x70000000;
	    seL4_MessageInfo_t tag;
	    tag = seL4_Wait(ep_object.cptr, &sender);

	    int code, number;
	    number = ipcbuf->msg[3];
	    code = ipcbuf->msg[4];

	    seL4_UserContext seL4_regs;
	    unsigned int regs[17];
	    seL4_TCB_ReadRegisters(seL4_CapInitThreadTCB,0,0,17,&seL4_regs);

	    //map from seL4 order to gdb order
	    regs[0] = seL4_regs.r0;
	    regs[1] = seL4_regs.r1;
	    regs[2] = seL4_regs.r2;
	    regs[3] = seL4_regs.r3;
	    regs[4] = seL4_regs.r4;
	    regs[5] = seL4_regs.r5;
	    regs[6] = seL4_regs.r6;
	    regs[7] = seL4_regs.r7;
	    regs[8] = seL4_regs.r8;
	    regs[9] = seL4_regs.r9;
	    regs[10] = seL4_regs.r10;
	    regs[11] = seL4_regs.r11;
	    regs[12] = seL4_regs.r12;
	    regs[13] = seL4_regs.sp;
	    regs[14] = seL4_regs.r14;
	    regs[15] = seL4_regs.pc;
	    regs[16] = seL4_regs.cpsr;

	    // call the gdb stub function to connect to the host
	    handle_exception(regs, number, code);

	    seL4_SetMR(0,regs[15]);	/* update PC */
	    seL4_Reply(tag);
	    seL4_TCB_Resume(seL4_CapInitThreadTCB);
	}

}
/* Map UART0 physical address, get virtual address for it, and finish configuring UART0 */
static void config_uart0(seL4_CPtr pd_cap)
{
    UNUSED int error;

    /* allocate a cslot for the frame */
    seL4_CPtr frame_cap;
    error = vka_cspace_alloc(&vka, &frame_cap);
    assert(error == 0);

    cspacepath_t frame;
    error = vka_cspace_alloc_path(&vka, &frame);

    /* find the physical frame */
    vka_cspace_make_path(&vka, frame_cap, &frame);
    error = simple_get_frame_cap(&simple, (void *)XUART0_BASE, seL4_PageBits, &frame);
    assert(error == 0);

    /* create a vspace object to manage our vspace */
    error = sel4utils_bootstrap_vspace_with_bootinfo_leaky(&vspace, &data, pd_cap, &vka, info);

    /* map in the frame */
    uart0_vaddr = (seL4_Word)vspace_map_pages(&vspace, &frame_cap, NULL, seL4_AllRights, 1, seL4_PageBits, 0);
    assert(error == 0);

    uart0_initChar();
}


int main(void)
{
    UNUSED int error;

    /* give us a name: useful for debugging if the thread faults */
    /* seL4_CapInitThreadTCB is a cap pointer to the root task's initial TCB.
     * It is part of the root task's boot environment and defined in bootinfo.h from libsel4:
     * https://github.com/seL4/seL4/blob/master/libsel4/include/sel4/bootinfo.h#L20
     */
    name_thread(seL4_CapInitThreadTCB, "hello-2");

    /* TODO 1: get boot info */
    /* hint: seL4_GetBootInfo()
     * seL4_BootInfo* seL4_GetBootInfo(void);
     * @return Pointer to the bootinfo, no failure.
     * https://github.com/seL4/seL4/blob/master/libsel4/include/sel4/bootinfo.h#L72
     */
    info = seL4_GetBootInfo();

    /* TODO 2: init simple */
    /* hint: simple_default_init_bootinfo()
     * void simple_default_init_bootinfo(simple_t *simple, seL4_BootInfo *bi);
     * @param simple Structure for the simple interface object. This gets initialised.
     * @param bi Pointer to the bootinfo describing what resources are available
     * https://github.com/seL4/libsel4simple-default/blob/master/include/simple-default/simple-default.h#L18
     */
    simple_default_init_bootinfo(&simple, info);

    /* TODO 3: print out bootinfo and other info about simple */
    /* hint: simple_print()
     * void simple_print(simple_t *simple);
     * @param simple Pointer to simple interface.
     * https://github.com/seL4/libsel4simple/blob/master/include/simple/simple.h#L199
     * https://github.com/seL4/libsel4simple/blob/master/include/simple/simple.h#L343
     */
    simple_print(&simple);

    /* TODO 4: create an allocator */
    /* hint: bootstrap_use_current_simple()
     * allocman_t *bootstrap_use_current_simple(simple_t *simple, uint32_t pool_size, char *pool);
     * @param simple Pointer to simple interface.
     * @param pool_size Size of the initial memory pool.
     * @param pool Initial memory pool.
     * @return returns NULL on error
     * https://github.com/seL4/libsel4allocman/blob/master/include/allocman/bootstrap.h#L172
     */
    allocman = bootstrap_use_current_simple(&simple, ALLOCATOR_STATIC_POOL_SIZE, allocator_mem_pool);
    assert(allocman);

    /* TODO 5: create a vka (interface for interacting with the underlying allocator) */
    /* hint: allocman_make_vka()
     * void allocman_make_vka(vka_t *vka, allocman_t *alloc);
     * @param vka Structure for the vka interface object.  This gets initialised.
     * @param alloc allocator to be used with this vka
     * https://github.com/seL4/libsel4allocman/blob/master/include/allocman/vka.h#L24
     */
    allocman_make_vka(&vka, allocman);

    /* TODO 6: get our cspace root cnode */
    /* hint: simple_get_cnode()
     * seL4_CPtr simple_get_cnode(simple_t *simple);
     * @param simple Pointer to simple interface.
     * @return The cnode backing the simple interface. no failure.
     * https://github.com/seL4/libsel4simple/blob/master/include/simple/simple.h#L275
     */
    seL4_CPtr cspace_cap;
    cspace_cap = simple_get_cnode(&simple);

    /* TODO 7: get our vspace root page diretory */
    /* hint: simple_get_pd()
     * seL4_CPtr simple_get_pd(simple_t *simple);
     * @param simple Pointer to simple interface.
     * @return The vspace (PD) backing the simple interface. no failure.
     * https://github.com/seL4/libsel4simple/blob/master/include/simple/simple.h#L293
     */
    seL4_CPtr pd_cap;
    pd_cap = simple_get_pd(&simple);


    /* TODO 8: create a new TCB */
    /* hint: vka_alloc_tcb()
     * int vka_alloc_tcb(vka_t *vka, vka_object_t *result);
     * @param vka Pointer to vka interface.
     * @param result Structure for the TCB object.  This gets initialised.
     * @return 0 on success
     * https://github.com/seL4/libsel4vka/blob/master/include/vka/object.h#L90
     */
    vka_object_t tcb_object = {0};
    error = vka_alloc_tcb(&vka, &tcb_object);
    assert(error == 0);

    /* TODO 9: initialise the new TCB */
    /* hint 1: seL4_TCB_Configure()
     * int seL4_TCB_Configure(seL4_TCB service, seL4_Word fault_ep, seL4_Uint8 priority, seL4_CNode cspace_root, seL4_CapData_t cspace_root_data, seL4_CNode vspace_root, seL4_CapData_t vspace_root_data, seL4_Word buffer, seL4_CPtr bufferFrame)
     * @param service Capability to the TCB which is being operated on.
     * @param fault_ep Endpoint which receives IPCs when this thread faults (must be in TCB's cspace).
     * @param priority The thread's new priority.
     * @param cspace_root The new CSpace root.
     * @param cspace_root_data Optionally set the guard and guard size of the new root CNode. If set to zero, this parameter has no effect.
     * @param vspace_root The new VSpace root.
     * @param vspace_root_data Has no effect on IA-32 or ARM processors.
     * @param buffer Address of the thread's IPC buffer. Must be 512-byte aligned. The IPC buffer may not cross a page boundary.
     * @param bufferFrame Capability to a page containing the thread?s IPC buffer.
     * @return 0 on success.
     * Note: this function is generated during build.  It is generated from the following definition:
     * https://github.com/seL4/seL4/blob/master/libsel4/include/interfaces/sel4.xml#L44
     * You can find out more about it in the API manual: http://sel4.systems/Info/Docs/seL4-manual.pdf
     *
     * hint 2: use seL4_CapNull for the fault endpoint
     * hint 3: use seL4_NilData for cspace and vspace data
     * hint 4: we don't need an IPC buffer frame or address yet
     */

    // need to setup an IPC buffer for the exception handler to receive info about the exception
    vka_object_t ipc_frame_object;
    error = vka_alloc_frame(&vka, 12, &ipc_frame_object);
    assert(error == 0);
    seL4_Word ipc_buffer_vaddr = 0x70000000;	// this is an arbitrary value I grabbed from the seL4 tutorial that works

    error = seL4_ARM_Page_Map(ipc_frame_object.cptr, pd_cap, ipc_buffer_vaddr,
        seL4_AllRights, seL4_ARM_Default_VMAttributes);

    if (error != 0) {
        vka_object_t pt_object;
        error =  vka_alloc_page_table(&vka, &pt_object);
        assert(error == 0);
    	error = seL4_ARM_PageTable_Map(pt_object.cptr, pd_cap,
            ipc_buffer_vaddr, seL4_ARM_Default_VMAttributes);
        assert(error == 0);

        /* TODO 5: then map the frame in */
        /* hint 1: use seL4_ARCH_Page_Map() as above
         * hint 2: for the rights, use seL4_AllRights
         * hint 3: for VM attributes use seL4_ARCH_Default_VMAttributes
         */
        error = seL4_ARM_Page_Map(ipc_frame_object.cptr, pd_cap,
            ipc_buffer_vaddr, seL4_AllRights, seL4_ARCH_Default_VMAttributes);
        assert(error == 0);
    }

    /* set the IPC buffer's virtual address in a field of the IPC buffer */
    seL4_IPCBuffer *ipcbuf = (seL4_IPCBuffer*)ipc_buffer_vaddr;
    ipcbuf->userData = ipc_buffer_vaddr;

    error = seL4_TCB_Configure(tcb_object.cptr, seL4_CapNull, seL4_MaxPrio, cspace_cap, seL4_NilData, pd_cap, seL4_NilData, ipc_buffer_vaddr, ipc_frame_object.cptr);
    assert(error == 0);


    /* TODO 10: give the new thread a name */
    /* hint: we've done thread naming before */
    name_thread(tcb_object.cptr, "hello-2: exception_handler");

    /*
     * set start up registers for the new thread:
     */

    seL4_UserContext regs = {0};

    /* TODO 11: set instruction pointer where the thread shoud start running */
    /* hint 1: sel4utils_set_instruction_pointer()
     * void sel4utils_set_instruction_pointer(seL4_UserContext *regs, seL4_Word value);
     * @param regs Data structure in which to set the instruction pointer value
     * @param value New instruction pointer value
     * https://github.com/seL4/libsel4utils/blob/master/arch_include/x86/sel4utils/arch/util.h#L30
     *
     * hint 2: we want the new thread to run the function "thread_2"
     */
    sel4utils_set_instruction_pointer(&regs, (seL4_Word)exception_handler);

    /* check that stack is aligned correctly */
    uintptr_t thread_2_stack_top = (uintptr_t)thread_2_stack + sizeof(thread_2_stack);
    assert(thread_2_stack_top % (sizeof(seL4_Word) * 2) == 0);

    /* TODO 12: set stack pointer for the new thread */
    /* hint 1: sel4utils_set_stack_pointer()
     * void sel4utils_set_stack_pointer(seL4_UserContext *regs, seL4_Word value);
     * @param regs  Data structure in which to set the stack pointer value
     * @param value New stack pointer value
     * https://github.com/seL4/libsel4utils/blob/master/arch_include/x86/sel4utils/arch/util.h#L42
     *
     * hint 2: remember the stack grows down!
     */
    sel4utils_set_stack_pointer(&regs, thread_2_stack_top);

    /* TODO 13: actually write the TCB registers.  We write 2 registers:
     * instruction pointer is first, stack pointer is second. */
    /* hint: seL4_TCB_WriteRegisters()
     * int seL4_TCB_WriteRegisters(seL4_TCB service, seL4_Bool resume_target, seL4_Uint8 arch_flags, seL4_Word count, seL4_UserContext *regs)
     * @param service Capability to the TCB which is being operated on.
     * @param resume_target The invocation should also resume the destination thread.
     * @param arch_flags Architecture dependent flags. These have no meaning on either IA-32 or ARM.
     * @param count The number of registers to be set.
     * @param regs Data structure containing the new register values.
     * @return 0 on success
     *
     * Note: this function is generated during build.  It is generated from the following definition:
     * https://github.com/seL4/seL4/blob/master/libsel4/include/interfaces/sel4.xml#L30
     * You can find out more about it in the API manual: http://sel4.systems/Info/Docs/seL4-manual.pdf
     */
    error = seL4_TCB_WriteRegisters(tcb_object.cptr, 0, 0, 2, &regs);
    assert(error == 0);

    /* TODO 14: start the new thread running */
    /* hint: seL4_TCB_Resume()
     * int seL4_TCB_Resume(seL4_TCB service)
     * @param service Capability to the TCB which is being operated on.
     * @return 0 on success
     *
     * Note: this function is generated during build.  It is generated from the following definition:
     * https://github.com/seL4/seL4/blob/master/libsel4/include/interfaces/sel4.xml#L69
     * You can find out more about it in the API manual: http://sel4.systems/Info/Docs/seL4-manual.pdf
     */
    error = vka_alloc_endpoint(&vka, &ep_object);
    assert(error == 0);

    error = seL4_TCB_SetSpace(seL4_CapInitThreadTCB, ep_object.cptr, seL4_CapInitThreadCNode, seL4_NilData, seL4_CapInitThreadVSpace, seL4_NilData);
    assert(error == 0);

    error = seL4_TCB_Resume(tcb_object.cptr);
    assert(error == 0);


    /* we are done, say hello */
    printf("main: hello world\n");

    config_uart0(pd_cap);


    breakpoint();

    printf("\n\tthe 1st key is: %x\n\n",key);

    key = 0xDEADBEEFU;
    printf("\n\tthe 2nd key is: %x\n\n",key);

    key = 0x1234;
    printf("\n\tthe 3rd key is: %x\n\n",key);


    /* never exit */
    while(1)
    {
        key++;
        printf("\n\tthe nth key is: %x\n\n",key);
        while(key > 100);
    }
    
    return 0;
}
