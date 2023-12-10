/**
 * @file vmaccess.c
 * @author Prof. Dr. Wolfgang Fohl, HAW Hamburg
 * @date 2010
 * @brief The access functions to virtual memory.
 */

#include "vmaccess.h"
#include <sys/ipc.h>
#include <sys/shm.h>

#include "syncdataexchange.h"
#include "vmem.h"
#include "debug.h"
#include "error.h"

/*
 * static variables
 */

static struct vmem_struct *vmem = NULL; //!< Reference to virtual memory

/**
 * The progression of time is simulated by the counter g_count, which is incremented by 
 * vmaccess on each memory access. The memory manager will be informed by a command, whenever 
 * a fixed period of time has passed. Hence the memory manager must be informed, whenever 
 * g_count % TIME_WINDOW == 0. 
 * Based on this information, memory manager will update aging information
 */

static int g_count = 0;    //!< global acces counter as quasi-timestamp - will be increment by each memory access
#define TIME_WINDOW   20

/**
 *****************************************************************************************
 *  @brief      This function setup the connection to virtual memory.
 *              The virtual memory has to be created by mmanage.c module.
 *
 *  @return     void
 ****************************************************************************************/
static void vmem_init(void) {

    /* Create System V shared memory */

    /* We are only using the shm, don't set the IPC_CREAT flag */

    /* attach shared memory to vmem */

}

/**
 *****************************************************************************************
 *  @brief      This function puts a page into memory (if required). Ref Bit of page table
 *              entry will be updated.
 *              If the time window handle by g_count has reached, the window window message
 *              will be send to the memory manager. 
 *              To keep conform with this log files, g_count must be increased before 
 *              the time window will be checked.
 *              vmem_read and vmem_write call this function.
 *
 *  @param      address The page that stores the contents of this address will be 
 *              put in (if required).
 * 
 *  @return     void
 ****************************************************************************************/
static void vmem_put_page_into_mem(int address) {

    int page = address / VMEM_PAGESIZE;
    if(!(vmem->pt[page].flags & PTF_PRESENT)){
        //struct msg message_FlagZero = {CMD_PAGEFAULT, page, g_count, PTF_PRESENT};
        //sendMsgToMmanager(message_FlagZero);
        return;
        }
    struct msg message_FlagOne = {CMD_PAGEFAULT, page, g_count, PTF_PRESENT};
    sendMsgToMmanager(message_FlagOne);
}

unsigned char vmem_read(int address) {
    if(!g_count){
        vmem_init();
    }
    vmem_put_page_into_mem(address);
    int virtual_pageNr = address / VMEM_PAGESIZE;
    int startAddress = virtual_pageNr * VMEM_NPAGES;
    int offset = address - startAddress;
    int pageFrame = vmem->pt[virtual_pageNr].frame;

    int phyAddress = pageFrame * VMEM_PAGESIZE + offset;

    if(!(vmem->pt[virtual_pageNr].flags & PTF_PRESENT)){
        return 0;
    }
    unsigned char data = vmem->mainMemory[phyAddress];
    g_count++;
    return data;
}

void vmem_write(int address, unsigned char data) {
    if(!g_count){
        vmem_init();
    }
    vmem_put_page_into_mem(address);
    int virtual_pageNr = address / VMEM_PAGESIZE;
    int startAddress = virtual_pageNr * VMEM_NPAGES;
    int offset = address - startAddress;
    int pageFrame = vmem->pt[virtual_pageNr].frame;

    int phyAddress = pageFrame * VMEM_PAGESIZE + offset;

    if(!(vmem->pt[virtual_pageNr].flags & PTF_PRESENT)){
        return;
    }
    vmem->mainMemory[phyAddress] = data;
    g_count++;
}
// EOF
