#include <stdint.h>
#include <sys/mman.h>

#include <cstdio>
#include <cstdlib>

#include "/home/ben/gem5/src/mem/pim.hh"
#include "clflush.h"
#include "piminst.h"

int main(){
    // assign our 2MB page (if we ran in SE mode there are
    // no 2MB pages, so we hope that we are given a continues
    // space).
    // we assign two pages, so we can be sure that the second
    // one will start at the middle of a free 2MB page
    void* addr1        = malloc(6*1024*1024);
    // since malloc apparently call the first address
    clflush(addr1);
    // try to align the address to a 2MB page.
    // first, addr2_pre is a location where we have a continues 2MB page.
    // then we mask it to get the virtual page 2MB page (this is actually not
    // scenery). the magic number 0x26000 is from debugging, the physical
    // address is not aligned to a 2MB page, so this is the offset assuming
    // the gem5 SE simulation is consistent with address assigning.
    void* addr2_pre    = (void *)((char*)addr1 + 3*1024*1024);
    void* addr2        = (void *)(((uintptr_t)addr2_pre &
                        (~(uintptr_t)pim::PAGEOFFSET_MASK))
                            - 0x26000);

    printf("addr1 : 0x%lX\n addr2_pre : 0x%lX\n addr2 : 0x%lX\n",
                (uintptr_t)addr1,(uintptr_t)addr2_pre,(uintptr_t)addr2);
    // set the opcode to communicate with the gem5 simulation.
    // the payload should be 0 for page addition
    uint64_t pim_op    = pim::setOpImm(pim::GEM5,0,0,0,0);

    unsigned a;
    printf("Add page to mPIM\n");
    pimInst(pim_op,(uint64_t)addr2);
    printf("Start writing\n");
    for ( int i = 0 ; i < 2*1024*1024 ; i++){
        *((uint8_t*)addr2 + i) = i & 0xFF;
        if (i % (64*1024) == 0){
            printf("write byte num %d\n",i);
        }
    }
    printf("Start flushing\n");
    for ( int i = 0 ; i < 2*1024*1024 ; i=i+64){
        clflush((void *)((uint8_t*)addr2 + i));
        if (i % (64*1024) == 0){
            printf("flush byte num %d\n",i);
        }
    }
    printf("Start reading\n");
    for ( int i = 0 ; i < 2*1024*1024 ; i++){
        a = (unsigned)(*((uint8_t*)addr2 + i)) & 0xFF;
        if ( a != (i & 0xFF)){
            printf("Read word %d (0x%X), bad data: 0x%X\n",i,
                (unsigned)(uintptr_t)((uint8_t*)addr2 + i),a);
            return 0;
        }
        if (i % (64*1024) == 0){
            printf("read byte num %d\n",i);
        }
    }

    return 0;
}
