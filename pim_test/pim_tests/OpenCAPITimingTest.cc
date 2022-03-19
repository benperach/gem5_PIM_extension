#include <stdint.h>
#include <sys/mman.h>

#include <cstdio>
#include <cstdlib>

#include "/home/ben/gem5/src/mem/pim.hh"
#include "clflush.h"
#include "piminst.h"

/*
##############
### README ###
##############

This test runs a read, write and Op going to the mPIM, in order to
measure the time of the interface to the mPIM. This interface is based
on the OpenCAPI interface, however it does not simulate it exactly (it is
based on the header sizes and the interface bandwidth, see documentation).
To test the interface, the gem5 debug flags of DRAM and MPIM should be
set, and the outputs are used to measured the time. The measurements
are done manually, since the output can be interleaved, making the output
complicated to extract using a script.
*/

int main(){
    // assign our 2MB page (if we ran in SE mode there are
    // no 2MB pages, so we hope that we are given a continues
    // space).
    // we assign more than two pages, so we can be sure that the
    // second one will start at the middle of a free 2MB page
    void* addr1        = malloc(6*1024*1024);
    // since malloc apparently call the first address
    clflush(addr1);
    // try to align the address to a 2MB page.
    // first, addr2_pre is a location where we have a continues 2MB page.
    // the magic number 0x105010 is from debugging, the physical
    // address is not aligned to a 2MB page, so this is the offset assuming
    // the gem5 SE simulation is consistent with address assigning.
    // In FS mode we will not need this, since the 2MB vpages will be
    // aligned in the first place.
    void* addr2_pre    = (void *)((uint8_t*)addr1 + 4*1024*1024);
    void* addr2        = (void *)((uint8_t*)addr2_pre
                            - 0x105010);

    printf("addr1 : 0x%lX\naddr2_pre : 0x%lX\naddr2 : 0x%lX\n",
                (uintptr_t)addr1,(uintptr_t)addr2_pre,(uintptr_t)addr2);
    // set the opcode to communicate with the gem5 simulation.
    // the payload should be 0 for page addition
    uint64_t pim_op    = pim::setOpImm(pim::GEM5,0,0,0,0);

    printf("Add page to mPIM\n");
    pimInst(pim_op,(uint64_t)addr2);

    //write and flush to mPIM
    *(uint8_t*)addr2 = 0x11;
    clflush(addr2);

    printf("Sending mPIM ScanEq\n");
    pim_op = pim::setOpImm(pim::SCAN_EQ,0,0,1,1);
    pimInst(pim_op,(uint64_t)addr2);

    *(uint8_t*)addr2 = 0x11;
    clflush(addr2);

    return 0;
}
