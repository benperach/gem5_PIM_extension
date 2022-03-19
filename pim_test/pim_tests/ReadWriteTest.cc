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
    void* addr1        = (char*) malloc(2*1024*1024);
    void* addr2        = (char*) malloc(2*1024*1024);
    // set the opcode to communicate with the gem5 simulation.
    // the payload should be 0 for page addition
    uint64_t pim_op    = pim::setOpImm(pim::GEM5,0,0,0,0);

    pimInst(pim_op,(uint64_t)addr2);

    // since malloc apparently call the first address
    clflush(addr2);
    uint64_t a = *(char*)addr2;
    *(char*)addr2 = 0xF;
    clflush(addr2);
    a = *(char*)addr2;

    return 0;
}
