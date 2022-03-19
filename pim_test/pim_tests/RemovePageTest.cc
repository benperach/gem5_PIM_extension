#include <stdint.h>
#include <sys/mman.h>

#include <cstdio>
#include <cstdlib>

#include "/home/ben/gem5/src/mem/pim.hh"
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
    uint64_t pim_add_op    = pim::setOpImm(pim::GEM5,0,0,0,0);

    pimInst(pim_add_op,(uint64_t)addr2);

    // to remove the page the paylod just need to be non zero
    uint64_t pim_remove_op = pim::setOpImm(pim::GEM5,0,0,0,1);

    pimInst(pim_remove_op,(uint64_t)addr2);
    return 0;
}
