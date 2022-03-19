#include <stdint.h>
#include <sys/mman.h>

#include <cstdio>
#include <cstdlib>

#include "/home/ben/gem5/src/mem/pim.hh"
#include "ColumnAccess.hh"
#include "clflush.h"
#include "piminst.h"

/*
##############
### README ###
##############

This test runs all the mPIM operations in several configurations
in order to test the timing of the mPIM ops.
To verify the results, the gem5 simulation should have the mPIMop
debug flag set. Using the output of the gem5 debug, a python script
(located in the script folder) verify the correctness of the timing.
The actual plots of of the mPIMop debug flags that are needed are
the op details and timing (see the python script for the lines that
it searches for), so you can disable the rest of the plots to make
the gem5 debug output file smaller (it will work with a big file as
well, but it will take a longer time and the debug file might be huge).
*/

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

    unsigned a;
    printf("Add page to mPIM\n");
    pimInst(pim_op,(uint64_t)addr2);

    //write and flush to mPIM for debugging
    *(uint8_t*)addr2 = 0x11;
    clflush(addr2);

    int NumOfCols = 1;
    int firstCol = 0;
    int resultCol = 2;
    uint8_t result;

    columnOperate colOp = columnOperate();
    void* opAddr = (void*)(
        (uintptr_t)addr2 + colOp.ADDR_COL_NUM_ARRAY[resultCol]
        );

    for (int len = 0; len < pim::BYTE_SIZE ; len++){
        printf("Sending mPIM ScanEq(0x%X)\n",len);
        pim_op = pim::setOpImm(pim::SCAN_EQ,0,0,len,(uint8_t)len);
        pimInst(pim_op,(uint64_t)opAddr);
    }
    for (int len = 0; len < pim::BYTE_SIZE ; len++){
        printf("Sending mPIM ScanNotEq(0x%X)\n",len);
        pim_op = pim::setOpImm(pim::SCAN_NEQ,0,0,len,(uint8_t)len);
        pimInst(pim_op,(uint64_t)opAddr);
    }
    for (int len = 0; len < pim::BYTE_SIZE ; len++){
        printf("Sending mPIM ScanLT(0x%X)\n",len);
        pim_op = pim::setOpImm(pim::SCAN_LT,0,0,len,(uint8_t)len);
        pimInst(pim_op,(uint64_t)opAddr);
    }
    for (int len = 0; len < pim::BYTE_SIZE ; len++){
        printf("Sending mPIM ScanLTEQ(0x%X)\n",len);
        pim_op = pim::setOpImm(pim::SCAN_LTEQ,0,0,len,(uint8_t)len);
        pimInst(pim_op,(uint64_t)opAddr);
    }
    for (int len = 0; len < pim::BYTE_SIZE ; len++){
        printf("Sending mPIM ScanGT(0x%X)\n",len);
        pim_op = pim::setOpImm(pim::SCAN_GT,0,0,len,(uint8_t)len);
        pimInst(pim_op,(uint64_t)opAddr);
    }
    for (int len = 0; len < pim::BYTE_SIZE ; len++){
        printf("Sending mPIM ScanGTEQ(0x%X)\n",len);
        pim_op = pim::setOpImm(pim::SCAN_GTEQ,0,0,len,(uint8_t)len);
        pimInst(pim_op,(uint64_t)opAddr);
    }
    for (int len = 0; len < pim::BYTE_SIZE ; len++){
        printf("Sending mPIM ScanBWTN(0,0x%X)\n",len);
        pim_op = pim::setOpTwoImm(pim::SCAN_BTWN,0,0,len,0,(uint8_t)len);
        pimInst(pim_op,(uint64_t)opAddr);
    }
    for (unsigned len = 0 ; len < pim::BYTE_SIZE ; len++){
        printf("Sending mPIM ReduAdd with length %d\n",len);
        pim_op    = pim::setOpImm(pim::REDU_SUM,0,0,len,0);
        pimInst(pim_op,(uint64_t)opAddr);
    }
    for (unsigned len = 0 ; len < pim::BYTE_SIZE ; len++){
        printf("Sending mPIM ReduMin with length %d\n",len);
        pim_op = pim::setOpImm(pim::REDU_MIN,0,0,len,0);
        pimInst(pim_op,(uint64_t)opAddr);
    }
    for (unsigned len = 0 ; len < pim::BYTE_SIZE ; len++){
        printf("Sending mPIM ReduMax with length %d\n",len);
        pim_op = pim::setOpImm(pim::REDU_MAX,0,0,len,0);
        pimInst(pim_op,(uint64_t)opAddr);
    }
    for (unsigned len = 0 ; len < pim::BYTE_SIZE ; len++){
        printf("Sending mPIM ColImmAdd(0x%X)\n",len);
        pim_op = pim::setOpImm(pim::COLIMM_ADD,0,0,len,(uint8_t)len);
        pimInst(pim_op,(uint64_t)opAddr);
    }
    for (unsigned len = 0 ; len < pim::BYTE_SIZE ; len++){
        printf("Sending mPIM ColImmSub(0x%X)\n",len);
        pim_op = pim::setOpImm(pim::COLIMM_SUB,0,0,len,(uint8_t)len);
        pimInst(pim_op,(uint64_t)opAddr);
    }
    for (unsigned len = 0 ; len < pim::BYTE_SIZE ; len++){
        printf("Sending mPIM ColImmSet(0x%X)\n",len);
        pim_op = pim::setOpImm(pim::COLIMM_SET,0,0,len,len%2);
        pimInst(pim_op,(uint64_t)opAddr);
    }
    for (unsigned len = 0 ; len < pim::BYTE_SIZE ; len++){
        printf("Sending mPIM ColNot with length %d\n",len);
        pim_op = pim::setOpImm(pim::COL_NOT,0,0,len,0);
        pimInst(pim_op,(uint64_t)opAddr);
    }
    for (unsigned len = 0 ; len < pim::BYTE_SIZE ; len++){
        printf("Sending mPIM ColAnd with length %d\n",len);
        pim_op = pim::setOpTwoSrc(pim::COL_AND,0,0,len,pim::BYTE_SIZE,len);
        pimInst(pim_op,(uint64_t)opAddr);
    }
    for (unsigned len = 0 ; len < pim::BYTE_SIZE ; len++){
        printf("Sending mPIM ColOr with length %d\n",len);
        pim_op = pim::setOpTwoSrc(pim::COL_OR,0,0,len,pim::BYTE_SIZE,len);
        pimInst(pim_op,(uint64_t)opAddr);
    }
    for (unsigned len = 0 ; len < pim::BYTE_SIZE ; len++){
        printf("Sending mPIM ColMul with length %d\n",len);
        pim_op = pim::setOpTwoSrc(pim::COL_MUL,0,0,len,pim::BYTE_SIZE,len);
        pimInst(pim_op,(uint64_t)opAddr);
    }
    for (unsigned len = 0 ; len < pim::BYTE_SIZE ; len++){
        printf("Sending mPIM ColEq with length %d\n",len);
        pim_op = pim::setOpTwoSrc(pim::COL_EQ,0,0,len,pim::BYTE_SIZE,len);
        pimInst(pim_op,(uint64_t)opAddr);
    }
    for (unsigned len = 0 ; len < pim::BYTE_SIZE ; len++){
        printf("Sending mPIM ColNeq with length %d\n",len);
        pim_op = pim::setOpTwoSrc(pim::COL_NEQ,0,0,len,pim::BYTE_SIZE,len);
        pimInst(pim_op,(uint64_t)opAddr);
    }
    for (unsigned len = 0 ; len < pim::BYTE_SIZE ; len++){
        printf("Sending mPIM ColLt with length %d\n",len);
        pim_op = pim::setOpTwoSrc(pim::COL_LT,0,0,len,pim::BYTE_SIZE,len);
        pimInst(pim_op,(uint64_t)opAddr);
    }
    for (unsigned len = 0 ; len < pim::BYTE_SIZE ; len++){
        printf("Sending mPIM ColLte with length %d\n",len);
        pim_op = pim::setOpTwoSrc(pim::COL_LTE,0,0,len,pim::BYTE_SIZE,len);
        pimInst(pim_op,(uint64_t)opAddr);
    }
    printf("Sending mPIM TransCol\n");
    pim_op = pim::setOpImm(pim::TRNS_COL,resultCol*pim::BYTE_SIZE,0,0,0);
    pimInst(pim_op,(uint64_t)opAddr);

    // do a read from an uncached location, so the simulation will finish
    // running the mPIM op before exiting
    char read_dummy = 0;
    for ( int i = 0 ; i < 40000 ; i++){
        read_dummy += *((char*)addr1+64*i);
        clflush((void*)((char*)addr1+64*i));
    }


    printf("Test end\n");
    return 0;
}
