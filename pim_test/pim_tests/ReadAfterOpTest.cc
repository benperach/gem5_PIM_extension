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

This test runs mPIM ops and reads to the same page to test the
dependency mechanism in the mPIM controller.
To verify the results, the gem5 simulation should have the MPIMop
and MPIM debug flag set. Using the output of the gem5 debug, it should
be checked that the reads and ops are delayed properly.
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

    printf("addr1 : 0x%lX\n addr2_pre : 0x%lX\n addr2 : 0x%lX\n",
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

    uint8_t comp_number = 0x00;

    int NumOfCols = 1;
    int firstCol = 0;
    int resultCol = 2;
    uint8_t result[2*pim::BYTE_SIZE];
    uint8_t expected[2*pim::BYTE_SIZE] =
    {0xAA,0x0,0xCC,0x00,0xF0,0x00,0x00,0x00,
    0xFF,0x00,0xFF,0x00,0xFF,0x00,0xFF,0x00};

    uint64_t pim_op_arr[pim::BYTE_SIZE];
    void* opAddr[pim::BYTE_SIZE];
    void* readAddr1[pim::BYTE_SIZE];
    void* readAddr2[pim::BYTE_SIZE];

    columnOperate colOp = columnOperate();

    colWrite* fcolWrite = new colWrite();
    colFlush* fcolFlush = new colFlush();

    printf("Start writing to column\n");
    // go over all the columns and write the data
    colOp(addr2,firstCol,NumOfCols,1,fcolWrite);


    printf("Start flushing data\n");
    // go over all the columns and flush the data
    colOp(addr2,firstCol,NumOfCols,1,fcolFlush);

    // alternate between operation on and reading from the
    // array
    printf("Starting Test\n");
    for (int i = 0; i < pim::BYTE_SIZE ; i++){
        opAddr[i] = (void*)(
            (uintptr_t)addr2
                    + colOp.ADDR_COL_NUM_ARRAY[resultCol+2*i]
            );
        pim_op_arr[i] = pim::setOpImm(pim::TRNS_COL,
                pim::BYTE_SIZE*(resultCol%2),i,0,0);
        readAddr1[i] = (void*)(
            (uintptr_t)addr2
                    + colOp.ADDR_COL_NUM_ARRAY[resultCol+2*i]
                    + colOp.ADDR_ROW_NUM_ARRAY[511]
            );
        readAddr2[i] = (void*)(
            (uintptr_t)addr2
                    + colOp.ADDR_COL_NUM_ARRAY[resultCol+2*i+2]
                    + colOp.ADDR_ROW_NUM_ARRAY[511-pim::MAT_READ_SIZE]
            );
    }
    for (int i = 0; i < pim::BYTE_SIZE ; i++){
        pimInst(pim_op_arr[i],(uint64_t)opAddr[i]);
        result[2*i] = *(uint8_t*)readAddr1[i];
        result[2*i+1] = *(uint8_t*)readAddr2[i];
    }
    for (int i = 0; i < 2*pim::BYTE_SIZE ; i++){
        if (result[i] != expected[i])
            printf("Error: in i = %d, expected = 0x%X, result = 0x%X\n",
                    i,expected[i],result[i]);
    }
    printf("Test end\n");

    return 0;
}
