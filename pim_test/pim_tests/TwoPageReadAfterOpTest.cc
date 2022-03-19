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
    void* addr1        = malloc(8*1024*1024);
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
                            - 0x105000);
    void* addr3_pre    = (void *)((uint8_t*)addr2_pre + 2*1024*1024);
    void* addr3        = (void *)((uint8_t*)addr3_pre
                            - 0x105000);

    printf("addr1 : 0x%lX\n addr2_pre : 0x%lX\n addr2 : 0x%lX\n"
                "addr3_pre : 0x%lX\n addr3 : 0x%lX\n",
                (uintptr_t)addr1,(uintptr_t)addr2_pre,(uintptr_t)addr2,
                (uintptr_t)addr3_pre,(uintptr_t)addr3);
    // set the opcode to communicate with the gem5 simulation.
    // the payload should be 0 for page addition
    uint64_t pim_op    = pim::setOpImm(pim::GEM5,0,0,0,0);

    unsigned a;
    printf("Add pages to mPIM\n");
    pimInst(pim_op,(uint64_t)addr2);
    pimInst(pim_op,(uint64_t)addr3);

    //write and flush to mPIM for debugging
    *(uint8_t*)addr2 = 0x11;
    *(uint8_t*)addr3 = 0x11;

    clflush(addr2);
    clflush(addr3);

    uint8_t comp_number = 0x00;

    int NumOfCols = 1;
    int firstCol = 0;
    int resultCol = 2;
    uint8_t result1[pim::BYTE_SIZE];
    uint8_t result2[pim::BYTE_SIZE];
    uint8_t expected[pim::BYTE_SIZE] =
    {0xAA,0xCC,0xF0,0x00,0xFF,0xFF,0xFF,0xFF};

    uint64_t pim_op_arr[pim::BYTE_SIZE];
    void* opAddr1[pim::BYTE_SIZE];
    void* readAddr1[pim::BYTE_SIZE];
    void* opAddr2[pim::BYTE_SIZE];
    void* readAddr2[pim::BYTE_SIZE];


    columnOperate colOp = columnOperate();

    colWrite* fcolWrite = new colWrite();
    colFlush* fcolFlush = new colFlush();

    printf("Start writing to column\n");
    // go over all the columns and write the data
    colOp(addr2,firstCol,NumOfCols,1,fcolWrite);
    colOp(addr3,firstCol,NumOfCols,1,fcolWrite);

    printf("Start flushing data\n");
    // go over all the columns and flush the data
    colOp(addr2,firstCol,NumOfCols,1,fcolFlush);
    colOp(addr3,firstCol,NumOfCols,1,fcolFlush);

    // alternate between operation on and reading from the
    // array
    printf("Starting Test\n");
    for (int i = 0; i < pim::BYTE_SIZE ; i++){
        opAddr1[i] = (void*)(
            (uintptr_t)addr2
                    + colOp.ADDR_COL_NUM_ARRAY[resultCol+2*i]
            );
        opAddr2[i] = (void*)(
            (uintptr_t)addr3
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
            (uintptr_t)addr3
                    + colOp.ADDR_COL_NUM_ARRAY[resultCol+2*i]
                    + colOp.ADDR_ROW_NUM_ARRAY[511]
            );
    }
    for (int i = 0; i < pim::BYTE_SIZE ; i++){
        pimInst(pim_op_arr[i],(uint64_t)opAddr1[i]);
        pimInst(pim_op_arr[i],(uint64_t)opAddr2[i]);
        result1[i] = *(uint8_t*)readAddr1[i];
        result2[i] = *(uint8_t*)readAddr2[i];
    }
    for (int i = 0; i < pim::BYTE_SIZE ; i++){
        if (result1[i] != expected[i])
            printf("Error 1 : in i = %d, expected = 0x%X, result1 = 0x%X\n",
                    i,expected[i],result1[i]);
        if (result2[i] != expected[i])
            printf("Error 2 : in i = %d, expected = 0x%X, result2 = 0x%X\n",
                    i,expected[i],result2[i]);
    }
    printf("Test end\n");

    return 0;
}
