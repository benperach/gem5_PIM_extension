#include <stdint.h>
#include <sys/mman.h>

#include <cstdio>
#include <cstdlib>

#include "/home/ben/gem5/src/mem/pim.hh"
#include "ColumnAccess.hh"
#include "clflush.h"
#include "piminst.h"

class colReadVerifyEqScan : public ColumnOperation {
  private:
    uint8_t comp_number;
    int dstOffset;
    void* addr2; // for debug
  public :
    colReadVerifyEqScan(void* _addr2) :
        comp_number(0),dstOffset(0),addr2(_addr2)
         {};

    virtual void operator() (uint8_t* addr, int col, int subarray,
                                int row, int mat, int chip, int startCol)
    {
        uint8_t result= *addr;
        uint8_t mask = 1U << dstOffset;
        uint8_t refResult = result&(~mask) +
                            mask*(((uint8_t)row) == comp_number);
        if (result != refResult) {
            printf("TestError: in row 0x%X, col 0x%X, mat 0x%X"
            ", subarray 0x%X, chip 0x%X, result = 0x%X, refResult = 0x%X\n",
                    row,col,mat,subarray,chip, result, refResult);
        }
    }

    inline void setCompNumber(uint8_t newCompNumber){
        comp_number = newCompNumber;
    }
    inline void setdstOffset(uint8_t newdstOffset){
        dstOffset = newdstOffset;
    }
};

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
    int resultCol = NumOfCols;
    uint8_t result;

    uint8_t compNumber = 0;

    columnOperate colOp = columnOperate();

    colWrite* fcolWrite = new colWrite();
    colFlush* fcolFlush = new colFlush();
    colReadVerifyEqScan* fcolReadVerifyEqScan =
            new colReadVerifyEqScan(addr2);

    void* opAddr = (void*)(
        (uintptr_t)addr2 + colOp.ADDR_COL_NUM_ARRAY[resultCol]
        );

    printf("Start writing to column\n");
    // go over all the columns and write the data
    colOp(addr2,firstCol,NumOfCols,1,fcolWrite);


    printf("Start flushing data\n");
    // go over all the columns and flush the data
    colOp(addr2,firstCol,NumOfCols,1,fcolFlush);

    for (int dstOffset = 0; dstOffset < pim::BYTE_SIZE ; dstOffset++){
        printf("reset the result column\n");
        colOp(addr2,resultCol,1,1,fcolWrite);
        printf("flush result column\n");
        colOp(addr2,resultCol,1,1,fcolFlush);

        compNumber = (uint8_t)(dstOffset+1);
        printf("Sending mPIM ScanEq(0x%X), offset = %d\n"
            ,compNumber,dstOffset);
        pim_op    = pim::setOpImm(pim::SCAN_EQ,
                    (resultCol%2)*pim::BYTE_SIZE+dstOffset,
                    0,pim::BYTE_SIZE-1,compNumber);
        pimInst(pim_op,(uint64_t)opAddr);
        fcolReadVerifyEqScan->setCompNumber(compNumber);
        fcolReadVerifyEqScan->setdstOffset(dstOffset);
        printf("Start read and verify data\n");
        // go over all the columns and read the data
        colOp(addr2,resultCol,1,1,fcolReadVerifyEqScan);
    }
    printf("Test end\n");

    return 0;
}
