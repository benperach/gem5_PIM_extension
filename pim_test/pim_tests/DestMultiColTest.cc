#include <stdint.h>
#include <sys/mman.h>

#include <cstdio>
#include <cstdlib>

#include "/home/ben/gem5/src/mem/pim.hh"
#include "ColumnAccess.hh"
#include "clflush.h"
#include "piminst.h"

class colReadVerifyColAdd : public ColumnOperation {
  private:
    uint8_t comp_number;
    int dstOffset;
    int dstLen;
    int dstColNum;
    void* addr2; // for debug
    columnOperate* colOp;
  public :
    colReadVerifyColAdd(void* _addr2,columnOperate* _colOp) :
        comp_number(0),dstOffset(0),dstLen(0),dstColNum(0),
        addr2(_addr2),colOp(_colOp)
         {};

    virtual void operator() (uint8_t* addr, int col, int subarray,
                                int row, int mat, int chip, int startCol)
    {
        uint8_t result;
        uint64_t totResult = 0;
        uint64_t expResult = 0;
        uint64_t expMask = 0;
        uint8_t mask;
        uint64_t operand;
        for ( int i = 0 ; i < dstColNum ; i++ ) {
            result = *(
                        (uint8_t*)(
                            ((uintptr_t)addr & ~(colOp->ADDR_COL_MASK))
                            | colOp->ADDR_COL_NUM_ARRAY[col+i]
                        )
                    );
            totResult |= result << (i*pim::BYTE_SIZE);
            if (i == 0){
                mask = ~pim::FIRST_COL_ANTYMASK[dstOffset];
                // if there is only one column we need to look at then this
                // is also the last column
                if (dstColNum == 1)
                    mask &=
                       pim::LAST_COL_MASK[(dstOffset+dstLen)%pim::BYTE_SIZE];
            // if this is the last column, and there are more then 1
            } else if (i == dstColNum - 1) {
                mask = pim::LAST_COL_MASK[(dstOffset+dstLen)%pim::BYTE_SIZE];
            // if this is not the first and not the last
            } else {
                mask = 0xFF;
            }
            expMask |= mask << (i*pim::BYTE_SIZE);
            expResult |= ((uint8_t)row) << (i*pim::BYTE_SIZE);
        }
        operand = (expResult + (uint64_t)comp_number) << dstOffset;
        expResult = (expResult & ~expMask) | (operand & expMask);
        if (totResult != expResult) {
            printf("TestError: in row 0x%X, col 0x%X, mat 0x%X"
            ", subarray 0x%X, chip 0x%X, totResult = 0x%lX, expResult = 0x%lX"
            ", expMask = 0x%lX, result = 0x%X\n",
                    row,col,mat,subarray,chip,totResult,expResult,
                            expMask,result);
        }
    }

    inline void setCompNumber(uint8_t newCompNumber){
        comp_number = newCompNumber;
    }
    inline void setdstOffset(uint8_t newdstOffset){
        dstOffset = newdstOffset;
    }
    inline void setdstLen(uint8_t newdstLen){
        dstLen = newdstLen;
    }
    inline void setdstColNum(uint8_t newdstColNum){
        dstColNum = newdstColNum;
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

    int NumOfCols = 3;
    int firstCol = 0;
    int resultCol = NumOfCols;
    int resultColNum = 0;
    uint8_t result;

    uint8_t compNumber = 0;

    columnOperate colOp = columnOperate();

    colWrite* fcolWrite = new colWrite();
    colFlush* fcolFlush = new colFlush();
    colReadVerifyColAdd* fcolReadVerifyColAdd =
            new colReadVerifyColAdd(addr2,&colOp);

    void* opAddr = (void*)(
        (uintptr_t)addr2 + colOp.ADDR_COL_NUM_ARRAY[resultCol]
        );

    printf("Start writing to column\n");
    // go over all the columns and write the data
    colOp(addr2,firstCol,NumOfCols,1,fcolWrite);


    printf("Start flushing data\n");
    // go over all the columns and flush the data
    colOp(addr2,firstCol,NumOfCols,2,fcolFlush);

    for (int dstOffset = 0; dstOffset < pim::BYTE_SIZE ; dstOffset++){
        for (int dstLen = 0; dstLen < 3*pim::BYTE_SIZE ; dstLen++){
            resultColNum = (dstLen+1 + dstOffset)/pim::BYTE_SIZE +
                            (((dstLen+1 + dstOffset)%pim::BYTE_SIZE) > 0);
            printf("resultCol = %d, resultColNum = %d\n", resultCol,
                            resultColNum);
            printf("reset the result column\n");
            colOp(addr2,resultCol,resultColNum,1,fcolWrite);
            printf("flush result column\n");
            colOp(addr2,resultCol,resultColNum+(resultCol+resultColNum)%2,
                        2,fcolFlush);

            compNumber = (uint8_t)(dstOffset+1);
            printf("Sending mPIM ColAdd(0x%X), Len = 0x%X, offset = %d\n"
                ,compNumber,dstLen,dstOffset);
            pim_op    = pim::setOpImm(pim::COLIMM_ADD,
                        (resultCol%2)*pim::BYTE_SIZE+dstOffset,
                        0,dstLen,compNumber);
            pimInst(pim_op,(uint64_t)opAddr);
            fcolReadVerifyColAdd->setCompNumber(compNumber);
            fcolReadVerifyColAdd->setdstOffset(dstOffset);
            fcolReadVerifyColAdd->setdstLen(dstLen+1);
            fcolReadVerifyColAdd->setdstColNum(resultColNum);
            printf("Start read and verify data\n");
            // go over all the columns and read the data
            colOp(addr2,resultCol,1,1,fcolReadVerifyColAdd);
        }
    }
    printf("Test end\n");

    return 0;
}
