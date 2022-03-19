#include <stdint.h>
#include <sys/mman.h>

#include <cstdio>
#include <cstdlib>

#include "/home/ben/gem5/src/mem/pim.hh"
#include "ColumnAccess.hh"
#include "clflush.h"
#include "piminst.h"

uint8_t reverse_bit(uint8_t num)
{
    uint8_t out = 0;
    for (int i = 0 ; i < sizeof(uint8_t)*pim::BYTE_SIZE ; i++){
        out <<= 1;
        out |= ((num >> i) & 1U);
    }
    return out;
}

class colReadVerifyMul : public ColumnOperation {
  private:
    int src1len;
    int src2len;
    uint8_t firstColMask;
    uint8_t secondColMask;
    uint16_t resultMask;
    void* addr2; // for debug
  public :
    colReadVerifyMul(void* _addr2) :
    firstColMask(0),secondColMask(0),resultMask(0),addr2(_addr2){};

    virtual void operator() (uint8_t* addr, int col, int subarray,
                                int row, int mat, int chip)
    {
        uint16_t result = (*(uint16_t*)addr) & resultMask;
        uint16_t expected = (uint16_t)((uint8_t)row&firstColMask)
                            * (uint16_t)(reverse_bit(row)&secondColMask);
        if (result == expected) {}
        else {
            printf("TestError: in row 0x%X, mat 0x%X, subarray 0x%X"
                    ", chip 0x%X, result = 0x%X, expected = 0x%X\n",
                    row,mat,subarray,chip, result, expected);
        }
        //printf("Addr : 0x%lX,  Addr-addr2=0x%lX\n",
        //    (uintptr_t)addr,(uintptr_t)addr-(uintptr_t)addr2);
    }

    inline void setSrc1Len(int len) {
        src1len = len;
        firstColMask = (1U<<(src1len+1))-1;
        resultMask = (1U<<((src1len+1)+(src2len+1)))-1;
    }
    inline void setSrc2Len(int len) {
        src2len = len;
        secondColMask = (1U<<(src2len+1))-1;
        resultMask = (1U<<((src1len+1)+(src2len+1)))-1;
    }
};

class colWriteSecond : public ColumnOperation {
  public :
    colWriteSecond() {};
    virtual void operator() (uint8_t* addr, int col, int subarray,
                                int row, int mat, int chip)
    {
        *addr = reverse_bit((uint8_t)row);
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
    int resultCol = 2;
    uint8_t result;

    columnOperate colOp = columnOperate();

    colWrite* fcolWrite = new colWrite();
    colWriteSecond* fcolWriteSec = new colWriteSecond();
    colFlush* fcolFlush = new colFlush();
    colReadVerifyMul* fcolReadVerifyMul =
            new colReadVerifyMul(addr2);

    printf("Start writing to first column\n");
    // go over all the columns and write the data
    colOp(addr2,firstCol,NumOfCols,1,fcolWrite);

    printf("Start writing to second column\n");
    // go over all the columns and write the data
    colOp(addr2,firstCol+1,NumOfCols,1,fcolWriteSec);

    printf("Start flushing data\n");
    // go over all the data columns and flush the data.
    // since we write to two columns we need to flush
    // them both, but two columns go in a single cache block
    // so it is enough to flush a single time to hit both columns.
    colOp(addr2,firstCol,NumOfCols,1,fcolFlush);

    for (int src1len = 0 ; src1len < pim::BYTE_SIZE ; src1len++){
        fcolReadVerifyMul->setSrc1Len(src1len);
        for (int src2len = 0 ; src2len < pim::BYTE_SIZE ; src2len++){
            fcolReadVerifyMul->setSrc2Len(src2len);
            printf("Sending mPIM COL_MUL (src1len,src2len) = (0x%X,0x%X)\n",
                    src1len,src2len);
            // the src1 len is 7 because we write (len-1)
            pim_op    = pim::setOpTwoSrc(pim::COL_MUL,2*pim::BYTE_SIZE,
                                            0,src1len,
                                            1*pim::BYTE_SIZE,src2len);
            pimInst(pim_op,(uint64_t)addr2);

            printf("Start read and verify data\n");
            // go over all the columns and read the data
            colOp(addr2,resultCol,1,1,fcolReadVerifyMul);

            printf("Start flushing results\n");
            // go over all the result columns and flush them.
            // since we write to two columns we need to flush
            // them both, but two columns go in a single cache block
            // so it is enough to flush a single time to hit both columns.
            colOp(addr2,resultCol,NumOfCols,1,fcolFlush);
        }
    }
    printf("Test end\n");

    return 0;
}
