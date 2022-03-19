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

class colReadVerifyEq : public ColumnOperation {
  private:
    void* addr2; // for debug
  public :
    colReadVerifyEq(void* _addr2) :
    addr2(_addr2) {};

    virtual void operator() (uint8_t* addr, int col, int subarray,
                                int row, int mat, int chip)
    {
        uint8_t result= *addr;
        uint8_t expected = ((uint8_t)row == reverse_bit(row));
        if (result%256 != expected) {
            printf("!!!TestError!!!!: in row 0x%X, mat 0x%X, subarray 0x%X"
                    ", chip 0x%X, result = 0x%X, expected = 0x%X\n",
                    row,mat,subarray,chip, result, expected);
        } else if ((mat == 0) && (subarray == 0) && (chip == 0)){
            printf("TestPass: in row 0x%X, mat 0x%X, subarray 0x%X"
                    ",  chip 0x%X, result = 0x%X, expected = 0x%X\n",
                    row,mat,subarray,chip, result, expected);
        }
        //printf("Addr : 0x%lX,  Addr-addr2=0x%lX\n",
        //    (uintptr_t)addr,(uintptr_t)addr-(uintptr_t)addr2);
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
    colReadVerifyEq* fcolReadVerifyEq =
            new colReadVerifyEq(addr2);

    printf("Start writing to first column\n");
    // go over all the columns and write the data
    colOp(addr2,firstCol,NumOfCols,1,fcolWrite);

    printf("Start writing to second column\n");
    // go over all the columns and write the data
    colOp(addr2,firstCol+1,NumOfCols,1,fcolWriteSec);

    printf("Start flushing data\n");
    // go over all the columns and flush the data
    colOp(addr2,firstCol,NumOfCols,1,fcolFlush);

    printf("Sending mPIM ColEq\n");
    // the src1 len is 7 because we write (len-1)
    pim_op    = pim::setOpTwoSrc(pim::COL_EQ,16,0,7,8,7);
    pimInst(pim_op,(uint64_t)addr2);

    printf("Start read and verify data\n");
    // go over all the columns and read the data
    colOp(addr2,resultCol,1,1,fcolReadVerifyEq);
    printf("Test end\n");

    return 0;
}
