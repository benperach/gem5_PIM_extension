#include <stdint.h>
#include <sys/mman.h>

#include <cstdio>
#include <cstdlib>

#include "/home/ben/gem5/src/mem/pim.hh"
#include "ColumnAccess.hh"
#include "clflush.h"
#include "piminst.h"

class colReadVerifyTransCol : public ColumnOperation {
  private:
    uint8_t* colByte;//[pim::MAT_DIM/pim::BYTE_SIZE];
    void* addr2; // for debug
  public :
    colReadVerifyTransCol(uint8_t* _colByte, void* _addr2):
    colByte(_colByte),addr2(_addr2) {};

    virtual void operator() (uint8_t* addr, int col, int subarray,
                                int row, int mat, int chip,
                                int startCol)
    {
        uint8_t expected = 0;
        if ((row % pim::MAT_READ_SIZE) == pim::MAT_READ_SIZE - 1){
            expected = colByte[(row/pim::MAT_READ_SIZE)*
                                (pim::MAT_READ_SIZE/pim::BYTE_SIZE)
                                +(col-startCol)];
        }

        uint8_t result = *addr;
        if ((result) != expected)
        {
            printf("TestError: in row 0x%X, col 0x%X , mat 0x%X"
                    ", subarray 0x%X, chip 0x%X, result = 0x%X"
                    ", expected = 0x%X\n",
                    row,col,mat,subarray,chip,result,
                    expected);
        } else if ( (row == pim::MAT_DIM - 1) && (subarray == 0)
                    && (mat == 0) && (chip == 0) ){
            printf("TestPass: in row 0x%X, col 0x%X , mat 0x%X"
                    ", subarray 0x%X, chip 0x%X, result = 0x%X"
                    ", expected = 0x%X\n",
                    row,col,mat,subarray,chip,result,
                    expected);
        }
    }
};

class colWriteTrans : public ColumnOperation {
  private:
    uint8_t* colByte;//[pim::MAT_DIM/pim::BYTE_SIZE];
  public :
    colWriteTrans(uint8_t* _colByte):
    colByte(_colByte) {};
    virtual void operator() (uint8_t* addr, int col, int subarray,
                                int row, int mat, int chip,
                                int startCol)
    {
        // write the number of row, cut it to fit a byte
        int byte_idx = row / pim::BYTE_SIZE;
        int bit_idx = row % pim::BYTE_SIZE;
        *addr = (uint8_t)((colByte[byte_idx] >> bit_idx) & 1U);
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

    // using a prime number
    uint8_t prime_int = 0x13;
    uint8_t acum = prime_int;

    uint8_t colByte[pim::MAT_DIM/pim::BYTE_SIZE] = {};
    for (int i = 0 ; i < pim::MAT_DIM/pim::BYTE_SIZE ; i ++){
        colByte[i] = acum;
        acum *= prime_int;
        printf("colByte[%d] = 0x%X\n",i,colByte[i]);
    }

    int NumOfCols = 1;
    int firstCol = 0;
    int resultCol = 1;
    uint8_t result;
    int resNumOfCol = pim::MAT_READ_SIZE/pim::BYTE_SIZE;

    columnOperate colOp = columnOperate();

    colWriteTrans* fcolWrite = new colWriteTrans(colByte);
    colFlush* fcolFlush = new colFlush();
    colReadVerifyTransCol* fcolReadVerifyTransCol =
            new colReadVerifyTransCol(colByte,addr2);

    printf("Start writing to column\n");
    // go over all the columns and write the data
    colOp(addr2,firstCol,NumOfCols,1,fcolWrite);


    printf("Start flushing data\n");
    // go over all the columns and flush the data
    colOp(addr2,firstCol,NumOfCols,1,fcolFlush);

    printf("Sending mPIM TransCol\n");
    pim_op = pim::setOpImm(pim::TRNS_COL,resultCol*pim::BYTE_SIZE
                        ,0,0,0);
    pimInst(pim_op,(uint64_t)addr2);
    printf("Start read and verify data\n");
    // go over all the columns and read the data
    colOp(addr2,resultCol,resNumOfCol,1,fcolReadVerifyTransCol);

    printf("Test end\n");

    return 0;
}
