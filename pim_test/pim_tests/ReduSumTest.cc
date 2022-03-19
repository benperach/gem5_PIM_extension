#include <stdint.h>
#include <sys/mman.h>

#include <cstdio>
#include <cstdlib>

#include "/home/ben/gem5/src/mem/pim.hh"
#include "ColumnAccess.hh"
#include "clflush.h"
#include "piminst.h"

class colReadVerifyReduSum : public ColumnOperation {
  private:
    unsigned comp_number;
    void* addr2; // for debug
    unsigned res_mask;
  public :
    colReadVerifyReduSum(void* _addr2):
    comp_number(0),addr2(_addr2),res_mask(0) {};

    virtual void operator() (uint8_t* addr, int col, int subarray,
                                int row, int mat, int chip,
                                int startCol)
    {
        uint8_t expected = 0;
        int curShift = (col-startCol)*pim::BYTE_SIZE;
        if (row == pim::MAT_DIM - 1){
            expected = (comp_number >> curShift) & 0xFF;
        }

        uint8_t result = *addr;
        if ((result & (res_mask >> curShift) & 0xFF) != expected)
        {
            printf("TestError: in row 0x%X, col 0x%X , mat 0x%X"
                    ", subarray 0x%X, chip 0x%X, result = 0x%X"
                    ", expected = 0x%X"
                    ", res_mask >> curShift = 0x%X\n",
                    row,col,mat,subarray,chip,result,
                    expected,(res_mask >> curShift) & 0xFF);
        } else if ( (row == pim::MAT_DIM - 1) && (subarray == 0)
                    && (mat == 0) && (chip == 0) ){
            printf("TestPass: in row 0x%X, col 0x%X , mat 0x%X"
                    ", subarray 0x%X, chip 0x%X, result = 0x%X"
                    ", expected = 0x%X"
                    ", res_mask >> curShift = 0x%X\n",
                    row,col,mat,subarray,chip,result,
                    expected,(res_mask >> curShift) & 0xFF);
        }
    }

    void setInputLen(unsigned len){
        comp_number = 0;
        for ( unsigned i = 0 ; i < (1U<<len) ; i++){
            comp_number += i;
        }
        comp_number *= pim::MAT_DIM/(1U<<len);
        printf("new comp_number = 0x%X\n",comp_number);
        res_mask = (1U<<(len+pim::LOG_MAT_DIM)) - 1;
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

    uint8_t comp_number = 0x00;

    int NumOfCols = 1;
    int firstCol = 0;
    int resultCol = 1;
    uint8_t result;
    int resNumOfCol = 0;

    columnOperate colOp = columnOperate();

    colWrite* fcolWrite = new colWrite();
    colFlush* fcolFlush = new colFlush();
    colReadVerifyReduSum* fcolReadVerifyReduSum =
            new colReadVerifyReduSum(addr2);

    printf("Start writing to column\n");
    // go over all the columns and write the data
    colOp(addr2,firstCol,NumOfCols,1,fcolWrite);


    printf("Start flushing data\n");
    // go over all the columns and flush the data
    colOp(addr2,firstCol,NumOfCols,1,fcolFlush);

    for (unsigned len = 0 ; len < pim::BYTE_SIZE ; len++){
        printf("Sending mPIM ReduAdd with length %d\n",len);
        pim_op = pim::setOpImm(pim::REDU_SUM,resultCol*pim::BYTE_SIZE
                    ,0,len,0);
        pimInst(pim_op,(uint64_t)addr2);
        printf("Start read and verify data\n");
        // go over all the columns and read the data
        fcolReadVerifyReduSum->setInputLen(len+1);
        resNumOfCol = (len+1+pim::LOG_MAT_DIM)/pim::BYTE_SIZE +
                      (((len+1+pim::LOG_MAT_DIM)%pim::BYTE_SIZE == 0) ?
                        0 : 1);
        colOp(addr2,resultCol,resNumOfCol,1,fcolReadVerifyReduSum);
        // flush results
        // go over all the columns and flush the data
        printf("Start flush results\n");
        colOp(addr2,resultCol,NumOfCols,1,fcolFlush);
        colOp(addr2,resultCol+2,NumOfCols,1,fcolFlush);
    }
    printf("Test end\n");

    return 0;
}
