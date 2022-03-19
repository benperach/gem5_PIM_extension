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
    void* addr2; // for debug
  public :
    colReadVerifyEqScan(uint8_t _comp_number, void* _addr2) :
    comp_number(_comp_number),addr2(_addr2) {};

    virtual void operator() (uint8_t* addr, int col, int subarray,
                                int row, int mat, int chip)
    {
        uint8_t result= *addr;
        if (row % 256 == comp_number % 256)
        {
            if ((result & 1u) != 1) {
                printf("TestError1: in row 0x%X, mat 0x%X, subarray 0x%X"
                        ", chip 0x%X, result = 0x%X\n",
                        row,mat,subarray,chip, result);
            }/* else {
                printf("TestPass1: in row 0x%X, mat 0x%X, subarray 0x%X"
                        ",  chip 0x%X, result = 0x%X\n",
                        row,mat,subarray,chip, result);
            }*/
            //printf("Addr : 0x%lX,  Addr-addr2=0x%lX\n",
            //    (uintptr_t)addr,(uintptr_t)addr-(uintptr_t)addr2);
        } else if (row % 256 != comp_number % 256)
        {
            if ((result & 1u) != 0) {
                printf("TestError0: in row 0x%X, mat 0x%X, subarray 0x%X"
                       ",  chip 0x%X, result = 0x%X\n",
                        row,mat,subarray,chip, result);
            }/* else {
                printf("TestPass0: in row 0x%X, mat 0x%X, subarray 0x%X"
                        ",  chip 0x%X, result = 0x%X\n",
                        row,mat,subarray,chip, result);
            }*/
        }
    }

    void setCompNumber(uint8_t newCompNumber){
        comp_number = newCompNumber;
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

    columnOperate colOp = columnOperate();

    colWrite* fcolWrite = new colWrite();
    colFlush* fcolFlush = new colFlush();
    colReadVerifyEqScan* fcolReadVerifyEqScan =
            new colReadVerifyEqScan(comp_number,addr2);

    printf("Start writing to column\n");
    // go over all the columns and write the data
    colOp(addr2,firstCol,NumOfCols,1,fcolWrite);


    printf("Start flushing data\n");
    // go over all the columns and flush the data
    colOp(addr2,firstCol,NumOfCols,1,fcolFlush);

    for (int i = 0; i < 256 ; i++){
        printf("Sending mPIM ScanEq(0x%X)\n",i);
        // the src1 len is 7 because we write (len-1)
        pim_op    = pim::setOpImm(pim::SCAN_EQ,8,0,7,(uint8_t)i);
        pimInst(pim_op,(uint64_t)addr2);
        fcolReadVerifyEqScan->setCompNumber((uint8_t)i);
        printf("Start read and verify data\n");
        // go over all the columns and read the data
        colOp(addr2,resultCol,1,1,fcolReadVerifyEqScan);
        printf("Start flushing data\n");
        // flush the result data
        colOp(addr2,resultCol,1,1,fcolFlush);
    }
    printf("Test end\n");

    return 0;
}
