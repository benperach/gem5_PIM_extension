#include <stdint.h>
#include <sys/mman.h>

#include <cstdio>
#include <cstdlib>

#include "/home/ben/gem5/src/mem/pim.hh"
#include "ColumnAccess.hh"
#include "clflush.h"
#include "piminst.h"

class colReadVerifyImmSet : public ColumnOperation {
  private:
    bool Imm;
    int Len;
    void* addr2; // for debug
  public :
    colReadVerifyImmSet(uint8_t _Imm, int _Len, void* _addr2) :
    Imm(_Imm),Len(_Len),addr2(_addr2) {};

    virtual void operator() (uint8_t* addr, int col, int subarray,
                                int row, int mat, int chip)
    {
        uint8_t result= *addr;
        uint8_t mask = (1U<<(Len+1))-1;
        uint8_t masked_row = ((uint8_t)row)&((uint8_t)(~mask));
        uint8_t expect = masked_row + Imm*mask;
        if ((uint8_t)(result%256) != expect) {
            printf("TestError: in row 0x%X, mat 0x%X, subarray 0x%X"
                    ", chip 0x%X, result = 0x%X, expect = 0x%X"
                    ", mask = 0x%X, masked_row = 0x%X\n",
                    row,mat,subarray,chip,result,expect,mask,masked_row);
        }/* else {
            printf("TestPass1: in row 0x%X, mat 0x%X, subarray 0x%X"
                    ",  chip 0x%X, result = 0x%X\n",
                    row,mat,subarray,chip, result);
        }*/
        //printf("Addr : 0x%lX,  Addr-addr2=0x%lX\n",
        //    (uintptr_t)addr,(uintptr_t)addr-(uintptr_t)addr2);
    }

    void setImm(bool newImm){
        Imm = newImm;
    }
    void setLen(int newLen){
        Len = newLen;
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

    uint8_t Imm = 0x00;

    int NumOfCols = 1;
    int firstCol = 0;
    int resultCol = 0;
    uint8_t result;

    columnOperate colOp = columnOperate();

    colWrite* fcolWrite = new colWrite();
    colFlush* fcolFlush = new colFlush();
    colReadVerifyImmSet* fcolReadVerifyImmSet =
            new colReadVerifyImmSet(Imm,0,addr2);

    for (int len = 0 ; len < pim::BYTE_SIZE ; len++){
        fcolReadVerifyImmSet->setLen(len);
        for (int i = 0; i < 2 ; i++){
            printf("Start writing to column\n");
            // go over all the columns and write the data
            colOp(addr2,firstCol,NumOfCols,1,fcolWrite);

            printf("Start flushing data\n");
            // go over all the columns and flush the data
            colOp(addr2,firstCol,NumOfCols,1,fcolFlush);

            printf("Sending mPIM ColImmSet(0x%X) with len = %d\n",i,len);
            pim_op    = pim::setOpImm(pim::COLIMM_SET,0,0,len,(uint8_t)i);
            pimInst(pim_op,(uint64_t)addr2);
            fcolReadVerifyImmSet->setImm((uint8_t)i);
            printf("Start read and verify data\n");
            // go over all the columns and read the data
            colOp(addr2,resultCol,1,1,fcolReadVerifyImmSet);
        }
    }
    printf("Test end\n");

    return 0;
}
