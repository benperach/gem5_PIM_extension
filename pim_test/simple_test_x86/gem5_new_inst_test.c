#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>

void pimInst(uint64_t pim_op, uint64_t ptr_d) {
    __asm __volatile__ (
        "mov %[pim_op], %%rax\n\t"       // move mPIM op to register RAX
        "mov %[ptr_d], %%rbx\n\t"         // move data address to register RBX
        //       |REXWRXB|pim op|| rrrbbb
        //".long 0b010010001111000000000011 \n\t"
        //         rrrbbb|pim op|| 2BOp ||REXWRXB
        ".long 0b00000011000010100000111101001000\n\t"
        :
        : [pim_op]       "r"  (pim_op),
          [ptr_d]       "r"  (ptr_d)
        : "rax", "rbx", "memory"
  );
}

int main(){
    void* addr        = (char*) malloc(1000);
    //void* addr        = mmap(NULL, 2*1024*1024,
    //    PROT_READ|PROT_WRITE, MAP_SHARED|MAP_ANONYMOUS|MAP_HUGETLB,-1,0);
    //void* addr      = mmap(NULL, 2*1024*1024,
    //    PROT_READ|PROT_WRITE, MAP_SHARED|MAP_ANONYMOUS,-1,0);
    uint64_t pim_op    = 0xFEDCBA9876543210;

    uint64_t dest = (uint64_t)addr + 1024*1024;

    for (int i = 0 ; i<20 ; i++){
        //printf("running the %d pim op\n", i);
        pimInst(pim_op,(uint64_t)addr);
        //addr <<= 1;
        pim_op <<= 1;
    }

    return 0;
}
