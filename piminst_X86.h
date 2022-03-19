
#ifndef __PIMINST_HH__
#define __PIMINST__HH__

void pimInst(uint64_t pim_op, uint64_t ptr_d) {
    __asm __volatile__ (
        "mov %[pim_op], %%rax\n\t"       // move PIM op to register RAX
        "mov %[ptr_d], %%rbx\n\t"         // move data address to register RBX
        //         rrrbbb|pim op|| 2BOp ||REXWRXB
        ".long 0b00000011000010100000111101001000\n\t"
        :
        : [pim_op]       "r"  (pim_op),
          [ptr_d]       "r"  (ptr_d)
        : "rax", "rbx", "memory"
  );
}

#endif // __PIMINST_HH__