
#ifndef __PIMINST_HH__
#define __PIMINST__HH__

void pimInst(uint64_t pim_op, uint64_t ptr_d) {
    __asm __volatile__ (
        "mov x11, %[pim_op]\n\t"    // move mPIM op to register x11
        "mov x10, %[ptr_d]\n\t"      // move data address to register x10
        // bits 31:30 = 11   :this is the size of the store, 64 bit is b11,
        // part of the original encoding, is not important
        // bit 29 = 1        :is part of the original encoding of STRX64_IMM,
        //so it is not important here.
        // bits 28:25 = 0011 :this is the identifier of the new instruction
        // bit 24 = 1        :is part of the original encoding of STRX64_IMM,
        // so it is not important here.
        // bits 23:22 = 00   :is part of the original encoding of STRX64_IMM,
        // so it is not important here.
        // bits 21:10 = 0x000:this is the IMM12 bits, we don't want any offset
        // so it is 0.
        // bits 9:5 = 01010 :this is the Rn field, should contain the address,
        // so this is X10
        // bits 4:0 = 01011  : this is the Rt field, should contain the mPIM
        // operation, so this is X11
        //        | |  | |||  imm     ||rn ||rt |
        ".long 0b11100111000000000000000101001011\n\t"
        :
        : [pim_op]       "r"  (pim_op),
          [ptr_d]       "r"  (ptr_d)
        : "x10", "x11", "memory"
  );
}

#endif // __PIMINST_HH__