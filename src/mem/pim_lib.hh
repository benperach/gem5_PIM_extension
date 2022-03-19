#include <algorithm>
#include <cmath>

//#include <algorithm.h>
#include <inttypes.h>

#include "/home/ben/gem5/src/base/types.hh"

//#include "/home/ben/gem5/mem/packet.hh"

#ifndef __PIM_HH__
#define __PIM_HH__

#define pimGetMask(len, startBit) (((((uint64_t)1U)<<len) - 1) << (startBit))

// this packet describe some constants relevant to the PIM

namespace pim {

/**  ###########    general section      ##############     **/

    const bool PERFORM_PIM_OPS             = true;

    const unsigned LOG_BYTE_SIZE            = 3;
    const unsigned BYTE_SIZE                = 1U << LOG_BYTE_SIZE; // bits

    const unsigned MAX_MAT_READ_SIZE        = 64; // bits
    const unsigned NUM_OF_SA_PER_MAT        = 32;
    const unsigned MAT_READ_SIZE            = 16; // bits
    const unsigned MATS_IN_READ             = MAX_MAT_READ_SIZE/MAT_READ_SIZE;

    const unsigned LOG_PIM_OPERAND_MAX_SIZE= 6;
    const unsigned PIM_OPERAND_MAX_SIZE    =
                    1U << LOG_PIM_OPERAND_MAX_SIZE; // bits

    const unsigned LOG_MAT_ROWS              = 10;
    const unsigned MAT_ROWS                  = 1U << LOG_MAT_ROWS; //cells
    const unsigned LOG_MAT_COLS              = 9;
    const unsigned MAT_COLS                  = 1U << LOG_MAT_COLS; //cells

    const unsigned LOG_MAT_PER_SUBARRY      = 2;
    const unsigned MAT_PER_SUBARRY          = 1U << LOG_MAT_PER_SUBARRY;

    const unsigned LOG_SUBARRY_PER_BANK_PER_CHIP    = 5;
    const unsigned SUBARRY_PER_BANK_PER_CHIP        =
                    1U << LOG_SUBARRY_PER_BANK_PER_CHIP;

    const unsigned LOG_BANKS_PER_RANK       = 6;
    const unsigned BANKS_PER_RANK           = 1U << LOG_BANKS_PER_RANK;

    const unsigned LOG_CHIPS_PER_RANK       = 3;
    const unsigned CHIPS_PER_RANK           = 1U << LOG_CHIPS_PER_RANK;

    const unsigned LOG_RANK_PER_CHANNEL     = 0;
    const unsigned RANK_PER_CHANNEL         = 1U << LOG_RANK_PER_CHANNEL;

    const unsigned LOG_NUM_OF_CHANNELS      = 3;
    const unsigned NUM_OF_CHANNELS          = 1U << LOG_NUM_OF_CHANNELS;

    const unsigned PIMRankSize = MAT_ROWS*MAT_COLS*MAT_PER_SUBARRY*
            SUBARRY_PER_BANK_PER_CHIP*BANKS_PER_RANK*CHIPS_PER_RANK;

    static_assert(MAX_MAT_READ_SIZE/MAT_READ_SIZE <=
                    MAT_PER_SUBARRY,
                    "ERROR: Too few mats in a subarray for a read.");

    const unsigned LOG_PAGE_SIZE = 21;
    const Addr PAGE_SIZE = 1U << LOG_PAGE_SIZE; // 2^LOG_PAGE_SIZE bytes
    const Addr PAGEOFFSET_MASK = PAGE_SIZE - 1;
    const Addr PAGE_MASK = ~PAGEOFFSET_MASK;

    const unsigned LOG_PAGE1G_SIZE = 30;
    const Addr PAGE1G_SIZE = 1U << LOG_PAGE1G_SIZE; // 2^LOG_PAGE1G_SIZE bytes
    // we assume a 1TB PIM size
    const unsigned NUM_OF_1GBPAGES_PER_PIM = 1024/NUM_OF_CHANNELS;
    const uint64_t PAGE_RATIO = PAGE1G_SIZE/PAGE_SIZE;

    static_assert(PAGE_SIZE*BYTE_SIZE >=
              MAT_ROWS*MAT_COLS*MAT_PER_SUBARRY*CHIPS_PER_RANK,
              "ERROR: a single subarry is bigger than a 2MB page.");
    //SUBARRAYS_PER_PIMCTRL = PAGE_SIZE*BYTE_SIZE/
    //    (MAT_ROWS*MAT_COLS*MAT_PER_SUBARRY*CHIPS_PER_RANK);
    // for 2MB page we assign all subarrays in the page to a single
    // PIM controller
    const unsigned LOG_SUBARRYS_PER_PAGE_PER_CHIP =
        LOG_PAGE_SIZE+LOG_BYTE_SIZE -
            (LOG_MAT_ROWS + LOG_MAT_COLS +
            LOG_MAT_PER_SUBARRY + LOG_CHIPS_PER_RANK);
    const unsigned SUBARRYS_PER_PAGE_PER_CHIP =
                1U << LOG_SUBARRYS_PER_PAGE_PER_CHIP;
    const unsigned LOG_SUBARRAYS_PER_PIMCTRL =
            LOG_SUBARRYS_PER_PAGE_PER_CHIP;
    const unsigned SUBARRAYS_PER_PIMCTRL =
                1U << LOG_SUBARRAYS_PER_PIMCTRL;
    const unsigned MATS_PER_PIMCTRL =
                    SUBARRAYS_PER_PIMCTRL*MAT_PER_SUBARRY;

    // for 1GB page we assign maximum of 256 mats to a single PIM controller
    const unsigned LOG_SUBARRYS_PER_1GBPAGE_PER_CHIP =
        LOG_PAGE1G_SIZE+LOG_BYTE_SIZE -
            (LOG_MAT_ROWS + LOG_MAT_COLS +
            LOG_MAT_PER_SUBARRY + LOG_CHIPS_PER_RANK);
    // we allow 256 mats for a single pim controller
    const unsigned LOG_MATS_PER_PIMCTRL_1GB = 8;
    const unsigned MATS_PER_PIMCTRL_1GB =
                1U << LOG_MATS_PER_PIMCTRL_1GB;
    const unsigned LOG_SUBARRAYS_PER_PIMCTRL_1GB =
            LOG_MATS_PER_PIMCTRL_1GB-LOG_MAT_PER_SUBARRY;
    const unsigned SUBARRAYS_PER_PIMCTRL_1GB =
                1U << LOG_SUBARRAYS_PER_PIMCTRL_1GB;
    const unsigned LOG_PIMCTRL_PER_1GBPAGE_PER_CHIP =
            LOG_SUBARRYS_PER_1GBPAGE_PER_CHIP-
            LOG_SUBARRAYS_PER_PIMCTRL_1GB;
    const unsigned PIMCTRL_PER_1GBPAGE_PER_CHIP =
                1U << LOG_PIMCTRL_PER_1GBPAGE_PER_CHIP;

    const unsigned PIMCTRL_PER_BANK =
            SUBARRY_PER_BANK_PER_CHIP/SUBARRAYS_PER_PIMCTRL;

    const unsigned ROWS_PER_PIMCTRL = MATS_PER_PIMCTRL*MAT_ROWS;

    const uint64_t MAT_ROWS_PER_PAGE = MAT_ROWS*MAT_PER_SUBARRY*
                                        SUBARRAYS_PER_PIMCTRL*CHIPS_PER_RANK;
    // constants for the cache block are bad since they are configured.
    //TODO: find how to get the configured size.
    const Addr CACHEBLOCK_SIZE = 1u<<6; // 64
    const Addr CACHEBLOCK_MASK = 0xFFFFFFFFFFFFFFFF - (CACHEBLOCK_SIZE - 1);

/**  ###########    Address section      ##############
    IMPORTENT:
    The accessPIM() function is adjusted for the address partitioned here,
    it is adjusted to try and find as much locality in the host cache.
    Other address partitions should work correctly, but might be slow.
**/

    // The column bits are the lsb, 0 through
    // log2(MAT_READ_SIZE/BYTE_SIZE)-1. I don't know how to
    // do that in C++ automatically...
    // The rest of the bits come after the MAT and the CHIP
    const unsigned     ADDR_COL_IDX[LOG_MAT_COLS-LOG_BYTE_SIZE] =
                            {0,6,7,8,9,10};

    // The first lsb of the MATs decides which MAT we are looking
    // at in a subarray of a single chip in a single read, so there
    // will be log2(MAX_MAT_READ_SIZE/MAT_READ_SIZE) bits as the
    // lsb, and total of log2(MAT_PER_SUBARRY).
    const unsigned     ADDR_MAT_IDX[LOG_MAT_PER_SUBARRY] = {1,2};

    // After deciding the MATS for a single subarray of a single
    // chip in a single read, we decide in which chip we are on.
    // Since we are spreading the read on all of the chips, all
    // the CHIP bit indexes should be in the cacheblock offset.
    // If the CHIP bits are NOT in the cacheblock offset the read
    // and write timing of the operations at pim_ctrl.cc should
    // be adjusted accordingly!
    const unsigned     ADDR_CHIP_IDX[LOG_CHIPS_PER_RANK] = {3,4,5};

    const unsigned     ADDR_ROW_IDX[LOG_MAT_ROWS] =
        {11,12,13,14,15,16,17,18,19,20};

    const unsigned     ADDR_SUBARRAY_IDX[LOG_SUBARRY_PER_BANK_PER_CHIP] =
    {21,31,32,33,34};

    const unsigned     ADDR_BANK_IDX[LOG_BANKS_PER_RANK] = {25,26,27,28,29,30};

    const unsigned     ADDR_RANK_IDX[LOG_RANK_PER_CHANNEL] = {};

    const unsigned     ADDR_CHANNLE_IDX[LOG_NUM_OF_CHANNELS] = {22,23,24};

    inline unsigned getAddrField(const Addr &addr, const unsigned* field,
            const unsigned len){
        unsigned fieldVal = 0;
        unsigned bitVal;
        for ( int i = 0 ; i < len ; i++){
            bitVal = addr >> field[i]; // moving the wanted bit to locating 0
            bitVal &= 1U; // masking the wanted bit
            bitVal <<= i; // moving the bit to its intended place
            fieldVal |= bitVal; // adding the bit to the total result
        }
        return fieldVal;
    }

    inline unsigned getPIMCtrlBankIdex(const Addr &addr){
        unsigned subarry_idx = getAddrField(addr, ADDR_SUBARRAY_IDX,
                                LOG_SUBARRY_PER_BANK_PER_CHIP);
        return subarry_idx >> LOG_SUBARRAYS_PER_PIMCTRL;
    }

/**  ###########    operation data section      ##############     **/
    const unsigned PIM_IST_LEN = 64; // bits
    // the ByteSize of the payload of a packet
    const unsigned PIM_PKT_SIZE = PIM_IST_LEN/BYTE_SIZE;

    const unsigned OPCODE_LEN = 5;
    enum pimOpcode {
        SCAN_EQ         = 0x00, // equality to an immediate
        SCAN_NEQ        = 0x01, // inequality to an immediate
        SCAN_LT         = 0x02, // less-than an immediate
        SCAN_LTEQ       = 0x03, // less-than-equal an immediate
        SCAN_GT         = 0x04, // grater-equal an immediate
        SCAN_GTEQ       = 0x05, // grater-than-equal an immediate
        SCAN_BTWN       = 0x06, // between immediate1 and immediate2
        REDU_SUM        = 0x07, // sum a column
        REDU_MIN        = 0x08, // find the min value from a column
        REDU_MAX        = 0x09, // find the min value from a column
        COLIMM_ADD      = 0x0A, // add an immediate to a column
        COLIMM_SUB      = 0x0B, // sub a column from an immediate
        COLIMM_SET      = 0x0C, // set several bit column to '1' or '0'
        COL_NOT         = 0x0D, // negate several bit column
        COL_AND         = 0x0E, // AND two columns
        COL_OR          = 0x0F, // OR two columns
        COL_MUL         = 0x10, // Multiply two columns
        COL_EQ          = 0x11, // equality between two columns
        COL_NEQ         = 0x12, // inequality between two columns
        COL_LT          = 0x13, // less-than between two columns
        COL_LTE         = 0x14, // less-than-equal between two columns
        TRNS_COL        = 0x15, // transfer column to rows
        SET_INTR        = 0x16, // set the intermediates offset

        GEM5            = 0x1F  // communicate with the simulation
    };

    ////// instruction fields /////////
    // fields that belong to all types
    //(except maybe for the gem5 communication type)
    // To convert the log2 to unsigned, we assume that all of the
    // numbers in the log2 function are a power of 2, so we expect
    // to get something close to a whole number. Adding 0.5 assure
    // that the integer part of the number is the result we want, and
    // casting to unsigned take only the integer part.
    const unsigned DSTOFFSET_BIT_LEN        =
                            (unsigned)(log2(MAX_MAT_READ_SIZE)+0.5);
    const unsigned SRC1OFFSET_BIT_LEN       = LOG_MAT_COLS;
    const unsigned SRC1LEN_BIT_LEN          = LOG_PIM_OPERAND_MAX_SIZE;

    const unsigned OPCODE_BIT_START         = 0;
    const unsigned OPCODE_BIT_END           = OPCODE_BIT_START + OPCODE_LEN-1;
    const unsigned DSTOFFSET_BIT_START      = OPCODE_BIT_END + 1;
    const unsigned DSTOFFSET_BIT_END        =
                            DSTOFFSET_BIT_START + DSTOFFSET_BIT_LEN - 1;
    const unsigned SRC1OFFSET_BIT_START     = DSTOFFSET_BIT_END + 1;
    const unsigned SRC1OFFSET_BIT_END       =
                            SRC1OFFSET_BIT_START + SRC1OFFSET_BIT_LEN - 1;
    const unsigned SRC1LEN_BIT_START        = SRC1OFFSET_BIT_END + 1;
    const unsigned SRC1LEN_BIT_END          =
                            SRC1LEN_BIT_START + SRC1LEN_BIT_LEN - 1;

    const uint64_t OPCODE_MASK              = pimGetMask(OPCODE_LEN,
                                                    OPCODE_BIT_START);
    const uint64_t DSTOFFSET_MASK           = pimGetMask(DSTOFFSET_BIT_LEN,
                                                    DSTOFFSET_BIT_START);
    const uint64_t SRC1OFFSET_MASK          = pimGetMask(SRC1OFFSET_BIT_LEN,
                                                    SRC1OFFSET_BIT_START);
    const uint64_t SRC1LEN_MASK             = pimGetMask(SRC1LEN_BIT_LEN,
                                                    SRC1LEN_BIT_START);

    // fields belonging to Type1, i.e. having a single immediate
    // we use the immediate as big as possible
    const unsigned IMM_BIT_LEN = PIM_IST_LEN - 1 - SRC1LEN_BIT_END <
                                           PIM_OPERAND_MAX_SIZE ?
                                   PIM_IST_LEN - 1 - SRC1LEN_BIT_END :
                                   PIM_OPERAND_MAX_SIZE;

    const unsigned IMM_BIT_START            = SRC1LEN_BIT_END + 1;
    const unsigned IMM_BIT_END              = IMM_BIT_LEN + IMM_BIT_START - 1;

    const uint64_t IMM_MASK = pimGetMask(IMM_BIT_LEN,IMM_BIT_START);
    static_assert(IMM_BIT_END < PIM_IST_LEN,
                    "ERROR: PIM instruction Type1 is too long.");

    // fields belonging to Type2, i.e. having two immediates
    const unsigned IMM1_BIT_LEN = (PIM_IST_LEN - 1 - SRC1LEN_BIT_END)/2 <
                                            PIM_OPERAND_MAX_SIZE ?
                                    (PIM_IST_LEN - 1 - SRC1LEN_BIT_END)/2 :
                                    PIM_OPERAND_MAX_SIZE;

    const unsigned IMM2_BIT_LEN             = IMM1_BIT_LEN;

    const unsigned IMM1_BIT_START  = SRC1LEN_BIT_END + 1;
    const unsigned IMM1_BIT_END    = IMM1_BIT_START + IMM1_BIT_LEN - 1;
    const unsigned IMM2_BIT_START  = IMM1_BIT_END + 1;
    const unsigned IMM2_BIT_END    = IMM2_BIT_START + IMM2_BIT_LEN - 1;

    const uint64_t IMM1_MASK                =
                    pimGetMask(IMM1_BIT_LEN,IMM1_BIT_START);
    const uint64_t IMM2_MASK                =
                    pimGetMask(IMM2_BIT_LEN,IMM2_BIT_START);
    static_assert(IMM2_BIT_END < PIM_IST_LEN,
                    "ERROR: PIM instruction Type2 is too long.");

    // fields belonging to Type3, i.e. no immediates
    const unsigned SRC2OFFSET_BIT_LEN       = LOG_MAT_COLS;
    const unsigned SRC2LEN_BIT_LEN          =
                (unsigned)(log2(PIM_OPERAND_MAX_SIZE) + 0.5);

    const unsigned SRC2OFFSET_BIT_START     = SRC1LEN_BIT_END + 1;
    const unsigned SRC2OFFSET_BIT_END       =
                        SRC2OFFSET_BIT_START + SRC2OFFSET_BIT_LEN - 1;
    const unsigned SRC2LEN_BIT_START        = SRC2OFFSET_BIT_END + 1;
    const unsigned SRC2LEN_BIT_END          =
                        SRC2LEN_BIT_START + SRC2LEN_BIT_LEN - 1;

    const uint64_t SRC2OFFSET_MASK          =
                    pimGetMask(SRC2OFFSET_BIT_LEN,SRC2OFFSET_BIT_START);
    const uint64_t SRC2LEN_MASK             =
                    pimGetMask(SRC2LEN_BIT_LEN,SRC2LEN_BIT_START);
    static_assert(SRC2LEN_BIT_END < PIM_IST_LEN,
                    "ERROR: PIM instruction Type3 is too long.");

    //helping functions in setting PIM op fields
    inline void setOpField(uint64_t& opData, const uint64_t fieldValue,
                            const unsigned fieldStart,
                            const unsigned fieldLen){
        uint64_t mask   = ((1U<<fieldLen) - 1) << fieldStart;
        opData = (opData & (~mask)) | ((fieldValue << fieldStart) & mask);
    }

    inline uint64_t setOpImm(uint64_t opcode, uint64_t dstoffset,
                            uint64_t src1offset, uint64_t src1len,
                            uint64_t imm){
        uint64_t op = 0;
        op |= (imm << IMM_BIT_START) & IMM_MASK;
        op |= (src1len << SRC1LEN_BIT_START) & SRC1LEN_MASK;
        op |= (src1offset << SRC1OFFSET_BIT_START) & SRC1OFFSET_MASK;
        op |= (dstoffset << DSTOFFSET_BIT_START) & DSTOFFSET_MASK;
        op |= (opcode << OPCODE_BIT_START) & OPCODE_MASK;
        return op;
    }

    inline uint64_t setOpTwoImm(uint64_t opcode, uint64_t dstoffset,
                            uint64_t src1offset, uint64_t src1len,
                            uint64_t imm1, uint64_t imm2){
        uint64_t op = 0;
        op |= (imm1 << IMM1_BIT_START) & IMM1_MASK;
        op |= (imm2 << IMM2_BIT_START) & IMM2_MASK;
        op |= (src1len << SRC1LEN_BIT_START) & SRC1LEN_MASK;
        op |= (src1offset << SRC1OFFSET_BIT_START) & SRC1OFFSET_MASK;
        op |= (dstoffset << DSTOFFSET_BIT_START) & DSTOFFSET_MASK;
        op |= (opcode << OPCODE_BIT_START) & OPCODE_MASK;
        return op;
    }

    inline uint64_t setOpTwoSrc(uint64_t opcode, uint64_t dstoffset,
                            uint64_t src1offset, uint64_t src1len,
                            uint64_t src2offset, uint64_t src2len){
        uint64_t op = 0;
        op |= (src2len << SRC2LEN_BIT_START) & SRC2LEN_MASK;
        op |= (src2offset << SRC2OFFSET_BIT_START) & SRC2OFFSET_MASK;
        op |= (src1len << SRC1LEN_BIT_START) & SRC1LEN_MASK;
        op |= (src1offset << SRC1OFFSET_BIT_START) & SRC1OFFSET_MASK;
        op |= (dstoffset << DSTOFFSET_BIT_START) & DSTOFFSET_MASK;
        op |= (opcode << OPCODE_BIT_START) & OPCODE_MASK;
        return op;
    }

    const unsigned FA_CYCLE = 18; // including initialization

    // helper functions to get fields out of the PIM packet data
    inline uint64_t getFiled(const uint64_t& Data,
                            const uint64_t mask,
                            const unsigned startbit)
                            {return (Data & mask) >> startbit;}

    inline uint64_t getFiled(const uint64_t& Data,
                            const uint64_t mask,
                            const unsigned startbit,
                            const unsigned len)
                            {return ((Data & mask) >> startbit) &
                                    ((1u << len) - 1);}

    inline unsigned getDestLen(const pimOpcode op,
                            const unsigned src1len,
                            const unsigned src2len,
                            const unsigned imm){
        // return the length of the output for each operation
        switch(op){
            case SCAN_EQ:
            case SCAN_NEQ:
            case SCAN_LT:
            case SCAN_LTEQ:
            case SCAN_GT:
            case SCAN_GTEQ:
            case SCAN_BTWN:
            case COL_EQ:
            case COL_NEQ:
            case COL_LT:
            case COL_LTE:
                return 1;
            case REDU_SUM:
                return src1len + LOG_MAT_ROWS;
            case REDU_MIN:
            case REDU_MAX:
                return src1len;
            case COLIMM_ADD:
            case COLIMM_SUB:
                // (int)log2(imm)+1 is the number of bits used for imm
                return std::max(((unsigned)log2(imm)+1),src1len)+1;
            case COL_NOT:
            case COL_AND:
            case COL_OR:
            case COLIMM_SET:
                return src1len;
            case COL_MUL:
                return src1len+src2len;
            case TRNS_COL:
                return MAT_READ_SIZE;
            case SET_INTR:
                return 0;
            default:
                printf("ERROR: PIM try to execute an unknown"
                        "opcode, terminating the simulation.");
                std::abort();
        }
    }

    // this function defines the number of cycles each PIM op takes.
    // documentation on the actual computation can be found in the PIM
    // project documents (i.e., the explanation on all magic-numbers
    // are in the documentation).
    inline unsigned pimOpCycles(const uint64_t* pimOpData){
        unsigned op         = getFiled(*pimOpData,
                                OPCODE_MASK,OPCODE_BIT_START);
        unsigned src1len    = getFiled(*pimOpData,
                                SRC1LEN_MASK,SRC1LEN_BIT_START)+1;
        uint64_t imm = getFiled(*pimOpData,
                                IMM_MASK, IMM_BIT_START, src1len);
        unsigned n;
        unsigned src2len = getFiled(*pimOpData,
                          SRC2LEN_MASK,SRC2LEN_BIT_START)+1;
        uint64_t imm1;
        uint64_t imm2;
        unsigned lt_n;
        unsigned gt_n;

        int min_len;
        int max_len;

        switch(op){
            case pimOpcode::SCAN_EQ:
                n = __builtin_popcountll(imm);
                //DPRINTF(PIMop,"popcount = %d\n",n);
                return (n*3+(src1len-n)*1+1);
                break;
            case pimOpcode::SCAN_NEQ:
                n = __builtin_popcountll(imm);
                return (n*3+(src1len-n)*1+3);
                break;
            case pimOpcode::SCAN_LT:
                n = __builtin_popcountll(imm);
                return (n*3+(src1len-n)*11+4);
                break;
            case pimOpcode::SCAN_LTEQ:
                n = __builtin_popcountll(imm);
               return (n*3+(src1len-n)*11+4);
                break;
            case pimOpcode::SCAN_GT:
                n = __builtin_popcountll(imm);
                return (n*3+(src1len-n)*11+2);
                break;
            case pimOpcode::SCAN_GTEQ:
                n = __builtin_popcountll(imm);
                return (n*3+(src1len-n)*11+6);
                break;
            case pimOpcode::SCAN_BTWN:
                imm1 = getFiled(*pimOpData, IMM1_MASK,
                               IMM1_BIT_START, src1len);
                imm2 = getFiled(*pimOpData, IMM2_MASK,
                               IMM2_BIT_START, src1len);
                lt_n = __builtin_popcountll(imm1);
                gt_n = __builtin_popcountll(imm2);
                return (lt_n*3+(src1len-lt_n)*11+4+
                        gt_n*3+(src1len-gt_n)*11+2+
                        6);
                break;
            case pimOpcode::REDU_SUM:
                return LOG_MAT_ROWS*(
                            (2+FA_CYCLE)*src1len+
                            FA_CYCLE+
                            (2+FA_CYCLE)*LOG_MAT_ROWS/2
                       ) - (2+FA_CYCLE)*LOG_MAT_ROWS*3/2 +
                       2*(src1len+1)*(MAT_ROWS-1) +
                       4*(src1len+LOG_MAT_ROWS);
                       // the last term is for moving the result
                       // to the desired row.
                break;
            case pimOpcode::REDU_MIN:
            case pimOpcode::REDU_MAX:
                return 2*src1len*(13*LOG_MAT_ROWS+MAT_ROWS-1) +
                        2*LOG_MAT_ROWS;
                break;
            case pimOpcode::COLIMM_ADD:
                return FA_CYCLE*src1len + 3;
                break;
            case pimOpcode::COLIMM_SUB:
                return (FA_CYCLE + 2)*src1len + 3;
                break;
            case pimOpcode::COLIMM_SET:
                return src1len;
                break;
            case pimOpcode::COL_NOT:
                return 2*src1len;
                break;
            case pimOpcode::COL_AND:
                assert(src1len == src2len);
                return 6*src1len;
                break;
            case pimOpcode::COL_OR:
                assert(src1len == src2len);
                return 4*src1len;
                break;
            case pimOpcode::COL_MUL:
                // it is suppose to be hard coded what column is used how,
                // but for ease of the project we decide dynamically
                min_len = std::min(src1len,src2len);
                max_len = std::max(src1len,src2len);
                return min_len*max_len*(6+FA_CYCLE)-
                                    max_len*(FA_CYCLE+1)+2*min_len-1;
                break;
            case pimOpcode::COL_EQ:
                assert(src1len == src2len);
                return 11*src1len+3;
                break;
            case pimOpcode::COL_NEQ:
                assert(src1len == src2len);
                return 11*src1len+1;
                break;
            case pimOpcode::COL_LT:
                assert(src1len == src2len);
                return 16*src1len+2;
                break;
            case pimOpcode::COL_LTE:
                assert(src1len == src2len);
                return 16*src1len+6;
                break;
            case pimOpcode::TRNS_COL:
                return 2*MAT_ROWS+2;
                break;
            case pimOpcode::SET_INTR:
                return 0;
                break;
            default:
                printf("ERROR: PIM try to schedule an unknown"\
                            "opcode, terminating the simulation.");
                std::abort();
        }
    }

    const uint8_t LAST_COL_MASK[BYTE_SIZE] = {0xFF, 0x01, 0x03, 0x07, 0x0F,
                                            0x1F, 0x3F, 0x7F};
    const uint8_t FIRST_COL_ANTYMASK[BYTE_SIZE] = {0x00, 0x01, 0x03, 0x07,
                                            0x0F, 0x1F, 0x3F, 0x7F};

    // these numbers are taken from the OpenCAPI 4.0 spec.
    // the PIM op packet is as a write with different payload size.
    const unsigned OPENCAPI_CONTROLPKT_SIZE     = 64; //bytes
    const unsigned OPENCAPI_DL_HEADER_SIZE      = 8; //bytes
    const unsigned OPENCAPI_READPKT_SIZE        = 14 +
                                    OPENCAPI_DL_HEADER_SIZE; //bytes
    const unsigned OPENCAPI_READRESPPKT_SIZE    = 4 +
                                    OPENCAPI_DL_HEADER_SIZE +
                                    CACHEBLOCK_SIZE; //bytes
    const unsigned OPENCAPI_WRITEPKT_SIZE       = 14 +
                                    OPENCAPI_DL_HEADER_SIZE +
                                    CACHEBLOCK_SIZE; //bytes
    const unsigned OPENCAPI_WRITERESPPKT_SIZE   = 4 +
                                    OPENCAPI_DL_HEADER_SIZE; //bytes
    const unsigned OPENCAPI_PIMPKT_SIZE         = 14 +
                                    OPENCAPI_DL_HEADER_SIZE +
                                    PIM_PKT_SIZE; //bytes

    // energy constants, in pJ
    // read, set, and reset are from Nishil's CONCEPT paper
    const double SA_READ_ENERGY = 0.84; // pJ
    const double SET_BIT_ENERGY = 6.9; // pJ
    const double RESET_BIT_ENERGY = 6.9; // pJ
    //const double MAGIC_BIT_ENERGY = 6.9; // pJ
    // take from Nishil's paper
    // "Logic Design Within Memristive Memories Using Memristor-Aided loGIC (MAGIC)"
    const double MAGIC_BIT_ENERGY = 0.0816; // pJ

    // power figures for a single PIM controller
    // this numbers were taken from synopsys DC using
    // a Verilog implementation of the controller
    const double PIMCNTRL_LEAKAGE_POWER = 1.0549e-5; // W
    // the switch power is the Internal + Switching power
    // given by synopsys DC
    const double PIMCNTRL_SWITCH_POWER  = 1.156692e-4; // W
}

#endif // __PIM_HH__
