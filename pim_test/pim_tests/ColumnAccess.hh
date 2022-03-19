#include <stdint.h>
#include <sys/mman.h>

#include <cstdio>
#include <cstdlib>

#include "/home/ben/gem5/src/mem/pim.hh"
#include "clflush.h"

Addr getAddrMask(const unsigned* field, const unsigned len){
    unsigned fieldVal = 0;
    unsigned bitVal;
    for (int i = 0 ; i < len ; i++){
        bitVal = 1U<<field[i];
        fieldVal |= bitVal; // adding the bit to the total result
    }
    return fieldVal;
}

// functions to generate constants to for the mPIM addressing
void generateConstFieldAddr(uintptr_t* mask_arr, const unsigned* idx,
                    const unsigned idx_len){
    // the index length must be the log2 of the output array length
    int arr_len = 1U<<idx_len;
    // for each number, go over all the relevant bits of it and set
    // them in the right location.
    for (int num = 0 ; num < arr_len ; num ++) {
        mask_arr[num] = 0;
        for (int i = 0 ; i < idx_len ; i++) {
            mask_arr[num] |= (uintptr_t)(((num>>i) & 1U)<<idx[i]);
        }
    }
}

class ColumnOperation{
  public:
    ColumnOperation() {};
    virtual void operator() (uint8_t* addr, int col, int subarray,
                    int row, int mat, int chip, int startCol) =0;
};

// this class needs to have only a single object,
// it is just to wrap things up in a clean way
class columnOperate{
  private :

  public :
    uintptr_t ADDR_MAT_MASK;
    uintptr_t ADDR_CHIP_MASK;
    uintptr_t ADDR_ROW_MASK;
    uintptr_t ADDR_COL_MASK;
    uintptr_t ADDR_SUBARRYS_PER_PIMCTRL_MASK;

    uintptr_t ADDR_MAT_NUM_ARRAY[1U<<pim::LOG_MAT_PER_SUBARRY];
    uintptr_t ADDR_CHIP_NUM_ARRAY[1U<<pim::LOG_CHIPS_PER_RANK];
    uintptr_t ADDR_ROW_NUM_ARRAY[1U<<pim::LOG_MAT_ROWS];
    uintptr_t ADDR_COL_NUM_ARRAY[1U<<(pim::LOG_MAT_COLS-pim::LOG_BYTE_SIZE)];
    uintptr_t ADDR_SUBARRYS_PER_PIMCTRL_NUM_ARRAY
                  [1U<<pim::LOG_SUBARRYS_PER_PIMCTRL];

    columnOperate(){
        // initialize arrays for the implementation of the mPIM operation.
        // The arrays contain constant value, which are pre-calculated here.
        ADDR_MAT_MASK =
            getAddrMask(pim::ADDR_MAT_IDX,pim::LOG_MAT_PER_SUBARRY);
        ADDR_CHIP_MASK =
            getAddrMask(pim::ADDR_CHIP_IDX,pim::LOG_CHIPS_PER_RANK);
        ADDR_ROW_MASK =
            getAddrMask(pim::ADDR_ROW_IDX,pim::LOG_MAT_ROWS);
        ADDR_COL_MASK =
            getAddrMask(pim::ADDR_COL_IDX,pim::LOG_MAT_COLS
                                        -pim::LOG_BYTE_SIZE);
        ADDR_SUBARRYS_PER_PIMCTRL_MASK =
            getAddrMask(pim::ADDR_SUBARRAY_IDX,
                        pim::LOG_SUBARRYS_PER_PIMCTRL);
        generateConstFieldAddr(ADDR_MAT_NUM_ARRAY,
            pim::ADDR_MAT_IDX,pim::LOG_MAT_PER_SUBARRY);
        generateConstFieldAddr(ADDR_CHIP_NUM_ARRAY,
            pim::ADDR_CHIP_IDX,pim::LOG_CHIPS_PER_RANK);
        generateConstFieldAddr(ADDR_ROW_NUM_ARRAY,
            pim::ADDR_ROW_IDX,pim::LOG_MAT_ROWS);
        generateConstFieldAddr(ADDR_COL_NUM_ARRAY,
            pim::ADDR_COL_IDX,pim::LOG_MAT_COLS-pim::LOG_BYTE_SIZE);
        generateConstFieldAddr(ADDR_SUBARRYS_PER_PIMCTRL_NUM_ARRAY,
                pim::ADDR_SUBARRAY_IDX, pim::LOG_SUBARRYS_PER_PIMCTRL);
        /*for (int i = 0 ;
            i < (1U<<pim::LOG_MAT_ROWS) ;
            i++)
        {
            printf("ADDR_ROW_NUM_ARRAY[%d] = 0x%lX\n",
                    i,ADDR_ROW_NUM_ARRAY[i]);
        }
        for (int i = 0 ;
            i < (1U<<pim::LOG_MAT_PER_SUBARRY) ;
            i++)
        {
            printf("ADDR_MAT_NUM_ARRAY[%d] = 0x%lX\n",
                    i,ADDR_MAT_NUM_ARRAY[i]);
        }
        for (int i = 0 ;
            i < (1U<<pim::LOG_CHIPS_PER_RANK) ;
            i++)
        {
            printf("ADDR_CHIP_NUM_ARRAY[%d] = 0x%lX\n",
                    i,ADDR_CHIP_NUM_ARRAY[i]);
        }
        for (int i = 0 ;
            i < (1U<<pim::LOG_MAT_COLS-pim::LOG_BYTE_SIZE) ;
            i++)
        {
            printf("ADDR_COL_NUM_ARRAY[%d] = 0x%lX\n",
                    i,ADDR_COL_NUM_ARRAY[i]);
        }
        for (int i = 0 ;
            i < (1U<<pim::LOG_SUBARRYS_PER_PIMCTRL) ;
            i++)
        {
            printf("ADDR_SUBARRYS_PER_PIMCTRL_NUM_ARRAY[%d] = 0x%lX\n",
                    i,ADDR_SUBARRYS_PER_PIMCTRL_NUM_ARRAY[i]);
        }*/
    }

    virtual void operator() (void* addr, int startCol, int colNum,
                     int colStrid, ColumnOperation* colFunc)
    {
        uint8_t* addr_col;
        uint8_t* addr_sub;
        uint8_t* addr_row;
        uint8_t* addr_mat;
        uint8_t* addr_chip;
        for (int col = startCol ; col < startCol+colNum ;
                                col+=colStrid)
        {
            addr_col = (uint8_t*)((uintptr_t)addr +
                        ADDR_COL_NUM_ARRAY[col]);
            for (int subarray = 0 ; subarray < pim::SUBARRYS_PER_PIMCTRL ;
                                                            subarray ++)
            {
                addr_sub = (uint8_t*)((uintptr_t)addr_col +
                            ADDR_SUBARRYS_PER_PIMCTRL_NUM_ARRAY[subarray]);
                for (int row = 0 ; row < pim::MAT_ROWS ; row ++)
                {
                    addr_row = (uint8_t*)((uintptr_t)addr_sub +
                                        ADDR_ROW_NUM_ARRAY[row]);
                    // go through all mats in a subarray
                    // (relevant bits in the cache offset)
                    for (int mat = 0 ;
                        mat < pim::MAT_PER_SUBARRY;
                        mat++)
                    {
                        addr_mat = (uint8_t*)((uintptr_t)addr_row +
                                    ADDR_MAT_NUM_ARRAY[mat]);
                        for (int chip = 0;
                            chip < pim::CHIPS_PER_RANK;
                            chip++)
                        {
                            addr_chip = (uint8_t*)((uintptr_t)addr_mat +
                                        ADDR_CHIP_NUM_ARRAY[chip]);
                            // do the operation
                            (*colFunc)(addr_chip, col, subarray,
                                            row, mat, chip, startCol);
                        }
                    }
                }
            }
        }
    }
};

class colWrite : public ColumnOperation {
  public :
    colWrite() {};
    virtual void operator() (uint8_t* addr, int col, int subarray,
                                int row, int mat, int chip,
                                int startCol)
    {
        // write the number of row, cut it to fit a byte
        *addr = (uint8_t)(row % 256);
        /*if (row == 0 || row == 1 || row == 2)
            printf("addr 0x%lX "
                    "in row 0x%X, mat 0x%X, subarray 0x%X"
                    ", chip 0x%X, result = 0x%X\n",
                    (uintptr_t)addr,
                    row,mat,subarray,chip, (uint8_t)(row % 256));
        */
    }
};

class colFlush : public ColumnOperation {
  public :
    colFlush() {};
    virtual void operator() (uint8_t* addr, int col, int subarray,
                                int row, int mat, int chip,
                                int startCol)
    {
        // flush the data
        clflush((void *)addr);
        /*if ((col >= 3) && (row == 1))
            printf("addr 0x%lX "
                    "in row 0x%X, mat 0x%X, subarray 0x%X"
                    ", chip 0x%X\n",
                    (uintptr_t)addr,
                    row,mat,subarray,chip);
        */
    }
};

