
#ifndef __CLFLUSH_HH__
#define __CLFLUSH_HH__

inline void clflush(volatile void *p)
            {
                asm volatile ("clflush (%0)" :: "r"(p));
            }

#endif // __CLFLUSH_HH__