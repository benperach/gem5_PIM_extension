# gem5_PIM_extension
A gem5 extension, introducing a bulk-bitwise PIM module. This extension implement bulk-bitwise PIM modulw with a programming model and additional X86/ARM instruction to interact with the PIM module.  

This extention was developed during the work on: 
See the paper for more details about the extention.
Plese cite the above paper when using this extention.

The extention is on gem5 19.

Assigning pages for PIM:
The PIM operations are designed to work on a 2MB pages. In order to assign 2MB page to the PIM module you must first allocate a 2MB page and then issue a PIM instruction with opcode 0x1f to an address in that page. The PIM instruction with that opcode is recognized by the DRAM controller and mark the page address range as to be assign to the PIM module. Note that any data writen to the page before issuing the PIM instruction will be lost.
To assign 2MB pages in linux, first you must ask linux to allocate enough 2MB huge pages and then in the application code asked (using mmap) for 2MB pages. The hack_back_ckpt.rcS in this project ask linux, after boot and before checkpointing, to assign 1024 2MB pages (the rest of the file is as supplied by the gem5 project). This is an example that can be modified as needed. 

PIM instructions:
For X86, the PIM instruction in inline assmbler is included in the piminst_X86.h file. For ARM, the PIM instruction in inline assmbler is included in the piminst_arm.h file. To use the PIM instruction in a program code, include the relevant file and use the pimInst function from the file.
The pimInst function receive data (64bit) and address (64bit). 
The data include the various fields for the PIM operations, defined by the specific PIM module used. The data fields are only addressed by the PIM module (with the exception of the page assigning procedure), so the fields can be adjusted in the src/mem/pim_cntrl.cc src/mem/pim_cntrl.hh src/mem/pim_lib.hh files.
The address of the pimInst function is the address for the page the operation will be performed on, and also specifiey the row/column/crossbar of the result (instruction might not use some or all of these fields). 
There are some halping functions in src/mem/pim_lib.hh that can help in building the data and address for the PIM instructions in your application code.

Changing addressing parameters: 
In the file src/mem/pim_lib.hh there is the addressing scheme of the PIM modules (address bit that indicate row, column, crossbar, subarry, bank, channel). This scheme can be changed manually by changing the parameters in this file. The addressing determind which address go to which location in the PIM module.

Adding PIM instructions:
This is done by assigning the relevent opcode and latency in the src/mem/pim_lib.hh file. To perform the actual PIM operation on the simulated memory, the accessPIM function in src/mem/pim_ctrl.cc file has to be adjusted.
