# How Each CPU was benchmarked

## Compiling Code 

In order to compile code, the riscv-gnu tool chain is required. In the 20190608-Base-Ratified Riscv ISA manual, the Zicsr instructions, which are required by these cpu designs in order to print output during simulation, were seperated from the base instruction set. The latest versions of the riscv toolchain will not compile the code with the flags present in the makefiles in the biriscv folder. 

With the correct multi-lib support, modern versions of gcc could work for compiling code for these CPUs, however, for simplicity, gcc 10.2 was used. While the author of these CPUs did not specify the compiler flags used for their reported benchmark scores, gcc 10.2 would be from around the same time these repositories were published, allowing for a closer comparison of our benchmark results and the authors of the CPU. 

The biriscv directory README contains instructions for setting up the build environment in order to compile ELF files for the three CPUs, and to run the testbench for the two pipelined CPUs. When building the riscv tool chain, ensure that the version being built is from June 2021 or earlier. Newer versions should work but will require chaning the march flags, and were not tested. 

Code is compiled using the Makefiles in the biriscv directory, and once the ELF is generated, it can be moved to the testbench areas of the other CPUs being tested. These makefiles can be modified to compile custom targets. We utilized the hello.c source file to implement test instructions (avoided needing to expand the build system).

## Multi Cycle 

Copy ELF into the tb directory. Rename this file to test.elf, and run make (after setting the correct environment variables). 

This did not work on Ubuntu 24.04 (the riscv tools were not found despite being on the PATH). To get around this, comment out the check for the riscv tool chain. Run make. Use the riscv tool chain objcopy utility to generate a binary file called tcm.bin (for example, riscv64-unknown-elf-objcopy hello.elf -O binary tcm.bin). Copy this file into the build directory, and then run make again. 

Parameters can be changed by modifying the riscv_core.v file in the src_v folder. 

## Simple Pipeline

Copy the elf file into the project (i.e. isa_sim/images/). Enter the top_cache_axi folder, and then the tb folder. Modify the top level makefile so that the name of the TEST_IMAGE matches the copied over elf. Run make run, and the code should execute. 

Parameters can be modified by changing them in the core/riscv/riscv_core.v file. 

## Dual Issue

At the top level, run make tcm=1 name_of_executable_in_sw, and the test code will compile, and the verilator tb will compile. The source .c files in any of the folders could be modified, and the process to compile and run the program will remain the same. Additional targets could be added to the makefile in the future. 

To change paramters, modify them in the src/top/riscv_tcm_top.v file. Run make clean between runs after changing parameters to ensure accurate results.  




