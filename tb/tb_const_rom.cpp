#include "Vtide_const_rom.h"
#include "verilated.h"
#include <cstdio>

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    auto dut = new Vtide_const_rom;
    uint32_t expected[64];
    for (int i=0;i<64;i++) expected[i]=0;
    expected[0]=0x40490FDB; expected[1]=0x3FC90FDB; expected[2]=0x40C90FDB; expected[3]=0xC0490FDB;
    expected[4]=0x3F800000; expected[5]=0x00000000; expected[6]=0xBF800000; expected[7]=0x358637BD;
    expected[8]=0x3ED413CD; expected[9]=0x401A827A; expected[10]=0x3F490FDB; expected[11]=0x3F7FFFF5;
    expected[12]=0xBEAAA52E; expected[13]=0x3E4C7B66; expected[14]=0xBE084A28; expected[15]=0x3727C5AC;
    expected[16]=0x322BCC77; expected[17]=0x3F000000; expected[18]=0x40000000; expected[19]=0xC0000000;
    int pass=0, fail=0;
    for (int i=0;i<64;i++) {
        dut->addr_i = i;
        dut->eval();
        if (dut->data_o == expected[i]) pass++;
        else { fail++; printf("ROM FAIL addr=%d got=%08x exp=%08x\n", i, dut->data_o, expected[i]); }
    }
    printf("CONST ROM test: %d pass, %d fail out of 64\n", pass, fail);
    return fail==0?0:1;
}
