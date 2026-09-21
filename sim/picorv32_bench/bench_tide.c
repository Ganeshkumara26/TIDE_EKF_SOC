#include <stdint.h>
#include "tide_core.h"
#define OUTC (*(volatile uint32_t*)0x10000000)
#define OUTN (*(volatile uint32_t*)0x10000004)
static inline uint32_t rdcycle(void){uint32_t c; __asm__ volatile("rdcycle %0":"=r"(c)); return c;}
static void puts_(const char*s){while(*s)OUTC=*s++;}
static void pr(const char*n,uint32_t v){puts_(n);OUTC=' ';OUTN=v;}
static void init_st(tide_st*s){ for(int i=0;i<6;i++)s->x[i]=0; for(int i=0;i<21;i++)s->P[i]=0; for(int i=0;i<3;i++){s->P[pk(i,i)]=100.0f;s->P[pk(3+i,3+i)]=25.0f;} }
int main(void){
  const float R[4]={1.0f,0.001f,0.001f,0.5f}; float z[4]={1100.0f,0.46f,0.18f,20.0f}; tide_st s;
  for(int rep=0;rep<2;rep++){
    init_st(&s); s.x[0]=1000;s.x[1]=500;s.x[2]=-200;s.x[3]=20;s.x[4]=10;s.x[5]=-2;
    uint32_t t0=rdcycle(); tide_predict(&s,0.01f,100.0f,0.1f,0.01f); uint32_t t1=rdcycle(); tide_update(&s,z,R,1); uint32_t t2=rdcycle();
    init_st(&s); s.x[0]=1000;s.x[1]=500;s.x[2]=-200;s.x[3]=20;s.x[4]=10;s.x[5]=-2;
    tide_predict(&s,0.01f,100.0f,0.1f,0.01f); uint32_t t3=rdcycle(); tide_update(&s,z,R,0); uint32_t t4=rdcycle();
    if(rep==1){ pr("predict_block_cycles",t1-t0); pr("update_joseph_seq_cycles",t2-t1); pr("update_simplified_seq_cycles",t4-t3); }
  }
  return 0; }
