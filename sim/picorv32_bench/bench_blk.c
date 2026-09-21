#include <stdint.h>
#define OUTC (*(volatile uint32_t*)0x10000000)
#define OUTN (*(volatile uint32_t*)0x10000004)
static inline uint32_t rdcycle(void){uint32_t c; __asm__ volatile("rdcycle %0":"=r"(c)); return c;}
static void puts_(const char*s){while(*s)OUTC=*s++;}
static void pr(const char*n,uint32_t v){puts_(n);OUTC=' ';OUTN=v;}
static float P[36],Pd[36],x[6],xd[6];
/* dense reference: P' = F P F^T + Q, F = [I dt*I; 0 I] (generic 6x6 matmuls) */
static void predict_dense(float*Pm,float*xm,float dt,float qp,float qv){
  float F[36],T[36],R[36]; for(int i=0;i<36;i++)F[i]=0; for(int i=0;i<6;i++)F[i*7]=1.0f; F[3]=F[10]=F[17]=dt;
  for(int i=0;i<3;i++)xm[i]=xm[i]+xm[i+3]*dt;
  for(int i=0;i<6;i++)for(int j=0;j<6;j++){float s=0;for(int k=0;k<6;k++)s+=F[i*6+k]*Pm[k*6+j];T[i*6+j]=s;}
  for(int i=0;i<6;i++)for(int j=0;j<6;j++){float s=0;for(int k=0;k<6;k++)s+=T[i*6+k]*F[j*6+k];R[i*6+j]=s;}
  for(int i=0;i<36;i++)Pm[i]=R[i]; for(int i=0;i<3;i++){Pm[i*7]+=qp;Pm[(i+3)*7]+=qv;}
}
/* block-structured: only unique elements, closed-form for the constant-velocity F */
static void predict_block(float*Pm,float*xm,float dt,float qp,float qv){
  float dt2=dt*dt; float a[3][3],b[3][3],c[3][3];
  for(int i=0;i<3;i++)xm[i]=xm[i]+xm[i+3]*dt;
  for(int i=0;i<3;i++)for(int j=i;j<3;j++){ float t=Pm[i*6+3+j]+Pm[j*6+3+i]; a[i][j]=Pm[i*6+j]+dt*t+dt2*Pm[(3+i)*6+3+j]; if(i==j)a[i][j]+=qp; }
  for(int i=0;i<3;i++)for(int j=0;j<3;j++) b[i][j]=Pm[i*6+3+j]+dt*Pm[(3+i)*6+3+j];
  for(int i=0;i<3;i++)for(int j=i;j<3;j++){ c[i][j]=Pm[(3+i)*6+3+j]; if(i==j)c[i][j]+=qv; }
  for(int i=0;i<3;i++)for(int j=i;j<3;j++){ Pm[i*6+j]=a[i][j]; Pm[j*6+i]=a[i][j]; Pm[(3+i)*6+3+j]=c[i][j]; Pm[(3+j)*6+3+i]=c[i][j]; }
  for(int i=0;i<3;i++)for(int j=0;j<3;j++){ Pm[i*6+3+j]=b[i][j]; Pm[(3+j)*6+i]=b[i][j]; }
}
int main(void){
  for(int i=0;i<6;i++){x[i]=xd[i]=0.5f*(i+1);for(int j=0;j<6;j++){float v=(i==j)?(10.0f+i):0.05f*(1+((i+j)&3)); P[i*6+j]=P[j*6+i]=v;}}
  for(int i=0;i<36;i++)Pd[i]=P[i];
  uint32_t t0=rdcycle(); predict_dense(Pd,xd,0.01f,0.1f,0.01f); uint32_t t1=rdcycle();
  predict_block(P,x,0.01f,0.1f,0.01f); uint32_t t2=rdcycle();
  /* warm second call for steady-state numbers */
  predict_dense(Pd,xd,0.01f,0.1f,0.01f); uint32_t t3=rdcycle(); predict_block(P,x,0.01f,0.1f,0.01f); uint32_t t4=rdcycle();
  pr("dense_predict_cycles",t1-t0); pr("block_predict_cycles",t2-t1); pr("dense_predict_cycles_2nd",t3-t2); pr("block_predict_cycles_2nd",t4-t3);
  float md=0; for(int i=0;i<36;i++){float d=P[i]-Pd[i]; if(d<0)d=-d; if(d>md)md=d;} pr("max_abs_diff_x1e6",(uint32_t)(md*1e6f));
  return 0; }
