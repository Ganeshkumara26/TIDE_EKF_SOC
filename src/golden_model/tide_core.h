#ifndef TIDE_CORE_H
#define TIDE_CORE_H
#include <stdint.h>
#include <math.h>
#ifdef COUNT_OPS
extern long c_mul,c_add,c_div,c_sqrt;
#define FMUL(a,b) (c_mul++,(a)*(b))
#define FADD(a,b) (c_add++,(a)+(b))
#define FSUB(a,b) (c_add++,(a)-(b))
#define FDIV(a,b) (c_div++,(a)/(b))
#define FSQRT(a)  (c_sqrt++,sqrtf(a))
#else
#define FMUL(a,b) ((a)*(b))
#define FADD(a,b) ((a)+(b))
#define FSUB(a,b) ((a)-(b))
#define FDIV(a,b) ((a)/(b))
#define FSQRT(a)  sqrtf(a)
#endif
typedef struct { float x[6]; float P[21]; } tide_st;          /* P: packed lower triangle, 21 unique elements */
static inline int pk(int i,int j){ return i>=j ? (i*(i+1))/2+j : (j*(j+1))/2+i; }
/* ---- atan2 in microcode-friendly form: octant reduction + Cephes-style minimax polynomial (no CORDIC, no libm) ---- */
static float tide_atan_red(float t){
  float a0=0.0f;
  if(t>0.41421356f){ float num=FSUB(t,1.0f),den=FADD(t,1.0f); t=FDIV(num,den); a0=0.78539816f; }
  float z=FMUL(t,t);
  float p=8.05374449538e-2f;
  p=FADD(FMUL(p,z),-1.38776856032e-1f); p=FADD(FMUL(p,z),1.99777106478e-1f); p=FADD(FMUL(p,z),-3.33329491539e-1f);
  p=FMUL(FMUL(p,z),t);
  return FADD(FADD(p,t),a0);
}
static float tide_atan2(float y,float x){
  float ax=fabsf(x),ay=fabsf(y); float mx=ax>ay?ax:ay, mn=ax>ay?ay:ax;
  if(mx==0.0f) return 0.0f;
  float a=tide_atan_red(FDIV(mn,mx));
  if(ay>ax) a=FSUB(1.57079633f,a);
  if(x<0.0f) a=FSUB(3.14159265f,a);
  if(y<0.0f) a=-a;
  return a;
}
/* ---- event-driven predict: constant-velocity F(dt), Q(dt) = (dt/dt0)*Q0, closed-form block equations on packed P ---- */
static void tide_predict(tide_st*s,float dt,float dt0inv,float qp,float qv){
  float dt2=FMUL(dt,dt), sc=FMUL(dt,dt0inv), qps=FMUL(qp,sc), qvs=FMUL(qv,sc);
  float *P=s->P;
  for(int i=0;i<3;i++) s->x[i]=FADD(s->x[i],FMUL(s->x[i+3],dt));
  float a[6],b[9];
  for(int i=0;i<3;i++)for(int j=0;j<=i;j++){
    float t=FADD(P[pk(i,3+j)],P[pk(j,3+i)]);
    float v=FADD(FADD(P[pk(i,j)],FMUL(dt,t)),FMUL(dt2,P[pk(3+i,3+j)]));
    if(i==j) v=FADD(v,qps);
    a[(i*(i+1))/2+j]=v; }
  for(int i=0;i<3;i++)for(int j=0;j<3;j++) b[i*3+j]=FADD(P[pk(i,3+j)],FMUL(dt,P[pk(3+i,3+j)]));
  for(int i=0;i<3;i++)for(int j=0;j<=i;j++) P[pk(i,j)]=a[(i*(i+1))/2+j];
  for(int i=0;i<3;i++)for(int j=0;j<3;j++) P[pk(3+j,i)]=b[i*3+j];       /* B(i,j) lives at (3+j,i) in the lower triangle */
  for(int i=0;i<3;i++) P[pk(3+i,3+i)]=FADD(P[pk(3+i,3+i)],qvs);
}
/* ---- scalar (sequential) measurement update, implicit H row h[0..nc-1], Joseph rank-1 form or simplified ---- */
static void tide_scalar_update(tide_st*s,const float*h,int nc,float y,float R,int joseph){
  float u[6],K[6]; float *P=s->P;
  for(int i=0;i<6;i++){ float acc=FMUL(P[pk(i,0)],h[0]); for(int j=1;j<nc;j++) acc=FADD(acc,FMUL(P[pk(i,j)],h[j])); u[i]=acc; }
  float S=FMUL(h[0],u[0]); for(int j=1;j<nc;j++) S=FADD(S,FMUL(h[j],u[j])); S=FADD(S,R);
  float iS=FDIV(1.0f,S);
  for(int i=0;i<6;i++) K[i]=FMUL(u[i],iS);
  for(int i=0;i<6;i++) s->x[i]=FADD(s->x[i],FMUL(K[i],y));
  if(!joseph){ for(int i=0;i<6;i++)for(int j=0;j<=i;j++) P[pk(i,j)]=FSUB(P[pk(i,j)],FMUL(K[i],u[j])); return; }
  float T[36],v[6],rk[6];
  for(int i=0;i<6;i++)for(int j=0;j<6;j++) if(i>=j||j<nc) T[i*6+j]=FSUB(P[pk(i,j)],FMUL(K[i],u[j]));
  for(int i=0;i<6;i++){ float acc=FMUL(T[i*6],h[0]); for(int l=1;l<nc;l++) acc=FADD(acc,FMUL(T[i*6+l],h[l])); v[i]=acc; }
  for(int i=0;i<6;i++) rk[i]=FMUL(R,K[i]);
  for(int i=0;i<6;i++)for(int j=0;j<=i;j++) P[pk(i,j)]=FADD(FSUB(T[i*6+j],FMUL(v[i],K[j])),FMUL(rk[i],K[j]));
}
/* ---- radar update: geometry + implicit Jacobian rows (never stored as a 4x6 matrix), 4 sequential scalars at the PRIOR linearization ---- */
static void tide_update(tide_st*s,const float*z,const float*R,int joseph){
  float pn=s->x[0],pe=s->x[1],pd=s->x[2],vn=s->x[3],ve=s->x[4],vd=s->x[5];
  float pn2=FMUL(pn,pn),pe2=FMUL(pe,pe),pd2=FMUL(pd,pd);
  float rg2=FADD(pn2,pe2); float r2=FADD(rg2,pd2); if(r2<1e-6f) r2=1e-6f; if(rg2<1e-6f) rg2=1e-6f;
  float r=FSQRT(r2), rg=FSQRT(rg2);
  float ir=FDIV(1.0f,r), irg2=FDIV(1.0f,rg2); float ir2=FMUL(ir,ir), irg=FMUL(rg,irg2);
  float az=tide_atan2(pe,pn), el=tide_atan2(-pd,rg);
  float dot=FADD(FADD(FMUL(vn,pn),FMUL(ve,pe)),FMUL(vd,pd)); float dop=FMUL(dot,ir);
  float y[4]; y[0]=FSUB(z[0],r); y[1]=FSUB(z[1],az); y[2]=FSUB(z[2],el); y[3]=FSUB(z[3],dop);
  if(y[1]>3.14159265f) y[1]=FSUB(y[1],6.28318531f); else if(y[1]<-3.14159265f) y[1]=FADD(y[1],6.28318531f);
  float h0[3]={FMUL(pn,ir),FMUL(pe,ir),FMUL(pd,ir)};
  float h1[3]={-FMUL(pe,irg2),FMUL(pn,irg2),0.0f};
  float c=FMUL(ir2,irg); float h2[3]={FMUL(FMUL(pd,pn),c),FMUL(FMUL(pd,pe),c),-FMUL(rg,ir2)};
  float dopir=FMUL(dop,ir); float h3[6]={FMUL(FSUB(vn,FMUL(dopir,pn)),ir),FMUL(FSUB(ve,FMUL(dopir,pe)),ir),FMUL(FSUB(vd,FMUL(dopir,pd)),ir),h0[0],h0[1],h0[2]};
  /* exact equivalence to the batch update (diagonal R): each later innovation is corrected for the state change
     produced by the earlier scalars:  y_k' = y_k - h_k . (x_now - x_prior)  */
  float xp[6]; for(int i=0;i<6;i++) xp[i]=s->x[i];
  const float *hk[4]={h0,h1,h2,h3}; const int nck[4]={3,3,3,6};
  for(int k=0;k<4;k++){
    float yk=y[k];
    if(k>0){ float corr=FMUL(hk[k][0],FSUB(s->x[0],xp[0])); for(int j=1;j<nck[k];j++) corr=FADD(corr,FMUL(hk[k][j],FSUB(s->x[j],xp[j]))); yk=FSUB(yk,corr); }
    tide_scalar_update(s,hk[k],nck[k],yk,R[k],joseph);
  }
}
#endif
