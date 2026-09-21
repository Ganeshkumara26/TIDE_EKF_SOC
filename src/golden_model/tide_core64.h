#ifndef TIDE64_H
#define TIDE64_H
#include <stdint.h>
#include <math.h>
#ifdef COUNT_OPS
extern long c_mul,c_add,c_div,c_sqrt;
#define DMUL(a,b) (c_mul++,(a)*(b))
#define DADD(a,b) (c_add++,(a)+(b))
#define DSUB(a,b) (c_add++,(a)-(b))
#define DDIV(a,b) (c_div++,(a)/(b))
#define DSQRT(a)  (c_sqrt++,sqrt(a))
#else
#define DMUL(a,b) ((a)*(b))
#define DADD(a,b) ((a)+(b))
#define DSUB(a,b) ((a)-(b))
#define DDIV(a,b) ((a)/(b))
#define DSQRT(a)  sqrt(a)
#endif
typedef struct { double x[6]; double P[21]; } tide64_st;          /* P: packed lower triangle, 21 unique elements */
static inline int pk64(int i,int j){ return i>=j ? (i*(i+1))/2+j : (j*(j+1))/2+i; }
/* ---- atan2 in microcode-friendly form: octant reduction + Cephes-style minimax polynomial (no CORDIC, no libm) ---- */
static double tide64_atan_red(double t){
  double a0=0.0f;
  if(t>0.41421356f){ double num=DSUB(t,1.0f),den=DADD(t,1.0f); t=DDIV(num,den); a0=0.78539816f; }
  double z=DMUL(t,t);
  double p=8.05374449538e-2f;
  p=DADD(DMUL(p,z),-1.38776856032e-1f); p=DADD(DMUL(p,z),1.99777106478e-1f); p=DADD(DMUL(p,z),-3.33329491539e-1f);
  p=DMUL(DMUL(p,z),t);
  return DADD(DADD(p,t),a0);
}
static double tide64_atan2(double y,double x){
  double ax=fabs(x),ay=fabs(y); double mx=ax>ay?ax:ay, mn=ax>ay?ay:ax;
  if(mx==0.0f) return 0.0f;
  double a=tide64_atan_red(DDIV(mn,mx));
  if(ay>ax) a=DSUB(1.57079633f,a);
  if(x<0.0f) a=DSUB(3.14159265f,a);
  if(y<0.0f) a=-a;
  return a;
}
/* ---- event-driven predict: constant-velocity F(dt), Q(dt) = (dt/dt0)*Q0, closed-form block equations on packed P ---- */
static void tide64_predict(tide64_st*s,double dt,double dt0inv,double qp,double qv){
  double dt2=DMUL(dt,dt), sc=DMUL(dt,dt0inv), qps=DMUL(qp,sc), qvs=DMUL(qv,sc);
  double *P=s->P;
  for(int i=0;i<3;i++) s->x[i]=DADD(s->x[i],DMUL(s->x[i+3],dt));
  double a[6],b[9];
  for(int i=0;i<3;i++)for(int j=0;j<=i;j++){
    double t=DADD(P[pk64(i,3+j)],P[pk64(j,3+i)]);
    double v=DADD(DADD(P[pk64(i,j)],DMUL(dt,t)),DMUL(dt2,P[pk64(3+i,3+j)]));
    if(i==j) v=DADD(v,qps);
    a[(i*(i+1))/2+j]=v; }
  for(int i=0;i<3;i++)for(int j=0;j<3;j++) b[i*3+j]=DADD(P[pk64(i,3+j)],DMUL(dt,P[pk64(3+i,3+j)]));
  for(int i=0;i<3;i++)for(int j=0;j<=i;j++) P[pk64(i,j)]=a[(i*(i+1))/2+j];
  for(int i=0;i<3;i++)for(int j=0;j<3;j++) P[pk64(3+j,i)]=b[i*3+j];       /* B(i,j) lives at (3+j,i) in the lower triangle */
  for(int i=0;i<3;i++) P[pk64(3+i,3+i)]=DADD(P[pk64(3+i,3+i)],qvs);
}
/* ---- scalar (sequential) measurement update, implicit H row h[0..nc-1], Joseph rank-1 form or simplified ---- */
static void tide64_scalar_update(tide64_st*s,const double*h,int nc,double y,double R,int joseph){
  double u[6],K[6]; double *P=s->P;
  for(int i=0;i<6;i++){ double acc=DMUL(P[pk64(i,0)],h[0]); for(int j=1;j<nc;j++) acc=DADD(acc,DMUL(P[pk64(i,j)],h[j])); u[i]=acc; }
  double S=DMUL(h[0],u[0]); for(int j=1;j<nc;j++) S=DADD(S,DMUL(h[j],u[j])); S=DADD(S,R);
  double iS=DDIV(1.0f,S);
  for(int i=0;i<6;i++) K[i]=DMUL(u[i],iS);
  for(int i=0;i<6;i++) s->x[i]=DADD(s->x[i],DMUL(K[i],y));
  if(!joseph){ for(int i=0;i<6;i++)for(int j=0;j<=i;j++) P[pk64(i,j)]=DSUB(P[pk64(i,j)],DMUL(K[i],u[j])); return; }
  double T[36],v[6],rk[6];
  for(int i=0;i<6;i++)for(int j=0;j<6;j++) if(i>=j||j<nc) T[i*6+j]=DSUB(P[pk64(i,j)],DMUL(K[i],u[j]));
  for(int i=0;i<6;i++){ double acc=DMUL(T[i*6],h[0]); for(int l=1;l<nc;l++) acc=DADD(acc,DMUL(T[i*6+l],h[l])); v[i]=acc; }
  for(int i=0;i<6;i++) rk[i]=DMUL(R,K[i]);
  for(int i=0;i<6;i++)for(int j=0;j<=i;j++) P[pk64(i,j)]=DADD(DSUB(T[i*6+j],DMUL(v[i],K[j])),DMUL(rk[i],K[j]));
}
/* ---- radar update: geometry + implicit Jacobian rows (never stored as a 4x6 matrix), 4 sequential scalars at the PRIOR linearization ---- */
static void tide64_update(tide64_st*s,const double*z,const double*R,int joseph){
  double pn=s->x[0],pe=s->x[1],pd=s->x[2],vn=s->x[3],ve=s->x[4],vd=s->x[5];
  double pn2=DMUL(pn,pn),pe2=DMUL(pe,pe),pd2=DMUL(pd,pd);
  double rg2=DADD(pn2,pe2); double r2=DADD(rg2,pd2); if(r2<1e-6f) r2=1e-6f; if(rg2<1e-6f) rg2=1e-6f;
  double r=DSQRT(r2), rg=DSQRT(rg2);
  double ir=DDIV(1.0f,r), irg2=DDIV(1.0f,rg2); double ir2=DMUL(ir,ir), irg=DMUL(rg,irg2);
  double az=tide64_atan2(pe,pn), el=tide64_atan2(-pd,rg);
  double dot=DADD(DADD(DMUL(vn,pn),DMUL(ve,pe)),DMUL(vd,pd)); double dop=DMUL(dot,ir);
  double y[4]; y[0]=DSUB(z[0],r); y[1]=DSUB(z[1],az); y[2]=DSUB(z[2],el); y[3]=DSUB(z[3],dop);
  if(y[1]>3.14159265f) y[1]=DSUB(y[1],6.28318531f); else if(y[1]<-3.14159265f) y[1]=DADD(y[1],6.28318531f);
  double h0[3]={DMUL(pn,ir),DMUL(pe,ir),DMUL(pd,ir)};
  double h1[3]={-DMUL(pe,irg2),DMUL(pn,irg2),0.0f};
  double c=DMUL(ir2,irg); double h2[3]={DMUL(DMUL(pd,pn),c),DMUL(DMUL(pd,pe),c),-DMUL(rg,ir2)};
  double dopir=DMUL(dop,ir); double h3[6]={DMUL(DSUB(vn,DMUL(dopir,pn)),ir),DMUL(DSUB(ve,DMUL(dopir,pe)),ir),DMUL(DSUB(vd,DMUL(dopir,pd)),ir),h0[0],h0[1],h0[2]};
  /* exact equivalence to the batch update (diagonal R): each later innovation is corrected for the state change
     produced by the earlier scalars:  y_k' = y_k - h_k . (x_now - x_prior)  */
  double xp[6]; for(int i=0;i<6;i++) xp[i]=s->x[i];
  const double *hk[4]={h0,h1,h2,h3}; const int nck[4]={3,3,3,6};
  for(int k=0;k<4;k++){
    double yk=y[k];
    if(k>0){ double corr=DMUL(hk[k][0],DSUB(s->x[0],xp[0])); for(int j=1;j<nck[k];j++) corr=DADD(corr,DMUL(hk[k][j],DSUB(s->x[j],xp[j]))); yk=DSUB(yk,corr); }
    tide64_scalar_update(s,hk[k],nck[k],yk,R[k],joseph);
  }
}
#endif
