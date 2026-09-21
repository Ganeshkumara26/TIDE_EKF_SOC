#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#define COUNT_OPS
#include "tide_core.h"
#include "ekf.h"
long c_mul,c_add,c_div,c_sqrt;
extern int g_simplified,g_fix_alias;
static uint64_t rs=99; static double ur(void){rs=rs*6364136223846793005ULL+1442695040888963407ULL;return ((rs>>11)&((1ULL<<53)-1))/(double)(1ULL<<53);}
static double nr(void){double a=ur()+1e-12,b=ur();return sqrt(-2*log(a))*cos(6.283185307179586*b);}
static void reset_counts(void){c_mul=c_add=c_div=c_sqrt=0;}
static void show(const char*n){printf("  %-34s mul=%4ld add/sub=%4ld div=%2ld sqrt=%ld  total_flops=%ld\n",n,c_mul,c_add,c_div,c_sqrt,c_mul+c_add);}
/* ---------- truth / measurements ---------- */
typedef struct{double p[3],v[3];}truth_t;
static void meas(const truth_t*t,double sr,double sa,double sd,float*z){ double r=sqrt(t->p[0]*t->p[0]+t->p[1]*t->p[1]+t->p[2]*t->p[2]),rg=sqrt(t->p[0]*t->p[0]+t->p[1]*t->p[1]);
  z[0]=(float)(r+sr*nr()); z[1]=(float)(atan2(t->p[1],t->p[0])+sa*nr()); z[2]=(float)(atan2(-t->p[2],rg)+sa*nr()); z[3]=(float)((t->v[0]*t->p[0]+t->v[1]*t->p[1]+t->v[2]*t->p[2])/r+sd*nr()); }
static void init_st(tide_st*s){ memset(s,0,sizeof *s); for(int i=0;i<3;i++){s->P[pk(i,i)]=100.0f;s->P[pk(3+i,3+i)]=25.0f;} }
int main(void){
  const float QP=0.1f,QV=0.01f,DT0INV=100.0f; const float R[4]={1.0f,0.001f,0.001f,0.5f};
  /* 1. atan2 accuracy */
  double maxe=0; for(int k=0;k<2000000;k++){ double y=(ur()-0.5)*2*pow(10,(ur()-0.5)*8), x=(ur()-0.5)*2*pow(10,(ur()-0.5)*8); double e=fabs((double)tide_atan2((float)y,(float)x)-atan2((double)(float)y,(double)(float)x)); if(e>maxe)maxe=e; }
  printf("[1] polynomial atan2 (octant + minimax, float32): max abs error vs double atan2 over 2e6 points = %.3e rad\n",maxe);
  /* 2. op counts */
  tide_st s; init_st(&s); for(int i=0;i<3;i++){s.x[i]=(float)(15+i);s.x[3+i]=(float)(-3+i);} float z[4]={18.0f,0.25f,0.5f,-2.9f};
  printf("\n[2] operation counts per call (float32; 'total_flops' = mul + add/sub):\n");
  reset_counts(); tide_predict(&s,0.01f,DT0INV,QP,QV); show("predict (block, 21-element P)");
  reset_counts(); tide_update(&s,z,R,1); show("update: 4 sequential scalars, Joseph");
  reset_counts(); init_st(&s); for(int i=0;i<3;i++){s.x[i]=(float)(15+i);s.x[3+i]=(float)(-3+i);} tide_update(&s,z,R,0); show("update: 4 sequential scalars, simplified");
  reset_counts(); (void)tide_atan2(0.3f,0.9f); show("one atan2 (polynomial)");
  /* 3. regression against the dense (fixed) EKF */
  printf("\n[3] regression vs dense EKF (aliasing fixed, batch 4x4 inverse, Joseph), 100 steps @100 Hz, 4 scenarios x 25 seeds:\n");
  double geos[4][6]={{15,4,10,-3,0,-0.5},{12,2,6,-3,0,-0.5},{1000,500,-200,20,10,-2},{-18,-3,5,2,0.5,-0.3}};
  double worst_x=0,worst_P=0; int rej=0; long steps=0;
  for(int g=0;g<4;g++)for(int seed=0;seed<25;seed++){
    g_simplified=0; g_fix_alias=1; ekf_state_t e; ekf_config_t c={0.1f,1.0f,0.001f,0.5f}; ekf_init(&e,&c); tide_st t; init_st(&t);
    truth_t tr={{geos[g][0],geos[g][1],geos[g][2]},{geos[g][3],geos[g][4],geos[g][5]}};
    for(int i=0;i<3;i++){ float v=(float)(tr.p[i]+(i==0?1.0:0.5)); e.x[i]=t.x[i]=v; e.x[3+i]=t.x[3+i]=(float)tr.v[i]; }
    for(int k=0;k<100;k++){ for(int i=0;i<3;i++)tr.p[i]+=tr.v[i]*0.01; float zz[4]; meas(&tr,1.0,sqrt(0.001),sqrt(0.5),zz);
      ekf_predict(&e,0.01f); float in[4]; int ret=ekf_update_radar(&e,zz[0],zz[1],zz[2],zz[3],in); if(ret!=0){rej++;continue;}
      tide_predict(&t,0.01f,DT0INV,QP,QV); tide_update(&t,zz,R,1); steps++;
      for(int i=0;i<6;i++){ double d=fabs((double)e.x[i]-t.x[i]); if(d>worst_x)worst_x=d; }
      for(int i=0;i<6;i++){ double a=e.P[i*7],b=t.P[pk(i,i)]; double rel=fabs(a-b)/fmax(fabs(a),1e-12); if(rel>worst_P)worst_P=rel; } } }
  printf("    steps compared=%ld  dense-gate rejects=%d  max |dx| = %.3e (m or m/s)  max relative |dP_ii| = %.3e\n",steps,rej,worst_x,worst_P);
  /* 4. OOSM checkpoint/journal/replay vs chronological oracle (bitwise) */
  printf("\n[4] out-of-sequence handling: 8-checkpoint ring (every 4 events), journal, shadow replay + atomic commit vs chronological oracle\n");
  #define NCK 8
  #define MCK 4
  typedef struct{uint32_t t;float z[4];int id;}ev_t; typedef struct{uint32_t t;tide_st s;}ck_t;
  int runs=0,bitexact=0,dropped_total=0,nodrop=0; long late=0,replay_ev=0; int maxreplay=0,maxj=0; long flops_replay=0,flops_normal=0; long hist[40]={0};
  for(int L=1;L<=32;L*=2){ for(int rr=0;rr<40;rr++){ runs++;
    int N=200; ev_t ev[200]; truth_t tr={{15,4,10},{-3,0,-0.5}};
    for(int i=0;i<N;i++){ for(int k=0;k<3;k++)tr.p[k]+=tr.v[k]*0.01; ev[i].t=(uint32_t)((i+1)*10000); ev[i].id=i; meas(&tr,1.0,sqrt(0.001),sqrt(0.5),ev[i].z); }
    double key[200]; int ord[200]; for(int i=0;i<N;i++){ key[i]=ev[i].t+(ur()<0.35?floor(ur()*(L+1))*10000.0:0.0)+ i*1e-3; ord[i]=i; }
    for(int a=0;a<N;a++)for(int b=a+1;b<N;b++) if(key[ord[b]]<key[ord[a]]){int t=ord[a];ord[a]=ord[b];ord[b]=t;}
    /* oracle */
    tide_st o; init_st(&o); for(int i=0;i<3;i++){o.x[i]=(float)(15+1.0*(i==0)+0.5*(i==1)+0.0);}
    tide_st init=o; init.x[3]=-3;init.x[4]=0;init.x[5]=-0.5f; o=init; uint32_t to=0;
    reset_counts(); for(int i=0;i<N;i++){ tide_predict(&o,(float)((int32_t)(ev[i].t-to))*1e-6f,DT0INV,QP,QV); tide_update(&o,ev[i].z,R,1); to=ev[i].t; } flops_normal+=c_mul+c_add;
    /* engine */
    tide_st cur=init; uint32_t tnow=0; ck_t ck[NCK]; int nck=1; ck[0].t=0; ck[0].s=init; ev_t jr[64]; int nj=0; int since=0; int dropped=0;
    reset_counts(); long f0=0;
    for(int q=0;q<N;q++){ ev_t*e=&ev[ord[q]];
      if(e->t>=tnow){ tide_predict(&cur,(float)((int32_t)(e->t-tnow))*1e-6f,DT0INV,QP,QV); tide_update(&cur,e->z,R,1); tnow=e->t; jr[nj++]=*e; since++;
        if(since>=MCK){ if(nck==NCK){ memmove(&ck[0],&ck[1],sizeof(ck_t)*(NCK-1)); nck--; } ck[nck].t=tnow; ck[nck].s=cur; nck++; since=0;
          int keep=0; for(int j=0;j<nj;j++) if(jr[j].t>ck[0].t) jr[keep++]=jr[j]; nj=keep; } }
      else { late++; if(e->t<=ck[0].t){ dropped++; continue; }
        int ci=0; for(int j=0;j<nck;j++) if(ck[j].t<e->t) ci=j; 
        int pos=nj; while(pos>0 && jr[pos-1].t>e->t){ jr[pos]=jr[pos-1]; pos--; } jr[pos]=*e; nj++;
        tide_st sh=ck[ci].s; uint32_t tc=ck[ci].t; nck=ci+1; int cnt=0; since=0; long fa=c_mul+c_add;
        for(int j=0;j<nj;j++){ if(jr[j].t<=ck[ci].t) continue; tide_predict(&sh,(float)((int32_t)(jr[j].t-tc))*1e-6f,DT0INV,QP,QV); tide_update(&sh,jr[j].z,R,1); tc=jr[j].t; cnt++; since++;
          if(since>=MCK){ if(nck==NCK){ memmove(&ck[0],&ck[1],sizeof(ck_t)*(NCK-1)); nck--; } ck[nck].t=tc; ck[nck].s=sh; nck++; since=0; } }
        cur=sh; tnow=tc; replay_ev+=cnt; if(cnt>maxreplay)maxreplay=cnt; if(cnt<40)hist[cnt]++; flops_replay+=(c_mul+c_add)-fa;
        { int keep=0; for(int j=0;j<nj;j++) if(jr[j].t>ck[0].t) jr[keep++]=jr[j]; nj=keep; } }
      if(nj>maxj)maxj=nj; }
    dropped_total+=dropped; if(dropped==0) nodrop++; if(dropped==0 && memcmp(&cur,&o,sizeof cur)==0) bitexact++; else if(dropped==0){ static int shown=0; if(shown++<2) printf("    MISMATCH L=%d run=%d: dx=%g\n",L,rr,fabs(cur.x[0]-o.x[0])); }
    (void)f0; } }
  printf("    runs=%d (max reorder lag L in {1,2,4,8,16,32} events); runs with no dropped event=%d; bit-exact vs oracle in those: %d/%d; dropped events total=%d\n",runs,nodrop,bitexact,nodrop,dropped_total);
  printf("    late events=%ld  mean replayed events per late event=%.2f  max replayed=%d  max journal occupancy=%d (capacity assumed 32)\n",late,late?(double)replay_ev/late:0,maxreplay,maxj);
  printf("    replayed-event histogram (count: occurrences):"); for(int i=0;i<40;i++) if(hist[i]) printf(" %d:%ld",i,hist[i]); printf("\n");
  return 0; }
