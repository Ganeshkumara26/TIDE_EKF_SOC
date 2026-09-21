#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include "tide_core.h"
#include "tide_core64.h"
#include "ekf.h"
#include "ekf64.h"
extern int g_simplified,g_fix_alias,g64_simplified,g64_fix_alias;
static uint64_t rs=5; static double ur(void){rs=rs*6364136223846793005ULL+1442695040888963407ULL;return ((rs>>11)&((1ULL<<53)-1))/(double)(1ULL<<53);}
static double nr(void){double a=ur()+1e-12,b=ur();return sqrt(-2*log(a))*cos(6.283185307179586*b);}
int main(void){
  double geos[4][6]={{15,4,10,-3,0,-0.5},{12,2,6,-3,0,-0.5},{1000,500,-200,20,10,-2},{-18,-3,5,2,0.5,-0.3}}; const char*gn[4]={"20 m","13 m","1.1 km","-18 m(behind)"};
  const double QP=0.1,QV=0.01,R[4]={1.0,0.001,0.001,0.5};
  for(int g=0;g<4;g++){
    double wB=0,wC=0,wD=0,wBP=0,wCP=0,wDP=0; int n=0;
    for(int seed=0;seed<25;seed++){
      g_simplified=0;g_fix_alias=1;g64_simplified=0;g64_fix_alias=1;
      ekf_state_t ef; ekf_config_t cf={0.1f,1.0f,0.001f,0.5f}; ekf_init(&ef,&cf);
      ekf64_state_t ed; ekf64_config_t cd={0.1,1.0,0.001,0.5}; ekf64_init(&ed,&cd);
      tide_st tf; memset(&tf,0,sizeof tf); for(int i=0;i<3;i++){tf.P[pk(i,i)]=100.0f;tf.P[pk(3+i,3+i)]=25.0f;}
      tide64_st td; memset(&td,0,sizeof td); for(int i=0;i<3;i++){td.P[pk64(i,i)]=100.0;td.P[pk64(3+i,3+i)]=25.0;}
      double p[3]={geos[g][0],geos[g][1],geos[g][2]},v[3]={geos[g][3],geos[g][4],geos[g][5]};
      for(int i=0;i<3;i++){ double x0=p[i]+(i==0?1.0:0.5); ef.x[i]=tf.x[i]=(float)x0; ed.x[i]=td.x[i]=x0; ef.x[3+i]=tf.x[3+i]=(float)v[i]; ed.x[3+i]=td.x[3+i]=v[i]; }
      for(int k=0;k<100;k++){
        for(int i=0;i<3;i++)p[i]+=v[i]*0.01; double r=sqrt(p[0]*p[0]+p[1]*p[1]+p[2]*p[2]),rg=sqrt(p[0]*p[0]+p[1]*p[1]);
        double zd[4]={r+nr(),atan2(p[1],p[0])+sqrt(0.001)*nr(),atan2(-p[2],rg)+sqrt(0.001)*nr(),(v[0]*p[0]+v[1]*p[1]+v[2]*p[2])/r+sqrt(0.5)*nr()};
        float zf[4]={(float)zd[0],(float)zd[1],(float)zd[2],(float)zd[3]}; float inf[4]; double ind[4]; float Rf[4]={1.0f,0.001f,0.001f,0.5f};
        ekf64_predict(&ed,0.01); int rd=ekf64_update_radar(&ed,zd[0],zd[1],zd[2],zd[3],ind);
        ekf_predict(&ef,0.01f); int rf=ekf_update_radar(&ef,zf[0],zf[1],zf[2],zf[3],inf);
        tide_predict(&tf,0.01f,100.0f,0.1f,0.01f); tide_update(&tf,zf,Rf,1);
        tide64_predict(&td,0.01,100.0,QP,QV); tide64_update(&td,zd,R,1);
        if(rd||rf) continue; n++;
        for(int i=0;i<6;i++){ /* reference = dense double */
          double ref=ed.x[i]; double b=fabs(td.x[i]-ref), c=fabs((double)tf.x[i]-ref), d=fabs((double)ef.x[i]-ref); if(b>wB)wB=b; if(c>wC)wC=c; if(d>wD)wD=d;
          double pr=ed.P[i*7]; double bp=fabs(td.P[pk64(i,i)]-pr)/pr, cp=fabs((double)tf.P[pk(i,i)]-pr)/pr, dp=fabs((double)ef.P[i*7]-pr)/pr; if(bp>wBP)wBP=bp; if(cp>wCP)wCP=cp; if(dp>wDP)wDP=dp; } } }
    printf("%-14s vs dense-DOUBLE reference (%d steps):\n   max |dx|:  tide-double %.2e | tide-float %.2e | dense-float %.2e\n   max rel dP_ii: tide-double %.2e | tide-float %.2e | dense-float %.2e\n",gn[g],n,wB,wC,wD,wBP,wCP,wDP); }
  return 0; }
