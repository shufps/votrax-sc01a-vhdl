#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <initializer_list>
#include <string>
#include <vector>

static double m_sclock = 0;
static double m_cclock = 0;
static void set_clocks(double mc) { m_sclock = mc/18.0; m_cclock = mc/36.0; }
static const double FIXED_SCALE = 32768.0;

static double bits_to_caps(uint32_t v, std::initializer_list<double> caps) {
    double t=0; for(double d:caps){if(v&1)t+=d;v>>=1;} return t;
}

static void build_standard(double*a,double*b,
    double c1t,double c1b,double c2t,double c2b,double c3,double c4,bool pw){
    double k0=c1t/(m_cclock*c1b);
    double k1=c4*c2t/(m_cclock*c1b*c3);
    double k2=c4*c2b/(m_cclock*m_cclock*c1b*c3);
    double fp=sqrt(fabs(k0*k1-k2))/(2*M_PI*k2);
    double zc=pw ? 2*M_PI*fp/tan(M_PI*fp/m_sclock) : 2*m_sclock;
    double m0=zc*k0,m1=zc*k1,m2=zc*zc*k2,b0=1+m1+m2;
    a[0]=(1+m0)/b0;a[1]=(3+m0)/b0;a[2]=(3-m0)/b0;a[3]=(1-m0)/b0;
    b[0]=1;b[1]=(3+m1-m2)/b0;b[2]=(3-m1-m2)/b0;b[3]=(1-m1+m2)/b0;
}
static double fpeak_std(double c1t,double c1b,double c2t,double c2b,double c3,double c4){
    double k0=c1t/(m_cclock*c1b),k1=c4*c2t/(m_cclock*c1b*c3),k2=c4*c2b/(m_cclock*m_cclock*c1b*c3);
    return sqrt(fabs(k0*k1-k2))/(2*M_PI*k2);
}

static void build_lowpass(double*a,double*b,double c1t,double c1b,bool pw){
    double k=c1b/(m_cclock*c1t)*(150.0/4000.0);
    double fp=1/(2*M_PI*k);
    double zc=pw ? 2*M_PI*fp/tan(M_PI*fp/m_sclock) : 2*m_sclock;
    double m=zc*k,b0=1+m;
    a[0]=1/b0; b[0]=1; b[1]=(1-m)/b0;
}
static double fpeak_lp(double c1t,double c1b){
    return 1/(2*M_PI*(c1b/(m_cclock*c1t)*(150.0/4000.0)));
}

static void build_noise(double*a,double*b,double c1,double c2t,double c2b,double c3,double c4,bool pw){
    double k0=c2t*c3*c2b/c4,k1=c2t*(m_cclock*c2b),k2=c1*c2t*c3/(m_cclock*c4);
    double fp=sqrt(1/k2)/(2*M_PI);
    double zc=pw ? 2*M_PI*fp/tan(M_PI*fp/m_sclock) : 2*m_sclock;
    double m0=zc*k0,m1=zc*k1,m2=zc*zc*k2,b0=1+m1+m2;
    a[0]=m0/b0;a[1]=0;a[2]=-m0/b0;
    b[0]=1;b[1]=(2-2*m2)/b0;b[2]=(1-m1+m2)/b0;
}
static double fpeak_noise(double c1,double c2t,double /*c2b*/,double /*c3*/,double c4){
    double k2=c1*c2t*1/(m_cclock*c4); // approximate, c2b/c3 cancel in ratio
    (void)k2;
    // recalc properly
    double k2r=c1*c2t*9523/(m_cclock*c4); // use actual c3
    return sqrt(1/k2r)/(2*M_PI);
}

static double maxdiff(double*w,double*s,int n){
    double mx=0; for(int i=0;i<n;i++){double d=fabs(w[i]-s[i])*FIXED_SCALE;if(d>mx)mx=d;} return mx;
}

int main(){
    printf("Votrax SC-01A prewarping error sweep  (sclock=main/18, cclock=main/36)\n");
    printf("Values = max LSB difference (2.15 fixed) between warped and no-warp coefficients\n\n");

    printf("%-10s %-8s %-8s | %-8s %-8s %-8s %-8s %-8s %-8s %-8s %-8s %-8s\n",
        "main(Hz)","sc(Hz)","Nq(Hz)","F1(0)","F1(all)","F2v(0)","F2v(all)","F3(0)","F3(all)","F4","Fx","Fn");
    printf("%-130s\n", "----------------------------------------------------------------------------------------------------------------------");

    for(double mc=450000; mc<=1100001; mc+=50000){
        set_clocks(mc);
        double ny=m_sclock/2;
        double aw[4],bw[4],as[4],bs[4];

        build_standard(aw,bw,11247,11797,949,52067,2280,166272,true);
        build_standard(as,bs,11247,11797,949,52067,2280,166272,false);
        double f1_0=maxdiff(aw,as,4)+maxdiff(bw,bs,4);

        double f1c3=2280+bits_to_caps(0xF,{2546,4973,9861,19724});
        build_standard(aw,bw,11247,11797,949,52067,f1c3,166272,true);
        build_standard(as,bs,11247,11797,949,52067,f1c3,166272,false);
        double f1_a=maxdiff(aw,as,4)+maxdiff(bw,bs,4);

        build_standard(aw,bw,24840,29154,829,38180,2352,34270,true);
        build_standard(as,bs,24840,29154,829,38180,2352,34270,false);
        double f2_0=maxdiff(aw,as,4)+maxdiff(bw,bs,4);

        double f2c2t=829+bits_to_caps(0xF,{1390,2965,5875,11297});
        double f2c3=2352+bits_to_caps(0x1F,{833,1663,3164,6327,12654});
        build_standard(aw,bw,24840,29154,f2c2t,38180,f2c3,34270,true);
        build_standard(as,bs,24840,29154,f2c2t,38180,f2c3,34270,false);
        double f2_a=maxdiff(aw,as,4)+maxdiff(bw,bs,4);

        build_standard(aw,bw,0,17594,868,18828,8480,50019,true);
        build_standard(as,bs,0,17594,868,18828,8480,50019,false);
        double f3_0=maxdiff(aw,as,4)+maxdiff(bw,bs,4);

        double f3c3=8480+bits_to_caps(0xF,{2226,4485,9056,18111});
        build_standard(aw,bw,0,17594,868,18828,f3c3,50019,true);
        build_standard(as,bs,0,17594,868,18828,f3c3,50019,false);
        double f3_a=maxdiff(aw,as,4)+maxdiff(bw,bs,4);

        build_standard(aw,bw,0,28810,1165,21457,8558,7289,true);
        build_standard(as,bs,0,28810,1165,21457,8558,7289,false);
        double f4=maxdiff(aw,as,4)+maxdiff(bw,bs,4);

        double aw2[1],bw2[2],as2[1],bs2[2];
        build_lowpass(aw2,bw2,1122,23131,true);
        build_lowpass(as2,bs2,1122,23131,false);
        double fx=maxdiff(aw2,as2,1)+maxdiff(bw2,bs2,2);

        double aw3[3],bw3[3],as3[3],bs3[3];
        build_noise(aw3,bw3,15500,14854,8450,9523,14083,true);
        build_noise(as3,bs3,15500,14854,8450,9523,14083,false);
        double fn=maxdiff(aw3,as3,3)+maxdiff(bw3,bs3,3);

        printf("%-10.0f %-8.0f %-8.0f | %-8.2f %-8.2f %-8.2f %-8.2f %-8.2f %-8.2f %-8.2f %-8.2f %-8.2f\n",
            mc,m_sclock,ny, f1_0,f1_a,f2_0,f2_a,f3_0,f3_a,f4,fx,fn);
    }

    printf("\n--- fpeak as %% of Nyquist (worst filters) ---\n");
    for(double mc:{450000.0,720000.0,1100000.0}){
        set_clocks(mc);
        double ny=m_sclock/2;
        double fp4=fpeak_std(0,28810,1165,21457,8558,7289);
        double fpx=fpeak_lp(1122,23131);
        printf("main=%.0fHz  ny=%.0fHz  |  F4: %.0fHz=%.1f%%  Fx: %.0fHz=%.1f%%\n",
            mc,ny, fp4,fp4/ny*100, fpx,fpx/ny*100);
    }
    return 0;
}
