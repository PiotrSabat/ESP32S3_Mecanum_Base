// Dwa scenariusze naraz:
//  A) recznie kreci sie kolem przy zadanej 0 -> regulator MUSI sie przeciwstawic
//  B) hamowanie z pelnej predkosci -> prad MUSI zostac niski
#include <cstdio>
#include <cmath>
#include <cstring>
#include "Motor.h"

uint32_t g_fakeMillis=0; int g_pwm[16]={0}; int64_t g_encoderCount=0;

static const float V_BAT=7.4f, V_NOM=6.0f, RPM_NL=160.0f, I_STALL=1.5f;
static const float R_W=V_NOM/I_STALL, K_E=V_NOM/RPM_NL, MAXPWM=511.0f;
static const float TAU=150.0f, DB=25.0f, CPR=960.0f;

static int failures=0;
static void check(bool c,const char* w){ printf("  [%s] %s\n", c?" OK ":"FAIL", w); if(!c)failures++; }

static const MotorConfig CFG={.pwmPin1=9,.pwmPin2=10,.pwmChannel1=0,.pwmChannel2=1,
 .encoderPinA=1,.encoderPinB=2,.invertDirection=false,.gearRatio=960,
 .pwmResolution=9,.pwmFrequency=20000,.Kp=3.0f,.Ki=0.03f,.Kd=50.0f,
 .outputMin=-511.0f,.outputMax=511.0f,.softStopDurationMs=500,.hardStopDurationMs=50};

int main(){
    printf("\n=== A. RECZNE KRECENIE KOLEM przy zadanej 0 ===\n");
    printf("   Wymuszamy obrot 30 RPM i patrzymy, czy silnik sie przeciwstawia.\n");
    printf("    t[ms]  wymuszone[RPM]   PWM   moment przeciwny?\n");
    {
        g_fakeMillis=0; g_encoderCount=0; memset(g_pwm,0,sizeof(g_pwm));
        Motor m(CFG);
        float pos=0; const float forced=30.0f;      // ktos kreci kolem do przodu
        int opornych=0, maxOpor=0;
        for(int i=0;i<40;i++){
            pos += forced/60000.0f*20.0f;            // wymuszona pozycja
            g_encoderCount=(int64_t)llround(pos*CPR);
            g_fakeMillis+=20;
            m.setTargetRPM(0.0f);
            m.update();
            int pw=m.getControlOutput();
            if(pw<0){ opornych++; if(pw<maxOpor) maxOpor=pw; }
            if(i%5==0) printf("   %5d  %12.1f  %5d   %s\n",(i+1)*20,forced,pw,
                              pw<0?"TAK - silnik oporuje":"nie");
        }
        printf("   Cykli z momentem przeciwnym: %d/40, najsilniejszy PWM: %d\n",opornych,maxOpor);
        check(opornych>30, "silnik przeciwstawia sie recznemu obrotowi");
        check(maxOpor<-50, "opor jest odczuwalny (PWM ponizej -50)");

        float emf=K_E*forced, v=fabsf((float)maxOpor)/MAXPWM*V_BAT;
        float duty=fabsf((float)maxOpor)/MAXPWM;
        printf("   Prad przy tym oporze: %.2f A na silnik (SEM tylko %.2f V przy %.0f RPM)\n",
               duty*(V_BAT+emf)/R_W, emf, forced);
    }

    printf("\n=== B. HAMOWANIE z pelnej predkosci (prad musi zostac niski) ===\n");
    {
        g_fakeMillis=0; g_encoderCount=0; memset(g_pwm,0,sizeof(g_pwm));
        Motor m(CFG);
        float om=0,pos=0,peak=0;
        auto plant=[&](uint32_t ms){
            for(uint32_t k=0;k<ms;k++){
                float u=(float)(g_pwm[0]-g_pwm[1]);
                float duty=fabsf(u)/MAXPWM, emf=K_E*om;
                float vfull=(u>0?V_BAT:-V_BAT);
                float cur = (u==0.0f)?0.0f : duty*fabsf(vfull-emf)/R_W*4.0f;
                if(cur>peak)peak=cur;
                float eff=fabsf(u)>DB?(u>0?u-DB:u+DB):0.0f;
                float t=(eff/MAXPWM)*RPM_NL*(V_BAT/V_NOM);
                om+=(t-om)*(1.0f/TAU); pos+=om/60000.0f;
                g_encoderCount=(int64_t)llround(pos*CPR); g_fakeMillis++;
            }
        };
        for(int i=0;i<150;i++){ m.setTargetRPM(180.0f); plant(20); m.update(); }
        printf("   Rozpedzone do %.1f RPM\n", om);
        peak=0;
        int worst=0;
        for(int i=0;i<120;i++){ m.setTargetRPM(0.0f); plant(20); m.update();
                                if(m.getControlOutput()<worst) worst=m.getControlOutput(); }
        printf("   Po hamowaniu: %.1f RPM, prad szczytowy %.2f A, najwiekszy rewers PWM %d\n",
               om, peak, worst);
        check(peak < 3.0f, "prad hamowania pozostaje niski (< 3 A)");
        check(fabsf(om) < 3.0f, "platforma faktycznie sie zatrzymala");
    }

    printf("\n%s (bledow: %d)\n", failures?"!!! SA BLEDY":"WSZYSTKO OK", failures);
    return failures?1:0;
}
