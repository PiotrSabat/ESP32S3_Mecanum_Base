// Two scenarios in one:
//  A) a wheel turned by hand with a command of 0 -> the controller MUST resist
//  B) braking from full speed -> the current MUST stay low
#include <cstdio>
#include <cmath>
#include <cstring>
#include "Motor.h"

uint32_t g_fakeMillis=0; int g_pwm[16]={0}; int64_t g_encoderCount=0;

static const float V_BAT=7.4f, V_NOM=6.0f, RPM_NL=160.0f, I_STALL=1.5f;
static const float R_W=V_NOM/I_STALL, K_E=V_NOM/RPM_NL, MAXPWM=511.0f;
// Current drawn by the LOADED drive at steady speed. Without this term the
// model accelerated to the no-load speed (~197 RPM at 7.4 V) and predicted
// twice the speed the platform actually reaches.
// Measured 2026-08-28: 3 m in 10.3 s, i.e. about 95 RPM at the wheel.
// (V_BAT - I_LOAD*R_W)/K_E = (7.4 - 4.0)/0.0375 = 91 RPM at full PWM.
static const float I_LOAD=1.0f;

static const float TAU=150.0f, DB=25.0f, CPR=1920.0f;

static int failures=0;
static void check(bool c,const char* w){ printf("  [%s] %s\n", c?" OK ":"FAIL", w); if(!c)failures++; }

static const MotorConfig CFG={.pwmPin1=9,.pwmPin2=10,.pwmChannel1=0,.pwmChannel2=1,
 .encoderPinA=1,.encoderPinB=2,.invertDirection=false,.gearRatio=1920,
 .pwmResolution=9,.pwmFrequency=20000,.Kp=6.0f,.Ki=0.06f,.Kd=100.0f,
 .outputMin=-511.0f,.outputMax=511.0f,.softStopDurationMs=500,.hardStopDurationMs=50};

int main(){
    printf("\n=== A. TURNING A WHEEL BY HAND with a command of 0 ===\n");
    printf("   Forcing 30 RPM and watching whether the motor pushes back.\n");
    printf("    t[ms]   forced[RPM]     PWM   counter-torque?\n");
    {
        g_fakeMillis=0; g_encoderCount=0; memset(g_pwm,0,sizeof(g_pwm));
        Motor m(CFG);
        float pos=0; const float forced=30.0f;      // someone turns the wheel forward
        int resisting=0, maxResist=0;
        for(int i=0;i<40;i++){
            pos += forced/60000.0f*20.0f;            // forced position
            g_encoderCount=(int64_t)llround(pos*CPR);
            g_fakeMillis+=20;
            m.setTargetRPM(0.0f);
            m.update();
            int pw=m.getControlOutput();
            if(pw<0){ resisting++; if(pw<maxResist) maxResist=pw; }
            if(i%5==0) printf("   %5d  %12.1f  %5d   %s\n",(i+1)*20,forced,pw,
                              pw<0?"YES - the motor resists":"no");
        }
        printf("   Cycles with counter-torque: %d/40, strongest PWM: %d\n",resisting,maxResist);
        check(resisting>30, "the motor resists being turned by hand");
        check(maxResist<-50, "the resistance is noticeable (PWM below -50)");

        float emf=K_E*forced;
        float duty=fabsf((float)maxResist)/MAXPWM;
        printf("   Current at that resistance: %.2f A per motor (EMF only %.2f V at %.0f RPM)\n",
               duty*(V_BAT+emf)/R_W, emf, forced);
    }

    printf("\n=== B. BRAKING from full speed (the current must stay low) ===\n");
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
                float vApplied=(eff/MAXPWM)*V_BAT;
                float t=(fabsf(vApplied)-I_LOAD*R_W)/K_E;  // the load eats part of the voltage
                if(t<0.0f) t=0.0f;
                if(vApplied<0.0f) t=-t;
                om+=(t-om)*(1.0f/TAU); pos+=om/60000.0f;
                g_encoderCount=(int64_t)llround(pos*CPR); g_fakeMillis++;
            }
        };
        for(int i=0;i<150;i++){ m.setTargetRPM((float)MAX_RPM); plant(20); m.update(); }
        printf("   Spun up to %.1f RPM\n", om);
        peak=0;
        int worst=0;
        for(int i=0;i<120;i++){ m.setTargetRPM(0.0f); plant(20); m.update();
                                if(m.getControlOutput()<worst) worst=m.getControlOutput(); }
        printf("   After braking: %.1f RPM, peak current %.2f A, largest reverse PWM %d\n",
               om, peak, worst);
        // Threshold grounded in hardware measurements: 5.61 A at startup
        // passed without a cutoff, 7.40 A tripped it. The previous 3 A came
        // from a model WITHOUT load, which understated the current almost
        // fourfold.
        check(peak < 5.5f, "braking current stays comfortably below the protection threshold");
        check(fabsf(om) < 3.0f, "the platform genuinely stopped");
    }

    printf("\n%s (failures: %d)\n", failures?"!!! FAILURES":"ALL OK", failures);
    return failures?1:0;
}
