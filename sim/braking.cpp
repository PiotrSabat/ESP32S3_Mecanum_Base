// BRAKING: spin up to MAX_RPM, then release the stick (command = 0).
// Measures the plugging current AND whether the motor throws itself into
// reverse after stopping (the symptom of a charged integral).
#include <cstdio>
#include <cmath>
#include <cstring>
#include "Motor.h"

uint32_t g_fakeMillis=0; int g_pwm[16]={0}; int64_t g_encoderCount=0;

static const float V_BAT=7.4f, V_NOM=6.0f, RPM_NL=160.0f, I_STALL=1.5f;
static const float R_W=V_NOM/I_STALL, K_E=V_NOM/RPM_NL;
static const float MAXPWM=511.0f, TAU=150.0f, DB=25.0f, CPR=1920.0f;
static const int NM=4;
// Current drawn by the LOADED drive at steady speed. Without this term the
// model accelerated to the no-load speed (~197 RPM at 7.4 V) and predicted
// twice the speed the platform actually reaches.
// Measured 2026-08-28: 3 m in 10.3 s, i.e. about 95 RPM at the wheel.
// (V_BAT - I_LOAD*R_W)/K_E = (7.4 - 4.0)/0.0375 = 91 RPM at full PWM.
static const float I_LOAD=1.0f;


static float om=0, pos=0, peak=0, peakOptim=0;
static void reset(){ om=0;pos=0;g_encoderCount=0;peak=0;peakOptim=0;memset(g_pwm,0,sizeof(g_pwm)); }

static void plant(uint32_t ms){
    for(uint32_t i=0;i<ms;i++){
        float u=(float)(g_pwm[0]-g_pwm[1]);
        float v=(u/MAXPWM)*V_BAT, emf=K_E*om;
        // Pessimistic model (fast decay): the full voltage difference.
        float ip=fabsf(v-emf)/R_W*NM;
        // Optimistic model (coasting during the off phase): current scaled by duty.
        float duty=fabsf(u)/MAXPWM;
        float vfull=(u>0?V_BAT:-V_BAT);
        float io=duty*fabsf(vfull-emf)/R_W*NM;
        if(u==0.0f){ ip=0; io=0; }             // PWM=0 -> bridge floating
        if(ip>peak)peak=ip; if(io>peakOptim)peakOptim=io;
        float eff=fabsf(u)>DB?(u>0?u-DB:u+DB):0.0f;
        float vApplied=(eff/MAXPWM)*V_BAT;
        float t=(fabsf(vApplied)-I_LOAD*R_W)/K_E;   // the load eats part of the voltage
        if(t<0.0f) t=0.0f;
        if(vApplied<0.0f) t=-t;
        om+=(t-om)*(1.0f/TAU); pos+=om/60000.0f;
        g_encoderCount=(int64_t)llround(pos*CPR); g_fakeMillis++;
    }
}

static const MotorConfig CFG={.pwmPin1=9,.pwmPin2=10,.pwmChannel1=0,.pwmChannel2=1,
 .encoderPinA=1,.encoderPinB=2,.invertDirection=false,.gearRatio=1920,
 .pwmResolution=9,.pwmFrequency=20000,.Kp=6.0f,.Ki=0.06f,.Kd=100.0f,
 .outputMin=-511.0f,.outputMax=511.0f,.softStopDurationMs=500,.hardStopDurationMs=50};

int main(){
    printf("\n=== SPIN UP TO MAX_RPM, THEN RELEASE THE STICK (command=0) ===\n");
    printf("Gains Kp=6.0 Ki=0.06 Kd=100, MAX_HOLDING_PWM=%d (limit falls with speed)\n\n",
           MAX_HOLDING_PWM);

    reset(); g_fakeMillis=0;
    Motor m(CFG);

    for(int i=0;i<150;i++){ m.setTargetRPM((float)MAX_RPM); plant(20); m.update(); }
    printf("Spun up to %.1f RPM, PWM=%d\n", om, m.getControlOutput());
    printf("Startup current: %.2f A (pessimistic) / %.2f A (optimistic)\n\n", peak, peakOptim);

    peak=0; peakOptim=0;
    printf("--- stick released ---\n");
    printf("  t[ms]  command   speed    PWM    notes\n");
    float minOm=1e9; int worstRev=0;
    for(int i=0;i<100;i++){
        m.setTargetRPM(0.0f);
        plant(20); m.update();
        int pw=m.getControlOutput();
        if(om<minOm) minOm=om;
        if(pw<worstRev) worstRev=pw;
        if(i<20 || i%10==0)
            printf("  %5d  %6.0f  %8.1f  %4d    %s\n",(i+1)*20, 0.0f, om, pw,
                   (om<-3.0f?"<-- REVERSE! the motor is being thrown backwards":""));
    }
    printf("\nBraking current: %.2f A (pessimistic) / %.2f A (optimistic)\n", peak, peakOptim);
    printf("Largest reverse PWM: %d\n", worstRev);
    printf("Lowest speed: %.1f RPM\n", minOm);

    int failures = 0;
    auto check=[&](bool c,const char* w){ printf("  [%s] %s\n", c?" OK ":"FAIL", w); if(!c)failures++; };

    // Thresholds with margin against the 5.61 A that did NOT trip the
    // protection on hardware, measured before the PID tuning session.
    check(peakOptim < 4.5f, "braking current stays comfortably below the protection threshold");
    check(minOm > -3.0f,    "no reverse kick after stopping (the integral does not charge up)");
    check(fabsf(om) < 3.0f, "the platform genuinely stopped");

    printf("\n%s (failures: %d)\n", failures?"!!! FAILURES":"ALL OK", failures);
    return failures ? 1 : 0;
}
