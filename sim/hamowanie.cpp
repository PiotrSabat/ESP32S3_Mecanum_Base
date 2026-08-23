// HAMOWANIE: rozpedzenie do 180 RPM, potem puszczenie drazka (zadana = 0).
// Mierzy prad hamowania przeciwpradem ORAZ czy silnik wyrzuca w rewers
// po zatrzymaniu (skutek naladowanej calki).
#include <cstdio>
#include <cmath>
#include <cstring>
#include "Motor.h"

uint32_t g_fakeMillis=0; int g_pwm[16]={0}; int64_t g_encoderCount=0;

static const float V_BAT=7.4f, V_NOM=6.0f, RPM_NL=160.0f, I_STALL=1.5f;
static const float R_W=V_NOM/I_STALL, K_E=V_NOM/RPM_NL;
static const float MAXPWM=511.0f, TAU=150.0f, DB=25.0f, CPR=960.0f;
static const int NM=4;

static float om=0, pos=0, peak=0, peakOptim=0;
static void reset(){ om=0;pos=0;g_encoderCount=0;peak=0;peakOptim=0;memset(g_pwm,0,sizeof(g_pwm)); }

static void plant(uint32_t ms){
    for(uint32_t i=0;i<ms;i++){
        float u=(float)(g_pwm[0]-g_pwm[1]);
        float v=(u/MAXPWM)*V_BAT, emf=K_E*om;
        // Model pesymistyczny (szybkie rozladowanie): pelna roznica napiec.
        float ip=fabsf(v-emf)/R_W*NM;
        // Model optymistyczny (wybieg w fazie off): prad tylko przez wypelnienie.
        float duty=fabsf(u)/MAXPWM;
        float vfull=(u>0?V_BAT:-V_BAT);
        float io=duty*fabsf(vfull-emf)/R_W*NM;
        if(u==0.0f){ ip=0; io=0; }             // PWM=0 -> mostek luzem
        if(ip>peak)peak=ip; if(io>peakOptim)peakOptim=io;
        float eff=fabsf(u)>DB?(u>0?u-DB:u+DB):0.0f;
        float t=(eff/MAXPWM)*RPM_NL*(V_BAT/V_NOM);
        om+=(t-om)*(1.0f/TAU); pos+=om/60000.0f;
        g_encoderCount=(int64_t)llround(pos*CPR); g_fakeMillis++;
    }
}

static const MotorConfig CFG={.pwmPin1=9,.pwmPin2=10,.pwmChannel1=0,.pwmChannel2=1,
 .encoderPinA=1,.encoderPinB=2,.invertDirection=false,.gearRatio=960,
 .pwmResolution=9,.pwmFrequency=20000,.Kp=3.0f,.Ki=0.03f,.Kd=50.0f,
 .outputMin=-511.0f,.outputMax=511.0f,.softStopDurationMs=500,.hardStopDurationMs=50};

int main(){
    printf("\n=== ROZPEDZENIE DO 180 RPM, POTEM PUSZCZENIE DRAZKA (zadana=0) ===\n");
    printf("Nastawy Kp=3.0 Ki=0.03 Kd=50, MAX_HOLDING_PWM=%d (limit malejacy z predkoscia)\n\n",
           MAX_HOLDING_PWM);

    reset(); g_fakeMillis=0;
    Motor m(CFG);

    for(int i=0;i<150;i++){ m.setTargetRPM(180.0f); plant(20); m.update(); }
    printf("Rozpedzone do %.1f RPM, PWM=%d\n", om, m.getControlOutput());
    printf("Prad rozruchu: %.2f A (pesym.) / %.2f A (optym.)\n\n", peak, peakOptim);

    peak=0; peakOptim=0;
    printf("--- puszczenie drazka ---\n");
    printf("  t[ms]  zadana  predkosc   PWM    uwagi\n");
    float minOm=1e9; int worstRev=0;
    for(int i=0;i<100;i++){
        m.setTargetRPM(0.0f);
        plant(20); m.update();
        int pw=m.getControlOutput();
        if(om<minOm) minOm=om;
        if(pw<worstRev) worstRev=pw;
        if(i<20 || i%10==0)
            printf("  %5d  %6.0f  %8.1f  %4d    %s\n",(i+1)*20, 0.0f, om, pw,
                   (om<-3.0f?"<-- REWERS! silnik wyrzuca w tyl":""));
    }
    printf("\nPrad hamowania: %.2f A (pesym.) / %.2f A (optym.)\n", peak, peakOptim);
    printf("Najwiekszy rewers PWM: %d\n", worstRev);
    printf("Najnizsza predkosc: %.1f RPM\n", minOm);

    int failures = 0;
    auto check=[&](bool c,const char* w){ printf("  [%s] %s\n", c?" OK ":"FAIL", w); if(!c)failures++; };

    // Progi z zapasem wzgledem 5.61 A, ktore na sprzecie NIE wywalalo
    // zabezpieczenia przed calym strojeniem PID.
    check(peakOptim < 4.5f, "prad hamowania z zapasem ponizej progu zabezpieczenia");
    check(minOm > -3.0f,    "brak wyrzutu w rewers po zatrzymaniu (calka nie laduje sie)");
    check(fabsf(om) < 3.0f, "platforma faktycznie sie zatrzymala");

    printf("\n%s (bledow: %d)\n", failures?"!!! SA BLEDY":"WSZYSTKO OK", failures);
    return failures ? 1 : 0;
}
