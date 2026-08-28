// Sprawdza skalowanie drazka -> RPM oraz normalizacje kinematyki:
// czy przy pelnym wychyleniu KIERUNEK jazdy pozostaje wierny.
#include <cstdio>
#include <cmath>
#include <cstring>
#include "MecanumDrive.h"

uint32_t g_fakeMillis=0; int g_pwm[16]={0}; int64_t g_encoderCount=0;

static int failures=0;
static void check(bool c,const char* w){ printf("  [%s] %s\n", c?" OK ":"FAIL", w); if(!c)failures++; }

// Kopia padAxisToRPM z main.cpp (ta sama arytmetyka, te same stale z parameters.h)
static float padAxisToRPM(int16_t raw){
    float v = raw;
    if (v >  (float)JOYSTICK_MAX) v =  (float)JOYSTICK_MAX;
    if (v < -(float)JOYSTICK_MAX) v = -(float)JOYSTICK_MAX;
    if (fabsf(v) < JOYSTICK_DEADZONE) return 0.0f;
    float sign = (v>0.0f)?1.0f:-1.0f;
    float mag = (fabsf(v)-JOYSTICK_DEADZONE)/(float)(JOYSTICK_MAX-JOYSTICK_DEADZONE);
    return sign*mag*(float)MAX_RPM;
}

static MotorConfig mk(int ch1,int ch2){
    MotorConfig c{}; c.pwmPin1=9;c.pwmPin2=10;c.pwmChannel1=ch1;c.pwmChannel2=ch2;
    c.encoderPinA=1;c.encoderPinB=2;c.invertDirection=false;   // bez inwersji, by czytac wprost
    c.gearRatio=1920;c.pwmResolution=9;c.pwmFrequency=20000;
    c.Kp=6.0f;c.Ki=0.06f;c.Kd=100.0f;c.outputMin=-511;c.outputMax=511;
    c.softStopDurationMs=500;c.hardStopDurationMs=50; return c;
}

int main(){
    printf("\n=== 1. SKALOWANIE DRAZKA -> RPM (MAX_RPM=%d, martwa strefa %d) ===\n",
           MAX_RPM, JOYSTICK_DEADZONE);
    struct {int stick; const char* opis;} pts[]={
        {0,"spoczynek"},{5,"szum w martwej strefie"},{12,"krawedz martwej strefy"},
        {128,"25% wychylenia"},{256,"50%"},{383,"75%"},{511,"100%"}};
    for(auto&p:pts) printf("   drazek %4d (%-24s) -> %7.1f RPM\n", p.stick,p.opis,padAxisToRPM(p.stick));

    check(padAxisToRPM(0)==0.0f,   "spoczynek daje dokladnie 0 RPM");
    check(padAxisToRPM(5)==0.0f,   "szum w martwej strefie daje dokladnie 0 RPM");
    check(fabsf(padAxisToRPM(511)-MAX_RPM)<0.01f, "pelne wychylenie daje dokladnie MAX_RPM");
    check(padAxisToRPM(-511)<0,    "ujemne wychylenie daje ujemne RPM");
    float polowa=padAxisToRPM(261);   // srodek zakresu za martwa strefa
    check(fabsf(polowa-MAX_RPM/2.0f)<3.0f, "polowa skoku daje mniej wiecej polowe predkosci");

    printf("\n=== 2. NORMALIZACJA: czy kierunek jazdy przetrwa nasycenie ===\n");
    Motor fl(mk(0,1)), fr(mk(2,3)), rl(mk(4,5)), rr(mk(6,7));
    MecanumDrive d(&fl,&fr,&rl,&rr);

    // Jazda na skos z obrotem, pelne wychylenie wszystkich trzech osi.
    float vx=padAxisToRPM(511), vy=padAxisToRPM(511), om=padAxisToRPM(511);
    printf("   wejscie: vx=%.1f vy=%.1f omega=%.1f RPM\n", vx,vy,om);

    // Wartosci teoretyczne PRZED normalizacja
    float t[4]={vy+vx+om, vy-vx-om, vy-vx+om, vy+vx-om};
    printf("   bez normalizacji byloby: FL=%.1f FR=%.1f RL=%.1f RR=%.1f\n",t[0],t[1],t[2],t[3]);

    d.drive(vx,vy,om);
    float g[4]={fl.getTargetRPM(),fr.getTargetRPM(),rl.getTargetRPM(),rr.getTargetRPM()};
    printf("   po normalizacji:         FL=%.1f FR=%.1f RL=%.1f RR=%.1f\n",g[0],g[1],g[2],g[3]);

    float maxG=0; for(int i=0;i<4;i++) maxG=fmaxf(maxG,fabsf(g[i]));
    check(maxG<=MAX_RPM+0.01f, "zadne kolo nie przekracza MAX_RPM");

    // Kluczowy test: proporcje musza zostac te same co przed normalizacja.
    bool proporcje=true;
    float ref = t[0]/g[0];
    for(int i=0;i<4;i++){
        if(fabsf(g[i])<0.01f && fabsf(t[i])<0.01f) continue;
        if(fabsf(t[i]/g[i]-ref) > 0.01f*fabsf(ref)) proporcje=false;
    }
    check(proporcje, "wszystkie kola przeskalowane TYM SAMYM wspolczynnikiem (kierunek wierny)");

    printf("\n=== 3. Male wychylenia NIE sa normalizowane (pelna rozdzielczosc) ===\n");
    float sx=padAxisToRPM(60), sy=padAxisToRPM(60);
    d.drive(sx,sy,0.0f);
    printf("   drazek 60/60: FL=%.1f FR=%.1f RL=%.1f RR=%.1f (oczekiwane FL=%.1f)\n",
           fl.getTargetRPM(),fr.getTargetRPM(),rl.getTargetRPM(),rr.getTargetRPM(), sy+sx);
    check(fabsf(fl.getTargetRPM()-(sy+sx))<0.01f, "przy malych wychyleniach brak ingerencji");

    printf("\n=== 4. Czysta jazda bokiem zachowuje symetrie ===\n");
    d.drive(padAxisToRPM(511),0.0f,0.0f);
    float b[4]={fl.getTargetRPM(),fr.getTargetRPM(),rl.getTargetRPM(),rr.getTargetRPM()};
    printf("   FL=%.1f FR=%.1f RL=%.1f RR=%.1f\n",b[0],b[1],b[2],b[3]);
    check(fabsf(b[0]-b[3])<0.01f && fabsf(b[1]-b[2])<0.01f,
          "FL=RR oraz FR=RL - poprawny wzorzec jazdy bokiem");
    check(fabsf(b[0]+b[1])<0.01f, "lewa i prawa strona przeciwne - brak obrotu");

    printf("\n%s (bledow: %d)\n", failures?"!!! SA BLEDY":"WSZYSTKO OK", failures);
    return failures?1:0;
}
