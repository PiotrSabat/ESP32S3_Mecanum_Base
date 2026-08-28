// Checks stick -> RPM scaling and the kinematics normalisation: whether the
// DIRECTION of travel survives full deflection on every axis.
#include <cstdio>
#include <cmath>
#include <cstring>
#include "MecanumDrive.h"
#include "pad_input.h"

uint32_t g_fakeMillis=0; int g_pwm[16]={0}; int64_t g_encoderCount=0;

static int failures=0;
static void check(bool c,const char* w){ printf("  [%s] %s\n", c?" OK ":"FAIL", w); if(!c)failures++; }

static MotorConfig mk(int ch1,int ch2){
    MotorConfig c{}; c.pwmPin1=9;c.pwmPin2=10;c.pwmChannel1=ch1;c.pwmChannel2=ch2;
    c.encoderPinA=1;c.encoderPinB=2;c.invertDirection=false;   // no inversion, so the numbers read directly
    c.gearRatio=1920;c.pwmResolution=9;c.pwmFrequency=20000;
    c.Kp=6.0f;c.Ki=0.06f;c.Kd=100.0f;c.outputMin=-511;c.outputMax=511;
    c.softStopDurationMs=500;c.hardStopDurationMs=50; return c;
}

int main(){
    printf("\n=== 1. STICK -> RPM SCALING (MAX_RPM=%d, dead zone %d) ===\n",
           MAX_RPM, JOYSTICK_DEADZONE);
    struct {int stick; const char* opis;} pts[]={
        {0,"at rest"},{5,"noise inside the dead zone"},{12,"edge of the dead zone"},
        {128,"25% deflection"},{256,"50%"},{383,"75%"},{511,"100%"}};
    for(auto&p:pts) printf("   stick %4d (%-26s) -> %7.1f RPM\n", p.stick,p.opis,padAxisToRPM(p.stick));

    check(padAxisToRPM(0)==0.0f,   "at rest gives exactly 0 RPM");
    check(padAxisToRPM(5)==0.0f,   "noise inside the dead zone gives exactly 0 RPM");
    check(fabsf(padAxisToRPM(511)-MAX_RPM)<0.01f, "full deflection gives exactly MAX_RPM");
    check(padAxisToRPM(-511)<0,    "negative deflection gives negative RPM");
    float half=padAxisToRPM(261);     // midpoint of the range past the dead zone
    check(fabsf(half-MAX_RPM/2.0f)<3.0f, "half the throw gives roughly half the speed");

    printf("\n=== 2. NORMALISATION: does the direction survive saturation ===\n");
    Motor fl(mk(0,1)), fr(mk(2,3)), rl(mk(4,5)), rr(mk(6,7));
    MecanumDrive d(&fl,&fr,&rl,&rr);

    // Diagonal travel with rotation, all three axes fully deflected.
    float vx=padAxisToRPM(511), vy=padAxisToRPM(511), om=padAxisToRPM(511);
    printf("   input: vx=%.1f vy=%.1f omega=%.1f RPM\n", vx,vy,om);

    // Theoretical values BEFORE normalisation
    float t[4]={vy+vx+om, vy-vx-om, vy-vx+om, vy+vx-om};
    printf("   without normalisation:  FL=%.1f FR=%.1f RL=%.1f RR=%.1f\n",t[0],t[1],t[2],t[3]);

    d.drive(vx,vy,om);
    float g[4]={fl.getTargetRPM(),fr.getTargetRPM(),rl.getTargetRPM(),rr.getTargetRPM()};
    printf("   after normalisation:     FL=%.1f FR=%.1f RL=%.1f RR=%.1f\n",g[0],g[1],g[2],g[3]);

    float maxG=0; for(int i=0;i<4;i++) maxG=fmaxf(maxG,fabsf(g[i]));
    check(maxG<=MAX_RPM+0.01f, "no wheel exceeds MAX_RPM");

    // The key check: the proportions must be the same as before normalisation.
    bool proportions=true;
    float ref = t[0]/g[0];
    for(int i=0;i<4;i++){
        if(fabsf(g[i])<0.01f && fabsf(t[i])<0.01f) continue;
        if(fabsf(t[i]/g[i]-ref) > 0.01f*fabsf(ref)) proportions=false;
    }
    check(proportions, "all wheels scaled by THE SAME factor (direction stays faithful)");

    printf("\n=== 3. Small deflections are NOT normalised (full resolution) ===\n");
    float sx=padAxisToRPM(60), sy=padAxisToRPM(60);
    d.drive(sx,sy,0.0f);
    printf("   stick 60/60: FL=%.1f FR=%.1f RL=%.1f RR=%.1f (expected FL=%.1f)\n",
           fl.getTargetRPM(),fr.getTargetRPM(),rl.getTargetRPM(),rr.getTargetRPM(), sy+sx);
    check(fabsf(fl.getTargetRPM()-(sy+sx))<0.01f, "small deflections pass through untouched");

    printf("\n=== 4. Pure sideways travel keeps its symmetry ===\n");
    d.drive(padAxisToRPM(511),0.0f,0.0f);
    float b[4]={fl.getTargetRPM(),fr.getTargetRPM(),rl.getTargetRPM(),rr.getTargetRPM()};
    printf("   FL=%.1f FR=%.1f RL=%.1f RR=%.1f\n",b[0],b[1],b[2],b[3]);
    check(fabsf(b[0]-b[3])<0.01f && fabsf(b[1]-b[2])<0.01f,
          "FL=RR and FR=RL - the correct sideways pattern");
    check(fabsf(b[0]+b[1])<0.01f, "left and right sides oppose each other - no net rotation");

    printf("\n=== 5. ARC STEERING: the right stick as tightness, not rotation ===\n");
    {
        const float FULL = (float)MAX_RPM;
        const int   S    = JOYSTICK_MAX;

        // a) At rest the turn stick must give a full pivot, otherwise a
        //    stationary platform could not rotate at all.
        float wStop = padAxisToOmega(S, 0.0f, 0.0f);
        printf("   at rest, full turn            -> omega %6.1f RPM\n", wStop);
        check(fabsf(wStop - FULL) < 0.5f, "at rest the stick gives a full pivot in place");

        // b) Full speed + full turn: the inner wheels must COUNTER-ROTATE.
        //    This is the heart of the fix - they used to sit at zero and the
        //    platform dragged itself round like a tank with one track.
        float wFast = padAxisToOmega(S, 0.0f, FULL);
        // On a turn one side gets FULL + wFast and the other FULL - wFast, so
        // the inner pair is fully described by one number.
        float innerFull = FULL - wFast;
        printf("   full throttle + full turn     -> omega %6.1f, inner %6.1f RPM\n",
               wFast, innerFull);
        check(wFast > FULL,      "at full speed the turn term exceeds MAX_RPM");
        check(innerFull < -1.0f, "the inner wheels counter-rotate instead of stopping");

        // c) A small deflection at full throttle must give a GENTLE arc:
        //    both sides still forward, the outer one faster.
        float wSoft = padAxisToOmega(S/4, 0.0f, FULL);
        float inner = FULL - wSoft, outer = FULL + wSoft;
        printf("   full throttle + quarter turn  -> inner %6.1f, outer %6.1f RPM\n",
               inner, outer);
        check(inner > 0.0f && outer > inner, "a quarter of stick gives a gentle arc, both sides forward");

        // d) Expo curve: half the stick gives LESS than half the rotation.
        float wHalf = padAxisToOmega(S/2, 0.0f, FULL);
        check(wHalf < 0.5f * wFast, "the expo curve concentrates resolution around centre");

        // e) Rotation scales with speed - that is what separates an arc from a pivot.
        float wSlow = padAxisToOmega(S, 0.0f, FULL * 0.5f);
        printf("   half throttle + full turn     -> omega %6.1f RPM\n", wSlow);
        check(wSlow < wFast, "slower travel gives a smaller rotation term (an arc, not a pivot)");

        // f) The wheel ratio AFTER NORMALISATION - this is the number the
        //    operator actually sees on the floor. TURN_INNER_RATIO_FULL must
        //    describe it directly, or the tuning knob is lying.
        {
            Motor fl(mk(0,1)), fr(mk(2,3)), rl(mk(4,5)), rr(mk(6,7));
            MecanumDrive d(&fl,&fr,&rl,&rr);
            d.drive(0.0f, FULL, padAxisToOmega(S, 0.0f, FULL));
            float outer = fl.getTargetRPM(), inner = fr.getTargetRPM();
            float ratio = inner / outer;
            printf("   after normalisation: outer %6.1f  inner %6.1f  -> ratio %+5.2f"
                   " (commanded %+5.2f)\n", outer, inner, ratio, TURN_INNER_RATIO_FULL);
            check(fabsf(ratio - TURN_INNER_RATIO_FULL) < 0.03f,
                  "the inner wheels work in the commanded proportion to the outer ones");
        }

        // g) Sideways travel counts as speed too - with mecanum wheels it is
        //    a first-class motion, so an arc makes sense there as well.
        float wSide = padAxisToOmega(S, FULL, 0.0f);
        check(fabsf(wSide - wFast) < 0.5f, "sideways travel gives the same rotation term as forward travel");
    }

    printf("\n%s (failures: %d)\n", failures?"!!! FAILURES":"ALL OK", failures);
    return failures?1:0;
}
