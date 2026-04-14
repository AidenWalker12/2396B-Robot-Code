#include "main.h"

extern lemlib::Chassis chassis;
int autoreset = 0;
float YSkills, XSkills;
extern Imu imu;
extern adi::Pneumatics hood, scraper, des, fhood;
extern int config, GLPConfig, FIRSTSTAGE_SPEED, SECONDSTAGE_SPEED;
int xs = 0, ys = 0;
void AutoReset(){
        if(autoreset == 1) {
        XSkills = xs;
        YSkills = ys;      
        autoreset = 0;  
        }
    }

void AutoScore(bool resetpos, bool resetimu, int x, int y) {
    if (resetimu == true) {
        imu.tare();
    }
    if (resetpos == true) {
        xs = x;
        ys = y;
        autoreset = 1;
    }
    hood.set_value(true);
    config = UP;
    FIRSTSTAGE_SPEED = 480;
    SECONDSTAGE_SPEED = 480;
    delay(250);
    FIRSTSTAGE_SPEED = 600;
    SECONDSTAGE_SPEED = 600;
    delay(1250);
    config = DOWN;
    delay(250);
    config = UP;
    delay(2000);
}
