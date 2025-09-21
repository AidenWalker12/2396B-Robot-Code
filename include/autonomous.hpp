#pragma once

void autonblueright();
void autonblueleft();
void autonredright();
void autonredleft();
void autonskills();

// Autonomous Selector
extern int autonmode;
enum autonmode { BLUERIGHT = 0, BLUELEFT = 1, REDRIGHT = 2, REDLEFT = 3, SKILLS = 4 };
