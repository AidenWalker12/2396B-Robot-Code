#include "main.h"

pros::adi::Pneumatics scraper('D', false, false);
pros::adi::Pneumatics hood('C' , false, false);

void Penumatics(){
        if (MasterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
        scraper.toggle();
        }
        if (MasterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        hood.toggle();
        }

}
