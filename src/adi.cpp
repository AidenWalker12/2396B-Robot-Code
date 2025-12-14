#include "main.h"

pros::adi::Pneumatics scraper('D', false, false);
pros::adi::Pneumatics hood('C' , false, false);
pros::adi::AnalogIn autosensor('A');
void Autosensor(){
         float angle = (float)autosensor.get_value() * (270.0 / 4095.0); 

}
void Penumatics(){
        if (MasterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
        scraper.toggle();
        }
        if (MasterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        hood.toggle();
        }

}
