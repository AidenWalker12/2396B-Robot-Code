#include "subsystems/pneumatics.hpp"

pros::adi::DigitalOut scraperDigital(1);
pros::adi::Pneumatics scraper('A', false, false);

void initPneumatics() {
    // nothing else needed
}
