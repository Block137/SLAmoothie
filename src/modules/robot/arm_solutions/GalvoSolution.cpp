#include "GalvoSolution.h"
#include "ActuatorCoordinates.h"

#include <fastmath.h>

#define mirrors_distance_checksum   CHECKSUM("mirrors_distance")
#define galvo_height_checksum       CHECKSUM("galvo_height")
#define dac_resolution_checksum     CHECKSUM("dac_resolution")
#define dac_neutral_checksum        CHECKSUM("dac_neutral")

//#define ROUND(x, y) (roundf(x * (float)(1e ## y)) / (float)(1e ## y))
//#define DEG2RAD      0.01745329251994329576923690768489F
//#define RAD2DEG     57.29577951308232087679815481410517F

GalvoSolution::GalvoSolution(Config* config)
{
    // distance between 2 mirrors (mm)
    mirrors_distance = config->value(mirrors_distance_checksum)->by_default(7.5f)->as_number();
    // distance from print surface (mm)
    galvo_height = config->value(galvo_height_checksum)->by_default(559.8076f)->as_number();
}

void GalvoSolution::cartesian_to_actuator(const float cartesian_mm[], ActuatorCoordinates &actuator_mm ) const
{
    actuator_mm[BETA_STEPPER]  = atanf(cartesian_mm[Y_AXIS] / this->galvo_height);
    actuator_mm[ALPHA_STEPPER] = atanf(cartesian_mm[X_AXIS] / ((this->galvo_height/ cosf(actuator_mm[BETA_STEPPER]) ) + this->mirrors_distance)  );
//    actuator_mm[GAMMA_STEPPER] = cartesian_mm[Z_AXIS];
}

void GalvoSolution::actuator_to_cartesian(const ActuatorCoordinates &actuator_mm, float cartesian_mm[] ) const
{
    cartesian_mm[Y_AXIS] =    this->galvo_height * tanf(actuator_mm[BETA_STEPPER]);
    cartesian_mm[X_AXIS] = ( (this->galvo_height / cosf(actuator_mm[BETA_STEPPER]) ) + this->mirrors_distance) * tanf(actuator_mm[ALPHA_STEPPER]);
//    cartesian_mm[Z_AXIS] = actuator_mm[GAMMA_STEPPER];
}
