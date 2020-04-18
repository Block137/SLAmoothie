#pragma once

#include "libs/Kernel.h"
#include "libs/Module.h"
#include "libs/Config.h"
#include "libs/nuts_bolts.h"
#include "BaseSolution.h"
#include "checksumm.h"
#include "ConfigValue.h"

class Config;

class GalvoSolution : public BaseSolution {
    public:
        GalvoSolution(Config*);
        void cartesian_to_actuator(const float[], ActuatorCoordinates &) const override;
        void actuator_to_cartesian(const ActuatorCoordinates &, float[] ) const override;
    private:
        float mirrors_distance; // distance between 2 mirrors
        float galvo_height; // distance from print surface
};
