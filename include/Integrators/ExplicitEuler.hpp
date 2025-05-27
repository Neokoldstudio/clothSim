#pragma once

/**
 * @file ExplicitEuler.hpp
 *
 * @brief Explicit Euler integration.
 *
 * Nom:
 * Code permanent :
 * Email :
 *
 */

#include "Integrators/Integrator.h"
#include "ParticleSystem.h"

#include <Eigen/Dense>

class ExplicitEuler : public Integrator
{
public:

    // TODO Implement explicit Euler integration that advances @a particleSystem by one time step @a dt.
    //
    virtual void step(ParticleSystem* particleSystem, float dt) override
    {
    }

};
