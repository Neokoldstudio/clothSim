#pragma once

/**
 * @file Midpoint.hpp
 *
 * @brief Midpoint method.
 *
 * Nom:
 * Code permanent :
 * Email :
 *
 */

#include "Integrators/Integrator.h"
#include "ParticleSystem.h"

class Midpoint : public Integrator
{
public:

    // TODO Implement midpoint integration that advances @a particleSystem by one time step @a dt.
    //
    virtual void step(ParticleSystem* particleSystem, float dt) override
    {
    }

};
