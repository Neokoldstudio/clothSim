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

#include "Integrator.h"
#include "ParticleSystem.h"

#include <Eigen/Dense>

class SemiImplicitEuler : public Integrator
{
public:

    // TODO Implement semi-explicit Euler integration that advances @a particleSystem by one time step @a dt.
    //   Hint: handle velocity and position  updates separately.
    //
    virtual void step(ParticleSystem* particleSystem, float dt) override
    {
    }

};
