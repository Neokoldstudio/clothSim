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
        Eigen::VectorXf dqdt;
        particleSystem->derivs(dqdt);
        Eigen::VectorXf q;
        particleSystem->getState(q);

        Eigen::VectorXf qMidPoint = q + 0.5f * dt * dqdt;
        Eigen::VectorXf dqdtMidPoint;//vector storing the derivatives at mid point

        //to compute the derivative at mid point, we can advance the system to the midpoint, and then call the derivs() fnc to get the correct derivatives at the midpoint
        particleSystem->setState(qMidPoint);
        particleSystem->derivs(dqdtMidPoint);

        q += dt*dqdtMidPoint;

        particleSystem->setState(q);
    }

};
