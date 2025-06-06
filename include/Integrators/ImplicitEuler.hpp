#pragma once

/**
 * @file ImplicitEuler.hpp
 *
 * @brief Implicit Euler integration.
 *
 * Nom:
 * Code permanent :
 * Email :
 *
 */

#include "Integrators/Integrator.h"
#include "Solvers/MatrixFreePGS.h"
#include "ParticleSystem.h"

#include <Eigen/Dense>

class ImplicitEuler : public Integrator
{
public:

    // TODO Implement implicit Euler integration that advances @a particleSystem by one time step @a dt.
    //
    //  Solve the linear system :
    //             (M - dt*dfdv - dt*dt*dfdx) deltav = h*f + dt*dt*dfdx*v0
    //  using the matrix-free projected Gauss Seidel method.
    //
    //  Use deltav to compute updated velocities  v = v + deltav,
    //  and then update positions  x = x + dt*v
    //
    //  A single iteration of the algorithm is sufficient.
    //
    virtual void step(ParticleSystem* particleSystem, float dt) override{
        std::vector<Eigen::Vector3f> deltav(particleSystem->getParticles().size(), Eigen::Vector3f::Zero());
        MatrixFreePGS solver(particleSystem);
        solver.solve(dt, deltav);

        for(Particle* p : particleSystem->getParticles())
        {
            p->v += deltav[p->index]; // Update velocities
            p->x += dt * p->v;        // Update positions
        }
    }
};