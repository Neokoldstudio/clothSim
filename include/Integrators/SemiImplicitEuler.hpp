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
        Eigen::VectorXf dqdt;
        particleSystem->derivs(dqdt);
        Eigen::VectorXf q;
        particleSystem->getState(q);

        int nbParticules = q.size() / 6;

        //This time, we want to separate the physical quantities into sperarate vectors
        Eigen::VectorXf positions = Eigen::VectorXf(nbParticules * 3);
        Eigen::VectorXf velocities = Eigen::VectorXf(nbParticules * 3);
        Eigen::VectorXf accelerations = Eigen::VectorXf(nbParticules * 3);

        for(int i = 0; i<nbParticules; i++)
        {
            positions.segment(3*i,3) = q.segment(6*i, 3); //first position in the q vector -> positions
            velocities.segment(3*i,3) = q.segment(6*i+3, 3); //second position in the q vector -> velocities
            accelerations.segment(3*i,3) = dqdt.segment(6*i +3,3);//second position in the dqdt vector -> accelerations (first position in dqdt being velocities, we can safely ignore it)
        }

        //actual integration step
        velocities += dt*accelerations; //-> new velocities
        positions += dt*velocities; //-> new positions based on the velocities at t+1

        //Now we can combine the two vectors into one state vector q !
        for(int i = 0; i<nbParticules; i++)
        {
            q.segment(6*i, 3) = positions.segment(3*i, 3); 
            q.segment(6*i + 3, 3) = velocities.segment(3*i, 3);
        }

        particleSystem->setState(q);
    }

};
