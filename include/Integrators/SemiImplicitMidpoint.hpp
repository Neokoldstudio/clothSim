#pragma once

/**
 * @file SemiImplicitMidpoint.hpp
 *
 * @brief Semi-Implicit Midpoint method.
 *
 * Nom: Pierre-Antoine Heredero
 * Code permanent : HERP30059500
 * Email : pierre-antoine.heredero.1@ens.etsmtl.ca
 *
 */

#include "Integrator.h"
#include "ParticleSystem.h"

#include <Eigen/Dense>
#include <omp.h>

class SemiImplicitMidpoint : public Integrator {
public:
  // TODO [X] Implement semi-explicit Euler integration that advances @a particleSystem by one time step @a dt.
  //          Hint: handle velocity and position  updates separately.
  //
  virtual void step(ParticleSystem* particleSystem, const float dt) override
  {
    for (int i = 0; i < 2; i++) {
      if (i != 0)
        particleSystem->computeForces();

      auto& particles = particleSystem->getParticles();

      #pragma omp parallel for
      for (int i = 0; i < particles.size(); ++i)
      {
        if (!particles[i]->fixed) {
            particles[i]->v += dt / 2.0f * particles[i]->f / particles[i]->m;
            particles[i]->x += dt / 2.0f * particles[i]->v;
        }
      }
    }
  }
};
