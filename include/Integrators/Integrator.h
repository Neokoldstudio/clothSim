#pragma once

class ParticleSystem;

class Integrator
{
public:

    // Interface for integrators.
    //
    virtual void step(ParticleSystem* particleSystem, float dt) = 0;

};
