#pragma once

/**
 * @file Cloth.h
 *
 * @brief A mass-spring cloth particle system.
 *
 * Nom:
 * Code permanent :
 * Email :
 *
 */

#include "ParticleSystem.h"

#include <vector>


// A simple cloth class.
//
//  The cloth is a nx-by-ny rectangular grid of particles arranged as rows.
//
//  The springs in the particle system are always stored according to the following layout
//  in ParticleSystem::m_springs
//
//  [structuralSpring_1 .... structuralSpring_n],[shearSpring_1.... shearSpring_n],[bendingSpring_1... bendingSpring_n]
//
//  The indices for the start of each of these blocks can be accessed by the getters:
//     getStructuralIndex(), getShearIndex(), and getBendingIndex()
//
class Cloth : public ParticleSystem
{
public:
    Cloth() : m_nx(0), m_ny(0), m_structuralIndex(0), m_shearIndex(0), m_bendingIndex(0)
    {
        m_particles.resize(0);
    }

    explicit Cloth(int _nx, int _ny) : m_nx(_nx), m_ny(_ny), m_structuralIndex(0), m_shearIndex(0), m_bendingIndex(0)
    {
        m_particles.resize(m_nx*m_ny);
    }

    virtual ~Cloth() { }

    // Returns pointer to the particle at grid coordinate (i,j)
    Particle* getParticle(int i, int j) { return m_particles[m_nx * i + j]; }

    // Returns the index of cloth particle at grid coordinate (i,j)
    int getParticleIndex(int i, int j) const { return m_nx * i + j; }

    // Width of the cloth in particles.
    int getWidth() const { return m_nx; }

    // Height of the cloth in particles.
    int getHeight() const { return m_ny; }

    // Returns the index of the first structural spring.
    int getStructuralIndex() const { return m_structuralIndex; }
    void setStructuralIndex(int _structuralIndex) { m_structuralIndex = _structuralIndex; }

    // Returns the index of the first shear spring.
    int getShearIndex() const { return m_shearIndex; }
    void setShearIndex(int _shearIndex) { m_shearIndex = _shearIndex; }

    // Return the index of the first bending spring.
    int getBendingIndex() const { return m_bendingIndex; }
    void setBendingIndex(int _bendingIndex) { m_bendingIndex = _bendingIndex; }


protected:
    int m_nx, m_ny;
    int m_structuralIndex, m_shearIndex, m_bendingIndex;

};


