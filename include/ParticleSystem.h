#pragma once

/**
 * @file ParticleSystem.h
 *
 * @brief A mass-spring particle system.
 *
 */

#include <Eigen/Dense>
#include <vector>

class Spring;

// A 3D particle class.
//
class Particle
{
public:
    int index;
    bool fixed;                                    // flag for static particles
    float m;                                       // mass
    Eigen::Vector3f x;                             // position
    Eigen::Vector3f v;                             // velocity
    Eigen::Vector3f f;                             // force
    std::vector<std::pair<Spring *, int>> springs; // Springs connected to this particle

    Particle(int _index, const Eigen::Vector3f &_x, const Eigen::Vector3f &_v, const Eigen::Vector3f &_f, float _m) : index(_index), x(_x), v(_v), f(_f), m(_m), fixed(false) {}

    Particle() : index(-1), fixed(false), x(0, 0, 0), v(0, 0, 0), f(0, 0, 0), m(1.0) {}
};

// A spring between two particles.
//
class Spring
{
public:
    Particle *particles[2];
    float k; // spring stiffness
    float b; // spring damping
    float r; // rest (neutral) length

    Eigen::Matrix3f dfdx; // Stiffness matrix
    Eigen::Matrix3f dfdv; // Damping matrix

    Spring(Particle *_p0, Particle *_p1, float _k, float _b, float _r) : k(_k), b(_b), r(_r)
    {
        assert(_p0 != nullptr);
        assert(_p1 != nullptr);

        // Copy this spring to the list maintained
        // by each particle
        particles[0] = _p0;
        particles[1] = _p1;
        _p0->springs.push_back({this, 0});
        _p1->springs.push_back({this, 1});
    }
};

// A 3D particle system class.
//
class ParticleSystem
{
protected:
    std::vector<Particle *> m_particles; // particles
    std::vector<Spring *> m_springs;     // springs

public:
    ParticleSystem() : m_particles(), m_springs() {}

    virtual ~ParticleSystem()
    {
        clear();
    }

    // Clear all particles and all springs
    void clear()
    {
        for (Particle *p : m_particles)
        {
            delete p;
        }
        m_particles.clear();
        for (Spring *s : m_springs)
        {
            delete s;
        }
        m_springs.clear();
    }

    // Add a particle.  The Particle is copied into m_particles.
    //
    void addParticle(Particle *_particle);

    // Add a spring. The Spring is copied into m_spring.
    //
    void addSpring(Spring *_spring);

    // Compute the forces acting on particles. The derivative vector @a dqdt has the layout :
    //   [ v1, f1/m1, v2, f2/m2, ... vn, fn/mn ]
    //
    void computeForces();

    // Compute velocities and forces acting on each particle
    //
    void derivs(Eigen::VectorXf &dqdt);

    // Get the state of the particle system.  The state vector @a q has the layout :
    //   [ x1, v1, x2, v2, ... xn, vn]
    //
    void getState(Eigen::VectorXf &q) const;

    // Set the state of the particle system.  The state vector @a q has the layout :
    //   [ x1, v1, x2, v2, ... xn, vn]
    //
    void setState(const Eigen::VectorXf &q);

    // Accessors for particles and springs
    //
    const std::vector<Particle *> &getParticles() const { return m_particles; }
    std::vector<Particle *> &getParticles() { return m_particles; }
    const std::vector<Spring *> &getSprings() const { return m_springs; }
    std::vector<Spring *> &getSprings() { return m_springs; }

    // Compute the dfdx matrix for each spring.
    void dfdx();
};
