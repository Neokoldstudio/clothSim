#include "ParticleSystem.h"

void ParticleSystem::addParticle(Particle *_particle) {
  m_particles.push_back(_particle);
}

void ParticleSystem::addSpring(Spring *_spring) {
  m_springs.push_back(_spring);
}

// Compute forces for each particle p and accumulate the net force in p.f
// Note: force should not be applied to fixed particles.
//
void ParticleSystem::computeForces() {

  const int numParticles = m_particles.size();
  // TODO Initialize and compute the gravity acting on each particle. -> Done
  //
  Eigen::Vector3f g(0, -9.81, 0);

  for (int i = 0; i < numParticles; i++) {
    m_particles[i]->f += g * m_particles[i]->m; // gravity
  }

  // TODO For each spring, add the force of the spring to each particle -> Done
  //      Recall that the force acting on particle with index0 is equal and
  //      opposite the force acting on index1.
  //
  const int numSprings = m_springs.size();

  for (int i = 0; i < numSprings; i++) {
    Spring *currentSpring = m_springs[i];

    Particle *part0 = currentSpring->particles[0];
    Particle *part1 = currentSpring->particles[1];

    Eigen::Vector3f delta = part1->x - part0->x; // vector from part0 to part1
    Eigen::Vector3f deltaNormalized =
        delta.normalized(); // normalized delta vec
    float length = delta.norm();
    float projectedVel =
        (part1->v - part0->v)
            .dot(deltaNormalized); // for damping : project the velocities onto
                                   // the delta vector

    Eigen::Vector3f f = (currentSpring->k * (length - currentSpring->r) +
                         currentSpring->b * (projectedVel)) *
                        delta.normalized();

    part0->f += f;
    part1->f -= f;
  }
}

// TODO Computes the derivative of the state vector and returns in @a dqdt.
//      Assume that computeForces() has already been called.
//
void ParticleSystem::derivs(Eigen::VectorXf &dqdt) {
  const int numParticles = m_particles.size();

  // Deriv vector has size 6n
  const int dim = 6 * numParticles;
  dqdt.resize(dim);

  // TODO Loop over all particles and compute dqdt.
  for (int i = 0; i < numParticles; i++) {
    dqdt.segment(6 * i, 3) = m_particles[i]->v;
    dqdt.segment(6 * i + 3, 3) = m_particles[i]->f / m_particles[i]->m;

    if (m_particles[i]->fixed) {
      dqdt.setConstant(0.0f);
    }
  }
}

// TODO Assemble the state vector q, which consists of particle positions and
// velocities
void ParticleSystem::getState(Eigen::VectorXf &q) const {
  const int numParticles = m_particles.size();

  // State vector has size 6n
  const int dim = 6 * numParticles;
  q.resize(dim);

  // TODO Loop over all particle and build vector q
  for (int i = 0; i < numParticles; i++) {
    q.segment(6 * i, 3) =
        m_particles[i]
            ->x; // get the postion of the particle and puts it in the vector q
    q.segment(6 * i + 3, 3) =
        m_particles[i]
            ->v; // get the velocity of the particle and puts it in the vector q
  }
}

// Update position and velocity of each particle using state vector q.
void ParticleSystem::setState(const Eigen::VectorXf &q) {
  // Assemble the particle system into the provided vectors
  const int numParticles = m_particles.size();
  const int dim = 6 * numParticles;

  assert(q.size() == dim);

  for (int i = 0; i < numParticles; ++i) {
    if (m_particles[i]->fixed) {
      // Uncomment the line below to update positions of 'fixed' particles.
      // m_particles[i]->x = q.segment(6*i,3);
      m_particles[i]->v.setZero();
    } else {
      m_particles[i]->x = q.segment(6 * i, 3);
      m_particles[i]->v = q.segment(6 * i + 3, 3);
    }
  }
}

// Construct the dfdx matrices per spring
void ParticleSystem::dfdx() {
  // TODO Compute the dfdx matrix for the springs (see slides)
  //
  for (Spring *spring : m_springs) {
  }
}