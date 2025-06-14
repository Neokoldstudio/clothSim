#include "Solvers/MatrixFreePGS.h"

#include "Eigen/src/Core/Matrix.h"
#include "ParticleSystem.h"

MatrixFreePGS::MatrixFreePGS(ParticleSystem* _particleSystem) : m_particleSystem(_particleSystem), m_iters(20)
{
}

void MatrixFreePGS::solve(float dt, std::vector<Eigen::Vector3f>& x)
{
    // TODO implement the matrix-free PGS solver for the particle systems to solve
    //  for (M - dt*dfdv - dt*dt*dfdx) x = dt * f + dt * dt * dfdx * v
    //
    // where x is assumed to be the velocity updates deltav used by the
    // integrator.
    //
    m_particleSystem->dfdx();
    int nbParticules = m_particleSystem->getParticles().size();

    x.resize(nbParticules);
    std::vector<Eigen::Vector3f> b(nbParticules);
    std::vector<Eigen::LDLT<Eigen::Matrix3f>> P(nbParticules);
    buildRHS(dt, b);
    buildBlockDiagonal(dt, P);

    for (int i = 0; i < nbParticules; i++) {
        Particle *p = m_particleSystem->getParticles()[i];
        if (p->fixed) x[i] = Eigen::Vector3f::Zero();
        else {
            x[i] = b[i];
            for(std::pair<Spring *, int> pair : p->springs) {
                int j = (pair.second+1)%2;
                Spring *s = pair.first;
                x[i] -= (dt*dt*s->dfdx) * x[s->particles[j]->index];
            }
            x[i] = P[i].solve(x[i]);
        }
    }
}

void MatrixFreePGS::buildRHS(float dt, std::vector<Eigen::Vector3f>& b)
{
    // TODO Build the right-hand side block vector:
    //   b = dt * f + dt * dt * dfdx * v
    // for each particle
    b.resize(m_particleSystem->getParticles().size());

    for (Particle* p : m_particleSystem->getParticles()) {
        b[p->index] = dt*p->f;
        for(std::pair<Spring *, int> pair : p->springs) {
            int j = (pair.second + 1) % 2;
            Particle* otherParticle = pair.first->particles[j];
            Spring *s = pair.first;
            b[p->index] += dt*dt*(s->dfdx * p->v - s->dfdx * otherParticle->v);
        }
    }
}

void MatrixFreePGS::buildBlockDiagonal(float dt, std::vector<Eigen::LDLT<Eigen::Matrix3f>>& P)
{
    // TODO Build and compute the Cholesky decomposition of the block diagonal matrices
    //   A = M - dt*dfdv - dt*dt*dfdx
    // Store the result in the array P, such that each entry contains
    //    P = llt(A)
    // for each particle.
    const int nbParticules = m_particleSystem->getParticles().size();
    std::vector<Eigen::Matrix3f> M(nbParticules);
    P.resize(nbParticules);

    for (Particle* p : m_particleSystem->getParticles()) {
        M[p->index] = p->m * Eigen::Matrix3f::Identity();

        for(std::pair<Spring *, int> pair : p->springs) {
            Spring* s = pair.first;
            M[p->index] -= dt*dt*s->dfdx;
        }
        P[p->index] = M[p->index].ldlt();
    }
}



