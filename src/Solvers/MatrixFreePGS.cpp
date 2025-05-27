#include "Solvers/MatrixFreePGS.h"

#include "ParticleSystem.h"

MatrixFreePGS::MatrixFreePGS(ParticleSystem* _particleSystem) : m_particleSystem(_particleSystem),
    m_iters(20)
{
}

void MatrixFreePGS::solve(float dt, std::vector<Eigen::Vector3f>& x)
{
    // TODO implement the matrix-free PGS solver for the particle systems to solve
    //  for (M - dt*dfdv - dt*dt*dfdx) x = dt * f + dt * dt * dfdx * v
    //
    // where x is assumed to be the velocity updates deltav used by the integrator.
    //

}

void MatrixFreePGS::buildRHS(float dt, std::vector<Eigen::Vector3f>& b)
{
    // TODO Build the right-hand side block vector: 
    //   b = dt * f + dt * dt * dfdx * v
    // for each particle
}

void MatrixFreePGS::buildBlockDiagonal(float dt, std::vector<Eigen::LDLT<Eigen::Matrix3f>>& P)
{
    // TODO Build and compute the Cholesky decomposition of the block diagonal matrices
    //   A = M - dt*dfdv - dt*dt*dfdx 
    // Store the result in the array P, such that each entry contains
    //    P = llt(A)
    // for each particle.

}



