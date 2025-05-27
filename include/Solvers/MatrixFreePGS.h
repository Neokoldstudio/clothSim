#pragma once

#include <Eigen/Dense>
#include <vector>

class ParticleSystem;

// A matrix free PGS solver for mass-spring systems.
//
class MatrixFreePGS
{
public:

    MatrixFreePGS(ParticleSystem* _particleSystem);

    // Solve (M - dt*dfdv - dt*dt*dfdx) x = dt * f + dt * dt * dfdx * v
    // using the projected Gauss-Seidel method.
    //
    void solve(float dt, std::vector<Eigen::Vector3f>& x);

    // Set the max iterations used by the solver.
    void setMaxIterations(int _iters) { m_iters = _iters; }

private:

    // Build the right-hand side block vector: 
    //   b = dt * f + dt * dt * dfdx * v
    // for each particle
    //
    void buildRHS(float dt, std::vector<Eigen::Vector3f>& b);

    // Build and compute the Cholesky decomposition of the block diagonal matrices
    //   A = M - dt*dfdv - dt*dt*dfdx 
    // Store the result in the array P, such that each entry contains
    //    P = llt(A)
    // for each particle.
    void buildBlockDiagonal(float dt, std::vector<Eigen::LDLT<Eigen::Matrix3f>>& P);

    ParticleSystem* m_particleSystem;

    int m_iters;
    std::vector<Eigen::LDLT<Eigen::Matrix3f>> m_P;              // Block diagonal matrices
    std::vector<Eigen::Vector3f> m_b;                           // Block vector of rhs values
};
