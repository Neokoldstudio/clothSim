#pragma once

/**
 * @file ClothViewer.h
 *
 * @brief Viewer for a cloth simulation application.
 *
 */
#include <Eigen/Dense>

namespace polyscope
{
    class SurfaceMesh;
    class PointCloud;
}

class Cloth;
class Integrator;
class Particle;

class ClothViewer 
{
public:
    ClothViewer();
    virtual ~ClothViewer();

    void start();

private:
    void createHangingCloth();
    void createTrampoline();

    void draw();
    void drawGUI();

    void initClothData();
    void updateClothData();

    Cloth* m_cloth;                     // The cloth particle system.
    polyscope::SurfaceMesh* m_clothMesh;    // Cloth surface mesh (visual)
    polyscope::PointCloud* m_clothPoints;   // Cloth particles (visual)

    int m_integratorIndex;              // The current integration method.
    bool m_paused;
    bool m_stepOnce;

    Eigen::VectorXf m_q0;               // Initial state of the particle system.

    // Simulation parameters
    float m_dt;                         // Time step parameter.
    float m_structuralStiffness;        // Cloth structural spring stiffness.
    float m_shearStiffness;             // Cloth shear spring stiffness.
    float m_bendingStiffness;           // Cloth bending spring stiffness.
    float m_damping;                    // Cloth damping.

    // Misc. cloth parameters
    int m_nx, m_ny;
    float m_width, m_height;


    Particle* m_pickParticle;           // The picked particle for mouse spring interaction (null by default)
};
