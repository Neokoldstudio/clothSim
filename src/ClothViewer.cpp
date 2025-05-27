#include "ClothViewer.h"

#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/point_cloud.h"
#include "polyscope/pick.h"
#include "polyscope/view.h"
#include "imgui.h"

#include <functional>
#include <iostream>

using namespace std;

#include "Cloth.h"
#include "ClothFactory.h"
#include "Integrators/ExplicitEuler.hpp"
#include "Integrators/SemiImplicitEuler.hpp"
#include "Integrators/Midpoint.hpp"
#include "Integrators/ImplicitEuler.hpp"

namespace polyscope
{
    namespace view
    {
        glm::vec2 worldToScreenCoords(glm::vec3 worldPos, float* depth = nullptr)
        {
            glm::mat4 view = polyscope::view::getCameraViewMatrix();
            glm::mat4 proj = polyscope::view::getCameraPerspectiveMatrix();
            glm::vec4 viewport = { 0., 0., polyscope::view::windowWidth, polyscope::view::windowHeight };

            glm::vec3 screenPos3 = glm::project(worldPos, view, proj, viewport);
            if (depth) *depth = screenPos3.z;

            return glm::vec2(screenPos3.x, polyscope::view::windowHeight - screenPos3.y);
        }
    }
}

namespace
{
    enum eIntegrators {
        kExplicitEuler = 0,
        kMidpoint,
        kSemiImplicitEuler,
        kImplicitEuler
    };

    static SemiImplicitEuler s_semiImplicitEuler;
    static ExplicitEuler s_explicitEuler;
    static Midpoint s_midpoint;
    static ImplicitEuler s_implicitEuler;

    // Stores instances of each integrator
    //
    static Integrator* integrators[4] = {
        &s_explicitEuler,
        &s_midpoint,
        &s_semiImplicitEuler,
        &s_implicitEuler
    };

    static const std::array<float, 3> pinColor = { 1.0f, 0.0f, 0.0f };
    static const std::array<float, 3> pointColor = { 1.0f, 1.0f, 0.0f };
}

ClothViewer::ClothViewer() :
    m_cloth(nullptr),
    m_clothMesh(nullptr),
    m_clothPoints(nullptr),
    m_pickParticle(nullptr),
    m_dt(0.01f), m_paused(true), m_stepOnce(false),
    m_structuralStiffness(1000.0f), m_shearStiffness(250.0f), m_bendingStiffness(50.0f), m_damping(0.0f), 
    m_nx(16), m_ny(16), m_width(8.0f), m_height(8.0f),
    m_integratorIndex(kExplicitEuler)
{

}

ClothViewer::~ClothViewer()
{
    delete m_cloth;
}

void ClothViewer::start()
{
    m_cloth = new Cloth;
    m_cloth->getState(m_q0);
    m_integratorIndex = kExplicitEuler;

    // Setup Polyscope
    polyscope::options::programName = "MTI855 Devoir 01 - Cloth Sim";
    polyscope::options::verbosity = 0;
    polyscope::options::usePrefsFile = false;
    polyscope::options::alwaysRedraw = true;
    polyscope::options::ssaaFactor = 2;
    polyscope::options::openImGuiWindowForUserCallback = true;
    polyscope::options::groundPlaneHeightFactor = 1.0; // adjust the plane height
    polyscope::options::buildGui = true;
    polyscope::options::maxFPS = 120;

    // initialize
    polyscope::init();

    // Specify the update callback
    polyscope::state::userCallback = std::bind(&ClothViewer::draw, this);

    createHangingCloth();

    // Show the window
    polyscope::show();

}


void ClothViewer::drawGUI()
{
    ImGui::Text("Simulation:");
    ImGui::Checkbox("Pause", &m_paused);
    if (ImGui::Button("Step once"))
    {
        m_stepOnce = true;
    }
    ImGui::PushItemWidth(100);
    ImGui::SliderFloat("Time step", &m_dt, 0.0f, 0.1f, "%.3f");
    ImGui::PopItemWidth();

    ImGui::Text("Cloth parameters: ");
    ImGui::PushItemWidth(200);
    ImGui::SliderFloat("Structural stiffness", &m_structuralStiffness, 0.0f, 10000.0f, "%.1f");
    ImGui::SliderFloat("Shear stiffness", &m_shearStiffness, 0.0f, 10000.0f, "%.1f");
    ImGui::SliderFloat("Bending stiffness", &m_bendingStiffness, 0.0f, 10000.0f, "%.1f");
    ImGui::SliderFloat("Damping", &m_damping, 0.0f, 100.0f, "%.1f");
    ImGui::PopItemWidth();

    ImGui::Text("Integrators: ");
    ImGui::RadioButton("Explicit", &m_integratorIndex, kExplicitEuler); ImGui::SameLine();
    ImGui::RadioButton("Midpoint", &m_integratorIndex, kMidpoint); ImGui::SameLine();
    ImGui::RadioButton("Semi-implicit", &m_integratorIndex, kSemiImplicitEuler); ImGui::SameLine();
    ImGui::RadioButton("Implicit", &m_integratorIndex, kImplicitEuler);

    ImGui::Text("Scenarios: ");
    ImGui::PushItemWidth(200);
    ImGui::SliderInt("Num. particles (horizontal)", &m_nx, 2, 256);
    ImGui::SliderInt("Num. particles (vertical)", &m_ny, 2, 256);
    ImGui::SliderFloat("Width:", &m_width, 1.0f, 100.0f, "%.1f");
    ImGui::SliderFloat("Height:", &m_height, 1.0f, 100.0f, "%.1f");
    ImGui::PopItemWidth();
    if (ImGui::Button("Hanging cloth")) {
        createHangingCloth();
    }
    else if (ImGui::Button("Trampoline cloth")) {
        createTrampoline();
    }

}

void ClothViewer::initClothData()
{
    const auto& particles = m_cloth->getParticles();
    const unsigned int numParticles = particles.size();
    const int width = m_cloth->getWidth();
    const int height = m_cloth->getHeight();
    const unsigned numTriangles = 2 * (width - 1) * (height - 1);

    Eigen::MatrixXf meshV(numParticles, 3);
    Eigen::MatrixXi meshF(numTriangles, 3);

    for (int i = 0; i < numParticles; ++i)
    {
        meshV.row(i) << particles[i]->x(0), particles[i]->x(1), particles[i]->x(2);
    }
    unsigned int k = 0;
    for (int j = 0; j < width - 1; ++j)
    {
        for (int i = 0; i < height - 1; ++i)
        {
            meshF.row(k++) << m_cloth->getParticleIndex(i, j), m_cloth->getParticleIndex(i + 1, j + 1), m_cloth->getParticleIndex(i + 1, j);
            meshF.row(k++) << m_cloth->getParticleIndex(i, j), m_cloth->getParticleIndex(i, j + 1), m_cloth->getParticleIndex(i + 1, j + 1);
        }
    }

    // Register the mesh with Polyscope
    m_clothMesh = polyscope::registerSurfaceMesh("cloth", meshV, meshF);
    m_clothMesh->setSmoothShade(true);

    // Register the particles point cloud with Polyscope
    m_clothPoints = polyscope::registerPointCloud("particles", meshV);
    m_clothPoints->setPointRadius(0.01);
    m_clothPoints->setPointRenderMode(polyscope::PointRenderMode::Sphere);
    m_clothPoints->addColorQuantity("colors", std::vector< std::array<float, 3> >(numParticles, pointColor))->setEnabled(true);
}

void ClothViewer::updateClothData()
{
    const auto& particles = m_cloth->getParticles();
    const unsigned int numParticles = particles.size();

    Eigen::MatrixXf meshV(numParticles, 3);
    for (int i = 0; i < numParticles; ++i)
    {
        meshV.row(i) << particles[i]->x(0), particles[i]->x(1), particles[i]->x(2);
    }
    m_clothMesh->updateVertexPositions(meshV);
    m_clothPoints->updatePointPositions(meshV);

    // Polyscope requires updating *all* point quantities
    //
    std::vector< std::array<float, 3> > pointColors(numParticles, pointColor);
    for (int i = 0; i < numParticles; ++i)
    {
        if (particles[i]->fixed)
            pointColors[i] = pinColor;
    }
    m_clothPoints->addColorQuantity("colors", pointColors);

    auto& springs = m_cloth->getSprings();
    for (int i = m_cloth->getStructuralIndex(); i < m_cloth->getShearIndex(); ++i)
    {
        springs[i]->k = m_structuralStiffness;
        springs[i]->b = m_damping;
    }
    for (int i = m_cloth->getShearIndex(); i < m_cloth->getBendingIndex(); ++i)
    {
        springs[i]->k = m_shearStiffness;
        springs[i]->b = m_damping;
    }
    for (int i = m_cloth->getBendingIndex(); i < springs.size(); ++i)
    {
        springs[i]->k = m_bendingStiffness;
        springs[i]->b = m_damping;
    }
}

void ClothViewer::draw()
{
    drawGUI();

	// Perform particle selection
	//
	if (ImGui::IsMouseDown(0) && ImGui::GetIO().KeyCtrl && m_pickParticle == nullptr)
	{
		const ImVec2 mouseP = ImGui::GetMousePos();
		const auto selection = polyscope::pick::evaluatePickQuery(mouseP.x, mouseP.y);

		if (m_clothPoints == selection.first)
		{
			const unsigned int pickInd = selection.second;
			auto& particles = m_cloth->getParticles();
			m_pickParticle = particles[pickInd];
		}
	}
	// Perform particle pinning
	// 
	else if (ImGui::IsMouseClicked(1) && ImGui::GetIO().KeyCtrl && m_pickParticle == nullptr)
	{
		const ImVec2 mouseP = ImGui::GetMousePos();
		const auto selection = polyscope::pick::evaluatePickQuery(mouseP.x, mouseP.y);

		if (m_clothPoints == selection.first)
		{
			const unsigned int pickInd = selection.second;
			auto& particles = m_cloth->getParticles();
			particles[pickInd]->fixed = !(particles[pickInd]->fixed);
		}
	}
	else if (ImGui::IsMouseReleased(0) && m_pickParticle)
	{
		m_pickParticle = nullptr;
	}

    // Simulation stepping
	//
	if (!m_paused || m_stepOnce)
	{
		// Compute cloth forces.
		//
		m_cloth->computeForces();

		// Particle being dragged by the mouse spring.
        // Note: mouse spring force must be applied after
        // computeForces() is called.
		// 
		if (ImGui::IsMouseDown(0) && ImGui::GetIO().KeyCtrl && m_pickParticle )
		{
			ImVec2 mouseP = ImGui::GetMousePos();

			glm::vec3 p = { m_pickParticle->x.x(), m_pickParticle->x.y(), m_pickParticle->x.z() };
			glm::vec2 screenCoord = polyscope::view::worldToScreenCoords(p);

			auto& particles = m_cloth->getParticles();

			glm::vec3 lookDir, upDir, rightDir;
			polyscope::view::getCameraFrame(lookDir, upDir, rightDir);

			glm::vec3 f = (mouseP.x - screenCoord.x) * rightDir + (screenCoord.y - mouseP.y) * upDir;

			// Apply mouse spring force.
			{
				static const float K_mouse = 50.0f;
				Eigen::Vector3f u(f.x, f.y, f.z);
				const float ulen = u.norm();
				u.normalize();

				if (ulen > 0.1f)
				{
					m_pickParticle->f += K_mouse * ulen * u - 0.1f * K_mouse * (m_pickParticle->v.dot(u)) * u;
				}
			}
		}

		// Step the simulation
		integrators[m_integratorIndex]->step(m_cloth, m_dt);

		m_stepOnce = false;
	}

	// Update cloth mesh and point positions for rendering, 
	//  as well as cloth params.
    //
    updateClothData();

}


void ClothViewer::createHangingCloth()
{
    assert(m_nx > 1);
    assert(m_ny > 1);

    const float xoff = 0.5f * m_width;
    const float zoff = 0.5f * m_height;

    delete m_cloth;
    m_cloth = ClothFactory::createHangingCloth(m_nx, m_ny, m_width / (m_nx - 1), m_height / (m_ny - 1), m_structuralStiffness, m_shearStiffness, m_bendingStiffness, m_damping, -xoff, -zoff);
    m_cloth->getState(m_q0);

    initClothData();
}

void ClothViewer::createTrampoline()
{
    assert(m_nx > 1);
    assert(m_ny > 1);

    const float xoff = 0.5f * m_width;
    const float zoff = 0.5f * m_height;

    delete m_cloth;
    m_cloth = ClothFactory::createTrampoline(m_nx, m_ny, m_width / (m_nx - 1), m_height / (m_ny - 1), m_structuralStiffness, m_shearStiffness, m_bendingStiffness, m_damping, -xoff, -zoff);
    m_cloth->getState(m_q0);

    initClothData();
}
