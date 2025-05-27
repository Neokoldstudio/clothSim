#include "ClothFactory.h"
#include "Cloth.h"

#include <Eigen/Dense>

// @a nx Number of particles in the x direction
// @a ny Number of particles in the y direction
// @a dx Spacing between particles along the x-axis
// @a dy Spacing between particles along the y-axis
// @a k1 Structural stiffness
// @a k2 Shearing stiffness
// @a k3 Bending stiffness
// @a b Spring damping
// @a startx Initial x position of the first particle.
// @a starty Initial y position of the first particle.
//
Cloth* ClothFactory::createHangingCloth(int nx, int ny, float dx, float dy, float k1, float k2, float k3, float b, float startx, float starty)
{
    assert(nx > 1 && ny > 1);

    Cloth* cloth = new Cloth(nx, ny);
    cloth->clear();

    int index = 0;

    // Create all particles.
    for (int i = 0; i < ny; ++i)
    {
        for (int j = 0; j < nx; ++j)
        {
            const float x = startx + (float)j * dx;
            const float y = starty + (float)i * dy;
            const float z = 2.0f;
            Particle* particle = new Particle(index, Eigen::Vector3f(x, y, z), Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0, 0, 0), 1.0);
            if (i == (ny - 1)) particle->fixed = true;
            cloth->addParticle(particle);
            ++index;
        }
    }
    // Structural springs.
    //
    cloth->setStructuralIndex(0);
    for (int i = 0; i < ny; ++i)
    {
        for (int j = 0; j < nx; ++j)
        {
            if (i > 0)
            {
                cloth->addSpring(new Spring(cloth->getParticle(i - 1, j), cloth->getParticle(i, j), k1, b, dy));
            }
            if (j > 0)
            {
                cloth->addSpring(new Spring(cloth->getParticle(i, j - 1), cloth->getParticle(i, j), k1, b, dx));
            }
        }
    }

    // Shear springs.
    //
    cloth->setShearIndex(cloth->getSprings().size());
    for (int i = 0; i < ny; ++i)
    {
        for (int j = 0; j < nx; ++j)
        {
            if (i > 0 && j > 0)
            {
                cloth->addSpring(new Spring(cloth->getParticle(i - 1, j - 1), cloth->getParticle(i, j), k2, b, std::sqrt(dx * dx + dy * dy)));
            }
            if (i < (ny - 1) && j >0)
            {
                cloth->addSpring(new Spring(cloth->getParticle(i + 1, j - 1), cloth->getParticle(i, j), k2, b, std::sqrt(dx * dx + dy * dy)));
            }
        }
    }

    cloth->setBendingIndex(cloth->getSprings().size());
    for (int i = 0; i < ny; ++i)
    {
        for (int j = 0; j < nx; ++j)
        {
            // Bend springs.
            //
            if (j > 1)
            {
                cloth->addSpring(new Spring(cloth->getParticle(i, j - 2), cloth->getParticle(i, j), k3, b, 2.0f * dx));
            }
            if (i > 1)
            {
                cloth->addSpring(new Spring(cloth->getParticle(i - 2, j), cloth->getParticle(i, j), k3, b, 2.0f * dy));
            }
        }
    }

    return cloth;
}

// @a nx Number of particles in the x direction
// @a ny Number of particles in the y direction
// @a dx Spacing between particles along the x-axis
// @a dz Spacing between particles along the y-axis
// @a k1 Structural stiffness
// @a k2 Shearing stiffness
// @a k3 Bending stiffness
// @a b Spring damping
// @a startx Initial x position of the first particle.
// @a starty Initial y position of the first particle.
//
Cloth* ClothFactory::createTrampoline(int nx, int nz, float dx, float dz, float k1, float k2, float k3, float b, float startx, float startz)
{
    assert(nx > 1 && nz > 1);

    Cloth* cloth = new Cloth(nx, nz);
    cloth->clear();

    int index = 0;
    const float lastz = startz + (float)nz * dz;
    for (int i = 0; i < nz; ++i)
    {
        for (int j = 0; j < nx; ++j)
        {
            const float x = startx + (float)j * dx;
            const float z = lastz - (float)i * dz;
            const float y = 2.0f;
            Particle* particle = new Particle(index, Eigen::Vector3f(x, y, z), Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0, 0, 0), 1.0);
            if (i == 0 && j == 0) particle->fixed = true;
            else if (i == nz - 1 && j == 0) particle->fixed = true;
            else if (i == nz - 1 && j == nx - 1) particle->fixed = true;
            else if (i == 0 && j == nx - 1) particle->fixed = true;
            cloth->addParticle(particle);
            ++index;
        }
    }

    // Structural springs.
    //
    cloth->setStructuralIndex(0);
    for (int i = 0; i < nz; ++i)
    {
        for (int j = 0; j < nx; ++j)
        {
            if (i > 0)
            {
                cloth->addSpring(new Spring(cloth->getParticle(i - 1, j), cloth->getParticle(i, j), k1, b, dz));
            }
            if (j > 0)
            {
                cloth->addSpring(new Spring(cloth->getParticle(i, j - 1), cloth->getParticle(i, j), k1, b, dx));
            }
        }
    }
    // Shear springs.
    //
    cloth->setShearIndex(cloth->getSprings().size());
    for (int i = 0; i < nz; ++i)
    {
        for (int j = 0; j < nx; ++j)
        {

            if (i > 0 && j > 0)
            {
                cloth->addSpring(new Spring(cloth->getParticle(i - 1, j - 1), cloth->getParticle(i, j), k2, b, std::sqrt(dx * dx + dz * dz)));
            }
            if (i < (nz - 1) && j >0)
            {
                cloth->addSpring(new Spring(cloth->getParticle(i + 1, j - 1), cloth->getParticle(i, j), k2, b, std::sqrt(dx * dx + dz * dz)));
            }
        }
    }

    // Bend springs.
    //
    cloth->setBendingIndex(cloth->getSprings().size());
    for (int i = 0; i < nz; ++i)
    {
        for (int j = 0; j < nx; ++j)
        {
            if (j > 1)
            {
                cloth->addSpring(new Spring(cloth->getParticle(i, j - 2), cloth->getParticle(i, j), k3, b, 2.0f * dx));
            }
            if (i > 1)
            {
                cloth->addSpring(new Spring(cloth->getParticle(i - 2, j), cloth->getParticle(i, j), k3, b, 2.0f * dz));
            }

        }
    }

    return cloth;
}
