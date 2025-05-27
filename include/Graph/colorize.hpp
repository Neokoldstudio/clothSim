#pragma once

/**
 * @file colorize.hpp
 *
 * @brief Utility functions for coloring particles based on the graph coloring algorithm.
 *
 * Nom: Pierre-Antoine Heredero
 * Code permanent : HERP30059500
 * Email : pierre-antoine.heredero.1@ens.etsmtl.ca
 * 
 */

#include <ParticleSystem.h>
#include <omp.h>
#include <utility>
#include <vector>

using namespace std;

inline void find_neighbors(const Particle* particle, vector<int>& neighbors)
{
  neighbors.clear();

  for (pair<Spring*, int> pair : particle->springs) {
    Spring* spring           = pair.first;
    Particle* other_particle = spring->particles[(pair.second + 1) % 2];

    int i0 = spring->particles[0]->index;
    int i1 = spring->particles[1]->index;

    assert(i0 != i1);
    assert(i0 == particle->index || i1 == particle->index);

    neighbors.push_back(i0 == particle->index ? i1 : i0);
  }
}

inline int find_color(const ParticleSystem* particleSystem,
                      const Particle* particle,
                      const vector<int> neighbors,
                      vector<int>& colors)
{
  auto& particles = particleSystem->getParticles();

  const int n = colors.size();

  vector<int> count(n, 0);

  for (auto& color : colors)
    for (auto& index : neighbors)
      if (color == particles[index]->color)
        count[color]++;

  for (int i = 0; i < n; ++i)
    if (count[i] == 0)
      return i;

  colors.push_back(n);

  return n;
}

inline vector<vector<int>> colorize(ParticleSystem* particleSystem)
{
  vector<int> colors;

  auto& particles = particleSystem->getParticles();

  #pragma omp parallel for
  for (int i = 0; i < particles.size(); ++i)
    particles[i]->color = -1;

  for (int i = 0; i < particles.size(); ++i)
  {
    vector<int> neighbors = {};
    find_neighbors(particles[i], neighbors);

    particles[i]->color = find_color(particleSystem, particles[i], neighbors, colors);
  }

  const int n = colors.size();

  vector<vector<int>> partitions;
  partitions.resize(n);

  #pragma omp parallel for
  for (int i = 0; i < colors.size(); ++i) {
    int index = 0;

    for (Particle* particle : particles) {
      if (particle->color == i)
        partitions[i].push_back(index);

      index++;
    }
  }

  return partitions;
}
