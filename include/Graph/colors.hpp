#pragma once

/**
 * @file colors.hpp
 *
 * @brief Const compliant array of colors encoded in unit RGB to be used in
 *        the graph coloring algorithm's visual representation.
 *
 * Nom: Pierre-Antoine Heredero
 * Code permanent : HERP30059500
 * Email : pierre-antoine.heredero.1@ens.etsmtl.ca
 * 
 */

#include <array>
#include <vector>

using namespace std;

const vector<array<float, 3>> COLORS = {
  {1.00f, 0.00f, 0.00f}, {0.00f, 1.00f, 0.00f}, {0.00f, 0.00f, 1.00f},
  {1.00f, 1.00f, 0.00f}, {1.00f, 0.00f, 1.00f}, {0.00f, 1.00f, 1.00f},
  {0.00f, 0.00f, 0.00f}, {0.50f, 0.00f, 0.00f}, {0.00f, 0.50f, 0.00f},
  {0.00f, 0.00f, 0.50f}, {0.50f, 0.50f, 0.00f}, {0.50f, 0.00f, 0.50f},
  {0.00f, 0.50f, 0.50f}, {0.50f, 0.50f, 0.50f}, {0.75f, 0.00f, 0.00f},
  {0.00f, 0.75f, 0.00f}, {0.00f, 0.00f, 0.75f}, {0.75f, 0.75f, 0.00f},
  {0.75f, 0.00f, 0.75f}, {0.00f, 0.75f, 0.75f}, {0.75f, 0.75f, 0.75f},
  {0.25f, 0.00f, 0.00f}, {0.00f, 0.25f, 0.00f}, {0.00f, 0.00f, 0.25f},
  {0.25f, 0.25f, 0.00f}, {0.25f, 0.00f, 0.25f}, {0.00f, 0.25f, 0.25f},
  {0.25f, 0.25f, 0.25f}, {0.13f, 0.00f, 0.00f}, {0.00f, 0.13f, 0.00f},
  {0.00f, 0.00f, 0.13f}, {0.13f, 0.13f, 0.00f}, {0.13f, 0.00f, 0.13f},
  {0.00f, 0.13f, 0.13f}, {0.13f, 0.13f, 0.13f}, {0.38f, 0.00f, 0.00f},
  {0.00f, 0.38f, 0.00f}, {0.00f, 0.00f, 0.38f}, {0.38f, 0.38f, 0.00f},
  {0.38f, 0.00f, 0.38f}, {0.00f, 0.38f, 0.38f}, {0.38f, 0.38f, 0.38f},
  {0.63f, 0.00f, 0.00f}, {0.00f, 0.63f, 0.00f}, {0.00f, 0.00f, 0.63f},
  {0.63f, 0.63f, 0.00f}, {0.63f, 0.00f, 0.63f}, {0.00f, 0.63f, 0.63f},
  {0.63f, 0.63f, 0.63f}, {0.88f, 0.00f, 0.00f}, {0.00f, 0.88f, 0.00f},
  {0.00f, 0.00f, 0.88f}, {0.88f, 0.88f, 0.00f}, {0.88f, 0.00f, 0.88f},
  {0.00f, 0.88f, 0.88f}, {0.88f, 0.88f, 0.88f}};
