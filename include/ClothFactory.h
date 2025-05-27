#pragma once

/**
 * @file ClothFactory.h
 *
 * @brief Factory class for creating cloth particle systems.
 *
 */

class Cloth;

// Simple factory class to create a Cloth.
//
class ClothFactory
{
public:

    static Cloth* createHangingCloth(int nx, int ny, float dx, float dy, float k1, float k2, float k3, float b, float startx, float starty);

    static Cloth* createTrampoline(int nx, int nz, float dx, float dz, float k1, float k2, float k3, float b, float startx, float startz);
};
