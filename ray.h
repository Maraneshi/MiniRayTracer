#pragma once

#include "vec3.h"

class ray {
public:
    Vec3 origin;
    Vec3 dir;
#ifdef NEW_INTERSECT
    Vec3 shear;
    uint32_t kx, ky, kz;
#endif
    float time = 0;
    // hit backfaces if inside a solid volume (e.g. inside glass), can count nested volumes
    int isInside = 0;
    uint8 dirMask = 0;

    ray() = default;

    static uint8 ComputeDirMask(const Vec3& dir)
    {
        const uint32 X = *(uint32*)&dir.x >> 31;
        const uint32 Y = *(uint32*)&dir.y >> 31;
        const uint32 Z = *(uint32*)&dir.z >> 31;
        const uint32 BitIndex = Z | (Y << 1) | (X << 2);
        return 1 << BitIndex;
    }
    
    // direction will be normalized
    ray(const Vec3& origin, const Vec3& dir, float time, int isInside = 0) : origin(origin), dir(normalize(dir)), time(time), isInside(isInside) { 

#ifdef NEW_INTERSECT
        // Watertight Ray/Triangle Intersection (http://jcgt.org/published/0002/01/05/paper.pdf)
        /* calculate dimension where the ray
            direction is maximal */
        kz = max_dim(vabs(dir));
        kx = kz + 1; if (kx == 3) kx = 0;
        ky = kx + 1; if (ky == 3) ky = 0;

        /* swap kx and ky dimension to preserve
            winding direction of triangles */
        if (dir[kz] < 0.0f) std::swap(kx, ky);

        /* calculate shear constants */
        float invZ = 1.0f / dir[kz];
        this->shear.x = dir[kx] * invZ;
        this->shear.y = dir[ky] * invZ;
        this->shear.z = invZ;
        this->shear.w = 0.0f;
#endif
        dirMask = ComputeDirMask(dir);
    }

    Vec3 eval(float t) const {
        return origin + t * dir;
    }

};