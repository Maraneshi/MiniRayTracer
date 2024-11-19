#pragma once

#include "ray.h"
#include "aabb.h"
#include "pcg.h"
#include <stdlib.h> // qsort

class material;

struct hit_record {
    float t = INFINITY;
    Vec3 p;
    Vec3 n;
    float u;
    float v;
    material *mat_ptr;
};


class scene_object {
public:
    virtual bool hit(const ray& r, float tmin, float tmax, hit_record *rec) const = 0;
    virtual bool bounding_box(aabb* box, float time0, float time1) const = 0;
    virtual float pdf_value(const Vec3& origin, const Vec3& dir, float time) const {
        return 0;
    }
    virtual Vec3 pdf_generate(const Vec3& origin, float time) const {
        return Vec3(1, 0, 0);
    }
    virtual void precompute_node_order() {}
    virtual ~scene_object() {}
};

//////////////////////////////////////////////////////////////////////////////////////

// TODO: test invalid bbox code (for objects like planes)

template <typename T>
class object_list final : public scene_object {
public:
    T **list;
    size_t count;
    aabb box;
    bool hasBox;

    object_list(T* l[], size_t n, float time0, float time1);

    bool hit(const ray& r, float tmin, float tmax, hit_record *rec) const override;
    bool bounding_box(aabb* b, float time0, float time1) const override;
    float pdf_value(const Vec3& origin, const Vec3& dir, float time) const override;
    Vec3 pdf_generate(const Vec3& origin, float time) const override;
};

template <typename T>
bool object_list<T>::bounding_box(aabb* b, float time0, float time1) const {
    if (!hasBox) {
        return false;
    }
    else {
        *b = box;
        return true;
    }
}

template <typename T>
float object_list<T>::pdf_value(const Vec3& origin, const Vec3& dir, float time) const {
    float sum = 0;
    for (size_t i = 0; i < count; i++) {
        sum += list[i]->pdf_value(origin, dir, time);
    }
    return sum / count;
}

template <typename T>
Vec3 object_list<T>::pdf_generate(const Vec3& origin, float time) const {
    int i = int(randf() * count);
    return list[i]->pdf_generate(origin, time);
}

template <typename T>
bool object_list<T>::hit(const ray& r, float tmin, float tmax, hit_record *rec) const {

    if (!hasBox || box.hit(r, tmin, tmax)) {
        hit_record cur_rec;
        bool hit = false;
        float closest = tmax;

        for (size_t i = 0; i < count; i++) {

            if (list[i]->hit(r, tmin, closest, &cur_rec)) {
                hit = true;
                closest = cur_rec.t;
                *rec = cur_rec;
            }
        }

        return hit;
    }
    else {
        return false;
    }

}

template <typename T>
object_list<T>::object_list(T* l[], size_t n, float time0, float time1) {

    list = l;
    count = n;

    Vec3 minbb(std::numeric_limits<float>::max(),    std::numeric_limits<float>::max(),    std::numeric_limits<float>::max());
    Vec3 maxbb(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());

    for (size_t i = 0; i < count; i++)
    {
        aabb curbox;
        bool cur_hasBox = list[i]->bounding_box(&curbox, time0, time1);

        if (cur_hasBox) {
            minbb = vmin(minbb, curbox.min);
            maxbb = vmax(maxbb, curbox.max);
        }
        else {
            hasBox = false;
            return;
        }
    }

    box = aabb(minbb, maxbb);
    hasBox = true;
}


///////////////////////////////////
//   BOUNDING VOLUME HIERARCHY   //
///////////////////////////////////

template <typename T>
class bvh_node final : public scene_object {
public:
    scene_object *left;
    scene_object *right;
    aabb box;
    uint8 node_order;
    
    bvh_node(T* list[], size_t n, float time0, float time1);

    bool bounding_box(aabb* b, float time0, float time1) const override {
        *b = box;
        return true;
    }
    bool hit(const ray& r, float tmin, float tmax, hit_record *rec) const override;

    void precompute_node_order() override
    {
        // http://www.codercorner.com/blog/?p=734

        const scene_object* P = left;
        const scene_object* N = right;

        aabb lbox, rbox;
        P->bounding_box(&lbox, 0, 1);
        N->bounding_box(&rbox, 0, 1);

        const Vec3& C0 = lbox.center();
        const Vec3& C1 = rbox.center();

        Vec3 DirPPP = Vec3( 1.0f,  1.0f,  1.0f).normalize();
        Vec3 DirPPN = Vec3( 1.0f,  1.0f, -1.0f).normalize();
        Vec3 DirPNP = Vec3( 1.0f, -1.0f,  1.0f).normalize();
        Vec3 DirPNN = Vec3( 1.0f, -1.0f, -1.0f).normalize();
        Vec3 DirNPP = Vec3(-1.0f,  1.0f,  1.0f).normalize();
        Vec3 DirNPN = Vec3(-1.0f,  1.0f, -1.0f).normalize();
        Vec3 DirNNP = Vec3(-1.0f, -1.0f,  1.0f).normalize();
        Vec3 DirNNN = Vec3(-1.0f, -1.0f, -1.0f).normalize();

        bool bPPP = dot(Vec3(C0 - C1), DirPPP) < 0.0f;
        bool bPPN = dot(Vec3(C0 - C1), DirPPN) < 0.0f;
        bool bPNP = dot(Vec3(C0 - C1), DirPNP) < 0.0f;
        bool bPNN = dot(Vec3(C0 - C1), DirPNN) < 0.0f;
        bool bNPP = dot(Vec3(C0 - C1), DirNPP) < 0.0f;
        bool bNPN = dot(Vec3(C0 - C1), DirNPN) < 0.0f;
        bool bNNP = dot(Vec3(C0 - C1), DirNNP) < 0.0f;
        bool bNNN = dot(Vec3(C0 - C1), DirNNN) < 0.0f;

        uint8 Code = 0;
        if (!bPPP)
            Code |= (1 << 7); // Bit 0: PPP
        if (!bPPN)
            Code |= (1 << 6); // Bit 1: PPN
        if (!bPNP)
            Code |= (1 << 5); // Bit 2: PNP
        if (!bPNN)
            Code |= (1 << 4); // Bit 3: PNN
        if (!bNPP)
            Code |= (1 << 3); // Bit 4: NPP
        if (!bNPN)
            Code |= (1 << 2); // Bit 5: NPN
        if (!bNNP)
            Code |= (1 << 1); // Bit 6: NNP
        if (!bNNN)
            Code |= (1 << 0); // Bit 7: NNN

        node_order = Code;
    }
};

template <typename T>
bool bvh_node<T>::hit(const ray& r, float tmin, float tmax, hit_record *rec) const {

    if (box.hit(r, tmin, tmax)) {

        // sort left/right nodes by which one is closer to the ray, skip farther node if we hit something inside the closer node
        // from http://www.codercorner.com/blog/?p=734
        scene_object* closer;
        scene_object* farther;

        // naive/slow version
        //aabb lbox, rbox;
        //left->bounding_box(&lbox, r.time, r.time);
        //right->bounding_box(&rbox, r.time, r.time);
        //if (dot(Vec3(lbox.center() - rbox.center()), r.dir) < 0.0f) {

        if (node_order & r.dirMask) {
            closer = left;
            farther = right;
        }
        else {
            closer = right;
            farther = left;
        }

        bool hit_closer = closer->hit(r, tmin, tmax, rec);
        if (hit_closer)
            return true;

        bool hit_farther = farther->hit(r, tmin, tmax, rec);

        return hit_closer || hit_farther;
    }
    else {
        return false;
    }
}

template <size_t axis, typename T>
inline int __cdecl box_compare(const void *a, const void *b) {

    aabb box_left, box_right;
    T *a_obj = *(T**) a;
    T *b_obj = *(T**) b;

    MRT_Assert(a_obj->bounding_box(&box_left, 0, 0) && b_obj->bounding_box(&box_right, 0, 0),
               "no bounding box in bvh_node constructor\n");

    float delta = box_left.min[axis] - box_right.min[axis];

    if (delta == 0.0f) { // MSVC explicitly requires this case for qsort, otherwise infinite loop can occur
        return 0;
    }
    else if (delta < 0.0f) {
        return -1;
    }
    else {
        return 1;
    }
}

typedef int(__cdecl *cmp_fn)(const void *a, const void *b);

template <typename T>
inline cmp_fn box_compare_dispatch(size_t axis) {
    switch (axis) {
    case 0: return box_compare<0, T>;
    case 1: return box_compare<1, T>;
    case 2: return box_compare<2, T>;
    default:
        return nullptr;
    }
}

template <typename T>
bvh_node<T>::bvh_node(T* list[], size_t n, float time0, float time1) {

    // TODO: build bbox once, then separate into two each recursion without rebuilding the entire thing from scratch (pass new bbox down)

    // create temporary list object to get bounding box of all objects (lazy coder at work!)
    object_list<T> ol(list, n, time0, time1);
    ol.bounding_box(&box, time0, time1);

    // choose split axis by picking largest extents on bb
    Vec3 dim = box.max - box.min;
    size_t axis = max_dim(dim);

    qsort(list, n, sizeof(T*), box_compare_dispatch<T>(axis));

    if (n == 1) {
        left = right = list[0];
    }
    else if (n == 2) {
        left = list[0];
        right = list[1];
    }
    else if (n < 11) {
        left = new object_list<T>(list, n / 2, time0, time1);
        right = new object_list<T>(list + (n / 2), n - (n / 2), time0, time1);
    }
    else {
        left = new bvh_node(list, n / 2, time0, time1);
        right = new bvh_node(list + (n / 2), n - (n / 2), time0, time1);
    }

    precompute_node_order();

    //aabb box_left, box_right;
    //MRT_Assert(left->bounding_box(&box_left, time0, time1) && right->bounding_box(&box_right, time0, time1),
    //           "no bounding box in bvh_node constructor\n");
    //box = surrounding_box(box_left, box_right);
}

/////////////////////////
//     TRANSLATION     //
/////////////////////////

class translate final : public scene_object {
public:
    scene_object *obj;
    Vec3 offset;

    translate(scene_object *o, const Vec3& displacement) : obj(o), offset(displacement) {}
    bool hit(const ray& r, float tmin, float tmax, hit_record *rec) const override;
    bool bounding_box(aabb *box, float time0, float time1) const override;
};


////////////////////////
//      ROTATION      //
////////////////////////

class rotate_y final : public scene_object {
public:
    scene_object *obj;
    aabb bbox;
    float sin_theta;
    float cos_theta;
    bool hasBox;

    rotate_y(scene_object *o, float angle);

    bool hit(const ray& r, float tmin, float tmax, hit_record *rec) const override;
    bool bounding_box(aabb *box, float time0, float time1) const override {
        *box = bbox;
        return hasBox;
    }
};