#pragma once

#include "scene_object.h"
#include "vec3.h"
#include <memory>
#include <vector>
#include <type_traits>

#ifdef NEW_INTERSECT
//#define BACKFACE_CULLING
#endif

struct triangle {
#ifdef NEW_INTERSECT
    Vec3 a, b, c;
    Vec3 an, bn, cn;
#else
    Vec3 m, u, v;
    Vec3 mn, un, vn;
#endif
    material* mat_ptr; // TODO: right now we can only really have one material per mesh, so we could move this outside into the BVH

    triangle() = default;

    triangle(const Vec3& a, const Vec3& b, const Vec3& c, material* mat);
    triangle(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& an, const Vec3& bn, const Vec3& cn, material* mat);

    bool hit(const ray& r, float tmin, float tmax, hit_record* rec) const;
    bool bounding_box(aabb* box, float time0, float time1) const;

    Vec3 get_centroid() const {
        return (m + (m + u) + (m + v)) * (1.0f / 3.0f);
    }
    void add_to_box(aabb* box) const {
        box->min = vmin(box->min, m);
        box->min = vmin(box->min, m + u);
        box->min = vmin(box->min, m + v);
        box->max = vmax(box->max, m);
        box->max = vmax(box->max, m + u);
        box->max = vmax(box->max, m + v);
    }
};

// the following code is based on https://jacco.ompf2.com/2022/04/13/how-to-build-a-bvh-part-1-basics/

struct pod_bvh_node {
    aabb box;
    uint32 left;
    //uint32 right; // implicit, is always left+1
    uint32 prim_offset;
    uint32 prim_count;
    uint8 node_order;
    bool is_leaf() const {
        return prim_count != 0;
    }
};

template<typename T>
class pod_bvh final : public scene_object {
    std::unique_ptr<T[]> prims;
    std::unique_ptr<pod_bvh_node[]> nodes;
    std::unique_ptr<Vec3[]> centroids;
    uint32 prim_count;
    uint32 node_count;
    uint32 root_node = 0;
public:
    pod_bvh(T list[], size_t n, float time0, float time1);
    bool hit(const ray& r, float tmin, float tmax, hit_record* rec) const override;
    bool hit(uint32 node_index, const ray& r, float tmin, float tmax, hit_record* rec) const;
    bool bounding_box(aabb* box, float time0, float time1) const override;
    void update_node_box(uint32 node_index);
    void subdivide(uint32 node_index);
    void precompute_node_order(uint32 node_index);
};


template<typename T>
inline pod_bvh<T>::pod_bvh(T list[], size_t n, float time0, float time1)
{
    prim_count = (uint32)n;
    
    nodes = std::make_unique_for_overwrite<pod_bvh_node[]>(n*2 - 1); // worst case allocation
    prims = std::make_unique_for_overwrite<T[]>(n);
    static_assert(std::is_trivially_copyable_v<T>);
    memcpy(prims.get(), list, n * sizeof(T)); // TODO: we can eliminate this copy later
    centroids = std::make_unique_for_overwrite<Vec3[]>(n);

    for (int i = 0; i < n; i++) {
        centroids[i] = list[i].get_centroid();
    }

    node_count = 1;
    auto& root = nodes[root_node];
    root.left = 0;
    //root.right = 0;
    root.prim_offset = 0;
    root.prim_count = prim_count;
    update_node_box(root_node);

    subdivide(root_node);
}

template<typename T>
inline void pod_bvh<T>::subdivide(uint32 node_index)
{
    auto& node = nodes[node_index];
    if (node.prim_count <= 2) return;

    // split pos is largest extent
    Vec3 extent2 = node.box.max - node.box.min;
    int axis = 0;
    if (extent2.y > extent2.x) axis = 1;
    if (extent2.z > extent2[axis]) axis = 2;
    float splitPos = node.box.min[axis] + extent2[axis] * 0.5f;

    // partition list by split pos
    int i = node.prim_offset;
    int j = i + node.prim_count - 1;
    while (i <= j)
    {
        if (centroids[i][axis] < splitPos)
            i++;
        else {
            std::swap(prims[i], prims[j]);
            std::swap(centroids[i], centroids[j]);
            j--;
        }
    }

    int left_count = i - node.prim_offset;
    if (left_count == 0 || left_count == (int)node.prim_count) return;

    // create child nodes
    int left_child_index = node_count++;
    int right_child_index = node_count++;
    nodes[left_child_index].prim_offset = node.prim_offset;
    nodes[left_child_index].prim_count = left_count;
    nodes[right_child_index].prim_offset = i;
    nodes[right_child_index].prim_count = node.prim_count - left_count;
    update_node_box(left_child_index);
    update_node_box(right_child_index);

    // finalize parent node
    node.left = left_child_index;
    //node.right = right_child_index; // always left+1
    node.prim_count = 0;
    precompute_node_order(node_index);

    // recurse
    subdivide(left_child_index);
    subdivide(right_child_index);
}

template<typename T>
inline void pod_bvh<T>::update_node_box(uint32 node_index)
{
    auto& node = nodes[node_index];

    constexpr float maxf = std::numeric_limits<float>::max();
    constexpr float minf = std::numeric_limits<float>::min();
    node.box = aabb(Vec3(maxf, maxf, maxf), Vec3(minf, minf, minf));

    for (size_t i = 0; i < node.prim_count; i++)
    {
        const T& leaf_prim = prims[node.prim_offset + i];
        leaf_prim.add_to_box(&node.box);
    }
}


template<typename T>
inline bool pod_bvh<T>::hit(uint32 node_index, const ray& r, float tmin, float tmax, hit_record* rec) const
{
    const auto& node = nodes[node_index];

    if (!node.box.hit(r, tmin, tmax)) return false;

    bool has_hit = false;
    if (node.is_leaf())
    {
        for (uint32 i = 0; i < node.prim_count; i++) {
            if (prims[node.prim_offset + i].hit(r, tmin, tmax, rec)) {
                has_hit = true;
                tmax = rec->t;
            }
        }
    }
    else
    {
        // sort left/right nodes by which one is closer to the ray, skip farther node if we hit something inside the closer node
        // from http://www.codercorner.com/blog/?p=734
        uint32 closer = 0;
        uint32 farther = 0;

        if (node.node_order & r.dirMask) {
            closer = node.left;
            farther = node.left + 1;
        }
        else {
            closer = node.left + 1;
            farther = node.left;
        }

        bool hit_closer = hit(closer, r, tmin, tmax, rec);
        if (hit_closer)
            return true;

        bool hit_farther = hit(farther, r, tmin, tmax, rec);

        return hit_closer || hit_farther;
    }
    return has_hit;
}

// TODO: is this okay?
static thread_local std::vector<const pod_bvh_node*> traversal_stack;

template<typename T>
inline bool pod_bvh<T>::hit(const ray& r, float tmin, float tmax, hit_record* rec) const
{
    return hit(root_node, r, tmin, tmax, rec);

    // TODO: the following is slightly slower than recursion, investigate

    //traversal_stack.clear();
    //traversal_stack.push_back(&nodes[root_node]);

    //bool has_hit = false;

    //do
    //{
    //    const pod_bvh_node* curnode = traversal_stack.back();
    //    traversal_stack.pop_back();

    //    if (!curnode->box.hit(r, tmin, tmax))
    //        continue;

    //    if (curnode->is_leaf())
    //    {
    //        for (size_t i = 0; i < curnode->prim_count; i++)
    //        {
    //            if (prims[curnode->prim_offset + i].hit(r, tmin, tmax, rec)) {
    //                has_hit = true;
    //                tmax = rec->t;
    //            }
    //        }
    //        if (has_hit)
    //            return true;
    //    }
    //    else {
    //        // sort left/right nodes by which one is closer to the ray, skip farther node if we hit something inside the closer node
    //        // from http://www.codercorner.com/blog/?p=734
    //        const pod_bvh_node* closer = nullptr;
    //        const pod_bvh_node* farther = nullptr;

    //        if (curnode->node_order & r.dirMask) {
    //            closer = &nodes[curnode->left];
    //            farther = &nodes[curnode->left + 1];
    //        }
    //        else {
    //            closer = &nodes[curnode->left + 1];
    //            farther = &nodes[curnode->left];
    //        }

    //        traversal_stack.push_back(farther);
    //        traversal_stack.push_back(closer);
    //    }

    //} while (!traversal_stack.empty());

    //return has_hit;
}

template<typename T>
inline bool pod_bvh<T>::bounding_box(aabb* box, float time0, float time1) const
{
    *box = nodes[0].box;
    return true;
}

// http://www.codercorner.com/blog/?p=734
template<typename T>
inline void pod_bvh<T>::precompute_node_order(uint32 node_index)
{
    auto& node = nodes[node_index];

    const aabb& lbox = nodes[node.left].box;
    const aabb& rbox = nodes[node.left+1].box;

    const Vec3& C0 = lbox.center();
    const Vec3& C1 = rbox.center();

    const Vec3 DirPPP = Vec3(1.0f, 1.0f, 1.0f).normalize();
    const Vec3 DirPPN = Vec3(1.0f, 1.0f, -1.0f).normalize();
    const Vec3 DirPNP = Vec3(1.0f, -1.0f, 1.0f).normalize();
    const Vec3 DirPNN = Vec3(1.0f, -1.0f, -1.0f).normalize();
    const Vec3 DirNPP = Vec3(-1.0f, 1.0f, 1.0f).normalize();
    const Vec3 DirNPN = Vec3(-1.0f, 1.0f, -1.0f).normalize();
    const Vec3 DirNNP = Vec3(-1.0f, -1.0f, 1.0f).normalize();
    const Vec3 DirNNN = Vec3(-1.0f, -1.0f, -1.0f).normalize();

    bool bPPP = dot(Vec3(C0 - C1), DirPPP) < 0.0f;
    bool bPPN = dot(Vec3(C0 - C1), DirPPN) < 0.0f;
    bool bPNP = dot(Vec3(C0 - C1), DirPNP) < 0.0f;
    bool bPNN = dot(Vec3(C0 - C1), DirPNN) < 0.0f;
    bool bNPP = dot(Vec3(C0 - C1), DirNPP) < 0.0f;
    bool bNPN = dot(Vec3(C0 - C1), DirNPN) < 0.0f;
    bool bNNP = dot(Vec3(C0 - C1), DirNNP) < 0.0f;
    bool bNNN = dot(Vec3(C0 - C1), DirNNN) < 0.0f;

    uint8 Code = 0;
    Code |= (!bPPP << 7); // Bit 0: PPP
    Code |= (!bPPN << 6); // Bit 1: PPN
    Code |= (!bPNP << 5); // Bit 2: PNP
    Code |= (!bPNN << 4); // Bit 3: PNN
    Code |= (!bNPP << 3); // Bit 4: NPP
    Code |= (!bNPN << 2); // Bit 5: NPN
    Code |= (!bNNP << 1); // Bit 6: NNP
    Code |= (!bNNN << 0); // Bit 7: NNN

    node.node_order = Code;
}



class triangle_scene_object final : public scene_object {
#ifdef NEW_INTERSECT
    Vec3 a, b, c;
    Vec3 an, bn, cn;
#else
    Vec3 m, u, v;
    Vec3 mn, un, vn;
#endif
    material* mat_ptr; // TODO: right now we can only really have one material per mesh, so we could move this outside into the BVH
public:
    triangle_scene_object(const Vec3& a, const Vec3& b, const Vec3& c, material* mat);
    triangle_scene_object(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& an, const Vec3& bn, const Vec3& cn, material* mat);

    bool hit(const ray& r, float tmin, float tmax, hit_record* rec) const;
    bool bounding_box(aabb* box, float time0, float time1) const;
};
