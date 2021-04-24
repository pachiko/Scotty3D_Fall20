
#include "../rays/bvh.h"
#include "debug.h"
#include <stack>

namespace PT {
// Number of buckets to use
const size_t nBuckets = 16;

// Bucket
template <typename Primitive>
struct Bucket {
    BBox bbox;
    float max_t;
    std::vector<Primitive*> prims;
};

// Initialize buckets.
template <typename Primitive>
std::vector<Bucket<Primitive>> initBuckets(size_t ax, const std::vector<Primitive> &prims,
const size_t start, const size_t size) {
    // Range of primitives' box
    float min_r = FLT_MAX, max_r = -FLT_MAX;

    // Get range
    for (size_t i = start; i < size; i++) {
        float min = prims[i].bbox().min[int(ax)];
        float max = prims[i].bbox().max[int(ax)];

        if (min < min_r) min_r = min;
        else if (max > max_r) max_r = max;
    }

    // Width of each bucket
    float width = (max_r - min_r) / nBuckets;

    // Set buckets' range
    std::vector<Bucket<Primitive>> buckets; 
    for (size_t i = 0; i < nBuckets; i++) {
        Bucket<Primitive> b;
        b.max_t = min_r + (i + 1)*width;
        buckets.push_back(b);
    }
    
    return buckets;
    // return std::move(buckets); // moving a local object in a return statement prevents copy ellision
}

// Compute index of bucket
template <typename Primitive>
size_t computeBucket(float centroid, const std::vector<Bucket<Primitive>> &buckets) {
    size_t i = 0;
    for (auto buck: buckets) {
        if (centroid < buck.max_t) return i;
        else i++;
    }
    return (i - 1 > 0) ? i - 1 : 0; // unlikely
}

template <typename Primitive>
void BVH<Primitive>::build(std::vector<Primitive> &&prims, size_t max_leaf_size) {
    // NOTE (PathTracer):
    // This BVH is parameterized on the type of the primitive it contains. This allows
    // us to build a BVH over any type that defines a certain interface. Specifically,
    // we use this to both build a BVH over triangles within each Tri_Mesh, and over
    // a variety of Objects (which might be Tri_Meshes, Spheres, etc.) in Pathtracer.
    //
    // The Primitive interface must implement these two functions:
    //      BBox bbox() const;
    //      Trace hit(const Ray& ray) const;
    // Hence, you may call bbox() and hit() on any value of type Primitive.
    //
    // Finally, also note that while a BVH is a tree structure, our BVH nodes don't
    // contain pointers to children, but rather indicies. This is because instead
    // of allocating each node individually, the BVH class contains a vector that
    // holds all of the nodes. Hence, to get the child of a node, you have to
    // look up the child index in this vector (e.g. nodes[node.l]). Similarly,
    // to create a new node, don't allocate one yourself - use BVH::new_node, which
    // returns the index of a newly added node.

    // Keep these
    nodes.clear();
    primitives = std::move(prims);
    // do not dereference prims now. it has no value. rvalue reference is used to transfer ownership
    // without copying. used commonly in explicit constructors

    // TODO (PathTracer): Task 3
    // Construct a BVH from the given vector of primitives and maximum leaf
    // size configuration. The starter code builds a BVH with a
    // single leaf node (which is also the root) that encloses all the
    // primitives.

    // Replace these (I think not?)
    BBox box;
    for (const Primitive &prim : primitives)
        box.enclose(prim.bbox());
    new_node(box, 0, primitives.size(), 0, 0);
    root_idx = 0;

    recursive_build(root_idx, max_leaf_size);
}

// Recursively build BVH
template <typename Primitive> void BVH<Primitive>::recursive_build(const size_t node_idx,
const size_t max_leaf_size) {
    // Recursively build BVH
    Node* n = &nodes[node_idx];

    // Keep track of min-cost partition and it's data for the current node
    float minCost = FLT_MAX;
    BBox leftBox, rightBox;
    std::vector<Primitive*> leftPrims, rightPrims;

    // Find min cost amongst all partitions, all planes
    for (size_t axis = 0; axis < 3; axis++) {
        std::vector<Bucket<Primitive>> buckets = initBuckets(axis, primitives, n->start, n->size);

        // Bucket the primitives
        for (size_t p = n->start; p < n->start + n->size; p++) {
            Primitive* prim = &primitives[p];
            float centroid = (prim->bbox().min[int(axis)] + prim->bbox().max[int(axis)]) / 2.f;
            size_t b = computeBucket(centroid, buckets);
            buckets[b].bbox.enclose(prim->bbox());
            buckets[b].prims.push_back(prim);
        }

        // Partition B - 1 times
        for (size_t b = 0; b < nBuckets - 1; b++) {
            std::vector<Primitive*> lp , rp; // Left and Right Primitive Vectors
            BBox lb, rb; // Left and Right Boxes

            for (size_t i = 0; i <= b; i++) { // Left Side
                lb.enclose(buckets[i].bbox);
                lp.insert(lp.end(), buckets[i].prims.begin(), buckets[i].prims.end());
            }

            for (size_t i = b + 1; i < nBuckets; i++) { // Right Side
                rb.enclose(buckets[i].bbox);
                rp.insert(rp.end(), buckets[i].prims.begin(), buckets[i].prims.end());
            }

            // Evaluate cost
            float cost = lb.surface_area()/n->bbox.surface_area()*lp.size() +
             rb.surface_area()/n->bbox.surface_area()*rp.size();
            
            // Update minimum
            if (cost < minCost) {
                leftBox = std::move(lb);
                rightBox = std::move(rb);
                leftPrims = std::move(lp);
                rightPrims = std::move(rp);
                cost = minCost;
            }

            // clear!
            lp.clear();
            rp.clear();
        }
        // clear !
        buckets.clear();
    }

    // Tie-breaker
    bool recalcBound = false;
    if (leftPrims.size() == 0) {
        leftPrims.assign(rightPrims.begin(), rightPrims.begin() + rightPrims.size()/2);
        rightPrims.erase(rightPrims.begin(), rightPrims.begin() + rightPrims.size()/2);
        recalcBound = true;
    } else if (rightPrims.size() == 0) {
        rightPrims.assign(leftPrims.begin(), leftPrims.begin() + leftPrims.size()/2);
        leftPrims.erase(leftPrims.begin(), leftPrims.begin() + leftPrims.size()/2);
        recalcBound = true;
    }
    if (recalcBound) {
        BBox new_rb, new_lb;
        for (size_t i = 0; i < rightPrims.size(); i++) {
            new_rb.enclose((*rightPrims[i]).bbox());
        }
        for (size_t i = 0; i < leftPrims.size(); i++) {
            new_lb.enclose((*leftPrims[i]).bbox()); 
        }
        rightBox = std::move(new_rb);
        leftBox = std::move(new_lb);
    }

    // Create child nodes using best partition; Assign to current node
    size_t leftChild = new_node(leftBox, n->start, leftPrims.size(), 0, 0);
    n = &nodes[node_idx];
    size_t rightChild = new_node(rightBox, n->start + leftPrims.size(), rightPrims.size(), 0, 0);
    n = &nodes[node_idx];
    n->l = leftChild;
    n->r = rightChild;

    // Rearrange primitives
    std::vector<Primitive> temp_prims;
    for (size_t i = 0; i < leftPrims.size(); i++) {
        temp_prims.push_back(std::move(*leftPrims[i]));
    }
    for (size_t i = 0; i < rightPrims.size(); i++) {
        temp_prims.push_back(std::move(*rightPrims[i]));
    }

    // Write back primitives
    std::move(temp_prims.begin(), temp_prims.begin() + temp_prims.size(), primitives.begin() + n->start);

    // Clear!
    temp_prims.clear();

    // recurse
    if (leftPrims.size() > max_leaf_size)  {
        leftPrims.clear();
        recursive_build(leftChild, max_leaf_size);
    }
    if (rightPrims.size() > max_leaf_size) {
        rightPrims.clear();
        recursive_build(rightChild, max_leaf_size);
    }
}

template <typename Primitive> Trace BVH<Primitive>::hit(const Ray &ray) const {

    // TODO (PathTracer): Task 3
    // Implement ray - BVH intersection test. A ray intersects
    // with a BVH aggregate if and only if it intersects a primitive in
    // the BVH that is not an aggregate.

    // The starter code simply iterates through all the primitives.
    // Again, remember you can use hit() on any Primitive value.

    Trace ret;
    ret.time = FLT_MAX;
    find_closest_hit(ray, 0, ret);
    return ret;
}

template <typename Primitive> void BVH<Primitive>::find_closest_hit(const Ray &ray, const size_t nodeIdx,
Trace &closest) const {
    Node n = nodes[nodeIdx];
    Vec2 times = Vec2(ray.time_bounds.x, ray.time_bounds.y);
    bool hitBox = n.bbox.hit(ray, times);
    if (!hitBox || (times.x > closest.time)) return;

    if (n.is_leaf()) {
        for (size_t i = n.start; i < n.start + n.size; i++) {
            Trace trc = primitives[i].hit(ray);
            closest = Trace::min(closest, trc);
            // If neither hits, Trace::min() returns a default Trace. Set the time again.
            if (closest.time == 0.f && !closest.hit) closest.time = FLT_MAX;
        }
    } else {
        // Times for left and right child node bbox hits
        Vec2 timeL = Vec2(ray.time_bounds.x, ray.time_bounds.y);
        Vec2 timeR = Vec2(ray.time_bounds.x, ray.time_bounds.y);

        // Don't really need the booleans
        nodes[n.l].bbox.hit(ray, timeL);
        nodes[n.r].bbox.hit(ray, timeR);

        // Determine which child node was hit first
        bool isLeftFirst = timeL.x <= timeR.x;
        size_t firstIdx = (isLeftFirst) ? n.l : n.r;
        size_t secondIdx = (isLeftFirst) ? n.r : n.l;
        float secondHitTime = (isLeftFirst) ? timeR.x : timeL.x;

        // Try nearer node first. Then try further node in-case
        find_closest_hit(ray, firstIdx, closest);
        if (secondHitTime < closest.time) find_closest_hit(ray, secondIdx, closest);
    }
}


template <typename Primitive>
BVH<Primitive>::BVH(std::vector<Primitive> &&prims, size_t max_leaf_size) {
    // Dont think anybody calls this constructor
    build(std::move(prims), max_leaf_size); // transfer ownership to build()
}

template <typename Primitive> bool BVH<Primitive>::Node::is_leaf() const {
    return l == 0 && r == 0;
}

template <typename Primitive>
size_t BVH<Primitive>::new_node(BBox box, size_t start, size_t size, size_t l, size_t r) {
    Node n;
    n.bbox = box;
    n.start = start;
    n.size = size;
    n.l = l;
    n.r = r;
    nodes.push_back(n);
    return nodes.size() - 1;
}

template <typename Primitive> BBox BVH<Primitive>::bbox() const { return nodes[root_idx].bbox; }

template <typename Primitive> std::vector<Primitive> BVH<Primitive>::destructure() {
    nodes.clear();
    return std::move(primitives);
}

template <typename Primitive> void BVH<Primitive>::clear() {
    nodes.clear();
    primitives.clear();
}

template <typename Primitive>
size_t BVH<Primitive>::visualize(GL::Lines &lines, GL::Lines &active, size_t level,
                                 const Mat4 &trans) const {

    std::stack<std::pair<size_t, size_t>> tstack;
    tstack.push({root_idx, 0});
    size_t max_level = 0;

    if (nodes.empty())
        return max_level;

    while (!tstack.empty()) {

        auto [idx, lvl] = tstack.top();
        max_level = std::max(max_level, lvl);
        const Node &node = nodes[idx];
        tstack.pop();

        Vec3 color = lvl == level ? Vec3(1.0f, 0.0f, 0.0f) : Vec3(1.0f);
        GL::Lines &add = lvl == level ? active : lines;

        BBox box = node.bbox;
        box.transform(trans);
        Vec3 min = box.min, max = box.max;

        auto edge = [&](Vec3 a, Vec3 b) { add.add(a, b, color); };

        edge(min, Vec3{max.x, min.y, min.z});
        edge(min, Vec3{min.x, max.y, min.z});
        edge(min, Vec3{min.x, min.y, max.z});
        edge(max, Vec3{min.x, max.y, max.z});
        edge(max, Vec3{max.x, min.y, max.z});
        edge(max, Vec3{max.x, max.y, min.z});
        edge(Vec3{min.x, max.y, min.z}, Vec3{max.x, max.y, min.z});
        edge(Vec3{min.x, max.y, min.z}, Vec3{min.x, max.y, max.z});
        edge(Vec3{min.x, min.y, max.z}, Vec3{max.x, min.y, max.z});
        edge(Vec3{min.x, min.y, max.z}, Vec3{min.x, max.y, max.z});
        edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, max.y, min.z});
        edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, min.y, max.z});

        if (node.l)
            tstack.push({node.l, lvl + 1});
        if (node.r)
            tstack.push({node.r, lvl + 1});

        if (!node.l && !node.r) {
            for (size_t i = node.start; i < node.start + node.size; i++) {
                size_t c = primitives[i].visualize(lines, active, level - lvl, trans);
                max_level = std::max(c, max_level);
            }
        }
    }
    return max_level;
}

} // namespace PT
