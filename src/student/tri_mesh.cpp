
#include "../rays/tri_mesh.h"
#include "debug.h"

namespace PT {

BBox Triangle::bbox() const {

    // TODO (PathTracer): Task 2
    // compute the bounding box of the triangle
    Vec3 p0 = vertex_list[v0].position;
    Vec3 p1 = vertex_list[v1].position;
    Vec3 p2 = vertex_list[v2].position;
    
    float xmin,ymin,zmin,xmax,ymax,zmax;

    xmin = std::min(p0.x, std::min(p1.x, p2.x));
    ymin = std::min(p0.y, std::min(p1.y, p2.y));
    zmin = std::min(p0.z, std::min(p1.z, p2.z));

    xmax = std::max(p0.x, std::max(p1.x, p2.x));
    ymax = std::max(p0.y, std::max(p1.y, p2.y));
    zmax = std::max(p0.z, std::max(p1.z, p2.z));
    return BBox(Vec3(xmin, ymin, zmin), Vec3(xmax, ymax, zmax));
}

Trace Triangle::hit(const Ray &ray) const {

    // Vertices of triangle - has position and surface normal
    Tri_Mesh_Vert v_0 = vertex_list[v0];
    Tri_Mesh_Vert v_1 = vertex_list[v1];
    Tri_Mesh_Vert v_2 = vertex_list[v2];

    // TODO (PathTracer): Task 2
    // Intersect this ray with a triangle defined by the three above points.
    Vec3 e1 = v_1.position - v_0.position; // p1 - p0
    Vec3 e2 = v_2.position - v_0.position; // p2 - p0
    Vec3 s = ray.point - v_0.position; // o - p0

    Vec3 sXnd = cross(s, -ray.dir); // (o - p0) X (-d)
    Vec3 e1Xe2 = cross(e1, e2); // (p1 - p0) X (p2 - p0)

    // Cramer's Rule: A^-1 = 1/det(A)*adj(A)
    float det = dot(e1Xe2, -ray.dir); // det([a b c]) = (a x b).c
    if (det == 0) {Trace ret; ret.hit = false; return ret;}
    // adj(A) y = |y b c|, |a y c|, |a b y|
    float u = -dot(sXnd, e2)/det;
    float v = dot(sXnd, e1)/det;
    float t = dot(e1Xe2, s)/det;

    Trace ret;
    // was there an intersection?
    ret.hit = (u >= 0) && (u <= 1) && (v >= 0) && (v <= 1) && (t <= ray.time_bounds.y) && (t >= ray.time_bounds.x);
    if (!ret.hit) return ret;
    ret.time = t; // at what time did the intersection occur?
    ray.time_bounds.y = t;
    ret.position = ray.at(t); // where was the intersection?
    // what was the surface normal at the intersection?
    // (this should be interpolated between the three vertex normals)
    ret.normal = (1 - u - v)*v_0.normal + u*v_1.normal + v*v_2.normal;
    return ret;
}

Triangle::Triangle(Tri_Mesh_Vert *verts, unsigned int v0, unsigned int v1, unsigned int v2)
    : vertex_list(verts), v0(v0), v1(v1), v2(v2) {}

void Tri_Mesh::build(const GL::Mesh &mesh) {

    verts.clear();
    triangles.clear();

    for (const auto &v : mesh.verts()) {
        verts.push_back({v.pos, v.norm});
    }

    const auto &idxs = mesh.indices();

    std::vector<Triangle> tris;
    for (size_t i = 0; i < idxs.size(); i += 3) {
        tris.push_back(Triangle(verts.data(), idxs[i], idxs[i + 1], idxs[i + 2]));
    }

    triangles.build(std::move(tris), 4);
}

Tri_Mesh::Tri_Mesh(const GL::Mesh &mesh, bool flip) { build(mesh); flip_normals = flip; }

BBox Tri_Mesh::bbox() const { return triangles.bbox(); }

Trace Tri_Mesh::hit(const Ray &ray) const { 
    Trace t = triangles.hit(ray); 
    if(flip_normals) t.normal = -t.normal;
    return t;
}

size_t Tri_Mesh::visualize(GL::Lines &lines, GL::Lines &active, size_t level,
                           const Mat4 &trans) const {
    return triangles.visualize(lines, active, level, trans);
}

} // namespace PT
