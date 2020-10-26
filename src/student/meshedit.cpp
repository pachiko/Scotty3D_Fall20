
#include <queue>
#include <set>
#include <unordered_map>
#include <iostream>
#include "../geometry/halfedge.h"
#include "debug.h"

/* Note on local operation return types:

    The local operations all return a std::optional<T> type. This is used so that your
    implementation can signify that it does not want to perform the operation for
    whatever reason (e.g. you don't want to allow the user to erase the last vertex).

    An optional can have two values: std::nullopt, or a value of the type it is
    parameterized on. In this way, it's similar to a pointer, but has two advantages:
    the value it holds need not be allocated elsewhere, and it provides an API that
    forces the user to check if it is null before using the value.

    In your implementaiton, if you have successfully performed the operation, you can
    simply return the required reference:

            ... collapse the edge ...
            return collapsed_vertex_ref;

    And if you wish to deny the operation, you can return the null optional:

            return std::nullopt;

    Note that the stubs below all reject their duties by returning the null optional.
*/

/*
    This method should replace the given vertex and all its neighboring
    edges and faces with a single face, returning the new face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_vertex(Halfedge_Mesh::VertexRef v) {

    (void)v;
    return std::nullopt;
}

/*
    This method should erase the given edge and return an iterator to the
    merged face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_edge(Halfedge_Mesh::EdgeRef e) {

    (void)e;
    return std::nullopt;
}

/*
    This method should collapse the given edge and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_edge(Halfedge_Mesh::EdgeRef e0) {
    if (e0->on_boundary()) {
        (void) e0;
        return std::nullopt;
    }

    // calculate new vertex pos
    HalfedgeRef h0 = e0->halfedge();
    HalfedgeRef h3 = h0->twin();
    VertexRef v0 = h0->vertex();
    v0->pos = (v0->pos + h3->vertex()->pos)/2;

    // get face degree first. This changes on the fly!
    unsigned int f0d = h0->face()->degree();
    unsigned int f3d = h3->face()->degree();

    // Keep v0 and remove v1. All halfedges pointing to v1 are changed to point to v0
    HalfedgeRef h = h0->next();
    while(h != h3) {
        h->vertex() = v0;
        h = h->twin()->next();
    }

    // Reassign elements
    modify_face(h0, f0d, v0);
    modify_face(h3, f3d);

    // Deallocate
    erase(h3->vertex());
    erase(e0);
    erase(h0);
    erase(h3);

    if (f0d == 3) 
        remove_triangle(h0);
    if (f3d == 3) 
        remove_triangle(h3);
    
    return v0;
}

/*
    This method modifies elements in a face (halfedges, vertices, edges) depending on the face degree.
    h: Inner halfedge on the edge to be collapsed.
    deg: Degree of the face
    v: vertex to be kept after collapsing edge. Needs to set its halfedge()
*/
void Halfedge_Mesh::modify_face(Halfedge_Mesh::HalfedgeRef h, unsigned int deg,
std::optional<Halfedge_Mesh::VertexRef> v) {
    // triangle deletes all its halfedges.
    // make the twins of the two halfedges adjacent to the triangle point to one another
    if (deg == 3) {
        // swap twins
        HalfedgeRef h1 = h->next()->twin(); // twin of top halfedge
        HalfedgeRef h2 = h->next()->next()->twin(); // twin of bottom halfedge
        h1->twin() = h2;
        h2->twin() = h1;
        // one edge gets deleted (in this case, the bottom).
        h2->edge() = h1->edge();
        // the remaining edge and vertices should point to a valid halfedge
        h1->vertex()->halfedge() = h1;
        h1->edge()->halfedge() = h1;
        if (v != std::nullopt)
            v.value()->halfedge() = h2; // only done once
    } else {
        // for non-triangles, join the broken halfedge chain.
        HalfedgeRef hh = h;
        while(hh->next() != h) hh = hh->next();
        hh->next() = h->next();
        // face needs to point to a valid halfedge too.
        hh->face()->halfedge() = hh;
        if (v != std::nullopt)
            v.value()->halfedge() = h->next(); // only done once
    }
}

/*
    This method should removes the halfedges, face and one edge.
*/
void Halfedge_Mesh::remove_triangle(Halfedge_Mesh::HalfedgeRef h) {
    erase(h->face());
    erase(h->next()->next()->edge());
    erase(h->next());
    erase(h->next()->next());
}

/*
    This method should collapse the given face and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_face(Halfedge_Mesh::FaceRef f) {

    (void)f;
    return std::nullopt;
}

/*
    This method should flip the given edge and return an iterator to the
    flipped edge.
*/
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::flip_edge(Halfedge_Mesh::EdgeRef e0) {
    if (e0->on_boundary()) {
        (void) e0;
        return std::nullopt;
    }
    
    // Get halfedges of edge
    HalfedgeRef h0 = e0->halfedge();
    HalfedgeRef h3 = h0->twin();
    
    // Keep track of old first edges' fields
    VertexRef v1 = h0->next()->vertex(); // h1->vertex() = v1
    EdgeRef e4 = h0->next()->edge(); // h1->edge() = e4
    HalfedgeRef h6 =  h0->next()->twin(); // h1->twin() = h6

    VertexRef v0 = h3->next()->vertex(); // h4->vertex = v0
    EdgeRef e2 = h3->next()->edge(); // h4->edge() = e2
    HalfedgeRef h8 = h3->next()->twin(); // h4->twin() = h8

    /*
    // check if either vertex of e0 forms a face with the other flip vertices
    // not good
        // check if either vertex of e0 forms a face with the other flip vertices
    for (unsigned int i = 0; i < h6->face()->degree() - 1; i++)
        h6 = h6->next();

    if (h6->vertex() == h3->next()->next()->vertex()) {
        (void) e0;
        return std::nullopt;
    }
     h6 = h6->next();

    for (unsigned int i = 0; i < h8->face()->degree() - 1; i++)
        h8 = h8->next();

    if (h8->vertex() == h0->next()->next()->vertex()) {
        (void) e0;
        return std::nullopt;
    }
     h8 = h8->next();
     
    if (h6->next()->next()->vertex() == h3->next()->next()->vertex() ||
    h8->next()->next()->vertex() == h0->next()->next()->vertex()) {
        (void) e0;
        return std::nullopt;
    }
    */

    // get flip vertices, always the 2nd regardless of face's degree (n-gon)
    h0->vertex() = h3->next()->next()->vertex(); // v2
    h3->vertex() = h0->next()->next()->vertex(); // v3

    // rotate CCW inner halfedges not on e0. cycle vertix, twin, edge. Also make twin & edge refer correctly
    while(true) {
        h0 = h0->next(); // h1 = h0->next
        if (h0->next() != e0->halfedge()) { // not the last halfedge
            cycle_half_edge(h0);
        } else {
            cycle_half_edge(h0, v0, h8, e2);
            break;
        }
    }

    while(true) {
        h3 = h3->next(); // h1 = h0->next
        if (h3->next() != e0->halfedge()->twin()) { // not the last halfedge
            cycle_half_edge(h3);
        } else {
            cycle_half_edge(h3, v1, h6, e4);
            break;
        }
    }

    return e0;
}

/*
    Cycle an inner half-edge with it's next(). This changes the current halfedge's vertex, twin and edge.
*/
void Halfedge_Mesh::cycle_half_edge(HalfedgeRef h, std::optional<VertexRef> v,
    std::optional<HalfedgeRef> tw, std::optional<EdgeRef> e) {
        if (v == std::nullopt)
            v = h->next()->vertex();
        h->vertex() = v.value();
        v.value()->halfedge() = h;

        if (tw == std::nullopt)
            tw = h->next()->twin();
        h->twin() = tw.value();
        tw.value()->twin() = h;

        if (e == std::nullopt)
            e = h->next()->edge();
        h->edge() = e.value();
        e.value()->halfedge() = h;

        return;
}

/*
    This method should split the given edge and return an iterator to the
    newly inserted vertex. The halfedge of this vertex should point along
    the edge that was split, rather than the new edges.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::split_edge(Halfedge_Mesh::EdgeRef e0) {
    // TODO: Allow on boundary splitting. The new vertex connects to 3 edges instead.

    // Triangle meshes only!
    if (e0->on_boundary() || e0->halfedge()->face()->degree() != 3 || e0->halfedge()->twin()->face()->degree() != 3) {
        (void) e0;
        return std::nullopt;
    }  

    // Initialize new vertex
    HalfedgeRef h0 = e0->halfedge();
    HalfedgeRef h3 = h0->twin();
    VertexRef v4 = new_vertex();
    v4->pos = (h0->vertex()->pos + h3->vertex()->pos)/2;

    // Save the second halfedges
    HalfedgeRef h1 = h0->next();
    HalfedgeRef h4 = h3->next();

    EdgeRef e6 = divide_face(h0, v4, false).value();
    EdgeRef e7 = divide_face(h3, v4, true).value();
    divide_face(h3->edge(), h1, e6, v4);
    divide_face(e0, h4, e7, v4);
    v4->halfedge() = h0->twin(); // h15
    return v4;
}

/*
    Returns a new edge by dividing a triangle into top and bottom.
    This deals with the bottom-left and top-right triangles.
    h: existing halfedge on the splitting edge (h0, h3)
    v4: Newly created vertex when splitting edge.
*/
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::divide_face(Halfedge_Mesh::HalfedgeRef h,
Halfedge_Mesh::VertexRef v4, bool alloc_first_edge) {
    FaceRef f = h->face(); // f0, f1
    f->halfedge() = h;

    if (alloc_first_edge) { // only e5
        EdgeRef e5 = new_edge();
        h->edge() = e5;
        e5->halfedge() = h;
    }

    EdgeRef e1 = new_edge(); // e6, e7
    HalfedgeRef h2 = h->next()->next(); // h2, h5
    h->next() = new_halfedge(); // h10, h11; remember to save h1 and h4 before this!
    e1->halfedge() = h->next(); 

    h->next()->edge() = e1;
    h->next()->face() = f;
    h->next()->next() = h2;
    h->next()->vertex() = v4;
    // need to allocate twin of new halfedge. Done later when twin is created.

    return e1;
}

/*
    Returns a new edge by dividing a triangle into top and bottom.
    This deals with the top-left and bottom-right triangles.
    e: First Edge (e0, e5)
    h1: Second Halfedge (h1, h4)
    e2: Last Edge (e6, e7)
    v4: Newly created vertex when splitting edge.
*/
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::divide_face(Halfedge_Mesh::EdgeRef e,
Halfedge_Mesh::HalfedgeRef h1, Halfedge_Mesh::EdgeRef e2, Halfedge_Mesh::VertexRef v4) {
    FaceRef f = new_face(); // f2, f3
    HalfedgeRef h = new_halfedge(); // h13, h15
    h->face() = f;
    f->halfedge() = h;

    h->twin() = e->halfedge(); // h0, h3
    e->halfedge()->twin() = h;
    h->edge() = e; // e0, e5
    h->vertex() = v4;
    h->next() = h1; // h1, h4. Hope you saved them earlier!
    
    h1->face() = f;
    h1->next() = new_halfedge(); // h12, h14

    h1->next()->twin() = e2->halfedge(); // h10, h11
    h1->next()->edge() = e2; // e6, e7
    e2->halfedge()->twin() = h1->next();
    h1->next()->next() = h; // h13, h15
    h1->next()->face() = f;
    h1->next()->vertex() = h1->twin()->vertex();

    return std::nullopt;
}

/* Note on the beveling process:

    Each of the bevel_vertex, bevel_edge, and bevel_face functions do not represent
    a full bevel operation. Instead, they should only update the _connectivity_ of
    the mesh, _not_ the positions of newly created vertices. In fact, you should set
    the positions of new vertices to be exactly the same as wherever they "started from."

    When you click on a mesh element while in bevel mode, one of those three functions
    is called. But, because you may then adjust the distance/offset of the newly
    beveled face, we need another method of updating the positions of the new vertices.

    This is where bevel_vertex_positions, bevel_edge_positions, and
    bevel_face_positions come in: these functions are called repeatedly as you
    move your mouse, the position of which determins the normal and tangent offset
    parameters. These functions are also passed an array of the original vertex
    positions: for  bevel_vertex, it has one element, the original vertex position,
    for bevel_edge,  two for the two vertices, and for bevel_face, it has the original
    position of each vertex in halfedge order. You should use these positions, as well
    as the normal and tangent offset fields to assign positions to the new vertices.

    Finally, note that the normal and tangent offsets are not relative values - you
    should compute a particular new position from them, not a delta to apply.
*/

/*
    This method should replace the vertex v with a face, corresponding to
    a bevel operation. It should return the new face.  NOTE: This method is
    responsible for updating the *connectivity* of the mesh only---it does not
    need to update the vertex positions.  These positions will be updated in
    Halfedge_Mesh::bevel_vertex_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_vertex(Halfedge_Mesh::VertexRef v) {

    (void)v;
    return std::nullopt;
}

/*
    This method should replace the edge e with a face, corresponding to a
    bevel operation. It should return the new face. NOTE: This method is
    responsible for updating the *connectivity* of the mesh only---it does not
    need to update the vertex positions.  These positions will be updated in
    Halfedge_Mesh::bevel_edge_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_edge(Halfedge_Mesh::EdgeRef e) {

    (void)e;
    return std::nullopt;
}

/*
    This method should replace the face f with an additional, inset face
    (and ring of faces around it), corresponding to a bevel operation. It
    should return the new face.  NOTE: This method is responsible for updating
    the *connectivity* of the mesh only---it does not need to update the vertex
    positions. These positions will be updated in
    Halfedge_Mesh::bevel_face_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_face(Halfedge_Mesh::FaceRef f) {
    if (f->is_boundary()) {
        (void) f;
        return std::nullopt;
    }

    HalfedgeRef h = f->halfedge(); // iterator for face's halfedges
    unsigned int d = f->degree(); // degree of face

    // vertices from earlier quads to keep track of.
    VertexRef first_vertex; // first vertex. only allocate for first iteration.
    VertexRef prev_vertex; // previous face's second vertex.
    
    // edges from earlier quads to keep track of.
    EdgeRef first_edge; // first edge. only allocate for first iteration.
    EdgeRef prev_edge; // previous face's left edge.

    // halfedges from earlier quads to keep track of.
    HalfedgeRef first_halfedge; // first half-edge. only allocate for first iteration.
    HalfedgeRef prev_halfedge; // previous face's last half-edge.

    for (unsigned int i = 0; i < d; ++i) {
        // Flags for iteration
        bool isFirst = (i == 0);
        bool isLast = (i == d - 1);

        // Keep track of old elements
        HalfedgeRef t = h->twin();
        VertexRef ov0 = h->vertex(); // old first vertex
        VertexRef ov1 = t->vertex(); // old second vertex
        EdgeRef e0 = t->edge();
        e0->halfedge() = t; // just incase e0 points to h.

        // Allocate new elements
        FaceRef nf = new_face();
        EdgeRef e2 = new_edge(); // new edge. always allocate for each iteration
        EdgeRef e1 = isFirst? new_edge() : prev_edge; // first edge
        EdgeRef e3 = isLast? first_edge : new_edge(); // last edge
        VertexRef nv0 = isFirst? new_vertex() : prev_vertex; // new first vertex
        VertexRef nv1 = isLast? first_vertex : new_vertex(); // new second vertex

        // New halfedges
        HalfedgeRef nh0 = new_halfedge();
        HalfedgeRef nh1 = new_halfedge();
        HalfedgeRef nh2 = new_halfedge();
        HalfedgeRef nh3 = new_halfedge();
        
        // Edge 2
        nh0->twin() = h; // swap twins
        h->twin() = nh0;

        h->vertex() = nv0; // new vertex
        nv0->halfedge() = h;

        nh0->vertex() = nv1; // new vertex. nv1's halfedge set later
        nh0->next() = nh1; // next halfedge
        nh0->edge() = e2; // new edge
        h->edge() = e2;
        e2->halfedge() = h;

        nh0->face() = nf; // new face
        nf->halfedge() = nh0; 
        
        // Edge 1
        e1->halfedge() = nh1;
        nh1->edge() = e1;
        nh1->vertex() = nv0;
        nh1->face() = nf;
        nh1->next() = nh2;
        if (!isFirst) {
            nh1->twin() = prev_halfedge;
            prev_halfedge->twin() = nh1;
        }

        // Edge 0
        nh2->vertex() = ov0;
        ov0->halfedge() = nh2;
        nh2->edge() = e0;
        nh2->twin() = t;
        t->twin() = nh2;
        nh2->face() = nf;
        nh2->next() = nh3;

        // Edge 3
        nh3->edge() = e3;
        nh3->face() = nf;
        nh3->vertex() = ov1;
        nh3->next() = nh0;
        if (isLast) {
            nh3->twin() = first_halfedge;
            first_halfedge->twin() = nh3;
        }

        // Set first and previous for next iterations
        if (isFirst) {
            first_edge = e1;
            first_vertex = nv0;
            first_halfedge = nh1;
        }
        if (!isLast) {
            prev_edge = e3;
            prev_vertex = nv1;
            prev_halfedge = nh3;
        }

        h = h->next(); // DONT FORGET TO ITERATE
    }

    return f;
}

/*
    Compute new vertex positions for the vertices of the beveled vertex.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the original vertex position and its associated outgoing edge
    to compute a new vertex position along the outgoing edge.
*/
void Halfedge_Mesh::bevel_vertex_positions(const std::vector<Vec3> &start_positions,
                                           Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while (h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
}

/*
    Compute new vertex positions for the vertices of the beveled edge.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the orig array) to compute an offset vertex position.

    Note that there is a 1-to-1 correspondence between halfedges in
    newHalfedges and vertex positions
    in orig.  So, you can write loops of the form

    for(size_t i = 0; i < new_halfedges.size(); i++)
    {
            Vector3D pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_edge_positions(const std::vector<Vec3> &start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while (h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
}

/*
    Compute new vertex positions for the vertices of the beveled face.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the start_positions array) to compute an offset vertex
    position.

    Note that there is a 1-to-1 correspondence between halfedges in
    new_halfedges and vertex positions
    in orig. So, you can write loops of the form

    for(size_t i = 0; i < new_halfedges.size(); i++)
    {
            Vec3 pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_face_positions(const std::vector<Vec3> &start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset,
                                         float normal_offset) {

    if (flip_orientation) normal_offset = -normal_offset;
    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while (h != face->halfedge());
 
    int N = (int)new_halfedges.size();
    //Vec3 n = cross(start_positions[1] - start_positions[0], start_positions[2] - start_positions[0]).unit();
    Vec3 n = face->normal();
    //std::cout << start_positions << std::endl; // NaN????
    for(int i = 0; i < N; i++) {
        // Assuming we're looking at vertex i, compute the indices
        // of the next and previous elements in the list using
        // modular arithmetic---note that to get the previous index,
        // we can't just subtract 1 because the mod operation in C++
        // doesn't behave quite how you might expect for negative
        // values!
        int a = (i+N-1) % N;
        int b = i;
        int c = (i+1) % N;

        // Get the actual 3D vertex coordinates at these vertices
        Vec3 pa = start_positions[a];
        Vec3 pb = start_positions[b];
        Vec3 pc = start_positions[c];
        // std::cout << pb << std::endl; all zeros???
        // Cross product as quarter rotation
        Vec3 ca = pa - pc;
        Vec3 t = cross(n, ca);
       
        new_halfedges[i]->vertex()->pos = pb + t*tangent_offset + n*normal_offset;
    }
}

/*
    Splits all non-triangular faces into triangles.
*/
void Halfedge_Mesh::triangulate() {

    // For each face...
}

/* Note on the quad subdivision process:

        Unlike the local mesh operations (like bevel or edge flip), we will perform
        subdivision by splitting *all* faces into quads "simultaneously."  Rather
        than operating directly on the halfedge data structure (which as you've
        seen is quite difficult to maintain!) we are going to do something a bit nicer:
           1. Create a raw list of vertex positions and faces (rather than a full-
              blown halfedge mesh).
           2. Build a new halfedge mesh from these lists, replacing the old one.
        Sometimes rebuilding a data structure from scratch is simpler (and even
        more efficient) than incrementally modifying the existing one.  These steps are
        detailed below.

  Step I: Compute the vertex positions for the subdivided mesh.
        Here we're going to do something a little bit strange: since we will
        have one vertex in the subdivided mesh for each vertex, edge, and face in
        the original mesh, we can nicely store the new vertex *positions* as
        attributes on vertices, edges, and faces of the original mesh. These positions
        can then be conveniently copied into the new, subdivided mesh.
        This is what you will implement in linear_subdivide_positions() and
        catmullclark_subdivide_positions().

  Steps II-IV are provided (see Halfedge_Mesh::subdivide()), but are still detailed
  here:

  Step II: Assign a unique index (starting at 0) to each vertex, edge, and
        face in the original mesh. These indices will be the indices of the
        vertices in the new (subdivided mesh).  They do not have to be assigned
        in any particular order, so long as no index is shared by more than one
        mesh element, and the total number of indices is equal to V+E+F, i.e.,
        the total number of vertices plus edges plus faces in the original mesh.
        Basically we just need a one-to-one mapping between original mesh elements
        and subdivided mesh vertices.

  Step III: Build a list of quads in the new (subdivided) mesh, as tuples of
        the element indices defined above. In other words, each new quad should be
        of the form (i,j,k,l), where i,j,k and l are four of the indices stored on
        our original mesh elements.  Note that it is essential to get the orientation
        right here: (i,j,k,l) is not the same as (l,k,j,i).  Indices of new faces
        should circulate in the same direction as old faces (think about the right-hand
        rule).

  Step IV: Pass the list of vertices and quads to a routine that clears
        the internal data for this halfedge mesh, and builds new halfedge data from
        scratch, using the two lists.
*/

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos.  The values of the positions are based on
    simple linear interpolation, e.g., the edge midpoints and face
    centroids.
*/
void Halfedge_Mesh::linear_subdivide_positions() {

    // For each vertex, assign Vertex::new_pos to
    // its original position, Vertex::pos.

    // For each edge, assign the midpoint of the two original
    // positions to Edge::new_pos.

    // For each face, assign the centroid (i.e., arithmetic mean)
    // of the original vertex positions to Face::new_pos. Note
    // that in general, NOT all faces will be triangles!
}

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos.  The values of the positions are based on
    the Catmull-Clark rules for subdivision.

    Note: this will only be called on meshes without boundary
*/
void Halfedge_Mesh::catmullclark_subdivide_positions() {

    // The implementation for this routine should be
    // a lot like Halfedge_Mesh:linear_subdivide_positions:(),
    // except that the calculation of the positions themsevles is
    // slightly more involved, using the Catmull-Clark subdivision
    // rules. (These rules are outlined in the Developer Manual.)

    // Faces

    // Edges

    // Vertices
}

/*
        This routine should increase the number of triangles in the mesh
        using Loop subdivision. Note: this is will only be called on triangle meshes.
*/
void Halfedge_Mesh::loop_subdivide() {

    // Compute new positions for all the vertices in the input mesh, using
    // the Loop subdivision rule, and store them in Vertex::new_pos.
    // -> At this point, we also want to mark each vertex as being a vertex of the
    //    original mesh. Use Vertex::is_new for this.
    // -> Next, compute the updated vertex positions associated with edges, and
    //    store it in Edge::new_pos.
    // -> Next, we're going to split every edge in the mesh, in any order.  For
    //    future reference, we're also going to store some information about which
    //    subdivided edges come from splitting an edge in the original mesh, and
    //    which edges are new, by setting the flat Edge::is_new. Note that in this
    //    loop, we only want to iterate over edges of the original mesh.
    //    Otherwise, we'll end up splitting edges that we just split (and the
    //    loop will never end!)
    // -> Now flip any new edge that connects an old and new vertex.
    // -> Finally, copy the new vertex positions into final Vertex::pos.

    // Each vertex and edge of the original surface can be associated with a
    // vertex in the new (subdivided) surface.
    // Therefore, our strategy for computing the subdivided vertex locations is to
    // *first* compute the new positions
    // using the connectivity of the original (coarse) mesh; navigating this mesh
    // will be much easier than navigating
    // the new subdivided (fine) mesh, which has more elements to traverse.  We
    // will then assign vertex positions in
    // the new mesh based on the values we computed for the original mesh.

    // Compute updated positions for all the vertices in the original mesh, using
    // the Loop subdivision rule.

    // Next, compute the updated vertex positions associated with edges.

    // Next, we're going to split every edge in the mesh, in any order. For
    // future reference, we're also going to store some information about which
    // subdivided edges come from splitting an edge in the original mesh, and
    // which edges are new.
    // In this loop, we only want to iterate over edges of the original
    // mesh---otherwise, we'll end up splitting edges that we just split (and
    // the loop will never end!)

    // Finally, flip any new edge that connects an old and new vertex.

    // Copy the updated vertex positions to the subdivided mesh.
}

/*
    Isotropic remeshing. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if this is not a triangle mesh)
*/
bool Halfedge_Mesh::isotropic_remesh() {

    // Compute the mean edge length.
    // Repeat the four main steps for 5 or 6 iterations
    // -> Split edges much longer than the target length (being careful about
    //    how the loop is written!)
    // -> Collapse edges much shorter than the target length.  Here we need to
    //    be EXTRA careful about advancing the loop, because many edges may have
    //    been destroyed by a collapse (which ones?)
    // -> Now flip each edge if it improves vertex degree
    // -> Finally, apply some tangential smoothing to the vertex positions

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate are called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    return false;
}

/* Helper type for quadric simplification */
struct Edge_Record {
    Edge_Record() {}
    Edge_Record(std::unordered_map<Halfedge_Mesh::VertexRef, Mat4> &vertex_quadrics,
                Halfedge_Mesh::EdgeRef e)
        : edge(e) {

        // Compute the combined quadric from the edge endpoints.
        // -> Build the 3x3 linear system whose solution minimizes the quadric error
        //    associated with these two endpoints.
        // -> Use this system to solve for the optimal position, and store it in
        //    Edge_Record::optimal.
        // -> Also store the cost associated with collapsing this edge in
        //    Edge_Record::cost.
    }
    Halfedge_Mesh::EdgeRef edge;
    Vec3 optimal;
    float cost;
};

/* Comparison operator for Edge_Records so std::set will properly order them */
bool operator<(const Edge_Record &r1, const Edge_Record &r2) {
    if (r1.cost != r2.cost) {
        return r1.cost < r2.cost;
    }
    Halfedge_Mesh::EdgeRef e1 = r1.edge;
    Halfedge_Mesh::EdgeRef e2 = r2.edge;
    return &*e1 < &*e2;
}

/** Helper type for quadric simplification
 *
 * A PQueue is a minimum-priority queue that
 * allows elements to be both inserted and removed from the
 * queue.  Together, one can easily change the priority of
 * an item by removing it, and re-inserting the same item
 * but with a different priority.  A priority queue, for
 * those who don't remember or haven't seen it before, is a
 * data structure that always keeps track of the item with
 * the smallest priority or "score," even as new elements
 * are inserted and removed.  Priority queues are often an
 * essential component of greedy algorithms, where one wants
 * to iteratively operate on the current "best" element.
 *
 * PQueue is templated on the type T of the object
 * being queued.  For this reason, T must define a comparison
 * operator of the form
 *
 *    bool operator<( const T& t1, const T& t2 )
 *
 * which returns true if and only if t1 is considered to have a
 * lower priority than t2.
 *
 * Basic use of a PQueue might look
 * something like this:
 *
 *    // initialize an empty queue
 *    PQueue<myItemType> queue;
 *
 *    // add some items (which we assume have been created
 *    // elsewhere, each of which has its priority stored as
 *    // some kind of internal member variable)
 *    queue.insert( item1 );
 *    queue.insert( item2 );
 *    queue.insert( item3 );
 *
 *    // get the highest priority item currently in the queue
 *    myItemType highestPriorityItem = queue.top();
 *
 *    // remove the highest priority item, automatically
 *    // promoting the next-highest priority item to the top
 *    queue.pop();
 *
 *    myItemType nextHighestPriorityItem = queue.top();
 *
 *    // Etc.
 *
 *    // We can also remove an item, making sure it is no
 *    // longer in the queue (note that this item may already
 *    // have been removed, if it was the 1st or 2nd-highest
 *    // priority item!)
 *    queue.remove( item2 );
 *
 */
template <class T> struct PQueue {
    void insert(const T &item) { queue.insert(item); }
    void remove(const T &item) {
        if (queue.find(item) != queue.end()) {
            queue.erase(item);
        }
    }
    const T &top(void) const { return *(queue.begin()); }
    void pop(void) { queue.erase(queue.begin()); }
    size_t size() { return queue.size(); }

    std::set<T> queue;
};

/*
    Mesh simplification. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if you can't simplify the mesh any
    further without destroying it.)
*/
bool Halfedge_Mesh::simplify() {

    std::unordered_map<VertexRef, Mat4> vertex_quadrics;
    std::unordered_map<FaceRef, Mat4> face_quadrics;
    std::unordered_map<EdgeRef, Edge_Record> edge_records;
    PQueue<Edge_Record> edge_queue;

    // Compute initial quadrics for each face by simply writing the plane equation
    // for the face in homogeneous coordinates. These quadrics should be stored
    // in face_quadrics
    // -> Compute an initial quadric for each vertex as the sum of the quadrics
    //    associated with the incident faces, storing it in vertex_quadrics
    // -> Build a priority queue of edges according to their quadric error cost,
    //    i.e., by building an Edge_Record for each edge and sticking it in the
    //    queue. You may want to use the above PQueue<Edge_Record> for this.
    // -> Until we reach the target edge budget, collapse the best edge. Remember
    //    to remove from the queue any edge that touches the collapsing edge
    //    BEFORE it gets collapsed, and add back into the queue any edge touching
    //    the collapsed vertex AFTER it's been collapsed. Also remember to assign
    //    a quadric to the collapsed vertex, and to pop the collapsed edge off the
    //    top of the queue.

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate are called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    return false;
}
