#include <vector>
#include <iostream>
#include <float.h>
#include <assert.h>
#include "meshEdit.h"
#include "mutablePriorityQueue.h"
#include "error_dialog.h"

namespace CMU462
{

  VertexIter HalfedgeMesh::splitEdge(EdgeIter e0)
  {
    // TODO: (meshEdit)
    // This method should split the given edge and return an iterator to the
    // newly inserted vertex. The halfedge of this vertex should point along
    // the edge that was split, rather than the new edges.

    showError("splitEdge() not implemented.");
    return VertexIter();
  }

  VertexIter HalfedgeMesh::collapseEdge(EdgeIter e)
  {
    // TODO: (meshEdit)
    // This method should collapse the given edge and return an iterator to
    // the new vertex created by the collapse.

    if (e->isBoundary())
    {
      return e->halfedge()->vertex();
    }

    //Collect Elements
    HalfedgeIter h0 = e->halfedge();
    HalfedgeIter h1 = h0->twin();

    HalfedgeIter h0_1 = h0->next();
    HalfedgeIter h0_2 = h0_1->next();

    HalfedgeIter h1_1 = h1->next();
    HalfedgeIter h1_2 = h1_1->next();

    HalfedgeIter prev_h0 = h0;
    while (prev_h0->next() != h0)
    {
      prev_h0 = prev_h0->next();
    }
    HalfedgeIter prev_h1 = h1;
    while (prev_h1->next() != h1)
    {
      prev_h1 = prev_h1->next();
    }

    EdgeIter e0_1 = h0_1->edge();
    EdgeIter e0_2 = h0_2->edge();
    EdgeIter e1_1 = h1_1->edge();
    EdgeIter e1_2 = h1_2->edge();

    FaceIter f0 = h0->face();
    FaceIter f1 = h1->face();

    VertexIter v0 = h0->vertex();
    VertexIter v1 = h1->vertex();

    VertexIter v0_1 = h0_2->vertex();
    VertexIter v1_1 = h1_2->vertex();

    std::vector<HalfedgeIter> v1_halfedges;
    HalfedgeIter h = h1->twin()->next();
    while (h != h1)
    {
      v1_halfedges.push_back(h);
      h = h->twin()->next();
    }

    bool delf0 = false;
    bool delf1 = false;

    int f0_degree = f0->degree();
    int f1_degree = f1->degree();

    if (f0_degree == 3) // We will have to delete f0
    {
      delf0 = true;
      if (h0_1->isBoundary() || h0_2->isBoundary())
      {
        return e->halfedge()->vertex();
      }

      h0_1->twin()->twin() = h0_2->twin();
      h0_2->twin()->twin() = h0_1->twin();
      h0_1->twin()->edge() = e0_2;

      e0_2->halfedge() = h0_1->twin();

      v0_1->halfedge() = h0_1->twin();
    }
    else // We will keep f0
    {
      prev_h0->next() = h0->next();
      f0->halfedge() = prev_h0;
    }

    if (f1_degree == 3) // We will have to delete f1
    {
      delf1 = true;
      if (h1_1->isBoundary() || h1_2->isBoundary())
      {
        return e->halfedge()->vertex();
      }

      h1_1->twin()->twin() = h1_2->twin();
      h1_2->twin()->twin() = h1_1->twin();
      h1_2->twin()->edge() = e1_1;

      e1_1->halfedge() = h1_2->twin();

      v1_1->halfedge() = h1_2->twin();
    }
    else // We will keep f1
    {
      prev_h1->next() = h1->next();
      f1->halfedge() = prev_h1;
    }

    // Set v0 position
    v0->position = e->centroid();
    v0->halfedge() = prev_h0->twin();

    // Assign all v1 neighbour half edges's vertex to v0
    for (HalfedgeIter h : v1_halfedges)
    {
      h->vertex() = v0;
    }

    deleteHalfedge(h0);
    deleteHalfedge(h1);
    deleteVertex(v1);
    deleteEdge(e);

    if (delf0)
    {
      deleteEdge(e0_1);
      deleteHalfedge(h0_1);
      deleteHalfedge(h0_2);
      deleteFace(f0);
    }

    if (delf1)
    {
      deleteEdge(e1_2);
      deleteHalfedge(h1_1);
      deleteHalfedge(h1_2);
      deleteFace(f1);
    }

    return v0;
  }

  VertexIter HalfedgeMesh::collapseFace(FaceIter f)
  {
    // This method should collapse the given face and return an iterator to
    // the new vertex created by the collapse.

    Vector3D f_centroid = f->centroid();
    std::vector<EdgeIter> f_edges;
    HalfedgeIter h = f->halfedge();
    VertexIter v;

    while (h->next() != f->halfedge())
    {
      f_edges.push_back(h->edge());
      h = h->next();
    }
    f_edges.push_back(h->edge());

    for (int i = 0; i < f_edges.size() - 2; i++)
    {
      std::cout << i << std::endl;
      v = collapseEdge(f_edges[i]);
    }

    v->position = f_centroid;

    return v;
  }

  FaceIter HalfedgeMesh::eraseVertex(VertexIter v)
  {
    // This method should replace the given vertex and all its neighboring
    // edges and faces with a single face, returning the new face.

    //Collect Elements
    std::vector<EdgeIter> edge_neighbours;
    HalfedgeIter h = v->halfedge();
    do
    {
      edge_neighbours.push_back(h->edge());
      h = h->twin()->next();
    } while (h != v->halfedge());

    //Erase Edges
    FaceIter f;
    while (!edge_neighbours.empty())
    {
      EdgeIter e = edge_neighbours.back();
      edge_neighbours.pop_back();
      f = eraseEdge(e);
    }

    return f;
  }

  FaceIter HalfedgeMesh::eraseEdge(EdgeIter e)
  {
    // This method should erase the given edge and return an iterator to the
    // merged face.

    // Do nothing if the edge is at Boundary
    if (e->isBoundary())
    {
      return e->halfedge()->face();
    }

    // Collect Elements
    // HALFEDGES
    HalfedgeIter h0 = e->halfedge();
    HalfedgeIter h1 = h0->twin();

    std::vector<HalfedgeIter> h0_neighbours;

    HalfedgeIter h0_next = h0;
    while (h0_next->next() != h0)
    {
      h0_next = h0_next->next();
      h0_neighbours.push_back(h0_next);
    }

    std::vector<HalfedgeIter> h1_neighbours;

    HalfedgeIter h1_next = h1;
    while (h1_next->next() != h1)
    {
      h1_next = h1_next->next();
      h1_neighbours.push_back(h1_next);
    }

    //VERTICES
    VertexIter v0 = h0->vertex();
    VertexIter v1 = h1->vertex();

    //FACES
    FaceIter f0 = h0->face();
    FaceIter f1 = h1->face();

    bool delv0 = v0->degree() == 1;
    bool delv1 = v1->degree() == 1;

    //Reassign Elements
    //HALFEDGES
    for (auto h : h1_neighbours)
    {
      h->face() = f0;
    }

    h1_neighbours.back()->next() = h0_neighbours.front();
    h0_neighbours.back()->next() = h1_neighbours.front();

    //VERTICES
    if (!delv0)
    {
      v0->halfedge() = h1_neighbours.front();
    }
    if (!delv1)
    {
      v1->halfedge() = h0_neighbours.front();
    }

    //FACES
    if (!delv0 || !delv1)
    {
      if (!delv0)
      {
        f0->halfedge() = h1_neighbours.front();
      }
      else if (!delv1)
      {
        f0->halfedge() = h0_neighbours.front();
      }
    }

    //Delete Elements
    h1_neighbours.clear();
    h0_neighbours.clear();
    deleteHalfedge(h0);
    deleteHalfedge(h1);
    deleteEdge(e);
    if (!delv0 && !delv1)
    {
      deleteFace(f1);
    }
    if (delv0)
    {
      deleteVertex(v0);
    }
    if (delv1)
    {
      deleteVertex(v1);
    }

    return f0;
  }

  EdgeIter HalfedgeMesh::flipEdge(EdgeIter e0)
  {
    // Return itself if the edge is on the boundary
    if (e0->isBoundary())
    {
      return e0;
    }

    // Collect Elements
    // HALFEDGES
    HalfedgeIter h0 = e0->halfedge();
    HalfedgeIter h1 = h0->twin();

    HalfedgeIter h0_next = h0->next();
    HalfedgeIter h0_next_next = h0_next->next();

    HalfedgeIter h1_next = h1->next();
    HalfedgeIter h1_next_next = h1_next->next();

    HalfedgeIter h0_prev = h0;
    while (h0_prev->next() != h0)
    {
      h0_prev = h0_prev->next();
    }
    HalfedgeIter h1_prev = h1;
    while (h1_prev->next() != h1)
    {
      h1_prev = h1_prev->next();
    }

    //VERTICES
    VertexIter v0 = h0->vertex();
    VertexIter v1 = h1->vertex();
    VertexIter v2 = h0_next_next->vertex();
    VertexIter v3 = h1_next_next->vertex();

    //FACES
    FaceIter f0 = h0->face();
    FaceIter f1 = h1->face();

    //Reassign Elements
    //HALFEDGES
    h0->next() = h0_next_next;
    h0->vertex() = v3;
    h1->next() = h1_next_next;
    h1->vertex() = v2;
    h0_next->next() = h1;
    h0_next->face() = f1;
    h1_next->next() = h0;
    h1_next->face() = f0;
    h0_prev->next() = h1_next;
    h1_prev->next() = h0_next;

    //VERTICES
    v0->halfedge() = h1_next;
    v1->halfedge() = h0_next;

    //FACES
    f0->halfedge() = h0;
    f1->halfedge() = h1;

    return e0;
  }

  void HalfedgeMesh::subdivideQuad(bool useCatmullClark)
  {
    // Unlike the local mesh operations (like bevel or edge flip), we will perform
    // subdivision by splitting *all* faces into quads "simultaneously."  Rather
    // than operating directly on the halfedge data structure (which as you've
    // seen
    // is quite difficult to maintain!) we are going to do something a bit nicer:
    //
    //    1. Create a raw list of vertex positions and faces (rather than a full-
    //       blown halfedge mesh).
    //
    //    2. Build a new halfedge mesh from these lists, replacing the old one.
    //
    // Sometimes rebuilding a data structure from scratch is simpler (and even
    // more
    // efficient) than incrementally modifying the existing one.  These steps are
    // detailed below.

    // TODO Step I: Compute the vertex positions for the subdivided mesh.  Here
    // we're
    // going to do something a little bit strange: since we will have one vertex
    // in
    // the subdivided mesh for each vertex, edge, and face in the original mesh,
    // we
    // can nicely store the new vertex *positions* as attributes on vertices,
    // edges,
    // and faces of the original mesh.  These positions can then be conveniently
    // copied into the new, subdivided mesh.
    // [See subroutines for actual "TODO"s]
    if (useCatmullClark)
    {
      computeCatmullClarkPositions();
    }
    else
    {
      computeLinearSubdivisionPositions();
    }

    // TODO Step II: Assign a unique index (starting at 0) to each vertex, edge,
    // and
    // face in the original mesh.  These indices will be the indices of the
    // vertices
    // in the new (subdivided mesh).  They do not have to be assigned in any
    // particular
    // order, so long as no index is shared by more than one mesh element, and the
    // total number of indices is equal to V+E+F, i.e., the total number of
    // vertices
    // plus edges plus faces in the original mesh.  Basically we just need a
    // one-to-one
    // mapping between original mesh elements and subdivided mesh vertices.
    // [See subroutine for actual "TODO"s]
    assignSubdivisionIndices();

    // TODO Step III: Build a list of quads in the new (subdivided) mesh, as
    // tuples of
    // the element indices defined above.  In other words, each new quad should be
    // of
    // the form (i,j,k,l), where i,j,k and l are four of the indices stored on our
    // original mesh elements.  Note that it is essential to get the orientation
    // right
    // here: (i,j,k,l) is not the same as (l,k,j,i).  Indices of new faces should
    // circulate in the same direction as old faces (think about the right-hand
    // rule).
    // [See subroutines for actual "TODO"s]
    vector<vector<Index>> subDFaces;
    vector<Vector3D> subDVertices;
    buildSubdivisionFaceList(subDFaces);
    buildSubdivisionVertexList(subDVertices);

    // TODO Step IV: Pass the list of vertices and quads to a routine that clears
    // the
    // internal data for this halfedge mesh, and builds new halfedge data from
    // scratch,
    // using the two lists.
    rebuild(subDFaces, subDVertices);
  }

  /**
 * Compute new vertex positions for a mesh that splits each polygon
 * into quads (by inserting a vertex at the face midpoint and each
 * of the edge midpoints).  The new vertex positions will be stored
 * in the members Vertex::newPosition, Edge::newPosition, and
 * Face::newPosition.  The values of the positions are based on
 * simple linear interpolation, e.g., the edge midpoints and face
 * centroids.
 */
  void HalfedgeMesh::computeLinearSubdivisionPositions()
  {
    // TODO For each vertex, assign Vertex::newPosition to
    // its original position, Vertex::position.

    // TODO For each edge, assign the midpoint of the two original
    // positions to Edge::newPosition.

    // TODO For each face, assign the centroid (i.e., arithmetic mean)
    // of the original vertex positions to Face::newPosition.  Note
    // that in general, NOT all faces will be triangles!
    showError("computeLinearSubdivisionPositions() not implemented.");
  }

  /**
 * Compute new vertex positions for a mesh that splits each polygon
 * into quads (by inserting a vertex at the face midpoint and each
 * of the edge midpoints).  The new vertex positions will be stored
 * in the members Vertex::newPosition, Edge::newPosition, and
 * Face::newPosition.  The values of the positions are based on
 * the Catmull-Clark rules for subdivision.
 */
  void HalfedgeMesh::computeCatmullClarkPositions()
  {
    // TODO The implementation for this routine should be
    // a lot like HalfedgeMesh::computeLinearSubdivisionPositions(),
    // except that the calculation of the positions themsevles is
    // slightly more involved, using the Catmull-Clark subdivision
    // rules. (These rules are outlined in the Developer Manual.)

    // TODO face

    // TODO edges

    // TODO vertices
    showError("computeCatmullClarkPositions() not implemented.");
  }

  /**
 * Assign a unique integer index to each vertex, edge, and face in
 * the mesh, starting at 0 and incrementing by 1 for each element.
 * These indices will be used as the vertex indices for a mesh
 * subdivided using Catmull-Clark (or linear) subdivision.
 */
  void HalfedgeMesh::assignSubdivisionIndices()
  {
    // TODO Start a counter at zero; if you like, you can use the
    // "Index" type (defined in halfedgeMesh.h)

    // TODO Iterate over vertices, assigning values to Vertex::index

    // TODO Iterate over edges, assigning values to Edge::index

    // TODO Iterate over faces, assigning values to Face::index
    showError("assignSubdivisionIndices() not implemented.");
  }

  /**
 * Build a flat list containing all the vertex positions for a
 * Catmull-Clark (or linear) subdivison of this mesh.  The order of
 * vertex positions in this list must be identical to the order
 * of indices assigned to Vertex::newPosition, Edge::newPosition,
 * and Face::newPosition.
 */
  void HalfedgeMesh::buildSubdivisionVertexList(vector<Vector3D> &subDVertices)
  {
    // TODO Resize the vertex list so that it can hold all the vertices.

    // TODO Iterate over vertices, assigning Vertex::newPosition to the
    // appropriate location in the new vertex list.

    // TODO Iterate over edges, assigning Edge::newPosition to the appropriate
    // location in the new vertex list.

    // TODO Iterate over faces, assigning Face::newPosition to the appropriate
    // location in the new vertex list.
    showError("buildSubdivisionVertexList() not implemented.");
  }

  /**
 * Build a flat list containing all the quads in a Catmull-Clark
 * (or linear) subdivision of this mesh.  Each quad is specified
 * by a vector of four indices (i,j,k,l), which come from the
 * members Vertex::index, Edge::index, and Face::index.  Note that
 * the ordering of these indices is important because it determines
 * the orientation of the new quads; it is also important to avoid
 * "bowties."  For instance, (l,k,j,i) has the opposite orientation
 * of (i,j,k,l), and if (i,j,k,l) is a proper quad, then (i,k,j,l)
 * will look like a bowtie.
 */
  void HalfedgeMesh::buildSubdivisionFaceList(vector<vector<Index>> &subDFaces)
  {
    // TODO This routine is perhaps the most tricky step in the construction of
    // a subdivision mesh (second, perhaps, to computing the actual Catmull-Clark
    // vertex positions).  Basically what you want to do is iterate over faces,
    // then for each for each face, append N quads to the list (where N is the
    // degree of the face).  For this routine, it may be more convenient to simply
    // append quads to the end of the list (rather than allocating it ahead of
    // time), though YMMV.  You can of course iterate around a face by starting
    // with its first halfedge and following the "next" pointer until you get
    // back to the beginning.  The tricky part is making sure you grab the right
    // indices in the right order---remember that there are indices on vertices,
    // edges, AND faces of the original mesh.  All of these should get used.  Also
    // remember that you must have FOUR indices per face, since you are making a
    // QUAD mesh!

    // TODO iterate over faces
    // TODO loop around face
    // TODO build lists of four indices for each sub-quad
    // TODO append each list of four indices to face list
    showError("buildSubdivisionFaceList() not implemented.");
  }

  FaceIter HalfedgeMesh::bevelVertex(VertexIter v)
  {
    // TODO This method should replace the vertex v with a face, corresponding to
    // a bevel operation. It should return the new face.  NOTE: This method is
    // responsible for updating the *connectivity* of the mesh only---it does not
    // need to update the vertex positions.  These positions will be updated in
    // HalfedgeMesh::bevelVertexComputeNewPositions (which you also have to
    // implement!)

    showError("bevelVertex() not implemented.");
    return facesBegin();
  }

  FaceIter HalfedgeMesh::bevelEdge(EdgeIter e)
  {
    // TODO This method should replace the edge e with a face, corresponding to a
    // bevel operation. It should return the new face.  NOTE: This method is
    // responsible for updating the *connectivity* of the mesh only---it does not
    // need to update the vertex positions.  These positions will be updated in
    // HalfedgeMesh::bevelEdgeComputeNewPositions (which you also have to
    // implement!)

    showError("bevelEdge() not implemented.");
    return facesBegin();
  }

  FaceIter HalfedgeMesh::bevelFace(FaceIter f)
  {
    // TODO This method should replace the face f with an additional, inset face
    // (and ring of faces around it), corresponding to a bevel operation. It
    // should return the new face.  NOTE: This method is responsible for updating
    // the *connectivity* of the mesh only---it does not need to update the vertex
    // positions.  These positions will be updated in
    // HalfedgeMesh::bevelFaceComputeNewPositions (which you also have to
    // implement!)

    showError("bevelFace() not implemented.");
    return facesBegin();
  }

  void HalfedgeMesh::bevelFaceComputeNewPositions(
      vector<Vector3D> &originalVertexPositions,
      vector<HalfedgeIter> &newHalfedges, double normalShift,
      double tangentialInset)
  {
    // TODO Compute new vertex positions for the vertices of the beveled face.
    //
    // These vertices can be accessed via newHalfedges[i]->vertex()->position for
    // i = 1, ..., newHalfedges.size()-1.
    //
    // The basic strategy here is to loop over the list of outgoing halfedges,
    // and use the preceding and next vertex position from the original mesh
    // (in the originalVertexPositions array) to compute an offset vertex
    // position.
    //
    // Note that there is a 1-to-1 correspondence between halfedges in
    // newHalfedges and vertex positions
    // in orig.  So, you can write loops of the form
    //
    // for( int i = 0; i < newHalfedges.size(); hs++ )
    // {
    //    Vector3D pi = originalVertexPositions[i]; // get the original vertex
    //    position correponding to vertex i
    // }
    //
  }

  void HalfedgeMesh::bevelVertexComputeNewPositions(
      Vector3D originalVertexPosition, vector<HalfedgeIter> &newHalfedges,
      double tangentialInset)
  {
    // TODO Compute new vertex positions for the vertices of the beveled vertex.
    //
    // These vertices can be accessed via newHalfedges[i]->vertex()->position for
    // i = 1, ..., hs.size()-1.
    //
    // The basic strategy here is to loop over the list of outgoing halfedges,
    // and use the preceding and next vertex position from the original mesh
    // (in the orig array) to compute an offset vertex position.
  }

  void HalfedgeMesh::bevelEdgeComputeNewPositions(
      vector<Vector3D> &originalVertexPositions,
      vector<HalfedgeIter> &newHalfedges, double tangentialInset)
  {
    // TODO Compute new vertex positions for the vertices of the beveled edge.
    //
    // These vertices can be accessed via newHalfedges[i]->vertex()->position for
    // i = 1, ..., newHalfedges.size()-1.
    //
    // The basic strategy here is to loop over the list of outgoing halfedges,
    // and use the preceding and next vertex position from the original mesh
    // (in the orig array) to compute an offset vertex position.
    //
    // Note that there is a 1-to-1 correspondence between halfedges in
    // newHalfedges and vertex positions
    // in orig.  So, you can write loops of the form
    //
    // for( int i = 0; i < newHalfedges.size(); i++ )
    // {
    //    Vector3D pi = originalVertexPositions[i]; // get the original vertex
    //    position correponding to vertex i
    // }
    //
  }

  void HalfedgeMesh::splitPolygons(vector<FaceIter> &fcs)
  {
    for (auto f : fcs)
      splitPolygon(f);
  }

  void HalfedgeMesh::splitPolygon(FaceIter f)
  {
    // TODO: (meshedit)
    // Triangulate a polygonal face
    showError("splitPolygon() not implemented.");
  }

  EdgeRecord::EdgeRecord(EdgeIter &_edge) : edge(_edge)
  {
    // TODO: (meshEdit)
    // Compute the combined quadric from the edge endpoints.
    // -> Build the 3x3 linear system whose solution minimizes the quadric error
    //    associated with these two endpoints.
    // -> Use this system to solve for the optimal position, and store it in
    //    EdgeRecord::optimalPoint.
    // -> Also store the cost associated with collapsing this edg in
    //    EdgeRecord::Cost.
  }

  void MeshResampler::upsample(HalfedgeMesh &mesh)
  // This routine should increase the number of triangles in the mesh using Loop
  // subdivision.
  {
    // TODO: (meshEdit)
    // Compute new positions for all the vertices in the input mesh, using
    // the Loop subdivision rule, and store them in Vertex::newPosition.
    // -> At this point, we also want to mark each vertex as being a vertex of the
    //    original mesh.
    // -> Next, compute the updated vertex positions associated with edges, and
    //    store it in Edge::newPosition.
    // -> Next, we're going to split every edge in the mesh, in any order.  For
    //    future reference, we're also going to store some information about which
    //    subdivided edges come from splitting an edge in the original mesh, and
    //    which edges are new, by setting the flat Edge::isNew. Note that in this
    //    loop, we only want to iterate over edges of the original mesh.
    //    Otherwise, we'll end up splitting edges that we just split (and the
    //    loop will never end!)
    // -> Now flip any new edge that connects an old and new vertex.
    // -> Finally, copy the new vertex positions into final Vertex::position.

    // Each vertex and edge of the original surface can be associated with a
    // vertex in the new (subdivided) surface.
    // Therefore, our strategy for computing the subdivided vertex locations is to
    // *first* compute the new positions
    // using the connectity of the original (coarse) mesh; navigating this mesh
    // will be much easier than navigating
    // the new subdivided (fine) mesh, which has more elements to traverse.  We
    // will then assign vertex positions in
    // the new mesh based on the values we computed for the original mesh.

    // Compute updated positions for all the vertices in the original mesh, using
    // the Loop subdivision rule.

    // Next, compute the updated vertex positions associated with edges.

    // Next, we're going to split every edge in the mesh, in any order.  For
    // future
    // reference, we're also going to store some information about which
    // subdivided
    // edges come from splitting an edge in the original mesh, and which edges are
    // new.
    // In this loop, we only want to iterate over edges of the original
    // mesh---otherwise,
    // we'll end up splitting edges that we just split (and the loop will never
    // end!)

    // Finally, flip any new edge that connects an old and new vertex.

    // Copy the updated vertex positions to the subdivided mesh.
    showError("upsample() not implemented.");
  }

  void MeshResampler::downsample(HalfedgeMesh &mesh)
  {
    // TODO: (meshEdit)
    // Compute initial quadrics for each face by simply writing the plane equation
    // for the face in homogeneous coordinates. These quadrics should be stored
    // in Face::quadric
    // -> Compute an initial quadric for each vertex as the sum of the quadrics
    //    associated with the incident faces, storing it in Vertex::quadric
    // -> Build a priority queue of edges according to their quadric error cost,
    //    i.e., by building an EdgeRecord for each edge and sticking it in the
    //    queue.
    // -> Until we reach the target edge budget, collapse the best edge. Remember
    //    to remove from the queue any edge that touches the collapsing edge
    //    BEFORE it gets collapsed, and add back into the queue any edge touching
    //    the collapsed vertex AFTER it's been collapsed. Also remember to assign
    //    a quadric to the collapsed vertex, and to pop the collapsed edge off the
    //    top of the queue.
    showError("downsample() not implemented.");
  }

  void MeshResampler::resample(HalfedgeMesh &mesh)
  {
    // TODO: (meshEdit)
    // Compute the mean edge length.
    // Repeat the four main steps for 5 or 6 iterations
    // -> Split edges much longer than the target length (being careful about
    //    how the loop is written!)
    // -> Collapse edges much shorter than the target length.  Here we need to
    //    be EXTRA careful about advancing the loop, because many edges may have
    //    been destroyed by a collapse (which ones?)
    // -> Now flip each edge if it improves vertex degree
    // -> Finally, apply some tangential smoothing to the vertex positions
    showError("resample() not implemented.");
  }

} // namespace CMU462
