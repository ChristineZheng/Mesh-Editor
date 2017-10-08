#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{
  Vector2D lerp(float t, Vector2D p1, Vector2D p2) {
    return (1-t) * p1 + t * p2;
  }

  void BezierCurve::evaluateStep()
  {
    // TODO Part 1.
    // Perform one step of the Bezier curve's evaluation at t using de Casteljau's algorithm for subdivision.
    // Store all of the intermediate control points into the 2D vector evaluatedLevels.
    std::vector<Vector2D> new_v;
    if (evaluatedLevels.size() == 1) {
      for (int i = 0; i < controlPoints.size()-1; ++i) {  
        new_v.push_back(lerp(t, controlPoints[i], controlPoints[i+1]));
      }
      evaluatedLevels.push_back(new_v);
    }
    else {
      if (evaluatedLevels.back().size() != 1) {
        for (int i = 0; i < evaluatedLevels.back().size()-1; ++i) {  
          new_v.push_back(lerp(t, evaluatedLevels.back()[i], evaluatedLevels.back()[i+1]));
        }
        evaluatedLevels.push_back(new_v);
      }
    }
  }

  Vector3D lerp3D(float t, Vector3D p1, Vector3D p2) {
    return (1-t) * p1 + t * p2;
  }

  Vector3D deCasteljau(vector<Vector3D> vec, double p) {
    vector<Vector3D> pre = vec;
    vector<Vector3D> next = vec;
    while (next.size() > 1) {
      next.clear();
      for (int i =0; i < pre.size()-1; ++i) {
        next.push_back(lerp3D(p, pre[i], pre[i+1]));
      }
      pre = next;
    }
    return next[0];
  }


  Vector3D BezierPatch::evaluate(double u, double v) const
  {
    // TODO Part 2.
    // Evaluate the Bezier surface at parameters (u, v) through 2D de Casteljau subdivision.
    // (i.e. Unlike Part 1 where we performed one subdivision level per call to evaluateStep, this function
    // should apply de Casteljau's algorithm until it computes the final, evaluated point on the surface)
    std::vector<Vector3D> column_vec;
    for (int i = 0; i < controlPoints.size(); ++i) {
      column_vec.push_back(deCasteljau(controlPoints[i], u));
    }
    return deCasteljau(column_vec, v);
  }

  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> points, double t) const
  {
    // TODO Part 2.
    // Optional helper function that you might find useful to implement as an abstraction when implementing BezierPatch::evaluate.
    // Given an array of 4 points that lie on a single curve, evaluates the Bezier curve at parameter t using 1D de Casteljau subdivision.

    return Vector3D();
  }



  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // TODO Returns an approximate unit normal at this vertex, computed by
    // TODO taking the area-weighted average of the normals of neighboring
    // TODO triangles, then normalizing.

    Vector3D n(0,0,0);
    HalfedgeCIter h = halfedge();
    h = h->twin();
    HalfedgeCIter h_orig = h;
    do {
      Vector3D e1 = (h->next()->vertex()->position - h->vertex()->position);
      Vector3D e2 = (h->next()->twin()->vertex()->position - h->next()->vertex()->position);
      n += cross(e1, e2);
      h = h->next();
      h = h->twin();
    } while (h != h_orig);
    return n.unit();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // TODO This method should flip the given edge and return an iterator to the flipped edge.

    if (!e0->halfedge()->face()->isBoundary() && !e0->halfedge()->twin()->face()->isBoundary()) {
      // before
      HalfedgeIter h0 = e0->halfedge();
      HalfedgeIter h1 = h0->next();
      HalfedgeIter h2 = h1->next();
      HalfedgeIter h3 = h0->twin();
      HalfedgeIter h4 = h3->next();
      HalfedgeIter h5 = h4->next();
      HalfedgeIter h6 = h1->twin();
      HalfedgeIter h7 = h2->twin();
      HalfedgeIter h8 = h4->twin();
      HalfedgeIter h9 = h5->twin();

      VertexIter v0 = h0->vertex();
      VertexIter v1 = h3->vertex();
      VertexIter v2 = h2->vertex();
      VertexIter v3 = h5->vertex();

      EdgeIter e1 = h1->edge();
      EdgeIter e2 = h2->edge();
      EdgeIter e3 = h4->edge();
      EdgeIter e4 = h5->edge();

      FaceIter f0 = h0->face();
      FaceIter f1 = h3->face();

      // after
      h0->next() = h1;
      h0->twin() = h3;
      h0->vertex() = v3;
      h0->edge() = e0;
      h0->face() = f0;

      h1->next() = h2;
      h1->twin() = h7;
      h1->vertex() = v2;
      h1->edge() = e2;
      h1->face() = f0;

      h2->next() = h0;
      h2->twin() = h8;
      h2->vertex() = v0;
      h2->edge() = e3;
      h2->face() = f0;

      h3->next() = h4;
      h3->twin() = h0;
      h3->vertex() = v2;
      h3->edge() = e0;
      h3->face() = f1;

      h4->next() = h5;
      h4->twin() = h9;
      h4->vertex() = v3;
      h4->edge() = e4;
      h4->face() = f1;

      h5->next() = h3;
      h5->twin() = h6;
      h5->vertex() = v1;
      h5->edge() = e1;
      h5->face() = f1;

      h6->next() = h6->next();   
      h6->twin() = h5;
      h6->vertex() = v2;
      h6->edge() = e1;
      h6->face() = h6->face();

      h7->next() = h7->next();
      h7->twin() = h1;
      h7->vertex() = v0;
      h7->edge() = e2;
      h7->face() = h7->face();

      h8->next() = h8->next();
      h8->twin() = h2;
      h8->vertex() = v3;
      h8->edge() = e3;
      h8->face() = h8->face();      

      h9->next() = h9->next();
      h9->twin() = h4;
      h9->vertex() = v1;
      h9->edge() = e4;
      h9->face() = h9->face();

      // vertices
      v0->halfedge() = h2;
      v1->halfedge() = h5;
      v2->halfedge() = h3;
      v3->halfedge() = h0;

      // edges
      e0->halfedge() = h0;
      e1->halfedge() = h5;
      e2->halfedge() = h1;
      e3->halfedge() = h2;
      e4->halfedge() = h4;

      // faces
      f0->halfedge() = h0;
      f1->halfedge() = h3;

      return e0;
    }
      // return e0;

      // for boundary
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // TODO This method should split the given edge and return an iterator to the newly inserted vertex.
    // TODO The halfedge of this vertex should point along the edge that was split, rather than the new edges.
    // if (!e0->halfedge()->face()->isBoundary() && !e0->halfedge()->twin()->face()->isBoundary()) {
    //   // before
    //   HalfedgeIter h0 = e0->halfedge();
    //   HalfedgeIter h1 = h0->next();
    //   HalfedgeIter h2 = h1->next();
    //   HalfedgeIter h3 = h0->twin();
    //   HalfedgeIter h4 = h3->next();
    //   HalfedgeIter h5 = h4->next();
    //   HalfedgeIter h6 = h1->twin();
    //   HalfedgeIter h7 = h2->twin();
    //   HalfedgeIter h8 = h4->twin();
    //   HalfedgeIter h9 = h5->twin();

    //   VertexIter v0 = h0->vertex();
    //   VertexIter v1 = h3->vertex();
    //   VertexIter v2 = h2->vertex();
    //   VertexIter v3 = h5->vertex();

    //   EdgeIter e1 = h1->edge();
    //   EdgeIter e2 = h2->edge();
    //   EdgeIter e3 = h4->edge();
    //   EdgeIter e4 = h5->edge();

    //   FaceIter f0 = h0->face();
    //   FaceIter f1 = h3->face();

    //   // after
    //   Vector3D i = e0->halfedge()->vertex()->position;
    //   Vector3D j = e0->halfedge()->twin()->vertex()->position;
    //   Vector3D new_v = (i + j) / 2;
    //   VertexIter m = this->newVertex();
    //   m->position = new_v;

    //   EdgeIter e5 = this->newEdge();
    //   EdgeIter e6 = this->newEdge();
    //   EdgeIter e7 = this->newEdge();

    //   FaceIter f2 = this->newFace();
    //   FaceIter f3 = this->newFace();

    //   HalfedgeIter h10 = this->newHalfedge();
    //   HalfedgeIter h11 = this->newHalfedge();
    //   HalfedgeIter h12 = this->newHalfedge();
    //   HalfedgeIter h13 = this->newHalfedge();
    //   HalfedgeIter h14 = this->newHalfedge();
    //   HalfedgeIter h15 = this->newHalfedge();

    //   h0->next() = h13;
    //   h0->twin() = h10;
    //   h0->vertex() = v0;
    //   h0->edge() = e0;
    //   h0->face() = f0;

    //   h1->next() = h12;
    //   h1->twin() = h6;
    //   h1->vertex() = v1;
    //   h1->edge() = e1;
    //   h1->face() = f2;

    //   h2->next() = h0;
    //   h2->twin() = h7;
    //   h2->vertex() = v2;
    //   h2->edge() = e2;
    //   h2->face() = f0;

    //   h3->next() = h15;
    //   h3->twin() = h11;
    //   h3->vertex() = v1;
    //   h3->edge() = e5;
    //   h3->face() = f1;

    //   h4->next() = h14;
    //   h4->twin() = h8;
    //   h4->vertex() = v0;
    //   h4->edge() = e3;
    //   h4->face() = f3;

    //   h5->next() = h3;
    //   h5->twin() = h9;
    //   h5->vertex() = v3;
    //   h5->edge() = e4;
    //   h5->face() = f1;

    //   h6->next() = h6->next();
    //   h6->twin() = h1;
    //   h6->vertex() = v2;
    //   h6->edge() = e1;
    //   h6->face() = h6->face();

    //   h7->next() = h7->next();
    //   h7->twin() = h2;
    //   h7->vertex() = v0;
    //   h7->edge() = e2;
    //   h7->face() = h7->face();

    //   h8->next() = h8->next();
    //   h8->twin() = h4;
    //   h8->vertex() = v3;
    //   h8->edge() = e3;
    //   h8->face() = h8->face();

    //   h9->next() = h9->next();
    //   h9->twin() = h5;
    //   h9->vertex() = v1;
    //   h9->edge() = e4;
    //   h9->face() = h9->face();

    //   h10->next() = h4;
    //   h10->twin() = h0;
    //   h10->vertex() = m;
    //   h10->edge() = e0;
    //   h10->face() = f3;

    //   h11->next() = h1;
    //   h11->twin() = h3;
    //   h11->vertex() = m;
    //   h11->edge() = e5;
    //   h11->face() = f2;

    //   h12->next() = h11;
    //   h12->twin() = h13;
    //   h12->vertex() = v2;
    //   h12->edge() = e7;
    //   h12->face() = f2;

    //   h13->next() = h2;
    //   h13->twin() = h12;
    //   h13->vertex() = m;
    //   h13->edge() = e7;
    //   h13->face() = f0;

    //   h14->next() = h10;
    //   h14->twin() = h15;
    //   h14->vertex() = v3;
    //   h14->edge() = e6;
    //   h14->face() = f3;

    //   h15->next() = h5;
    //   h15->twin() = h14;
    //   h15->vertex() = m;
    //   h15->edge() = e6;
    //   h15->face() = f1;

    //   // vertex
    //   m->halfedge() = h10;  // ?? or h11
    //   v0->halfedge() = h0;
    //   v1->halfedge() = h3;
    //   v2->halfedge() = h12;
    //   v3->halfedge() = h14;

    //   // edges
    //   e0->halfedge() = h0;
    //   e1->halfedge() = h1;
    //   e2->halfedge() = h2;
    //   e3->halfedge() = h4;
    //   e4->halfedge() = h5;
    //   e5->halfedge() = h3;
    //   e6->halfedge() = h14;
    //   e7->halfedge() = h12;

    //   // faces
    //   f0->halfedge() = h0;
    //   f1->halfedge() = h3;
    //   f2->halfedge() = h11;
    //   f3->halfedge() = h10;
    // }
    // return e0->halfedge()->vertex();

    if (!e0->halfedge()->face()->isBoundary() && !e0->halfedge()->twin()->face()->isBoundary()) {
      // before
      HalfedgeIter h0 = e0->halfedge();
      HalfedgeIter h1 = h0->next();
      HalfedgeIter h2 = h1->next();
      HalfedgeIter h3 = h0->twin();
      HalfedgeIter h4 = h3->next();
      HalfedgeIter h5 = h4->next();
      HalfedgeIter h6 = h1->twin();
      HalfedgeIter h7 = h2->twin();
      HalfedgeIter h8 = h4->twin();
      HalfedgeIter h9 = h5->twin();

      VertexIter v0 = h0->vertex();
      VertexIter v1 = h3->vertex();
      VertexIter v2 = h2->vertex();
      VertexIter v3 = h5->vertex();

      EdgeIter e1 = h1->edge();
      EdgeIter e2 = h2->edge();
      EdgeIter e3 = h4->edge();
      EdgeIter e4 = h5->edge();

      FaceIter f0 = h0->face();
      FaceIter f1 = h3->face();

      // after
      Vector3D i = e0->halfedge()->vertex()->position;
      Vector3D j = e0->halfedge()->twin()->vertex()->position;
      Vector3D new_v = (i + j) / 2;
      VertexIter m = this->newVertex();
      m->position = new_v;
      // mark new vertex as new for part 6
      m->isNew = true;

      EdgeIter e5 = this->newEdge();
      EdgeIter e6 = this->newEdge();
      EdgeIter e7 = this->newEdge();
      // mark two new edges to new for part 6
      e6->isNew = true;
      e7->isNew = true;

      FaceIter f2 = this->newFace();
      FaceIter f3 = this->newFace();

      HalfedgeIter h10 = this->newHalfedge();
      HalfedgeIter h11 = this->newHalfedge();
      HalfedgeIter h12 = this->newHalfedge();
      HalfedgeIter h13 = this->newHalfedge();
      HalfedgeIter h14 = this->newHalfedge();
      HalfedgeIter h15 = this->newHalfedge();

      h0->next() = h13;
      h0->twin() = h10;
      h0->vertex() = v0;
      h0->edge() = e0;
      h0->face() = f0;

      h1->next() = h12;
      h1->twin() = h6;
      h1->vertex() = v1;
      h1->edge() = e1;
      h1->face() = f2;

      h2->next() = h0;
      h2->twin() = h7;
      h2->vertex() = v2;
      h2->edge() = e2;
      h2->face() = f0;

      h3->next() = h15;
      h3->twin() = h11;
      h3->vertex() = v1;
      h3->edge() = e5;
      h3->face() = f1;

      h4->next() = h14;
      h4->twin() = h8;
      h4->vertex() = v0;
      h4->edge() = e3;
      h4->face() = f3;

      h5->next() = h3;
      h5->twin() = h9;
      h5->vertex() = v3;
      h5->edge() = e4;
      h5->face() = f1;

      h6->next() = h6->next();
      h6->twin() = h1;
      h6->vertex() = v2;
      h6->edge() = e1;
      h6->face() = h6->face();

      h7->next() = h7->next();
      h7->twin() = h2;
      h7->vertex() = v0;
      h7->edge() = e2;
      h7->face() = h7->face();

      h8->next() = h8->next();
      h8->twin() = h4;
      h8->vertex() = v3;
      h8->edge() = e3;
      h8->face() = h8->face();

      h9->next() = h9->next();
      h9->twin() = h5;
      h9->vertex() = v1;
      h9->edge() = e4;
      h9->face() = h9->face();

      h10->next() = h4;
      h10->twin() = h0;
      h10->vertex() = m;
      h10->edge() = e0;
      h10->face() = f3;

      h11->next() = h1;
      h11->twin() = h3;
      h11->vertex() = m;
      h11->edge() = e5;
      h11->face() = f2;

      h12->next() = h11;
      h12->twin() = h13;
      h12->vertex() = v2;
      h12->edge() = e7;
      h12->face() = f2;

      h13->next() = h2;
      h13->twin() = h12;
      h13->vertex() = m;
      h13->edge() = e7;
      h13->face() = f0;

      h14->next() = h10;
      h14->twin() = h15;
      h14->vertex() = v3;
      h14->edge() = e6;
      h14->face() = f3;

      h15->next() = h5;
      h15->twin() = h14;
      h15->vertex() = m;
      h15->edge() = e6;
      h15->face() = f1;

      // vertex
      m->halfedge() = h10;  
      v0->halfedge() = h0;
      v1->halfedge() = h3;
      v2->halfedge() = h12;
      v3->halfedge() = h14;

      // edges
      e0->halfedge() = h0;
      e1->halfedge() = h1;
      e2->halfedge() = h2;
      e3->halfedge() = h4;
      e4->halfedge() = h5;
      e5->halfedge() = h3;
      e6->halfedge() = h14;
      e7->halfedge() = h12;

      // faces
      f0->halfedge() = h0;
      f1->halfedge() = h3;
      f2->halfedge() = h11;
      f3->halfedge() = h10;

      return m;
    }
    // return e0->halfedge()->vertex();

    // boundary 
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // Each vertex and edge of the original surface can be associated with a vertex in the new (subdivided) surface.
    // Therefore, our strategy for computing the subdivided vertex locations is to *first* compute the new positions
    // using the connectity of the original (coarse) mesh; navigating this mesh will be much easier than navigating
    // the new subdivided (fine) mesh, which has more elements to traverse. We will then assign vertex positions in
    // the new mesh based on the values we computed for the original mesh.


    // TODO Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // TODO and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // TODO a vertex of the original mesh.


    // TODO Next, compute the updated vertex positions associated with edges, and store it in Edge::newPosition.


    // TODO Next, we're going to split every edge in the mesh, in any order.  For future
    // TODO reference, we're also going to store some information about which subdivided
    // TODO edges come from splitting an edge in the original mesh, and which edges are new,
    // TODO by setting the flat Edge::isNew.  Note that in this loop, we only want to iterate
    // TODO over edges of the original mesh---otherwise, we'll end up splitting edges that we
    // TODO just split (and the loop will never end!)


    // TODO Now flip any new edge that connects an old and new vertex.


    // TODO Finally, copy the new vertex positions into final Vertex::position.



// TODO Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
// TODO and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
// TODO a vertex of the original mesh.

    Vector3D ori_p;
    Vector3D neighbor_p;
    for(VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {

// update old vertices' position->new position
      ori_p = v->position;
      neighbor_p = Vector3D(0,0,0);

      // find neighbor positions
      HalfedgeCIter h = v->halfedge();
      int num_neighbor = 0;
      do {
        HalfedgeCIter h_twin = h->twin();
        neighbor_p += h_twin->vertex()->position;
        h = h_twin->next();
        num_neighbor += 1;
      } while (h != v->halfedge());

      // find u, n
      double u = 0;
      if (num_neighbor == 3) {
        u = 3.0 / 16.0;
      }
      else {
        u = 3.0 / (8.0 * num_neighbor);
      }

      // update old vertex's new position
      v->isNew = false;
      v->newPosition = (1 - num_neighbor * u) * ori_p + u * neighbor_p;
    }


// TODO Next, compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
    EdgeIter e = mesh.edgesBegin();
    Vector3D AB(0,0,0);   // new vertex halfedge
    Vector3D CD(0,0,0);   // new vertex's new edges
    while (e != mesh.edgesEnd()) {
      EdgeIter nextEdge = e;
      nextEdge++;

      AB = (e->halfedge()->vertex()->position + e->halfedge()->twin()->vertex()->position);
      CD = (e->halfedge()->next()->next()->vertex()->position + e->halfedge()->twin()->next()->next()->vertex()->position);

      e->newPosition = 3.0/8.0 * AB + 1.0/8.0 * CD;

      e->isNew = false;
      e = nextEdge;
    }
    

    // TODO Next, we're going to split every edge in the mesh, in any order.  For future
    // TODO reference, we're also going to store some information about which subdivided
    // TODO edges come from splitting an edge in the original mesh, and which edges are new,
    // TODO by setting the flat Edge::isNew.  Note that in this loop, we only want to iterate
    // TODO over edges of the original mesh---otherwise, we'll end up splitting edges that we
    // TODO just split (and the loop will never end!)

// split edge & create new vertices
    Vector3D midpoint(0,0,0);
    EdgeIter e1 = mesh.edgesBegin();
    while (e1 != mesh.edgesEnd()) {
      EdgeIter nextEdge1 = e1;
      nextEdge1++;

      if (!e1->halfedge()->vertex()->isNew && !e1->halfedge()->twin()->vertex()->isNew) {
        midpoint = e1->newPosition;
        VertexIter new_v = mesh.splitEdge(e1);
        new_v->newPosition = midpoint;  // update new vertex position now???
      }
      e1 = nextEdge1;
    }

// TODO Now flip any new edge that connects an old and new vertex.
    EdgeIter e2 = mesh.edgesBegin();
    while (e2 != mesh.edgesEnd()) {
      EdgeIter nextEdge2 = e2;
      nextEdge2++;

      if (e2->isNew) {
        bool v1 = e2->halfedge()->vertex()->isNew;
        bool v2 = e2->halfedge()->twin()->vertex()->isNew;
        if ((v1 && !v2) || (!v1 && v2)) {
          mesh.flipEdge(e2);
        }
      }
      e2 = nextEdge2;
    }

// TODO Finally, copy the new vertex positions into final Vertex::position.
    for(VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
      v->position = v->newPosition;
    }
  }

}







