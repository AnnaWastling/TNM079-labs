#include "QuadricDecimationMesh.h"

const QuadricDecimationMesh::VisualizationMode QuadricDecimationMesh::QuadricIsoSurfaces =
    NewVisualizationMode("Quadric Iso Surfaces");

void QuadricDecimationMesh::Initialize() {
    // Allocate memory for the quadric array
    size_t numVerts = mVerts.size();
    mQuadrics.reserve(numVerts);
    std::streamsize width = std::cerr.precision();  // store stream precision
    for (size_t i = 0; i < numVerts; i++) {

        // Compute quadric for vertex i here
        mQuadrics.push_back(createQuadricForVert(i));

        // Calculate initial error, should be numerically close to 0

        glm::vec3 v0 = mVerts[i].pos;
        glm::vec4 v(v0[0], v0[1], v0[2], 1);
        auto m = mQuadrics.back();

        // TODO CHECK
        auto error = glm::dot(v, (m * v));
        // std::cerr << std::scientific << std::setprecision(2) << error << " ";
    }
    std::cerr << std::setprecision(width) << std::fixed;  // reset stream precision

    // Run the initialize for the parent class to initialize the edge collapses
    DecimationMesh::Initialize();
}

/*! \lab2 Implement the computeCollapse here */
/*!
 * \param[in,out] collapse The edge collapse object to (re-)compute,
 * DecimationMesh::EdgeCollapse
 */
void QuadricDecimationMesh::computeCollapse(EdgeCollapse* collapse) {
    // Compute collapse->position and collapse->cost here
    // based on the quadrics at the edge endpoints
    auto v1 = e(collapse->halfEdge).vert;
    auto v2 = e(e(collapse->halfEdge).next).vert;

    auto Q1 = createQuadricForVert(v1);
    auto Q2 = createQuadricForVert(v2);
    auto Q = Q1 + Q2;
    //find a collapse->position for v which minimizes the error
    auto Qhat = Q;
    Qhat[0][3] = 0.0f;
    Qhat[1][3] = 0.0f;
    Qhat[2][3] = 0.0f;
    Qhat[3][3] = 1.0f;
    glm::vec4 zero(0.0f, 0.0f, 0.0f, 1.0f);
    
    //error
    // A square matrix is singular(=invertible) if and only if its determinant is zero.
    float EPSILON = 0.0000000001f;
    //if (glm::determinant(Qhat) =! EPSILON) {
    //    glm::vec4 v = glm::inverse(Qhat)*zero;
    //    collapse->position = v;

    //    //error function
    //    //multiply transposed vector with vector = dotproduct 
    //    float deltaV = glm::dot(v, Qhat * v); 
    //    collapse->cost = deltaV;
    //} else {
    //    collapse->position = glm::vec3((v1 + v2) / 2);
    //}

    std::cerr << "computeCollapse in QuadricDecimationMesh not implemented.\n";
}

/*! After each edge collapse the vertex properties need to be updated */
void QuadricDecimationMesh::updateVertexProperties(size_t ind) {
    DecimationMesh::updateVertexProperties(ind);
    mQuadrics[ind] = createQuadricForVert(ind);
}

/*!
 * \param[in] indx vertex index, points into HalfEdgeMesh::mVerts
 */
glm::mat4 QuadricDecimationMesh::createQuadricForVert(size_t indx) const {
    glm::mat4 Q({0.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f},
                {0.0f, 0.0f, 0.0f, 0.0f});
    std::vector<size_t> V_planes = FindNeighborFaces(indx);

    for (auto f_index : V_planes) {
      Q += createQuadricForFace(f_index);
    }
    

    return Q;
}

/*!
 * \param[in] indx face index, points into HalfEdgeMesh::mFaces
 */
glm::mat4 QuadricDecimationMesh::createQuadricForFace(size_t indx) const {

    // Calculate the quadric (outer product of plane parameters) for a face
    // here using the formula from Garland and Heckbert
    float a, b, c, d;
    // The quadric for a vertex is the sum of all the quadrics for the adjacent
    // faces Tip: Matrix4x4 has an operator +=

    // P
    glm::vec3 plane = f(indx).normal;
    a = plane[0];
    b = plane[1];
    c = plane[2];
    auto face = f(indx).edge;
    auto edge = e(face).vert;
    auto vert = v(edge).pos;

    d = -1 * glm::dot(vert, plane);
    //Kpi
    glm::mat4 K({a*a, a*b, a*c, a*d}, {a*b, b*b, b*c, b*d}, {c*a, c*b, c*c, c*d},
                {a*d, b*d, c*d, d*d});
    return K;
}

void QuadricDecimationMesh::Render() {
    DecimationMesh::Render();

    glEnable(GL_LIGHTING);
    glMatrixMode(GL_MODELVIEW);

    if (mVisualizationMode == QuadricIsoSurfaces) {
        // Apply transform
        glPushMatrix();  // Push modelview matrix onto stack

        // Implement the quadric visualization here
        std::cout << "Quadric visualization not implemented" << std::endl;

        // Restore modelview matrix
        glPopMatrix();
    }
}
