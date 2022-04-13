#include "QuadricDecimationMesh.h"
#include <VC++/glm/gtx/string_cast.hpp>

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
    size_t v1 = e(collapse->halfEdge).vert;
    size_t v2 = e(e(collapse->halfEdge).next).vert;

    glm::vec4 posV1(v(v1).pos,1.0f);
    glm::vec4 posV2(v(v2).pos, 1.0f);
    glm::vec4 between((posV1 + posV2) *0.5f);


    //std::cerr << glm::to_string(posV1)  << std::endl;

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
    //Qhat = glm::transpose(Qhat);
    // A square matrix is singular(=invertible) if and only if its determinant is not zero.
    float EPSILON = 0.0000000001f;
    bool notInvertible = abs(glm::determinant(Qhat)) < EPSILON;
    float costV1 = glm::dot(posV1, Q * posV1); 
    float costV2 = glm::dot(posV2, Q * posV2); 
    float costBetween = glm::dot(between, Q * between); 

    if (!notInvertible) {
        glm::vec4 v = glm::inverse(Qhat)*zero;
        collapse->position = glm::vec3(v);
    } 
    else {
        // compare costfunction for other 3 cases
        //get the one with lowest cost
        if (costV1 < costV2 && costV1 < costBetween) {
            collapse->position = posV1;
        } else if (costV2 < costV1 && costV2 < costBetween) {
            collapse->position = posV2;
        } else {
        collapse->position = between;
       }

    }
    glm::vec4 position(collapse->position, 1.0f); 
    // error function
    // multiply transposed vector with vector = dotproduct
    float deltaV = glm::dot(position, Q * position); 
    collapse->cost = deltaV;
    //std::cerr << "computeCollapse in QuadricDecimationMesh not implemented.\n";
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

    std::vector<size_t> V_ring = FindNeighborFaces(indx);

    for (size_t f_index : V_ring) {

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

    //compute one matrix Kpi from each plane and sum up in createQuadricForVert
    // P , (outer product)
    //take the planes that meet at the vertex
    //planets ekvation
    glm::vec3 plane = f(indx).normal;
    a = plane[0];
    b = plane[1];
    c = plane[2];
    size_t face = f(indx).edge;
    size_t edge = e(face).vert;
    glm::vec3 vert = v(edge).pos;

    d = -1 * glm::dot(plane,vert);
    //Kpi
    glm::mat4 K(
        {a*a, a*b, a*c, a*d},
        {a*b, b*b, b*c, b*d}, 
        {c*a, c*b, c*c, c*d},
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
