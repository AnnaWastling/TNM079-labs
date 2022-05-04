#pragma once

#include "Levelset/LevelSetOperator.h"
#include "Math/Function3D.h"
#include "Levelset/LevelSet.h"

/*! \brief A level set operator that does external advection
 *
 * This class implements level set advectionr in an external vector field by the
 * PDE
 *
 *  \f$
 *  \dfrac{\partial \phi}{\partial t} + \mathbf{V}(\mathbf{x})\cdot \nabla \phi
 * = 0 \f$
 */
//! \lab4 Implement advection in external vector field
class OperatorAdvect : public LevelSetOperator {
protected:
    Function3D<glm::vec3> *mVectorField;

public:
    OperatorAdvect(LevelSet *LS, Function3D<glm::vec3> *vf)
        : LevelSetOperator(LS), mVectorField(vf) {}

    virtual float ComputeTimestep() {
        // Compute and return a stable timestep
        // (Hint: Function3D::GetMaxValue())
        float timestep = mLS->GetDx();
        glm::vec3 max(mVectorField->GetMaxValue());
        float V = std::max(max.x, max.y); //need the maximum value to get the direction of V
        V = std::max(V, max.z);
       
        return timestep/V;
    }

    virtual void Propagate(float time) {
        // Determine timestep for stability
        float dt = ComputeTimestep();

        // Propagate level set with stable timestep dt
        // until requested time is reached
        for (float elapsed = 0; elapsed < time;) {
            if (dt > time - elapsed) dt = time - elapsed;
            elapsed += dt;

            IntegrateEuler(dt);
            // IntegrateRungeKutta(dt);
        }
    }

    virtual float Evaluate(size_t i, size_t j, size_t k) {
        float x = i;
        float y = j;
        float z = k;
        mLS -> TransformGridToWorld(x, y, z);
        

        // Compute the rate of change (dphi/dt)
        glm::vec3 v = mVectorField->GetValue(x, y, z);

        glm::vec3 gradient; 

        if (v.x < 0) {
            gradient.x = mLS->DiffXp(i, j, k);
        } else {
            gradient.x = mLS->DiffXm(i, j, k);
        }

        if (v.y < 0) {
            gradient.y = mLS->DiffYp(i, j, k);
        } else {
            gradient.y = mLS->DiffYm(i, j, k);
        }

        if (v.z < 0) {
            gradient.z = mLS->DiffZp(i, j, k);
        } else {
            gradient.z = mLS->DiffZm(i, j, k);
        }

        // Remember that the point (i,j,k) is given in grid coordinates, while
        // the velocity field used for advection needs to be sampled in
        // world coordinates (x,y,z). You can use LevelSet::TransformGridToWorld()
        // for this task.
        return glm::dot(-v, gradient);
    }
};