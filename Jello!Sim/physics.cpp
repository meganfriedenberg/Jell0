#include "jello.h"
#include "physics.h"

/* Computes acceleration to every control point of the jello cube, 
   which is in state given by 'jello'.
   Returns result in array 'a'. */


// FOR EACH struct of springs
// // <vector>* of springs 

void handleCollision(struct world* jello, struct point totalForce[8][8][8], point& forcePoint, point& collisionPoint, int& collSide, point& currVelocity);

// given a list of springs that have kHook as a member variable, jello, and the accumulated forces, compute the hook force for each spring, then add it to the accumulated forces
void computeKHook(struct world* jello, struct point totalForce[8][8][8], std::vector<spring>* springList)
{
    // kHook equation is: -kHook(length(vector(A-B)) - restLength) * (normalize(A-B))
    //double kHook = jello->kElastic;
    point fromBtoA;
    point hookForce;
    double length;

    for (int i = 0; i < springList->size(); i++) // loop thru all springs
    {
        double kHook = springList->at(i).kHook;
        if (kHook == 0.0)
        {
            kHook = jello->kElastic;
        }

        hookForce.x = 0.0;
        hookForce.y = 0.0;
        hookForce.z = 0.0;

        int p1XVal = (springList->at(i).p1).x;
        int p1YVal = (springList->at(i).p1).y;
        int p1ZVal = (springList->at(i).p1).z;

        int p2XVal = (springList->at(i).p2).x;
        int p2YVal = (springList->at(i).p2).y;
        int p2ZVal = (springList->at(i).p2).z;


        pDIFFERENCE(jello->p[p1XVal][p1YVal][p1ZVal], jello->p[p2XVal][p2YVal][p2ZVal], fromBtoA); // the vector from B to A is A-B
        if(!(fromBtoA.x == 0 && fromBtoA.y == 0 && fromBtoA.z == 0)) // prevent divide by zero error
            pNORMALIZE(fromBtoA);

        // getting kHook * (len(vector(A-B)) - restLength
        double firstProduct = (length - springList->at(i).restingLength) * (-1) * (kHook);
        pMULTIPLY(fromBtoA, firstProduct, hookForce);

        // hookForce now stores the force exerted on point A
        point a = springList->at(i).p1;
        point b = springList->at(i).p2;
        totalForce[(int)(a.x)][(int)(a.y)][(int)(a.z)].x += hookForce.x;
        totalForce[(int)(a.x)][(int)(a.y)][(int)(a.z)].y += hookForce.y;
        totalForce[(int)(a.x)][(int)(a.y)][(int)(a.z)].z += hookForce.z;

        // by negating the force, its the force for point B
        totalForce[(int)(b.x)][(int)(b.y)][(int)(b.z)].x -= hookForce.x;
        totalForce[(int)(b.x)][(int)(b.y)][(int)(b.z)].y -= hookForce.y;
        totalForce[(int)(b.x)][(int)(b.y)][(int)(b.z)].z -= hookForce.z;
    }
}
// compute the dampening force for a list of given springs and add the dampening forces to the accumulating forces
void computeKDamp(struct world* jello, struct point totalForce[8][8][8], std::vector<spring>* springList)
{
    // L is the vector from BtoA so (A-B)
    // vA and vB are the velocities, not the vector
    // kDamping equation is: -kDamping( ((vA-vB).dot(L)) / len(L) ) * normalized(L)

    double length;
    point fromBtoA;
    point velocity;
    point dampForce;

    for (int i = 0; i < springList->size(); i++) // loop thru all springs
    {
        double kDamp = springList->at(i).kD;
        if (kDamp == 0.0)
        {
            kDamp = jello->dElastic;
        }


        dampForce.x = 0.0;
        dampForce.y = 0.0;
        dampForce.z = 0.0;

        int p1XVal = (springList->at(i).p1).x;
        int p1YVal = (springList->at(i).p1).y;
        int p1ZVal = (springList->at(i).p1).z;

        int p2XVal = (springList->at(i).p2).x;
        int p2YVal = (springList->at(i).p2).y;
        int p2ZVal = (springList->at(i).p2).z;

 
        pDIFFERENCE(jello->v[p1XVal][p1YVal][p1ZVal], jello->v[p2XVal][p2YVal][p2ZVal], velocity); // vA - vB
        pDIFFERENCE(jello->p[p1XVal][p1YVal][p1ZVal], jello->p[p2XVal][p2YVal][p2ZVal], fromBtoA); // A-B vector

        // dot product (vA-vB) dot L
        double dotProduct;
        pDOT(velocity, fromBtoA, dotProduct);

        // normalize L before dividing, so we have the length variable to use
        pNORMALIZE(fromBtoA);

        pMULTIPLY(fromBtoA, (-kDamp * (dotProduct / length)), dampForce);

        pNORMALIZE(fromBtoA);


        // dampForce now stores the force exerted on point A
        // negate dampForce for point B
        point a = springList->at(i).p1;
        point b = springList->at(i).p2;
        totalForce[(int)(a.x)][(int)(a.y)][(int)(a.z)].x += dampForce.x;
        totalForce[(int)(a.x)][(int)(a.y)][(int)(a.z)].y += dampForce.y;
        totalForce[(int)(a.x)][(int)(a.y)][(int)(a.z)].z += dampForce.z;

        totalForce[(int)(b.x)][(int)(b.y)][(int)(b.z)].x -= dampForce.x;
        totalForce[(int)(b.x)][(int)(b.y)][(int)(b.z)].y -= dampForce.y;
        totalForce[(int)(b.x)][(int)(b.y)][(int)(b.z)].z -= dampForce.z;
    }

}

void testWallCollision(struct world* jello, struct point totalForce[8][8][8], point& collisionPoint, int& collSide)
{
    // jello has all the positions in p
    // the box bounds are [-2,2]x[-2,2]x[-2,2]
   
    // could be the bullet thru the paper problem, so test ALL points, since where does the collision actually happen?
    // would the top layer detect it or would 1-2 layers in detect?

    for (int i = 0; i < 8; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            for (int k = 0; k < 8; k++) // handle collision has to be called in every instance here, since needs to be checked on every mass point
            {
                point forcePoint;
                forcePoint.x = i;
                forcePoint.y = j;
                forcePoint.z = k;


                point currPos = jello->p[i][j][k];
                point currVelocity = jello->v[i][j][k];
                if (currPos.x > 2 || currPos.x < -2) // collided on the x axis
                {
                    collSide = 1;
                    collisionPoint = currPos;
                    handleCollision(jello, totalForce, forcePoint, collisionPoint, collSide, currVelocity);

                }
                if (currPos.y > 2 || currPos.y < -2) // collided on the y axis
                {
                    collSide = 2;
                    collisionPoint = currPos;
                    handleCollision(jello, totalForce, forcePoint, collisionPoint, collSide, currVelocity);
                }
                if (currPos.z > 2 || currPos.z < -2) // collided on the z axis
                {
                    collSide = 3;
                    collisionPoint = currPos;
                    handleCollision(jello, totalForce, forcePoint, collisionPoint, collSide, currVelocity);
                }

            }
            // if handle collision is called here,then rows are skipped
        }
    }

}
// computing the kHook collision force for a single spring, given the forcePoint is the masspoint experiencing the collision
void computeKHookCollision(struct world* jello, struct point totalForce[8][8][8], point& forcePoint, spring collisionSpring)
{
    double length;
    point fromBtoA;

    point hookForce;
    hookForce.x = 0.0;
    hookForce.y = 0.0;
    hookForce.z = 0.0;

    pDIFFERENCE(collisionSpring.p1, collisionSpring.p2, fromBtoA);
    if (!(fromBtoA.x == 0 && fromBtoA.y == 0 && fromBtoA.z == 0)) // double check so no divide by zero error
        pNORMALIZE(fromBtoA);

    double firstProduct = (length - collisionSpring.restingLength) * (-1) * (collisionSpring.kHook);
    pMULTIPLY(fromBtoA, firstProduct, hookForce);

    
    totalForce[(int)forcePoint.x][(int)forcePoint.y][(int)forcePoint.z].x += hookForce.x;
    totalForce[(int)forcePoint.x][(int)forcePoint.y][(int)forcePoint.z].y += hookForce.y;
    totalForce[(int)forcePoint.x][(int)forcePoint.y][(int)forcePoint.z].z += hookForce.z;


}

// computing the kDamp collision force for a single spring, given the forcePoint is the masspoint experiencing the collision
void computeKDampCollision(struct world* jello, struct point totalForce[8][8][8], point& forcePoint, spring collisionSpring, point& currVelocity)
{
    double length;
    point fromBtoA;
    point velocity;

    point vA = currVelocity; // velocity going INTO the collision spring when we touch

    point vB; 
    vB.x = 1;
    vB.y = 1;
    vB.z = 1; // init velocity with 1, modify this for different type of simulations

    point dampForce;
    dampForce.x = 0.0;
    dampForce.y = 0.0;
    dampForce.z = 0.0;

    pDIFFERENCE(vA, vB, velocity); // vA - vB
    pDIFFERENCE(collisionSpring.p1, collisionSpring.p2, fromBtoA); // computing L, A-B

    // dot product (vA-vB) dot L
    double dotProduct;
    pDOT(velocity, fromBtoA, dotProduct);

    // normalize L before dividing
    pNORMALIZE(fromBtoA); // will ensure normal is proper direction

    pMULTIPLY(fromBtoA, (-collisionSpring.kD * (dotProduct / length)), dampForce);

    // magnitude is proportional to the amount of penetration

    totalForce[(int)forcePoint.x][(int)forcePoint.y][(int)forcePoint.z].x += dampForce.x;
    totalForce[(int)forcePoint.x][(int)forcePoint.y][(int)forcePoint.z].y += dampForce.y;
    totalForce[(int)forcePoint.x][(int)forcePoint.y][(int)forcePoint.z].z += dampForce.z;

}

// given the collision point, the mass spring point that experienced the collision, the axis of collision, and the total forces, calculate the collision forces for that one mass point
void handleCollision(struct world* jello, struct point totalForce[8][8][8], point& forcePoint, point& collisionPoint, int& collSide, point& currVelocity)
{
    spring penaltySpring;
    penaltySpring.restingLength = 0.0;
    penaltySpring.kHook = jello->kCollision;
    penaltySpring.kD = jello->dCollision;

    point intersection = collisionPoint;
    point totalCollForce;
    totalCollForce.x = 0.0;
    totalCollForce.y = 0.0;
    totalCollForce.z = 0.0;

    if (collSide == 1) // x axis
    {
        collisionPoint.x = (int)collisionPoint.x;
    }
    else if (collSide == 2) // y axis
    {
        collisionPoint.y = (int)collisionPoint.y;
    }
    else if (collSide == 3) // z axis
    {
        collisionPoint.z = (int)collisionPoint.z;
    }
    else
    {
        return;
    }
    penaltySpring.p1 = intersection;
    penaltySpring.p2 = collisionPoint;

    // needs to calculate the damping/hook collision forces
    std::vector<spring> temp; // create the collision spring
    temp.push_back(penaltySpring);

    // calculate collision forces and add them to totalForce
    computeKHookCollision(jello, totalForce, forcePoint, penaltySpring);
    computeKDampCollision(jello, totalForce, forcePoint, penaltySpring, currVelocity);


}

// find the closest force field cube point based on the current mass point and the passed in resolution
void getClosestForceFieldPoint(struct world* jello, point jelloPoint, point& forceFieldPoint)
{
    forceFieldPoint.x = 0;
    forceFieldPoint.y = 0;
    forceFieldPoint.z = 0;

    double forceFieldCubeLength =  (1.0 / ((jello->resolution) - 1)) * 4; // a 1x1x1 cube has length 4

    if (jelloPoint.x >= -2 && jelloPoint.x <= 2) // inside the bounding box, so which "cube" is it 
    {
        forceFieldPoint.x = (int)(floor((jelloPoint.x + 2) / forceFieldCubeLength));
    }
    else if (jelloPoint.x > 2) // if it happens to be less than -2, keep it zero, since force field is only going to be INSIDE the bounding box
    {
        // if outside the bounding box, give the largest possible value
        forceFieldPoint.x = (int)((1.0 / forceFieldCubeLength) * 4);
    }
    // prevent indexing out of bounds on jello->forceField
    if (forceFieldPoint.x >= jello->resolution - 1)
    {
        forceFieldPoint.x = jello->resolution - 2;
    }
    if (forceFieldPoint.x < 0)
    {
        forceFieldPoint.x = 0;
    }


    // now repeat for .y and .z
    if (jelloPoint.y >= -2 && jelloPoint.y <= 2)
    {
        forceFieldPoint.y = (int)(floor((jelloPoint.y + 2) / forceFieldCubeLength));
    }
    else if (jelloPoint.y > 2)
    {
        // if outside the bounding box, give the largest possible value
        forceFieldPoint.y = (int)((1.0 / forceFieldCubeLength) * 4);
    }
    // prevent indexing out of bounds on jello->forceField
    if (forceFieldPoint.y >= jello->resolution - 1)
    {
        forceFieldPoint.y = jello->resolution - 2;
    }
    if (forceFieldPoint.y < 0)
    {
        forceFieldPoint.y = 0;
    }


    if (jelloPoint.z >= -2 && jelloPoint.z <= 2)
    {
        forceFieldPoint.z = (int)(floor((jelloPoint.z + 2) / forceFieldCubeLength));
    }
    else if (jelloPoint.z > 2)
    {
        // if outside the bounding box, give the largest possible value
        forceFieldPoint.z = (int)((1.0 / forceFieldCubeLength) * 4);
    }
    // prevent indexing out of bounds on jello->forceField
    if (forceFieldPoint.z >= jello->resolution - 1)
    {
        forceFieldPoint.z = jello->resolution - 2;
    }
    if (forceFieldPoint.z < 0)
    {
        forceFieldPoint.z = 0;
    }
}


// compute the force field forces for the 512 mass points
void computeForceField(struct world* jello, struct point totalForce[8][8][8])
{
    // need to use trilinear interpolation
    point interpolationPoints[8];
    /*point a000; // 0
    point a100; // 1
    point a110; // 2
    point a010; // 3
    point a001; // 4
    point a101; // 5
    point a111; // 6
    point a011;*/ // 7

    point closestForceFieldPoint;

    double alpha;
    double beta;
    double gamma;

    double forceFieldCubeLength = (1.0 / ((jello->resolution) - 1)) * 4; // by Piazza post @60_f1, a 1x1x1 cube has length 4


    for (int i = 0; i < 8; i++) // init all to 0
    {
        point currPoint;
        currPoint.x = 0;
        currPoint.y = 0;
        currPoint.z = 0;
        interpolationPoints[i] = currPoint;
    }


    for (int i = 0; i < 8; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            for (int k = 0; k < 8; k++) // yes we have 512 mass points, but not 512 force field points. need to find the closest point to where we are
            {

                getClosestForceFieldPoint(jello, jello->p[i][j][k], closestForceFieldPoint);

                // get the corners based on the closest force field point (ow this made my head hurt)
                double xLow = (closestForceFieldPoint.x * forceFieldCubeLength) -2;
                double yLow = (closestForceFieldPoint.y * forceFieldCubeLength) - 2;
                double zLow = (closestForceFieldPoint.x * forceFieldCubeLength) - 2;
                
                // how to get vectors from the force field vector list:
                //      "The force field datapoint (i,j,k) is stored in memory at 
                //      jello->forceField[i * jello->resolution * jello->resolution + j * jello->resolution + k]"
                // set a000
                int index = ((closestForceFieldPoint.x) * jello->resolution * jello->resolution) + ((closestForceFieldPoint.y) * jello->resolution) + closestForceFieldPoint.z;
                interpolationPoints[0] = jello->forceField[index];

                // set a100
                index = ((closestForceFieldPoint.x + 1) * jello->resolution * jello->resolution) + ((closestForceFieldPoint.y) * jello->resolution) + closestForceFieldPoint.z;
                interpolationPoints[1] = jello->forceField[index];

                // set a110
                index = ((closestForceFieldPoint.x + 1) * jello->resolution * jello->resolution) + ((closestForceFieldPoint.y+1) * jello->resolution) + closestForceFieldPoint.z;
                interpolationPoints[2] = jello->forceField[index];

                // set a010
                index = ((closestForceFieldPoint.x) * jello->resolution * jello->resolution) + ((closestForceFieldPoint.y+1) * jello->resolution) + closestForceFieldPoint.z;
                interpolationPoints[3] = jello->forceField[index];

                // set a001
                index = ((closestForceFieldPoint.x) * jello->resolution * jello->resolution) + ((closestForceFieldPoint.y) * jello->resolution) + closestForceFieldPoint.z+1;
                interpolationPoints[4] = jello->forceField[index];

                // set a101
                index = ((closestForceFieldPoint.x+1) * jello->resolution * jello->resolution) + ((closestForceFieldPoint.y) * jello->resolution) + closestForceFieldPoint.z+1;
                interpolationPoints[5] = jello->forceField[index];

                // set a111
                index = ((closestForceFieldPoint.x+1) * jello->resolution * jello->resolution) + ((closestForceFieldPoint.y+1) * jello->resolution) + closestForceFieldPoint.z+1;
                interpolationPoints[6] = jello->forceField[index];

                // set a011
                index = ((closestForceFieldPoint.x) * jello->resolution * jello->resolution) + ((closestForceFieldPoint.y + 1) * jello->resolution) + closestForceFieldPoint.z + 1;
                interpolationPoints[7] = jello->forceField[index];

                // formula is f1=(x-xLow)/(xHigh-xLow) , but in xHigh-xLow only the length doesnt get canceled out
                alpha = (jello->p[i][j][k].x - xLow) / forceFieldCubeLength;
                beta = (jello->p[i][j][k].y - yLow) / forceFieldCubeLength;
                gamma = (jello->p[i][j][k].z - zLow) / forceFieldCubeLength;

                // the trilinear interpolation equations from class

                pMULTIPLY(interpolationPoints[0], ((1-alpha)*(1-beta)*(1-gamma)), interpolationPoints[0]); // a000
                pMULTIPLY(interpolationPoints[1], ((alpha) * (1 - beta) * (1 - gamma)), interpolationPoints[1]); // a100
                pMULTIPLY(interpolationPoints[2], ((alpha) * (beta) * (1 - gamma)), interpolationPoints[2]); // a110
                pMULTIPLY(interpolationPoints[3], ((1 - alpha) * (beta) * (1 - gamma)), interpolationPoints[3]); // a010
                pMULTIPLY(interpolationPoints[4], ((1 - alpha) * (1 - beta) * (gamma)), interpolationPoints[4]); // a001
                pMULTIPLY(interpolationPoints[5], ((alpha) * (1 - beta) * (gamma)), interpolationPoints[5]); // a101
                pMULTIPLY(interpolationPoints[6], ((alpha) * (beta) * (gamma)), interpolationPoints[6]); // a111
                pMULTIPLY(interpolationPoints[7], ((1 - alpha) * (beta) * (gamma)), interpolationPoints[7]); // a011

                // then add all the new forces

                double temp = totalForce[i][j][k].x;
                double temp2 = totalForce[i][j][k].y;
                double temp3 = totalForce[i][j][k].z;

                totalForce[i][j][k].x += (interpolationPoints[0].x + interpolationPoints[1].x + interpolationPoints[2].x + interpolationPoints[3].x
                    + interpolationPoints[4].x + interpolationPoints[5].x + interpolationPoints[6].x + interpolationPoints[7].x);
                totalForce[i][j][k].y += (interpolationPoints[0].y + interpolationPoints[1].y + interpolationPoints[2].y + interpolationPoints[3].y
                    + interpolationPoints[4].y + interpolationPoints[5].y + interpolationPoints[6].y + interpolationPoints[7].y);
                totalForce[i][j][k].z += (interpolationPoints[0].z + interpolationPoints[1].z + interpolationPoints[2].z + interpolationPoints[3].z
                    + interpolationPoints[4].z + interpolationPoints[5].z + interpolationPoints[6].z + interpolationPoints[7].z);

                // below is code to test in debug mode (breakpoint on 488 and 493) to see the forceField values

                //if (temp != totalForce[i][j][k].x || temp2 != totalForce[i][j][k].y || temp3 != totalForce[i][j][k].z)
                //{
                //    int abc = 1;
                //}

                //if (index > 27050 && index < 28000)
                //{
                //    int here = 5;
                //}

            }
        }
    }

}


void computeAcceleration(struct world * jello, struct point a[8][8][8], struct structuralSprings* structSpringList, struct bendSprings* bendSpringList, struct shearSprings* shearSpringList)
{
    // acceleration is acc = (total force/mass), or total force * (1/mass)

    // initialize force to 0
    point totalForce[8][8][8];
    for (int i = 0; i < 8; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            for (int k = 0; k < 8; k++)
            {
                totalForce[i][j][k].x = 0.0f;
                totalForce[i][j][k].y = 0.0f;
                totalForce[i][j][k].z = 0.0f;
            }
        }
    }

    // need to calculate F(hook) and F(dampening)
    computeKHook(jello, totalForce, structSpringList->springs);
    computeKHook(jello, totalForce, shearSpringList->springs);
    computeKHook(jello, totalForce, bendSpringList->springs);

    computeKDamp(jello, totalForce, structSpringList->springs);
    computeKDamp(jello, totalForce, shearSpringList->springs);
    computeKDamp(jello, totalForce, bendSpringList->springs);

    if (jello->resolution != 0) // see if there is a force field, value of 0 means there isn't one
    {
        computeForceField(jello, totalForce);
    }


    point collisionPoint;
    int collSide;
    testWallCollision(jello, totalForce, collisionPoint, collSide);

    

    // last step which is total force / mass to get acc
    for (int i = 0; i < 8; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            for (int k = 0; k < 8; k++)
            {
                a[i][j][k].x = (totalForce[i][j][k].x) * ((1.0f) / jello->mass);
                a[i][j][k].y = (totalForce[i][j][k].y) * ((1.0f) / jello->mass);
                a[i][j][k].z = (totalForce[i][j][k].z) * ((1.0f) / jello->mass);
            }
        }
    }



}

/* performs one step of Euler Integration */
/* as a result, updates the jello structure */
void Euler(struct world * jello, struct structuralSprings * structSpringList, struct bendSprings * bendSpringList, struct shearSprings * shearSpringList)
{
  int i,j,k;
  point a[8][8][8];

  computeAcceleration(jello, a, structSpringList, bendSpringList, shearSpringList);
  
  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
        jello->p[i][j][k].x += jello->dt * jello->v[i][j][k].x;
        jello->p[i][j][k].y += jello->dt * jello->v[i][j][k].y;
        jello->p[i][j][k].z += jello->dt * jello->v[i][j][k].z;
        jello->v[i][j][k].x += jello->dt * a[i][j][k].x;
        jello->v[i][j][k].y += jello->dt * a[i][j][k].y;
        jello->v[i][j][k].z += jello->dt * a[i][j][k].z;

      }
}

/* performs one step of RK4 Integration */
/* as a result, updates the jello structure */
void RK4(struct world * jello, struct structuralSprings * structSpringList, struct bendSprings * bendSpringList, struct shearSprings * shearSpringList)
{
  point F1p[8][8][8], F1v[8][8][8], 
        F2p[8][8][8], F2v[8][8][8],
        F3p[8][8][8], F3v[8][8][8],
        F4p[8][8][8], F4v[8][8][8];

  point a[8][8][8];


  struct world buffer;

  int i,j,k;

  buffer = *jello; // make a copy of jello

  computeAcceleration(jello, a, structSpringList, bendSpringList, shearSpringList);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         pMULTIPLY(jello->v[i][j][k],jello->dt,F1p[i][j][k]);
         pMULTIPLY(a[i][j][k],jello->dt,F1v[i][j][k]);
         pMULTIPLY(F1p[i][j][k],0.5,buffer.p[i][j][k]);
         pMULTIPLY(F1v[i][j][k],0.5,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }

  computeAcceleration(&buffer, a, structSpringList, bendSpringList, shearSpringList);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F2p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F2p[i][j][k]);
         // F2v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F2v[i][j][k]);
         pMULTIPLY(F2p[i][j][k],0.5,buffer.p[i][j][k]);
         pMULTIPLY(F2v[i][j][k],0.5,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }

  computeAcceleration(&buffer, a, structSpringList, bendSpringList, shearSpringList);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F3p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F3p[i][j][k]);
         // F3v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F3v[i][j][k]);
         pMULTIPLY(F3p[i][j][k],1.0,buffer.p[i][j][k]);
         pMULTIPLY(F3v[i][j][k],1.0,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }
         
  computeAcceleration(&buffer, a, structSpringList, bendSpringList, shearSpringList);


  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F3p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F4p[i][j][k]);
         // F3v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F4v[i][j][k]);

         pMULTIPLY(F2p[i][j][k],2,buffer.p[i][j][k]);
         pMULTIPLY(F3p[i][j][k],2,buffer.v[i][j][k]);
         pSUM(buffer.p[i][j][k],buffer.v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F1p[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F4p[i][j][k],buffer.p[i][j][k]);
         pMULTIPLY(buffer.p[i][j][k],1.0 / 6,buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],jello->p[i][j][k],jello->p[i][j][k]);

         pMULTIPLY(F2v[i][j][k],2,buffer.p[i][j][k]);
         pMULTIPLY(F3v[i][j][k],2,buffer.v[i][j][k]);
         pSUM(buffer.p[i][j][k],buffer.v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F1v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F4v[i][j][k],buffer.p[i][j][k]);
         pMULTIPLY(buffer.p[i][j][k],1.0 / 6,buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],jello->v[i][j][k],jello->v[i][j][k]);
      }


  return;  
}
