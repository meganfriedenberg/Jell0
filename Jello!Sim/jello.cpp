/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

  Your name:
  <Megan Friedenberg>

*/

#include "jello.h"
#include "showCube.h"
#include "input.h"
#include "physics.h"
#include <vector>

// camera parameters
double Theta = pi / 6;
double Phi = pi / 6;
double R = 6;

// mouse control
int g_iMenuId;
int g_vMousePos[2];
int g_iLeftMouseButton,g_iMiddleMouseButton,g_iRightMouseButton;

// number of images saved to disk so far
int sprite=0;

// these variables control what is displayed on screen
int shear=0, bend=0, structural=1, pause=0, viewingMode=0, saveScreenToFile=0;

struct world jello;

// if these aren't declared here, gets erased off the stack
struct structuralSprings structSpringList;
struct bendSprings bendSpringList;
struct shearSprings shearSpringList;

bool initializeSprings = false; // only initialize once
// if these vectors are not declared here, does NOT work, for some reason gets erased from the structs
std::vector<spring> structuralSprings;
std::vector<spring> bendSprings;
std::vector<spring> shearSprings;


int windowWidth, windowHeight;

void myinit()
{
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(90.0,1.0,0.01,1000.0);

  // set background color to grey
  glClearColor(0.5, 0.5, 0.5, 0.0);

  glCullFace(GL_BACK);
  glEnable(GL_CULL_FACE);

  glShadeModel(GL_SMOOTH);
  glEnable(GL_POLYGON_SMOOTH);
  glEnable(GL_LINE_SMOOTH);

  return; 
}

// see if the spring passed in is part of the passed in spring list so springs are not accidentally created twice
bool checkSpringExistence(std::vector<spring>& springList, struct spring springToFind)
{
    for (int i = 0; i < springList.size(); i++) // loop thru all springs
    {
        point currPointA = springList[i].p1;
        point currPointB = springList[i].p2;

        if (springToFind.p1.x == currPointA.x)
        {
            if (springToFind.p1.y == currPointA.y)
            {
                if (springToFind.p1.z == currPointA.z)
                {
                    // we know one of the spring matches, check other
                    if (springToFind.p2.x == currPointB.x)
                    {
                        if (springToFind.p2.y == currPointB.y)
                        {
                            if (springToFind.p2.z == currPointB.z)
                            {
                                return true;
                            }
                        }
                    }
                }
            }
        }
        if (springToFind.p2.x == currPointA.x)
        {
            if (springToFind.p2.y == currPointA.y)
            {
                if (springToFind.p2.z == currPointA.z)
                {
                    // we know one of the spring matches, check other
                    if (springToFind.p1.x == currPointB.x)
                    {
                        if (springToFind.p1.y == currPointB.y)
                        {
                            if (springToFind.p1.z == currPointB.z)
                            {
                                return true;
                            }
                        }
                    }
                }
            }
        }
    }

    return false;
}
// helper function to see if the current position passed in (i,j,k) is out of bounds (of the mass-spring system)
bool checkSpringNeighbor(int i, int j, int k)
{
    if (i > 7 || i < 0 || j > 7 || j < 0 || k > 7 || k < 0)
    {
        return false;
    }

    return true;

}
// create all structural springs for mass-spring system
void initializeStructural(struct structuralSprings* springList)
{
    //std::vector<spring> structural; // if a temporary vector is created here, it is garbage collected.
    point aCurr;
    point bCurr;

    spring temp;
    temp.kHook = jello.kElastic;
    temp.kD = jello.dElastic;
    temp.restingLength = (1.0f) / (7.0f); // from in class
   
    // Below code is to test if it is pass by value or pass by reference
    //aSpring.x = 0;
    //aSpring.y = 0;
    //aSpring.z = 0;

    //bSpring.x = 1;
    //bSpring.y = 1;
    //bSpring.z = 1;

    //temp.p1 = aSpring;
    //temp.p2 = bSpring;

    //structuralSprings.push_back(temp);

    //bool found = checkSpringExistence(structuralSprings, temp);

    // end test code for pass by value/reference

    for (int i = 0; i < 8; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            for (int k = 0; k < 8; k++)
            {
                aCurr.x = i;
                aCurr.y = j;
                aCurr.z = k;

                // Every (i,j,k) is connected to neighbors: (i+1,j,k), (i-1,j,k), (i,j-1,k), (i,j+1,k),(i,j,k-1),(i,j,k+1) <- this is from the slides
                // check if this is a valid spring first (not out of bounds), then check if this spring has already been created
                
                // check (i+1,j,k)
                if(checkSpringNeighbor(i+1, j, k))
                {
                    bCurr.x = i+1;
                    bCurr.y = j;
                    bCurr.z = k;

                    temp.p1 = aCurr;
                    temp.p2 = bCurr;
                    if (!(checkSpringExistence(structuralSprings, temp))) // if the spring doesn't already exist
                    {
                        structuralSprings.push_back(temp);
                    }
                }
                // check (i-1,j,k)
                if (checkSpringNeighbor(i-1, j, k))
                {
                    bCurr.x = i-1;
                    bCurr.y = j;
                    bCurr.z = k;

                    temp.p1 = aCurr;
                    temp.p2 = bCurr;
                    if (!(checkSpringExistence(structuralSprings, temp))) // if the spring doesn't already exist
                    {
                        structuralSprings.push_back(temp);
                    }
                }
                // check (i,j-1,k)
                if (checkSpringNeighbor(i, j-1, k))
                {
                    bCurr.x = i;
                    bCurr.y = j-1;
                    bCurr.z = k;

                    temp.p1 = aCurr;
                    temp.p2 = bCurr;
                    if (!(checkSpringExistence(structuralSprings, temp))) // if the spring doesn't already exist
                    {
                        structuralSprings.push_back(temp);
                    }
                }
                // check (i,j+1,k)
                if (checkSpringNeighbor(i, j+1, k))
                {
                    bCurr.x = i;
                    bCurr.y = j+1;
                    bCurr.z = k;

                    temp.p1 = aCurr;
                    temp.p2 = bCurr;
                    if (!(checkSpringExistence(structuralSprings, temp))) // if the spring doesn't already exist
                    {
                        structuralSprings.push_back(temp);
                    }
                }
                // check (i,j,k-1)
                if (checkSpringNeighbor(i, j, k-1))
                {
                    bCurr.x = i;
                    bCurr.y = j;
                    bCurr.z = k-1;

                    temp.p1 = aCurr;
                    temp.p2 = bCurr;
                    if (!(checkSpringExistence(structuralSprings, temp))) // if the spring doesn't already exist
                    {
                        structuralSprings.push_back(temp);
                    }
                }
                // check (i,j,k+1)
                if (checkSpringNeighbor(i, j, k+1))
                {
                    bCurr.x = i;
                    bCurr.y = j;
                    bCurr.z = k+1;

                    temp.p1 = aCurr;
                    temp.p2 = bCurr;
                    if (!(checkSpringExistence(structuralSprings, temp))) // if the spring doesn't already exist
                    {
                        structuralSprings.push_back(temp);
                    }
                }

            }
        }
    }


    springList->springs = &structuralSprings;
}
// create all shear springs for mass-spring system
void initializeShear(struct shearSprings* springList)
{
    point aCurr;
    point bCurr;
    spring temp;
    temp.kHook = jello.kElastic;
    temp.kD = jello.dElastic;

    // 2 different rest lengths (from in class)
    // for the diagonals of a face, its length is sqrt(2)/7
    // for the diagonals going across the cube, its length is sqrt(3)/7
    double faceRestingLength = (sqrt(2)) * ((1.0f) / (7.0f));
    double diagonalRestingLength = (sqrt(3)) * ((1.0f) / (7.0f));


    for (int i = 0; i < 8; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            for (int k = 0; k < 8; k++)
            {
                aCurr.x = i;
                aCurr.y = j;
                aCurr.z = k;

                // Every node (i,j,k) is connected to its diagonal neighbors
                // cube has 6 faces,2 shear springs connecting diagonals, so 12 total per cube
                // face diagonals are: 
                //(i+1,j+1,k), (i+1,j,k+1),(i,j+1,k+1), // curr cube
                //(i,j-1,k+1), (i+1,j-1,k), (i-1,j,k+1) // the one in front
                //(i+1,j,k-1), (i,j+1,k-1), (i-1,j,k-1) // the one to the left
                //(i,j-1,k-1), (i-1,j+1,k), (i-1,j-1,k)

                // face diagonals first
                // check if this is a valid spring first (not out of bounds), then check if this spring has already been created
                 
                // check (i+1,j+1,k)
                if (checkSpringNeighbor(i+1, j+1, k))
                {
                    bCurr.x = i + 1;
                    bCurr.y = j + 1;
                    bCurr.z = k;

                    temp.p1 = aCurr;
                    temp.p2 = bCurr;

                    if (!checkSpringExistence(shearSprings, temp))
                    {
                        temp.restingLength = faceRestingLength;
                        shearSprings.push_back(temp);
                    }
                }
                // check (i+1,j,k+1)
                if (checkSpringNeighbor(i+1, j, k+1))
                {
                    bCurr.x = i + 1;
                    bCurr.y = j;
                    bCurr.z = k+1;

                    temp.p1 = aCurr;
                    temp.p2 = bCurr;

                    if (!checkSpringExistence(shearSprings, temp))
                    {
                        temp.restingLength = faceRestingLength;
                        shearSprings.push_back(temp);
                    }
                }
                //check (i,j+1,k+1)
                if (checkSpringNeighbor(i,j+1,k+1))
                {
                    bCurr.x = i;
                    bCurr.y = j+1;
                    bCurr.z = k+1;

                    temp.p1 = aCurr;
                    temp.p2 = bCurr;

                    if (!checkSpringExistence(shearSprings, temp))
                    {
                        temp.restingLength = faceRestingLength;
                        shearSprings.push_back(temp);
                    }
                }
                //check (i,j-1,k+1)
                if (checkSpringNeighbor(i, j - 1, k + 1))
                {
                    bCurr.x = i;
                    bCurr.y = j - 1;
                    bCurr.z = k + 1;

                    temp.p1 = aCurr;
                    temp.p2 = bCurr;

                    if (!checkSpringExistence(shearSprings, temp))
                    {
                        temp.restingLength = faceRestingLength;
                        shearSprings.push_back(temp);
                    }
                }
                //check (i+1,j-1,k)
                if (checkSpringNeighbor(i + 1, j - 1, k))
                {
                    bCurr.x = i+1;
                    bCurr.y = j - 1;
                    bCurr.z = k;

                    temp.p1 = aCurr;
                    temp.p2 = bCurr;

                    if (!checkSpringExistence(shearSprings, temp))
                    {
                        temp.restingLength = faceRestingLength;
                        shearSprings.push_back(temp);
                    }
                }
                //check (i-1,j,k+1)
                if (checkSpringNeighbor(i-1, j, k+1))
                {
                    bCurr.x = i - 1;
                    bCurr.y = j;
                    bCurr.z = k+1;

                    temp.p1 = aCurr;
                    temp.p2 = bCurr;

                    if (!checkSpringExistence(shearSprings, temp))
                    {
                        temp.restingLength = faceRestingLength;
                        shearSprings.push_back(temp);
                    }
                }
                //check (i+1,j,k-1)
                if (checkSpringNeighbor(i + 1, j, k - 1))
                {
                    bCurr.x = i + 1;
                    bCurr.y = j;
                    bCurr.z = k - 1;

                    temp.p1 = aCurr;
                    temp.p2 = bCurr;

                    if (!checkSpringExistence(shearSprings, temp))
                    {
                        temp.restingLength = faceRestingLength;
                        shearSprings.push_back(temp);
                    }
                }
                //check (i,j+1,k-1)
                if (checkSpringNeighbor(i, j+1, k - 1))
                {
                    bCurr.x = i;
                    bCurr.y = j+1;
                    bCurr.z = k - 1;

                    temp.p1 = aCurr;
                    temp.p2 = bCurr;

                    if (!checkSpringExistence(shearSprings, temp))
                    {
                        temp.restingLength = faceRestingLength;
                        shearSprings.push_back(temp);
                    }
                }
                //check (i-1,j,k-1)
                if (checkSpringNeighbor(i-1, j, k - 1))
                {
                    bCurr.x = i-1;
                    bCurr.y = j;
                    bCurr.z = k - 1;

                    temp.p1 = aCurr;
                    temp.p2 = bCurr;

                    if (!checkSpringExistence(shearSprings, temp))
                    {
                        temp.restingLength = faceRestingLength;
                        shearSprings.push_back(temp);
                    }
                }
                //check (i,j-1,k-1)
                if (checkSpringNeighbor(i, j-1, k - 1))
                {
                    bCurr.x = i;
                    bCurr.y = j-1;
                    bCurr.z = k - 1;

                    temp.p1 = aCurr;
                    temp.p2 = bCurr;

                    if (!checkSpringExistence(shearSprings, temp))
                    {
                        temp.restingLength = faceRestingLength;
                        shearSprings.push_back(temp);
                    }
                }
                //check (i-1,j+1,k)
                if (checkSpringNeighbor(i-1, j + 1, k))
                {
                    bCurr.x = i-1;
                    bCurr.y = j + 1;
                    bCurr.z = k;

                    temp.p1 = aCurr;
                    temp.p2 = bCurr;

                    if (!checkSpringExistence(shearSprings, temp))
                    {
                        temp.restingLength = faceRestingLength;
                        shearSprings.push_back(temp);
                    }
                }
                //check (i-1,j-1,k)
                if (checkSpringNeighbor(i - 1, j - 1, k))
                {
                    bCurr.x = i - 1;
                    bCurr.y = j - 1;
                    bCurr.z = k;

                    temp.p1 = aCurr;
                    temp.p2 = bCurr;

                    if (!checkSpringExistence(shearSprings, temp))
                    {
                        temp.restingLength = faceRestingLength;
                        shearSprings.push_back(temp);
                    }
                }


                //those were the 12 face springs! now do the across the cube! there should be 8 (from in class)
                //(i+1,j+1,k+1),(i-1,j+1,k+1),(i+1,j+1,k-1),(i-1,j+1,k-1) // curr cube, the one under, the left one, bottom left one
                //(i-1,j-1,k+1),(i+1,j-1,k+1),(i+1,j-1,k-1),(i-1,j-1,k-1) // the cubes "in front"

                //check (i+1,j+1,k+1)
                if (checkSpringNeighbor(i+1,j+1, k+1))
                {
                    bCurr.x = i+1;
                    bCurr.y = j+1;
                    bCurr.z = k+1;

                    temp.p1 = aCurr;
                    temp.p2 = bCurr;

                    if (!checkSpringExistence(shearSprings, temp))
                    {
                        temp.restingLength = diagonalRestingLength;
                        shearSprings.push_back(temp);
                    }
                }
                //check (i-1,j+1,k+1)
                if (checkSpringNeighbor(i-1, j+1, k+1))
                {
                    bCurr.x = i - 1;
                    bCurr.y = j + 1;
                    bCurr.z = k + 1;

                    temp.p1 = aCurr;
                    temp.p2 = bCurr;

                    if (!checkSpringExistence(shearSprings, temp))
                    {
                        temp.restingLength = diagonalRestingLength;
                        shearSprings.push_back(temp);
                    }
                }
                //check (i+1,j+1,k-1)
                if (checkSpringNeighbor(i + 1, j + 1, k - 1))
                {
                    bCurr.x = i + 1;
                    bCurr.y = j + 1;
                    bCurr.z = k - 1;

                    temp.p1 = aCurr;
                    temp.p2 = bCurr;

                    if (!checkSpringExistence(shearSprings, temp))
                    {
                        temp.restingLength = diagonalRestingLength;
                        shearSprings.push_back(temp);
                    }
                }
                //check (i-1,j+1,k-1)
                if (checkSpringNeighbor(i - 1, j + 1, k - 1))
                {
                    bCurr.x = i - 1;
                    bCurr.y = j + 1;
                    bCurr.z = k - 1;

                    temp.p1 = aCurr;
                    temp.p2 = bCurr;

                    if (!checkSpringExistence(shearSprings, temp))
                    {
                        temp.restingLength = diagonalRestingLength;
                        shearSprings.push_back(temp);
                    }
                }
                //check (i-1,j-1,k+1)
                if (checkSpringNeighbor(i - 1, j - 1, k + 1))
                {
                    bCurr.x = i - 1;
                    bCurr.y = j - 1;
                    bCurr.z = k + 1;

                    temp.p1 = aCurr;
                    temp.p2 = bCurr;

                    if (!checkSpringExistence(shearSprings, temp))
                    {
                        temp.restingLength = diagonalRestingLength;
                        shearSprings.push_back(temp);
                    }
                }
                //check (i+1,j-1,k+1)
                if (checkSpringNeighbor(i + 1, j - 1, k + 1))
                {
                    bCurr.x = i + 1;
                    bCurr.y = j - 1;
                    bCurr.z = k + 1;

                    temp.p1 = aCurr;
                    temp.p2 = bCurr;

                    if (!checkSpringExistence(shearSprings, temp))
                    {
                        temp.restingLength = diagonalRestingLength;
                        shearSprings.push_back(temp);
                    }
                }
                //check (i+1,j-1,k-1)
                if (checkSpringNeighbor(i + 1, j - 1, k - 1))
                {
                    bCurr.x = i + 1;
                    bCurr.y = j - 1;
                    bCurr.z = k - 1;

                    temp.p1 = aCurr;
                    temp.p2 = bCurr;

                    if (!checkSpringExistence(shearSprings, temp))
                    {
                        temp.restingLength = diagonalRestingLength;
                        shearSprings.push_back(temp);
                    }
                }
                //check (i-1,j-1,k-1)
                if (checkSpringNeighbor(i - 1, j - 1, k - 1))
                {
                    bCurr.x = i - 1;
                    bCurr.y = j - 1;
                    bCurr.z = k - 1;

                    temp.p1 = aCurr;
                    temp.p2 = bCurr;

                    if (!checkSpringExistence(shearSprings, temp))
                    {
                        temp.restingLength = diagonalRestingLength;
                        shearSprings.push_back(temp);
                    }
                }
            }
        }
    }



    springList->springs = &shearSprings;
}


// create all bend springs for the mass-spring system
void initializeBend(struct bendSprings* springList)
{
    // bend springs -> "every node connected to its second neighbor" (6 neighbors)

    point aCurr;
    point bCurr;
    spring temp;

    temp.kHook = jello.kElastic;
    temp.kD = jello.dElastic;
    // rest length is double the length of a structural so *2 (from in class)
    double restingLength = 2 * ((1.0f) / (7.0f));



    for (int i = 0; i < 8; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            for (int k = 0; k < 8; k++)
            {
                aCurr.x = i;
                aCurr.y = j;
                aCurr.z = k;

                //all 2nd neighbors are:
                //(i+2,j,k),(i-2,j,k),(i,j+2,k),(i,j-2,k),(i,j,k+2),(i,j,k-2)

                // first check if creating the spring is out of bounds (of the cube), next check if this spring already exists

                //check (i+2,j,k)
                if (checkSpringNeighbor(i+2,j,k))
                {
                    bCurr.x = i+2;
                    bCurr.y = j;
                    bCurr.z = k;

                    temp.p1 = aCurr;
                    temp.p2 = bCurr;

                    if (!checkSpringExistence(bendSprings, temp))
                    {
                        temp.restingLength = restingLength;
                        bendSprings.push_back(temp);
                    }
                }
                //check (i-2,j,k)
                if (checkSpringNeighbor(i - 2, j, k))
                {
                    bCurr.x = i - 2;
                    bCurr.y = j;
                    bCurr.z = k;

                    temp.p1 = aCurr;
                    temp.p2 = bCurr;

                    if (!checkSpringExistence(bendSprings, temp))
                    {
                        temp.restingLength = restingLength;
                        bendSprings.push_back(temp);
                    }
                }
                //check (i,j+2,k)
                if (checkSpringNeighbor(i, j+2, k))
                {
                    bCurr.x = i;
                    bCurr.y = j+2;
                    bCurr.z = k;

                    temp.p1 = aCurr;
                    temp.p2 = bCurr;

                    if (!checkSpringExistence(bendSprings, temp))
                    {
                        temp.restingLength = restingLength;
                        bendSprings.push_back(temp);
                    }
                }
                //check (i,j-2,k)
                if (checkSpringNeighbor(i, j - 2, k))
                {
                    bCurr.x = i;
                    bCurr.y = j - 2;
                    bCurr.z = k;

                    temp.p1 = aCurr;
                    temp.p2 = bCurr;

                    if (!checkSpringExistence(bendSprings, temp))
                    {
                        temp.restingLength = restingLength;
                        bendSprings.push_back(temp);
                    }
                }
                //check (i,j,k+2)
                if (checkSpringNeighbor(i, j, k+2))
                {
                    bCurr.x = i;
                    bCurr.y = j;
                    bCurr.z = k+2;

                    temp.p1 = aCurr;
                    temp.p2 = bCurr;

                    if (!checkSpringExistence(bendSprings, temp))
                    {
                        temp.restingLength = restingLength;
                        bendSprings.push_back(temp);
                    }
                }
                //check (i,j,k-2)
                if (checkSpringNeighbor(i, j, k - 2))
                {
                    bCurr.x = i;
                    bCurr.y = j;
                    bCurr.z = k - 2;

                    temp.p1 = aCurr;
                    temp.p2 = bCurr;

                    if (!checkSpringExistence(bendSprings, temp))
                    {
                        temp.restingLength = restingLength;
                        bendSprings.push_back(temp);
                    }
                }
            }
        }
    }


    springList->springs = &bendSprings;


}

void reshape(int w, int h) 
{
  // Prevent a divide by zero, when h is zero.
  // You can't make a window of zero height.
  if(h == 0)
    h = 1;

  glViewport(0, 0, w, h);

  // Reset the coordinate system before modifying
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  // Set the perspective
  double aspectRatio = 1.0 * w / h;
  gluPerspective(60.0f, aspectRatio, 0.01f, 1000.0f);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity(); 

  windowWidth = w;
  windowHeight = h;

  glutPostRedisplay();
}

void display()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  // camera parameters are Phi, Theta, R
  gluLookAt(R * cos(Phi) * cos (Theta), R * sin(Phi) * cos (Theta), R * sin (Theta),
	        0.0,0.0,0.0, 0.0,0.0,1.0);


  /* Lighting */
  /* You are encouraged to change lighting parameters or make improvements/modifications
     to the lighting model . 
     This way, you will personalize your assignment and your assignment will stick out. 
  */

  // global ambient light
  GLfloat aGa[] = { 0.0, 0.0, 0.0, 0.0 };
  
  // light 's ambient, diffuse, specular
  GLfloat lKa0[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKd0[] = { 1.0, 1.0, 1.0, 1.0 };
  GLfloat lKs0[] = { 1.0, 1.0, 1.0, 1.0 };

  GLfloat lKa1[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKd1[] = { 1.0, 0.0, 0.0, 1.0 };
  GLfloat lKs1[] = { 1.0, 0.0, 0.0, 1.0 };

  GLfloat lKa2[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKd2[] = { 1.0, 1.0, 0.0, 1.0 };
  GLfloat lKs2[] = { 1.0, 1.0, 0.0, 1.0 };

  GLfloat lKa3[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKd3[] = { 0.0, 1.0, 1.0, 1.0 };
  GLfloat lKs3[] = { 0.0, 1.0, 1.0, 1.0 };

  GLfloat lKa4[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKd4[] = { 0.0, 0.0, 1.0, 1.0 };
  GLfloat lKs4[] = { 0.0, 0.0, 1.0, 1.0 };

  GLfloat lKa5[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKd5[] = { 1.0, 0.0, 1.0, 1.0 };
  GLfloat lKs5[] = { 1.0, 0.0, 1.0, 1.0 };

  GLfloat lKa6[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKd6[] = { 1.0, 1.0, 1.0, 1.0 };
  GLfloat lKs6[] = { 1.0, 1.0, 1.0, 1.0 };

  GLfloat lKa7[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat lKd7[] = { 0.0, 1.0, 1.0, 1.0 };
  GLfloat lKs7[] = { 0.0, 1.0, 1.0, 1.0 };

  // light positions and directions
  GLfloat lP0[] = { -1.999, -1.999, -1.999, 1.0 };
  GLfloat lP1[] = { 1.999, -1.999, -1.999, 1.0 };
  GLfloat lP2[] = { 1.999, 1.999, -1.999, 1.0 };
  GLfloat lP3[] = { -1.999, 1.999, -1.999, 1.0 };
  GLfloat lP4[] = { -1.999, -1.999, 1.999, 1.0 };
  GLfloat lP5[] = { 1.999, -1.999, 1.999, 1.0 };
  GLfloat lP6[] = { 1.999, 1.999, 1.999, 1.0 };
  GLfloat lP7[] = { -1.999, 1.999, 1.999, 1.0 };
  
  // jelly material color

  GLfloat mKa[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat mKd[] = { 0.3, 0.3, 0.3, 1.0 };
  GLfloat mKs[] = { 1.0, 1.0, 1.0, 1.0 };
  GLfloat mKe[] = { 0.0, 0.0, 0.0, 1.0 };

  /* set up lighting */
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, aGa);
  glLightModelf(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
  glLightModelf(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);

  // set up cube color
  glMaterialfv(GL_FRONT, GL_AMBIENT, mKa);
  glMaterialfv(GL_FRONT, GL_DIFFUSE, mKd);
  glMaterialfv(GL_FRONT, GL_SPECULAR, mKs);
  glMaterialfv(GL_FRONT, GL_EMISSION, mKe);
  glMaterialf(GL_FRONT, GL_SHININESS, 120);
    
  // macro to set up light i
  #define LIGHTSETUP(i)\
  glLightfv(GL_LIGHT##i, GL_POSITION, lP##i);\
  glLightfv(GL_LIGHT##i, GL_AMBIENT, lKa##i);\
  glLightfv(GL_LIGHT##i, GL_DIFFUSE, lKd##i);\
  glLightfv(GL_LIGHT##i, GL_SPECULAR, lKs##i);\
  glEnable(GL_LIGHT##i)
  
  LIGHTSETUP (0);
  LIGHTSETUP (1);
  LIGHTSETUP (2);
  LIGHTSETUP (3);
  LIGHTSETUP (4);
  LIGHTSETUP (5);
  LIGHTSETUP (6);
  LIGHTSETUP (7);

  // enable lighting
  glEnable(GL_LIGHTING);    
  glEnable(GL_DEPTH_TEST);

  // show the cube
  showCube(&jello);

  glDisable(GL_LIGHTING);

  // show the bounding box
  showBoundingBox();
 
  glutSwapBuffers();
}

void doIdle() // called every timestep-ish
{
    if (!initializeSprings) // if we haven't initialized the springs yet, do now
    {
        initializeStructural(&structSpringList);
        initializeShear(&shearSpringList);
        initializeBend(&bendSpringList);

        initializeSprings = true;
    }
  char s[20]="picxxxx.ppm";
  int i;
  
  // save screen to file
  s[3] = 48 + (sprite / 1000);
  s[4] = 48 + (sprite % 1000) / 100;
  s[5] = 48 + (sprite % 100 ) / 10;
  s[6] = 48 + sprite % 10;

  if (saveScreenToFile==1)
  {
    saveScreenshot(windowWidth, windowHeight, s);
    saveScreenToFile=0; // save only once, change this if you want continuos image generation (i.e. animation)
    sprite++;
  }

  if (sprite >= 300) // allow only 300 snapshots
  {
    exit(0);	
  }

  if (pause == 0)
  {
    // TODO
    // insert code which appropriately performs one step of the cube simulation:
      if (strcmp(jello.integrator, "Euler") == 0)
      {
          Euler(&jello, &structSpringList, &bendSpringList, &shearSpringList); // can confirm, works with both methods
      }
      else if (strcmp(jello.integrator, "RK4") == 0)
      {

          RK4(&jello, &structSpringList, &bendSpringList, &shearSpringList);

          // the below code is to understand how timesteps were taking place
    
          //Euler(&jello, &structSpringList, &bendSpringList, &shearSpringList);
          // timestep check (#5)
         /* for (int i = 0; i <= 7; i++)
          {
              for (int j = 0; j <= 7; j++)
              {
                  for (int k = 0; k <= 7; k++)
                  {
                      jello.p[i][j][k].z = jello.p[i][j][k].z + 0.005;
                  }
              }
          }*/
      }

  }

  glutPostRedisplay();
}

int main (int argc, char ** argv)
{
  if (argc<2)
  {  
    printf ("Oops! You didn't say the jello world file!\n");
    printf ("Usage: %s [worldfile]\n", argv[0]);
    exit(0);
  }

  readWorld(argv[1],&jello);

  glutInit(&argc,argv);
  
  /* double buffered window, use depth testing, 640x480 */
  glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  
  windowWidth = 640;
  windowHeight = 480;
  glutInitWindowSize (windowWidth, windowHeight);
  glutInitWindowPosition (0,0);
  glutCreateWindow ("Jello cube");

  /* tells glut to use a particular display function to redraw */
  glutDisplayFunc(display);

  /* replace with any animate code */
  glutIdleFunc(doIdle);

  /* callback for mouse drags */
  glutMotionFunc(mouseMotionDrag);

  /* callback for window size changes */
  glutReshapeFunc(reshape);

  /* callback for mouse movement */
  glutPassiveMotionFunc(mouseMotion);

  /* callback for mouse button changes */
  glutMouseFunc(mouseButton);

  /* register for keyboard events */
  glutKeyboardFunc(keyboardFunc);

  /* do initialization */
  myinit();

  /* forever sink in the black hole */
  glutMainLoop();

  return(0);
}

