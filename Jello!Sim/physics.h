#ifndef _PHYSICS_H_
#define _PHYSICS_H_

void computeAcceleration(struct world* jello, struct point a[8][8][8], struct structuralSprings* structSpringList, struct bendSprings* bendSpringList, struct shearSprings* shearSpringList);

// perform one step of Euler and Runge-Kutta-4th-order integrators
// updates the jello structure accordingly

void Euler(struct world* jello, struct structuralSprings* structSpringList, struct bendSprings* bendSpringList, struct shearSprings* shearSpringList);
void RK4(struct world* jello, struct structuralSprings* structSpringList, struct bendSprings* bendSpringList, struct shearSprings* shearSpringList);

#endif

