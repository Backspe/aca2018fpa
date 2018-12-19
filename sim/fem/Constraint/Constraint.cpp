#include "Constraint.h"
using namespace FEM;
Constraint::
Constraint(const double& stiffness)
	:mStiffness(stiffness)
{
	
}

void
Constraint::
SetStiffness(double stiffness)
{
	mStiffness = stiffness;
	return;
}

double
Constraint::
GetStiffness()
{
	return mStiffness;
}
