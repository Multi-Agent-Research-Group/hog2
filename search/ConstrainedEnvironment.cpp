#include "ConstrainedEnvironment.h"

int DrawableConstraint::width;
int DrawableConstraint::length;
int DrawableConstraint::height;


template<typename BB, typename Action>
void ConstrainedEnvironment<BB,Action>::AddConstraint(Constraint<BB> const* c){constraints->insert(c);}
template<typename BB, typename Action>
void ConstrainedEnvironment<BB,Action>::AddConstraints(std::vector<std::unique_ptr<Constraint<BB> const>> const& cs){for(auto const& c:cs)constraints->insert(c.get());}
