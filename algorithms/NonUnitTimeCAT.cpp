#include "NonUnitTimeCAT.h"

std::ostream& operator<<(std::ostream& ss, IntervalData v) {ss << "<H1:"<<v.hash1<<"-->"<<v.hash2<<" A:"<<unsigned(v.agent)<<">"; return ss;}

template <typename state, typename action>
double NonUnitTimeCAT<state,action>::bucketWidth=1.0;
