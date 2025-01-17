#include "Boost3DPointHash.h"

//  ¬”“_ˆÈ‰º‚Ì—LŒøŒ…”‚ğ3Œ…‚Æ‚µ‚½ê‡‚Ì2“_ŠÔ‹——£
double Boost3DPointHash::RoundDistance(const Boost3DPointHash &other) const
{
    double dDist = bg::distance(*this, other);
    return CUtil::RoundN(dDist, ncl_common_def::POINT_SIGNIFICANT_DIGITS);
}
