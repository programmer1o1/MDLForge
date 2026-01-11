//========= Copyright Valve Corporation, All rights reserved. ============//
//
// Purpose: 
//
//=====================================================================================//

#ifndef _MATH_PFNS_H_
#define _MATH_PFNS_H_

#pragma once

#include <math.h>

// misyl: This is faster than doing fsincos these days.
inline void SinCos( float radians, float *RESTRICT sine, float *RESTRICT cosine )
{
	*sine = sinf( radians );
	*cosine = cosf( radians );
}

#ifndef FastRSqrt
#define FastRSqrt( x ) ( 1.0f / ::sqrtf( x ) )
#endif

#ifndef FastCos
#define FastCos ::cosf
#endif
#ifndef FastSqrt
#define FastSqrt ::sqrtf
#endif
#ifndef FastSinCos
#define FastSinCos ::SinCos
#endif
#ifndef FastRSqrtFast
#define FastRSqrtFast FastRSqrt
#endif

#endif // _MATH_PFNS_H_
