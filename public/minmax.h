//========= Copyright © 1996-2005, Valve Corporation, All rights reserved. ============//
//
// Purpose:
//
// $NoKeywords: $
//=============================================================================//

#ifndef MINMAX_H
#define MINMAX_H

#if defined( _WIN32 )
#pragma once
#endif

#if defined( POSIX ) && defined( __cplusplus )
	#ifndef VALVE_MINMAX_FUNCTIONS_DEFINED
		#define VALVE_MINMAX_FUNCTIONS_DEFINED

		#include <type_traits>

		#ifdef min
			#undef min
		#endif
		#ifdef max
			#undef max
		#endif

		template <typename T>
		constexpr const T& min( const T& a, const T& b )
		{
			return ( b < a ) ? b : a;
		}

		template <typename T, typename U, typename C = std::common_type_t<T, U>>
		constexpr C min( T a, U b )
		{
			return ( (C)a < (C)b ) ? (C)a : (C)b;
		}

		template <typename T>
		constexpr const T& max( const T& a, const T& b )
		{
			return ( a < b ) ? b : a;
		}

		template <typename T, typename U, typename C = std::common_type_t<T, U>>
		constexpr C max( T a, U b )
		{
			return ( (C)a < (C)b ) ? (C)b : (C)a;
		}
	#endif
#endif

#if !defined( POSIX )
	#ifndef min
		#define min(a,b)  (((a) < (b)) ? (a) : (b))
	#endif
	#ifndef max
		#define max(a,b)  (((a) > (b)) ? (a) : (b))
	#endif
#endif

#endif // MINMAX_H
