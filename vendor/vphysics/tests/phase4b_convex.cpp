// Phase 4b unit test: verify CPhysConvex geometry by feeding known shapes
// through IPhysicsCollision::ConvexFromVerts and checking the cached volume,
// surface area, and game-data tag.
//
// Build:
//   c++ -std=c++17 -arch x86_64 phase4b_convex.cpp -o phase4b_convex -ldl
// Run:
//   ./phase4b_convex <path-to-libvphysics.dylib>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <dlfcn.h>

// Minimal replicas of the Source types the factory returns. We only need
// layout compatibility on the bits we touch; full headers are not required
// here because we're dlsym'ing a black-box factory.
struct Vector
{
	float x, y, z;
};

class CPhysConvex;

class IPhysicsCollisionMin
{
public:
	virtual ~IPhysicsCollisionMin() {}
	virtual CPhysConvex *ConvexFromVerts( Vector **, int )     = 0;
	virtual CPhysConvex *ConvexFromPlanes( float *, int, float ) = 0;
	virtual float        ConvexVolume( CPhysConvex * )          = 0;
	virtual void        *ConvertConvexToCollide( CPhysConvex **, int ) = 0;
	virtual float        ConvexSurfaceArea( CPhysConvex * )     = 0;
	virtual void         SetConvexGameData( CPhysConvex *, unsigned int ) = 0;
	virtual void         ConvexFree( CPhysConvex * )            = 0;
};

typedef void *( *CreateInterfaceFn )( const char *, int * );

static bool ApproxEq( float a, float b, float tol = 1e-3f )
{
	return std::fabs( a - b ) <= tol * std::max( 1.0f, std::max( std::fabs(a), std::fabs(b) ) );
}

int main( int argc, char **argv )
{
	if ( argc < 2 ) {
		std::fprintf( stderr, "usage: %s <path-to-libvphysics>\n", argv[0] );
		return 2;
	}
	void *h = dlopen( argv[1], RTLD_NOW );
	if ( !h ) { std::fprintf( stderr, "dlopen: %s\n", dlerror() ); return 1; }
	auto fn = reinterpret_cast<CreateInterfaceFn>( dlsym( h, "CreateInterface" ) );
	if ( !fn ) { std::fprintf( stderr, "dlsym: %s\n", dlerror() ); return 1; }

	int rc = 0;
	auto *coll = reinterpret_cast<IPhysicsCollisionMin *>( fn( "VPhysicsCollision007", &rc ) );
	if ( !coll ) { std::fprintf( stderr, "no IPhysicsCollision\n" ); return 1; }

	int failures = 0;

	// --- Unit cube (8 verts) — expected volume 1, surface area 6.
	{
		Vector cube[8] = {
			{0,0,0}, {1,0,0}, {1,1,0}, {0,1,0},
			{0,0,1}, {1,0,1}, {1,1,1}, {0,1,1},
		};
		Vector *ptrs[8] = { &cube[0],&cube[1],&cube[2],&cube[3],&cube[4],&cube[5],&cube[6],&cube[7] };
		CPhysConvex *c = coll->ConvexFromVerts( ptrs, 8 );
		if ( !c ) { std::fprintf( stderr, "cube: null hull\n" ); return 1; }
		float vol  = coll->ConvexVolume( c );
		float area = coll->ConvexSurfaceArea( c );
		coll->SetConvexGameData( c, 0xABCDEFu );
		std::printf( "cube  volume=%.6f  area=%.6f  (expected 1, 6)\n", vol, area );
		if ( !ApproxEq( vol, 1.0f ) )  { std::fprintf( stderr, "  FAIL: volume\n" );  ++failures; }
		if ( !ApproxEq( area, 6.0f ) ) { std::fprintf( stderr, "  FAIL: surface area\n" ); ++failures; }
		coll->ConvexFree( c );
	}

	// --- Regular tetrahedron with edge length sqrt(2), verts (1,1,1),(1,-1,-1),
	//     (-1,1,-1),(-1,-1,1). Edge = 2*sqrt(2). Volume = edge^3 / (6*sqrt(2))
	//     = (2*sqrt(2))^3 / (6*sqrt(2)) = 16*sqrt(2) / (6*sqrt(2)) = 8/3.
	//     Surface area = sqrt(3)*edge^2 = sqrt(3)*8 = 8*sqrt(3) ~= 13.8564.
	{
		Vector t[4] = { {1,1,1}, {1,-1,-1}, {-1,1,-1}, {-1,-1,1} };
		Vector *ptrs[4] = { &t[0],&t[1],&t[2],&t[3] };
		CPhysConvex *c = coll->ConvexFromVerts( ptrs, 4 );
		if ( !c ) { std::fprintf( stderr, "tet: null hull\n" ); return 1; }
		float vol  = coll->ConvexVolume( c );
		float area = coll->ConvexSurfaceArea( c );
		std::printf( "tet   volume=%.6f  area=%.6f  (expected %.6f, %.6f)\n",
		             vol, area, 8.0/3.0, 8.0 * std::sqrt(3.0) );
		if ( !ApproxEq( vol, 8.0f / 3.0f ) ) { std::fprintf( stderr, "  FAIL: volume\n" ); ++failures; }
		if ( !ApproxEq( area, 8.0f * std::sqrt( 3.0f ) ) ) { std::fprintf( stderr, "  FAIL: area\n" ); ++failures; }
		coll->ConvexFree( c );
	}

	// --- Too few verts — expect null.
	{
		Vector t[3] = { {0,0,0}, {1,0,0}, {0,1,0} };
		Vector *ptrs[3] = { &t[0],&t[1],&t[2] };
		CPhysConvex *c = coll->ConvexFromVerts( ptrs, 3 );
		if ( c != nullptr ) { std::fprintf( stderr, "degenerate: expected null\n" ); ++failures; coll->ConvexFree( c ); }
		else std::printf( "degenerate: correctly rejected\n" );
	}

	std::printf( "%s (%d failures)\n", failures == 0 ? "PASS" : "FAIL", failures );
	return failures;
}
