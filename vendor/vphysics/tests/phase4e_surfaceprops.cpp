// Phase 4e test: parse a small surfaceproperties.txt blob and verify
// IPhysicsSurfaceProps returns the correct fields, including base inheritance.

#include <cmath>
#include <cstdio>
#include <cstring>

#include "test_dl.h"

class IPhysicsSurfacePropsMin
{
public:
	virtual ~IPhysicsSurfacePropsMin() {}
	virtual int   ParseSurfaceData( const char *, const char * ) = 0;
	virtual int   SurfacePropCount() = 0;
	virtual int   GetSurfaceIndex( const char * ) = 0;
	virtual void  GetPhysicsProperties( int, float *, float *, float *, float * ) = 0;
};

typedef void *( *CreateInterfaceFn )( const char *, int * );

int failures = 0;
#define CHECK( cond, ... ) do { \
	if ( !(cond) ) { std::fprintf( stderr, "FAIL: " __VA_ARGS__ ); std::fprintf( stderr, "\n" ); ++failures; } \
} while ( 0 )

int main( int argc, char **argv )
{
	if ( argc < 2 ) return 2;
	void *h = TestDlOpen( argv[1] );
	if ( !h ) { std::fprintf( stderr, "%s\n", TestDlError() ); return 1; }
	auto fn = reinterpret_cast<CreateInterfaceFn>( TestDlSym( h, "CreateInterface" ) );
	int rc = 0;
	auto *props = reinterpret_cast<IPhysicsSurfacePropsMin *>( fn( "VPhysicsSurfaceProps001", &rc ) );

	CHECK( props->SurfacePropCount() == 0, "initial count 0" );
	CHECK( props->GetSurfaceIndex( "default" ) == -1, "empty lookup" );

	const char *kv =
		"// Surface properties test data\n"
		"\"default\"\n"
		"{\n"
		"    \"density\"    \"2000\"\n"
		"    \"elasticity\" \"0.25\"\n"
		"    \"friction\"   \"0.8\"\n"
		"    \"thickness\"  \"0.0\"\n"
		"}\n"
		"\n"
		"\"wood\"\n"
		"{\n"
		"    \"base\"      \"default\"\n"
		"    \"density\"   \"700\"\n"
		"    \"friction\"  \"0.7\"\n"
		"}\n"
		"\n"
		"\"concrete\"\n"
		"{\n"
		"    \"base\"      \"default\"\n"
		"    \"density\"   \"2400\"\n"
		"}\n";

	int added = props->ParseSurfaceData( "surfaceproperties.txt", kv );
	CHECK( added == 3, "added %d (expected 3)", added );
	CHECK( props->SurfacePropCount() == 3, "count %d", props->SurfacePropCount() );

	int iDefault  = props->GetSurfaceIndex( "default" );
	int iWood     = props->GetSurfaceIndex( "wood" );
	int iConcrete = props->GetSurfaceIndex( "concrete" );
	int iMissing  = props->GetSurfaceIndex( "nonesuch" );
	CHECK( iDefault  >= 0 && iWood >= 0 && iConcrete >= 0, "all 3 indices found" );
	CHECK( iMissing == -1, "missing lookup" );

	float d, t, f, e;

	// default: direct
	props->GetPhysicsProperties( iDefault, &d, &t, &f, &e );
	CHECK( std::fabs( d - 2000.0f ) < 0.01f, "default.density=%f", d );
	CHECK( std::fabs( f - 0.8f )    < 1e-4f, "default.friction=%f", f );

	// wood: own density=700 (override), own friction=0.7, inherits elasticity=0.25 from base
	props->GetPhysicsProperties( iWood, &d, &t, &f, &e );
	CHECK( std::fabs( d - 700.0f ) < 0.01f, "wood.density=%f", d );
	CHECK( std::fabs( f - 0.7f )   < 1e-4f, "wood.friction=%f", f );
	CHECK( std::fabs( e - 0.25f )  < 1e-4f, "wood.elasticity inherited = %f", e );

	// concrete: inherits friction from default
	props->GetPhysicsProperties( iConcrete, &d, &t, &f, &e );
	CHECK( std::fabs( d - 2400.0f ) < 0.01f, "concrete.density=%f", d );
	CHECK( std::fabs( f - 0.8f )    < 1e-4f, "concrete.friction inherited = %f", f );

	// Missing index returns zeros.
	d = t = f = e = 99.0f;
	props->GetPhysicsProperties( iMissing, &d, &t, &f, &e );
	CHECK( d == 0.0f && t == 0.0f && f == 0.0f && e == 0.0f, "missing returns zeros" );

	std::printf( "%s (%d failures)\n", failures == 0 ? "PASS" : "FAIL", failures );
	return failures;
}
