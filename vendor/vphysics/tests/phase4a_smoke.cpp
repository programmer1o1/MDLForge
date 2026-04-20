// Standalone Phase 4a smoke test: verify the clean-room vphysics shared
// library can be dlopened and that CreateInterface returns non-null handles
// for both interface versions studiomdl requires.
//
// Build manually:
//   c++ -std=c++17 phase4a_smoke.cpp -o phase4a_smoke -ldl
// Run:
//   ./phase4a_smoke path/to/libvphysics.dylib

#include <cstdio>
#include <cstdlib>
#include <dlfcn.h>

typedef void *(*CreateInterfaceFn)( const char *, int * );

int main( int argc, char **argv )
{
	if ( argc < 2 ) {
		std::fprintf( stderr, "usage: %s <path-to-libvphysics>\n", argv[0] );
		return 2;
	}
	void *h = dlopen( argv[1], RTLD_NOW );
	if ( !h ) {
		std::fprintf( stderr, "dlopen failed: %s\n", dlerror() );
		return 1;
	}
	auto fn = reinterpret_cast<CreateInterfaceFn>( dlsym( h, "CreateInterface" ) );
	if ( !fn ) {
		std::fprintf( stderr, "dlsym(CreateInterface) failed: %s\n", dlerror() );
		return 1;
	}
	int rc = -1;
	void *coll  = fn( "VPhysicsCollision007",  &rc );
	void *props = fn( "VPhysicsSurfaceProps001", &rc );
	void *bogus = fn( "VPhysicsNonexistent000",  &rc );

	std::printf( "collision=%p surfaceprops=%p bogus=%p rc=%d\n", coll, props, bogus, rc );

	if ( !coll || !props || bogus )
		return 1;
	return 0;
}
