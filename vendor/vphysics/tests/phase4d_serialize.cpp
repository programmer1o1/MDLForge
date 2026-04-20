// Phase 4d round-trip test: build a cube CPhysCollide through the vphysics
// factory, serialize it, then re-parse the bytes to confirm structural
// validity (magic values, offsets, sizes all line up).

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "test_dl.h"

struct Vector { float x, y, z; };
class CPhysConvex;
class CPhysCollide;

// Mirror the minimum vtable prefix we need from IPhysicsCollision.
class IPhysicsCollisionMin
{
public:
	virtual ~IPhysicsCollisionMin() {}
	virtual CPhysConvex  *ConvexFromVerts( Vector **, int ) = 0;
	virtual CPhysConvex  *ConvexFromPlanes( float *, int, float ) = 0;
	virtual float         ConvexVolume( CPhysConvex * ) = 0;
	virtual CPhysCollide *ConvertConvexToCollide( CPhysConvex **, int ) = 0;
	virtual float         ConvexSurfaceArea( CPhysConvex * ) = 0;
	virtual void          SetConvexGameData( CPhysConvex *, unsigned int ) = 0;
	virtual void          ConvexFree( CPhysConvex * ) = 0;
	// polysoup 4
	virtual void         *PolysoupCreate() = 0;
	virtual void          PolysoupDestroy( void * ) = 0;
	virtual void          PolysoupAddTriangle( void *, const Vector &, const Vector &, const Vector &, int ) = 0;
	virtual CPhysCollide *ConvertPolysoupToCollide( void *, bool ) = 0;
	// the two we care about
	virtual int   CollideSize( CPhysCollide * ) = 0;
	virtual int   CollideWrite( char *, CPhysCollide * ) = 0;
};

// Expected on-disk layout (see phy_format.h / phy_serialize.cpp).
#pragma pack(push, 1)
struct CompactSurfaceHeader
{
	uint32_t vphysicsID;
	int16_t  version;
	int16_t  modelType;
	int32_t  surfaceSize;
	float    dragAxisAreas[3];
	int32_t  axisMapSize;
};
struct IVPCompactSurface
{
	float    massCenter[3];
	float    rotInertia[3];
	float    upperLimitRadius;
	uint32_t maxDevAndByteSize;
	int32_t  offsetLedgetreeRoot;
	int32_t  unused[3];
};
struct IVPCompactLedgeNode
{
	int32_t offsetRight;
	int32_t offsetLedge;
	float   center[3];
	float   radius;
	uint8_t box[3];
	uint8_t pad;
};
struct IVPCompactLedge
{
	int32_t  cPointOffset;
	int32_t  backref;
	uint32_t flags;
	uint16_t nTriangles;
	int16_t  unknown;
};
#pragma pack(pop)

static_assert( sizeof( CompactSurfaceHeader ) == 28, "hdr size" );
static_assert( sizeof( IVPCompactSurface ) == 48,    "surf size" );
static_assert( sizeof( IVPCompactLedgeNode ) == 28,  "node size" );
static_assert( sizeof( IVPCompactLedge ) == 16,      "ledge size" );

typedef void *( *CreateInterfaceFn )( const char *, int * );

int failures = 0;
#define CHECK( cond, ... ) do { \
	if ( !(cond) ) { std::fprintf( stderr, "FAIL: %s:%d ", __FILE__, __LINE__ ); std::fprintf( stderr, __VA_ARGS__ ); std::fprintf( stderr, "\n" ); ++failures; } \
} while ( 0 )

int main( int argc, char **argv )
{
	if ( argc < 2 ) { std::fprintf( stderr, "usage: %s <libvphysics>\n", argv[0] ); return 2; }
	void *h = TestDlOpen( argv[1] );
	if ( !h ) { std::fprintf( stderr, "dlopen: %s\n", TestDlError() ); return 1; }
	auto fn = reinterpret_cast<CreateInterfaceFn>( TestDlSym( h, "CreateInterface" ) );
	int rc = 0;
	auto *coll = reinterpret_cast<IPhysicsCollisionMin *>( fn( "VPhysicsCollision007", &rc ) );

	// Build a cube → CPhysCollide with 1 convex.
	Vector cube[8] = { {0,0,0},{1,0,0},{1,1,0},{0,1,0},{0,0,1},{1,0,1},{1,1,1},{0,1,1} };
	Vector *ptrs[8] = { cube+0,cube+1,cube+2,cube+3,cube+4,cube+5,cube+6,cube+7 };
	CPhysConvex *cv = coll->ConvexFromVerts( ptrs, 8 );
	CHECK( cv != nullptr, "ConvexFromVerts returned null" );
	CPhysConvex *convexes[1] = { cv };
	CPhysCollide *c = coll->ConvertConvexToCollide( convexes, 1 );
	CHECK( c != nullptr, "ConvertConvexToCollide returned null" );

	int sz = coll->CollideSize( c );
	CHECK( sz > 0, "CollideSize=%d", sz );
	std::printf( "CollideSize = %d bytes\n", sz );

	std::vector<uint8_t> buf;
	buf.resize( sz );
	int written = coll->CollideWrite( reinterpret_cast<char *>( buf.data() ), c );
	CHECK( written == sz, "CollideWrite wrote %d of %d", written, sz );

	// --- Re-parse and validate.
	constexpr size_t kHdrSize = sizeof( CompactSurfaceHeader ); // 28
	auto *hdr  = reinterpret_cast<CompactSurfaceHeader *>( buf.data() );
	auto *surf = reinterpret_cast<IVPCompactSurface *>( buf.data() + kHdrSize );
	CHECK( hdr->vphysicsID == ( 'V' | ('P'<<8) | ('H'<<16) | ('Y'<<24) ), "magic VPHY" );
	CHECK( hdr->version    == 0x100, "version" );
	CHECK( hdr->modelType  == 0,     "modelType" );
	CHECK( hdr->surfaceSize > 0,     "surfaceSize" );
	CHECK( hdr->surfaceSize == sz - (int)kHdrSize, "surfaceSize=%d expected %d", hdr->surfaceSize, sz - (int)kHdrSize );
	CHECK( hdr->axisMapSize == 0,    "axisMapSize" );

	// IVPS magic in unused[2].
	uint32_t ivps;
	std::memcpy( &ivps, &surf->unused[2], 4 );
	const uint32_t expectedIVPS = 'I' | ('V'<<8) | ('P'<<16) | ('S'<<24);
	CHECK( ivps == expectedIVPS, "IVPS magic" );

	// Mass center: for a unit cube (0..1), should be (0.5,0.5,0.5).
	CHECK( std::fabs( surf->massCenter[0] - 0.5f ) < 1e-3f, "massCenter x = %f", surf->massCenter[0] );
	CHECK( std::fabs( surf->massCenter[1] - 0.5f ) < 1e-3f, "massCenter y" );
	CHECK( std::fabs( surf->massCenter[2] - 0.5f ) < 1e-3f, "massCenter z" );

	// upper_limit_radius = distance from (0.5,0.5,0.5) to corner (1,1,1) = sqrt(0.75)
	CHECK( std::fabs( surf->upperLimitRadius - std::sqrt(0.75f) ) < 1e-3f,
	       "upperLimitRadius=%f expected %f", surf->upperLimitRadius, std::sqrt(0.75f) );

	// Ledgetree root offset should point into the IVP surface block; we laid
	// it out at offset 48 (right after IVPCompactSurface).
	CHECK( surf->offsetLedgetreeRoot == 48, "ledgetree root offset = %d", surf->offsetLedgetreeRoot );

	// The root node for N=1 is terminal.
	auto *root = reinterpret_cast<IVPCompactLedgeNode *>( buf.data() + kHdrSize + 48 );
	CHECK( root->offsetRight == 0, "root.offsetRight (terminal)" );
	CHECK( root->offsetLedge > 0,  "root.offsetLedge (points to ledge)" );

	auto *ledge = reinterpret_cast<IVPCompactLedge *>(
		reinterpret_cast<uint8_t *>( root ) + root->offsetLedge );
	CHECK( ledge->nTriangles >= 12, "cube should have ≥12 triangles, got %u", ledge->nTriangles );
	CHECK( ( ledge->flags & 0x3 ) == 0, "has_children_flag = 0 (leaf)" );
	CHECK( ( ( ledge->flags >> 2 ) & 0x3 ) == 1, "is_compact_flag = 1" );
	const uint32_t ledgeSizeDiv16 = ledge->flags >> 8;
	CHECK( ledgeSizeDiv16 > 0, "size_div_16 nonzero" );
	std::printf( "ledge: %u triangles, size=%u bytes\n", ledge->nTriangles, ledgeSizeDiv16 * 16 );

	std::printf( "%s (%d failures)\n", failures == 0 ? "PASS" : "FAIL", failures );
	return failures;
}
