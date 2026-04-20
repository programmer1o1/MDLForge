// phy_dump — parse a Source 1 .phy file and print its structure.
//
// Used for validating that our CollideWrite produces files structurally
// compatible with what Valve's stock studiomdl emits. Works on any real
// .phy captured from a shipped model.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#pragma pack(push, 1)
struct phyheader_t
{
	int32_t size;
	int32_t id;
	int32_t solidCount;
	int32_t checksum;
};
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
struct IVPCompactTriangle
{
	uint32_t triHeader;
	uint32_t edges[3];
};
struct IVPCompactPoint
{
	float xyz[4];
};
#pragma pack(pop)

static void PrintMagic( const char *label, uint32_t v )
{
	char s[5] = {};
	std::memcpy( s, &v, 4 );
	for ( int i = 0; i < 4; ++i )
		if ( s[i] < 0x20 || s[i] > 0x7E ) s[i] = '.';
	std::printf( "%s 0x%08x (\"%s\")\n", label, v, s );
}

static int nodeCount = 0;
static int ledgeCount = 0;
static int triTotal = 0;
static int ptTotal = 0;

static void WalkLedge( const uint8_t *surfaceBase, const IVPCompactLedge *ledge, int depth )
{
	++ledgeCount;
	uint32_t sizeDiv16 = ledge->flags >> 8;
	std::printf( "%*sLedge @ +%ld  triangles=%u  size=%u bytes  flags=0x%x (hasChildren=%u compact=%u)\n",
		depth * 2, "",
		(long)( reinterpret_cast<const uint8_t *>( ledge ) - surfaceBase ),
		ledge->nTriangles, sizeDiv16 * 16,
		ledge->flags,
		ledge->flags & 3, ( ledge->flags >> 2 ) & 3 );

	triTotal += ledge->nTriangles;

	// Gather point indices referenced by triangles to count points.
	const auto *tris = reinterpret_cast<const IVPCompactTriangle *>(
		reinterpret_cast<const uint8_t *>( ledge ) + sizeof( IVPCompactLedge ) );
	int maxPointIdx = -1;
	for ( uint16_t t = 0; t < ledge->nTriangles; ++t )
	{
		for ( int e = 0; e < 3; ++e )
		{
			const int pi = tris[t].edges[e] & 0xFFFF;
			if ( pi > maxPointIdx ) maxPointIdx = pi;
		}
	}
	const int nPts = maxPointIdx + 1;
	ptTotal += nPts;

	const auto *pts = reinterpret_cast<const IVPCompactPoint *>(
		reinterpret_cast<const uint8_t *>( ledge ) + ledge->cPointOffset );
	std::printf( "%*s  %d points (inferred), first=(%.3f, %.3f, %.3f)\n",
		depth * 2, "", nPts,
		pts ? pts[0].xyz[0] : 0, pts ? pts[0].xyz[1] : 0, pts ? pts[0].xyz[2] : 0 );
}

static const uint8_t *g_surfaceEnd = nullptr;

static void WalkLedgetree( const uint8_t *surfaceBase, const IVPCompactLedgeNode *node, int depth, int maxDepth = 16 )
{
	if ( depth > maxDepth ) { std::printf( "%*s[max depth]\n", depth * 2, "" ); return; }
	const uint8_t *np = reinterpret_cast<const uint8_t *>( node );
	if ( np < surfaceBase || np + sizeof( IVPCompactLedgeNode ) > g_surfaceEnd )
	{
		std::printf( "%*s[node out of bounds +%ld]\n", depth * 2, "", (long)( np - surfaceBase ) );
		return;
	}
	++nodeCount;
	std::printf( "%*sNode @ +%ld  right=%d  ledgeOff=%d  center=(%.2f,%.2f,%.2f) r=%.2f  box=(%u,%u,%u)\n",
		depth * 2, "", (long)( np - surfaceBase ),
		node->offsetRight, node->offsetLedge,
		node->center[0], node->center[1], node->center[2], node->radius,
		node->box[0], node->box[1], node->box[2] );

	if ( node->offsetRight == 0 )
	{
		const uint8_t *lp = np + node->offsetLedge;
		if ( lp < surfaceBase || lp + sizeof( IVPCompactLedge ) > g_surfaceEnd )
		{
			std::printf( "%*s[ledge out of bounds]\n", depth * 2, "" );
			return;
		}
		WalkLedge( surfaceBase, reinterpret_cast<const IVPCompactLedge *>( lp ), depth + 1 );
	}
	else
	{
		const auto *left  = reinterpret_cast<const IVPCompactLedgeNode *>( np + sizeof( IVPCompactLedgeNode ) );
		const auto *right = reinterpret_cast<const IVPCompactLedgeNode *>( np + node->offsetRight );
		WalkLedgetree( surfaceBase, left,  depth + 1, maxDepth );
		WalkLedgetree( surfaceBase, right, depth + 1, maxDepth );
	}
}

int main( int argc, char **argv )
{
	if ( argc < 2 ) { std::fprintf( stderr, "usage: %s <file.phy>\n", argv[0] ); return 2; }

	FILE *f = std::fopen( argv[1], "rb" );
	if ( !f ) { std::perror( argv[1] ); return 1; }
	std::fseek( f, 0, SEEK_END );
	long sz = std::ftell( f );
	std::fseek( f, 0, SEEK_SET );
	std::vector<uint8_t> bytes( sz );
	std::fread( bytes.data(), 1, sz, f );
	std::fclose( f );

	std::printf( "=== %s — %ld bytes ===\n", argv[1], sz );

	auto *hdr = reinterpret_cast<phyheader_t *>( bytes.data() );
	std::printf( "phyheader_t: size=%d id=%d solidCount=%d checksum=0x%08x\n",
		hdr->size, hdr->id, hdr->solidCount, hdr->checksum );

	const uint8_t *cursor = bytes.data() + hdr->size;

	for ( int s = 0; s < hdr->solidCount; ++s )
	{
		int32_t solidSize = *reinterpret_cast<const int32_t *>( cursor );
		std::printf( "\n--- solid %d : %d bytes ---\n", s, solidSize );
		cursor += 4;

		const uint8_t *solidStart = cursor;
		auto *cHdr = reinterpret_cast<const CompactSurfaceHeader *>( solidStart );
		std::printf( "CompactSurfaceHeader: surfaceSize=%d version=0x%x modelType=%d\n",
			cHdr->surfaceSize, cHdr->version, cHdr->modelType );
		PrintMagic( "  vphysicsID =", cHdr->vphysicsID );
		std::printf( "  dragAxisAreas = (%.3f, %.3f, %.3f)  axisMapSize = %d\n",
			cHdr->dragAxisAreas[0], cHdr->dragAxisAreas[1], cHdr->dragAxisAreas[2], cHdr->axisMapSize );

		const uint8_t *surfBase = solidStart + sizeof( CompactSurfaceHeader );
		auto *surf = reinterpret_cast<const IVPCompactSurface *>( surfBase );
		std::printf( "IVPCompactSurface:\n" );
		std::printf( "  massCenter = (%.4f, %.4f, %.4f)\n", surf->massCenter[0], surf->massCenter[1], surf->massCenter[2] );
		std::printf( "  rotInertia = (%.4f, %.4f, %.4f)\n", surf->rotInertia[0], surf->rotInertia[1], surf->rotInertia[2] );
		std::printf( "  upperLimitRadius = %.4f\n", surf->upperLimitRadius );
		std::printf( "  byteSize = %u   maxDev = %u\n",
			( surf->maxDevAndByteSize >> 8 ) & 0x00FFFFFF,
			surf->maxDevAndByteSize & 0xFF );
		std::printf( "  offsetLedgetreeRoot = %d\n", surf->offsetLedgetreeRoot );
		PrintMagic( "  unused[2] =", static_cast<uint32_t>( surf->unused[2] ) );

		// Walk the tree.
		nodeCount = ledgeCount = triTotal = ptTotal = 0;
		g_surfaceEnd = surfBase + cHdr->surfaceSize;
		const auto *root = reinterpret_cast<const IVPCompactLedgeNode *>( surfBase + surf->offsetLedgetreeRoot );
		std::printf( "\nLedgetree walk:\n" );
		WalkLedgetree( surfBase, root, 0 );
		std::printf( "\nSummary: %d nodes, %d ledges, %d triangles, ~%d points\n",
			nodeCount, ledgeCount, triTotal, ptTotal );

		cursor += solidSize;
	}

	// Trailing KV1 block (solid{...} entries).
	if ( cursor < bytes.data() + sz )
	{
		long tail = ( bytes.data() + sz ) - cursor;
		std::printf( "\n=== Trailing KV1 block: %ld bytes ===\n", tail );
		// Print first 300 chars for a peek.
		size_t peek = std::min<long>( 300, tail );
		std::fwrite( cursor, 1, peek, stdout );
		if ( peek < static_cast<size_t>( tail ) )
			std::printf( "\n... [%ld bytes truncated] ...\n", tail - (long)peek );
		std::printf( "\n" );
	}
	return 0;
}
