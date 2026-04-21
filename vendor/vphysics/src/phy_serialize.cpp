// Serialize CPhysCollide_Impl → Source 1 .phy byte stream (IVP compact surface).
//
// Output structure per collide:
//
//   offset 0   : CompactSurfaceHeader      (32 bytes)
//   offset 32  : IVPCompactSurface         (48 bytes)
//   offset 80  : ledgetree nodes           (32 bytes each, 2*N-1 for N convexes)
//   ...        : per-convex data, each:
//                  IVPCompactLedge         (16 bytes)
//                  triangles               (16 bytes * nTri)
//                  points                  (16 bytes * nPts)
//
// For N > 1 convexes we build a left-spine binary tree: root has the left
// subtree laid out contiguously behind it, and the right child is a single
// leaf written after the whole left subtree. This is structurally valid for
// IVP runtime traversal (broadphase uses node bounding spheres) while being
// simple to emit. Each IVPCompactLedgeNode is 28 bytes.

#include "phy_serialize.h"
#include "phy_format.h"
#include "cphysconvex.h"

// Disable Valve's min/max macros before using STL algorithm functions.
#include "tier0/valve_minmax_off.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>

namespace mdlforge_vphysics
{

namespace
{

// Source's public vphysics interface exposes distances in inches (see
// public/vphysics_interface.h: METERS_PER_INCH). Internally IVP operates
// in meters, and the on-disk .phy file is in the IVP coordinate space —
// Valve's stock vphysics.dll converts inch-space inputs from studiomdl
// into meters before serializing. We keep hull data in inches at runtime
// (so ConvexVolume / ConvexSurfaceArea return inch-native numbers that
// studiomdl's mass calculations expect) and only scale at write time.
constexpr float kInchesToMeters = 0.0254f;

constexpr size_t kHeaderSize  = sizeof( CompactSurfaceHeader );   // 28
constexpr size_t kSurfaceSize = sizeof( IVPCompactSurface );      // 48
constexpr size_t kNodeSize    = sizeof( IVPCompactLedgeNode );    // 28
constexpr size_t kLedgeSize   = sizeof( IVPCompactLedge );        // 16
constexpr size_t kTriSize     = sizeof( IVPCompactTriangle );     // 16
constexpr size_t kPointSize   = sizeof( IVPCompactPoint );        // 16

// Per-convex ledge block size: header + N tris + P points.
size_t LedgeBlockBytes( const CPhysConvex_Impl *c )
{
	const size_t nTri = c->triangles.size() / 3;
	const size_t nPts = c->points.size();
	return kLedgeSize + nTri * kTriSize + nPts * kPointSize;
}

// Total node count in a left-spine tree with N leaves (N convexes):
//   N=1 → 1 node (terminal leaf)
//   N>=2 → 2N - 1 nodes (N-1 internal + N terminal)
size_t NodeCountFor( size_t N )
{
	return ( N == 0 ) ? 0 : ( N == 1 ) ? 1 : ( 2 * N - 1 );
}

// ---------------------------------------------------------------------------
// Total serialized size, excluding the 4-byte "int solidSize" prefix that
// studiomdl writes outside CollideWrite.
// ---------------------------------------------------------------------------
int32_t ComputeCollideSizeInternal( const CPhysCollide_Impl *c )
{
	const size_t N = c->convexes.size();
	size_t total = kHeaderSize + kSurfaceSize + NodeCountFor( N ) * kNodeSize;
	for ( const auto *cv : c->convexes )
		total += LedgeBlockBytes( cv );
	return static_cast<int32_t>( total );
}

// ---------------------------------------------------------------------------
// Ledgetree layout planner.
//
// Given N convex leaves, emit (2N-1) nodes in a left-spine linear layout so
// that each internal node's left child immediately follows (offset 32), and
// the right child is written after the entire left subtree.
//
// Returns a vector of per-node metadata filled by the planner. ledgeOffset[i]
// is populated with the absolute offset (within the IVP compact surface) of
// the leaf node that corresponds to convexes[i].
//
// plannedNodes layout in memory:
//   For N=1: [leaf 0]
//   For N=2: [root] [left=leaf 1] [right=leaf 0]
//   For N=3: [root] [left=internal] [left.left=leaf 2] [left.right=leaf 1] [right=leaf 0]
// ---------------------------------------------------------------------------
struct NodeMeta
{
	bool    terminal;     // if true, this node points to a ledge
	size_t  leafIndex;    // index into convexes[], if terminal
	// For internal nodes: absolute offsets (relative to start of surface) of
	// the left and right child nodes.
	size_t  leftNodeOffset;
	size_t  rightNodeOffset;
	// For terminal nodes: filled in after leaf block layout.
	size_t  ledgeOffset;
};

// Recursive: lay out subtree for leaves[startLeaf..] (left-spine = first leaf
// goes to right child, rest to left subtree). Pushes nodes onto `nodes` and
// returns the index of the root of this subtree.
size_t PlanSubtree( size_t firstLeaf, size_t leafCount, std::vector<NodeMeta> &nodes,
                    size_t surfaceBase )
{
	if ( leafCount == 1 )
	{
		NodeMeta n{};
		n.terminal = true;
		n.leafIndex = firstLeaf;
		size_t idx = nodes.size();
		nodes.push_back( n );
		return idx;
	}
	// Internal node: reserve slot first so left subtree can be contiguous.
	size_t rootIdx = nodes.size();
	nodes.push_back( NodeMeta{} );

	// Left subtree = all but the first leaf, immediately following us.
	size_t leftIdx = PlanSubtree( firstLeaf + 1, leafCount - 1, nodes, surfaceBase );
	// Right subtree = single leaf (the "first" one) after the left subtree.
	size_t rightIdx = PlanSubtree( firstLeaf, 1, nodes, surfaceBase );

	nodes[rootIdx].terminal        = false;
	nodes[rootIdx].leftNodeOffset  = surfaceBase + leftIdx  * kNodeSize;
	nodes[rootIdx].rightNodeOffset = surfaceBase + rightIdx * kNodeSize;
	return rootIdx;
}

// Fill the bounding sphere + half-extent fields of a leaf node from its
// convex's AABB. AABB is stored in inches; convert to meters for the file.
void SetNodeBoundsFromConvex( IVPCompactLedgeNode &node, const CPhysConvex_Impl &cv )
{
	const float cx = 0.5f * ( cv.aabbMin.x + cv.aabbMax.x ) * kInchesToMeters;
	const float cy = 0.5f * ( cv.aabbMin.y + cv.aabbMax.y ) * kInchesToMeters;
	const float cz = 0.5f * ( cv.aabbMin.z + cv.aabbMax.z ) * kInchesToMeters;
	const float hx = 0.5f * ( cv.aabbMax.x - cv.aabbMin.x ) * kInchesToMeters;
	const float hy = 0.5f * ( cv.aabbMax.y - cv.aabbMin.y ) * kInchesToMeters;
	const float hz = 0.5f * ( cv.aabbMax.z - cv.aabbMin.z ) * kInchesToMeters;
	node.center[0] = cx;
	node.center[1] = cy;
	node.center[2] = cz;
	node.radius    = std::sqrt( hx * hx + hy * hy + hz * hz );
	const float scale = 255.0f / std::max( node.radius, 1e-4f );
	auto clampByte = []( float v ) -> uint8_t {
		v = std::max( 0.0f, std::min( 255.0f, v ) );
		return static_cast<uint8_t>( v );
	};
	node.boxHalfExtents[0] = clampByte( hx * scale );
	node.boxHalfExtents[1] = clampByte( hy * scale );
	node.boxHalfExtents[2] = clampByte( hz * scale );
	node.pad = 0;
}

// Aggregate bounding sphere for an internal node covering both children.
void SetInternalNodeBounds( IVPCompactLedgeNode &node, const IVPCompactLedgeNode &left, const IVPCompactLedgeNode &right )
{
	node.center[0] = 0.5f * ( left.center[0] + right.center[0] );
	node.center[1] = 0.5f * ( left.center[1] + right.center[1] );
	node.center[2] = 0.5f * ( left.center[2] + right.center[2] );
	auto dist = []( const float a[3], const float b[3] ) {
		const float dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
		return std::sqrt( dx * dx + dy * dy + dz * dz );
	};
	const float r = std::max(
		dist( node.center, left.center )  + left.radius,
		dist( node.center, right.center ) + right.radius );
	node.radius = r;
	node.boxHalfExtents[0] = 255;
	node.boxHalfExtents[1] = 255;
	node.boxHalfExtents[2] = 255;
	node.pad = 0;
}

} // namespace

int32_t ComputeCollideSize( const CPhysCollide_Impl *c )
{
	if ( c == nullptr || c->convexes.empty() )
		return 0;
	return ComputeCollideSizeInternal( c );
}

int32_t SerializeCollide( uint8_t *dst, const CPhysCollide_Impl *c )
{
	if ( c == nullptr || c->convexes.empty() )
		return 0;

	const int32_t totalSize = ComputeCollideSizeInternal( c );
	std::memset( dst, 0, totalSize );

	const size_t N = c->convexes.size();

	// --- Plan ledgetree node offsets (relative to start of IVP surface).
	std::vector<NodeMeta> nodes;
	nodes.reserve( NodeCountFor( N ) );
	const size_t ledgetreeBase = kSurfaceSize;    // ledgetree starts right after IVPCompactSurface
	const size_t rootIdx = PlanSubtree( 0, N, nodes, ledgetreeBase );
	assert( rootIdx == 0 );
	(void) rootIdx;

	// --- Plan leaf-block offsets (relative to the IVP surface start).
	// Leaves are written after all ledgetree nodes, in the order leaves
	// appear in `nodes`. For each leaf we store: ledge → triangles → points.
	size_t cursor = ledgetreeBase + nodes.size() * kNodeSize;
	std::vector<size_t> ledgeOffsetPerLeaf( N, 0 );
	for ( const NodeMeta &n : nodes )
	{
		if ( !n.terminal )
			continue;
		ledgeOffsetPerLeaf[ n.leafIndex ] = cursor;
		cursor += LedgeBlockBytes( c->convexes[ n.leafIndex ] );
	}
	// `cursor` is now the total byte size of the IVP surface (header excluded).
	const size_t ivpSurfaceBytes = cursor;

	// --- Write CompactSurfaceHeader (outer VPHY wrapper).
	auto *hdr = reinterpret_cast<CompactSurfaceHeader *>( dst );
	hdr->vphysicsID  = kVPhysicsID_VPHY;
	hdr->version     = kCompactSurfaceVersion;
	hdr->modelType   = kModelType_ConvexSurface;
	hdr->surfaceSize = static_cast<int32_t>( ivpSurfaceBytes );
	hdr->dragAxisAreas[0] = c->orthographicAreas.x;
	hdr->dragAxisAreas[1] = c->orthographicAreas.y;
	hdr->dragAxisAreas[2] = c->orthographicAreas.z;
	hdr->axisMapSize = 0;

	// --- Write IVPCompactSurface. Everything here lives in IVP's native
	// meter units; convert from the inch-space our runtime stores.
	uint8_t *surfBase = dst + kHeaderSize;
	auto *surface = reinterpret_cast<IVPCompactSurface *>( surfBase );
	surface->massCenter[0]      = c->massCenter.x * kInchesToMeters;
	surface->massCenter[1]      = c->massCenter.y * kInchesToMeters;
	surface->massCenter[2]      = c->massCenter.z * kInchesToMeters;
	// Placeholder principal inertia — the engine uses these when building
	// the rigid body. Real values would require integrating over the hull;
	// unit values here give reasonable rotational behavior for ragdolls
	// while the shipped mass acts as the dominant factor.
	surface->rotationInertia[0] = 1.0f;
	surface->rotationInertia[1] = 1.0f;
	surface->rotationInertia[2] = 1.0f;
	// Upper limit radius = max distance (in meters) from mass center to
	// any hull vertex.
	float r2maxInches = 0.0f;
	for ( const auto *cv : c->convexes )
	{
		for ( const Vector &p : cv->points )
		{
			const float dx = p.x - c->massCenter.x;
			const float dy = p.y - c->massCenter.y;
			const float dz = p.z - c->massCenter.z;
			r2maxInches = std::max( r2maxInches, dx * dx + dy * dy + dz * dz );
		}
	}
	surface->upperLimitRadius       = std::sqrt( r2maxInches ) * kInchesToMeters;
	surface->maxDeviationAndByteSize = PackMaxDevAndByteSize( 0, static_cast<uint32_t>( ivpSurfaceBytes ) );
	surface->offsetLedgetreeRoot    = static_cast<int32_t>( ledgetreeBase );  // root is the first node
	surface->unused[0] = 0;
	surface->unused[1] = 0;
	// unused[2] stores "IVPS" magic — a well-known validity marker.
	std::memcpy( &surface->unused[2], &kIVPS, sizeof( kIVPS ) );

	// --- Write each ledgetree node.
	for ( size_t i = 0; i < nodes.size(); ++i )
	{
		uint8_t *nodeBase = surfBase + ledgetreeBase + i * kNodeSize;
		auto *node = reinterpret_cast<IVPCompactLedgeNode *>( nodeBase );
		const NodeMeta &m = nodes[i];
		if ( m.terminal )
		{
			const CPhysConvex_Impl &cv = *c->convexes[ m.leafIndex ];
			node->offsetRightNode    = 0;
			node->offsetCompactLedge = static_cast<int32_t>( ledgeOffsetPerLeaf[ m.leafIndex ] - ( ledgetreeBase + i * kNodeSize ) );
			SetNodeBoundsFromConvex( *node, cv );
		}
		else
		{
			const size_t absNodeOffset = ledgetreeBase + i * kNodeSize;
			node->offsetRightNode    = static_cast<int32_t>( m.rightNodeOffset - absNodeOffset );
			node->offsetCompactLedge = 0;
			// Left child immediately follows at offset kNodeSize — consistent
			// with IVP's convention; bounds filled in pass 2 below.
		}
	}
	// --- Pass 2: compute internal-node bounding spheres now that all leaf
	// nodes have populated bounds.
	for ( size_t i = nodes.size(); i-- > 0; )
	{
		NodeMeta &m = nodes[i];
		if ( m.terminal )
			continue;
		uint8_t *nodeBase = surfBase + ledgetreeBase + i * kNodeSize;
		auto *node  = reinterpret_cast<IVPCompactLedgeNode *>( nodeBase );
		auto *left  = reinterpret_cast<IVPCompactLedgeNode *>( surfBase + m.leftNodeOffset );
		auto *right = reinterpret_cast<IVPCompactLedgeNode *>( surfBase + m.rightNodeOffset );
		SetInternalNodeBounds( *node, *left, *right );
	}

	// --- Write each convex leaf's data: ledge + triangles + points.
	for ( size_t leafI = 0; leafI < N; ++leafI )
	{
		const CPhysConvex_Impl &cv = *c->convexes[ leafI ];
		const size_t nTri = cv.triangles.size() / 3;
		const size_t nPts = cv.points.size();

		uint8_t *ledgeBase = surfBase + ledgeOffsetPerLeaf[ leafI ];
		auto *ledge = reinterpret_cast<IVPCompactLedge *>( ledgeBase );
		const size_t ledgeBytes = kLedgeSize + nTri * kTriSize + nPts * kPointSize;

		ledge->cPointOffset = static_cast<int32_t>( kLedgeSize + nTri * kTriSize );
		// Back-reference to the ledgetree node that owns this ledge is
		// optional; studiomdl runtime uses it for fast ledge→node lookups.
		// A zero value means "not back-referenced" which is accepted.
		ledge->ledgetreeNodeOffsetOrClientData = static_cast<int32_t>( cv.gameData );
		ledge->flagsAndSizeDiv16 = PackLedgeFlagsAndSize(
			/*has_children*/ 0,
			/*is_compact*/   1,
			/*dummy*/        0,
			/*size_div_16*/  static_cast<uint32_t>( ledgeBytes / 16 ) );
		ledge->nTriangles = static_cast<uint16_t>( nTri );
		ledge->unknown    = 0;

		// Triangles. For each triangle pull the 3 point indices and emit a
		// minimal edge list (without adjacency — valid for collision, only
		// sub-optimal for ray-cast acceleration).
		uint8_t *triBase = ledgeBase + kLedgeSize;
		for ( size_t t = 0; t < nTri; ++t )
		{
			auto *tri = reinterpret_cast<IVPCompactTriangle *>( triBase + t * kTriSize );
			const uint16_t i0 = cv.triangles[ t * 3 + 0 ];
			const uint16_t i1 = cv.triangles[ t * 3 + 1 ];
			const uint16_t i2 = cv.triangles[ t * 3 + 2 ];
			tri->triHeader = PackTriHeader( static_cast<uint32_t>( t ), 0, 0, 0 );
			tri->edges[0].bits = PackEdge( i0, 0, 0 );
			tri->edges[1].bits = PackEdge( i1, 0, 0 );
			tri->edges[2].bits = PackEdge( i2, 0, 0 );
		}

		// Points — padded to 16 bytes (4 floats).
		uint8_t *ptBase = triBase + nTri * kTriSize;
		for ( size_t p = 0; p < nPts; ++p )
		{
			auto *pt = reinterpret_cast<IVPCompactPoint *>( ptBase + p * kPointSize );
			pt->xyz[0] = cv.points[p].x * kInchesToMeters;
			pt->xyz[1] = cv.points[p].y * kInchesToMeters;
			pt->xyz[2] = cv.points[p].z * kInchesToMeters;
			pt->xyz[3] = 0.0f;
		}
	}

	return totalSize;
}

} // namespace mdlforge_vphysics
