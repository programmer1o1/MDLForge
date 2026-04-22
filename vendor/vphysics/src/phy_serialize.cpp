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

// Source uses (X forward, Y left, Z up); Valve's stock vphysics emits the
// .phy in IVP's body-local frame which is Source rotated +90° about the X
// axis — i.e. (x, y, z) -> (x, -z, y). Empirically verified by compiling
// niktoper's SMD with our tool and comparing mass-center and AABB-box
// fields against Valve's shipped niktoper_pm.phy: all solids agree field
// by field under this remap, disagree without it. Without this transform
// every hull renders rotated 90° about its bone's local X-axis in-engine.
static inline void SourceToIVP( float sx, float sy, float sz,
                                float &ox, float &oy, float &oz )
{
	ox =  sx;
	oy = -sz;
	oz =  sy;
}

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
// convex's AABB. AABB is stored in Source-space inches; convert to IVP
// meters (with axis remap) for the file.
void SetNodeBoundsFromConvex( IVPCompactLedgeNode &node, const CPhysConvex_Impl &cv )
{
	const float srcCx = 0.5f * ( cv.aabbMin.x + cv.aabbMax.x ) * kInchesToMeters;
	const float srcCy = 0.5f * ( cv.aabbMin.y + cv.aabbMax.y ) * kInchesToMeters;
	const float srcCz = 0.5f * ( cv.aabbMin.z + cv.aabbMax.z ) * kInchesToMeters;
	const float srcHx = 0.5f * ( cv.aabbMax.x - cv.aabbMin.x ) * kInchesToMeters;
	const float srcHy = 0.5f * ( cv.aabbMax.y - cv.aabbMin.y ) * kInchesToMeters;
	const float srcHz = 0.5f * ( cv.aabbMax.z - cv.aabbMin.z ) * kInchesToMeters;
	SourceToIVP( srcCx, srcCy, srcCz, node.center[0], node.center[1], node.center[2] );
	// Half-extents remap too (absolute values since they're unsigned).
	float hx, hy, hz;
	SourceToIVP( srcHx, srcHy, srcHz, hx, hy, hz );
	hx = std::fabs( hx ); hy = std::fabs( hy ); hz = std::fabs( hz );
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

	// --- Layout: matches Valve's shipped .phy files.
	//
	//   [IVPCompactSurface (48 bytes)]
	//   [leaf 0: ledge (16) + tris (16*nTri) + points (16*nPts)]
	//   [leaf 1: ...]
	//   ...
	//   [ledgetree nodes (28 bytes each)]
	//
	// Ledge blocks are placed FIRST so they stay 16-byte aligned (each leaf
	// block size is a multiple of 16). The 28-byte ledgetree nodes go at the
	// end where their smaller alignment is harmless. Placing nodes first
	// would leave every ledge/point block 12 bytes off — IVP's runtime does
	// 16-byte SSE-aligned loads on point coords, and on a 64-bit Source
	// engine the misalignment shows up as hulls rotated relative to the
	// bone they're supposed to follow.
	std::vector<size_t> ledgeOffsetPerLeaf( N, 0 );
	size_t cursor = kSurfaceSize;
	for ( size_t i = 0; i < N; ++i )
	{
		ledgeOffsetPerLeaf[ i ] = cursor;
		cursor += LedgeBlockBytes( c->convexes[ i ] );
	}
	const size_t ledgetreeBase = cursor;

	// --- Plan ledgetree node offsets (relative to start of IVP surface).
	std::vector<NodeMeta> nodes;
	nodes.reserve( NodeCountFor( N ) );
	const size_t rootIdx = PlanSubtree( 0, N, nodes, ledgetreeBase );
	assert( rootIdx == 0 );
	(void) rootIdx;

	cursor += nodes.size() * kNodeSize;
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
	// meter units with the Source→IVP axis remap applied.
	uint8_t *surfBase = dst + kHeaderSize;
	auto *surface = reinterpret_cast<IVPCompactSurface *>( surfBase );
	SourceToIVP(
		c->massCenter.x * kInchesToMeters,
		c->massCenter.y * kInchesToMeters,
		c->massCenter.z * kInchesToMeters,
		surface->massCenter[0], surface->massCenter[1], surface->massCenter[2] );
	// Principal moments of inertia per unit mass. Each convex's own
	// inertia diagonal is computed via tetrahedral integration in
	// CPhysConvex_Impl; aggregate across convexes weighted by volume,
	// then convert from inch² to meter² and apply the Source→IVP axis
	// swap (diagonal entries for Y and Z swap under the (x,-z,y)
	// rotation; X stays put).
	//
	// Without real values here — e.g., with isotropic unit inertia —
	// Source's constraint solver misbehaves: ragdolls sink through the
	// floor, respond to forces as if they have orders-of-magnitude too
	// much angular resistance, and fall noticeably slowly.
	float accIxx = 0.0f, accIyy = 0.0f, accIzz = 0.0f;
	float totalV = 0.0f;
	for ( const auto *cv : c->convexes )
	{
		accIxx += cv->inertiaDiag.x * cv->volume;
		accIyy += cv->inertiaDiag.y * cv->volume;
		accIzz += cv->inertiaDiag.z * cv->volume;
		totalV += cv->volume;
	}
	if ( totalV > 1e-6f )
	{
		accIxx /= totalV;
		accIyy /= totalV;
		accIzz /= totalV;
	}
	const float inchSqToMeterSq = kInchesToMeters * kInchesToMeters;
	surface->rotationInertia[0] = accIxx * inchSqToMeterSq;
	surface->rotationInertia[1] = accIzz * inchSqToMeterSq;  // Source Z → IVP Y
	surface->rotationInertia[2] = accIyy * inchSqToMeterSq;  // Source Y → IVP Z
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

		// Triangles + embedded edges.
		//
		// `pierce_index` is a plain triangle index identifying the face on
		// the far side of the hull that a ray from this triangle's
		// centroid along its inward normal exits through — IVP uses it to
		// bracket penetration-depth queries.
		//
		// `opposite_index` is a *signed 15-bit offset in 4-byte slot
		// units* from this edge to its twin half-edge in the adjacent
		// triangle — not a linear table index. The slot indexing treats
		// each triangle as occupying four 4-byte slots (header + 3
		// edges): slot(t, e) = 4*t + 1 + e. BuildOppositeEdgeOffsets
		// precomputes the signed (twin − self) value; we pack it through
		// PackEdge's 15-bit mask which naturally sign-extends negatives.
		uint8_t *triBase = ledgeBase + kLedgeSize;
		for ( size_t t = 0; t < nTri; ++t )
		{
			auto *tri = reinterpret_cast<IVPCompactTriangle *>( triBase + t * kTriSize );
			const uint16_t i0 = cv.triangles[ t * 3 + 0 ];
			const uint16_t i1 = cv.triangles[ t * 3 + 1 ];
			const uint16_t i2 = cv.triangles[ t * 3 + 2 ];
			const int16_t  o0 = cv.oppositeEdges[ t * 3 + 0 ];
			const int16_t  o1 = cv.oppositeEdges[ t * 3 + 1 ];
			const int16_t  o2 = cv.oppositeEdges[ t * 3 + 2 ];
			const uint16_t pierce = cv.pierceIndices[ t ];
			tri->triHeader = PackTriHeader( static_cast<uint32_t>( t ), pierce, 0, 0 );
			tri->edges[0].bits = PackEdge( i0, static_cast<uint32_t>( o0 ), 0 );
			tri->edges[1].bits = PackEdge( i1, static_cast<uint32_t>( o1 ), 0 );
			tri->edges[2].bits = PackEdge( i2, static_cast<uint32_t>( o2 ), 0 );
		}

		// Points — padded to 16 bytes (4 floats).
		uint8_t *ptBase = triBase + nTri * kTriSize;
		for ( size_t p = 0; p < nPts; ++p )
		{
			auto *pt = reinterpret_cast<IVPCompactPoint *>( ptBase + p * kPointSize );
			SourceToIVP(
				cv.points[p].x * kInchesToMeters,
				cv.points[p].y * kInchesToMeters,
				cv.points[p].z * kInchesToMeters,
				pt->xyz[0], pt->xyz[1], pt->xyz[2] );
			pt->xyz[3] = 0.0f;
		}
	}

	return totalSize;
}

} // namespace mdlforge_vphysics
