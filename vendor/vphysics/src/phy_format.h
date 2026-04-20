// Binary layout constants for Source 1 .phy file contents.
//
// Reference sources (clean; used only to understand field meanings, sizes,
// and offsets — structs below are our own redeclarations):
//   * Valve public SDK header: public/phyfile.h (outer phyheader_t)
//   * niklasb/bspfuzz (BSD-3-Clause) — binary layout for
//     legacysurfaceheader_t and compactsurfaceheader_t.
//   * gelly-gmod/PHYParser (GPL-3) — field names / bitfield interpretations,
//     referenced for interoperability understanding only.
//
// A file-format layout is a fact, not copyrightable expression. The code
// below is original. No code from any leaked Valve source tree has been
// used.

#pragma once

#include <cstdint>

namespace mdlforge_vphysics
{

// 'V','P','H','Y' little-endian
static constexpr uint32_t kVPhysicsID_VPHY = 'V' | ( 'P' << 8 ) | ( 'H' << 16 ) | ( 'Y' << 24 );
// 'I','V','P','S' little-endian — magic marking a valid IVP compact surface.
static constexpr uint32_t kIVPS = 'I' | ( 'V' << 8 ) | ( 'P' << 16 ) | ( 'S' << 24 );

static constexpr int16_t  kCompactSurfaceVersion = 0x100;
static constexpr int16_t  kModelType_ConvexSurface = 0;

// ---- compactsurfaceheader_t — 28 bytes, outer VPHY wrapper -----------------
// Note: studiomdl's write.cpp prepends a separate `int solidSize` outside
// the block CollideWrite produces. Some references (e.g. niklasb/bspfuzz)
// fold that prepended int into their compactsurfaceheader_t declaration,
// producing a 32-byte struct. The ACTUAL bytes CollideWrite emits start at
// vphysicsID and total 28 bytes.
struct CompactSurfaceHeader
{
	uint32_t vphysicsID;      // kVPhysicsID_VPHY
	int16_t  version;
	int16_t  modelType;
	int32_t  surfaceSize;     // bytes of IVP compact surface + tree + ledges + tris + pts
	float    dragAxisAreas[3];
	int32_t  axisMapSize;
};
static_assert( sizeof( CompactSurfaceHeader ) == 28, "CompactSurfaceHeader must be 28 bytes" );

// ---- ivpcompactsurface_t — 48 bytes ----------------------------------------
struct IVPCompactSurface
{
	float    massCenter[3];
	float    rotationInertia[3];
	float    upperLimitRadius;
	// max_deviation : 8 | byte_size : 24 — little-endian bit packing.
	uint32_t maxDeviationAndByteSize;
	int32_t  offsetLedgetreeRoot;       // offset from &this to root ledgetree node
	int32_t  unused[3];                 // unused[2] = kIVPS
};
static_assert( sizeof( IVPCompactSurface ) == 48, "IVPCompactSurface must be 48 bytes" );

// ---- ivpcompactledgenode_t — 28 bytes --------------------------------------
// offset_right_node == 0 → this node is a terminal leaf pointing to a ledge
// via offset_compact_node; the left child (if non-terminal) immediately
// follows this node in memory at offset sizeof(IVPCompactLedgeNode).
struct IVPCompactLedgeNode
{
	int32_t offsetRightNode;
	int32_t offsetCompactLedge;         // byte offset from &this to IVPCompactLedge (leaf only)
	float   center[3];
	float   radius;
	uint8_t boxHalfExtents[3];          // quantized half-extents
	uint8_t pad;
};
static_assert( sizeof( IVPCompactLedgeNode ) == 28, "IVPCompactLedgeNode must be 28 bytes" );

// ---- ivpcompactledge_t — 16 bytes ------------------------------------------
struct IVPCompactLedge
{
	int32_t  cPointOffset;              // offset from &this to points buffer
	int32_t  ledgetreeNodeOffsetOrClientData;
	// has_children_flag : 2 | is_compact_flag : 2 | dummy : 4 | size_div_16 : 24
	uint32_t flagsAndSizeDiv16;
	uint16_t nTriangles;
	int16_t  unknown;
};
static_assert( sizeof( IVPCompactLedge ) == 16, "IVPCompactLedge must be 16 bytes" );

// ---- ivpcompactedge_t — 4 bytes --------------------------------------------
// start_point_index : 16 | opposite_index : 15 | is_virtual : 1
struct IVPCompactEdge
{
	uint32_t bits;
};
static_assert( sizeof( IVPCompactEdge ) == 4, "IVPCompactEdge must be 4 bytes" );

// ---- ivpcompacttriangle_t — 16 bytes ---------------------------------------
// tri_index : 12 | pierce_index : 12 | mat_index : 7 | is_virtual : 1
struct IVPCompactTriangle
{
	uint32_t       triHeader;
	IVPCompactEdge edges[3];
};
static_assert( sizeof( IVPCompactTriangle ) == 16, "IVPCompactTriangle must be 16 bytes" );

// Points are stored as 4 floats (x,y,z,unused) — 16-byte aligned.
struct IVPCompactPoint
{
	float xyz[4];
};
static_assert( sizeof( IVPCompactPoint ) == 16, "IVPCompactPoint must be 16 bytes" );

// Pack a `max_deviation:8 | byte_size:24` bitfield.
static inline uint32_t PackMaxDevAndByteSize( uint8_t maxDev, uint32_t byteSize )
{
	return ( static_cast<uint32_t>( maxDev ) & 0xFFu ) | ( ( byteSize & 0x00FFFFFFu ) << 8 );
}

// Pack ledge flags: has_children_flag:2 | is_compact_flag:2 | dummy:4 | size_div_16:24.
static inline uint32_t PackLedgeFlagsAndSize( uint32_t hasChildren, uint32_t isCompact, uint32_t dummy, uint32_t sizeDiv16 )
{
	return ( hasChildren & 0x3u )
	     | ( ( isCompact & 0x3u ) << 2 )
	     | ( ( dummy     & 0xFu ) << 4 )
	     | ( ( sizeDiv16 & 0x00FFFFFFu ) << 8 );
}

// Pack triangle header.
static inline uint32_t PackTriHeader( uint32_t triIndex, uint32_t pierceIndex, uint32_t matIndex, uint32_t isVirtual )
{
	return ( triIndex    & 0xFFFu )
	     | ( ( pierceIndex & 0xFFFu ) << 12 )
	     | ( ( matIndex    & 0x7Fu )  << 24 )
	     | ( ( isVirtual   & 0x1u )   << 31 );
}

// Pack edge: start:16 | opposite:15 | virtual:1.
static inline uint32_t PackEdge( uint32_t startPoint, uint32_t oppositeIdx, uint32_t isVirtual )
{
	return ( startPoint  & 0xFFFFu )
	     | ( ( oppositeIdx & 0x7FFFu ) << 16 )
	     | ( ( isVirtual   & 0x1u )    << 31 );
}

} // namespace mdlforge_vphysics
