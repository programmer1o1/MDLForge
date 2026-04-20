// Minimal surfaceproperties.txt parser backing IPhysicsSurfaceProps.
//
// Implements the subset studiomdl consumes: density, thickness, friction,
// elasticity. Other keys are parsed but discarded. "base" inheritance is
// supported shallowly (one lookup chain, no cycles).

#pragma once

#include <string>
#include <unordered_map>
#include <vector>

namespace mdlforge_vphysics
{

struct SurfacePropEntry
{
	std::string name;
	std::string base;         // name of parent, if any
	float       density     = 2000.0f;
	float       thickness   = 0.0f;
	float       friction    = 0.8f;
	float       elasticity  = 0.25f;
	bool        hasDensity    = false;
	bool        hasThickness  = false;
	bool        hasFriction   = false;
	bool        hasElasticity = false;
};

class SurfacePropsTable
{
public:
	// Parse a single surfaceproperties.txt blob. Returns number of entries
	// added (existing entries with the same name get overwritten).
	int Parse( const char *pText );

	int   Count() const               { return static_cast<int>( m_entries.size() ); }
	int   IndexOf( const char *name ) const;
	const SurfacePropEntry *At( int index ) const;

	// Compose effective properties (following base chain) for the given index.
	// Returns false if out of range.
	bool  GetEffective( int index, float *density, float *thickness, float *friction, float *elasticity ) const;

private:
	std::vector<SurfacePropEntry>            m_entries;
	std::unordered_map<std::string, int>     m_nameToIndex;
};

} // namespace mdlforge_vphysics
