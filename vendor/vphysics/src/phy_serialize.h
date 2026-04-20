// Serialization of our internal CPhysCollide_Impl into the Source 1 .phy
// byte format (IVP compact surface).

#pragma once

#include "cphyscollide.h"

#include <cstdint>
#include <vector>

namespace mdlforge_vphysics
{

// Serialized byte size for a given CPhysCollide, matching exactly what
// CollideSerialize would write. Returns 0 if c is null / empty.
int32_t ComputeCollideSize( const CPhysCollide_Impl *c );

// Write the serialized bytes to dst. dst must point to at least
// ComputeCollideSize(c) bytes. Returns the number of bytes written.
int32_t SerializeCollide( uint8_t *dst, const CPhysCollide_Impl *c );

} // namespace mdlforge_vphysics
