//========= Copyright Valve Corporation, All rights reserved. ============//
//
// Purpose:
//
//===========================================================================//

#include "mathlib/ssemath.h"
#include "mathlib/ssequaternion.h"

const fltx4 Four_PointFives={0.5,0.5,0.5,0.5};
#ifndef _X360
const fltx4 Four_Zeros={0.0,0.0,0.0,0.0};
const fltx4 Four_Ones={1.0,1.0,1.0,1.0};
#endif
const fltx4 Four_Twos={2.0,2.0,2.0,2.0};
const fltx4 Four_Threes={3.0,3.0,3.0,3.0};
const fltx4 Four_Fours={4.0,4.0,4.0,4.0};
const fltx4 Four_Origin={0,0,0,1};
const fltx4 Four_NegativeOnes={-1,-1,-1,-1};

const fltx4 Four_2ToThe21s={ (float) (1<<21), (float) (1<<21), (float) (1<<21), (float)(1<<21) };
const fltx4 Four_2ToThe22s={ (float) (1<<22), (float) (1<<22), (float) (1<<22), (float)(1<<22) };
const fltx4 Four_2ToThe23s={ (float) (1<<23), (float) (1<<23), (float) (1<<23), (float)(1<<23) };
const fltx4 Four_2ToThe24s={ (float) (1<<24), (float) (1<<24), (float) (1<<24), (float)(1<<24) };

const fltx4 Four_Point225s={ .225, .225, .225, .225 };
const fltx4 Four_Epsilons={FLT_EPSILON,FLT_EPSILON,FLT_EPSILON,FLT_EPSILON};

const fltx4 Four_FLT_MAX={FLT_MAX,FLT_MAX,FLT_MAX,FLT_MAX};
const fltx4 Four_Negative_FLT_MAX={-FLT_MAX,-FLT_MAX,-FLT_MAX,-FLT_MAX};
const fltx4 g_SIMD_0123 = { 0., 1., 2., 3. };

const fltx4 g_QuatMultRowSign[4] =
{
	{  1.0f,  1.0f, -1.0f, 1.0f },
	{ -1.0f,  1.0f,  1.0f, 1.0f },
	{  1.0f, -1.0f,  1.0f, 1.0f },
	{ -1.0f, -1.0f, -1.0f, 1.0f }
};

const uint32 ALIGN16 g_SIMD_clear_signmask[4] ALIGN16_POST = {0x7fffffff,0x7fffffff,0x7fffffff,0x7fffffff};
const uint32 ALIGN16 g_SIMD_signmask[4] ALIGN16_POST = { 0x80000000, 0x80000000, 0x80000000, 0x80000000 };
const uint32 ALIGN16 g_SIMD_lsbmask[4] ALIGN16_POST = { 0xfffffffe, 0xfffffffe, 0xfffffffe, 0xfffffffe };
const uint32 ALIGN16 g_SIMD_clear_wmask[4] ALIGN16_POST = { 0xffffffff, 0xffffffff, 0xffffffff, 0 };
const uint32 ALIGN16 g_SIMD_AllOnesMask[4] ALIGN16_POST = { 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff }; // ~0,~0,~0,~0
const uint32 ALIGN16 g_SIMD_Low16BitsMask[4] ALIGN16_POST = { 0xffff, 0xffff, 0xffff, 0xffff }; // 0xffff x 4

const uint32 ALIGN16 g_SIMD_ComponentMask[4][4] ALIGN16_POST =
{
	{ 0xFFFFFFFF, 0, 0, 0 }, { 0, 0xFFFFFFFF, 0, 0 }, { 0, 0, 0xFFFFFFFF, 0 }, { 0, 0, 0, 0xFFFFFFFF }
};

const uint32 ALIGN16 g_SIMD_SkipTailMask[4][4] ALIGN16_POST =
{
	{ 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff },
	{ 0xffffffff, 0x00000000, 0x00000000, 0x00000000 },
	{ 0xffffffff, 0xffffffff, 0x00000000, 0x00000000 },
	{ 0xffffffff, 0xffffffff, 0xffffffff, 0x00000000 },
};
