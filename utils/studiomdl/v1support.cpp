//========= Copyright � 1996-2005, Valve Corporation, All rights reserved. ============//
//
// Purpose: 
//
// $NoKeywords: $
//
//=============================================================================//

//
// studiomdl.c: generates a studio .mdl file from a .qc script
// sources/<scriptname>.mdl.
//


#pragma warning( disable : 4244 )
#pragma warning( disable : 4237 )
#pragma warning( disable : 4305 )


#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <math.h>

#include "cmdlib.h"
#include "scriplib.h"
#include "mathlib.h"
#include "studio.h"
#include "studiomdl.h"


// Exact-match lookup (material, position, texcoord) through a hash table instead of a linear scan
// over every vertex so far; among the candidates the lowest index whose normal is within
// normal_blend wins, exactly like the original loop. Makes big SMDs (60k+ triangles) load in
// seconds instead of minutes.
#include <unordered_map>
#include <vector>
#include <cstring>

static std::unordered_map<uint64_t, std::vector<int> > s_vlistHash;
static int s_vlistHashCount = 0;

static uint64_t VlistKey( int material, const Vector &v, const Vector2D &t )
{
	uint32_t w[6];
	memcpy( &w[0], &v.x, 4 ); memcpy( &w[1], &v.y, 4 ); memcpy( &w[2], &v.z, 4 );
	memcpy( &w[3], &t.x, 4 ); memcpy( &w[4], &t.y, 4 ); w[5] = (uint32_t)material;
	for ( int k = 0; k < 6; k++ ) if ( w[k] == 0x80000000u ) w[k] = 0;	// -0.0 == 0.0
	uint64_t h = 1469598103934665603ull;
	for ( int k = 0; k < 6; k++ ) { h ^= w[k]; h *= 1099511628211ull; }
	return h;
}

int lookup_index( s_source_t *psource, int material, Vector& vertex, Vector& normal, Vector2D texcoord )
{
	int i;

	if ( numvlist == 0 || numvlist < s_vlistHashCount )
	{
		s_vlistHash.clear();				// new source file
		s_vlistHashCount = 0;
	}
	for ( ; s_vlistHashCount < numvlist; s_vlistHashCount++ )	// entries added elsewhere
	{
		int j = s_vlistHashCount;
		s_vlistHash[ VlistKey( v_listdata[j].m, g_vertex[j], g_texcoord[j] ) ].push_back( j );
	}

	uint64_t key = VlistKey( material, vertex, texcoord );
	std::unordered_map<uint64_t, std::vector<int> >::iterator it = s_vlistHash.find( key );
	if ( it != s_vlistHash.end() )
	{
		const std::vector<int> &cand = it->second;
		for ( size_t c = 0; c < cand.size(); c++ )
		{
			i = cand[c];
			if (v_listdata[i].m == material
				&& DotProduct( g_normal[i], normal ) > normal_blend
				&& VectorCompare( g_vertex[i], vertex )
				&& g_texcoord[i][0] == texcoord[0]
				&& g_texcoord[i][1] == texcoord[1])
			{
				v_listdata[i].lastref = numvlist;
				return i;
			}
		}
	}

	i = numvlist;
	if (i >= MAXSTUDIOVERTS) {
		MdlError( "too many indices in source: \"%s\"\n", psource->filename);
	}

	VectorCopy( vertex, g_vertex[i] );
	VectorCopy( normal, g_normal[i] );
	Vector2Copy( texcoord, g_texcoord[i] );

	v_listdata[i].v = i;
	v_listdata[i].m = material;
	v_listdata[i].n = i;
	v_listdata[i].t = i;

	v_listdata[i].firstref = numvlist;
	v_listdata[i].lastref = numvlist;

	numvlist = i + 1;
	s_vlistHash[ key ].push_back( i );
	s_vlistHashCount = numvlist;
	return i;
}


void ParseFaceData( s_source_t *psource, int material, s_face_t *pFace )
{
	int index[3];
	int i, j;
	Vector p;
	Vector normal;
	Vector2D t;
	int		iCount, bones[MAXSTUDIOSRCBONES];
	float   weights[MAXSTUDIOSRCBONES];
	int bone;

	for (j = 0; j < 3; j++)
	{
		memset( g_szLine, 0, sizeof( g_szLine ) );

		if (fgets( g_szLine, sizeof( g_szLine ), g_fpInput ) == NULL)
		{
			MdlError(
				"%s: unexpected EOF at line %d while reading vertex %d/3 of a triangle.\n"
				"  This usually means the 'triangles' section is missing an 'end' marker,\n"
				"  or the SMD was written with non-LF line endings that confused the parser.\n",
				g_szFilename, g_iLinecount, j + 1 );
		}

		iCount = 0;

		g_iLinecount++;
		i = sscanf( g_szLine, "%d %f %f %f %f %f %f %f %f %d %d %f %d %f %d %f %d %f",
			&bone, 
			&p[0], &p[1], &p[2], 
			&normal[0], &normal[1], &normal[2], 
			&t[0], &t[1],
			&iCount,
			&bones[0], &weights[0], &bones[1], &weights[1], &bones[2], &weights[2], &bones[3], &weights[3] );
			
		if (i < 9) 
			continue;

		if (bone < 0 || bone >= psource->numbones) 
		{
			MdlError("bogus bone index\n%d %s :\n%s", g_iLinecount, g_szFilename, g_szLine );
		}

		//Scale face pos
		scale_vertex( p );
		
		// continue parsing more bones.
		// FIXME: don't we have a built in parser that'll do this?
		if (iCount > 4)
		{
			int k;
			int ctr = 0;
			char *token;
			for (k = 0; k < 18; k++)
			{
				while (g_szLine[ctr] == ' ')
				{
					ctr++;
				}
				token = strtok( &g_szLine[ctr], " " );
				ctr += strlen( token ) + 1;
			}
			for (k = 4; k < iCount && k < MAXSTUDIOSRCBONES; k++)
			{
				while (g_szLine[ctr] == ' ')
				{
					ctr++;
				}
				token = strtok( &g_szLine[ctr], " " );
				ctr += strlen( token ) + 1;

				bones[k] = atoi(token);

				token = strtok( &g_szLine[ctr], " " );
				ctr += strlen( token ) + 1;
			
				weights[k] = atof(token);
			}
			// printf("%d ", iCount );

			//printf("\n");
			//exit(1);
		}

		// adjust_vertex( p );
		// scale_vertex( p );

		// move vertex position to object space.
		// VectorSubtract( p, psource->bonefixup[bone].worldorg, tmp );
		// VectorTransform(tmp, psource->bonefixup[bone].im, p );

		// move normal to object space.
		// VectorCopy( normal, tmp );
		// VectorTransform(tmp, psource->bonefixup[bone].im, normal );
		// VectorNormalize( normal );

		// invert v
		t[1] = 1.0 - t[1];

		index[j] = lookup_index( psource, material, p, normal, t );

		if (i == 9 || iCount == 0)
		{
			g_bone[index[j]].numbones = 1;
			g_bone[index[j]].bone[0] = bone;
			g_bone[index[j]].weight[0] = 1.0;
		}
		else
		{
			iCount = SortAndBalanceBones( iCount, MAXSTUDIOBONEWEIGHTS, bones, weights );

			g_bone[index[j]].numbones = iCount;
			for (i = 0; i < iCount; i++)
			{
				g_bone[index[j]].bone[i] = bones[i];
				g_bone[index[j]].weight[i] = weights[i];
			}
		}
	}

	// pFace->material = material; // BUG
	pFace->a		= index[0];
	pFace->b		= index[1];
	pFace->c		= index[2];
	Assert( ((pFace->a & 0xF0000000) == 0) && ((pFace->b & 0xF0000000) == 0) && 
		((pFace->c & 0xF0000000) == 0) );

	if (flip_triangles)
	{
		j = pFace->b;  pFace->b  = pFace->c;  pFace->c  = j;
	}
}

void Grab_Triangles( s_source_t *psource )
{
	int		i;
	int		tcount = 0;	
	Vector	vmin, vmax;

	vmin[0] = vmin[1] = vmin[2] = 99999;
	vmax[0] = vmax[1] = vmax[2] = -99999;

	g_numfaces = 0;
	numvlist = 0;
 
	//
	// load the base triangles
	//
	int texture;
	int material;
	char texturename[64];

	while (1) 
	{
		if (fgets( g_szLine, sizeof( g_szLine ), g_fpInput ) == NULL) 
			break;

		g_iLinecount++;

		// check for end
		if (IsEnd( g_szLine )) 
			break;

		// Look for extra junk that we may want to avoid...
		int nLineLength = strlen( g_szLine );
		if (nLineLength >= 64)
		{
			MdlWarning("Unexpected data at line %d, (need a texture name) ignoring...\n", g_iLinecount );
			continue;
		}

		// strip off trailing smag
		strncpy( texturename, g_szLine, 63 );
		for (i = strlen( texturename ) - 1; i >= 0 && ! isgraph( texturename[i] ); i--)
		{
		}
		texturename[i + 1] = '\0';

		// funky texture overrides
		for (i = 0; i < numrep; i++)  
		{
			if (sourcetexture[i][0] == '\0') 
			{
				strcpy( texturename, defaulttexture[i] );
				break;
			}
			if (stricmp( texturename, sourcetexture[i]) == 0) 
			{
				strcpy( texturename, defaulttexture[i] );
				break;
			}
		}

		if (texturename[0] == '\0')
		{
			// weird source problem, skip them
			fgets( g_szLine, sizeof( g_szLine ), g_fpInput );
			fgets( g_szLine, sizeof( g_szLine ), g_fpInput );
			fgets( g_szLine, sizeof( g_szLine ), g_fpInput );
			g_iLinecount += 3;
			continue;
		}

		if (stricmp( texturename, "null.bmp") == 0 || stricmp( texturename, "null.tga") == 0)
		{
			// skip all faces with the null texture on them.
			fgets( g_szLine, sizeof( g_szLine ), g_fpInput );
			fgets( g_szLine, sizeof( g_szLine ), g_fpInput );
			fgets( g_szLine, sizeof( g_szLine ), g_fpInput );
			g_iLinecount += 3;
			continue;
		}

		texture = lookup_texture( texturename, sizeof( texturename ) );
		psource->texmap[texture] = texture;	// hack, make it 1:1
		material = use_texture_as_material( texture );

		if ( g_numfaces >= MAXSTUDIOTRIANGLES )
			MdlError( "%s: too many triangles (more than %d)\n", psource->filename, MAXSTUDIOTRIANGLES );

		s_face_t f;
		ParseFaceData( psource, material, &f );
	
		g_src_uface[g_numfaces] = f;
		g_face[g_numfaces].material = material;
		g_numfaces++;
	}

	BuildIndividualMeshes( psource );
}


int Load_SMD ( s_source_t *psource )
{
	char	cmd[1024];
	int		option;

	if (!OpenGlobalFile( psource->filename ))
		return 0;

	if( !g_quiet )
	{
		printf ("SMD MODEL %s\n", psource->filename);
	}

	g_iLinecount = 0;

	while (fgets( g_szLine, sizeof( g_szLine ), g_fpInput ) != NULL) 
	{
		g_iLinecount++;
		int numRead = sscanf( g_szLine, "%s %d", cmd, &option );

		// Blank line
		if ((numRead == EOF) || (numRead == 0))
			continue;

		if (stricmp( cmd, "version" ) == 0) 
		{
			if (option != 1) 
			{
				MdlError("bad version\n");
			}
		}
		else if (stricmp( cmd, "nodes" ) == 0) 
		{
			psource->numbones = Grab_Nodes( psource->localBone );
		}
		else if (stricmp( cmd, "skeleton" ) == 0) 
		{
			Grab_Animation( psource );
		}
		else if (stricmp( cmd, "triangles" ) == 0) 
		{
			Grab_Triangles( psource );
		}
		else if (stricmp( cmd, "vertexanimation" ) == 0) 
		{
			Grab_Vertexanimation( psource );
		}
		else if ((strncmp( cmd, "//", 2 ) == 0) || (strncmp( cmd, ";", 1 ) == 0) || (strncmp( cmd, "#", 1 ) == 0))
		{
			continue;
		}
		else 
		{
			MdlWarning("unknown studio command \"%s\"\n", cmd );
		}
	}
	fclose( g_fpInput );

	is_v1support = true;

	return 1;
}





















