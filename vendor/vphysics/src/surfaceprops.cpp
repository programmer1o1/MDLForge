#include "surfaceprops.h"

#include <cctype>
#include <cstdlib>
#include <cstring>

namespace mdlforge_vphysics
{

namespace
{

// ---- Minimal KV1 tokenizer ---------------------------------------------
// Produces successive tokens: either a quoted string, or a brace `{`/`}`,
// or an unquoted identifier. Comments (`// …\n` and `/* … */`) are skipped,
// as is all whitespace. Returns empty string on EOF.
class KV1Lexer
{
public:
	explicit KV1Lexer( const char *p ) : m_cur( p ) {}

	bool  eof()   const { return *m_cur == '\0'; }

	// Next token. Sets isBrace true for `{` / `}` (token = "{" or "}"). Empty
	// token + eof means end-of-input.
	std::string Next( bool *isBrace = nullptr )
	{
		SkipWhitespaceAndComments();
		if ( isBrace ) *isBrace = false;
		if ( *m_cur == '\0' )
			return {};
		if ( *m_cur == '{' || *m_cur == '}' )
		{
			std::string t( 1, *m_cur );
			++m_cur;
			if ( isBrace ) *isBrace = true;
			return t;
		}
		if ( *m_cur == '"' )
		{
			++m_cur;
			const char *start = m_cur;
			while ( *m_cur && *m_cur != '"' && *m_cur != '\n' )
				++m_cur;
			std::string t( start, m_cur );
			if ( *m_cur == '"' )
				++m_cur;
			return t;
		}
		// Unquoted identifier — read until whitespace or delimiter.
		const char *start = m_cur;
		while ( *m_cur && !std::isspace( static_cast<unsigned char>( *m_cur ) ) && *m_cur != '{' && *m_cur != '}' && *m_cur != '"' )
			++m_cur;
		return std::string( start, m_cur );
	}

private:
	void SkipWhitespaceAndComments()
	{
		while ( *m_cur )
		{
			if ( std::isspace( static_cast<unsigned char>( *m_cur ) ) )
			{
				++m_cur;
				continue;
			}
			if ( m_cur[0] == '/' && m_cur[1] == '/' )
			{
				while ( *m_cur && *m_cur != '\n' )
					++m_cur;
				continue;
			}
			if ( m_cur[0] == '/' && m_cur[1] == '*' )
			{
				m_cur += 2;
				while ( *m_cur && !( m_cur[0] == '*' && m_cur[1] == '/' ) )
					++m_cur;
				if ( *m_cur )
					m_cur += 2;
				continue;
			}
			break;
		}
	}

	const char *m_cur;
};

int CaseInsensitiveCompare( const std::string &a, const char *b )
{
	size_t i = 0;
	while ( i < a.size() && b[i] )
	{
		const int ca = std::tolower( static_cast<unsigned char>( a[i] ) );
		const int cb = std::tolower( static_cast<unsigned char>( b[i] ) );
		if ( ca != cb ) return ca - cb;
		++i;
	}
	if ( i < a.size() ) return 1;
	if ( b[i] != '\0' ) return -1;
	return 0;
}

float ParseFloat( const std::string &s )
{
	return static_cast<float>( std::strtod( s.c_str(), nullptr ) );
}

} // namespace

int SurfacePropsTable::Parse( const char *pText )
{
	if ( pText == nullptr )
		return 0;

	KV1Lexer lex( pText );
	int added = 0;

	while ( !lex.eof() )
	{
		bool brace = false;
		std::string name = lex.Next( &brace );
		if ( name.empty() )
			break;
		if ( brace )
			continue; // stray brace

		// Expect opening `{`.
		std::string open = lex.Next( &brace );
		if ( !brace || open != "{" )
			continue; // malformed — skip

		// Parse key/value pairs until `}`.
		SurfacePropEntry entry;
		entry.name = name;

		while ( !lex.eof() )
		{
			std::string key = lex.Next( &brace );
			if ( brace && key == "}" )
				break;
			if ( key.empty() )
				break;
			std::string value = lex.Next( &brace );
			if ( brace )
				break; // unexpected brace mid-pair

			if ( CaseInsensitiveCompare( key, "base" ) == 0 )
				entry.base = value;
			else if ( CaseInsensitiveCompare( key, "density" ) == 0 )
				{ entry.density = ParseFloat( value );    entry.hasDensity = true; }
			else if ( CaseInsensitiveCompare( key, "thickness" ) == 0 )
				{ entry.thickness = ParseFloat( value );  entry.hasThickness = true; }
			else if ( CaseInsensitiveCompare( key, "friction" ) == 0 )
				{ entry.friction = ParseFloat( value );   entry.hasFriction = true; }
			else if ( CaseInsensitiveCompare( key, "elasticity" ) == 0 )
				{ entry.elasticity = ParseFloat( value ); entry.hasElasticity = true; }
			// Other keys (audioreflectivity, jumpfactor, dampening, gamematerial, …)
			// are silently ignored — studiomdl never reads them.
		}

		auto it = m_nameToIndex.find( entry.name );
		if ( it == m_nameToIndex.end() )
		{
			int idx = static_cast<int>( m_entries.size() );
			m_nameToIndex[entry.name] = idx;
			m_entries.push_back( std::move( entry ) );
			++added;
		}
		else
		{
			m_entries[it->second] = std::move( entry );
		}
	}
	return added;
}

int SurfacePropsTable::IndexOf( const char *name ) const
{
	if ( name == nullptr )
		return -1;
	auto it = m_nameToIndex.find( name );
	return ( it == m_nameToIndex.end() ) ? -1 : it->second;
}

const SurfacePropEntry *SurfacePropsTable::At( int index ) const
{
	if ( index < 0 || index >= static_cast<int>( m_entries.size() ) )
		return nullptr;
	return &m_entries[index];
}

bool SurfacePropsTable::GetEffective( int index, float *density, float *thickness, float *friction, float *elasticity ) const
{
	const SurfacePropEntry *e = At( index );
	if ( e == nullptr )
		return false;

	// Walk base chain (max 8 hops — defensive against cycles) to fill missing fields.
	bool haveD = e->hasDensity,    haveT = e->hasThickness;
	bool haveF = e->hasFriction,   haveE = e->hasElasticity;
	float d = e->density, t = e->thickness, f = e->friction, el = e->elasticity;

	int depth = 0;
	const SurfacePropEntry *cur = e;
	while ( cur && ( !haveD || !haveT || !haveF || !haveE ) && depth++ < 8 )
	{
		if ( cur->base.empty() )
			break;
		int bi = IndexOf( cur->base.c_str() );
		if ( bi < 0 )
			break;
		cur = At( bi );
		if ( cur == nullptr )
			break;
		if ( !haveD && cur->hasDensity )    { d  = cur->density;    haveD = true; }
		if ( !haveT && cur->hasThickness )  { t  = cur->thickness;  haveT = true; }
		if ( !haveF && cur->hasFriction )   { f  = cur->friction;   haveF = true; }
		if ( !haveE && cur->hasElasticity ) { el = cur->elasticity; haveE = true; }
	}

	if ( density )    *density    = d;
	if ( thickness )  *thickness  = t;
	if ( friction )   *friction   = f;
	if ( elasticity ) *elasticity = el;
	return true;
}

} // namespace mdlforge_vphysics
