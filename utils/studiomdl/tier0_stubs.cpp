// Minimal tier0/vstdlib stubs to let studiomdl link without external DLLs.
#include "tier0/dbg.h"
#include "tier0/icommandline.h"
#include "tier0/platform.h"
#include "tier0/vcrmode.h"
#include "color.h"
#include "vstdlib/ikeyvaluessystem.h"
#include "vstdlib/random.h"
#include "icvar.h"
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <unordered_map>
#include <string>
#include <vector>

VCR_t *g_pVCR = nullptr;

//------------------------------------------------------------------------------
// Platform stubs (tier0)
//------------------------------------------------------------------------------
bool Plat_IsInDebugSession()
{
	return false;
}

void Plat_DebugString( const char *pMsg )
{
	if ( pMsg && *pMsg )
		fputs( pMsg, stderr );
}

bool Is64BitOS()
{
#if defined( PLATFORM_64BITS )
	return true;
#else
	// When we're building 32-bit (Win32 CI), it's still useful for this to be correct-ish.
	return ( sizeof( void * ) == 8 );
#endif
}

// Spew plumbing (minimal).
static SpewOutputFunc_t g_SpewOutputFunc = nullptr;
static SpewType_t g_SpewType = SPEW_MESSAGE;
static const tchar *g_SpewGroup = "default";
static int g_SpewLevel = 0;
static Color g_SpewColor( 255, 255, 255, 255 );
static bool g_AllAssertsDisabled = false;
static AssertFailedNotifyFunc_t g_AssertFailedNotifyFunc = nullptr;

SpewRetval_t DefaultSpewFunc( SpewType_t type, const tchar *msg )
{
    (void)type;
    fputs( msg ? msg : "", stderr );
    return SPEW_CONTINUE;
}

SpewRetval_t DefaultSpewFuncAbortOnAsserts( SpewType_t type, const tchar *msg )
{
	SpewRetval_t ret = DefaultSpewFunc( type, msg );
	return ( type == SPEW_ASSERT ) ? SPEW_ABORT : ret;
}

void SpewOutputFunc( SpewOutputFunc_t func )
{
	g_SpewOutputFunc = func;
}

SpewOutputFunc_t GetSpewOutputFunc( void )
{
	return g_SpewOutputFunc ? g_SpewOutputFunc : DefaultSpewFunc;
}

const tchar *GetSpewOutputGroup( void )
{
	return g_SpewGroup;
}

int GetSpewOutputLevel( void )
{
	return g_SpewLevel;
}

const Color *GetSpewOutputColor( void )
{
	return &g_SpewColor;
}

void SpewActivate( const tchar *pGroupName, int level )
{
	g_SpewGroup = pGroupName ? pGroupName : "default";
	g_SpewLevel = level;
}

bool IsSpewActive( const tchar *, int )
{
	return true;
}

void _SpewInfo( SpewType_t type, const tchar *, int )
{
	g_SpewType = type;
}

static SpewRetval_t SpewDispatch( const tchar *msg )
{
	SpewOutputFunc_t func = GetSpewOutputFunc();
	return func ? func( g_SpewType, msg ) : SPEW_CONTINUE;
}

SpewRetval_t _SpewMessage( PRINTF_FORMAT_STRING const tchar *pMsg, ... )
{
	char buffer[4096];
	va_list args;
	va_start( args, pMsg );
	vsnprintf( buffer, sizeof( buffer ), pMsg ? pMsg : "", args );
	va_end( args );
	return SpewDispatch( buffer );
}

SpewRetval_t _DSpewMessage( const tchar *pGroupName, int level, PRINTF_FORMAT_STRING const tchar *pMsg, ... )
{
	g_SpewGroup = pGroupName ? pGroupName : g_SpewGroup;
	g_SpewLevel = level;

	char buffer[4096];
	va_list args;
	va_start( args, pMsg );
	vsnprintf( buffer, sizeof( buffer ), pMsg ? pMsg : "", args );
	va_end( args );
	return SpewDispatch( buffer );
}

SpewRetval_t ColorSpewMessage( SpewType_t type, const Color *pColor, PRINTF_FORMAT_STRING const tchar *pMsg, ... )
{
	g_SpewType = type;
	if ( pColor )
		g_SpewColor = *pColor;

	char buffer[4096];
	va_list args;
	va_start( args, pMsg );
	vsnprintf( buffer, sizeof( buffer ), pMsg ? pMsg : "", args );
	va_end( args );
	return SpewDispatch( buffer );
}

SpewRetval_t ColorSpewMessage2( SpewType_t type, const Color &color, PRINTF_FORMAT_STRING const tchar *pMsg, ... )
{
	g_SpewType = type;
	g_SpewColor = color;

	char buffer[4096];
	va_list args;
	va_start( args, pMsg );
	vsnprintf( buffer, sizeof( buffer ), pMsg ? pMsg : "", args );
	va_end( args );
	return SpewDispatch( buffer );
}

void _ExitOnFatalAssert( const tchar *pFile, int line )
{
	fprintf( stderr, "Fatal assert at %s:%d\n", pFile ? pFile : "?", line );
	abort();
}

bool ShouldUseNewAssertDialog()
{
	return true;
}

bool SetupWin32ConsoleIO()
{
	return false;
}

bool DoNewAssertDialog( const tchar *, int, const tchar * )
{
	return false;
}

bool AreAllAssertsDisabled()
{
	return g_AllAssertsDisabled;
}

void SetAllAssertsDisabled( bool bAssertsEnabled )
{
	g_AllAssertsDisabled = bAssertsEnabled;
}

void SetAssertFailedNotifyFunc( AssertFailedNotifyFunc_t func )
{
	g_AssertFailedNotifyFunc = func;
}

void CallAssertFailedNotifyFunc( const char *pchFile, int nLine, const char *pchMessage )
{
	if ( g_AssertFailedNotifyFunc )
		g_AssertFailedNotifyFunc( pchFile, nLine, pchMessage );
}

#if !defined( _WIN32 ) && !defined( WIN32 )
void InstallSpewFunction()
{
	setvbuf( stdout, NULL, _IONBF, 0 );
	setvbuf( stderr, NULL, _IONBF, 0 );
	SpewOutputFunc( DefaultSpewFunc );
}
#endif

void Error( PRINTF_FORMAT_STRING const char *pMsg, ... )
{
    va_list args; va_start( args, pMsg );
    vfprintf( stderr, pMsg, args );
    va_end( args );
    exit( 1 );
}

void Msg( PRINTF_FORMAT_STRING const tchar *pMsg, ... )
{
    va_list args; va_start( args, pMsg );
    vfprintf( stdout, pMsg, args );
    va_end( args );
}

void DevMsg( int, PRINTF_FORMAT_STRING const tchar *pMsg, ... )
{
    va_list args; va_start( args, pMsg );
    vfprintf( stdout, pMsg, args );
    va_end( args );
}

void DevMsg( PRINTF_FORMAT_STRING const tchar *pMsg, ... )
{
    va_list args; va_start( args, pMsg );
    vfprintf( stdout, pMsg, args );
    va_end( args );
}

void Warning( PRINTF_FORMAT_STRING const tchar *pMsg, ... )
{
    va_list args; va_start( args, pMsg );
    vfprintf( stderr, pMsg, args );
    va_end( args );
}

void DevWarning( int, PRINTF_FORMAT_STRING const tchar *pMsg, ... )
{
    va_list args; va_start( args, pMsg );
    vfprintf( stderr, pMsg, args );
    va_end( args );
}

void DevWarning( PRINTF_FORMAT_STRING const tchar *pMsg, ... )
{
    va_list args; va_start( args, pMsg );
    vfprintf( stderr, pMsg, args );
    va_end( args );
}

void AssertMsgImplementation( bool, const char *pMsg, ... )
{
    va_list args; va_start( args, pMsg );
    vfprintf( stderr, pMsg, args );
    va_end( args );
}

void AssertMsgOnceImplementation( bool b, const char *pMsg, ... )
{
    if ( !b ) return;
    va_list args; va_start( args, pMsg );
    vfprintf( stderr, pMsg, args );
    va_end( args );
}

void AssertFailed( const char *pFileName, int nLine, const char *pMsg )
{
    fprintf( stderr, "Assert failed %s:%d: %s\n", pFileName, nLine, pMsg ? pMsg : "" );
}

class ICommandLineNull : public ICommandLine
{
public:
    void CreateCmdLine( const char *cmdline ) override
    {
        m_cmdLineString = cmdline ? cmdline : "";
        ParseString();
    }

    void CreateCmdLine( int argc, char **argv ) override
    {
        m_args.clear();
        m_cmdLineString.clear();
        for ( int i = 0; i < argc; ++i )
        {
            if ( argv[i] )
            {
                m_args.emplace_back( argv[i] );
                if ( !m_cmdLineString.empty() )
                    m_cmdLineString.push_back( ' ' );
                m_cmdLineString += argv[i];
            }
        }
    }

    const char *GetCmdLine( void ) const override { return m_cmdLineString.c_str(); }

    const char *CheckParm( const char *parm, const char **ppValue ) const override
    {
        int idx = FindParm( parm );
        if ( idx <= 0 )
        {
            if ( ppValue ) *ppValue = nullptr;
            return nullptr;
        }
        if ( ppValue )
        {
            if ( idx < static_cast<int>( m_args.size() ) )
            {
                const std::string &maybe = m_args[idx];
                if ( maybe.empty() || maybe[0] == '-' || maybe[0] == '+' )
                    *ppValue = nullptr;
                else
                    *ppValue = maybe.c_str();
            }
            else
            {
                *ppValue = nullptr;
            }
        }
        return m_args[idx - 1].c_str();
    }

    void RemoveParm( const char *parm ) override
    {
        for ( auto it = m_args.begin(); it != m_args.end(); )
        {
            if ( _stricmp( it->c_str(), parm ) == 0 )
                it = m_args.erase( it );
            else
                ++it;
        }
        RebuildCmdLine();
    }

    void AppendParm( const char *pszParm, const char *pszValues ) override
    {
        if ( pszParm && *pszParm )
            m_args.emplace_back( pszParm );
        if ( pszValues && *pszValues )
            m_args.emplace_back( pszValues );
        RebuildCmdLine();
    }

    const char *ParmValue( const char *psz, const char *pDefaultVal = nullptr ) const override
    {
        const char *val = nullptr;
        CheckParm( psz, &val );
        return val ? val : pDefaultVal;
    }

    int ParmValue( const char *psz, int nDefaultVal ) const override
    {
        const char *val = ParmValue( psz, static_cast<const char *>( nullptr ) );
        return val ? atoi( val ) : nDefaultVal;
    }

    float ParmValue( const char *psz, float flDefaultVal ) const override
    {
        const char *val = ParmValue( psz, static_cast<const char *>( nullptr ) );
        return val ? static_cast<float>( atof( val ) ) : flDefaultVal;
    }

    int ParmCount() const override { return static_cast<int>( m_args.size() ); }

    int FindParm( const char *psz ) const override
    {
        if ( !psz )
            return 0;
        for ( size_t i = 0; i < m_args.size(); ++i )
        {
            if ( _stricmp( m_args[i].c_str(), psz ) == 0 )
                return static_cast<int>( i + 1 ); // 1-based to match Valve semantics
        }
        return 0;
    }

    const char *GetParm( int nIndex ) const override
    {
        if ( nIndex < 0 || nIndex >= static_cast<int>( m_args.size() ) )
            return "";
        return m_args[nIndex].c_str();
    }

private:
    void ParseString()
    {
        m_args.clear();
        std::string current;
        bool inQuotes = false;
        for ( size_t i = 0; i < m_cmdLineString.size(); ++i )
        {
            char c = m_cmdLineString[i];
            if ( c == '\"' )
            {
                inQuotes = !inQuotes;
            }
            else if ( isspace( static_cast<unsigned char>( c ) ) && !inQuotes )
            {
                if ( !current.empty() )
                {
                    m_args.push_back( current );
                    current.clear();
                }
            }
            else
            {
                current.push_back( c );
            }
        }
        if ( !current.empty() )
            m_args.push_back( current );
    }

    void RebuildCmdLine()
    {
        m_cmdLineString.clear();
        for ( size_t i = 0; i < m_args.size(); ++i )
        {
            if ( i ) m_cmdLineString.push_back( ' ' );
            m_cmdLineString += m_args[i];
        }
    }

    std::vector<std::string> m_args;
    std::string m_cmdLineString;
};

static ICommandLineNull g_CommandLineNull;
ICommandLine *CommandLine_Tier0() { return &g_CommandLineNull; }

bool HushAsserts() { return true; }
void HushAsserts( bool ) {}

static int RandomInt_Impl( int minVal, int maxVal )
{
    if ( maxVal < minVal )
        return minVal;
    if ( maxVal == minVal )
        return minVal;
    return minVal + ( rand() % ( ( maxVal - minVal ) + 1 ) );
}

#if defined( _WIN32 )
static int __cdecl RandomInt_Stub( int minVal, int maxVal )
{
    return RandomInt_Impl( minVal, maxVal );
}
extern "C" __declspec(selectany) int( __cdecl* _imp__RandomInt )( int, int ) = RandomInt_Stub;
#else
extern "C" int RandomInt( int minVal, int maxVal )
{
    return RandomInt_Impl( minVal, maxVal );
}
#endif

extern "C" ICvar *GetCVarIF() { return nullptr; }

const CPUInformation *GetCPUInformation()
{
    static CPUInformation info = {};
    return &info;
}

class KeyValuesSystemStub : public IKeyValuesSystem
{
public:
    void RegisterSizeofKeyValues( int ) override {}
    void *AllocKeyValuesMemory( int size ) override { return malloc( size ); }
    void FreeKeyValuesMemory( void *pMem ) override { free( pMem ); }

    HKeySymbol GetSymbolForString( const char *name ) override
    {
        if ( !name ) return INVALID_KEY_SYMBOL;
        auto it = m_symbolToId.find( name );
        if ( it != m_symbolToId.end() )
            return it->second;
        HKeySymbol id = static_cast<HKeySymbol>( m_symbols.size() );
        m_symbols.emplace_back( name );
        m_symbolToId.emplace( m_symbols.back(), id );
        return id;
    }

    const char *GetStringForSymbol( HKeySymbol symbol ) override
    {
        if ( symbol < 0 || symbol >= static_cast<HKeySymbol>( m_symbols.size() ) )
            return "";
        return m_symbols[symbol].c_str();
    }

    void AddKeyValuesToMemoryLeakList( void *, HKeySymbol ) override {}
    void RemoveKeyValuesFromMemoryLeakList( void * ) override {}

private:
    std::vector<std::string> m_symbols;
    std::unordered_map<std::string, HKeySymbol> m_symbolToId;
};

static KeyValuesSystemStub g_KeyValuesSystemStub;

#if defined( _WIN32 )
static IKeyValuesSystem *__cdecl KeyValuesSystem_Stub()
{
    return &g_KeyValuesSystemStub;
}
extern "C" __declspec(selectany) IKeyValuesSystem *( __cdecl* _imp__KeyValuesSystem )() = KeyValuesSystem_Stub;
#else
extern "C" IKeyValuesSystem *KeyValuesSystem()
{
    return &g_KeyValuesSystemStub;
}
#endif
