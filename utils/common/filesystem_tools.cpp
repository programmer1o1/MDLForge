//=======================================================================
// Minimal filesystem glue for standalone studiomdl
//=======================================================================
#ifdef _WIN32
#include <windows.h>
#include <direct.h>
#include <io.h> // _chmod
#else
#include <unistd.h>
#endif

#include <stdio.h>
#include <sys/stat.h>
#include <string>

#include "tier1/utlbuffer.h"
#include "vstdlib/strtools.h"
#include "filesystem_tools.h"
#include "vstdlib/icommandline.h"
#include "tier1/keyvalues.h"
#include "tier2/tier2.h"

// memdbgon must be the last include file in a .cpp file!!!
#include <tier0/memdbgon.h>

char qdir[1024];
char gamedir[1024];

static void GetExecutableDir( char *out, int outSize )
{
    if ( !out || outSize <= 0 )
        return;
#ifdef _WIN32
    char exePath[MAX_PATH];
    DWORD len = GetModuleFileNameA( NULL, exePath, sizeof( exePath ) );
    if ( len > 0 && len < sizeof( exePath ) )
    {
        Q_StripFilename( exePath );
        Q_FixSlashes( exePath );
        Q_strncpy( out, exePath, outSize );
        return;
    }
#endif
    out[0] = '\0';
}

class CSimpleFileSystem : public IBaseFileSystem
{
public:
    CSimpleFileSystem()
    {
        m_szGameDir[0] = '\0';
    }

    void SetGameDir( const char *path )
    {
        if ( path )
        {
            Q_strncpy( m_szGameDir, path, sizeof( m_szGameDir ) );
            Q_FixSlashes( m_szGameDir );
            Q_StripTrailingSlash( m_szGameDir );
            Q_AppendSlash( m_szGameDir, sizeof( m_szGameDir ) );
        }
        else
        {
            m_szGameDir[0] = '\0';
        }
    }

    // IBaseFileSystem implementation
    int Read( void *pOutput, int size, FileHandle_t file ) override
    {
        FILE *fp = reinterpret_cast< FILE * >( file );
        if ( !fp )
            return 0;
        return static_cast<int>( fread( pOutput, 1, size, fp ) );
    }

    int Write( void const *pInput, int size, FileHandle_t file ) override
    {
        FILE *fp = reinterpret_cast< FILE * >( file );
        if ( !fp )
            return 0;
        return static_cast<int>( fwrite( pInput, 1, size, fp ) );
    }

    FileHandle_t Open( const char *pFileName, const char *pOptions, const char *pathID = 0 ) override
    {
        std::string path = ResolvePath( pFileName, pathID );
        if ( path.empty() )
            return NULL;
        return reinterpret_cast< FileHandle_t >( fopen( path.c_str(), pOptions ) );
    }

    void Close( FileHandle_t file ) override
    {
        FILE *fp = reinterpret_cast< FILE * >( file );
        if ( fp )
            fclose( fp );
    }

    void Seek( FileHandle_t file, int pos, FileSystemSeek_t seekType ) override
    {
        FILE *fp = reinterpret_cast< FILE * >( file );
        if ( !fp )
            return;
        int base = ( seekType == FILESYSTEM_SEEK_HEAD ) ? SEEK_SET : ( seekType == FILESYSTEM_SEEK_CURRENT ? SEEK_CUR : SEEK_END );
        fseek( fp, pos, base );
    }

    unsigned int Tell( FileHandle_t file ) override
    {
        FILE *fp = reinterpret_cast< FILE * >( file );
        if ( !fp )
            return 0;
        long pos = ftell( fp );
        return pos < 0 ? 0u : static_cast<unsigned int>( pos );
    }

    unsigned int Size( FileHandle_t file ) override
    {
        FILE *fp = reinterpret_cast< FILE * >( file );
        if ( !fp )
            return 0;
        long cur = ftell( fp );
        fseek( fp, 0, SEEK_END );
        long len = ftell( fp );
        fseek( fp, cur, SEEK_SET );
        return len < 0 ? 0u : static_cast<unsigned int>( len );
    }

    unsigned int Size( const char *pFileName, const char *pPathID = 0 ) override
    {
        FileHandle_t fh = Open( pFileName, "rb", pPathID );
        if ( !fh )
            return 0;
        unsigned int len = Size( fh );
        Close( fh );
        return len;
    }

    void Flush( FileHandle_t file ) override
    {
        FILE *fp = reinterpret_cast< FILE * >( file );
        if ( fp )
            fflush( fp );
    }

    bool Precache( const char *pFileName, const char *pPathID = 0 ) override
    {
        return FileExists( pFileName, pPathID );
    }

    bool FileExists( const char *pFileName, const char *pPathID = 0 ) override
    {
        std::string path = ResolvePath( pFileName, pPathID );
        if ( path.empty() )
            return false;
#ifdef _WIN32
        return _access( path.c_str(), 0 ) == 0;
#else
        return access( path.c_str(), F_OK ) == 0;
#endif
    }

    bool IsFileWritable( char const *pFileName, const char *pPathID = 0 ) override
    {
        std::string path = ResolvePath( pFileName, pPathID );
        if ( path.empty() )
            return false;
#ifdef _WIN32
        return _access( path.c_str(), 2 ) == 0;
#else
        return access( path.c_str(), W_OK ) == 0;
#endif
    }

    bool SetFileWritable( char const *pFileName, bool writable, const char *pPathID = 0 ) override
    {
        std::string path = ResolvePath( pFileName, pPathID );
        if ( path.empty() )
            return false;
#ifdef _WIN32
        int mode = writable ? _S_IREAD | _S_IWRITE : _S_IREAD;
        return _chmod( path.c_str(), mode ) == 0;
#else
        return chmod( path.c_str(), writable ? 0666 : 0444 ) == 0;
#endif
    }

    long GetFileTime( const char *pFileName, const char *pPathID = 0 ) override
    {
        std::string path = ResolvePath( pFileName, pPathID );
        if ( path.empty() )
            return -1;
#ifdef _WIN32
        struct _stat buf;
        if ( _stat( path.c_str(), &buf ) == -1 )
            return -1;
        return buf.st_mtime;
#else
        struct stat buf;
        if ( stat( path.c_str(), &buf ) == -1 )
            return -1;
        return buf.st_mtime;
#endif
    }

    bool ReadFile( const char *pFileName, const char *pPath, CUtlBuffer &buf, int nMaxBytes = 0, int nStartingByte = 0, FSAllocFunc_t pfnAlloc = NULL ) override
    {
        FileHandle_t fh = Open( pFileName, "rb", pPath );
        if ( !fh )
            return false;

        unsigned int len = Size( fh );
        if ( nStartingByte > 0 && nStartingByte < static_cast<int>( len ) )
        {
            Seek( fh, nStartingByte, FILESYSTEM_SEEK_HEAD );
            len -= nStartingByte;
        }
        if ( nMaxBytes > 0 && len > static_cast<unsigned int>( nMaxBytes ) )
            len = nMaxBytes;

        std::string temp;
        temp.resize( len );
        int read = Read( temp.data(), static_cast<int>( len ), fh );
        Close( fh );
        if ( read <= 0 )
            return false;
        buf.Clear();
        buf.Put( temp.data(), read );
        return true;
    }

    bool WriteFile( const char *pFileName, const char *pPath, CUtlBuffer &buf ) override
    {
        FileHandle_t fh = Open( pFileName, "wb", pPath );
        if ( !fh )
            return false;
        int bytes = buf.TellPut();
        bool ok = Write( buf.Base(), bytes, fh ) == bytes;
        Close( fh );
        return ok;
    }

private:
    std::string ResolvePath( const char *pFileName, const char *pPathID ) const
    {
        if ( !pFileName || !*pFileName )
            return std::string();

        if ( Q_IsAbsolutePath( pFileName ) )
            return std::string( pFileName );

        char base[1024];
        base[0] = '\0';
        if ( pPathID && ( !Q_stricmp( pPathID, "GAME" ) || !Q_stricmp( pPathID, "MOD" ) ) )
        {
            Q_strncpy( base, m_szGameDir, sizeof( base ) );
        }
        else if ( pPathID && !Q_stricmp( pPathID, "EXECUTABLE_PATH" ) )
        {
            GetExecutableDir( base, sizeof( base ) );
        }

        if ( !base[0] )
        {
#ifdef _WIN32
            _getcwd( base, sizeof( base ) );
#else
            getcwd( base, sizeof( base ) );
#endif
        }

        size_t len = strlen( base );
        if ( len > 0 && base[len - 1] != '/' && base[len - 1] != '\\' )
            Q_strncat( base, "/", sizeof( base ), COPY_ALL_CHARACTERS );

        char buffer[1024];
        Q_snprintf( buffer, sizeof( buffer ), "%s%s", base, pFileName );
        Q_FixSlashes( buffer );
        return std::string( buffer );
    }

    char m_szGameDir[MAX_PATH];
};

static CSimpleFileSystem g_SimpleFileSystem;
IBaseFileSystem *g_pFileSystem = NULL;
IBaseFileSystem *g_pFullFileSystem = NULL;
CSysModule *g_pFullFileSystemModule = NULL;

static bool HasGameInfo( const char *dir )
{
    char test[MAX_PATH];
    Q_snprintf( test, sizeof( test ), "%s%sgameinfo.txt", dir, dir[0] && (dir[strlen(dir)-1] != '/' && dir[strlen(dir)-1] != '\\') ? "/" : "" );
#ifdef _WIN32
    if ( _access( test, 0 ) == 0 )
        return true;
    Q_snprintf( test, sizeof( test ), "%s%sgameinfo.gi", dir, dir[0] && (dir[strlen(dir)-1] != '/' && dir[strlen(dir)-1] != '\\') ? "/" : "" );
    return _access( test, 0 ) == 0;
#else
    if ( access( test, F_OK ) == 0 )
        return true;
    Q_snprintf( test, sizeof( test ), "%s%sgameinfo.gi", dir, dir[0] && (dir[strlen(dir)-1] != '/' && dir[strlen(dir)-1] != '\\') ? "/" : "" );
    return access( test, F_OK ) == 0;
#endif
}

static bool FindGameDirFromFile( const char *pFilename, char *out, int outLen )
{
    char probe[MAX_PATH];
    Q_MakeAbsolutePath( probe, sizeof( probe ), pFilename ? pFilename : ".", NULL );
    Q_StripFilename( probe );
    Q_FixSlashes( probe );

    while ( probe[0] )
    {
        if ( HasGameInfo( probe ) )
        {
            Q_strncpy( out, probe, outLen );
            Q_StripTrailingSlash( out );
            Q_AppendSlash( out, outLen );
            return true;
        }
        if ( !Q_StripLastDir( probe, sizeof( probe ) ) )
            break;
    }
    return false;
}

static void SetQDirFromFilename( const char *pFilename )
{
    if ( !pFilename )
        pFilename = ".";
    Q_MakeAbsolutePath( qdir, sizeof( qdir ), pFilename, NULL );
    Q_StripFilename( qdir );
    Q_FixSlashes( qdir );
    Q_StripTrailingSlash( qdir );
    Q_AppendSlash( qdir, sizeof( qdir ) );
}

bool FileSystem_Init( const char *pFilename, int maxMemoryUsage, FSInitType_t initType, bool bOnlyUseFilename )
{
    (void)maxMemoryUsage;
    (void)initType;
    (void)bOnlyUseFilename;

    SetQDirFromFilename( pFilename );

    const char *pGameParam = CommandLine()->ParmValue( "-game", static_cast<const char*>( nullptr ) );
    if ( pGameParam )
    {
        Q_MakeAbsolutePath( gamedir, sizeof( gamedir ), pGameParam );
    }
    else if ( const char *vproj = getenv( "VPROJECT" ) )
    {
        Q_MakeAbsolutePath( gamedir, sizeof( gamedir ), vproj );
    }
    else if ( !FindGameDirFromFile( pFilename, gamedir, sizeof( gamedir ) ) )
    {
        // Default to the directory containing the source file if nothing else.
        Q_strncpy( gamedir, qdir, sizeof( gamedir ) );
    }

    Q_FixSlashes( gamedir );
    Q_StripTrailingSlash( gamedir );
    Q_AppendSlash( gamedir, sizeof( gamedir ) );

    g_SimpleFileSystem.SetGameDir( gamedir );
    g_pFileSystem = g_pFullFileSystem = &g_SimpleFileSystem;
    return g_pFileSystem != NULL;
}

void FileSystem_Term()
{
    g_pFileSystem = g_pFullFileSystem = NULL;
}

bool FileSystem_SetGame( const char *szModDir )
{
    if ( !szModDir )
        return false;
    Q_MakeAbsolutePath( gamedir, sizeof( gamedir ), szModDir );
    Q_FixSlashes( gamedir );
    Q_StripTrailingSlash( gamedir );
    Q_AppendSlash( gamedir, sizeof( gamedir ) );
    g_SimpleFileSystem.SetGameDir( gamedir );
    return true;
}

static void *SimpleFileSystemFactory( const char *pName, int *pReturnCode )
{
    if ( pReturnCode ) *pReturnCode = IFACE_OK;
    if ( !Q_stricmp( pName, BASEFILESYSTEM_INTERFACE_VERSION ) )
        return g_pFileSystem;
    return NULL;
}

CreateInterfaceFn FileSystem_GetFactory( void )
{
    return SimpleFileSystemFactory;
}
