//=======================================================================
// Material system bootstrap (stubbed for standalone studiomdl)
//=======================================================================
#include "platform_shim.h"
#include "materialsystem/imaterialsystem.h"
#include "materialsystem/materialsystem_config.h"
#include <cmdlib.h>
#include "tier0/dbg.h"
#include "filesystem.h"
#include "cmdlib.h"
#include "tier2/tier2.h"

extern void MdlError( char const *pMsg, ... );

IMaterialSystem *g_pMaterialSystem = NULL;
CreateInterfaceFn g_MatSysFactory = NULL;
CreateInterfaceFn g_ShaderAPIFactory = NULL;

// Standalone build: we skip loading the legacy material system DLLs entirely.
void InitMaterialSystem( const char *materialBaseDirPath )
{
        (void)materialBaseDirPath;
        g_pMaterialSystem = NULL;
        g_MatSysFactory = NULL;
        g_ShaderAPIFactory = NULL;
}
