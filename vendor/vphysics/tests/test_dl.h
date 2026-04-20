// Platform-agnostic dlopen/dlsym for test harnesses. Keeps the per-test
// noise down and lets the same test source compile on POSIX and Windows.

#pragma once

#if defined(_WIN32)
	#ifndef WIN32_LEAN_AND_MEAN
	#  define WIN32_LEAN_AND_MEAN
	#endif
	#ifndef NOMINMAX
	#  define NOMINMAX
	#endif
	#include <windows.h>

	static inline void *TestDlOpen( const char *path )
	{
		return reinterpret_cast<void *>( LoadLibraryA( path ) );
	}
	static inline void *TestDlSym( void *handle, const char *name )
	{
		return reinterpret_cast<void *>(
			GetProcAddress( reinterpret_cast<HMODULE>( handle ), name ) );
	}
	static inline const char *TestDlError() { return "LoadLibraryA/GetProcAddress failed"; }
#else
	#include <dlfcn.h>

	static inline void *TestDlOpen( const char *path ) { return dlopen( path, RTLD_NOW ); }
	static inline void *TestDlSym( void *h, const char *name ) { return dlsym( h, name ); }
	static inline const char *TestDlError() { return dlerror(); }
#endif
