#ifndef IAPPSYSTEM_H
#define IAPPSYSTEM_H

#if defined(_WIN32)
#pragma once
#endif

#include "tier1/interface.h"

struct AppSystemInfo_t;

enum InitReturnVal_t
{
	INIT_FAILED = 0,
	INIT_OK,
	INIT_LAST_VAL,
};

enum AppSystemTier_t
{
	APP_SYSTEM_TIER0 = 0,
	APP_SYSTEM_TIER1,
	APP_SYSTEM_TIER2,
	APP_SYSTEM_TIER3,
	APP_SYSTEM_TIER_OTHER,
};

abstract_class IAppSystem
{
public:
	virtual bool Connect( CreateInterfaceFn factory ) = 0;
	virtual void Disconnect() = 0;

	// Reconnect to a particular interface (for debugging)
	virtual void *QueryInterface( const char *pInterfaceName ) = 0;

	// Init, shutdown
	virtual InitReturnVal_t Init() = 0;
	virtual void Shutdown() = 0;

	// Returns all dependent libraries
	virtual const AppSystemInfo_t *GetDependencies() { return nullptr; }

	// Returns the tier
	virtual AppSystemTier_t GetTier() { return APP_SYSTEM_TIER_OTHER; }

	// Reconnect to a particular interface
	virtual void Reconnect( CreateInterfaceFn factory, const char *pInterfaceName )
	{
		Disconnect();
		Connect( factory );
	}

	// Returns whether or not the app system is a singleton
	virtual bool IsSingleton() { return true; }
};

template <class IInterface>
class CTier0AppSystem : public IAppSystem, public IInterface
{
public:
	explicit CTier0AppSystem( bool bIsPrimaryAppSystem = true )
		: m_bIsPrimaryAppSystem( bIsPrimaryAppSystem )
	{
	}

	bool Connect( CreateInterfaceFn factory ) override { return true; }
	void Disconnect() override {}

	void *QueryInterface( const char *pInterfaceName ) override
	{
		return nullptr;
	}

	InitReturnVal_t Init() override { return INIT_OK; }
	void Shutdown() override {}

	AppSystemTier_t GetTier() override { return APP_SYSTEM_TIER0; }
	bool IsSingleton() override { return !m_bIsPrimaryAppSystem; }
	bool IsPrimaryAppSystem() const { return m_bIsPrimaryAppSystem; }

protected:
	bool m_bIsPrimaryAppSystem;
};

#endif // IAPPSYSTEM_H
