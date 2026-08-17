#pragma once

#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers

#define _CRTDBG_MAP_ALLOC
#include <cstdlib>
#include <crtdbg.h>

// Windows Header Files
#include <windows.h>
#include <evntprov.h>
#include <strsafe.h>
#include <initguid.h>
#include <propvarutil.h>
#include <mfapi.h>
#include <mfidl.h>
#include <mfvirtualcamera.h>
#include <mferror.h>
#include <mfcaptureengine.h>
#include <ks.h>
#include <ksproxy.h>
#include <ksmedia.h>
#include <uuids.h>
#include "winrt\Windows.ApplicationModel.h"

// std
#include <string>
#include <format>
#include <vector>

// WIL, requires "Microsoft.Windows.ImplementationLibrary" nuget
#include "wil/result.h"
#include "wil/stl.h"
#include "wil/win32_helpers.h"
#include "wil/com.h"

// C++/WinRT, requires "Microsoft.Windows.CppWinRT" nuget
#include "winrt/base.h"

// project globals
#include "wintrace.h"

#pragma comment(lib, "mfsensorgroup")

// 上位机 backend.rs 里的友好名与 CLSID 必须与此处一致。
#define MAI2_VCAM_FRIENDLY_NAME L"mai2control Virtual Camera"
// b7c5f1a2-3d64-4e8b-9a11-2f6c8d0e4a73
extern GUID CLSID_Mai2Vcam;
