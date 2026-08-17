#include "pch.h"
#include "Undocumented.h"
#include "Tools.h"
#include "EnumNames.h"
#include "MFTools.h"
#include "FrameReader.h"
#include "MediaStream.h"
#include "MediaSource.h"
#include "Activator.h"

// b7c5f1a2-3d64-4e8b-9a11-2f6c8d0e4a73: 必须与 control_software/src/vcam/backend.rs 的 _CLSID 一致。
GUID CLSID_Mai2Vcam = { 0xb7c5f1a2,0x3d64,0x4e8b,{0x9a,0x11,0x2f,0x6c,0x8d,0x0e,0x4a,0x73} };
HMODULE _hModule;

BOOL APIENTRY DllMain(HMODULE hModule, DWORD dwReason, LPVOID lpReserved)
{
	switch (dwReason)
	{
	case DLL_PROCESS_ATTACH:
		_hModule = hModule;
		WinTraceRegister();
		WINTRACE(L"DllMain DLL_PROCESS_ATTACH '%s'", GetCommandLine());
		DisableThreadLibraryCalls(hModule);

		wil::SetResultLoggingCallback([](wil::FailureInfo const& failure) noexcept
			{
				wchar_t str[2048];
				if (SUCCEEDED(wil::GetFailureLogString(str, _countof(str), failure)))
				{
					WinTrace(2, 0, str); // 2 => error
				}
			});
		break;

	case DLL_PROCESS_DETACH:
		WINTRACE(L"DllMain DLL_PROCESS_DETACH '%s'", GetCommandLine());
		WinTraceUnregister();
		break;
	}
	return TRUE;
}

struct ClassFactory : winrt::implements<ClassFactory, IClassFactory>
{
	STDMETHODIMP CreateInstance(IUnknown* outer, GUID const& riid, void** result) noexcept final
	{
		RETURN_HR_IF_NULL(E_POINTER, result);
		*result = nullptr;
		if (outer)
			RETURN_HR(CLASS_E_NOAGGREGATION);

		auto vcam = winrt::make_self<Activator>();
		RETURN_IF_FAILED(vcam->Initialize());
		auto hr = vcam->QueryInterface(riid, result);
		if (FAILED(hr))
		{
			auto iid = GUID_ToStringW(riid);
			WINTRACE(L"ClassFactory QueryInterface failed on IID %s", iid.c_str());
		}
		return hr;
	}

	STDMETHODIMP LockServer(BOOL) noexcept final
	{
		return S_OK;
	}
};

__control_entrypoint(DllExport)
STDAPI DllCanUnloadNow()
{
	if (winrt::get_module_lock())
	{
		WINTRACE(L"DllCanUnloadNow S_FALSE");
		return S_FALSE;
	}

	winrt::clear_factory_cache();
	WINTRACE(L"DllCanUnloadNow S_OK");
	return S_OK;
}

_Check_return_
STDAPI DllGetClassObject(_In_ REFCLSID rclsid, _In_ REFIID riid, _Outptr_ LPVOID FAR* ppv)
{
	WINTRACE(L"DllGetClassObject rclsid:%s riid:%s", GUID_ToStringW(rclsid).c_str(), GUID_ToStringW(riid).c_str());
	RETURN_HR_IF_NULL(E_POINTER, ppv);
	*ppv = nullptr;

	if (rclsid == CLSID_Mai2Vcam)
		return winrt::make_self<ClassFactory>()->QueryInterface(riid, ppv);

	RETURN_HR(E_NOINTERFACE);
}

using registry_key = winrt::handle_type<registry_traits>;

STDAPI DllRegisterServer()
{
	std::wstring exePath = wil::GetModuleFileNameW(_hModule).get();
	WINTRACE(L"DllRegisterServer '%s'", exePath.c_str());
	auto clsid = GUID_ToStringW(CLSID_Mai2Vcam, false);
	std::wstring path = L"Software\\Classes\\CLSID\\" + clsid + L"\\InprocServer32";

	// note: a vcam *must* be registered in HKEY_LOCAL_MACHINE
	// for the frame server to be able to talk with it.
	registry_key key;
	RETURN_IF_WIN32_ERROR(RegWriteKey(HKEY_LOCAL_MACHINE, path.c_str(), key.put()));
	RETURN_IF_WIN32_ERROR(RegWriteValue(key.get(), nullptr, exePath));
	RETURN_IF_WIN32_ERROR(RegWriteValue(key.get(), L"ThreadingModel", L"Both"));
	return S_OK;
}

STDAPI DllUnregisterServer()
{
	std::wstring exePath = wil::GetModuleFileNameW(_hModule).get();
	WINTRACE(L"DllUnregisterServer '%s'", exePath.c_str());
	auto clsid = GUID_ToStringW(CLSID_Mai2Vcam, false);
	std::wstring path = L"Software\\Classes\\CLSID\\" + clsid;
	RETURN_IF_WIN32_ERROR(RegDeleteTree(HKEY_LOCAL_MACHINE, path.c_str()));
	return S_OK;
}
