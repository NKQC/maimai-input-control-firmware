// 进程内 COM 服务器入口 + 自注册。
//
// 注册做两件事(两者缺一, 摄像头就不会出现在应用的设备列表里):
//   1. HKCR\CLSID\{clsid}\InprocServer32 → 本 DLL 路径, ThreadingModel=Both;
//   2. IFilterMapper2::RegisterFilter 把过滤器登记到 CLSID_VideoInputDeviceCategory,
//      即"视频输入设备"类别, 消费端通过 ICreateDevEnum 枚举该类别得到设备矩。
// 32 位与 64 位 DLL 各自注册到各自的注册表视图: 64 位应用只看 64 位视图,
// 32 位应用只看 WOW6432Node 视图, 所以两个位宽都要装。

#include "vcam_filter.h"

#ifndef MERIT_DO_NOT_USE
#define MERIT_DO_NOT_USE 0x200000
#endif

static HRESULT _ClsidText(WCHAR* buffer, int capacity) {
    return StringFromGUID2(CLSID_Mai2VcamDshow, buffer, capacity) > 0 ? S_OK : E_FAIL;
}

static HRESULT _WriteString(HKEY key, const WCHAR* name, const WCHAR* value) {
    const DWORD bytes = (DWORD)((wcslen(value) + 1) * sizeof(WCHAR));
    LSTATUS status = RegSetValueExW(key, name, 0, REG_SZ, (const BYTE*)value, bytes);
    return status == ERROR_SUCCESS ? S_OK : HRESULT_FROM_WIN32(status);
}

static HRESULT _RegisterClsid() {
    WCHAR clsid[64] = {0};
    HRESULT hr = _ClsidText(clsid, 64);
    if (FAILED(hr)) {
        return hr;
    }
    WCHAR module[MAX_PATH] = {0};
    if (GetModuleFileNameW(g_mai2vcamModule, module, MAX_PATH) == 0) {
        return HRESULT_FROM_WIN32(GetLastError());
    }
    WCHAR path[128] = {0};
    swprintf_s(path, L"CLSID\\%s", clsid);

    HKEY key = nullptr;
    LSTATUS status = RegCreateKeyExW(HKEY_CLASSES_ROOT, path, 0, nullptr, 0, KEY_WRITE, nullptr,
                                     &key, nullptr);
    if (status != ERROR_SUCCESS) {
        return HRESULT_FROM_WIN32(status);
    }
    hr = _WriteString(key, nullptr, MAI2VCAM_FRIENDLY_NAME);
    HKEY inproc = nullptr;
    if (SUCCEEDED(hr)) {
        status = RegCreateKeyExW(key, L"InprocServer32", 0, nullptr, 0, KEY_WRITE, nullptr, &inproc,
                                 nullptr);
        hr = status == ERROR_SUCCESS ? S_OK : HRESULT_FROM_WIN32(status);
    }
    if (SUCCEEDED(hr)) {
        hr = _WriteString(inproc, nullptr, module);
    }
    if (SUCCEEDED(hr)) {
        hr = _WriteString(inproc, L"ThreadingModel", L"Both");
    }
    if (inproc != nullptr) {
        RegCloseKey(inproc);
    }
    RegCloseKey(key);
    return hr;
}

static void _UnregisterClsid() {
    WCHAR clsid[64] = {0};
    if (FAILED(_ClsidText(clsid, 64))) {
        return;
    }
    WCHAR path[128] = {0};
    swprintf_s(path, L"CLSID\\%s", clsid);
    // ★整棵删, 不要逐个删子键★
    // RegDeleteKeyW 在目标键下仍有子键时直接失败。这里至少有 InprocServer32, 旧实现是"先删它,
    // 再删父键" —— 只要将来有任何别的组件往这个 CLSID 下加过一个子键(TypeLib / ProgID 之类),
    // 父键就再也删不掉, 而返回值一直没人看, 于是留下"DLL 已删、CLSID 键还在"的残壳。
    // 消费端枚举的是类别登记项, 那个残壳会让摄像头继续出现在设备列表里, 且此时 DLL 已经没了,
    // 再也没法靠 regsvr32 /u 补救。RegDeleteTreeW 连子键一起删, 从根上消掉这条失败路径。
    RegDeleteTreeW(HKEY_CLASSES_ROOT, path);
}

// 过滤器在类别里的登记信息: 单个输出针脚, 只报 NV12。
static HRESULT _MapperRegister(bool add) {
    IFilterMapper2* mapper = nullptr;
    HRESULT hr = CoCreateInstance(CLSID_FilterMapper2, nullptr, CLSCTX_INPROC_SERVER,
                                  IID_IFilterMapper2, (void**)&mapper);
    if (FAILED(hr) || mapper == nullptr) {
        return FAILED(hr) ? hr : E_NOINTERFACE;
    }
    if (add) {
        REGPINTYPES types = {};
        types.clsMajorType = &MEDIATYPE_Video;
        types.clsMinorType = &MEDIASUBTYPE_NV12;

        REGFILTERPINS pin = {};
        pin.strName = const_cast<LPWSTR>(MAI2VCAM_PIN_NAME);
        pin.bRendered = FALSE;
        pin.bOutput = TRUE;
        pin.bZero = FALSE;
        pin.bMany = FALSE;
        pin.clsConnectsToFilter = nullptr;
        pin.strConnectsToPin = nullptr;
        pin.nMediaTypes = 1;
        pin.lpMediaType = &types;

        REGFILTER2 filter = {};
        filter.dwVersion = 1;
        // 捕获源统一用 MERIT_DO_NOT_USE: 只应被显式枚举选中, 不参与智能连接的自动插入。
        filter.dwMerit = MERIT_DO_NOT_USE;
        filter.cPins = 1;
        filter.rgPins = &pin;

        hr = mapper->RegisterFilter(CLSID_Mai2VcamDshow, MAI2VCAM_FRIENDLY_NAME, nullptr,
                                    &CLSID_VideoInputDeviceCategory, MAI2VCAM_FRIENDLY_NAME,
                                    &filter);
    } else {
        hr = mapper->UnregisterFilter(&CLSID_VideoInputDeviceCategory, MAI2VCAM_FRIENDLY_NAME,
                                      CLSID_Mai2VcamDshow);
    }
    mapper->Release();
    return hr;
}

STDAPI DllGetClassObject(REFCLSID clsid, REFIID riid, void** object) {
    if (object == nullptr) {
        return E_POINTER;
    }
    *object = nullptr;
    if (clsid != CLSID_Mai2VcamDshow) {
        return CLASS_E_CLASSNOTAVAILABLE;
    }
    Mai2VcamClassFactory* factory = new Mai2VcamClassFactory();
    if (factory == nullptr) {
        return E_OUTOFMEMORY;
    }
    HRESULT hr = factory->QueryInterface(riid, object);
    factory->Release();
    return hr;
}

STDAPI DllCanUnloadNow() { return Mai2VcamModuleLocks() == 0 ? S_OK : S_FALSE; }

STDAPI DllRegisterServer() {
    HRESULT init = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);
    HRESULT hr = _RegisterClsid();
    if (SUCCEEDED(hr)) {
        hr = _MapperRegister(true);
        if (FAILED(hr)) {
            // 类别登记失败时不留下"半装"状态。
            _UnregisterClsid();
        }
    }
    Mai2VcamLog("DllRegisterServer hr=0x%08X", hr);
    if (SUCCEEDED(init)) {
        CoUninitialize();
    }
    return hr;
}

STDAPI DllUnregisterServer() {
    HRESULT init = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);
    HRESULT hr = _MapperRegister(false);
    _UnregisterClsid();
    Mai2VcamLog("DllUnregisterServer hr=0x%08X", hr);
    if (SUCCEEDED(init)) {
        CoUninitialize();
    }
    // 类别项本来就不存在时不算失败: 卸载必须幂等。
    return SUCCEEDED(hr) || hr == HRESULT_FROM_WIN32(ERROR_FILE_NOT_FOUND) ? S_OK : hr;
}

BOOL WINAPI DllMain(HINSTANCE module, DWORD reason, LPVOID) {
    if (reason == DLL_PROCESS_ATTACH) {
        g_mai2vcamModule = module;
        DisableThreadLibraryCalls(module);
    }
    return TRUE;
}
