#include "vcam_common.h"

#include <stdarg.h>

// 本 TU 是唯一定义自有 GUID 的地方(initguid.h 必须在 DEFINE_GUID 之前)。
#include <initguid.h>
DEFINE_GUID(CLSID_Mai2VcamDshow, 0x6e5a1c74, 0x2f83, 0x4c9b, 0x9d, 0x1e, 0x7a, 0x4b, 0x0f, 0x3c,
            0x58, 0xe2);

HINSTANCE g_mai2vcamModule = nullptr;

static volatile LONG g_moduleLocks = 0;

void Mai2VcamLockModule() { InterlockedIncrement(&g_moduleLocks); }
void Mai2VcamUnlockModule() { InterlockedDecrement(&g_moduleLocks); }
long Mai2VcamModuleLocks() { return InterlockedCompareExchange(&g_moduleLocks, 0, 0); }

void Mai2VcamLog(const char* format, ...) {
    // 日志目录与上位机安装目录同源; 建不出来就直接放弃(过滤器绝不能因为日志失败而失败)。
    wchar_t directory[MAX_PATH] = {0};
    DWORD written = GetEnvironmentVariableW(L"ProgramData", directory, MAX_PATH);
    if (written == 0 || written >= MAX_PATH) {
        return;
    }
    wchar_t path[MAX_PATH] = {0};
    wcsncpy_s(path, MAX_PATH, directory, _TRUNCATE);
    wcsncat_s(path, MAX_PATH, L"\\mai2control", _TRUNCATE);
    CreateDirectoryW(path, nullptr);
    wcsncat_s(path, MAX_PATH, L"\\mai2vcam_dshow.log", _TRUNCATE);

    HANDLE file = CreateFileW(path, FILE_APPEND_DATA, FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr,
                             OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (file == INVALID_HANDLE_VALUE) {
        return;
    }
    char line[1024] = {0};
    SYSTEMTIME now = {};
    GetLocalTime(&now);
    int header = _snprintf_s(line, sizeof(line), _TRUNCATE, "[%04u-%02u-%02u %02u:%02u:%02u.%03u pid=%lu] ",
                             now.wYear, now.wMonth, now.wDay, now.wHour, now.wMinute, now.wSecond,
                             now.wMilliseconds, GetCurrentProcessId());
    if (header < 0) {
        CloseHandle(file);
        return;
    }
    va_list args;
    va_start(args, format);
    _vsnprintf_s(line + header, sizeof(line) - header, _TRUNCATE, format, args);
    va_end(args);
    size_t length = strlen(line);
    if (length + 2 < sizeof(line)) {
        line[length++] = '\r';
        line[length++] = '\n';
        line[length] = '\0';
    }
    DWORD bytes = 0;
    WriteFile(file, line, (DWORD)length, &bytes, nullptr);
    CloseHandle(file);
}

HRESULT Mai2VcamCopyMediaType(AM_MEDIA_TYPE* destination, const AM_MEDIA_TYPE* source) {
    if (destination == nullptr || source == nullptr) {
        return E_POINTER;
    }
    *destination = *source;
    destination->pbFormat = nullptr;
    destination->pUnk = nullptr;
    if (source->cbFormat != 0 && source->pbFormat != nullptr) {
        destination->pbFormat = (BYTE*)CoTaskMemAlloc(source->cbFormat);
        if (destination->pbFormat == nullptr) {
            destination->cbFormat = 0;
            return E_OUTOFMEMORY;
        }
        memcpy(destination->pbFormat, source->pbFormat, source->cbFormat);
    } else {
        destination->cbFormat = 0;
    }
    if (source->pUnk != nullptr) {
        destination->pUnk = source->pUnk;
        destination->pUnk->AddRef();
    }
    return S_OK;
}

void Mai2VcamFreeMediaTypeContents(AM_MEDIA_TYPE* type) {
    if (type == nullptr) {
        return;
    }
    if (type->pbFormat != nullptr) {
        CoTaskMemFree(type->pbFormat);
        type->pbFormat = nullptr;
    }
    type->cbFormat = 0;
    if (type->pUnk != nullptr) {
        type->pUnk->Release();
        type->pUnk = nullptr;
    }
}

void Mai2VcamDeleteMediaType(AM_MEDIA_TYPE* type) {
    if (type == nullptr) {
        return;
    }
    Mai2VcamFreeMediaTypeContents(type);
    CoTaskMemFree(type);
}

HRESULT Mai2VcamBuildMediaType(AM_MEDIA_TYPE* type, LONGLONG frameInterval) {
    if (type == nullptr) {
        return E_POINTER;
    }
    if (frameInterval < MAI2VCAM_DEFAULT_INTERVAL || frameInterval > MAI2VCAM_MAX_INTERVAL) {
        frameInterval = MAI2VCAM_DEFAULT_INTERVAL;
    }
    ZeroMemory(type, sizeof(AM_MEDIA_TYPE));
    VIDEOINFOHEADER* info = (VIDEOINFOHEADER*)CoTaskMemAlloc(sizeof(VIDEOINFOHEADER));
    if (info == nullptr) {
        return E_OUTOFMEMORY;
    }
    ZeroMemory(info, sizeof(VIDEOINFOHEADER));
    info->rcSource = {0, 0, MAI2VCAM_WIDTH, MAI2VCAM_HEIGHT};
    info->rcTarget = info->rcSource;
    info->dwBitRate = (DWORD)((LONGLONG)MAI2VCAM_FRAME_BYTES * 8 * 10000000LL / frameInterval);
    info->AvgTimePerFrame = frameInterval;
    info->bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    info->bmiHeader.biWidth = MAI2VCAM_WIDTH;
    // NV12 是平面 YUV: 高度取正(自上而下), 与 RGB 的倒置约定无关。
    info->bmiHeader.biHeight = MAI2VCAM_HEIGHT;
    info->bmiHeader.biPlanes = 1;
    info->bmiHeader.biBitCount = 12;
    info->bmiHeader.biCompression = MAKEFOURCC('N', 'V', '1', '2');
    info->bmiHeader.biSizeImage = MAI2VCAM_FRAME_BYTES;

    type->majortype = MEDIATYPE_Video;
    type->subtype = MEDIASUBTYPE_NV12;
    type->bFixedSizeSamples = TRUE;
    type->bTemporalCompression = FALSE;
    type->lSampleSize = MAI2VCAM_FRAME_BYTES;
    type->formattype = FORMAT_VideoInfo;
    type->pUnk = nullptr;
    type->cbFormat = sizeof(VIDEOINFOHEADER);
    type->pbFormat = (BYTE*)info;
    return S_OK;
}

bool Mai2VcamAcceptMediaType(const AM_MEDIA_TYPE* type) {
    if (type == nullptr) {
        return false;
    }
    // 部分指定的类型(GUID_NULL)按通配处理: 图构建器常给出只填了 majortype 的类型。
    if (type->majortype != GUID_NULL && type->majortype != MEDIATYPE_Video) {
        return false;
    }
    if (type->subtype != GUID_NULL && type->subtype != MEDIASUBTYPE_NV12) {
        return false;
    }
    if (type->formattype == GUID_NULL || type->cbFormat == 0 || type->pbFormat == nullptr) {
        return true;
    }
    if (type->formattype != FORMAT_VideoInfo || type->cbFormat < sizeof(VIDEOINFOHEADER)) {
        return false;
    }
    const VIDEOINFOHEADER* info = (const VIDEOINFOHEADER*)type->pbFormat;
    if (info->bmiHeader.biWidth != MAI2VCAM_WIDTH) {
        return false;
    }
    LONG height = info->bmiHeader.biHeight < 0 ? -info->bmiHeader.biHeight : info->bmiHeader.biHeight;
    if (height != MAI2VCAM_HEIGHT) {
        return false;
    }
    if (info->bmiHeader.biCompression != MAKEFOURCC('N', 'V', '1', '2')) {
        return false;
    }
    if (info->AvgTimePerFrame != 0 &&
        (info->AvgTimePerFrame < MAI2VCAM_DEFAULT_INTERVAL ||
         info->AvgTimePerFrame > MAI2VCAM_MAX_INTERVAL)) {
        return false;
    }
    return true;
}

LONGLONG Mai2VcamIntervalOf(const AM_MEDIA_TYPE* type) {
    if (type == nullptr || type->formattype != FORMAT_VideoInfo || type->pbFormat == nullptr ||
        type->cbFormat < sizeof(VIDEOINFOHEADER)) {
        return MAI2VCAM_DEFAULT_INTERVAL;
    }
    LONGLONG interval = ((const VIDEOINFOHEADER*)type->pbFormat)->AvgTimePerFrame;
    if (interval < MAI2VCAM_DEFAULT_INTERVAL || interval > MAI2VCAM_MAX_INTERVAL) {
        return MAI2VCAM_DEFAULT_INTERVAL;
    }
    return interval;
}
