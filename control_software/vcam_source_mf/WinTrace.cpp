#include "pch.h"
#include "Tools.h"

static GUID GUID_WinTraceProvider = { 0x3f2a91c6,0x5d47,0x4be1,{0x9c,0x02,0x7a,0xd3,0x5e,0x18,0xb6,0x40} };

REGHANDLE _traceHandle = 0;

// 日志落在媒体源 DLL 的安装目录(%ProgramData%\mai2control), 不写死盘符; 目录不存在时
// CreateFile 直接失败, 由调用方静默忽略 —— Frame Server 里没有日志比崩掉重要。
static PCWSTR TraceFile()
{
	static WCHAR path[MAX_PATH]{};
	if (!path[0])
	{
		WCHAR root[MAX_PATH]{};
		if (!GetEnvironmentVariableW(L"ProgramData", root, _countof(root)))
		{
			StringCchCopyW(root, _countof(root), L"C:\\ProgramData");
		}
		StringCchPrintfW(path, _countof(path), L"%s\\mai2control\\mai2vcam_source.log", root);
	}
	return path;
}

static void WriteTraceLine(PCWSTR string) noexcept
{
	SYSTEMTIME time{};
	GetLocalTime(&time);

	WCHAR image[MAX_PATH]{};
	GetModuleFileNameW(nullptr, image, _countof(image));
	PCWSTR process = image;
	for (PCWSTR current = image; *current; ++current)
	{
		if (*current == L'\\' || *current == L'/')
			process = current + 1;
	}

	WCHAR line[4096]{};
	StringCchPrintfW(line, _countof(line),
		L"%04u-%02u-%02u %02u:%02u:%02u.%03u pid=%lu process=%s %s\r\n",
		time.wYear, time.wMonth, time.wDay, time.wHour, time.wMinute,
		time.wSecond, time.wMilliseconds, GetCurrentProcessId(), process,
		string ? string : L"");

	CHAR utf8[8192]{};
	auto bytes = WideCharToMultiByte(CP_UTF8, 0, line, -1, utf8, _countof(utf8), nullptr, nullptr);
	if (bytes <= 1)
		return;

	auto file = CreateFileW(TraceFile(), FILE_APPEND_DATA,
		FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
		nullptr, OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
	if (file == INVALID_HANDLE_VALUE)
		return;

	DWORD written = 0;
	WriteFile(file, utf8, static_cast<DWORD>(bytes - 1), &written, nullptr);
	FlushFileBuffers(file);
	CloseHandle(file);
}

HRESULT GetTraceId(GUID* pGuid)
{
	if (!pGuid)
		return E_INVALIDARG;

	*pGuid = GUID_WinTraceProvider;
	return S_OK;
}

ULONG WinTraceRegister()
{
	return EventRegister(&GUID_WinTraceProvider, nullptr, nullptr, &_traceHandle);
}

void WinTraceUnregister()
{
	auto h = _traceHandle;
	if (h)
	{
		_traceHandle = 0;
		EventUnregister(h);
	}
}

void WinTraceFormat(UCHAR level, ULONGLONG keyword, PCWSTR format, ...)
{
	WCHAR trace[2048]{};
	va_list args;
	va_start(args, format);
	StringCchPrintfW(trace, 10, L"%08X:", GetCurrentThreadId());
	StringCchVPrintfW(trace + 9, _countof(trace) - 9, format, args);
	va_end(args);
	WinTrace(level, keyword, trace);
}

void WinTraceFormat(UCHAR level, ULONGLONG keyword, PCSTR format, ...)
{
	CHAR trace[2048]{};
	va_list args;
	va_start(args, format);
	StringCchPrintfA(trace, 10, "%08X:", GetCurrentThreadId());
	StringCchVPrintfA(trace + 9, _countof(trace) - 9, format, args);
	va_end(args);
	WinTrace(level, keyword, trace);
}

void WinTrace(UCHAR level, ULONGLONG keyword, PCSTR string)
{
	auto wide = to_wstring(string ? string : "");
	WinTrace(level, keyword, wide.c_str());
}

void WinTrace(UCHAR level, ULONGLONG keyword, PCWSTR string)
{
	WriteTraceLine(string);
	if (_traceHandle)
		EventWriteString(_traceHandle, level, keyword, string ? string : L"");
}
