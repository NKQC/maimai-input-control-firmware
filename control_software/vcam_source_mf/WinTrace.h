#pragma once

HRESULT GetTraceId(GUID* pGuid);

ULONG WinTraceRegister();
void WinTraceUnregister();

void WinTrace(UCHAR Level, ULONGLONG Keyword, PCWSTR String);
void WinTraceFormat(UCHAR Level, ULONGLONG Keyword, PCWSTR pszFormat, ...);

void WinTrace(UCHAR Level, ULONGLONG Keyword, PCSTR String);
void WinTraceFormat(UCHAR Level, ULONGLONG Keyword, PCSTR pszFormat, ...);

#define WINTRACE(...) WinTraceFormat(0, 0, __VA_ARGS__)
#pragma once
