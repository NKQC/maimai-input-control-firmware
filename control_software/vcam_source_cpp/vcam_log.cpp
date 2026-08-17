// 诊断日志的唯一实现。**两个消费端 DLL 都编译本 TU**:
//   · vcam_source_cpp/mai2vcam_dshow.vcxproj  (DirectShow 源过滤器)
//   · vcam_source_mf/mai2vcam_source.vcxproj  (Media Foundation 媒体源)
// 声明在 vcam_queue_layout.h。只依赖 Win32, 不碰 DirectShow / MF 任何类型。

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif

#include <windows.h>

#include <stdarg.h>
#include <stdio.h>

#include "vcam_queue_layout.h"

void Mai2VcamLog(const char* format, ...) {
    // 日志目录与上位机安装目录同源; 建不出来就直接放弃(消费端 DLL 绝不能因为日志失败而失败)。
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
    int header =
        _snprintf_s(line, sizeof(line), _TRUNCATE, "[%04u-%02u-%02u %02u:%02u:%02u.%03u pid=%lu] ",
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
