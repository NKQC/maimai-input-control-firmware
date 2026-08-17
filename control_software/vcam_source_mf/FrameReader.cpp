#include "pch.h"
#include "Undocumented.h"
#include "Tools.h"
#include "EnumNames.h"
#include "MFTools.h"
#include "FrameReader.h"

static constexpr PCWSTR SHARE_NAMES[] =
{
	L"Global\\mai2control_vcam_frame",
	L"Local\\mai2control_vcam_frame",
};

HRESULT FrameReader::Start(UINT width, UINT height)
{
	// NV12 的 2x2 色度块要求偶数分辨率。
	RETURN_HR_IF(E_INVALIDARG, !width || !height || (width & 1) || (height & 1));

	if (_width != width || _height != height)
	{
		_Close();
		_width = width;
		_height = height;
		try
		{
			_pixels.assign((size_t)width * height * 3, 0);
		}
		catch (...)
		{
			_width = 0;
			_height = 0;
			RETURN_HR(E_OUTOFMEMORY);
		}
	}

	_frame = 0;
	_retry = 0;
	// 不假定像素缓冲是干净的: 置 false 让首次读取失败时真的去刷黑帧。
	_black = false;
	WINTRACE(L"FrameReader::Start %u x %u", width, height);
	return S_OK;
}

// 只读打开共享映射: 先 Global(上位机以管理员运行时的正常情况), 再退 Local。
bool FrameReader::_Open()
{
	auto needed = sizeof(_Header) + (size_t)_width * _height * 3;
	for (auto name : SHARE_NAMES)
	{
		auto map = OpenFileMappingW(FILE_MAP_READ, FALSE, name);
		if (!map)
			continue;

		auto view = reinterpret_cast<const BYTE*>(MapViewOfFile(map, FILE_MAP_READ, 0, 0, 0));
		MEMORY_BASIC_INFORMATION info{};
		if (!view || !VirtualQuery(view, &info, sizeof(info)) || info.RegionSize < needed)
		{
			WINTRACE(L"FrameReader::_Open '%s' too small (region:%zu needed:%zu)", name, view ? info.RegionSize : 0, needed);
			if (view)
			{
				UnmapViewOfFile(view);
			}
			CloseHandle(map);
			continue;
		}

		_map = map;
		_view = view;
		_valid = false;
		WINTRACE(L"FrameReader::_Open '%s' mapped %zu bytes", name, info.RegionSize);
		return true;
	}
	return false;
}

void FrameReader::_Close()
{
	if (_view)
	{
		UnmapViewOfFile(_view);
		_view = nullptr;
	}
	if (_map)
	{
		CloseHandle(_map);
		_map = nullptr;
	}
	_valid = false;
}

// 返回 false = 本帧没有可用像素(未映射/头非法), 由调用方转黑帧。
bool FrameReader::_Capture()
{
	if (!_view)
	{
		if (_retry)
		{
			_retry--;
			return false;
		}
		_retry = _kOpenRetryFrames;
		if (!_Open())
			return false;
	}

	auto header = reinterpret_cast<const _Header*>(_view);
	if (header->magic != _kMagic || header->version != _kVersion ||
		header->width != _width || header->height != _height || header->format != _kFormatRgb24)
	{
		if (_valid || !_frame)
		{
			WINTRACE(L"FrameReader::_Capture invalid header magic:0x%08X version:%u %u x %u format:%u",
				header->magic, header->version, header->width, header->height, header->format);
		}
		_valid = false;
		return false;
	}

	if (!_valid)
	{
		_valid = true;
		WINTRACE(L"FrameReader::_Capture header ok, sequence:%u", _Sequence(header));
	}

	// sequence 是生产者"像素已写完"的提交标志: 拷贝前后一致才说明这帧没被写穿。
	// 连续撕裂说明生产者正在高频刷新, 直接用拷到的内容出帧, 不闪黑。
	auto source = _view + sizeof(_Header);
	for (UINT i = 0; i < _kTearRetries; i++)
	{
		auto before = _Sequence(header);
		CopyMemory(_pixels.data(), source, _pixels.size());
		if (before == _Sequence(header))
			break;
	}
	return true;
}

void FrameReader::_Blacken()
{
	if (_black)
		return;

	ZeroMemory(_pixels.data(), _pixels.size());
	_black = true;
}

HRESULT FrameReader::Generate(IMFSample* sample, REFGUID format, IMFSample** outSample)
{
	RETURN_HR_IF_NULL(E_POINTER, sample);
	RETURN_HR_IF_NULL(E_POINTER, outSample);
	*outSample = nullptr;
	RETURN_HR_IF(MF_E_NOT_INITIALIZED, _pixels.empty());

	if (_Capture())
	{
		_black = false;
	}
	else
	{
		_Blacken();
	}

	wil::com_ptr_nothrow<IMFMediaBuffer> mediaBuffer;
	RETURN_IF_FAILED(sample->GetBufferByIndex(0, &mediaBuffer));

	HRESULT hr;
	wil::com_ptr_nothrow<IMF2DBuffer2> buffer2D;
	if (SUCCEEDED(mediaBuffer->QueryInterface(IID_PPV_ARGS(&buffer2D))))
	{
		BYTE* scanline;
		LONG pitch;
		BYTE* start;
		DWORD length;
		RETURN_IF_FAILED(buffer2D->Lock2DSize(MF2DBuffer_LockFlags_Write, &scanline, &pitch, &start, &length));
		hr = _Write(scanline, pitch, length, format);
		buffer2D->Unlock2D();
	}
	else
	{
		// allocator 给的是线性缓冲时按媒体类型的默认 stride 自己排布。
		auto nv12 = format == MFVideoFormat_NV12;
		BYTE* data;
		DWORD max = 0;
		DWORD current = 0;
		RETURN_IF_FAILED(mediaBuffer->Lock(&data, &max, &current));
		hr = _Write(data, (LONG)(nv12 ? _width : _width * 4), max, format);
		if (SUCCEEDED(hr))
		{
			hr = mediaBuffer->SetCurrentLength(nv12 ? _width * _height * 3 / 2 : _width * _height * 4);
		}
		mediaBuffer->Unlock();
	}
	RETURN_IF_FAILED(hr);

	_frame++;
	sample->AddRef();
	*outSample = sample;
	return S_OK;
}

HRESULT FrameReader::_Write(BYTE* scanline, LONG pitch, DWORD length, REFGUID format) const
{
	RETURN_HR_IF_NULL(E_POINTER, scanline);
	if (format == MFVideoFormat_NV12)
		return _WriteNv12(scanline, pitch, length);

	return _WriteRgb32(scanline, pitch, length);
}

// MFVideoFormat_RGB32 = BGRX。pitch 为负时代表自底向上缓冲, 由 scanline0 + y*pitch 自然处理。
HRESULT FrameReader::_WriteRgb32(BYTE* scanline, LONG pitch, DWORD length) const
{
	auto stride = (ULONG)(pitch < 0 ? -pitch : pitch);
	RETURN_HR_IF(E_UNEXPECTED, stride < _width * 4);
	RETURN_HR_IF(E_UNEXPECTED, length < stride * _height);

	auto source = _pixels.data();
	for (UINT y = 0; y < _height; y++)
	{
		auto dst = scanline + (LONG)y * pitch;
		for (UINT x = 0; x < _width; x++)
		{
			dst[0] = source[2];
			dst[1] = source[1];
			dst[2] = source[0];
			dst[3] = 0xFF;
			dst += 4;
			source += 3;
		}
	}
	return S_OK;
}

// NV12: Y 平面 + 交错 UV 平面, 两个平面共用同一 pitch, UV 起点为 scanline + pitch*height。
HRESULT FrameReader::_WriteNv12(BYTE* scanline, LONG pitch, DWORD length) const
{
	RETURN_HR_IF(E_UNEXPECTED, pitch <= 0 || (ULONG)pitch < _width);
	RETURN_HR_IF(E_UNEXPECTED, length < (ULONG)pitch * _height * 3 / 2);

	auto chroma = scanline + (size_t)pitch * _height;
	for (UINT y = 0; y < _height; y += 2)
	{
		auto top = _pixels.data() + (size_t)y * _width * 3;
		auto bottom = top + (size_t)_width * 3;
		auto lumaTop = scanline + (size_t)y * pitch;
		auto lumaBottom = lumaTop + pitch;
		auto uv = chroma + (size_t)(y / 2) * pitch;
		for (UINT x = 0; x < _width; x += 2)
		{
			lumaTop[0] = _Luma(top);
			lumaTop[1] = _Luma(top + 3);
			lumaBottom[0] = _Luma(bottom);
			lumaBottom[1] = _Luma(bottom + 3);
			_Chroma((top[0] + top[3] + bottom[0] + bottom[3]) / 4,
				(top[1] + top[4] + bottom[1] + bottom[4]) / 4,
				(top[2] + top[5] + bottom[2] + bottom[5]) / 4, uv);
			top += 6;
			bottom += 6;
			lumaTop += 2;
			lumaBottom += 2;
			uv += 2;
		}
	}
	return S_OK;
}
