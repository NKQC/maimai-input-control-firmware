#pragma once

// 上位机(control_software/src/vcam/share.rs)发布的 RGB24 帧读取器, 取代参考实现里
// 用 D2D/D3D 画测试图的 FrameGenerator。
//
// 协议(必须与 share.rs 完全一致): 32 字节头 + width*height*3 像素(源像素序 R,G,B)。
// 头部 u32 小端: magic 0x4D324356 / version 1 / width / height / format 1(RGB24) /
// sequence(生产者写完像素后以 release 语义发布) / 2 个保留字。
// 命名空间依次尝试 Global\mai2control_vcam_frame, 然后 Local\mai2control_vcam_frame。
class FrameReader
{
public:
	FrameReader() = default;
	~FrameReader()
	{
		_Close();
	}

	HRESULT Start(UINT width, UINT height);
	HRESULT Generate(IMFSample* sample, REFGUID format, IMFSample** outSample);

private:
	struct _Header
	{
		ULONG magic;
		ULONG version;
		ULONG width;
		ULONG height;
		ULONG format;
		ULONG sequence;
		ULONG reserved[2];
	};

	static constexpr ULONG _kMagic = 0x4D324356;
	static constexpr ULONG _kVersion = 1;
	static constexpr ULONG _kFormatRgb24 = 1;
	static constexpr UINT _kOpenRetryFrames = 15;  // 约 0.5s @30fps 重试一次打开
	static constexpr UINT _kTearRetries = 3;

	bool _Open();
	void _Close();
	bool _Capture();
	void _Blacken();
	HRESULT _Write(BYTE* scanline, LONG pitch, DWORD length, REFGUID format) const;
	HRESULT _WriteRgb32(BYTE* scanline, LONG pitch, DWORD length) const;
	HRESULT _WriteNv12(BYTE* scanline, LONG pitch, DWORD length) const;

	static ULONG _Sequence(const _Header* header)
	{
		auto value = *reinterpret_cast<volatile const ULONG*>(&header->sequence);
		MemoryBarrier();
		return value;
	}

	static BYTE _Clamp(int value)
	{
		return (BYTE)(value < 0 ? 0 : (value > 255 ? 255 : value));
	}

	// BT.601 full range: 生产者画的是 0/255 纯黑白 QR, 用 full range 才不会被压成 16..235。
	static BYTE _Luma(const BYTE* rgb)
	{
		return _Clamp((77 * rgb[0] + 150 * rgb[1] + 29 * rgb[2] + 128) >> 8);
	}

	static void _Chroma(int r, int g, int b, BYTE* uv)
	{
		uv[0] = _Clamp((((-43 * r - 85 * g + 128 * b + 128) >> 8) + 128));
		uv[1] = _Clamp((((128 * r - 107 * g - 21 * b + 128) >> 8) + 128));
	}

	UINT _width = 0;
	UINT _height = 0;
	ULONGLONG _frame = 0;
	UINT _retry = 0;
	bool _black = false;
	bool _valid = false;
	HANDLE _map = nullptr;
	const BYTE* _view = nullptr;
	std::vector<BYTE> _pixels;
};
