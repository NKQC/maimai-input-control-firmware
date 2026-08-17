#include "vcam_filter.h"

// ── 枚举器 ──────────────────────────────────────────────────────────────────────

namespace {

class PinEnumerator : public IEnumPins {
public:
    PinEnumerator(IPin* pin, ULONG index) : _pin(pin), _index(index) {
        _pin->AddRef();
        Mai2VcamLockModule();
    }
    ~PinEnumerator() {
        _pin->Release();
        Mai2VcamUnlockModule();
    }

    STDMETHODIMP QueryInterface(REFIID riid, void** object) override {
        if (object == nullptr) {
            return E_POINTER;
        }
        if (riid == IID_IUnknown || riid == IID_IEnumPins) {
            *object = static_cast<IEnumPins*>(this);
            AddRef();
            return S_OK;
        }
        *object = nullptr;
        return E_NOINTERFACE;
    }
    STDMETHODIMP_(ULONG) AddRef() override { return (ULONG)InterlockedIncrement(&_references); }
    STDMETHODIMP_(ULONG) Release() override {
        LONG value = InterlockedDecrement(&_references);
        if (value == 0) {
            delete this;
        }
        return (ULONG)value;
    }

    STDMETHODIMP Next(ULONG count, IPin** pins, ULONG* fetched) override {
        if (pins == nullptr) {
            return E_POINTER;
        }
        if (count > 1 && fetched == nullptr) {
            return E_INVALIDARG;
        }
        ULONG taken = 0;
        if (count > 0 && _index == 0) {
            _pin->AddRef();
            pins[0] = _pin;
            taken = 1;
            _index = 1;
        }
        if (fetched != nullptr) {
            *fetched = taken;
        }
        return taken == count ? S_OK : S_FALSE;
    }
    STDMETHODIMP Skip(ULONG count) override {
        _index += count;
        return _index > 1 ? S_FALSE : S_OK;
    }
    STDMETHODIMP Reset() override {
        _index = 0;
        return S_OK;
    }
    STDMETHODIMP Clone(IEnumPins** enumerator) override {
        if (enumerator == nullptr) {
            return E_POINTER;
        }
        *enumerator = new PinEnumerator(_pin, _index);
        return *enumerator == nullptr ? E_OUTOFMEMORY : S_OK;
    }

private:
    LONG _references = 1;
    IPin* _pin = nullptr;
    ULONG _index = 0;
};

class TypeEnumerator : public IEnumMediaTypes {
public:
    TypeEnumerator(LONGLONG interval, int width, int height, ULONG index)
        : _interval(interval), _width(width), _height(height), _index(index) {
        Mai2VcamLockModule();
    }
    ~TypeEnumerator() { Mai2VcamUnlockModule(); }

    STDMETHODIMP QueryInterface(REFIID riid, void** object) override {
        if (object == nullptr) {
            return E_POINTER;
        }
        if (riid == IID_IUnknown || riid == IID_IEnumMediaTypes) {
            *object = static_cast<IEnumMediaTypes*>(this);
            AddRef();
            return S_OK;
        }
        *object = nullptr;
        return E_NOINTERFACE;
    }
    STDMETHODIMP_(ULONG) AddRef() override { return (ULONG)InterlockedIncrement(&_references); }
    STDMETHODIMP_(ULONG) Release() override {
        LONG value = InterlockedDecrement(&_references);
        if (value == 0) {
            delete this;
        }
        return (ULONG)value;
    }

    STDMETHODIMP Next(ULONG count, AM_MEDIA_TYPE** types, ULONG* fetched) override {
        if (types == nullptr) {
            return E_POINTER;
        }
        if (count > 1 && fetched == nullptr) {
            return E_INVALIDARG;
        }
        ULONG taken = 0;
        if (count > 0 && _index == 0) {
            AM_MEDIA_TYPE* type = (AM_MEDIA_TYPE*)CoTaskMemAlloc(sizeof(AM_MEDIA_TYPE));
            if (type == nullptr) {
                return E_OUTOFMEMORY;
            }
            if (FAILED(Mai2VcamBuildMediaType(type, _interval, _width, _height))) {
                CoTaskMemFree(type);
                return E_OUTOFMEMORY;
            }
            types[0] = type;
            taken = 1;
            _index = 1;
        }
        if (fetched != nullptr) {
            *fetched = taken;
        }
        return taken == count ? S_OK : S_FALSE;
    }
    STDMETHODIMP Skip(ULONG count) override {
        _index += count;
        return _index > 1 ? S_FALSE : S_OK;
    }
    STDMETHODIMP Reset() override {
        _index = 0;
        return S_OK;
    }
    STDMETHODIMP Clone(IEnumMediaTypes** enumerator) override {
        if (enumerator == nullptr) {
            return E_POINTER;
        }
        *enumerator = new TypeEnumerator(_interval, _width, _height, _index);
        return *enumerator == nullptr ? E_OUTOFMEMORY : S_OK;
    }

private:
    LONG _references = 1;
    LONGLONG _interval = MAI2VCAM_DEFAULT_INTERVAL;
    int _width = MAI2VCAM_DEFAULT_WIDTH;
    int _height = MAI2VCAM_DEFAULT_HEIGHT;
    ULONG _index = 0;
};

// 媒体类型是否"完整指定"(足够直接拿去 ReceiveConnection)。
bool FullySpecified(const AM_MEDIA_TYPE* type) {
    return type != nullptr && type->majortype != GUID_NULL && type->subtype != GUID_NULL &&
           type->formattype != GUID_NULL && type->cbFormat >= sizeof(VIDEOINFOHEADER) &&
           type->pbFormat != nullptr;
}

}  // namespace

// ── 针脚 ────────────────────────────────────────────────────────────────────────

Mai2VcamPin::Mai2VcamPin(Mai2VcamFilter* owner) : _owner(owner) {
    InitializeCriticalSection(&_lock);
    Mai2VcamQueryQueueSize(&_width, &_height);
    Mai2VcamBuildMediaType(&_mt, MAI2VCAM_DEFAULT_INTERVAL, _width, _height);
    _hasMt = true;
    Mai2VcamLog("pin: 分辨率取自共享队列 = %dx%d", _width, _height);
}

// 未连接时把队列头的分辨率搬到本针脚上。这是分辨率透传的**唯一**入口, 所有格式相关的
// IPin/IAMStreamConfig 方法都先过它一次。
//
// ★为什么不能只在构造时读一次★ 消费端(尤其 Windows 帧服务器与部分游戏)会把过滤器实例缓存
// 住反复复用: 上位机改完分辨率、用户在消费端重新打开摄像头时, 走的可能是同一个针脚对象的
// 第二轮协商。只在构造时读就意味着那一轮仍按旧尺寸协商, 于是"重开也没用"。
// 已连接则一律不动 —— 那时媒体类型已与下游定死(见 _width 的说明)。
void Mai2VcamPin::_RefreshSize() {
    int width = 0;
    int height = 0;
    Mai2VcamQueryQueueSize(&width, &height);
    EnterCriticalSection(&_lock);
    if (_peer != nullptr || (width == _width && height == _height)) {
        LeaveCriticalSection(&_lock);
        return;
    }
    AM_MEDIA_TYPE updated = {};
    if (FAILED(Mai2VcamBuildMediaType(&updated, _interval, width, height))) {
        LeaveCriticalSection(&_lock);
        return;
    }
    const int previousWidth = _width;
    const int previousHeight = _height;
    if (_hasMt) {
        Mai2VcamFreeMediaTypeContents(&_mt);
    }
    _mt = updated;
    _hasMt = true;
    _width = width;
    _height = height;
    LeaveCriticalSection(&_lock);
    Mai2VcamLog("pin: 分辨率随队列更新 %dx%d → %dx%d(未连接, 透传新尺寸)", previousWidth,
                previousHeight, width, height);
}

Mai2VcamPin::~Mai2VcamPin() {
    Inactive();
    _ReleaseConnection();
    if (_hasMt) {
        Mai2VcamFreeMediaTypeContents(&_mt);
        _hasMt = false;
    }
    if (_qualitySink != nullptr) {
        _qualitySink->Release();
        _qualitySink = nullptr;
    }
    DeleteCriticalSection(&_lock);
}

STDMETHODIMP Mai2VcamPin::QueryInterface(REFIID riid, void** object) {
    if (object == nullptr) {
        return E_POINTER;
    }
    if (riid == IID_IUnknown || riid == IID_IPin) {
        *object = static_cast<IPin*>(this);
    } else if (riid == IID_IAMStreamConfig) {
        *object = static_cast<IAMStreamConfig*>(this);
    } else if (riid == IID_IKsPropertySet) {
        *object = static_cast<IKsPropertySet*>(this);
    } else if (riid == IID_IQualityControl) {
        *object = static_cast<IQualityControl*>(this);
    } else {
        *object = nullptr;
        return E_NOINTERFACE;
    }
    AddRef();
    return S_OK;
}

STDMETHODIMP_(ULONG) Mai2VcamPin::AddRef() { return _owner->AddRef(); }
STDMETHODIMP_(ULONG) Mai2VcamPin::Release() { return _owner->Release(); }

bool Mai2VcamPin::IsConnected() {
    EnterCriticalSection(&_lock);
    bool connected = _peer != nullptr;
    LeaveCriticalSection(&_lock);
    return connected;
}

STDMETHODIMP Mai2VcamPin::Connect(IPin* receive, const AM_MEDIA_TYPE* type) {
    if (receive == nullptr) {
        return E_POINTER;
    }
    if (_owner->State() != State_Stopped) {
        return VFW_E_NOT_STOPPED;
    }
    EnterCriticalSection(&_lock);
    if (_peer != nullptr) {
        LeaveCriticalSection(&_lock);
        return VFW_E_ALREADY_CONNECTED;
    }
    LeaveCriticalSection(&_lock);

    // 连接前最后一次机会把队列尺寸搬过来: 之后这个尺寸就与下游定死了。
    _RefreshSize();
    if (type != nullptr && !Mai2VcamAcceptMediaType(type, _width, _height)) {
        return VFW_E_TYPE_NOT_ACCEPTED;
    }
    if (FullySpecified(type)) {
        return _Attempt(receive, type);
    }
    // 部分指定或未指定: 用本针脚唯一支持的类型(帧率沿用对方给出的合理值)。
    AM_MEDIA_TYPE mine = {};
    HRESULT hr = Mai2VcamBuildMediaType(&mine, Mai2VcamIntervalOf(type), _width, _height);
    if (FAILED(hr)) {
        return hr;
    }
    hr = _Attempt(receive, &mine);
    Mai2VcamFreeMediaTypeContents(&mine);
    return hr;
}

HRESULT Mai2VcamPin::_Attempt(IPin* receive, const AM_MEDIA_TYPE* type) {
    AM_MEDIA_TYPE candidate = {};
    HRESULT hr = Mai2VcamCopyMediaType(&candidate, type);
    if (FAILED(hr)) {
        return hr;
    }
    hr = receive->ReceiveConnection(static_cast<IPin*>(this), &candidate);
    if (FAILED(hr)) {
        Mai2VcamFreeMediaTypeContents(&candidate);
        return hr;
    }
    IMemInputPin* input = nullptr;
    hr = receive->QueryInterface(IID_IMemInputPin, (void**)&input);
    if (FAILED(hr) || input == nullptr) {
        receive->Disconnect();
        Mai2VcamFreeMediaTypeContents(&candidate);
        return VFW_E_NO_TRANSPORT;
    }

    EnterCriticalSection(&_lock);
    if (_hasMt) {
        Mai2VcamFreeMediaTypeContents(&_mt);
    }
    _mt = candidate;
    _hasMt = true;
    _interval = Mai2VcamIntervalOf(&_mt);
    _peer = receive;
    _peer->AddRef();
    _input = input;
    LeaveCriticalSection(&_lock);

    hr = _DecideAllocator();
    if (FAILED(hr)) {
        _ReleaseConnection();
        receive->Disconnect();
        Mai2VcamLog("pin: allocator 协商失败 hr=0x%08X", hr);
        return hr;
    }
    Mai2VcamLog("pin: 已连接下游, NV12 %dx%d 帧间隔=%lld(100ns)", _width, _height, _interval);
    return S_OK;
}

HRESULT Mai2VcamPin::_DecideAllocator() {
    const LONG frameBytes = (LONG)Mai2VcamFrameBytes(_width, _height);
    ALLOCATOR_PROPERTIES request = {};
    if (FAILED(_input->GetAllocatorRequirements(&request))) {
        ZeroMemory(&request, sizeof(request));
    }
    IMemAllocator* allocator = nullptr;
    // 优先用下游提供的分配器: 渲染器/编码器常有自己的缓冲要求。
    if (FAILED(_input->GetAllocator(&allocator))) {
        allocator = nullptr;
    }
    for (int attempt = 0; attempt < 2; attempt++) {
        if (allocator == nullptr) {
            HRESULT hr = CoCreateInstance(CLSID_MemoryAllocator, nullptr, CLSCTX_INPROC_SERVER,
                                          IID_IMemAllocator, (void**)&allocator);
            if (FAILED(hr) || allocator == nullptr) {
                return VFW_E_NO_ALLOCATOR;
            }
        }
        ALLOCATOR_PROPERTIES want = {};
        want.cBuffers = request.cBuffers > 4 ? request.cBuffers : 4;
        want.cbBuffer =
                request.cbBuffer > frameBytes ? request.cbBuffer : frameBytes;
        want.cbAlign = request.cbAlign > 0 ? request.cbAlign : 1;
        want.cbPrefix = request.cbPrefix;
        ALLOCATOR_PROPERTIES actual = {};
        HRESULT hr = allocator->SetProperties(&want, &actual);
        if (SUCCEEDED(hr) && actual.cbBuffer >= frameBytes && actual.cBuffers >= 1) {
            hr = _input->NotifyAllocator(allocator, FALSE);
            if (SUCCEEDED(hr)) {
                EnterCriticalSection(&_lock);
                if (_allocator != nullptr) {
                    _allocator->Release();
                }
                _allocator = allocator;
                LeaveCriticalSection(&_lock);
                return S_OK;
            }
        }
        // 下游分配器不肯给足缓冲 → 换自建分配器再试一次, 不静默接受过小缓冲。
        allocator->Release();
        allocator = nullptr;
    }
    return VFW_E_NO_ALLOCATOR;
}

void Mai2VcamPin::_ReleaseConnection() {
    EnterCriticalSection(&_lock);
    IPin* peer = _peer;
    IMemInputPin* input = _input;
    IMemAllocator* allocator = _allocator;
    _peer = nullptr;
    _input = nullptr;
    _allocator = nullptr;
    LeaveCriticalSection(&_lock);
    if (allocator != nullptr) {
        allocator->Decommit();
        allocator->Release();
    }
    if (input != nullptr) {
        input->Release();
    }
    if (peer != nullptr) {
        peer->Release();
    }
}

STDMETHODIMP Mai2VcamPin::ReceiveConnection(IPin*, const AM_MEDIA_TYPE*) {
    // 输出针脚不接受反向连接。
    return E_UNEXPECTED;
}

STDMETHODIMP Mai2VcamPin::Disconnect() {
    if (_owner->State() != State_Stopped) {
        return VFW_E_NOT_STOPPED;
    }
    if (!IsConnected()) {
        return S_FALSE;
    }
    Inactive();
    _ReleaseConnection();
    Mai2VcamLog("pin: 下游已断开(生产者不受影响)");
    return S_OK;
}

STDMETHODIMP Mai2VcamPin::ConnectedTo(IPin** pin) {
    if (pin == nullptr) {
        return E_POINTER;
    }
    EnterCriticalSection(&_lock);
    IPin* peer = _peer;
    if (peer != nullptr) {
        peer->AddRef();
    }
    LeaveCriticalSection(&_lock);
    *pin = peer;
    return peer == nullptr ? VFW_E_NOT_CONNECTED : S_OK;
}

STDMETHODIMP Mai2VcamPin::ConnectionMediaType(AM_MEDIA_TYPE* type) {
    if (type == nullptr) {
        return E_POINTER;
    }
    EnterCriticalSection(&_lock);
    HRESULT hr = VFW_E_NOT_CONNECTED;
    if (_peer != nullptr && _hasMt) {
        hr = Mai2VcamCopyMediaType(type, &_mt);
    } else {
        ZeroMemory(type, sizeof(AM_MEDIA_TYPE));
    }
    LeaveCriticalSection(&_lock);
    return hr;
}

STDMETHODIMP Mai2VcamPin::QueryPinInfo(PIN_INFO* info) {
    if (info == nullptr) {
        return E_POINTER;
    }
    info->pFilter = static_cast<IBaseFilter*>(_owner);
    info->pFilter->AddRef();
    info->dir = PINDIR_OUTPUT;
    wcsncpy_s(info->achName, MAX_PIN_NAME, MAI2VCAM_PIN_NAME, _TRUNCATE);
    return S_OK;
}

STDMETHODIMP Mai2VcamPin::QueryDirection(PIN_DIRECTION* direction) {
    if (direction == nullptr) {
        return E_POINTER;
    }
    *direction = PINDIR_OUTPUT;
    return S_OK;
}

STDMETHODIMP Mai2VcamPin::QueryId(LPWSTR* id) {
    if (id == nullptr) {
        return E_POINTER;
    }
    const size_t bytes = (wcslen(MAI2VCAM_PIN_NAME) + 1) * sizeof(WCHAR);
    LPWSTR value = (LPWSTR)CoTaskMemAlloc(bytes);
    if (value == nullptr) {
        return E_OUTOFMEMORY;
    }
    memcpy(value, MAI2VCAM_PIN_NAME, bytes);
    *id = value;
    return S_OK;
}

STDMETHODIMP Mai2VcamPin::QueryAccept(const AM_MEDIA_TYPE* type) {
    _RefreshSize();
    return Mai2VcamAcceptMediaType(type, _width, _height) ? S_OK : S_FALSE;
}

STDMETHODIMP Mai2VcamPin::EnumMediaTypes(IEnumMediaTypes** enumerator) {
    if (enumerator == nullptr) {
        return E_POINTER;
    }
    // 图构建器就是靠这里报的类型决定要协商什么尺寸, 所以它必须反映队列**此刻**的分辨率。
    _RefreshSize();
    EnterCriticalSection(&_lock);
    LONGLONG interval = _interval;
    const int width = _width;
    const int height = _height;
    LeaveCriticalSection(&_lock);
    *enumerator = new TypeEnumerator(interval, width, height, 0);
    return *enumerator == nullptr ? E_OUTOFMEMORY : S_OK;
}

STDMETHODIMP Mai2VcamPin::QueryInternalConnections(IPin**, ULONG* count) {
    if (count != nullptr) {
        *count = 0;
    }
    return E_NOTIMPL;
}

STDMETHODIMP Mai2VcamPin::EndOfStream() { return E_UNEXPECTED; }
STDMETHODIMP Mai2VcamPin::BeginFlush() { return E_UNEXPECTED; }
STDMETHODIMP Mai2VcamPin::EndFlush() { return E_UNEXPECTED; }
STDMETHODIMP Mai2VcamPin::NewSegment(REFERENCE_TIME, REFERENCE_TIME, double) { return S_OK; }

// ── IAMStreamConfig ─────────────────────────────────────────────────────────────

STDMETHODIMP Mai2VcamPin::SetFormat(AM_MEDIA_TYPE* type) {
    if (type == nullptr) {
        return E_POINTER;
    }
    // 未连接时先与队列对齐, 再判可接受: 分辨率由生产者说了算, SetFormat 只能改帧率这类次要属性,
    // 拿旧尺寸来设一律拒绝(而不是默默按旧尺寸接受, 那就等于在这里把透传破掉)。
    _RefreshSize();
    if (!Mai2VcamAcceptMediaType(type, _width, _height)) {
        return VFW_E_INVALIDMEDIATYPE;
    }
    AM_MEDIA_TYPE updated = {};
    HRESULT hr = Mai2VcamBuildMediaType(&updated, Mai2VcamIntervalOf(type), _width, _height);
    if (FAILED(hr)) {
        return hr;
    }
    EnterCriticalSection(&_lock);
    IPin* peer = _peer;
    if (peer != nullptr) {
        peer->AddRef();
    }
    LeaveCriticalSection(&_lock);
    if (peer != nullptr) {
        hr = peer->QueryAccept(&updated);
        peer->Release();
        if (hr != S_OK) {
            Mai2VcamFreeMediaTypeContents(&updated);
            return VFW_E_INVALIDMEDIATYPE;
        }
    }
    EnterCriticalSection(&_lock);
    if (_hasMt) {
        Mai2VcamFreeMediaTypeContents(&_mt);
    }
    _mt = updated;
    _hasMt = true;
    _interval = Mai2VcamIntervalOf(&_mt);
    _notifyType = peer != nullptr;
    LeaveCriticalSection(&_lock);
    Mai2VcamLog("pin: SetFormat 帧间隔=%lld(100ns)", _interval);
    return S_OK;
}

STDMETHODIMP Mai2VcamPin::GetFormat(AM_MEDIA_TYPE** type) {
    if (type == nullptr) {
        return E_POINTER;
    }
    _RefreshSize();
    AM_MEDIA_TYPE* copy = (AM_MEDIA_TYPE*)CoTaskMemAlloc(sizeof(AM_MEDIA_TYPE));
    if (copy == nullptr) {
        return E_OUTOFMEMORY;
    }
    EnterCriticalSection(&_lock);
    HRESULT hr = Mai2VcamCopyMediaType(copy, &_mt);
    LeaveCriticalSection(&_lock);
    if (FAILED(hr)) {
        CoTaskMemFree(copy);
        return hr;
    }
    *type = copy;
    return S_OK;
}

STDMETHODIMP Mai2VcamPin::GetNumberOfCapabilities(int* count, int* size) {
    if (count == nullptr || size == nullptr) {
        return E_POINTER;
    }
    *count = 1;
    *size = sizeof(VIDEO_STREAM_CONFIG_CAPS);
    return S_OK;
}

STDMETHODIMP Mai2VcamPin::GetStreamCaps(int index, AM_MEDIA_TYPE** type, BYTE* capabilities) {
    if (type == nullptr || capabilities == nullptr) {
        return E_POINTER;
    }
    if (index != 0) {
        return S_FALSE;
    }
    // 消费端的"分辨率下拉"读的就是这里: 必须报队列此刻的尺寸, 且上下界与它相同(本源不缩放)。
    _RefreshSize();
    EnterCriticalSection(&_lock);
    const int width = _width;
    const int height = _height;
    LeaveCriticalSection(&_lock);
    AM_MEDIA_TYPE* copy = (AM_MEDIA_TYPE*)CoTaskMemAlloc(sizeof(AM_MEDIA_TYPE));
    if (copy == nullptr) {
        return E_OUTOFMEMORY;
    }
    HRESULT hr = Mai2VcamBuildMediaType(copy, MAI2VCAM_DEFAULT_INTERVAL, width, height);
    if (FAILED(hr)) {
        CoTaskMemFree(copy);
        return hr;
    }
    const LONGLONG frameBytes = (LONGLONG)Mai2VcamFrameBytes(width, height);
    VIDEO_STREAM_CONFIG_CAPS* caps = (VIDEO_STREAM_CONFIG_CAPS*)capabilities;
    ZeroMemory(caps, sizeof(VIDEO_STREAM_CONFIG_CAPS));
    caps->guid = FORMAT_VideoInfo;
    caps->VideoStandard = AnalogVideo_None;
    caps->InputSize.cx = width;
    caps->InputSize.cy = height;
    caps->MinCroppingSize = caps->InputSize;
    caps->MaxCroppingSize = caps->InputSize;
    caps->CropGranularityX = 1;
    caps->CropGranularityY = 1;
    caps->CropAlignX = 1;
    caps->CropAlignY = 1;
    caps->MinOutputSize = caps->InputSize;
    caps->MaxOutputSize = caps->InputSize;
    caps->OutputGranularityX = 1;
    caps->OutputGranularityY = 1;
    // 固定分辨率源: 不做缩放/裁剪, 只在帧率上给出协商范围(30fps..5fps)。
    caps->MinFrameInterval = MAI2VCAM_DEFAULT_INTERVAL;
    caps->MaxFrameInterval = MAI2VCAM_MAX_INTERVAL;
    caps->MinBitsPerSecond = (LONG)(frameBytes * 8 * 10000000LL / MAI2VCAM_MAX_INTERVAL);
    caps->MaxBitsPerSecond = (LONG)(frameBytes * 8 * 10000000LL / MAI2VCAM_DEFAULT_INTERVAL);
    *type = copy;
    return S_OK;
}

// ── IKsPropertySet ──────────────────────────────────────────────────────────────

STDMETHODIMP Mai2VcamPin::Set(REFGUID, DWORD, void*, DWORD, void*, DWORD) { return E_NOTIMPL; }

STDMETHODIMP Mai2VcamPin::Get(REFGUID set, DWORD id, void*, DWORD, void* property,
                              DWORD propertyLength, DWORD* returned) {
    if (set != AMPROPSETID_Pin) {
        return E_PROP_SET_UNSUPPORTED;
    }
    if (id != AMPROPERTY_PIN_CATEGORY) {
        return E_PROP_ID_UNSUPPORTED;
    }
    if (property == nullptr && returned == nullptr) {
        return E_POINTER;
    }
    if (returned != nullptr) {
        *returned = sizeof(GUID);
    }
    if (property == nullptr) {
        return S_OK;
    }
    if (propertyLength < sizeof(GUID)) {
        return E_UNEXPECTED;
    }
    *(GUID*)property = PIN_CATEGORY_CAPTURE;
    return S_OK;
}

STDMETHODIMP Mai2VcamPin::QuerySupported(REFGUID set, DWORD id, DWORD* support) {
    if (set != AMPROPSETID_Pin) {
        return E_PROP_SET_UNSUPPORTED;
    }
    if (id != AMPROPERTY_PIN_CATEGORY) {
        return E_PROP_ID_UNSUPPORTED;
    }
    if (support != nullptr) {
        *support = KSPROPERTY_SUPPORT_GET;
    }
    return S_OK;
}

// ── IQualityControl ─────────────────────────────────────────────────────────────

STDMETHODIMP Mai2VcamPin::Notify(IBaseFilter*, Quality) {
    // 实时源不做质量调节: 帧率由共享队列与协商节拍决定, 丢帧交给下游。
    return S_OK;
}

STDMETHODIMP Mai2VcamPin::SetSink(IQualityControl* sink) {
    EnterCriticalSection(&_lock);
    if (_qualitySink != nullptr) {
        _qualitySink->Release();
    }
    _qualitySink = sink;
    if (_qualitySink != nullptr) {
        _qualitySink->AddRef();
    }
    LeaveCriticalSection(&_lock);
    return S_OK;
}

// ── 推流线程 ────────────────────────────────────────────────────────────────────

HRESULT Mai2VcamPin::Active() {
    if (!IsConnected()) {
        return S_OK;
    }
    EnterCriticalSection(&_lock);
    if (_thread != nullptr) {
        LeaveCriticalSection(&_lock);
        return S_OK;
    }
    if (_allocator != nullptr) {
        HRESULT hr = _allocator->Commit();
        if (FAILED(hr)) {
            LeaveCriticalSection(&_lock);
            Mai2VcamLog("pin: allocator Commit 失败 hr=0x%08X", hr);
            return hr;
        }
    }
    _frameNumber = 0;
    _stop = CreateEventW(nullptr, TRUE, FALSE, nullptr);
    if (_stop == nullptr) {
        LeaveCriticalSection(&_lock);
        return HRESULT_FROM_WIN32(GetLastError());
    }
    _thread = CreateThread(nullptr, 0, _ThreadProc, this, 0, nullptr);
    if (_thread == nullptr) {
        CloseHandle(_stop);
        _stop = nullptr;
        LeaveCriticalSection(&_lock);
        return HRESULT_FROM_WIN32(GetLastError());
    }
    LeaveCriticalSection(&_lock);
    Mai2VcamLog("pin: 推流线程已启动");
    return S_OK;
}

bool Mai2VcamPin::IsStreaming() {
    EnterCriticalSection(&_lock);
    const bool streaming = _thread != nullptr;
    LeaveCriticalSection(&_lock);
    return streaming;
}

// 停流的**唯一**正确顺序, 一步都不能挪:
//   1) 置停止事件            —— 线程下一次等待立即返回
//   2) 下游 BeginFlush       —— 正阻塞在 Receive 里的线程立即返回
//   3) allocator Decommit    —— 正阻塞在 GetBuffer 里的线程立即返回(VFW_E_NOT_COMMITTED)
//   4) **无限**等线程退出    —— 三条阻塞通道都已打开, 线程必定能出来
//   5) EndFlush / 关句柄
//
// ★为什么不能有超时★: 旧实现等 3s 就放弃, 然后照样 CloseHandle(thread/stop) 并 Decommit ——
// 可那个还活着的线程仍在用 _stop 句柄、仍在碰 owner 与 allocator, 于是"超时"这条路径直接
// 等价于一条 use-after-free 通道(过滤器随后被析构, 线程还在跑)。宁可在这里等, 也不能放野线程。
HRESULT Mai2VcamPin::Inactive() {
    EnterCriticalSection(&_lock);
    HANDLE thread = _thread;
    HANDLE stop = _stop;
    IPin* peer = _peer;
    if (peer != nullptr) {
        peer->AddRef();
    }
    IMemAllocator* allocator = _allocator;
    if (allocator != nullptr) {
        allocator->AddRef();
    }
    // 成员先清空: 推流线程每轮都重读 _stop, 读到 nullptr 即自行收尾退出。
    // 句柄本身留到 join 之后才关, 线程持有的局部副本因此始终有效。
    _thread = nullptr;
    _stop = nullptr;
    LeaveCriticalSection(&_lock);

    if (stop != nullptr) {
        SetEvent(stop);
    }
    if (thread != nullptr) {
        if (peer != nullptr) {
            peer->BeginFlush();
        }
        if (allocator != nullptr) {
            allocator->Decommit();
        }
        WaitForSingleObject(thread, INFINITE);
        CloseHandle(thread);
        if (peer != nullptr) {
            peer->EndFlush();
        }
        Mai2VcamLog("pin: 推流线程已停止");
    } else if (allocator != nullptr) {
        // 没起过线程也要保证 allocator 回到未提交态(Active 可能刚 Commit 就失败了)。
        allocator->Decommit();
    }
    if (stop != nullptr) {
        CloseHandle(stop);
    }
    if (allocator != nullptr) {
        allocator->Release();
    }
    if (peer != nullptr) {
        peer->Release();
    }
    return S_OK;
}

DWORD WINAPI Mai2VcamPin::_ThreadProc(LPVOID context) {
    // 线程内会 CoCreateInstance? 不会: 分配器在连接阶段就绪。仍初始化 MTA 以便下游
    // 在 Receive 内部做 COM 调用时有合法的公寓。
    CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    static_cast<Mai2VcamPin*>(context)->_PushLoop();
    CoUninitialize();
    return 0;
}

void Mai2VcamPin::_PushLoop() {
    LARGE_INTEGER frequency = {};
    QueryPerformanceFrequency(&frequency);
    LARGE_INTEGER origin = {};
    QueryPerformanceCounter(&origin);

    for (;;) {
        EnterCriticalSection(&_lock);
        HANDLE stop = _stop;
        IMemAllocator* allocator = _allocator;
        if (allocator != nullptr) {
            allocator->AddRef();
        }
        IMemInputPin* input = _input;
        if (input != nullptr) {
            input->AddRef();
        }
        const LONGLONG interval = _interval;
        // 已连接期间分辨率冻结(_RefreshSize 见 _peer != nullptr 即不动), 这里只取本地副本,
        // 免得在锁外读成员。
        const int width = _width;
        const int height = _height;
        const long frameBytes = Mai2VcamFrameBytes(width, height);
        const bool notifyType = _notifyType;
        _notifyType = false;
        AM_MEDIA_TYPE pending = {};
        bool hasPending = false;
        if (notifyType && _hasMt) {
            hasPending = SUCCEEDED(Mai2VcamCopyMediaType(&pending, &_mt));
        }
        const LONGLONG frame = _frameNumber;
        LeaveCriticalSection(&_lock);

        if (stop == nullptr || allocator == nullptr || input == nullptr) {
            if (allocator != nullptr) {
                allocator->Release();
            }
            if (input != nullptr) {
                input->Release();
            }
            if (hasPending) {
                Mai2VcamFreeMediaTypeContents(&pending);
            }
            break;
        }

        IMediaSample* sample = nullptr;
        HRESULT hr = allocator->GetBuffer(&sample, nullptr, nullptr, 0);
        if (SUCCEEDED(hr) && sample != nullptr) {
            BYTE* data = nullptr;
            if (SUCCEEDED(sample->GetPointer(&data)) && data != nullptr &&
                sample->GetSize() >= frameBytes) {
                _reader.Read(data, width, height);
                sample->SetActualDataLength(frameBytes);
                REFERENCE_TIME start = frame * interval;
                REFERENCE_TIME end = start + interval;
                sample->SetTime(&start, &end);
                sample->SetSyncPoint(TRUE);
                sample->SetDiscontinuity(frame == 0 ? TRUE : FALSE);
                sample->SetPreroll(FALSE);
                if (hasPending) {
                    sample->SetMediaType(&pending);
                }
                hr = input->Receive(sample);
            } else {
                hr = E_UNEXPECTED;
            }
            sample->Release();
        }
        if (hasPending) {
            Mai2VcamFreeMediaTypeContents(&pending);
        }
        allocator->Release();
        input->Release();

        if (hr == S_OK) {
            EnterCriticalSection(&_lock);
            _frameNumber = frame + 1;
            LeaveCriticalSection(&_lock);
        } else if (hr == VFW_E_NOT_COMMITTED || hr == VFW_E_WRONG_STATE) {
            // 停机/断连引起, 等停止事件即可。
        } else if (FAILED(hr)) {
            Mai2VcamLog("pin: Receive 失败 hr=0x%08X", hr);
        }

        // 节拍: 以协商帧间隔为准的绝对时间轴, 避免累积漂移。
        LARGE_INTEGER now = {};
        QueryPerformanceCounter(&now);
        const LONGLONG elapsed100ns =
            frequency.QuadPart == 0
                ? 0
                : (now.QuadPart - origin.QuadPart) * 10000000LL / frequency.QuadPart;
        const LONGLONG due100ns = (frame + 1) * interval;
        DWORD wait = 0;
        if (due100ns > elapsed100ns) {
            wait = (DWORD)((due100ns - elapsed100ns) / 10000LL);
        }
        if (WaitForSingleObject(stop, wait) == WAIT_OBJECT_0) {
            break;
        }
    }
    _reader.Close();
}

// ── 过滤器 ──────────────────────────────────────────────────────────────────────

Mai2VcamFilter::Mai2VcamFilter() {
    InitializeCriticalSection(&_lock);
    wcsncpy_s(_name, MAX_FILTER_NAME, MAI2VCAM_FRIENDLY_NAME, _TRUNCATE);
    _pin = new Mai2VcamPin(this);
    Mai2VcamLockModule();
}

Mai2VcamFilter::~Mai2VcamFilter() {
    if (_pin != nullptr) {
        delete _pin;
        _pin = nullptr;
    }
    if (_clock != nullptr) {
        _clock->Release();
        _clock = nullptr;
    }
    DeleteCriticalSection(&_lock);
    Mai2VcamUnlockModule();
}

STDMETHODIMP Mai2VcamFilter::QueryInterface(REFIID riid, void** object) {
    if (object == nullptr) {
        return E_POINTER;
    }
    if (riid == IID_IUnknown || riid == IID_IPersist || riid == IID_IMediaFilter ||
        riid == IID_IBaseFilter) {
        *object = static_cast<IBaseFilter*>(this);
    } else if (riid == IID_IAMFilterMiscFlags) {
        *object = static_cast<IAMFilterMiscFlags*>(this);
    } else {
        *object = nullptr;
        return E_NOINTERFACE;
    }
    AddRef();
    return S_OK;
}

STDMETHODIMP_(ULONG) Mai2VcamFilter::AddRef() { return (ULONG)InterlockedIncrement(&_references); }

STDMETHODIMP_(ULONG) Mai2VcamFilter::Release() {
    LONG value = InterlockedDecrement(&_references);
    if (value == 0) {
        delete this;
    }
    return (ULONG)value;
}

STDMETHODIMP Mai2VcamFilter::GetClassID(CLSID* id) {
    if (id == nullptr) {
        return E_POINTER;
    }
    *id = CLSID_Mai2VcamDshow;
    return S_OK;
}

FILTER_STATE Mai2VcamFilter::State() {
    EnterCriticalSection(&_lock);
    FILTER_STATE state = _state;
    LeaveCriticalSection(&_lock);
    return state;
}

IReferenceClock* Mai2VcamFilter::Clock() {
    EnterCriticalSection(&_lock);
    IReferenceClock* clock = _clock;
    if (clock != nullptr) {
        clock->AddRef();
    }
    LeaveCriticalSection(&_lock);
    return clock;
}

STDMETHODIMP Mai2VcamFilter::Stop() {
    EnterCriticalSection(&_lock);
    const bool wasActive = _state != State_Stopped;
    _state = State_Stopped;
    LeaveCriticalSection(&_lock);
    if (wasActive && _pin != nullptr) {
        _pin->Inactive();
    }
    return S_OK;
}

STDMETHODIMP Mai2VcamFilter::Pause() {
    EnterCriticalSection(&_lock);
    const bool fromStopped = _state == State_Stopped;
    _state = State_Paused;
    LeaveCriticalSection(&_lock);
    // 与 DirectShow 源过滤器约定一致: 线程在 Paused 就开始推帧, 渲染器会阻塞在 Receive 上,
    // 图因此能完成 Paused→Running 的切换而不必等第一帧。
    if (fromStopped && _pin != nullptr) {
        HRESULT hr = _pin->Active();
        if (FAILED(hr)) {
            // ★失败必须完整回滚★: 旧实现先置 Paused 再 Active, 失败了状态却留在 Paused ——
            // 图以为自己暂停成功, 之后的 Run 会在没有推流线程的前提下把状态推到 Running,
            // 表现为"设备在、一帧不出"。这里把 allocator/线程残留清干净并退回 Stopped。
            _pin->Inactive();
            EnterCriticalSection(&_lock);
            _state = State_Stopped;
            LeaveCriticalSection(&_lock);
            Mai2VcamLog("filter: Pause 激活失败 hr=0x%08X, 已回滚到 Stopped", hr);
            return hr;
        }
    }
    return S_OK;
}

STDMETHODIMP Mai2VcamFilter::Run(REFERENCE_TIME) {
    EnterCriticalSection(&_lock);
    const bool fromStopped = _state == State_Stopped;
    LeaveCriticalSection(&_lock);
    if (fromStopped) {
        HRESULT hr = Pause();
        if (FAILED(hr)) {
            return hr;
        }
    }
    // ★Running 的前提是"帧真的在流"★: 已连接下游却没有推流线程, 说明 Active 那一步没成,
    // 此时置 Running 只会骗过图与应用。未连接则本来就不需要线程(源过滤器可以空跑)。
    if (_pin != nullptr && _pin->IsConnected() && !_pin->IsStreaming()) {
        _pin->Inactive();
        EnterCriticalSection(&_lock);
        _state = State_Stopped;
        LeaveCriticalSection(&_lock);
        Mai2VcamLog("filter: Run 被拒(推流线程未启动), 已回滚到 Stopped");
        return VFW_E_CANNOT_RENDER;
    }
    EnterCriticalSection(&_lock);
    _state = State_Running;
    LeaveCriticalSection(&_lock);
    return S_OK;
}

STDMETHODIMP Mai2VcamFilter::GetState(DWORD, FILTER_STATE* state) {
    if (state == nullptr) {
        return E_POINTER;
    }
    *state = State();
    return S_OK;
}

STDMETHODIMP Mai2VcamFilter::SetSyncSource(IReferenceClock* clock) {
    EnterCriticalSection(&_lock);
    if (_clock != nullptr) {
        _clock->Release();
    }
    _clock = clock;
    if (_clock != nullptr) {
        _clock->AddRef();
    }
    LeaveCriticalSection(&_lock);
    return S_OK;
}

STDMETHODIMP Mai2VcamFilter::GetSyncSource(IReferenceClock** clock) {
    if (clock == nullptr) {
        return E_POINTER;
    }
    *clock = Clock();
    return S_OK;
}

STDMETHODIMP Mai2VcamFilter::EnumPins(IEnumPins** enumerator) {
    if (enumerator == nullptr) {
        return E_POINTER;
    }
    if (_pin == nullptr) {
        return E_UNEXPECTED;
    }
    *enumerator = new PinEnumerator(static_cast<IPin*>(_pin), 0);
    return *enumerator == nullptr ? E_OUTOFMEMORY : S_OK;
}

STDMETHODIMP Mai2VcamFilter::FindPin(LPCWSTR id, IPin** pin) {
    if (pin == nullptr) {
        return E_POINTER;
    }
    *pin = nullptr;
    if (id == nullptr || _pin == nullptr || _wcsicmp(id, MAI2VCAM_PIN_NAME) != 0) {
        return VFW_E_NOT_FOUND;
    }
    *pin = static_cast<IPin*>(_pin);
    (*pin)->AddRef();
    return S_OK;
}

STDMETHODIMP Mai2VcamFilter::QueryFilterInfo(FILTER_INFO* info) {
    if (info == nullptr) {
        return E_POINTER;
    }
    EnterCriticalSection(&_lock);
    wcsncpy_s(info->achName, MAX_FILTER_NAME, _name, _TRUNCATE);
    info->pGraph = _graph;
    if (info->pGraph != nullptr) {
        info->pGraph->AddRef();
    }
    LeaveCriticalSection(&_lock);
    return S_OK;
}

STDMETHODIMP Mai2VcamFilter::JoinFilterGraph(IFilterGraph* graph, LPCWSTR name) {
    EnterCriticalSection(&_lock);
    // 规范要求这里**不加引用**: 图持有过滤器, 过滤器再持有图就形成环。
    _graph = graph;
    if (name != nullptr) {
        wcsncpy_s(_name, MAX_FILTER_NAME, name, _TRUNCATE);
    }
    LeaveCriticalSection(&_lock);
    return S_OK;
}

STDMETHODIMP Mai2VcamFilter::QueryVendorInfo(LPWSTR* vendor) {
    if (vendor != nullptr) {
        *vendor = nullptr;
    }
    return E_NOTIMPL;
}

STDMETHODIMP_(ULONG) Mai2VcamFilter::GetMiscFlags() { return AM_FILTER_MISC_FLAGS_IS_SOURCE; }

// ── 类工厂 ──────────────────────────────────────────────────────────────────────

// ★类工厂本身必须锁住模块★: DllCanUnloadNow 只看模块计数, 而 DllGetClassObject 返回的
// 工厂对象可以被调用方长期持有(枚举设备时很常见)。工厂不计数的话, COM 完全可以在工厂还活着时
// 把 DLL 卸掉, 下一次 CreateInstance 就调进已经 unmap 的代码里。
Mai2VcamClassFactory::Mai2VcamClassFactory() { Mai2VcamLockModule(); }
Mai2VcamClassFactory::~Mai2VcamClassFactory() { Mai2VcamUnlockModule(); }

STDMETHODIMP Mai2VcamClassFactory::QueryInterface(REFIID riid, void** object) {
    if (object == nullptr) {
        return E_POINTER;
    }
    if (riid == IID_IUnknown || riid == IID_IClassFactory) {
        *object = static_cast<IClassFactory*>(this);
        AddRef();
        return S_OK;
    }
    *object = nullptr;
    return E_NOINTERFACE;
}

STDMETHODIMP_(ULONG) Mai2VcamClassFactory::AddRef() {
    return (ULONG)InterlockedIncrement(&_references);
}

STDMETHODIMP_(ULONG) Mai2VcamClassFactory::Release() {
    LONG value = InterlockedDecrement(&_references);
    if (value == 0) {
        delete this;
    }
    return (ULONG)value;
}

STDMETHODIMP Mai2VcamClassFactory::CreateInstance(IUnknown* outer, REFIID riid, void** object) {
    if (object == nullptr) {
        return E_POINTER;
    }
    *object = nullptr;
    if (outer != nullptr) {
        return CLASS_E_NOAGGREGATION;
    }
    Mai2VcamFilter* filter = new Mai2VcamFilter();
    if (filter == nullptr) {
        return E_OUTOFMEMORY;
    }
    HRESULT hr = filter->QueryInterface(riid, object);
    filter->Release();
    return hr;
}

STDMETHODIMP Mai2VcamClassFactory::LockServer(BOOL lock) {
    if (lock) {
        Mai2VcamLockModule();
    } else {
        Mai2VcamUnlockModule();
    }
    return S_OK;
}
