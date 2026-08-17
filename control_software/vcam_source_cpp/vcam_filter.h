// DirectShow 视频捕获源过滤器: 一个过滤器 + 一个输出针脚(NV12 推流)。
//
// 只实现消费端真正会用到的接口集合:
//   过滤器: IBaseFilter(含 IPersist/IMediaFilter) + IAMFilterMiscFlags(声明自己是源)
//   针脚:   IPin + IAMStreamConfig(格式/帧率协商) + IKsPropertySet(针脚类别=Capture)
//           + IQualityControl(质量消息吞掉)
// 针脚生命周期绑定过滤器: 针脚的 AddRef/Release 直接转给过滤器, 这是 DirectShow 的常规做法,
// 也避免"图还持有针脚而过滤器已析构"的悬垂。

#pragma once

#include "vcam_common.h"
#include "vcam_queue.h"

class Mai2VcamFilter;

class Mai2VcamPin : public IPin, public IAMStreamConfig, public IKsPropertySet, public IQualityControl {
public:
    explicit Mai2VcamPin(Mai2VcamFilter* owner);
    ~Mai2VcamPin();

    // IUnknown — 引用计数委托给过滤器。
    STDMETHODIMP QueryInterface(REFIID riid, void** object) override;
    STDMETHODIMP_(ULONG) AddRef() override;
    STDMETHODIMP_(ULONG) Release() override;

    // IPin
    STDMETHODIMP Connect(IPin* receive, const AM_MEDIA_TYPE* type) override;
    STDMETHODIMP ReceiveConnection(IPin* connector, const AM_MEDIA_TYPE* type) override;
    STDMETHODIMP Disconnect() override;
    STDMETHODIMP ConnectedTo(IPin** pin) override;
    STDMETHODIMP ConnectionMediaType(AM_MEDIA_TYPE* type) override;
    STDMETHODIMP QueryPinInfo(PIN_INFO* info) override;
    STDMETHODIMP QueryDirection(PIN_DIRECTION* direction) override;
    STDMETHODIMP QueryId(LPWSTR* id) override;
    STDMETHODIMP QueryAccept(const AM_MEDIA_TYPE* type) override;
    STDMETHODIMP EnumMediaTypes(IEnumMediaTypes** enumerator) override;
    STDMETHODIMP QueryInternalConnections(IPin** pins, ULONG* count) override;
    STDMETHODIMP EndOfStream() override;
    STDMETHODIMP BeginFlush() override;
    STDMETHODIMP EndFlush() override;
    STDMETHODIMP NewSegment(REFERENCE_TIME start, REFERENCE_TIME stop, double rate) override;

    // IAMStreamConfig
    STDMETHODIMP SetFormat(AM_MEDIA_TYPE* type) override;
    STDMETHODIMP GetFormat(AM_MEDIA_TYPE** type) override;
    STDMETHODIMP GetNumberOfCapabilities(int* count, int* size) override;
    STDMETHODIMP GetStreamCaps(int index, AM_MEDIA_TYPE** type, BYTE* capabilities) override;

    // IKsPropertySet — 捕获图靠 PIN_CATEGORY_CAPTURE 找到取流针脚。
    STDMETHODIMP Set(REFGUID set, DWORD id, void* instance, DWORD instanceLength, void* property,
                     DWORD propertyLength) override;
    STDMETHODIMP Get(REFGUID set, DWORD id, void* instance, DWORD instanceLength, void* property,
                     DWORD propertyLength, DWORD* returned) override;
    STDMETHODIMP QuerySupported(REFGUID set, DWORD id, DWORD* support) override;

    // IQualityControl
    STDMETHODIMP Notify(IBaseFilter* sender, Quality quality) override;
    STDMETHODIMP SetSink(IQualityControl* sink) override;

    // 过滤器状态迁移驱动的内部接口。
    HRESULT Active();
    HRESULT Inactive();
    bool IsConnected();
    // 推流线程是否确实在跑。Run() 靠它拒绝"没有线程却宣称 Running"的状态。
    bool IsStreaming();

private:
    static DWORD WINAPI _ThreadProc(LPVOID context);
    void _PushLoop();
    HRESULT _Attempt(IPin* receive, const AM_MEDIA_TYPE* type);
    HRESULT _DecideAllocator();
    void _ReleaseConnection();
    // 未连接时重新从队列头取一次分辨率, 变了就连 _mt 一起换掉。已连接则原样返回(见 _width 说明)。
    void _RefreshSize();

    Mai2VcamFilter* _owner = nullptr;
    CRITICAL_SECTION _lock = {};

    IPin* _peer = nullptr;
    IMemInputPin* _input = nullptr;
    IMemAllocator* _allocator = nullptr;
    IQualityControl* _qualitySink = nullptr;
    AM_MEDIA_TYPE _mt = {};
    bool _hasMt = false;
    LONGLONG _interval = MAI2VCAM_DEFAULT_INTERVAL;
    // SetFormat 在连接状态下改帧率时置位, 下一帧携带新类型通知下游。
    bool _notifyType = false;
    // 本针脚报出去的分辨率 = 共享队列头里生产者当前的分辨率(生产者不在则用回落值)。
    //
    // ★分辨率硬透传, 一路不缩放★ 未连接期间的每个格式相关入口(构造 / QueryAccept /
    // EnumMediaTypes / GetFormat / GetStreamCaps / Connect)都会先 _RefreshSize() 重取一次,
    // 所以下游协商到的必然就是上位机设置的那个尺寸 —— 即使过滤器实例被消费端缓存着复用,
    // 只要它重新打开(重新走一遍协商)就会拿到新值。
    //
    // ★为什么连上之后不能再跟着队列变★ DirectShow 的媒体类型在 Connect 时就与下游定死了,
    // 下游据此申请缓冲、配置转换与渲染; 运行中换尺寸没有合法的通知路径(SetMediaType 只能改
    // 同尺寸下的次要属性)。所以连接期间本值冻结, 生产者此时改分辨率只会让
    // Mai2VcamQueueReader::Read 判尺寸不等而输出占位帧(**不缩放**), 由上位机侧强制卸载重建
    // 摄像头 + 消费端重开来收敛。
    int _width = MAI2VCAM_DEFAULT_WIDTH;
    int _height = MAI2VCAM_DEFAULT_HEIGHT;

    HANDLE _thread = nullptr;
    HANDLE _stop = nullptr;
    LONGLONG _frameNumber = 0;
    Mai2VcamQueueReader _reader;
};

class Mai2VcamFilter : public IBaseFilter, public IAMFilterMiscFlags {
public:
    Mai2VcamFilter();
    ~Mai2VcamFilter();

    // IUnknown
    STDMETHODIMP QueryInterface(REFIID riid, void** object) override;
    STDMETHODIMP_(ULONG) AddRef() override;
    STDMETHODIMP_(ULONG) Release() override;

    // IPersist
    STDMETHODIMP GetClassID(CLSID* id) override;

    // IMediaFilter
    STDMETHODIMP Stop() override;
    STDMETHODIMP Pause() override;
    STDMETHODIMP Run(REFERENCE_TIME start) override;
    STDMETHODIMP GetState(DWORD milliseconds, FILTER_STATE* state) override;
    STDMETHODIMP SetSyncSource(IReferenceClock* clock) override;
    STDMETHODIMP GetSyncSource(IReferenceClock** clock) override;

    // IBaseFilter
    STDMETHODIMP EnumPins(IEnumPins** enumerator) override;
    STDMETHODIMP FindPin(LPCWSTR id, IPin** pin) override;
    STDMETHODIMP QueryFilterInfo(FILTER_INFO* info) override;
    STDMETHODIMP JoinFilterGraph(IFilterGraph* graph, LPCWSTR name) override;
    STDMETHODIMP QueryVendorInfo(LPWSTR* vendor) override;

    // IAMFilterMiscFlags
    STDMETHODIMP_(ULONG) GetMiscFlags() override;

    FILTER_STATE State();
    IReferenceClock* Clock();

private:
    CRITICAL_SECTION _lock = {};
    LONG _references = 1;
    FILTER_STATE _state = State_Stopped;
    IReferenceClock* _clock = nullptr;
    // 图指针按规范**不加引用**, 否则形成引用环导致图永不释放。
    IFilterGraph* _graph = nullptr;
    WCHAR _name[MAX_FILTER_NAME] = {0};
    Mai2VcamPin* _pin = nullptr;
};

// 类工厂。
class Mai2VcamClassFactory : public IClassFactory {
public:
    // 工厂存活期间锁住模块(见 vcam_filter.cpp 的注释): 只靠 LockServer 不够, 调用方未必调它。
    Mai2VcamClassFactory();
    ~Mai2VcamClassFactory();

    STDMETHODIMP QueryInterface(REFIID riid, void** object) override;
    STDMETHODIMP_(ULONG) AddRef() override;
    STDMETHODIMP_(ULONG) Release() override;
    STDMETHODIMP CreateInstance(IUnknown* outer, REFIID riid, void** object) override;
    STDMETHODIMP LockServer(BOOL lock) override;

private:
    LONG _references = 1;
};
