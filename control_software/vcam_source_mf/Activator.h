#pragma once

struct Activator;
using ActivatorBase = winrt::implements<Activator, CBaseAttributes<IMFActivate>>;

struct Activator : ActivatorBase
{
public:
	STDMETHODIMP QueryInterface(REFIID riid, void** ppv) noexcept override
	{
		auto hr = ActivatorBase::QueryInterface(riid, ppv);
		WINTRACE(L"Activator::QueryInterface iid:%s hr:0x%08X object:%p", GUID_ToStringW(riid).c_str(), hr, ppv ? *ppv : nullptr);
		return hr;
	}

	// IMFActivate
	STDMETHOD(ActivateObject(REFIID riid, void** ppv));
	STDMETHOD(ShutdownObject)();
	STDMETHOD(DetachObject)();

public:
	Activator()
	{
		SetBaseAttributesTraceName(L"ActivatorAtts");
	}

	HRESULT Initialize();

private:
	int32_t query_interface_tearoff(winrt::guid const& id, void** object) const noexcept override
	{
		if (id == winrt::guid_of<IMFAttributes>())
		{
			const_cast<Activator*>(this)->AddRef();
			*object = static_cast<IMFAttributes*>(const_cast<Activator*>(this));
			return S_OK;
		}

		RETURN_HR_MSG(E_NOINTERFACE, "Activator QueryInterface failed on IID %s", GUID_ToStringW(id).c_str());
	}

private:
	winrt::com_ptr<MediaSource> _source;
};

