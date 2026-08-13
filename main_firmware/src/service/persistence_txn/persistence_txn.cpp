#include "persistence_txn.h"

#include <Arduino.h>
#include "../nv_store/nv_store.h"

PersistenceTxn* PersistenceTxn::_instance = nullptr;

PersistenceTxn* PersistenceTxn::getInstance() {
    if (_instance == nullptr) {
        static PersistenceTxn instance;
        _instance = &instance;
    }
    return _instance;
}

PersistenceTxn::Result PersistenceTxn::begin(uint8_t seq) {
    if (_active) return _seq == seq ? Result::NONE : Result::BUSY;
    if (_terminal_ready && _seq == seq) return Result::NONE;
    if (_completed_valid && _completed_seq == seq &&
        (uint32_t)(millis() - _completed_at_ms) <= 10000u) {
        _seq = seq;
        _terminal = _completed;
        _terminal_ready = true;
        return Result::NONE;
    }
    _seq = seq;
    _fail_base = NvStore::getInstance()->commit_fail_count();
    _prepare_requested = false;
    _prepared = false;
    _prepare_failed = false;
    _terminal_ready = false;
    _terminal = Result::NONE;
    _active = true;
    return Result::NONE;
}

bool PersistenceTxn::take_prepare_request() {
    if (!_active || _prepare_requested) return false;
    _prepare_requested = true;
    return true;
}

void PersistenceTxn::note_prepare_result(bool ok) {
    // Preparation belongs exclusively to a claimed active transaction. Ordinary
    // runtime commit windows must not turn a future SAVE_CONFIG into a
    // completed transaction before it has claimed its snapshot.
    if (!_active || !_prepare_requested) return;
    _prepared = true;
    if (!ok) _prepare_failed = true;
}

void PersistenceTxn::poll() {
    if (!_active || !_prepared) return;
    NvStore* store = NvStore::getInstance();
    if (_prepare_failed || store->commit_fail_count() != _fail_base) {
        _finish(Result::CONFIG_ERROR);
        return;
    }
    // The final state is only observable after no request signal, no dirty
    // region and no sector write remains. This is the verified flash barrier.
    if (!store->commit_in_progress() && !store->dirty()) {
        _finish(Result::OK);
    }
}

bool PersistenceTxn::completed(uint8_t seq) const {
    return _completed_valid && _completed_seq == seq &&
        (uint32_t)(millis() - _completed_at_ms) <= 10000u;
}

bool PersistenceTxn::take_terminal(uint8_t* seq, Result* result) {
    if (!_terminal_ready) return false;
    if (seq != nullptr) *seq = _seq;
    if (result != nullptr) *result = _terminal;
    _terminal_ready = false;
    return true;
}

void PersistenceTxn::_finish(Result result) {
    _active = false;
    _terminal = result;
    _completed_seq = _seq;
    _completed = result;
    _completed_valid = true;
    _completed_at_ms = millis();
    _terminal_ready = true;
}
