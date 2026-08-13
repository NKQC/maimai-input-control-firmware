#pragma once

#include <cstdint>

/**
 * PersistenceTxn is the single SAVE_CONFIG completion barrier.
 *
 * A request is accepted once, then remains pending until every requested store
 * has entered NvStore and NvStore has finished verified commits.  The service
 * never owns flash buffers and never performs I/O itself; main.cpp supplies the
 * existing safe commit window and reports preparation failures.
 */
class PersistenceTxn {
public:
    enum class Result : uint8_t {
        NONE = 0,
        OK,
        CONFIG_ERROR,
        BUSY,
    };

    static PersistenceTxn* getInstance();

    /// Starts a save barrier. Repeating the same sequence is idempotent; a
    /// different sequence cannot overtake an active transaction.
    Result begin(uint8_t seq);
    /// Claims the one-time request that transfers current service snapshots into
    /// NvStore. Replayed sequence numbers never claim it again.
    bool take_prepare_request();
    void note_prepare_result(bool ok);
    void poll();

    bool active() const { return _active; }
    bool owns(uint8_t seq) const { return _active && _seq == seq; }
    bool awaiting_response(uint8_t seq) const {
        return (_active || _terminal_ready) && _seq == seq;
    }
    bool completed(uint8_t seq) const;
    bool take_terminal(uint8_t* seq, Result* result);

private:
    PersistenceTxn() = default;
    PersistenceTxn(const PersistenceTxn&) = delete;

    void _finish(Result result);

    static PersistenceTxn* _instance;
    uint8_t _seq = 0;
    uint32_t _fail_base = 0;
    bool _active = false;
    bool _prepare_requested = false;
    bool _prepared = false;
    bool _prepare_failed = false;
    bool _terminal_ready = false;
    uint8_t _completed_seq = 0;
    bool _completed_valid = false;
    uint32_t _completed_at_ms = 0;
    Result _terminal = Result::NONE;
    Result _completed = Result::NONE;
};
