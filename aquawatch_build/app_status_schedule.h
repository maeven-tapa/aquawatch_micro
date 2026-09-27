#pragma once
#include <stdint.h>

// Startup is immediately due. Subsequent batches wait three minutes after
// completion; unavailable-network checks retry without sending an SMS burst.
class AppStatusSchedule {
 public:
  static constexpr uint32_t INTERVAL_MS = 180000;
  static constexpr uint32_t NETWORK_RETRY_MS = 10000;
  bool due(uint32_t now) const {
    return !hasAttempt_ || uint32_t(now - lastAttemptAt_) >=
      (networkRetry_ ? NETWORK_RETRY_MS : INTERVAL_MS);
  }
  void networkUnavailable(uint32_t now) { record(now, true); }
  void batchFinished(uint32_t now) { record(now, false); }
 private:
  void record(uint32_t now, bool retry) {
    lastAttemptAt_ = now; networkRetry_ = retry; hasAttempt_ = true;
  }
  uint32_t lastAttemptAt_ = 0;
  bool hasAttempt_ = false, networkRetry_ = false;
};
