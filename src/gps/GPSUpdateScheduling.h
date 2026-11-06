#pragma once

#include "configuration.h"

// Approximates the GPS_HARDSLEEP/GPS_SOFTSLEEP crossover curve without pow(); see .cpp for the
// sampled reference values it interpolates between.
uint32_t gpsHardsleepThresholdMs(uint32_t predictedSearchSecs);

// Inputs for the motion gate; assembled by the caller from config, the motion provider and the
// scheduler state. Ages use UINT32_MAX for "never". Defaults fail toward passthrough: an
// omitted quiet window means "never moving", an omitted age bound means "always stale".
struct MotionGateInputs {
    bool enabled = false;         // Compile-time gate AND an accelerometer is present
    bool hasFixSinceBoot = false; // First-fix rule: gating never applies before this
    uint32_t secondsSinceValidFix = UINT32_MAX;
    uint32_t secondsSinceMotion = UINT32_MAX;
    uint32_t consecutiveSearchFailures = 0; // >0 keeps failed-search backoff in charge
    uint32_t quietSecs = UINT32_MAX;        // Motion younger than this counts as "moving"
    uint32_t maxFixAgeSecs = 0;             // A quiet fix older than this forces a refresh search
};

// Pure decision helper: returns the effective ms until the next search given the stock remaining
// time. 0 = search now (fresh motion with a pre-motion fix), unchanged = stock cadence, raised to
// >= kMotionRecheckMs = postponed while quiet with a fresh fix. Declared for unit tests.
constexpr uint32_t kMotionRecheckMs = 5000;
uint32_t applyMotionGate(uint32_t remainingMs, const MotionGateInputs &in);

// Encapsulates code responsible for the timing of GPS updates
class GPSUpdateScheduling
{
  public:
    // Marks the time of these events, for calculation use
    void informSearching();
    void informValidFix();
    void informGotLock();      // Predicted lock-time is recalculated here
    void informSearchFailed(); // Search ended without a fix; prediction is left untouched

    void reset();           // Reset the prediction - after GPS::disable() / GPS::enable()
    bool isUpdateDue();     // Is it time to begin searching for a GPS position?
    bool searchedTooLong(); // Have we been searching for too long?
    bool hasValidFixSinceSearchStarted() const;

    uint32_t
    msUntilNextSearch(); // How long until we need to begin searching for a GPS position? Info provided to GPS hardware for sleep
    uint32_t elapsedSearchMs();           // How long have we been searching so far?
    uint32_t predictedSearchDurationMs(); // How long do we expect to spend searching for a lock?
    uint32_t secondsSinceValidFix();      // UINT32_MAX before the first fix; history, not prediction
    bool everValidFix() const { return validFixEverSeen; }
    uint32_t searchFailures() const { return consecutiveFailures; } // For the motion gate's backoff rule

  private:
    void updateLockTimePrediction(); // Called from informGotLock
    bool searching = false;          // Set by the inform*() calls; never inferred from stamp ordering
    bool validFixReceived = false;
    uint32_t searchStartedMs = 0;
    uint32_t searchEndedMs = 0;
    uint32_t lastValidFixMs = 0;   // Survives reset(): describes history, not the current prediction
    bool validFixEverSeen = false; // Flag, not a stamp!=0 test: a stamp can legally be 0 at the wrap
    uint32_t searchCount = 0;
    uint32_t predictedMsToGetLock = 0;
    uint32_t consecutiveFailures = 0; // Count of search cycles that ended without a fix; reset on lock

    const float weighting = 0.2; // Controls exponential smoothing of lock-times prediction. 20% weighting of "latest lock-time".
};
