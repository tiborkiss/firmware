#include "Arduino.h"
#include "TestUtil.h"
#include "UptimeClock.h"
#include "gps/GPSUpdateScheduling.h"
#include <cmath>
#include <cstdio>
#include <unity.h>

void setUp(void)
{
    Time::setTestMillis(0);
}
void tearDown(void)
{
    Time::useRealClock();
}

// Confirms gpsHardsleepThresholdMs()'s pow()-free lookup table tracks the original
// `2750 * pow(seconds, 1.22)` curve closely.
static double originalFormula(uint32_t seconds)
{
    return 2750.0 * std::pow((double)seconds, 1.22);
}

static void test_matches_original_formula_at_sampled_points(void)
{
    // Off-breakpoint values only - a breakpoint interpolates exactly by construction, so it would
    // test nothing here (test_exact_at_table_breakpoints covers those). Includes both worst-error
    // inputs: 7s (1.60%) and 728s (0.55%). Capped at 900s, the pre-existing 15-minute search clamp.
    const uint32_t samples[] = {4, 6, 7, 8, 9, 33, 100, 150, 500, 728, 899};
    for (uint32_t s : samples) {
        double expected = originalFormula(s);
        uint32_t actual = gpsHardsleepThresholdMs(s);
        // Pure integer arithmetic, so results are bit-identical everywhere - no float noise to
        // leave headroom for, and these sit just above the measured worst cases.
        double tolerance = expected * (s < 10 ? 0.02 : 0.0075);
        TEST_ASSERT_DOUBLE_WITHIN(tolerance, expected, (double)actual);
    }
}

static void test_zero_seconds_is_zero(void)
{
    TEST_ASSERT_EQUAL_UINT32(0, gpsHardsleepThresholdMs(0));
}

static void test_monotonically_nondecreasing(void)
{
    uint32_t prev = gpsHardsleepThresholdMs(0);
    for (uint32_t s = 1; s <= 1200; s += 7) {
        uint32_t cur = gpsHardsleepThresholdMs(s);
        TEST_ASSERT_GREATER_OR_EQUAL_UINT32(prev, cur);
        prev = cur;
    }
}

static void test_exact_at_table_breakpoints(void)
{
    // Every breakpoint must return its own sampled value. Catches an off-by-one in the segment
    // scan, which a percentage bound on interpolated points would absorb.
    const uint32_t breakpoints[] = {0, 1, 2, 3, 5, 10, 15, 20, 30, 45, 60, 90, 120, 180, 240, 300, 450, 600, 900};
    for (uint32_t s : breakpoints) {
        char msg[64];
        snprintf(msg, sizeof(msg), "breakpoint %us", s);
        // Within 2ms, not exact: the 30s entry is rounded 1ms high, and pow() can differ by an ULP
        // across libm implementations. A real off-by-one in the scan misses by thousands.
        TEST_ASSERT_UINT32_WITHIN_MESSAGE(2, (uint32_t)(originalFormula(s) + 0.5), gpsHardsleepThresholdMs(s), msg);
    }
}

static void test_clamps_above_table_range(void)
{
    uint32_t atMax = gpsHardsleepThresholdMs(900);
    TEST_ASSERT_EQUAL_UINT32(atMax, gpsHardsleepThresholdMs(2000));
    TEST_ASSERT_EQUAL_UINT32(atMax, gpsHardsleepThresholdMs(UINT32_MAX));
}

static void test_clamp_boundary(void)
{
    // The clamp must engage exactly at the last table point, not before or after it.
    TEST_ASSERT_LESS_THAN_UINT32(gpsHardsleepThresholdMs(900), gpsHardsleepThresholdMs(899));
    TEST_ASSERT_EQUAL_UINT32(gpsHardsleepThresholdMs(900), gpsHardsleepThresholdMs(901));
}

// elapsedSearchMs() across the 32-bit millis() wrap. Ordering the two raw stamps, as it used to,
// reports an idle receiver as searching or a searching one as idle, and searchedTooLong() acts on it.

// A search that has not started yet reads as idle, not as a search of length millis().
static void test_elapsed_is_zero_before_any_search(void)
{
    GPSUpdateScheduling s;
    Time::setTestMillis(90 * 1000);
    TEST_ASSERT_EQUAL_UINT32(0, s.elapsedSearchMs());
}

static void test_elapsed_tracks_the_clock_while_searching(void)
{
    GPSUpdateScheduling s;
    Time::setTestMillis(10 * 1000);
    s.informSearching();
    Time::advanceTestMillis(7 * 1000);
    TEST_ASSERT_EQUAL_UINT32(7 * 1000, s.elapsedSearchMs());
}

static void test_elapsed_is_zero_once_the_search_ends(void)
{
    GPSUpdateScheduling s;
    Time::setTestMillis(10 * 1000);
    s.informSearching();
    Time::advanceTestMillis(7 * 1000);
    s.informGotLock();
    Time::advanceTestMillis(60 * 1000);
    TEST_ASSERT_EQUAL_UINT32(0, s.elapsedSearchMs());

    s.informSearching();
    Time::advanceTestMillis(3 * 1000);
    s.informSearchFailed();
    TEST_ASSERT_EQUAL_UINT32(0, s.elapsedSearchMs());
}

// Start before the wrap, still searching after it: elapsed must be the real 10s, not ~49.7 days.
static void test_elapsed_is_exact_across_the_wrap(void)
{
    GPSUpdateScheduling s;
    Time::setTestMillis(0xFFFFF000u);
    s.informSearching();
    Time::advanceTestMillis(0x1000u + 6 * 1000); // 4.096s to the wrap, then 6s past it
    TEST_ASSERT_EQUAL_UINT32(0x1000u + 6 * 1000, s.elapsedSearchMs());
}

// The regression: started before the wrap, ended after it, so searchStartedMs > searchEndedMs.
// The receiver is idle and elapsed must say so.
static void test_search_ending_after_the_wrap_reads_as_idle(void)
{
    GPSUpdateScheduling s;
    Time::setTestMillis(0xFFFFF000u);
    s.informSearching();
    Time::advanceTestMillis(0x1000u + 2 * 1000);
    s.informGotLock();
    // The stamps really are inverted: the search ended at a smaller millis() than it started at.
    TEST_ASSERT_LESS_THAN_UINT32(0xFFFFF000u, Time::getMillis());
    Time::advanceTestMillis(30 * 60 * 1000);
    TEST_ASSERT_EQUAL_UINT32(0, s.elapsedSearchMs());
}

// The mirror image: the previous search ended before the wrap, this one started after it, so
// searchStartedMs < searchEndedMs while a search is genuinely in progress.
static void test_search_starting_after_the_wrap_reads_as_searching(void)
{
    GPSUpdateScheduling s;
    Time::setTestMillis(0xFFFFF000u);
    s.informSearching();
    Time::advanceTestMillis(1000);
    s.informGotLock();
    Time::advanceTestMillis(0x1000u); // over the wrap
    s.informSearching();
    Time::advanceTestMillis(12 * 1000);
    TEST_ASSERT_EQUAL_UINT32(12 * 1000, s.elapsedSearchMs());
}

static void test_reset_clears_the_search_state(void)
{
    GPSUpdateScheduling s;
    Time::setTestMillis(10 * 1000);
    s.informSearching();
    Time::advanceTestMillis(5 * 1000);
    s.reset();
    TEST_ASSERT_EQUAL_UINT32(0, s.elapsedSearchMs());
}

// Seconds since the last valid fix: UINT32_MAX before the first fix, ages afterward, and the
// stamp survives reset() (it is history for the motion gate's freshness bound, not prediction).
static void test_fix_age_is_never_before_first_fix(void)
{
    GPSUpdateScheduling s;
    Time::setTestMillis(60 * 1000);
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, s.secondsSinceValidFix());
    TEST_ASSERT_FALSE(s.everValidFix());
}

static void test_fix_age_tracks_time_since_valid_fix(void)
{
    GPSUpdateScheduling s;
    Time::setTestMillis(10 * 1000);
    s.informSearching();
    Time::advanceTestMillis(7 * 1000);
    s.informValidFix();
    Time::advanceTestMillis(33 * 1000);
    TEST_ASSERT_EQUAL_UINT32(33, s.secondsSinceValidFix());
    TEST_ASSERT_TRUE(s.everValidFix());
}

static void test_fix_age_ignores_stamps_outside_a_search(void)
{
    GPSUpdateScheduling s;
    Time::setTestMillis(10 * 1000);
    s.informSearching();
    Time::advanceTestMillis(5 * 1000);
    s.informValidFix(); // the real stamp, mid-search
    s.informGotLock();  // search over: a stray informValidFix must not restamp
    Time::advanceTestMillis(50 * 1000);
    s.informValidFix();
    // 50s since the real mid-search stamp; 0 would mean the stray call restamped.
    TEST_ASSERT_EQUAL_UINT32(50, s.secondsSinceValidFix());
}

static void test_fix_age_survives_reset_and_ages_on(void)
{
    GPSUpdateScheduling s;
    Time::setTestMillis(10 * 1000);
    s.informSearching();
    s.informValidFix();
    Time::advanceTestMillis(20 * 1000);
    s.reset();
    Time::advanceTestMillis(30 * 1000);
    TEST_ASSERT_TRUE(s.everValidFix());
    TEST_ASSERT_EQUAL_UINT32(50, s.secondsSinceValidFix());
}

static void test_fix_age_is_exact_across_the_wrap(void)
{
    GPSUpdateScheduling s;
    Time::setTestMillis(0xFFFFF000u);
    s.informSearching();
    Time::advanceTestMillis(0x1000u); // fix lands exactly on the wrap: stamp == 0, still valid
    s.informValidFix();
    Time::advanceTestMillis(9 * 1000);
    TEST_ASSERT_TRUE(s.everValidFix());
    TEST_ASSERT_EQUAL_UINT32(9, s.secondsSinceValidFix());
}

// The motion gate matrix. Semantics under test (see applyMotionGate):
//  - disabled or no fix since boot: stock remaining passes through, including an already-due 0;
//  - moving: early search only when the fix is older than the quiet window (a fix from before
//    the motion burst); otherwise the periodic cadence governs;
//  - quiet + fresh fix: postponed to at least kMotionRecheckMs;
//  - quiet + fix older than maxFixAgeSecs: stock cadence (refresh despite quiet).

static MotionGateInputs gateInputs(void)
{
    MotionGateInputs in;
    in.enabled = true;
    in.hasFixSinceBoot = true;
    in.secondsSinceValidFix = 60;
    in.secondsSinceMotion = UINT32_MAX; // quiet by default
    in.quietSecs = 300;
    in.maxFixAgeSecs = 900;
    return in;
}

static void test_gate_disabled_passes_stock_through(void)
{
    MotionGateInputs in = gateInputs();
    in.enabled = false;
    TEST_ASSERT_EQUAL_UINT32(0, applyMotionGate(0, in));
    TEST_ASSERT_EQUAL_UINT32(45000, applyMotionGate(45000, in));
}

static void test_gate_without_fix_since_boot_passes_stock_through(void)
{
    MotionGateInputs in = gateInputs();
    in.hasFixSinceBoot = false;
    TEST_ASSERT_EQUAL_UINT32(0, applyMotionGate(0, in));
    TEST_ASSERT_EQUAL_UINT32(45000, applyMotionGate(45000, in));
}

static void test_motion_with_premotion_fix_forces_immediate_search(void)
{
    MotionGateInputs in = gateInputs();
    in.secondsSinceMotion = 5;     // moving
    in.secondsSinceValidFix = 600; // fix predates the burst (>= quiet)
    TEST_ASSERT_EQUAL_UINT32(0, applyMotionGate(45000, in));
    TEST_ASSERT_EQUAL_UINT32(0, applyMotionGate(0, in));
}

static void test_continuous_motion_keeps_periodic_cadence(void)
{
    // Walking along: motion ongoing but the fix is recent (< quiet) - no forced search,
    // and no postponement either; the stock remaining (periodic interval) governs.
    MotionGateInputs in = gateInputs();
    in.secondsSinceMotion = 5;
    in.secondsSinceValidFix = 60;
    TEST_ASSERT_EQUAL_UINT32(45000, applyMotionGate(45000, in));
    TEST_ASSERT_EQUAL_UINT32(0, applyMotionGate(0, in));
}

static void test_quiet_with_fresh_fix_postpones(void)
{
    MotionGateInputs in = gateInputs(); // quiet, fix 60s old
    // Postponement only ever extends: below the floor it is raised to the floor...
    TEST_ASSERT_EQUAL_UINT32(kMotionRecheckMs, applyMotionGate(1000, in));
    TEST_ASSERT_EQUAL_UINT32(kMotionRecheckMs, applyMotionGate(0, in));
    // ...above the floor the stock remaining passes unchanged.
    TEST_ASSERT_EQUAL_UINT32(45000, applyMotionGate(45000, in));
    TEST_ASSERT_EQUAL_UINT32(60000, applyMotionGate(60000, in));
}

static void test_quiet_boundary_between_moving_and_still(void)
{
    MotionGateInputs in = gateInputs();
    in.secondsSinceMotion = 299; // still moving: younger than the window
    in.secondsSinceValidFix = 400;
    TEST_ASSERT_EQUAL_UINT32(0, applyMotionGate(45000, in));
    in.secondsSinceMotion = 300; // exactly the window: quiet now
    TEST_ASSERT_EQUAL_UINT32(45000, applyMotionGate(45000, in));
    TEST_ASSERT_EQUAL_UINT32(kMotionRecheckMs, applyMotionGate(1000, in));
}

static void test_quiet_with_stale_fix_falls_back_to_stock(void)
{
    MotionGateInputs in = gateInputs();
    in.secondsSinceValidFix = 901;                       // past maxFixAgeSecs
    TEST_ASSERT_EQUAL_UINT32(0, applyMotionGate(0, in)); // due: refresh despite quiet
    TEST_ASSERT_EQUAL_UINT32(45000, applyMotionGate(45000, in));
    in.secondsSinceValidFix = 900; // exactly fresh: boundary stays postponed
    TEST_ASSERT_EQUAL_UINT32(kMotionRecheckMs, applyMotionGate(1000, in));
}

static void test_never_moved_since_boot_counts_as_quiet(void)
{
    // Sensor present but silent since boot: quiet semantics, safe refresh at the age bound.
    MotionGateInputs in = gateInputs();
    in.secondsSinceMotion = UINT32_MAX;
    TEST_ASSERT_EQUAL_UINT32(kMotionRecheckMs, applyMotionGate(1000, in));
    in.secondsSinceValidFix = UINT32_MAX; // and no fix: first-fix rule path via hasFixSinceBoot
    in.hasFixSinceBoot = false;
    TEST_ASSERT_EQUAL_UINT32(45000, applyMotionGate(45000, in));
}

// The gate is the wake decision, not an adjunct: a due stock search (remaining 0) must be
// suppressed while quiet with a fresh fix, and only the gate's own 0 may wake.
static void test_gate_suppresses_an_already_due_stock_search(void)
{
    MotionGateInputs in = gateInputs(); // quiet, fix 60s old, fresh
    TEST_ASSERT_EQUAL_UINT32(kMotionRecheckMs, applyMotionGate(0, in));
    in.secondsSinceValidFix = 901; // stale: the due search goes through
    TEST_ASSERT_EQUAL_UINT32(0, applyMotionGate(0, in));
}

// Failed-search backoff stays in charge: motion with a stale fix must not retrigger doomed
// searches while consecutive failures are pending (indoor motion case).
static void test_pending_search_failures_block_the_motion_wake(void)
{
    MotionGateInputs in = gateInputs();
    in.secondsSinceMotion = 5;     // moving
    in.secondsSinceValidFix = 600; // pre-motion fix
    in.consecutiveSearchFailures = 1;
    TEST_ASSERT_EQUAL_UINT32(45000, applyMotionGate(45000, in));
    TEST_ASSERT_EQUAL_UINT32(0, applyMotionGate(0, in)); // stock due still wakes
    in.consecutiveSearchFailures = 0;
    TEST_ASSERT_EQUAL_UINT32(0, applyMotionGate(45000, in)); // forced wake returns
}

// Degenerate configurations fail toward stock rather than toward permanent suppression.
static void test_gate_default_inputs_pass_stock_through(void)
{
    MotionGateInputs in; // enabled=false; but even forcing the gate on, bounds fail safe:
    in.enabled = true;
    in.hasFixSinceBoot = true;
    in.secondsSinceMotion = 5;    // moving vs UINT32_MAX quiet default
    in.secondsSinceValidFix = 60; // but the fix is young: no force
    TEST_ASSERT_EQUAL_UINT32(45000, applyMotionGate(45000, in));
    in.secondsSinceMotion = UINT32_MAX; // quiet, and maxFixAgeSecs default 0: always stale
    TEST_ASSERT_EQUAL_UINT32(45000, applyMotionGate(45000, in));
    TEST_ASSERT_EQUAL_UINT32(0, applyMotionGate(0, in));
}

static void test_gate_degenerate_zero_quiet_window(void)
{
    MotionGateInputs in = gateInputs();
    in.quietSecs = 0; // nothing can be "moving": always quiet
    in.secondsSinceMotion = 0;
    TEST_ASSERT_EQUAL_UINT32(kMotionRecheckMs, applyMotionGate(1000, in)); // fresh fix postpones
    in.secondsSinceValidFix = 901;
    TEST_ASSERT_EQUAL_UINT32(0, applyMotionGate(0, in)); // stale fix still refreshes
}

static void test_gate_inverted_bounds_refresh_early(void)
{
    // maxFixAge < quiet: while quiet, the fix goes stale before the window ends, so the
    // honesty bound dominates and refresh searches happen at maxFixAge, not quiet.
    MotionGateInputs in = gateInputs();
    in.quietSecs = 300;
    in.maxFixAgeSecs = 60;
    in.secondsSinceValidFix = 30; // fresh
    TEST_ASSERT_EQUAL_UINT32(kMotionRecheckMs, applyMotionGate(1000, in));
    in.secondsSinceValidFix = 61; // stale already
    TEST_ASSERT_EQUAL_UINT32(0, applyMotionGate(0, in));
}

static void test_gate_floor_boundary_is_unchanged_at_the_floor(void)
{
    MotionGateInputs in = gateInputs();
    TEST_ASSERT_EQUAL_UINT32(kMotionRecheckMs, applyMotionGate(kMotionRecheckMs, in));
}

void setup()
{
    delay(10);
    initializeTestEnvironment();
    UNITY_BEGIN();
    RUN_TEST(test_matches_original_formula_at_sampled_points);
    RUN_TEST(test_zero_seconds_is_zero);
    RUN_TEST(test_monotonically_nondecreasing);
    RUN_TEST(test_exact_at_table_breakpoints);
    RUN_TEST(test_clamps_above_table_range);
    RUN_TEST(test_clamp_boundary);
    RUN_TEST(test_elapsed_is_zero_before_any_search);
    RUN_TEST(test_elapsed_tracks_the_clock_while_searching);
    RUN_TEST(test_elapsed_is_zero_once_the_search_ends);
    RUN_TEST(test_elapsed_is_exact_across_the_wrap);
    RUN_TEST(test_search_ending_after_the_wrap_reads_as_idle);
    RUN_TEST(test_search_starting_after_the_wrap_reads_as_searching);
    RUN_TEST(test_reset_clears_the_search_state);
    RUN_TEST(test_fix_age_is_never_before_first_fix);
    RUN_TEST(test_fix_age_tracks_time_since_valid_fix);
    RUN_TEST(test_fix_age_ignores_stamps_outside_a_search);
    RUN_TEST(test_fix_age_survives_reset_and_ages_on);
    RUN_TEST(test_fix_age_is_exact_across_the_wrap);
    RUN_TEST(test_gate_disabled_passes_stock_through);
    RUN_TEST(test_gate_without_fix_since_boot_passes_stock_through);
    RUN_TEST(test_motion_with_premotion_fix_forces_immediate_search);
    RUN_TEST(test_continuous_motion_keeps_periodic_cadence);
    RUN_TEST(test_quiet_with_fresh_fix_postpones);
    RUN_TEST(test_quiet_boundary_between_moving_and_still);
    RUN_TEST(test_quiet_with_stale_fix_falls_back_to_stock);
    RUN_TEST(test_never_moved_since_boot_counts_as_quiet);
    RUN_TEST(test_gate_suppresses_an_already_due_stock_search);
    RUN_TEST(test_pending_search_failures_block_the_motion_wake);
    RUN_TEST(test_gate_default_inputs_pass_stock_through);
    RUN_TEST(test_gate_degenerate_zero_quiet_window);
    RUN_TEST(test_gate_inverted_bounds_refresh_early);
    RUN_TEST(test_gate_floor_boundary_is_unchanged_at_the_floor);
    exit(UNITY_END());
}

void loop() {}
