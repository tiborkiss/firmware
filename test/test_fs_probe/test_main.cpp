// Tests for fsRootUsable() and the fsInit() hook that drives it - the boot-time probe that catches a
// filesystem which mounts successfully but cannot store anything (littlefs accepts a mount whose root
// metadata pair has been erased below the superblock; every save then fails silently and forever,
// observed twice on nRF52 field devices).
//
// The probe is the only code path in the mount sequence that exercises root create/write/read/delete,
// so these pin: the pass on a healthy root with no probe residue, the removal of a stale probe file
// left by an interrupted boot, the fail when the probe name is occupied by an entry that cannot be
// removed (the native stand-in for a corrupt root: an un-removable name plus an open-for-write
// rejection), and that fsInit() routes a failed probe into fsSelfHeal() and skips it otherwise.
//
// fsSelfHeal() is given a strong override here so the weak default in FSCommon.cpp (which would
// format the sandbox) never runs and the hook invocation becomes observable.
#include "TestUtil.h"
// FSCommon.h is what defines FSCom, so it has to be included before anything tests for it.
#include "FSCommon.h"
#include <cstdio>
#include <cstdlib>
#include <string>
#include <unistd.h>
#include <unity.h>

#ifdef FSCom

static const char *probePath = "/.fsprobe";

static int healCalls = 0;

// Strong definition wins over the weak default in FSCommon.cpp.
void fsSelfHeal()
{
    healCalls++;
}

// PortduinoFS::rmdir() is unlink() underneath and cannot remove a directory, so go to the host
// (same escape hatch as test_nodedb_save_retry's helper of the same shape).
static bool hostRmdir(const char *fsPath)
{
    std::string host = std::string(getenv("HOME")) + "/.portduino/default" + fsPath;
    return ::rmdir(host.c_str()) == 0;
}

// Clears both shapes the probe path can take: a plain stale file, or the planted phantom directory
// (with its content, which is what makes the directory un-removable).
static void clearProbePath(void)
{
    if (FSCom.exists(probePath)) {
        char inner[64];
        snprintf(inner, sizeof(inner), "%s/inner", probePath);
        FSCom.remove(inner);
        if (!FSCom.rmdir(probePath)) {
            hostRmdir(probePath);
            FSCom.remove(probePath);
        }
    }
}

// Plants a non-empty directory at the probe name: remove() fails on it and open-for-write rejects
// it, which is the failure shape of a corrupted littlefs root as seen through the host backend.
static void plantPhantom(void)
{
    clearProbePath();
    FSCom.mkdir(probePath);
    char inner[64];
    snprintf(inner, sizeof(inner), "%s/inner", probePath);
    File f = FSCom.open(inner, FILE_O_WRITE);
    TEST_ASSERT_TRUE((bool)f);
    f.write((uint8_t)'y');
    f.close();
}

// 1. Healthy root: passes, and leaves no residue behind.
void test_probe_passes_and_leaves_no_residue(void)
{
    clearProbePath();
    TEST_ASSERT_TRUE(fsRootUsable());
    TEST_ASSERT_FALSE(FSCom.exists(probePath));
}

// 2. A stale probe file from an interrupted boot is cleared and still yields a pass.
void test_probe_clears_stale_probe_file(void)
{
    clearProbePath();
    File f = FSCom.open(probePath, FILE_O_WRITE);
    TEST_ASSERT_TRUE((bool)f);
    f.write((uint8_t)'s');
    f.close();
    TEST_ASSERT_TRUE(FSCom.exists(probePath));

    TEST_ASSERT_TRUE(fsRootUsable());
    TEST_ASSERT_FALSE(FSCom.exists(probePath));
}

// 3. Probe name occupied by an un-removable entry: fails (this is the phantom-mount detection).
void test_probe_fails_on_unusable_root(void)
{
    plantPhantom();
    TEST_ASSERT_FALSE(fsRootUsable());
    clearProbePath();
    TEST_ASSERT_FALSE(FSCom.exists(probePath));
}

// 4. fsInit() routes a failing probe into the (here overridden) self-heal hook exactly once.
void test_fsinit_heals_unusable_root(void)
{
    plantPhantom();
    healCalls = 0;
    fsInit();
    TEST_ASSERT_EQUAL_INT(1, healCalls);
    clearProbePath();
}

// 5. A healthy root never triggers the hook.
void test_fsinit_skips_heal_when_healthy(void)
{
    clearProbePath();
    healCalls = 0;
    fsInit();
    TEST_ASSERT_EQUAL_INT(0, healCalls);
}

#endif // FSCom

void setup()
{
    initializeTestEnvironment();
    UNITY_BEGIN();
#ifdef FSCom
    RUN_TEST(test_probe_passes_and_leaves_no_residue);
    RUN_TEST(test_probe_clears_stale_probe_file);
    RUN_TEST(test_probe_fails_on_unusable_root);
    RUN_TEST(test_fsinit_heals_unusable_root);
    RUN_TEST(test_fsinit_skips_heal_when_healthy);
#endif
    exit(UNITY_END());
}

void loop() {}
