#include <unity.h>
#include "staging_probe_gate.h"

void setUp() {}
void tearDown() {}

void test_only_disconnected_idle_menu_can_begin_probe() {
    staging_probe::Gate gate;
    TEST_ASSERT_EQUAL_UINT32(0, gate.ticket()); // boot / homing
    gate.menuIdle(true);
    const auto idle = gate.ticket();
    TEST_ASSERT_TRUE(gate.current(idle));
    gate.bleConnected(true);
    TEST_ASSERT_TRUE(gate.menuIsIdle()); // BLE does not change the menu heartbeat
    TEST_ASSERT_FALSE(gate.current(idle));
    TEST_ASSERT_EQUAL_UINT32(0, gate.ticket());
    gate.menuIdle(false);
    gate.bleConnected(false);
    TEST_ASSERT_EQUAL_UINT32(0, gate.ticket()); // local motion
    gate.menuIdle(true);
    TEST_ASSERT_TRUE(gate.current(gate.ticket()));
    TEST_ASSERT_FALSE(gate.current(idle));
}

void test_rapid_entry_and_reconnect_cannot_send_or_credit_a_stale_probe() {
    staging_probe::Gate gate;
    gate.menuIdle(true);
    const auto started = gate.ticket();
    gate.menuIdle(false);
    gate.menuIdle(true);
    TEST_ASSERT_FALSE(gate.current(started));
    const auto next = gate.ticket();
    gate.bleConnected(true);
    gate.bleConnected(true); // second observer joins
    gate.bleConnected(true); // one remains
    TEST_ASSERT_EQUAL_UINT32(0, gate.ticket());
    gate.bleConnected(false);
    TEST_ASSERT_FALSE(gate.current(next));
    const auto idle = gate.ticket();
    gate.menuIdle(true);
    TEST_ASSERT_TRUE(gate.current(idle));
}

void test_inflight_handshake_drops_on_owner_thread_before_http_can_send() {
    staging_probe::Gate gate;
    gate.menuIdle(true);
    int opens = 0, closes = 0;
    auto ticket = gate.ticket();
    const int result = gate.connect(ticket, [&] {
        ++opens;
        gate.menuIdle(false);
        gate.menuIdle(true);
        return 1; // TLS connected successfully after the user changed mode
    }, [&] { ++closes; });
    TEST_ASSERT_EQUAL_INT(0, result);
    TEST_ASSERT_EQUAL_INT(1, opens);
    TEST_ASSERT_EQUAL_INT(1, closes);
    TEST_ASSERT_FALSE(gate.current(ticket));
    TEST_ASSERT_EQUAL_INT(0, gate.connect(ticket, [&] { return ++opens; }, [&] { ++closes; }));
    TEST_ASSERT_EQUAL_INT(1, opens);
    ticket = gate.ticket();
    TEST_ASSERT_EQUAL_INT(1, gate.connect(ticket, [] { return 1; }, [&] { ++closes; }));
    TEST_ASSERT_EQUAL_INT(1, closes);
}

void test_probe_cadence_uses_last_actual_attempt_and_survives_uptime_wrap() {
    TEST_ASSERT_TRUE(staging_probe::due(0, 5000));
    TEST_ASSERT_FALSE(staging_probe::due(100, 30099));
    TEST_ASSERT_TRUE(staging_probe::due(100, 30100));
    const uint32_t last = UINT32_MAX - 20000;
    TEST_ASSERT_FALSE(staging_probe::due(last, 9998));
    TEST_ASSERT_TRUE(staging_probe::due(last, 9999));
}
int main() {
    UNITY_BEGIN();
    RUN_TEST(test_only_disconnected_idle_menu_can_begin_probe);
    RUN_TEST(test_rapid_entry_and_reconnect_cannot_send_or_credit_a_stale_probe);
    RUN_TEST(test_inflight_handshake_drops_on_owner_thread_before_http_can_send);
    RUN_TEST(test_probe_cadence_uses_last_actual_attempt_and_survives_uptime_wrap);
    return UNITY_END();
}
