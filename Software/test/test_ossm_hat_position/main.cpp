#include <unity.h>
#include <vector>
#include <climits>
#include "../../src/services/ossmHatPositionProtocol.hpp"
#include "../../src/services/ossmHatProtocol.hpp"
#include "../../src/services/ossmHatTxAudit.hpp"

using namespace ossm_hat;

void setUp() {}
void tearDown() {}

namespace {
void crc(std::vector<uint8_t> &frame) {
    const auto value = AimFrame::crc16(frame.data(), frame.size());
    frame.push_back(value & 255);
    frame.push_back(value >> 8);
}

std::vector<uint8_t> controls(uint16_t mode = 1, uint16_t output = 0,
                              uint16_t rpm = 37, uint16_t accel = 1500) {
    std::vector<uint8_t> frame{1, 3, 8};
    for (const auto word : {mode, output, rpm, accel}) {
        frame.push_back(word >> 8);
        frame.push_back(word & 255);
    }
    crc(frame);
    return frame;
}

std::vector<uint8_t> positionReply(int32_t current) {
    const uint32_t bits = static_cast<uint32_t>(current);
    std::vector<uint8_t> frame{1, 0x7B, static_cast<uint8_t>(bits >> 8),
        static_cast<uint8_t>(bits), static_cast<uint8_t>(bits >> 24),
        static_cast<uint8_t>(bits >> 16)};
    crc(frame);
    return frame;
}

bool acknowledge(PositionQueue &queue, const PositionAction &action, uint32_t now) {
    std::vector<uint8_t> response;
    if (action.readsControls()) {
        response = controls(1, action.kind == PositionActionKind::VerifyEnable ? 1 : 0);
    } else if (action.function == AimFrame::fcWriteSingle) {
        response.assign(action.frame, action.frame + action.frameLength);
    } else response = positionReply(action.targetCounts);
    return queue.complete(action, response.data(), response.size(), now);
}

void prepare(PositionQueue &queue, uint64_t session = 17, int32_t anchor = -1234) {
    TEST_ASSERT_TRUE(queue.prepare(session, anchor, 37, 1500, 100));
    for (uint32_t i = 0; i < 5; ++i) {
        PositionAction action;
        TEST_ASSERT_TRUE(queue.take(101 + i * 5, action));
        TEST_ASSERT_FALSE(action.motion());
        TEST_ASSERT_TRUE(acknowledge(queue, action, 102 + i * 5));
    }
    TEST_ASSERT_TRUE(queue.status().prepared);
    TEST_ASSERT_FALSE(queue.status().enabled);
}

void enable(PositionQueue &queue, uint64_t session = 17, uint32_t now = 200) {
    TEST_ASSERT_TRUE(queue.enable(session, now));
    PositionAction action;
    TEST_ASSERT_TRUE(queue.take(now, action));
    TEST_ASSERT_EQUAL(static_cast<int>(PositionActionKind::Enable), static_cast<int>(action.kind));
    TEST_ASSERT_TRUE(acknowledge(queue, action, now + 1));
    TEST_ASSERT_FALSE(queue.status().enabled);  // echo alone never proves enable
    TEST_ASSERT_TRUE(queue.take(now + 2, action));
    TEST_ASSERT_TRUE(acknowledge(queue, action, now + 3));
    TEST_ASSERT_TRUE(queue.status().enabled);
}
}

void test_absolute_manual_frame_and_word_swapped_positive_ack() {
    const uint8_t expected[] = {1, 0x7B, 0, 0, 0x27, 0x10, 0xFF, 0xFC};
    uint8_t frame[8];
    TEST_ASSERT_EQUAL_UINT(8, AimFrame::buildMoveAbsolute(frame, 1, 10000));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, frame, 8);
    const uint8_t ack[] = {1, 0x7B, 0x27, 0x10, 0, 0, 0xEE, 0xB1};
    const auto parsed = AimFrame::parseReply(ack, sizeof(ack), 1, 0x7B, 8);
    TEST_ASSERT_EQUAL(static_cast<int>(AimFrame::Status::Ok), static_cast<int>(parsed.status));
    TEST_ASSERT_EQUAL_INT32(10000, AimFrame::moveReplyValue(ack));
}

void test_absolute_reply_current_word_can_differ_from_correlated_intended_target() {
    // Generic fixed vectors, not fixture recordings. The request is MSB-first;
    // the immediate current-position response has the low word first.
    const uint8_t requests[][8] = {
        {1, 0x7B, 0, 0, 0x27, 0x10, 0xFF, 0xFC},
        {1, 0x7B, 0xFF, 0xFF, 0xB1, 0xE0, 0x90, 0x3C}};
    const uint8_t responses[][8] = {
        {1, 0x7B, 0x09, 0xC4, 0, 0, 0xA7, 0xA1},
        {1, 0x7B, 0xAD, 0xF8, 0xFF, 0xFF, 0x45, 0x2D}};
    const int32_t targets[] = {10000, -20000};
    const int32_t currents[] = {2500, -21000};
    for (unsigned i = 0; i < 2; ++i) {
        PositionQueue queue;
        prepare(queue, 17, 0);
        enable(queue);
        TEST_ASSERT_TRUE(queue.queue(17, 41, targets[i], 37, 1500, 300, 100));
        PositionAction action;
        TEST_ASSERT_TRUE(queue.take(300, action));
        TEST_ASSERT_EQUAL_HEX8_ARRAY(requests[i], action.frame, 8);
        TEST_ASSERT_TRUE(ownedPositionReply(action, responses[i], 8));
        TEST_ASSERT_TRUE(queue.complete(action, responses[i], 8, 301));
        const auto state = queue.status();
        TEST_ASSERT_FALSE(state.faulted);
        TEST_ASSERT_TRUE(state.enabled);
        TEST_ASSERT_FALSE(state.pending);
        TEST_ASSERT_EQUAL_UINT32(41, state.acknowledgedRequestId);
        TEST_ASSERT_EQUAL_INT32(targets[i], state.acknowledgedTargetCounts);
        TEST_ASSERT_EQUAL_INT32(currents[i], state.vendorReplyValue);
        TEST_ASSERT_TRUE(state.vendorReplyAvailable);
        TEST_ASSERT_EQUAL_UINT32(301, state.vendorReplyAtMs);
    }
}

void test_absolute_response_never_refreshes_coherent_physical_telemetry() {
    Decoder observer;
    observer.expectRead(1, telemetryStart, telemetryCount, 100);
    std::vector<uint8_t> physical{1, 3, 20};
    const uint16_t words[] = {0x10, 250, 0, 7848, 0, 0, 0, 0, 1234, 0};
    for (const uint16_t word : words) {
        physical.push_back(word >> 8);
        physical.push_back(word & 255);
    }
    crc(physical);
    for (const uint8_t byte : physical) observer.push(byte, 101);
    const auto before = observer.sample();
    TEST_ASSERT_EQUAL_UINT32(1, before.sequence);
    const uint8_t response[] = {1, 0x7B, 0x09, 0xC4, 0, 0, 0xA7, 0xA1};
    uint8_t events = 0;
    for (const uint8_t byte : response) events |= observer.push(byte, 401);
    // Outside an explicitly owned response window, a valid same-shaped frame
    // still establishes external ownership. It cannot stand in for FC03 data.
    TEST_ASSERT_TRUE(events & Decoder::ExternalRequest);
    TEST_ASSERT_FALSE(events & Decoder::Telemetry);
    TEST_ASSERT_EQUAL_UINT32(before.sequence, observer.sample().sequence);
    TEST_ASSERT_EQUAL_UINT32(before.updatedAtMs, observer.sample().updatedAtMs);
    TEST_ASSERT_EQUAL_INT32(1234, observer.sample().positionCounts);
    TEST_ASSERT_EQUAL_FLOAT(before.currentAmps, observer.sample().currentAmps);
    TEST_ASSERT_EQUAL_FLOAT(before.voltageVolts, observer.sample().voltageVolts);
    TEST_ASSERT_EQUAL_UINT16(before.alarm, observer.sample().alarm);
    TEST_ASSERT_FALSE(fresh(observer.sample(), 401));
}

void test_fc16_incremental_vectors_have_low_word_first_and_are_never_targets() {
    const uint16_t plus[] = {0x0FA0, 0};
    const uint16_t minus[] = {0xF060, 0xFFFF};
    const uint8_t expectedPlus[] = {1, 0x10, 0, 0x0C, 0, 2, 4, 0x0F, 0xA0, 0, 0, 0xF0, 0xCC};
    const uint8_t expectedMinus[] = {1, 0x10, 0, 0x0C, 0, 2, 4, 0xF0, 0x60, 0xFF, 0xFF, 0xC1, 0x54};
    uint8_t frame[13];
    TEST_ASSERT_EQUAL_UINT(13, AimFrame::buildWriteMultiple(frame, 1, 0x0C, plus, 2));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expectedPlus, frame, 13);
    TEST_ASSERT_EQUAL_UINT(13, AimFrame::buildWriteMultiple(frame, 1, 0x0C, minus, 2));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expectedMinus, frame, 13);
    TEST_ASSERT_EQUAL_UINT32(32768, encoderCountsPerRevolution);
}

void test_prepare_has_only_disabled_hold_ram_setup_and_verified_readback() {
    PositionQueue queue;
    TEST_ASSERT_TRUE(queue.prepare(17, -1234, 37, 1500, 100));
    const PositionActionKind expected[] = {PositionActionKind::ReadInitial,
        PositionActionKind::Hold, PositionActionKind::Speed,
        PositionActionKind::Acceleration, PositionActionKind::VerifySetup};
    for (uint32_t i = 0; i < 5; ++i) {
        PositionAction action;
        TEST_ASSERT_TRUE(queue.take(101 + i * 5, action));
        TEST_ASSERT_EQUAL(static_cast<int>(expected[i]), static_cast<int>(action.kind));
        TEST_ASSERT_NOT_EQUAL(AimFrame::fcWriteMultiple, action.function);
        if (action.function == AimFrame::fcWriteSingle)
            TEST_ASSERT_TRUE(AimFrame::be16(action.frame + 2) == 2 ||
                             AimFrame::be16(action.frame + 2) == 3);
        TEST_ASSERT_TRUE(acknowledge(queue, action, 102 + i * 5));
    }
    TEST_ASSERT_TRUE(queue.status().prepared);
    TEST_ASSERT_FALSE(queue.status().enabled);
    PositionAction none;
    TEST_ASSERT_FALSE(queue.take(150, none));  // no automatic output enable
    TEST_ASSERT_FALSE(queue.queue(17, 1, 0, 37, 1500, 150, 100));
}

void test_prepare_refuses_live_output_mode0_and_wrong_settings_readback() {
    for (unsigned scenario = 0; scenario < 3; ++scenario) {
        PositionQueue queue;
        TEST_ASSERT_TRUE(queue.prepare(17, 0, 37, 1500, 0));
        PositionAction action;
        TEST_ASSERT_TRUE(queue.take(0, action));
        const auto reply = controls(scenario == 0 ? 0 : 1, scenario == 1 ? 1 : 0);
        if (scenario < 2) {
            TEST_ASSERT_FALSE(queue.complete(action, reply.data(), reply.size(), 1));
        } else {
            TEST_ASSERT_TRUE(queue.complete(action, reply.data(), reply.size(), 1));
            for (int i = 0; i < 3; ++i) {
                TEST_ASSERT_TRUE(queue.take(2 + i * 2, action));
                TEST_ASSERT_TRUE(acknowledge(queue, action, 3 + i * 2));
            }
            TEST_ASSERT_TRUE(queue.take(10, action));
            const auto wrong = controls(1, 0, 38, 1500);
            TEST_ASSERT_FALSE(queue.complete(action, wrong.data(), wrong.size(), 11));
        }
        TEST_ASSERT_TRUE(queue.status().faulted);
        TEST_ASSERT_FALSE(queue.status().prepared);
        TEST_ASSERT_EQUAL(static_cast<int>(OssmHatPositionFault::ControlMismatch),
                          static_cast<int>(queue.status().fault));
    }
}

void test_enable_needs_explicit_request_exact_echo_and_readback() {
    PositionQueue queue;
    prepare(queue);
    enable(queue);
    TEST_ASSERT_FALSE(queue.enable(17, 205));
    TEST_ASSERT_FALSE(queue.status().pending);
}

void test_enable_readback_off_cannot_claim_output_enabled() {
    PositionQueue queue;
    prepare(queue);
    TEST_ASSERT_TRUE(queue.enable(17, 200));
    PositionAction action;
    TEST_ASSERT_TRUE(queue.take(200, action));
    TEST_ASSERT_TRUE(acknowledge(queue, action, 201));
    TEST_ASSERT_TRUE(queue.take(202, action));
    const auto off = controls();
    TEST_ASSERT_FALSE(queue.complete(action, off.data(), off.size(), 203));
    TEST_ASSERT_FALSE(queue.status().enabled);
    TEST_ASSERT_TRUE(queue.status().faulted);
}

void test_coalesces_latest_target_without_replaying_a_fifo() {
    PositionQueue queue;
    prepare(queue);
    enable(queue);
    TEST_ASSERT_TRUE(queue.queue(17, 1, 1000, 37, 1500, 300, 100));
    TEST_ASSERT_TRUE(queue.queue(17, 2, -4000, 37, 1500, 301, 100));
    PositionAction action;
    TEST_ASSERT_TRUE(queue.take(302, action));
    TEST_ASSERT_EQUAL_UINT32(2, action.requestId);
    TEST_ASSERT_EQUAL_INT32(-4000, action.targetCounts);
    TEST_ASSERT_EQUAL_UINT8(AimFrame::fcMoveAbsolute, action.function);
    TEST_ASSERT_TRUE(acknowledge(queue, action, 303));
    TEST_ASSERT_EQUAL_UINT32(2, queue.status().acknowledgedRequestId);
    TEST_ASSERT_FALSE(queue.status().pending);
    TEST_ASSERT_FALSE(queue.take(330, action));
}

void test_old_inflight_ack_does_not_clear_a_newer_pending_target() {
    PositionQueue queue;
    prepare(queue);
    enable(queue);
    TEST_ASSERT_TRUE(queue.queue(17, 1, 100, 37, 1500, 300, 100));
    PositionAction old;
    TEST_ASSERT_TRUE(queue.take(300, old));
    TEST_ASSERT_TRUE(queue.queue(17, 2, 200, 37, 1500, 301, 100));
    const auto currentReply = positionReply(90);
    TEST_ASSERT_TRUE(queue.complete(old, currentReply.data(), currentReply.size(), 302));
    TEST_ASSERT_EQUAL_UINT32(1, queue.status().acknowledgedRequestId);
    TEST_ASSERT_EQUAL_INT32(100, queue.status().acknowledgedTargetCounts);
    TEST_ASSERT_EQUAL_INT32(90, queue.status().vendorReplyValue);
    TEST_ASSERT_TRUE(queue.status().pending);
    PositionAction next;
    TEST_ASSERT_FALSE(queue.take(319, next));  // at most one target per20ms
    TEST_ASSERT_TRUE(queue.take(320, next));
    TEST_ASSERT_EQUAL_UINT32(2, next.requestId);
    TEST_ASSERT_EQUAL_INT32(200, next.targetCounts);
}

void test_cancel_then_new_session_ignores_old_ack_and_forbids_old_intent() {
    PositionQueue queue;
    prepare(queue);
    enable(queue);
    TEST_ASSERT_TRUE(queue.queue(17, 1, 100, 37, 1500, 300, 100));
    PositionAction old;
    TEST_ASSERT_TRUE(queue.take(300, old));
    queue.cancel();
    TEST_ASSERT_FALSE(queue.enable(17, 301));
    TEST_ASSERT_FALSE(queue.queue(17, 2, 200, 37, 1500, 301, 100));
    TEST_ASSERT_FALSE(queue.prepare(17, 0, 37, 1500, 301));
    TEST_ASSERT_TRUE(queue.prepare(18, 0, 37, 1500, 302));
    TEST_ASSERT_FALSE(acknowledge(queue, old, 303));
    TEST_ASSERT_EQUAL_UINT64(18, queue.status().session);
    TEST_ASSERT_FALSE(queue.status().faulted);
    TEST_ASSERT_FALSE(queue.status().enabled);
}

void test_expired_target_cancels_before_any_move_and_retains_first_fault() {
    PositionQueue queue;
    prepare(queue);
    enable(queue);
    TEST_ASSERT_TRUE(queue.queue(17, 1, 100, 37, 1500, 300, 50));
    PositionAction action;
    TEST_ASSERT_FALSE(queue.take(351, action));
    TEST_ASSERT_EQUAL(static_cast<int>(OssmHatPositionFault::Expired),
                      static_cast<int>(queue.status().fault));
    queue.cancel();
    TEST_ASSERT_EQUAL(static_cast<int>(OssmHatPositionFault::Expired),
                      static_cast<int>(queue.status().fault));
    TEST_ASSERT_FALSE(queue.status().enabled);
}

void test_live_configuration_changes_zero_sessions_and_duplicate_ids_are_rejected() {
    PositionQueue queue;
    TEST_ASSERT_FALSE(queue.prepare(0, 0, 37, 1500, 0));
    TEST_ASSERT_FALSE(queue.prepare(17, 0, 3001, 1500, 0));
    TEST_ASSERT_FALSE(queue.prepare(17, 0, 37, 60001, 0));
    prepare(queue);
    enable(queue);
    TEST_ASSERT_FALSE(queue.queue(0, 1, 100, 37, 1500, 300, 100));
    TEST_ASSERT_FALSE(queue.queue(17, 1, 100, 38, 1500, 300, 100));
    TEST_ASSERT_FALSE(queue.queue(17, 1, 100, 37, 60000, 300, 100));
    TEST_ASSERT_FALSE(queue.queue(17, 0, 100, 37, 1500, 300, 100));
    TEST_ASSERT_TRUE(queue.queue(17, 1, 100, 37, 1500, 300, 100));
    TEST_ASSERT_FALSE(queue.queue(17, 1, 200, 37, 1500, 301, 100));
}

void test_bad_crc_wrong_unit_function_length_and_fc06_echo_are_rejected() {
    for (unsigned scenario = 0; scenario < 4; ++scenario) {
        PositionQueue queue;
        prepare(queue);
        enable(queue);
        TEST_ASSERT_TRUE(queue.queue(17, 1, 10000, 37, 1500, 300, 100));
        PositionAction action;
        TEST_ASSERT_TRUE(queue.take(300, action));
        auto reply = positionReply(2500);
        if (scenario == 0) reply.back() ^= 1;
        if (scenario == 1) { reply.resize(6); reply[0] = 2; crc(reply); }
        if (scenario == 2) { reply.resize(6); reply[1] = 0x78; crc(reply); }
        if (scenario == 3) reply.push_back(0x55);
        TEST_ASSERT_FALSE(queue.complete(action, reply.data(), reply.size(), 301));
        TEST_ASSERT_TRUE(queue.status().faulted);
        TEST_ASSERT_FALSE(queue.status().enabled);
        TEST_ASSERT_EQUAL_UINT32(0, queue.status().acknowledgedRequestId);
        // A corrupt/unrelated reply must not overwrite the earlier valid
        // disabled-hold response retained for diagnostics.
        TEST_ASSERT_TRUE(queue.status().vendorReplyAvailable);
        TEST_ASSERT_EQUAL_INT32(-1234, queue.status().vendorReplyValue);
        TEST_ASSERT_EQUAL_UINT32(107, queue.status().vendorReplyAtMs);
    }
    PositionQueue queue;
    prepare(queue);
    TEST_ASSERT_TRUE(queue.enable(17, 200));
    PositionAction enableAction;
    TEST_ASSERT_TRUE(queue.take(200, enableAction));
    uint8_t wrongEcho[8];
    AimFrame::buildWriteSingle(wrongEcho, 1, 1, 0);
    TEST_ASSERT_FALSE(queue.complete(enableAction, wrongEcho, 8, 201));
}

void test_signed_count_boundaries_serialize_without_overflow() {
    PositionQueue queue;
    prepare(queue);
    enable(queue);
    for (unsigned i = 0; i < 2; ++i) {
        const int32_t target = i == 0 ? INT32_MIN : INT32_MAX;
        TEST_ASSERT_TRUE(queue.queue(17, i + 1, target, 37, 1500, 300 + 20 * i, 100));
        PositionAction action;
        TEST_ASSERT_TRUE(queue.take(300 + 20 * i, action));
        TEST_ASSERT_EQUAL_UINT8(0x7B, action.frame[1]);
        auto reply = positionReply(target);
        reply.insert(reply.begin(), 0);
        TEST_ASSERT_TRUE(queue.complete(action, reply.data(), reply.size(), 301 + 20 * i));
        TEST_ASSERT_EQUAL_INT32(target, queue.status().acknowledgedTargetCounts);
    }
}

void test_late_current_position_after_cancel_is_recognized_for_priority_stop_drain() {
    PositionQueue queue;
    prepare(queue);
    enable(queue);
    TEST_ASSERT_TRUE(queue.queue(17, 1, -4000, 37, 1500, 300, 100));
    PositionAction action;
    TEST_ASSERT_TRUE(queue.take(300, action));
    LatePositionReply late;
    TEST_ASSERT_TRUE(late.expect(action, action.frameLength, 300));
    auto reply = positionReply(-4100);  // current word differs from old target
    for (unsigned i = 0; i < 4; ++i) TEST_ASSERT_FALSE(late.push(reply[i], 301));
    queue.cancel();
    for (unsigned i = 4; i < 7; ++i) TEST_ASSERT_FALSE(late.push(reply[i], 302));
    TEST_ASSERT_TRUE(late.push(reply[7], 302));
    TEST_ASSERT_FALSE(acknowledge(queue, action, 303));  // never revives intent
    TEST_ASSERT_FALSE(late.push(reply[7], 303));
}

void test_retained_reply_rejects_wrong_unit_function_crc_and_expired_window() {
    PositionAction action;
    action.function = 0x7B;
    action.replyLength = 8;
    action.targetCounts = 100;
    action.frameLength = AimFrame::buildMoveAbsolute(action.frame, 1, action.targetCounts);
    LatePositionReply late;
    for (unsigned scenario = 0; scenario < 3; ++scenario) {
        TEST_ASSERT_TRUE(late.expect(action, action.frameLength, 300));
        auto wrong = positionReply(25);
        if (scenario == 0) { wrong.resize(6); wrong[0] = 2; crc(wrong); }
        if (scenario == 1) { wrong.resize(6); wrong[1] = 0x78; crc(wrong); }
        if (scenario == 2) wrong.back() ^= 1;
        for (const auto byte : wrong) TEST_ASSERT_FALSE(late.push(byte, 301));
    }
    TEST_ASSERT_TRUE(late.expect(action, action.frameLength, 300));
    const auto right = positionReply(25);
    for (const auto byte : right) TEST_ASSERT_FALSE(late.push(byte, 401));
}

void test_late_current_word_has_one_bounded_owner_window_and_never_hides_a_later_master() {
    PositionAction action;
    action.function = AimFrame::fcMoveAbsolute;
    action.replyLength = 8;
    action.targetCounts = 10000;
    action.frameLength = AimFrame::buildMoveAbsolute(action.frame, 1, action.targetCounts);
    LatePositionReply late;
    Decoder observer;
    const uint8_t response[] = {0, 1, 0x7B, 0x09, 0xC4, 0, 0, 0xA7, 0xA1};
    TEST_ASSERT_TRUE(late.expect(action, action.frameLength, 300));
    bool external = false;
    for (const uint8_t byte : response) {
        const bool own = late.push(byte, 301);
        if ((observer.push(byte, 301) & Decoder::ExternalRequest) && !own) external = true;
    }
    TEST_ASSERT_FALSE(external);
    // The committed frame's reply was consumed once. Another frame, even
    // identical, is no longer in an owned window and remains foreign traffic.
    for (const uint8_t byte : response) {
        const bool own = late.push(byte, 302);
        if ((observer.push(byte, 302) & Decoder::ExternalRequest) && !own) external = true;
    }
    TEST_ASSERT_TRUE(external);
}

void test_retained_fc06_reply_still_needs_the_exact_register_value_echo() {
    PositionAction action;
    action.function = AimFrame::fcWriteSingle;
    action.replyLength = 8;
    action.frameLength = AimFrame::buildWriteSingle(action.frame, 1, 1, 1);
    LatePositionReply late;
    TEST_ASSERT_TRUE(late.expect(action, action.frameLength, 300));
    uint8_t wrong[8];
    AimFrame::buildWriteSingle(wrong, 1, 1, 0);
    for (const uint8_t byte : wrong) TEST_ASSERT_FALSE(late.push(byte, 301));
    TEST_ASSERT_TRUE(late.expect(action, action.frameLength, 300));
    for (unsigned i = 0; i < 7; ++i) TEST_ASSERT_FALSE(late.push(action.frame[i], 301));
    TEST_ASSERT_TRUE(late.push(action.frame[7], 301));
}

void test_short_uart_write_cannot_open_or_retain_an_owned_reply_window() {
    PositionAction action;
    action.function = AimFrame::fcMoveAbsolute;
    action.replyLength = 8;
    action.frameLength = AimFrame::buildMoveAbsolute(action.frame, 1, 10000);
    const uint8_t response[] = {1, 0x7B, 0x09, 0xC4, 0, 0, 0xA7, 0xA1};
    for (size_t accepted = 0; accepted < action.frameLength; ++accepted) {
        LatePositionReply late;
        TEST_ASSERT_TRUE(late.expect(action, action.frameLength, 300));
        TEST_ASSERT_FALSE(late.expect(action, accepted, 301));
        Decoder observer;
        uint8_t events = 0;
        for (const uint8_t byte : response) {
            TEST_ASSERT_FALSE(late.push(byte, 302));
            events |= observer.push(byte, 302);
        }
        TEST_ASSERT_TRUE(events & Decoder::ExternalRequest);
        TEST_ASSERT_FALSE(late.incomplete(302));
    }
    LatePositionReply late;
    action.replyLength = 0;
    TEST_ASSERT_FALSE(late.expect(action, action.frameLength, 300));
}

void test_split_leading_zero_late_current_reply_does_not_abort_priority_stop_echo() {
    PositionAction action;
    action.function = AimFrame::fcMoveAbsolute;
    action.replyLength = 8;
    action.frameLength = AimFrame::buildMoveAbsolute(action.frame, 1, 10000);
    LatePositionReply late;
    TEST_ASSERT_TRUE(late.expect(action, action.frameLength, 300));
    Decoder observer;
    const uint8_t oldReply[] = {0, 1, 0x7B, 0x09, 0xC4, 0, 0, 0xA7, 0xA1};
    uint8_t response[80] = {};
    size_t received = 0;
    for (unsigned i = 0; i < 8; ++i) {
        response[received++] = oldReply[i];
        TEST_ASSERT_FALSE(late.push(oldReply[i], 301));
        TEST_ASSERT_FALSE(observer.push(oldReply[i], 301) & Decoder::ExternalRequest);
    }
    const auto partial = AimFrame::parseReply(response, received, 1, AimFrame::fcWriteSingle, 8);
    TEST_ASSERT_EQUAL(static_cast<int>(AimFrame::Status::BadReply), static_cast<int>(partial.status));
    TEST_ASSERT_TRUE(late.incomplete(302));
    TEST_ASSERT_TRUE(stopReplyCanContinue(partial.status, received, late, 302));
    TEST_ASSERT_FALSE(stopReplyCanContinue(partial.status, received, late, 401));

    response[received++] = oldReply[8];  // fragmented final CRC arrives later
    TEST_ASSERT_TRUE(late.push(oldReply[8], 303));
    TEST_ASSERT_TRUE(observer.push(oldReply[8], 303) & Decoder::ExternalRequest);
    // This complete retained reply is consumed, so stopExchange clears its
    // response buffer and continues waiting for the exact stop FC06 echo.
    received = 0;
    uint8_t stop[8];
    buildSerialOutputDisable(stop);
    for (unsigned i = 0; i < sizeof(stop); ++i) {
        response[received++] = stop[i];
        TEST_ASSERT_FALSE(late.push(stop[i], 304));
        const auto parsed = AimFrame::parseReply(response, received, 1, AimFrame::fcWriteSingle, 8);
        const bool ownEcho = parsed.status == AimFrame::Status::Ok &&
            std::memcmp(response + parsed.offset, stop, 6) == 0;
        const uint8_t events = observer.push(stop[i], 304);
        TEST_ASSERT_FALSE((events & Decoder::ExternalRequest) && !ownEcho);
        if (i < sizeof(stop) - 1) {
            TEST_ASSERT_TRUE(stopReplyCanContinue(parsed.status, received, late, 304));
        } else TEST_ASSERT_TRUE(ownEcho);
    }
}

void test_stop_waits_for_discovery_and_has_a_bounded_rollover_safe_failure() {
    TEST_ASSERT_FALSE(deferredStopReady(true, 101, 100, false, true, false, false));
    TEST_ASSERT_FALSE(deferredStopReady(true, 1099, 100, false, false, false, false));
    TEST_ASSERT_TRUE(deferredStopReady(true, 1100, 100, false, true, true, true));
    TEST_ASSERT_TRUE(deferredStopReady(true, 10100, 100, false, false, false, false));
    TEST_ASSERT_TRUE(deferredStopReady(true, 101, 100, true, true, false, false));
    TEST_ASSERT_FALSE(deferredStopReady(false, 10100, 100, true, true, true, true));
    TEST_ASSERT_FALSE(deferredStopReady(true, 0x10, 0xFFFFFFF0, false, true, false, false));
    TEST_ASSERT_TRUE(deferredStopReady(true, 0x3000, 0xFFFFFFF0, false, true, false, false));
}

void test_repeated_enable_request_does_not_extend_expired_intent() {
    PositionQueue queue;
    prepare(queue);
    TEST_ASSERT_TRUE(queue.enable(17, 200));
    TEST_ASSERT_TRUE(queue.enable(17, 299));
    PositionAction action;
    TEST_ASSERT_FALSE(queue.take(301, action));
    TEST_ASSERT_TRUE(queue.status().faulted);
    TEST_ASSERT_EQUAL(static_cast<int>(OssmHatPositionFault::Expired),
                      static_cast<int>(queue.status().fault));
}

void test_stop_never_switches_mode_or_claims_mode0_is_serial_disabled() {
    uint8_t frame[8];
    TEST_ASSERT_EQUAL_UINT(8, buildSerialOutputDisable(frame));
    const uint8_t expected[] = {1, 6, 0, 1, 0, 0, 0xD8, 0x0A};
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, frame, 8);
    TEST_ASSERT_FALSE(serialOutputDisabled(0, 0));
    TEST_ASSERT_FALSE(serialOutputDisabled(0, 1));
    TEST_ASSERT_FALSE(serialOutputDisabled(1, 1));
    TEST_ASSERT_TRUE(serialOutputDisabled(1, 0));
    TEST_ASSERT_TRUE(serialOutputDisabled(1, 6));  // unrelated status bits
}

void test_cancel_after_committed_enable_cannot_complete_or_queue_after_stop() {
    PositionQueue queue;
    prepare(queue);
    TEST_ASSERT_TRUE(queue.enable(17, 200));
    PositionAction committed;
    TEST_ASSERT_TRUE(queue.take(200, committed));
    TEST_ASSERT_TRUE(queue.current(committed));  // final owner check before TX
    queue.cancel();  // cancellation can occur after this commit point
    // The sole owner can finish that precommitted frame before its stop frame.
    // Its late ACK cannot revive output state, enqueue another frame, or
    // acknowledge a later session after the owner's verified stop completes.
    TEST_ASSERT_FALSE(acknowledge(queue, committed, 201));
    PositionAction afterStop;
    TEST_ASSERT_FALSE(queue.take(202, afterStop));
    TEST_ASSERT_FALSE(queue.enable(17, 202));
    TEST_ASSERT_TRUE(queue.prepare(18, -1234, 37, 1500, 203));
    TEST_ASSERT_FALSE(acknowledge(queue, committed, 204));
    TEST_ASSERT_FALSE(queue.status().enabled);
    TEST_ASSERT_EQUAL_UINT64(18, queue.status().session);
}

void test_stop_reassertion_needs_restored_fresh_sole_link_and_retains_success() {
    OssmHatStopStatus stop;
    stop.requested = true;
    stop.complete = true;
    stop.completedAtMs = 100;
    TEST_ASSERT_FALSE(retrySerialStopNeeded(stop, 1099, true));
    TEST_ASSERT_TRUE(retrySerialStopNeeded(stop, 1100, true));
    TEST_ASSERT_FALSE(retrySerialStopNeeded(stop, 1100, false));
    stop.passiveBlocked = true;
    TEST_ASSERT_FALSE(retrySerialStopNeeded(stop, 1100, true));
    stop.passiveBlocked = false;
    stop.verified = true;
    TEST_ASSERT_FALSE(retrySerialStopNeeded(stop, 1100, true));
    stop.verified = false;
    stop.complete = false;
    TEST_ASSERT_FALSE(retrySerialStopNeeded(stop, 1100, true));
    stop.complete = true;
    stop.completedAtMs = 0xFFFFFFF0;
    TEST_ASSERT_FALSE(retrySerialStopNeeded(stop, 0x10, true));
    TEST_ASSERT_TRUE(retrySerialStopNeeded(stop, 0x400, true));
}

void test_tx_audit_counts_only_bytes_accepted_for_control_frames() {
    TxAudit audit;
    uint8_t read[8], target[8];
    AimFrame::buildRead(read, 1, 0x0E, 10);
    AimFrame::buildMoveAbsolute(target, 1, -5678);
    audit.accepted(read, 8, 8, 10);
    audit.accepted(target, 8, 0, 11);
    TEST_ASSERT_EQUAL_UINT64(0, audit.snapshot().controlWrites);
    audit.accepted(target, 8, 8, 12);
    const auto result = audit.snapshot();
    TEST_ASSERT_EQUAL_UINT64(1, result.controlWrites);
    TEST_ASSERT_EQUAL_UINT64(1, result.absoluteTargets);
    TEST_ASSERT_EQUAL_INT32(-5678, result.lastValue);
    TEST_ASSERT_EQUAL_UINT16(UINT16_MAX, result.lastRegister);
    TEST_ASSERT_EQUAL_UINT32(12, result.lastAtMs);
    TEST_ASSERT_TRUE(result.lastCompleteFrame);
}

void test_tx_audit_tracks_setup_enable_disable_and_mode_without_counting_reads() {
    TxAudit audit;
    uint8_t frame[8];
    for (uint16_t reg : {2, 3, 1, 1, 0}) {
        const uint16_t value = reg == 2 ? 37 : reg == 3 ? 1500 :
            reg == 1 && audit.snapshot().outputEnables == 0 ? 1 : 0;
        AimFrame::buildWriteSingle(frame, 1, reg, value);
        audit.accepted(frame, 8, 8, 10 + reg);
    }
    const auto beforeRead = audit.snapshot();
    TEST_ASSERT_EQUAL_UINT64(5, beforeRead.controlWrites);
    TEST_ASSERT_EQUAL_UINT64(2, beforeRead.setupWrites);
    TEST_ASSERT_EQUAL_UINT64(1, beforeRead.outputEnables);
    TEST_ASSERT_EQUAL_UINT64(1, beforeRead.outputDisables);
    TEST_ASSERT_EQUAL_UINT64(1, beforeRead.modeWrites);
    TEST_ASSERT_EQUAL(static_cast<int>(OssmHatControlWriteType::ModeWrite),
                      static_cast<int>(beforeRead.lastType));
    AimFrame::buildRead(frame, 1, 0, 26);
    audit.accepted(frame, 8, 8, 100);
    TEST_ASSERT_EQUAL_UINT64(5, audit.snapshot().controlWrites);
    TEST_ASSERT_EQUAL_UINT32(beforeRead.lastAtMs, audit.snapshot().lastAtMs);
}

void test_partial_control_frame_invalidates_attribution_conservatively() {
    TxAudit audit;
    uint8_t frame[8];
    AimFrame::buildWriteSingle(frame, 1, 1, 1);
    audit.accepted(frame, 8, 4, 100);
    TEST_ASSERT_EQUAL_UINT64(1, audit.snapshot().controlWrites);
    TEST_ASSERT_FALSE(audit.snapshot().lastCompleteFrame);
    TEST_ASSERT_EQUAL_UINT16(4, audit.snapshot().lastAcceptedBytes);
    TEST_ASSERT_EQUAL_UINT64(1, audit.snapshot().outputEnables);
}

void test_committed_stop_changes_audit_after_gpio_snapshot_but_readback_does_not() {
    TxAudit audit;
    uint8_t frame[8];
    AimFrame::buildMoveAbsolute(frame, 1, -5678);
    audit.accepted(frame, 8, 8, 100);
    const auto gpioCut = audit.snapshot().controlWrites;
    AimFrame::buildRead(frame, 1, 0x0E, 10);
    for (unsigned i = 0; i < 10; ++i) audit.accepted(frame, 8, 8, 110 + i);
    TEST_ASSERT_EQUAL_UINT64(gpioCut, audit.snapshot().controlWrites);
    buildSerialOutputDisable(frame);
    audit.accepted(frame, 8, 8, 200);
    TEST_ASSERT_EQUAL_UINT64(gpioCut + 1, audit.snapshot().controlWrites);
    TEST_ASSERT_EQUAL(static_cast<int>(OssmHatControlWriteType::OutputDisable),
                      static_cast<int>(audit.snapshot().lastType));
    AimFrame::buildRead(frame, 1, 0, 2);
    audit.accepted(frame, 8, 8, 210);
    TEST_ASSERT_EQUAL_UINT64(gpioCut + 1, audit.snapshot().controlWrites);
}

void test_position_request_keeps_full_existing_control_block_through_cancel_cleanup() {
    TEST_ASSERT_TRUE(fullControlPoll(false,false,10,0));
    TEST_ASSERT_FALSE(fullControlPoll(false,true,20,0));
    TEST_ASSERT_TRUE(fullControlPoll(false,true,1000,0));
    PositionQueue queue;
    TEST_ASSERT_TRUE(queue.prepare(17,0,37,1500,100));
    TEST_ASSERT_TRUE(fullControlPoll(queue.status().requested,true,120,100));
    queue.cancel();
    TEST_ASSERT_TRUE(queue.status().requested);
    TEST_ASSERT_TRUE(fullControlPoll(queue.status().requested,true,140,120));
    uint8_t read[8];
    AimFrame::buildRead(read,1,0,26);
    const uint8_t expected[] = {1,3,0,0,0,26,0xC4,0x01};
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected,read,8);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_absolute_manual_frame_and_word_swapped_positive_ack);
    RUN_TEST(test_absolute_reply_current_word_can_differ_from_correlated_intended_target);
    RUN_TEST(test_absolute_response_never_refreshes_coherent_physical_telemetry);
    RUN_TEST(test_fc16_incremental_vectors_have_low_word_first_and_are_never_targets);
    RUN_TEST(test_prepare_has_only_disabled_hold_ram_setup_and_verified_readback);
    RUN_TEST(test_prepare_refuses_live_output_mode0_and_wrong_settings_readback);
    RUN_TEST(test_enable_needs_explicit_request_exact_echo_and_readback);
    RUN_TEST(test_enable_readback_off_cannot_claim_output_enabled);
    RUN_TEST(test_coalesces_latest_target_without_replaying_a_fifo);
    RUN_TEST(test_old_inflight_ack_does_not_clear_a_newer_pending_target);
    RUN_TEST(test_cancel_then_new_session_ignores_old_ack_and_forbids_old_intent);
    RUN_TEST(test_expired_target_cancels_before_any_move_and_retains_first_fault);
    RUN_TEST(test_live_configuration_changes_zero_sessions_and_duplicate_ids_are_rejected);
    RUN_TEST(test_bad_crc_wrong_unit_function_length_and_fc06_echo_are_rejected);
    RUN_TEST(test_signed_count_boundaries_serialize_without_overflow);
    RUN_TEST(test_late_current_position_after_cancel_is_recognized_for_priority_stop_drain);
    RUN_TEST(test_retained_reply_rejects_wrong_unit_function_crc_and_expired_window);
    RUN_TEST(test_late_current_word_has_one_bounded_owner_window_and_never_hides_a_later_master);
    RUN_TEST(test_retained_fc06_reply_still_needs_the_exact_register_value_echo);
    RUN_TEST(test_short_uart_write_cannot_open_or_retain_an_owned_reply_window);
    RUN_TEST(test_split_leading_zero_late_current_reply_does_not_abort_priority_stop_echo);
    RUN_TEST(test_stop_waits_for_discovery_and_has_a_bounded_rollover_safe_failure);
    RUN_TEST(test_repeated_enable_request_does_not_extend_expired_intent);
    RUN_TEST(test_stop_never_switches_mode_or_claims_mode0_is_serial_disabled);
    RUN_TEST(test_cancel_after_committed_enable_cannot_complete_or_queue_after_stop);
    RUN_TEST(test_stop_reassertion_needs_restored_fresh_sole_link_and_retains_success);
    RUN_TEST(test_tx_audit_counts_only_bytes_accepted_for_control_frames);
    RUN_TEST(test_tx_audit_tracks_setup_enable_disable_and_mode_without_counting_reads);
    RUN_TEST(test_partial_control_frame_invalidates_attribution_conservatively);
    RUN_TEST(test_committed_stop_changes_audit_after_gpio_snapshot_but_readback_does_not);
    RUN_TEST(test_position_request_keeps_full_existing_control_block_through_cancel_cleanup);
    return UNITY_END();
}
