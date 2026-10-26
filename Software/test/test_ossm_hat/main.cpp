#include <unity.h>

#include <array>
#include <vector>

#include "../../src/services/ossmHatProtocol.hpp"

void setUp() {}
void tearDown() {}

using ossm_hat::Decoder;

namespace {

void appendCrc(std::vector<uint8_t> &frame) {
    const uint16_t crc = AimFrame::crc16(frame.data(), frame.size());
    frame.push_back(crc & 0xFF);
    frame.push_back(crc >> 8);
}

std::vector<uint8_t> request(uint16_t start = 0x0E, uint16_t count = 12,
                             uint8_t id = 1) {
    std::vector<uint8_t> result(8);
    AimFrame::buildRead(result.data(), id, start, count);
    return result;
}

std::vector<uint8_t> reply(uint16_t start = 0x0E, uint16_t count = 12,
                          int32_t position = -87259, uint8_t id = 1) {
    std::array<uint16_t, 26> words{};
    words[0x0E] = 0x10;
    words[0x0F] = 3500;  // 1.75 A
    words[0x11] = 7848;  // 24 V
    words[0x16] = static_cast<uint32_t>(position) & 0xFFFF;
    words[0x17] = static_cast<uint32_t>(position) >> 16;
    std::vector<uint8_t> result{id, 3, static_cast<uint8_t>(2 * count)};
    for (size_t i = start; i < start + count; ++i) {
        result.push_back(words[i] >> 8);
        result.push_back(words[i] & 0xFF);
    }
    appendCrc(result);
    return result;
}

uint8_t feed(Decoder &decoder, const std::vector<uint8_t> &bytes,
             uint32_t time = 100) {
    uint8_t events = 0;
    for (auto byte : bytes) events |= decoder.push(byte, time);
    return events;
}

}  // namespace

void test_passive_guard_block_decodes_all_fields() {
    Decoder decoder;
    TEST_ASSERT_EQUAL(Decoder::ExternalRequest, feed(decoder, request()));
    decoder.push(0, 101);  // known HAT turnaround glitch
    TEST_ASSERT_EQUAL(Decoder::Telemetry, feed(decoder, reply(), 102));
    const auto sample = decoder.sample();
    TEST_ASSERT_EQUAL_INT32(-87259, sample.positionCounts);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.75f, sample.currentAmps);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 24.0f, sample.voltageVolts);
    TEST_ASSERT_EQUAL_HEX16(0x10, sample.alarm);
    TEST_ASSERT_EQUAL_UINT32(102, sample.updatedAtMs);
    TEST_ASSERT_TRUE(ossm_hat::fresh(sample, 200));
    TEST_ASSERT_FALSE(ossm_hat::fresh(sample, 353));
}

void test_manual_read_vector_and_captured_glitch_reply() {
    // YZ-AIM manual read-position vector and Alex's V2.3 bench capture.
    // These fixed bytes verify CRC independently of this test's builders.
    const uint8_t expected[]{1, 3, 0, 0x16, 0, 2, 0x25, 0xCF};
    uint8_t actual[8];
    AimFrame::buildRead(actual, 1, 0x16, 2);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, actual, sizeof(expected));
    const uint8_t captured[]{0, 1, 3, 2, 3, 0x20, 0xB9, 0x6C};
    const auto parsed = AimFrame::parseReply(captured, sizeof(captured), 1, 3, 7);
    TEST_ASSERT_EQUAL(static_cast<int>(AimFrame::Status::Ok),
                      static_cast<int>(parsed.status));
    TEST_ASSERT_EQUAL_UINT32(1, parsed.skipped);
    TEST_ASSERT_EQUAL_UINT16(800, AimFrame::be16(captured + parsed.offset + 3));
}

void test_active_read_does_not_report_external_master() {
    Decoder decoder;
    decoder.expectRead(1, 0x0E, 10, 10);
    TEST_ASSERT_EQUAL(Decoder::Telemetry, feed(decoder, reply(0x0E, 10, 65537), 11));
    TEST_ASSERT_EQUAL_INT32(65537, decoder.sample().positionCounts);
}

void test_encoder_high_and_low_words_include_signed_position() {
    for (int32_t position : {INT32_MIN, INT32_MAX, -1, 0, 32768}) {
        Decoder decoder;
        feed(decoder, request());
        feed(decoder, reply(0x0E, 12, position), 101);
        TEST_ASSERT_EQUAL_INT32(position, decoder.sample().positionCounts);
    }
}

void test_whole_register_snapshot_uses_requested_offset() {
    Decoder decoder;
    feed(decoder, request(0, 26));
    TEST_ASSERT_EQUAL(Decoder::Telemetry, feed(decoder, reply(0, 26), 101));
    TEST_ASSERT_EQUAL_INT32(-87259, decoder.sample().positionCounts);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.75f, decoder.sample().currentAmps);
}

void test_unsolicited_response_cannot_detect_hat() {
    Decoder decoder;
    TEST_ASSERT_EQUAL(Decoder::None, feed(decoder, reply()));
    TEST_ASSERT_EQUAL_UINT32(0, decoder.sample().sequence);
}

void test_partial_read_never_makes_current_voltage_fresh() {
    Decoder decoder;
    feed(decoder, request(0x16, 2));
    TEST_ASSERT_EQUAL(Decoder::None, feed(decoder, reply(0x16, 2), 101));
    TEST_ASSERT_EQUAL_UINT32(0, decoder.sample().sequence);
}

void test_bad_crc_does_not_refresh_a_sample() {
    Decoder decoder;
    feed(decoder, request());
    feed(decoder, reply(), 101);
    decoder.reset();
    feed(decoder, request(), 400);
    auto damaged = reply();
    damaged[6] ^= 1;
    feed(decoder, damaged, 401);
    TEST_ASSERT_EQUAL_UINT32(1, decoder.sample().sequence);
    TEST_ASSERT_EQUAL_UINT32(101, decoder.sample().updatedAtMs);
    TEST_ASSERT_FALSE(ossm_hat::fresh(decoder.sample(), 401));
    TEST_ASSERT_GREATER_THAN_UINT32(0, decoder.crcErrors());
}

void test_wrong_server_and_byte_count_rejected() {
    Decoder decoder;
    feed(decoder, request());
    TEST_ASSERT_EQUAL(Decoder::None, feed(decoder, reply(0x0E, 12, 0, 2), 101));
    TEST_ASSERT_EQUAL_UINT32(0, decoder.sample().sequence);
    TEST_ASSERT_EQUAL(Decoder::None, feed(decoder, reply(0x0E, 10), 102));
    TEST_ASSERT_EQUAL_UINT32(0, decoder.sample().sequence);
}

void test_timeout_rejects_late_reply() {
    Decoder decoder;
    feed(decoder, request(), 100);
    TEST_ASSERT_EQUAL(Decoder::None, feed(decoder, reply(), 201));
    TEST_ASSERT_EQUAL_UINT32(0, decoder.sample().sequence);
}

void test_fragmented_frame_publishes_only_after_crc() {
    Decoder decoder;
    feed(decoder, request(), 100);
    const auto bytes = reply();
    for (size_t i = 0; i < bytes.size() - 1; ++i) {
        TEST_ASSERT_EQUAL(Decoder::None, decoder.push(bytes[i], 101));
        TEST_ASSERT_EQUAL_UINT32(0, decoder.sample().sequence);
    }
    TEST_ASSERT_EQUAL(Decoder::Telemetry, decoder.push(bytes.back(), 102));
}

void test_exception_invalidates_pending_request() {
    Decoder decoder;
    feed(decoder, request(), 100);
    std::vector<uint8_t> exception{1, 0x83, 2};
    appendCrc(exception);
    feed(decoder, exception, 101);
    TEST_ASSERT_EQUAL(Decoder::None, feed(decoder, reply(), 102));
    TEST_ASSERT_EQUAL_UINT32(0, decoder.sample().sequence);
}

void test_other_master_address_and_stop_write_are_observed() {
    Decoder decoder;
    TEST_ASSERT_EQUAL(Decoder::ExternalRequest, feed(decoder, request(0, 1, 2)));
    std::vector<uint8_t> stop(8);
    AimFrame::buildWriteSingle(stop.data(), 1, 1, 0);
    TEST_ASSERT_EQUAL(Decoder::ExternalRequest, feed(decoder, stop));
}

void test_zero_noise_and_reset_allow_reconnect() {
    Decoder decoder;
    feed(decoder, {0, 0, 255, 255, 0, 0});
    feed(decoder, request());
    TEST_ASSERT_EQUAL(Decoder::Telemetry, feed(decoder, reply(), 101));
    decoder.reset();
    feed(decoder, request(), 500);
    TEST_ASSERT_EQUAL(Decoder::Telemetry, feed(decoder, reply(), 501));
    TEST_ASSERT_EQUAL_UINT32(2, decoder.sample().sequence);
}

void test_freshness_handles_millis_rollover_and_empty_sample() {
    OssmHatTelemetry sample;
    TEST_ASSERT_FALSE(ossm_hat::fresh(sample, 0));
    sample.sequence = 1;
    sample.updatedAtMs = UINT32_MAX - 100;
    TEST_ASSERT_TRUE(ossm_hat::fresh(sample, 20));
    TEST_ASSERT_FALSE(ossm_hat::fresh(sample, 200));
}

void test_noise_header_and_truncated_reply_recover_on_next_transaction() {
    Decoder decoder;
    feed(decoder, {1, 3, 64});  // unsupported unsolicited length must not block
    feed(decoder, request(), 100);
    TEST_ASSERT_EQUAL(Decoder::Telemetry, feed(decoder, reply(), 101));
    feed(decoder, request(), 200);
    feed(decoder, {1, 3, 24, 0}, 201);  // truncated previous reply
    feed(decoder, request(), 210);
    TEST_ASSERT_TRUE(feed(decoder, reply(), 211) & Decoder::Telemetry);
    TEST_ASSERT_EQUAL_UINT32(2, decoder.sample().sequence);
}

void test_fc10_acknowledgement_does_not_block_next_read() {
    Decoder decoder;
    std::vector<uint8_t> acknowledgement{1, 0x10, 0, 0x0C, 0, 2};
    appendCrc(acknowledgement);
    TEST_ASSERT_EQUAL(Decoder::ExternalRequest, feed(decoder, acknowledgement));
    feed(decoder, request());
    TEST_ASSERT_EQUAL(Decoder::Telemetry, feed(decoder, reply(), 101));
}

void test_continuity_changes_after_gap_even_without_consumer_reads() {
    Decoder decoder;
    feed(decoder, request(), 100);
    feed(decoder, reply(), 101);
    TEST_ASSERT_EQUAL_UINT32(1, decoder.sample().continuity);
    decoder.reset();  // ordinary transaction/baud reset preserves continuity
    feed(decoder, request(), 200);
    feed(decoder, reply(), 201);
    TEST_ASSERT_EQUAL_UINT32(1, decoder.sample().continuity);
    // No getter was called while stale: the next fresh response must still
    // invalidate any previous physical home/coordinate anchor.
    decoder.reset();
    feed(decoder, request(), 500);
    feed(decoder, reply(), 501);
    TEST_ASSERT_EQUAL_UINT32(2, decoder.sample().continuity);
    TEST_ASSERT_TRUE(ossm_hat::fresh(decoder.sample(), 501));
    TEST_ASSERT_EQUAL_UINT32(3, decoder.sample().sequence);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_manual_read_vector_and_captured_glitch_reply);
    RUN_TEST(test_passive_guard_block_decodes_all_fields);
    RUN_TEST(test_active_read_does_not_report_external_master);
    RUN_TEST(test_encoder_high_and_low_words_include_signed_position);
    RUN_TEST(test_whole_register_snapshot_uses_requested_offset);
    RUN_TEST(test_unsolicited_response_cannot_detect_hat);
    RUN_TEST(test_partial_read_never_makes_current_voltage_fresh);
    RUN_TEST(test_bad_crc_does_not_refresh_a_sample);
    RUN_TEST(test_wrong_server_and_byte_count_rejected);
    RUN_TEST(test_timeout_rejects_late_reply);
    RUN_TEST(test_fragmented_frame_publishes_only_after_crc);
    RUN_TEST(test_exception_invalidates_pending_request);
    RUN_TEST(test_other_master_address_and_stop_write_are_observed);
    RUN_TEST(test_zero_noise_and_reset_allow_reconnect);
    RUN_TEST(test_freshness_handles_millis_rollover_and_empty_sample);
    RUN_TEST(test_noise_header_and_truncated_reply_recover_on_next_transaction);
    RUN_TEST(test_fc10_acknowledgement_does_not_block_next_read);
    RUN_TEST(test_continuity_changes_after_gap_even_without_consumer_reads);
    return UNITY_END();
}
