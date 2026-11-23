#include <unity.h>

#include <atomic>
#include <string>
#include <thread>
#include <vector>

#include "utils/bounded_queue.h"

void setUp() {}
void tearDown() {}

void test_fifo_order() {
    BoundedQueue<int, 4> queue;
    TEST_ASSERT_TRUE(queue.empty());
    TEST_ASSERT_TRUE(queue.push(1));
    TEST_ASSERT_TRUE(queue.push(2));
    int value = 0;
    TEST_ASSERT_TRUE(queue.peek(value));
    TEST_ASSERT_EQUAL(1, value);
    TEST_ASSERT_TRUE(queue.pop(value));
    TEST_ASSERT_EQUAL(1, value);
    TEST_ASSERT_TRUE(queue.pop(value));
    TEST_ASSERT_EQUAL(2, value);
    TEST_ASSERT_FALSE(queue.pop(value));
    TEST_ASSERT_FALSE(queue.peek(value));
}

void test_full_queue_drops_newest_by_default() {
    BoundedQueue<int, 3> queue;
    for (int i = 0; i < 3; i++) TEST_ASSERT_TRUE(queue.push(i));
    TEST_ASSERT_FALSE(queue.push(99));
    TEST_ASSERT_EQUAL(3, queue.size());
    TEST_ASSERT_EQUAL(1, queue.dropped());
    int value = -1;
    queue.pop(value);
    TEST_ASSERT_EQUAL(0, value);
}

void test_full_queue_can_drop_oldest() {
    // Streaming targets: the latest position matters most.
    BoundedQueue<int, 3> queue;
    for (int i = 0; i < 5; i++) queue.push(i, true);
    TEST_ASSERT_EQUAL(3, queue.size());
    int value = -1;
    queue.pop(value);
    TEST_ASSERT_EQUAL(2, value);
}

void test_replace_all_keeps_only_the_stop() {
    BoundedQueue<std::string, 4> queue;
    for (int i = 0; i < 4; i++) queue.push("set:speed:" + std::to_string(i));
    queue.replaceAll("go:menu");
    TEST_ASSERT_EQUAL(1, queue.size());
    std::string value;
    queue.pop(value);
    TEST_ASSERT_EQUAL_STRING("go:menu", value.c_str());
}

void test_clear_empties() {
    BoundedQueue<int, 8> queue;
    for (int i = 0; i < 8; i++) queue.push(i);
    queue.clear();
    TEST_ASSERT_TRUE(queue.empty());
    TEST_ASSERT_TRUE(queue.push(1));
}

void test_concurrent_producers_and_consumer() {
    // Three producers (BLE host, RAD BLE, NimBLE loop) and one consumer.
    // Nothing is lost or duplicated beyond what the bound drops.
    BoundedQueue<int, 64> queue;
    constexpr int kPerProducer = 20000;
    std::atomic<bool> done{false};
    std::atomic<long> consumed{0};
    std::atomic<long> sum{0};
    std::thread consumer([&] {
        int value;
        while (!done.load() || !queue.empty()) {
            if (queue.pop(value)) {
                consumed++;
                sum += value;
            }
        }
    });
    std::atomic<long> accepted{0};
    std::atomic<long> acceptedSum{0};
    std::vector<std::thread> producers;
    for (int p = 0; p < 3; p++) {
        producers.emplace_back([&, p] {
            for (int i = 0; i < kPerProducer; i++) {
                const int value = p * kPerProducer + i;
                if (queue.push(value)) {
                    accepted++;
                    acceptedSum += value;
                }
            }
        });
    }
    for (auto &producer : producers) producer.join();
    done = true;
    consumer.join();
    TEST_ASSERT_EQUAL(accepted.load(), consumed.load());
    TEST_ASSERT_EQUAL(acceptedSum.load(), sum.load());
    TEST_ASSERT_EQUAL(3L * kPerProducer, accepted.load() + (long)queue.dropped());
    TEST_ASSERT_TRUE(queue.size() <= decltype(queue)::capacity);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_fifo_order);
    RUN_TEST(test_full_queue_drops_newest_by_default);
    RUN_TEST(test_full_queue_can_drop_oldest);
    RUN_TEST(test_replace_all_keeps_only_the_stop);
    RUN_TEST(test_clear_empties);
    RUN_TEST(test_concurrent_producers_and_consumer);
    return UNITY_END();
}
