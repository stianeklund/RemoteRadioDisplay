#include <stdio.h>
#include <string.h>
#include "unity.h"

// Helper function to format memory channel number according to TS-590SG conventions
// Channels 0-99: "M.CH 00" to "M.CH 99"
// Channels 100-109: "M.CH P00" to "M.CH P09" (Program Memory)
// Channels 110-119: "M.CH E00" to "M.CH E09" (Emergency Memory)
// Channels 120-299: "M.CH 120" to "M.CH 299"
static void format_memory_channel_text(char *buffer, size_t buffer_size, uint16_t channel) {
    if (channel >= 100 && channel <= 109) {
        // Program Memory: P00-P09
        snprintf(buffer, buffer_size, "M.CH P%02d", channel - 100);
    } else if (channel >= 110 && channel <= 119) {
        // Emergency Memory: E00-E09
        snprintf(buffer, buffer_size, "M.CH E%02d", channel - 110);
    } else if (channel < 100) {
        // Standard channels: 00-99
        snprintf(buffer, buffer_size, "M.CH %02d", channel);
    } else {
        // Extended channels: 120-299
        snprintf(buffer, buffer_size, "M.CH %d", channel);
    }
}

void setUp(void) {
    // Set up before each test
}

void tearDown(void) {
    // Clean up after each test
}

void test_memory_channel_format_standard_00(void) {
    char buffer[16];
    format_memory_channel_text(buffer, sizeof(buffer), 0);
    TEST_ASSERT_EQUAL_STRING("M.CH 00", buffer);
}

void test_memory_channel_format_standard_05(void) {
    char buffer[16];
    format_memory_channel_text(buffer, sizeof(buffer), 5);
    TEST_ASSERT_EQUAL_STRING("M.CH 05", buffer);
}

void test_memory_channel_format_standard_50(void) {
    char buffer[16];
    format_memory_channel_text(buffer, sizeof(buffer), 50);
    TEST_ASSERT_EQUAL_STRING("M.CH 50", buffer);
}

void test_memory_channel_format_standard_99(void) {
    char buffer[16];
    format_memory_channel_text(buffer, sizeof(buffer), 99);
    TEST_ASSERT_EQUAL_STRING("M.CH 99", buffer);
}

void test_memory_channel_format_program_100(void) {
    char buffer[16];
    format_memory_channel_text(buffer, sizeof(buffer), 100);
    TEST_ASSERT_EQUAL_STRING("M.CH P00", buffer);
}

void test_memory_channel_format_program_105(void) {
    char buffer[16];
    format_memory_channel_text(buffer, sizeof(buffer), 105);
    TEST_ASSERT_EQUAL_STRING("M.CH P05", buffer);
}

void test_memory_channel_format_program_109(void) {
    char buffer[16];
    format_memory_channel_text(buffer, sizeof(buffer), 109);
    TEST_ASSERT_EQUAL_STRING("M.CH P09", buffer);
}

void test_memory_channel_format_emergency_110(void) {
    char buffer[16];
    format_memory_channel_text(buffer, sizeof(buffer), 110);
    TEST_ASSERT_EQUAL_STRING("M.CH E00", buffer);
}

void test_memory_channel_format_emergency_115(void) {
    char buffer[16];
    format_memory_channel_text(buffer, sizeof(buffer), 115);
    TEST_ASSERT_EQUAL_STRING("M.CH E05", buffer);
}

void test_memory_channel_format_emergency_119(void) {
    char buffer[16];
    format_memory_channel_text(buffer, sizeof(buffer), 119);
    TEST_ASSERT_EQUAL_STRING("M.CH E09", buffer);
}

void test_memory_channel_format_extended_120(void) {
    char buffer[16];
    format_memory_channel_text(buffer, sizeof(buffer), 120);
    TEST_ASSERT_EQUAL_STRING("M.CH 120", buffer);
}

void test_memory_channel_format_extended_200(void) {
    char buffer[16];
    format_memory_channel_text(buffer, sizeof(buffer), 200);
    TEST_ASSERT_EQUAL_STRING("M.CH 200", buffer);
}

void test_memory_channel_format_extended_299(void) {
    char buffer[16];
    format_memory_channel_text(buffer, sizeof(buffer), 299);
    TEST_ASSERT_EQUAL_STRING("M.CH 299", buffer);
}

void app_main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_memory_channel_format_standard_00);
    RUN_TEST(test_memory_channel_format_standard_05);
    RUN_TEST(test_memory_channel_format_standard_50);
    RUN_TEST(test_memory_channel_format_standard_99);

    RUN_TEST(test_memory_channel_format_program_100);
    RUN_TEST(test_memory_channel_format_program_105);
    RUN_TEST(test_memory_channel_format_program_109);

    RUN_TEST(test_memory_channel_format_emergency_110);
    RUN_TEST(test_memory_channel_format_emergency_115);
    RUN_TEST(test_memory_channel_format_emergency_119);

    RUN_TEST(test_memory_channel_format_extended_120);
    RUN_TEST(test_memory_channel_format_extended_200);
    RUN_TEST(test_memory_channel_format_extended_299);

    UNITY_END();
}
