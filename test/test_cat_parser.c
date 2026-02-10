#include <stdio.h>
#include <string.h>
#include "unity.h"
#include "cat_parser.h"

#include "cat_parser.h"
#include "lvgl.h"

static uint32_t last_freq_a = 0;
static int last_rit_freq = 0;
static int last_xit_freq = 0;
static bool last_rit_status = false;
static bool last_xit_status = false;
static bool last_tx_status = false;
static int last_mode = -1;
static bool last_split_status = false;
static bool last_tone_status = false;
static int last_vfo_function = -1; // For FR/FT commands
static transverter_xo_data_t last_xo_data = {0};
static transverter_ex056_data_t last_ex056_data = {0};
static transverter_state_t last_transverter_state = {0};
static int last_swr_value = -1;
static int last_power_value = -1;
static int last_alc_value = -1;

void lv_msg_send(uint32_t msg_id, const void * payload)
{
    switch(msg_id) {
        case LV_MSG_VFO_FUNCTION_UPDATED:
            last_vfo_function = *(int*)payload;
            break;
        case LV_MSG_FREQ_A_UPDATED:
            last_freq_a = *(uint32_t*)payload;
            break;
        case LV_MSG_RIT_FREQ_UPDATED:
            last_rit_freq = *(int*)payload;
            break;
        case LV_MSG_XIT_FREQ_UPDATED:
            last_xit_freq = *(int*)payload;
            break;
        case LV_MSG_RIT_STATUS_UPDATED:
            last_rit_status = *(bool*)payload;
            break;
        case LV_MSG_XIT_STATUS_UPDATED:
            last_xit_status = *(bool*)payload;
            break;
        case LV_MSG_TX_STATUS_UPDATED:
            last_tx_status = *(bool*)payload;
            break;
        case LV_MSG_MODE_UPDATED:
            last_mode = *(int*)payload;
            break;
        case LV_MSG_SPLIT_STATUS_UPDATED:
            last_split_status = *(bool*)payload;
            break;
        case LV_MSG_TONE_STATUS_UPDATED:
            last_tone_status = *(bool*)payload;
            break;
        case LV_MSG_TRANSVERTER_XO_UPDATED:
            last_xo_data = *(transverter_xo_data_t*)payload;
            break;
        case LV_MSG_TRANSVERTER_EX056_UPDATED:
            last_ex056_data = *(transverter_ex056_data_t*)payload;
            break;
        case LV_MSG_TRANSVERTER_STATE_UPDATED:
            last_transverter_state = *(transverter_state_t*)payload;
            break;
        case LV_MSG_SWR_UPDATED:
            last_swr_value = *(int*)payload;
            break;
        case LV_MSG_POWER_UPDATED:
            last_power_value = *(int*)payload;
            break;
        case LV_MSG_ALC_UPDATED:
            last_alc_value = *(int*)payload;
            break;
    }
}

// Mock function for UI toggle state (always return true for testing)
bool ui_get_xvtr_offset_mix_enabled(void) {
    return true;
}

void test_parse_if_command(void)
{
    // Test case 1: All features off
    const char* test_response1 = "IF00014070000     +000000000000000;";
    parse_if_command(test_response1);
    
    TEST_ASSERT_EQUAL_UINT32(14070000, last_freq_a);
    TEST_ASSERT_EQUAL_INT(0, last_rit_freq);
    TEST_ASSERT_FALSE(last_rit_status);
    TEST_ASSERT_FALSE(last_xit_status);
    TEST_ASSERT_FALSE(last_tx_status);
    TEST_ASSERT_EQUAL_INT(0, last_mode);
    TEST_ASSERT_FALSE(last_split_status);
    TEST_ASSERT_FALSE(last_tone_status);

    // Test case 2: Various features on
    const char* test_response2 = "IF00028500500     -0001201011100110;";
    parse_if_command(test_response2);

    TEST_ASSERT_EQUAL_UINT32(28500500, last_freq_a);
    TEST_ASSERT_EQUAL_INT(-120, last_rit_freq);
    TEST_ASSERT_TRUE(last_rit_status);
    TEST_ASSERT_TRUE(last_xit_status);
    TEST_ASSERT_TRUE(last_tx_status);
    TEST_ASSERT_EQUAL_INT(1, last_mode);
    TEST_ASSERT_TRUE(last_split_status);
    TEST_ASSERT_TRUE(last_tone_status);

    // Test case 3: Different mode and memory bank
    const char* test_response3 = "IF00007100000     +000000203000000;";
    parse_if_command(test_response3);

    TEST_ASSERT_EQUAL_UINT32(7100000, last_freq_a);
    TEST_ASSERT_EQUAL_INT(0, last_rit_freq);
    TEST_ASSERT_FALSE(last_rit_status);
    TEST_ASSERT_FALSE(last_xit_status);
    TEST_ASSERT_FALSE(last_tx_status);
    TEST_ASSERT_EQUAL_INT(2, last_mode);
    TEST_ASSERT_FALSE(last_split_status);
    TEST_ASSERT_FALSE(last_tone_status);

    // Add more assertions for memory bank, scan status, etc. if implemented
}

void test_parse_fa_frequency(void)
{
    // Test case for parse_fa_frequency
    const char* test_response = "FA00014070000";
    uint32_t result = parse_fa_frequency(test_response);
    TEST_ASSERT_EQUAL(14070000, result);
}

void test_parse_fr_ft_command(void) {
    // Reset states before each sub-test might be good if not done by Unity's TEST_CASE
    last_vfo_function = -1;
    last_split_status = false; // Default to simplex

    // Test FR0 (VFO A, Simplex)
    parse_fr_ft_command("FR0;");
    TEST_ASSERT_EQUAL_INT(0, last_vfo_function);
    TEST_ASSERT_FALSE(last_split_status);

    // Test FR1 (VFO B, Simplex)
    parse_fr_ft_command("FR1;");
    TEST_ASSERT_EQUAL_INT(1, last_vfo_function);
    TEST_ASSERT_FALSE(last_split_status);

    // Test FR2 (Memory, Simplex)
    parse_fr_ft_command("FR2;");
    TEST_ASSERT_EQUAL_INT(2, last_vfo_function);
    TEST_ASSERT_FALSE(last_split_status);

    // Test FT0 (TX same as RX, Simplex) - Correct TS-590SG behavior
    parse_fr_ft_command("FT0;");
    TEST_ASSERT_EQUAL_INT(0, last_vfo_function);
    TEST_ASSERT_FALSE(last_split_status);        // FT0 = simplex

    // Test FT1 (TX opposite from RX, Split) - Correct TS-590SG behavior  
    parse_fr_ft_command("FT1;");
    TEST_ASSERT_EQUAL_INT(1, last_vfo_function);
    TEST_ASSERT_TRUE(last_split_status);         // FT1 = split

    // Test FT2 (Memory TX)
    parse_fr_ft_command("FT2;");
    TEST_ASSERT_EQUAL_INT(2, last_vfo_function); // Selects Memory
    TEST_ASSERT_FALSE(last_split_status);        // Split status unclear for memory

    // Test short command
    last_vfo_function = -1; // Reset for this specific check
    parse_fr_ft_command("FR"); 
    TEST_ASSERT_EQUAL_INT(-1, last_vfo_function); // Should not parse

    // Test invalid P1
    last_vfo_function = -1;
    parse_fr_ft_command("FR3;");
    TEST_ASSERT_EQUAL_INT(-1, last_vfo_function); // Should not parse validly
}

void test_parse_xo_command(void)
{
    // Test XO command parsing - Plus direction with 14.5 GHz offset
    const char* test_xo_plus = "XO014500000000;";
    parse_xo_command(test_xo_plus);
    
    TEST_ASSERT_TRUE(last_xo_data.valid);
    TEST_ASSERT_TRUE(last_xo_data.direction_plus);
    TEST_ASSERT_EQUAL_UINT64(14500000000LL, last_xo_data.offset_frequency);
    TEST_ASSERT_TRUE(last_transverter_state.xo_data.valid);
    
    // Test XO command parsing - Minus direction with 10.368 GHz offset
    const char* test_xo_minus = "XO110368000000;";
    parse_xo_command(test_xo_minus);
    
    TEST_ASSERT_TRUE(last_xo_data.valid);
    TEST_ASSERT_FALSE(last_xo_data.direction_plus);
    TEST_ASSERT_EQUAL_UINT64(10368000000LL, last_xo_data.offset_frequency);
    
    // Test invalid XO command (too short)
    last_xo_data.valid = false;
    parse_xo_command("XO0145;");
    TEST_ASSERT_FALSE(last_xo_data.valid);
    
    // Test invalid XO command (invalid direction)
    parse_xo_command("XO214500000000;");
    TEST_ASSERT_FALSE(last_xo_data.valid);
}

void test_parse_ex_command(void)
{
    // Test EX056 command parsing - Transverter disabled
    const char* test_ex056_disabled = "EX05600000;";
    parse_ex_command(test_ex056_disabled);
    
    TEST_ASSERT_TRUE(last_ex056_data.valid);
    TEST_ASSERT_FALSE(last_ex056_data.enabled);
    TEST_ASSERT_FALSE(last_ex056_data.power_down_mode);
    TEST_ASSERT_TRUE(last_transverter_state.ex056_data.valid);
    
    // Test EX056 command parsing - Transverter enabled
    const char* test_ex056_enabled = "EX05600001;";
    parse_ex_command(test_ex056_enabled);
    
    TEST_ASSERT_TRUE(last_ex056_data.valid);
    TEST_ASSERT_TRUE(last_ex056_data.enabled);
    TEST_ASSERT_FALSE(last_ex056_data.power_down_mode);
    
    // Test EX056 command parsing - Transverter enabled with power down
    const char* test_ex056_power_down = "EX05600002;";
    parse_ex_command(test_ex056_power_down);
    
    TEST_ASSERT_TRUE(last_ex056_data.valid);
    TEST_ASSERT_TRUE(last_ex056_data.enabled);
    TEST_ASSERT_TRUE(last_ex056_data.power_down_mode);
    
    // Test non-EX056 command (should be ignored)
    last_ex056_data.valid = false;
    parse_ex_command("EX05700001;");
    TEST_ASSERT_FALSE(last_ex056_data.valid);
    
    // Test invalid EX056 value
    parse_ex_command("EX05600003;");
    TEST_ASSERT_FALSE(last_ex056_data.valid);
}

void test_transverter_offset_calculations(void)
{
    // Set up test transverter state
    transverter_state_t* state = get_transverter_state();
    state->xo_data.valid = true;
    state->xo_data.direction_plus = true;
    state->xo_data.offset_frequency = 10368000000LL; // 10.368 GHz
    state->ex056_data.valid = true;
    state->ex056_data.enabled = true;
    
    // Test plus direction: display = radio + offset
    uint64_t radio_freq = 144000000; // 144 MHz
    uint64_t display_freq = apply_transverter_offset_for_display(radio_freq);
    uint64_t expected_display = 144000000 + 10368000000LL; // 10.512 GHz
    TEST_ASSERT_EQUAL_UINT64(expected_display, display_freq);
    
    // Test reverse calculation: radio = display - offset
    uint64_t computed_radio = compute_radio_frequency_from_display(display_freq);
    TEST_ASSERT_EQUAL_UINT64(radio_freq, computed_radio);
    
    // Test minus direction: display = radio - offset
    state->xo_data.direction_plus = false;
    radio_freq = 10512000000LL; // 10.512 GHz
    display_freq = apply_transverter_offset_for_display(radio_freq);
    expected_display = 10512000000LL - 10368000000LL; // 144 MHz
    TEST_ASSERT_EQUAL_UINT64(expected_display, display_freq);
    
    // Test disabled transverter (should return unchanged frequency)
    state->ex056_data.enabled = false;
    display_freq = apply_transverter_offset_for_display(144000000);
    TEST_ASSERT_EQUAL_UINT64(144000000, display_freq);
}

void test_parse_rm_command(void)
{
    // Reset meter values
    last_swr_value = -1;
    last_power_value = -1;
    last_alc_value = -1;

    // Test SWR meter (P1=1)
    parse_rm_command("RM10015;");
    TEST_ASSERT_EQUAL_INT(15, last_swr_value);
    TEST_ASSERT_EQUAL_INT(-1, last_power_value); // Should not be updated
    TEST_ASSERT_EQUAL_INT(-1, last_alc_value);   // Should not be updated

    // Test COMP/Power meter (P1=2) - This was the bug, should now work
    last_swr_value = -1; // Reset
    parse_rm_command("RM20025;");
    TEST_ASSERT_EQUAL_INT(25, last_power_value);
    TEST_ASSERT_EQUAL_INT(-1, last_swr_value);   // Should not be updated
    TEST_ASSERT_EQUAL_INT(-1, last_alc_value);   // Should not be updated

    // Test ALC meter (P1=3)
    last_power_value = -1; // Reset
    parse_rm_command("RM30010;");
    TEST_ASSERT_EQUAL_INT(10, last_alc_value);
    TEST_ASSERT_EQUAL_INT(-1, last_swr_value);   // Should not be updated
    TEST_ASSERT_EQUAL_INT(-1, last_power_value); // Should not be updated

    // Test No Selection (P1=0) - should not update any meters
    last_swr_value = -1;
    last_power_value = -1;
    last_alc_value = -1;
    parse_rm_command("RM00020;");
    TEST_ASSERT_EQUAL_INT(-1, last_swr_value);
    TEST_ASSERT_EQUAL_INT(-1, last_power_value);
    TEST_ASSERT_EQUAL_INT(-1, last_alc_value);

    // Test value clamping (over 30)
    parse_rm_command("RM10035;");
    TEST_ASSERT_EQUAL_INT(30, last_swr_value); // Should be clamped to 30

    // Test invalid command (too short)
    last_swr_value = -1;
    parse_rm_command("RM1;");
    TEST_ASSERT_EQUAL_INT(-1, last_swr_value); // Should not be updated

    // Test invalid meter type
    parse_rm_command("RM40015;");
    TEST_ASSERT_EQUAL_INT(-1, last_swr_value); // Should not be updated
}

// Add more test functions for other parsing functions
