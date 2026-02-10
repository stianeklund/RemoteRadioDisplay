#include "unity.h"

// Declare your test function prototypes here
void test_parse_if_command(void);
void test_parse_fa_frequency(void);
// ... other test function declarations ...

void app_main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_parse_if_command);
    RUN_TEST(test_parse_fa_frequency);
    // ... run other tests ...
    UNITY_END();
}
