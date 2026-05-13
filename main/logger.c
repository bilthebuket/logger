#include "lib.h"
#include "tests.h"

void app_main(void)
{
#ifdef CONFIG_RUN_TESTS
	run_tests();
#else
	setup_uart();
	process_data_from_gps_sensor(&uart_read_bytes);
#endif
}
