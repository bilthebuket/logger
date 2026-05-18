#include "lib.h"
#include "tests.h"

void app_main(void)
{
#ifdef CONFIG_RUN_TESTS
	printf("test\n");
	run_tests();
#else
	printf("prod\n");
	setup_uart();
	process_data_from_gps_sensor(&uart_read_bytes);
#endif
}
