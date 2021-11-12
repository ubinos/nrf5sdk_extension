set(INCLUDE__NRF5SDK_EXTENSION TRUE)
set(PROJECT_UBINOS_LIBRARIES ${PROJECT_UBINOS_LIBRARIES} nrf5sdk_extension)

set_cache_default(NRF5SDK__DTTY_NRF_UART_READ_BUFFER_SIZE "512" STRING "nrf5sdk dtty uart read buffer size")
set_cache_default(NRF5SDK__DTTY_NRF_UART_WRITE_BUFFER_SIZE "1024 * 10" STRING "nrf5sdk dtty uart read buffer size")

