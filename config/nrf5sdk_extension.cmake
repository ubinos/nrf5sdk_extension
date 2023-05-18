set(INCLUDE__NRF5SDK_EXTENSION TRUE)
set(PROJECT_UBINOS_LIBRARIES ${PROJECT_UBINOS_LIBRARIES} nrf5sdk_extension)

set_cache_default(NRF5SDK__DTTY_NRF_ISR_WRITE_BUFFER_SIZE "128" STRING "nrf5sdk dtty write buffer size for in interrupt or critical section")

set_cache_default(NRF5SDK__DTTY_NRF_UART_READ_BUFFER_SIZE "512" STRING "nrf5sdk dtty uart read buffer size")
set_cache_default(NRF5SDK__DTTY_NRF_UART_WRITE_BUFFER_SIZE "1024 * 10" STRING "nrf5sdk dtty uart write buffer size")

set_cache_default(NRF5SDK__DTTY_NRF_LIBUARTE_READ_BUFFER_SIZE "512" STRING "nrf5sdk dtty libuarte read buffer size")
set_cache_default(NRF5SDK__DTTY_NRF_LIBUARTE_WRITE_BUFFER_SIZE "1024 * 10" STRING "nrf5sdk dtty libuarte write buffer size")

set_cache_default(NRF5SDK__DTTY_NRF_BLE_UART_READ_BUFFER_SIZE "512" STRING "nrf5sdk dtty ble uart read buffer size")
set_cache_default(NRF5SDK__DTTY_NRF_BLE_UART_WRITE_BUFFER_SIZE "1024 * 10" STRING "nrf5sdk ble dtty uart write buffer size")
