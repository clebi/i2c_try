# i2c_try

`i2c_try` is a attempt to implement interrupt driven i2c communications. The project uses `cortex-m-rtic` to manage concurrency and consumes `stm32f3xx-hal` to access peripherals.
This project is under active developmement to figure the best way to run interrupt driven i2c communications.

The project is develop on a STM32F3DISCOVERY board.

## Requirements

- Rust 1.36.0+

## License

Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  [https://www.apache.org/licenses/LICENSE-2.0][L1])
