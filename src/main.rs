#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

extern crate cast;
extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate heapless;
extern crate panic_semihosting;
extern crate stm32f3xx_hal;

#[rtic::app(device = stm32f3xx_hal::pac, peripherals = true)]
mod app {
    use rtt_target::{rtt_init_print, rprintln};
    use core::convert::TryInto;
    use heapless::String;
    use stm32f3xx_hal::{
        gpio::{gpiob, Alternate, Gpiob, OpenDrain, Pin, U},
        i2cint::{I2cInt, State},
        pac::I2C1,
        prelude::*,
        rcc::RccExt,
    };

    const I2C_LSM_ADDR: u8 = 0b0011110;
    const LSM_MAG_CONTINUOUS: [u8; 2] = [0x02, 0b00];
    const LSM_ENABLE_WRITE: [u8; 2] = [0, 0b10010000];
    const LSM_MAG_TEMP_H_REG_WRITE: [u8; 1] = [0x31];
    // const LSM_MAG_TEMP_L_REG_WRITE: [u8; 1] = [0x32];

    enum TaskMode {
        Write,
        // Read,
        WriteRead,
    }

    struct I2cTask {
        pub mode: TaskMode,
        pub addr: u8,
        pub buf: &'static [u8],
    }

    const TASKS: [I2cTask; 3] = [
        I2cTask {
            // Configure magnetic sensor operating mode to continuous mode
            mode: TaskMode::Write,
            addr: I2C_LSM_ADDR,
            buf: &LSM_MAG_CONTINUOUS,
        },
        I2cTask {
            // Enable temperature sensor
            mode: TaskMode::Write,
            addr: I2C_LSM_ADDR,
            buf: &LSM_ENABLE_WRITE,
        },
        I2cTask {
            // Read the content of temperature output register
            mode: TaskMode::WriteRead,
            addr: I2C_LSM_ADDR,
            buf: &LSM_MAG_TEMP_H_REG_WRITE,
        },
    ];

    type I2cInt1 = I2cInt<
        I2C1,
        (
            Pin<Gpiob, U<6>, Alternate<OpenDrain, 4>>,
            Pin<Gpiob, U<7>, Alternate<OpenDrain, 4>>,
        ),
    >;

    #[shared]
    struct Shared {
        i2c1: I2cInt1,
    }

    #[local]
    struct Local {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        rtt_init_print!();
        let device: stm32f3xx_hal::pac::Peripherals = cx.device;
        let mut rcc = device.RCC.constrain();

        let mut flash = device.FLASH.constrain();
        let clocks = rcc.cfgr.freeze(&mut flash.acr);

        rprintln!("init: configure gpio for i2c1");
        let mut gpiob: gpiob::Parts = device.GPIOB.split(&mut rcc.ahb);
        let pins = (
            gpiob
                .pb6
                .into_af_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl), // SCL
            gpiob
                .pb7
                .into_af_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl), // SDA
        );

        let mut i2c1 = I2cInt::new_int(
            device.I2C1,
            pins,
            100.kHz().try_into().unwrap(),
            clocks,
            &mut rcc.apb1,
        );
        rprintln!("init: enable interrupts and device for i2c1");
        i2c1.enable_interrupts();
        i2c1.enable();

        (Shared { i2c1 }, Local {}, init::Monotonics())
    }

    #[idle(shared = [i2c1])]
    fn idle(mut cx: idle::Context) -> ! {
        let mut task_index = 0;
        let mut task_running = false;
        loop {
            if !task_running && task_index < TASKS.len() {
                let task = &TASKS[task_index];
                match task.mode {
                    TaskMode::Write => {
                        task_running = true;
                        rprintln!("idle: write to i2c1 device :{:08b}", task.addr);
                        rprintln!("idle: task : {} send to {:08b} !!!", task_index, task.addr);
                        for byte in task.buf {
                            rprintln!("idle: send buf: {:08b}", byte);
                        }
                        cx.shared.i2c1.lock(|i2c1: &mut I2cInt1| {
                            i2c1.write(task.addr, task.buf).unwrap();
                        });
                    }
                    TaskMode::WriteRead => {
                        task_running = true;
                        rprintln!("idle: write_read to i2c1 device : {:08b}", task.addr);
                        rprintln!(
                            "idle: task : {} write read from {:08b} !!!",
                            task_index,
                            task.addr
                        );
                        cx.shared.i2c1.lock(|i2c1: &mut I2cInt1| {
                            i2c1.write_read(task.addr, task.buf, 2).unwrap();
                        });
                    }
                }
                task_index += 1;
            }
            cx.shared
                .i2c1
                .lock(|i2c1: &mut I2cInt1| match i2c1.last_error {
                    Some(_) => {
                        rprintln!("idle: stop with error !!!");
                        i2c1.reset_state();
                    }
                    None => match i2c1.get_tx_state() {
                        State::Idle => {}
                        State::TxStop => {
                            i2c1.reset_state();
                            task_running = false;
                        }
                        State::RxStop => {
                            let buf = i2c1.get_rx_buf(2);
                            for byte in buf {
                                rprintln!("idle: rx stop: buf: {:08b}", byte);
                            }
                            i2c1.reset_state();
                            task_running = false;
                        }
                        _ => {
                            rprintln!("idle: isr state not managed");
                        }
                    },
                });
        }
    }

    #[task(binds = I2C1_EV_EXTI23, shared = [i2c1])]
    fn i2c1_ev(mut cx: i2c1_ev::Context) {
        cx.shared.i2c1.lock(|i2c1| {
            rprintln!("I2C1_EV: {:08b}", i2c1.get_isr());
            i2c1.interrupt();
            let mut s: String<16> = String::new();
            core::fmt::write(&mut s, format_args!("{}", i2c1.get_tx_state())).unwrap();
            rprintln!("i2c1 tx state = {}", s);
        });
    }

    #[task(binds = I2C1_ER, shared = [i2c1])]
    fn i2c1_er(mut cx: i2c1_er::Context) {
        cx.shared.i2c1.lock(|i2c1| {
            rprintln!("I2C1_ER: {:08b}", i2c1.get_isr());
            i2c1.interrupt();
        })
    }
}
