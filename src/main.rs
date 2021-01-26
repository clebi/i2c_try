#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

extern crate cast;
extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate panic_semihosting;
extern crate stm32f3xx_hal;
extern crate heapless;

// mod i2c.rs;

use stm32f3xx_hal::{
    i2cint::{I2cInt, State},
    gpio::{gpiob, AF4},
    pac::{I2C1},
    prelude::*,
};

use defmt_rtt as _;
use heapless::String;
use heapless::consts::*;
use core::fmt;

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

type I2cInt1 = I2cInt<I2C1, (gpiob::PB6<AF4>, gpiob::PB7<AF4>)>;

#[rtic::app(device = stm32f3xx_hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        i2c1: I2cInt1
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        let device: stm32f3xx_hal::pac::Peripherals = cx.device;
        let mut rcc = device.RCC.constrain();

        let mut flash = device.FLASH.constrain();
        let clocks = rcc.cfgr.freeze(&mut flash.acr);

        defmt::debug!("init: configure gpio for i2c1");
        let mut gpiob: gpiob::Parts = device.GPIOB.split(&mut rcc.ahb);
        let pins = (
            gpiob.pb6.into_af4(&mut gpiob.moder, &mut gpiob.afrl), // SCL
            gpiob.pb7.into_af4(&mut gpiob.moder, &mut gpiob.afrl), // SDA
        );


        let mut i2c1 = I2cInt::new_int(device.I2C1, pins, 100.khz(), clocks, &mut rcc.apb1);
        defmt::debug!("init: enable interrupts and device for i2c1");
        i2c1.enable_interrupts();
        i2c1.enable();

        init::LateResources { i2c1: i2c1 }
    }

    #[idle(resources = [i2c1])]
    fn idle(mut cx: idle::Context) -> ! {
        let mut task_index = 0;
        let mut task_running = false;
        loop {
            if !task_running && task_index < TASKS.len() {
                let task = &TASKS[task_index];
                match task.mode {
                    TaskMode::Write => {
                        task_running = true;
                        defmt::info!("idle: write to i2c1 device : {:0..7}", task.addr);
                        defmt::debug!("idle: task : {:usize} send to {:u8} !!!", task_index, task.addr);
                        for byte in task.buf {
                            defmt::debug!("idle: send buf: {:u8}", byte);
                        }
                        cx.resources.i2c1.lock(|i2c1: &mut I2cInt1| {
                            i2c1.write(task.addr, task.buf).unwrap();
                        });
                    }
                    TaskMode::WriteRead => {
                        task_running = true;
                        defmt::info!("idle: write_read to i2c1 device : {:0..7}", task.addr);
                        defmt::debug!(
                            "idle: task : {:usize} write read from {:u8} !!!",
                            task_index,
                            task.addr
                        );
                        cx.resources.i2c1.lock(|i2c1: &mut I2cInt1| {
                            i2c1.write_read(task.addr, task.buf, 2).unwrap();
                        });
                    }
                }
                task_index += 1;
            }
            cx.resources
                .i2c1
                .lock(|i2c1: &mut I2cInt1| match i2c1.last_error {
                    Some(_) => {
                        defmt::error!("idle: stop with error !!!");
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
                                defmt::info!("idle: rx stop: buf: {:u8}", byte);
                            }
                            i2c1.reset_state();
                            task_running = false;
                        }
                        _ => {
                            defmt::debug!("idle: isr state not managed");
                        }
                    },
                });
        }
    }

    #[task(binds = I2C1_EV_EXTI23, resources = [i2c1])]
    fn i2c1_ev(cx: i2c1_ev::Context) {
        let i2c1: &mut I2cInt1 = cx.resources.i2c1;
        defmt::debug!("I2C1_EV: {:0..7}", i2c1.get_isr());
        i2c1.interrupt();
        let mut s: String<U16> = String::new();
        fmt::write(&mut s, format_args!("{}", i2c1.get_tx_state())).unwrap();
        defmt::debug!("i2c1 tx state = {:str}", s);
    }

    #[task(binds = I2C1_ER, resources = [i2c1])]
    fn i2c1_er(cx: i2c1_er::Context) {
        let i2c1: &mut I2cInt1 = cx.resources.i2c1;
        defmt::debug!("I2C1_ER: {:0..7}", i2c1.get_isr());
        i2c1.interrupt();
    }
};
