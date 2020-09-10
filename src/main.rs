#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

extern crate cast;
extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate panic_semihosting;
extern crate stm32f3xx_hal;

mod i2c;

use cortex_m_semihosting::hprintln;
use i2c::{I2c1, State};
use stm32f3xx_hal::prelude::*;

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

#[rtic::app(device = stm32f3xx_hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        i2c1: I2c1,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        let device: stm32f3xx_hal::pac::Peripherals = cx.device;
        let mut rcc = device.RCC.constrain();

        let mut flash = device.FLASH.constrain();
        let clocks = rcc.cfgr.freeze(&mut flash.acr);

        let mut gpiob: stm32f3xx_hal::gpio::gpiob::Parts = device.GPIOB.split(&mut rcc.ahb);
        let scl = gpiob.pb6.into_af4(&mut gpiob.moder, &mut gpiob.afrl);
        let sda = gpiob.pb7.into_af4(&mut gpiob.moder, &mut gpiob.afrl);

        let mut i2c1 = I2c1::new(
            device.I2C1,
            (scl, sda),
            400.khz().into(),
            clocks,
            &mut rcc.apb1,
        );
        i2c1.enable_interrupts();
        i2c1.enable();

        init::LateResources { i2c1: i2c1 }
    }

    #[idle(resources = [i2c1])]
    fn idle(mut cx: idle::Context) -> ! {
        let mut task_index = 0;
        let mut task_running = false;
        hprintln!("write to i2c1 device : 0b0011110",).unwrap();
        loop {
            if !task_running && task_index < TASKS.len() {
                let task = &TASKS[task_index];
                match task.mode {
                    TaskMode::Write => {
                        task_running = true;
                        hprintln!("idle: task : {} send to {:#08b} !!!", task_index, task.addr)
                            .unwrap();
                        for byte in task.buf {
                            hprintln!("idle: send buf: {:#08b}", byte).unwrap();
                        }
                        cx.resources.i2c1.lock(|i2c1: &mut I2c1| {
                            i2c1.write(task.addr, task.buf).unwrap();
                        });
                    }
                    TaskMode::WriteRead => {
                        task_running = true;
                        hprintln!(
                            "idle: task : {} write read from {:#08b} !!!",
                            task_index,
                            task.addr
                        )
                        .unwrap();
                        cx.resources.i2c1.lock(|i2c1: &mut I2c1| {
                            i2c1.write_read(task.addr, task.buf, 1).unwrap();
                        });
                    }
                }
                task_index += 1;
            }
            cx.resources
                .i2c1
                .lock(|i2c1: &mut I2c1| match i2c1.last_error {
                    Some(_) => {
                        hprintln!("idle: stop with error !!!").unwrap();
                        i2c1.reset_state();
                    }
                    None => match i2c1.get_tx_state() {
                        State::Idle => {}
                        State::TxStop => {
                            i2c1.reset_state();
                            task_running = false;
                        }
                        State::RxStop => {
                            let buf = i2c1.get_rx_buf(1);
                            for byte in buf {
                                hprintln!("idle: rx stop: buf: {:#08b}", byte).unwrap();
                            }
                            i2c1.reset_state();
                            task_running = false;
                        }
                        _ => {
                            hprintln!("idle: isr state not managed").unwrap();
                        }
                    },
                });
        }
    }

    #[task(binds = I2C1_EV_EXTI23, resources = [i2c1])]
    fn i2c1_ev(cx: i2c1_ev::Context) {
        let i2c1: &mut I2c1 = cx.resources.i2c1;
        hprintln!("I2C1_EV: {:#016b}", i2c1.get_isr()).unwrap();
        i2c1.interrupt();
        hprintln!("i2c1 tx state = {}", i2c1.get_tx_state()).unwrap();
    }

    #[task(binds = I2C1_ER, resources = [i2c1])]
    fn i2c1_er(cx: i2c1_er::Context) {
        let i2c1: &mut I2c1 = cx.resources.i2c1;
        hprintln!("I2C1 ER").unwrap();
        i2c1.interrupt();
    }
};
