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
use i2c::{I2c1, TxState};
use stm32f3xx_hal::prelude::*;

fn tx_state_str(state: TxState) -> &'static str {
    match state {
        TxState::None => "None",
        TxState::TxStart => "TxStart",
        TxState::TxReady => "TxReady",
        TxState::TxSent => "TxSent",
        TxState::TxComplete => "TxComplete",
        TxState::TxStop => "TxStop",
        TxState::TxNack => "TxNack",
    }
}

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
        hprintln!("write to i2c1 device : 0b0011110",).unwrap();
        cx.resources.i2c1.lock(|i2c1| {
            i2c1.write_start(0b0011110, 1);
        });
        loop {
            cx.resources
                .i2c1
                .lock(|i2c1: &mut I2c1| match i2c1.get_tx_state() {
                    TxState::None => {}
                    TxState::TxStop => {
                        hprintln!("idle: tx stop, end !!!").unwrap();
                        i2c1.reset_tx_state();
                    }
                    _ => {
                        hprintln!("idle: isr state not managed").unwrap();
                    }
                });
        }
    }

    #[task(binds = I2C1_EV_EXTI23, resources = [i2c1])]
    fn i2c1_ev(cx: i2c1_ev::Context) {
        let i2c1: &mut I2c1 = cx.resources.i2c1;
        hprintln!("i2c1 tx state = {}", tx_state_str(i2c1.get_tx_state())).unwrap();
        hprintln!("I2C1_EV: 0b{:032b}", i2c1.get_isr()).unwrap();
        let isr_state = i2c1.isr_state();
        match isr_state {
            TxState::TxReady => {
                hprintln!("I2C1_EV: write buffer is ready").unwrap();
                i2c1.write_tx_buffer(0);
            }
            TxState::TxStop => {
                hprintln!("I2C1_EV: write end").unwrap();
                i2c1.clear_stop_int();
            }
            _ => hprintln!("I2C1_EV: isr state not managed").unwrap(),
        }
    }

    #[task(binds = I2C1_ER, resources = [i2c1])]
    fn i2c1_er(cx: i2c1_er::Context) {
        let _: &mut I2c1 = cx.resources.i2c1;
        hprintln!("I2C1 ER").unwrap();
    }
};
