//! IRIS kinematically coupled 3-axis piezo focus stage controller
//!
#![no_std]
#![no_main]

use core::fmt::Write;

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use iris_focus_hal as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
    spi::Spi,
    gpio::FunctionSpi,
    uart::{DataBits, StopBits, UartConfig},
    timer::Timer,
};

use embedded_hal::spi::MODE_2; // inverted clock, changes on rising edge, samples on falling
use fugit::RateExtU32;

use embedded_hal::PwmPin;

static IRIS_SID: &'static str = "V1 PIEZO\n";
static CMD_TERM: &'static str = "OK\n";
const MAX_CMD_LEN: usize = 256;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // ------ configure a timer
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);

    // ------ configure PWMs
    let mut pwm_slices = bsp::hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // Configure PWM4
    let pwm = &mut pwm_slices.pwm4;
    pwm.set_ph_correct();
    pwm.enable();

    // Output channel B on PWM4 to GPIO 25
    let channel = &mut pwm.channel_b;
    channel.output_to(pins.led);

    // ------ configure UART
    let uart_pins = (
        // UART TX (characters sent from RP2040) on pin 1 (GPIO0)
        pins.gpio20.into_mode(),
        // UART RX (characters received by RP2040) on pin 2 (GPIO1)
        pins.gpio21.into_mode(),
        // RTS/CTS not used because they are swapped on the prototype
        // CTS,
        // pins.gpio22.into_mode(),
        // RTS,
        // pins.gpio23.into_mode(),
    );
    let mut uart = bsp::hal::uart::UartPeripheral::new(pac.UART1, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    // ------ configure SPI
    let mut spi_csn = pins.gpio1
    .into_push_pull_output_in_state(embedded_hal::digital::v2::PinState::High);
    pins.gpio2.into_mode::<FunctionSpi>();
    pins.gpio3.into_mode::<FunctionSpi>();
    pins.gpio4.into_mode::<FunctionSpi>();
    let mut spi = Spi::<_, _, 8>::new(pac.SPI0).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        25_000_000u32.Hz(), // max is 50 MHz
        &MODE_2,
    );
    // software reset
    write_dac(&mut spi, &mut spi_csn, &mut delay, 5, 0xA);
    delay.delay_ms(1); // wait for reset to go through

    // check ID of DAC
    let id = read_dac(&mut spi, &mut spi_csn, &mut delay, 1);
    if id != 0x1417 {
        error!("Incorrect DAC ID: {:?}", id);
    } else {
        info!("DAC OK: {:?}", id);
    }
    // setup gain: refdiv/2, gain = 1
    write_dac(&mut spi, &mut spi_csn, &mut delay, 4, 0x100);
    // set to broadcast mode, dacs 0-2 + sync update
    write_dac(&mut spi, &mut spi_csn, &mut delay, 2, 0x0007);
    // power down DAC 3
    write_dac(&mut spi, &mut spi_csn, &mut delay, 3, 0x8);

    // ------ configure piezo enable GPIO
    let mut piezo_enable = pins.gpio5.into_push_pull_output();
    piezo_enable.set_low().unwrap();

    // ------ main loop
    uart.write_str(IRIS_SID).ok();
    info!("{}", IRIS_SID); // also send a copy to the debug console
    const BRIGHT_MIN: u16 = 0;
    const BRIGHT_MAX: u16 = 25000;
    const BRIGHT_RATE: u16 = 1;
    let mut brightness: u16 = 0;
    let mut going_up = true;
    // we're in no-std, so...we have to statically allocate our incoming buffer.
    // we don't get a Vec, so we have to manage this old-school.
    let mut cmdbuf = [0u8; MAX_CMD_LEN];
    let mut cmdbuf_index = 0;
    let mut last_instant = timer.get_counter();
    let mut last_all_value = 0;
    // let mut test: u16 = 0;
    loop {
        let this_instant = timer.get_counter();
        if this_instant.checked_duration_since(last_instant).unwrap().to_micros() > 60 {
            let next_bright = if going_up {
                let next_bright = brightness.saturating_add(BRIGHT_RATE);
                if next_bright >= BRIGHT_MAX {
                    going_up = false;
                }
                next_bright
            } else {
                let next_bright = brightness.saturating_sub(BRIGHT_RATE);
                if next_bright <= BRIGHT_MIN {
                    going_up = true;
                }
                next_bright
            };
            channel.set_duty(next_bright);
            brightness = next_bright;
            last_instant = this_instant;
        }

        while uart.uart_is_readable() {
            match uart.read_raw(&mut cmdbuf[cmdbuf_index..]) {
                Ok(len) => {
                    cmdbuf_index += len;
                    // info!("got {} chars", len);
                },
                Err(_e) => error!("Uart read error, ignoring"),
            }
        }
        // cmdbuf_index should not be able to overflow because the bounds is specified by the slice
        // indices passed into read_row()
        // E - enable HV
        // D - disable HV
        // A n - set all axes to a value n, which is a decimal from 0-16383
        // X n - set channel 0 to n
        // Y and Z are TBD
        // I - return the DAC ID in binary
        match extract_verb(&cmdbuf[..cmdbuf_index]) {
            Some(verb) => {
                let tokens = verb.split(' ');
                let mut verb: Option<&str> = None;
                for t in tokens {
                    match t {
                        "?" => {
                            uart.write_str(IRIS_SID).ok();
                        }
                        "A" | "X" | "Y" | "Z" => {
                            verb = Some(t);
                        }
                        "E" => {
                            piezo_enable.set_high().ok();
                        }
                        "D" => {
                            piezo_enable.set_low().ok();
                        }
                        "I" => {
                            // serialize this out as binary because...embedded
                            // Rust is dumb and it has no String/Vec and the UART
                            // object does not implement a write! trait.
                            for i in 0..16 {
                                if (id >> (15 - i)) & 1 == 0 {
                                    uart.write_char('0').ok();
                                } else {
                                    uart.write_char('1').ok();
                                }
                            }
                            uart.write_char('\n').ok();
                        }
                        _ => {
                            match verb {
                                // "A" - set all channels
                                Some("A") => {
                                    match u16::from_str_radix(t, 10) {
                                        Ok(v) => {
                                            if v == last_all_value {
                                                // don't do anything if there is no change
                                                continue;
                                            }

                                            // transition between values through a number of steps, to try
                                            // to prevent shaking the samples too much.
                                            const TARGET_STEPS: i32 = 800;
                                            let mut step = (v as i32 - last_all_value as i32) / TARGET_STEPS;
                                            if step == 0 {
                                                // make sure we at least step by 1; we'll terminate early if necessary
                                                if v > last_all_value {
                                                    step = 1;
                                                } else {
                                                    step = -1;
                                                }
                                            }
                                            let mut value = last_all_value as i32 + step;
                                            for _ in 0..TARGET_STEPS {
                                                // check if we need to terminate
                                                let terminate = if step > 0 {
                                                    if value >= v as i32 {
                                                        value = v as i32;
                                                        true
                                                    } else {
                                                        false
                                                    }
                                                } else {
                                                    if value <= v as i32 {
                                                        value = v as i32;
                                                        true
                                                    } else {
                                                        false
                                                    }
                                                };
                                                // sanity check, because wrap-arounds would be really bad.
                                                if value < 0 {
                                                    value = 0;
                                                }
                                                write_dac(&mut spi, &mut spi_csn, &mut delay, 0x8, (value as u16) << 2);
                                                write_dac(&mut spi, &mut spi_csn, &mut delay, 0x9, (value as u16) << 2);
                                                write_dac(&mut spi, &mut spi_csn, &mut delay, 0xA, (value as u16) << 2);
                                                write_dac(&mut spi, &mut spi_csn, &mut delay, 0x5, 0b1_0000);
                                                delay.delay_us(20);
                                                if terminate {
                                                    break
                                                }
                                                value += step;
                                            }
                                            if value != v as i32 {
                                                // we didn't hit the final value because of rounding. Add one more step
                                                // to get us there.
                                                write_dac(&mut spi, &mut spi_csn, &mut delay, 0x8, v << 2);
                                                write_dac(&mut spi, &mut spi_csn, &mut delay, 0x9, v << 2);
                                                write_dac(&mut spi, &mut spi_csn, &mut delay, 0xA, v << 2);
                                                write_dac(&mut spi, &mut spi_csn, &mut delay, 0x5, 0b1_0000);
                                            }
                                            last_all_value = v;
                                        }
                                        _ => {
                                            uart.write_str("ARG?\n").ok();
                                        }
                                    }
                                }
                                Some("X") => {
                                    match u16::from_str_radix(t, 10) {
                                        Ok(v) => {
                                            write_dac(&mut spi, &mut spi_csn, &mut delay, 0x8, v << 2);
                                            write_dac(&mut spi, &mut spi_csn, &mut delay, 0x5, 0b1_0000);
                                        }
                                        _ => {
                                            uart.write_str("ARG?\n").ok();
                                        }
                                    }
                                }
                                // TODO: Y, Z; modularize the above code some first...
                                _ => {
                                    uart.write_str("CMD?\n").ok();
                                }
                            }
                        }
                    }
                }
                uart.write_str(CMD_TERM).ok();
                // clear the buf for next iteration
                cmdbuf_index = 0;
                cmdbuf.fill(0);
            }
            None => {
                // ignore
            }
        }

        // testing routine
        /*
        write_dac(&mut spi, &mut spi_csn, &mut delay, 0x8, test);
        write_dac(&mut spi, &mut spi_csn, &mut delay, 0x9, 65535 - test);
        write_dac(&mut spi, &mut spi_csn, &mut delay, 0xA, test);
        // trigger the DAC
        write_dac(&mut spi, &mut spi_csn, &mut delay, 0x5, 0b1_0000);

        test = test.wrapping_add(256);
        // let id = read_dac(&mut spi, &mut spi_csn, &mut delay, 1);
        // println!("id: {:x}, test: {}", id, test);
        */
    }
}

/// Commands are of the format 'verb\n'. Anything after \n is ignored.
/// The sender must wait until it receives an 'OK\n' before sending the next command.
fn extract_verb(buf: &[u8]) -> Option<&str> {
    // find the position of the \n, which delimits the verb.
    let mut cmd_end = 0;
    for (i, src) in buf.iter().enumerate() {
        if *src == b'\n' || *src == b'\r' {
            cmd_end = i;
            break;
        }
    }
    if cmd_end != 0 {
        Some(core::str::from_utf8(&buf[..cmd_end]).unwrap_or("INVALID"))
    } else {
        None
    }
}

/// Format write values for the DAC70504
fn write_dac <S, P> (
    spi: &mut S,
    spi_csn: &mut P,
    delay: &mut cortex_m::delay::Delay,
    addr: u8,
    dat: u16)
where
    // figuring out these type bounds were a bitch. For my future self:
    // 1. The type bounds are in embedded_hal, not bsp::hal.
    // 2. You just need to specify the bounds for the operation you need on the peripheral;
    // not for the entire peripheral (specify 'type bound' on just what you need, and no more)
    // 3. Look in bsp::hal to find the method signature of the macro trait, then resolve
    // it backwards to the specific part of embedded_hal that you're using as the bounds.
    S: embedded_hal::blocking::spi::Transfer<u8>,
    P: embedded_hal::digital::v2::OutputPin,
{
    let mut wd = [
        0b0000_0000u8 | addr & 0xF,
        (dat >> 8) as u8,
        dat as u8,
    ];
    spi_csn.set_low().ok();
    let _ = spi.transfer(&mut wd).ok();
    spi_csn.set_high().ok();
    delay.delay_us(1);
}

fn read_dac<S, P> (
    spi: &mut S,
    spi_csn: &mut P,
    delay: &mut cortex_m::delay::Delay,
    addr: u8
) -> u16
where
    S: embedded_hal::spi::FullDuplex<u8>
    + embedded_hal::blocking::spi::Write<u8>
    + embedded_hal::blocking::spi::Transfer<u8>,
    P: embedded_hal::digital::v2::OutputPin,
{
    let rd = [
        0b1000_0000u8 | addr & 0xF,
        0,
        0,
    ];
    // send over the command to read
    spi_csn.set_low().ok();
    spi.write(&rd).ok();
    spi_csn.set_high().ok();
    // wait to turn-around the DAC
    delay.delay_us(1);

    // now recover the data by sending NOPs
    spi_csn.set_low().ok();
    let mut ret = rd.clone();
    spi.transfer(&mut ret).ok();
    spi_csn.set_high().ok();

    (ret[1] as u16) << 8 | (ret[2] as u16)
}
