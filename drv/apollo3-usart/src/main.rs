// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! A driver for the STM32F4 U(S)ART.
//!
//! # IPC protocol
//!
//! ## `write` (1)
//!
//! Sends the contents of lease #0. Returns when completed.

#![no_std]
#![no_main]

use ambiq_apollo3_pac as device;

use userlib::*;
use zerocopy::AsBytes;

task_slot!(PWRCTRL, pwrctrl_driver);

#[derive(Copy, Clone, Debug, FromPrimitive)]
enum Operation {
    Write = 1,
}

#[repr(u32)]
enum ResponseCode {
    BadArg = 2,
    Busy = 3,
}

// TODO: it is super unfortunate to have to write this by hand, but deriving
// ToPrimitive makes us check at runtime whether the value fits
impl From<ResponseCode> for u32 {
    fn from(rc: ResponseCode) -> Self {
        rc as u32
    }
}

struct Transmit {
    caller: hl::Caller<()>,
    len: usize,
    pos: usize,
}

#[export_name = "main"]
fn main() -> ! {
    // Turn the actual peripheral on so that we can interact with it.
    turn_on_usart();

    // From thin air, pluck a pointer to the USART register block.
    //
    // Safety: this is needlessly unsafe in the API. The USART is essentially a
    // static, and we access it through a & reference so aliasing is not a
    // concern. Were it literally a static, we could just reference it.
    let usart = unsafe { &*device::UART0::ptr() };
    usart.cr.reset();
    usart.cr.modify(|_, w| {
        w.clken()
            .set_bit()
            .clksel()
            ._24mhz()
            .uarten()
            .clear_bit()
            .rxe()
            .clear_bit()
            .txe()
            .clear_bit()
    });

    // // The UART has clock and is out of reset, but isn't actually on until we:
    // usart.cr1.write(|w| w.ue().enabled());
    // Work out our baud rate divisor.
    const BAUDRATE: u32 = 115_200;
    const CLOCK_HZ: u32 = 24_000_000;
    const CYCLES_PER_BIT: u32 = 16;
    const INTEGER_DIVISOR: u32 = CLOCK_HZ / (BAUDRATE * CYCLES_PER_BIT);
    const INTERMEDIATE: u64 =
        (CLOCK_HZ as u64 * 64) / (BAUDRATE * CYCLES_PER_BIT) as u64;
    const FRACTION_DIVISOR: u32 =
        (INTERMEDIATE - INTEGER_DIVISOR as u64 * 64) as u32;
    usart
        .ibrd
        .write(|w| unsafe { w.divint().bits(INTEGER_DIVISOR as u16) });
    usart
        .fbrd
        .write(|w| unsafe { w.divfrac().bits(FRACTION_DIVISOR as u8) });
    usart
        .ifls
        .write(|w| unsafe { w.txiflsel().bits(2).rxiflsel().bits(2) }); // interrupt when FIFO is half full
    usart.lcrh.write(|w| {
        unsafe { w.wlen().bits(3) }
            .pen()
            .clear_bit()
            .stp2()
            .clear_bit()
            .fen()
            .set_bit()
    }); // 8-bit, no parity, 1 stop bit, enable FIFO
    usart
        .cr
        .modify(|_, w| w.uarten().set_bit().rxe().set_bit().txe().set_bit()); // enable uart, enable rx, enable tx

    turn_on_gpio();

    // TODO: the fact that we interact with GPIOA directly here is an expedient
    // hack, but control of the GPIOs should probably be centralized somewhere.
    let gpio = unsafe { &*device::GPIO::ptr() };

    gpio.encb.write(|w| unsafe { w.encb().bits(1 << 16) }); // disable pin 48
    gpio.padkey.write(|w| w.padkey().key());
    gpio.padregm.modify(|_, w| w.pad48fncsel().uart0tx());
    gpio.padkey.write(|w| unsafe { w.bits(0) });

    // Turn on our interrupt. We haven't enabled any interrupt sources at the
    // USART side yet, so this won't trigger notifications yet.
    sys_irq_control(1, true);

    // Field messages.
    let mask = 1;
    let mut tx: Option<Transmit> = None;

    loop {
        hl::recv(
            // Buffer (none required)
            &mut [],
            // Notification mask
            mask,
            // State to pass through to whichever closure below gets run
            &mut tx,
            // Notification handler
            |txref, bits| {
                if bits & 1 != 0 {
                    // Handling an interrupt. To allow for spurious interrupts,
                    // check the individual conditions we care about, and
                    // unconditionally re-enable the IRQ at the end of the handler.

                    let txe = usart.mis.read().txmis().bit_is_set();
                    if txe {
                        usart.iec.modify(|_, w| w.txic().set_bit());
                        // TX register empty. Do we need to send something?
                        step_transmit(&usart, txref);
                    }

                    sys_irq_control(1, true);
                }
            },
            // Message handler
            |txref, op, msg| match op {
                Operation::Write => {
                    // Validate lease count and buffer sizes first.
                    let ((), caller) =
                        msg.fixed_with_leases(1).ok_or(ResponseCode::BadArg)?;

                    // Deny incoming writes if we're already running one.
                    if txref.is_some() {
                        return Err(ResponseCode::Busy);
                    }

                    let borrow = caller.borrow(0);
                    let info = borrow.info().ok_or(ResponseCode::BadArg)?;
                    // Provide feedback to callers if they fail to provide a
                    // readable lease (otherwise we'd fail accessing the borrow
                    // later, which is a defection case and we won't reply at
                    // all).
                    if !info.attributes.contains(LeaseAttributes::READ) {
                        return Err(ResponseCode::BadArg);
                    }

                    // Okay! Begin a transfer!
                    *txref = Some(Transmit {
                        caller,
                        pos: 0,
                        len: info.len,
                    });

                    // OR the TX register empty signal into the USART interrupt.
                    usart.ier.modify(|_, w| w.txim().set_bit()); // enable TX interrupt

                    step_transmit(&usart, txref);

                    // We'll do the rest as interrupts arrive.
                    Ok(())
                }
            },
        );
    }
}

fn turn_on_usart() {
    let pwrctrl_driver = PWRCTRL.get_task_id();

    const ENABLE_PERIPH: u16 = 1;
    let bit = 7; // PWRUART0
    let (code, _) = userlib::sys_send(
        pwrctrl_driver,
        ENABLE_PERIPH,
        bit.as_bytes(),
        &mut [],
        &[],
    );
    assert_eq!(code, 0);

    // const LEAVE_RESET: u16 = 4;
    // let (code, _) = userlib::sys_send(
    //     rcc_driver,
    //     LEAVE_RESET,
    //     pnum.as_bytes(),
    //     &mut [],
    //     &[],
    // );
    // assert_eq!(code, 0);
}

fn turn_on_gpio() {}

fn step_transmit(
    usart: &device::uart0::RegisterBlock,
    tx: &mut Option<Transmit>,
) {
    // Clearer than just using replace:
    fn end_transmission(
        usart: &device::uart0::RegisterBlock,
        state: &mut Option<Transmit>,
    ) -> hl::Caller<()> {
        usart.ier.modify(|_, w| w.txim().clear_bit()); // disable TX interrupt
        core::mem::replace(state, None).unwrap().caller
    }

    let txs = if let Some(txs) = tx { txs } else { return };

    while usart.fr.read().txff().bit_is_clear() {
        if let Some(byte) = txs.caller.borrow(0).read_at::<u8>(txs.pos) {
            // Stuff byte into transmitter.
            usart.dr.write(|w| unsafe { w.data().bits(byte) });

            txs.pos += 1;
            if txs.pos == txs.len {
                end_transmission(usart, tx).reply(());
                break;
            }
        } else {
            end_transmission(usart, tx).reply_fail(ResponseCode::BadArg);
            break;
        }
    }
}
