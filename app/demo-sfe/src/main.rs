// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#![no_std]
#![no_main]

#[cfg(not(any(feature = "panic-itm", feature = "panic-semihosting")))]
compile_error!(
    "Must have either feature panic-itm or panic-semihosting enabled"
);

// Panic behavior controlled by Cargo features:
#[cfg(feature = "panic-itm")]
extern crate panic_itm; // breakpoint on `rust_begin_unwind` to catch panics
#[cfg(feature = "panic-semihosting")]
extern crate panic_semihosting; // requires a debugger

// We have to do this if we don't otherwise use it to ensure its vector table
// gets linked in.
#[cfg(feature = "ambiq-apollo3-pac")]
extern crate ambiq_apollo3_pac;
#[cfg(feature = "stm32f3")]
extern crate stm32f3;
#[cfg(feature = "stm32f4")]
extern crate stm32f4;
use ambiq_apollo3_pac::{
    cachectrl, clkgen::clkkey, CACHECTRL, CLKGEN, GPIO, MCUCTRL, PWRCTRL,
};

use cortex_m::peripheral;
use cortex_m_rt::entry;
use kern::app::App;

extern "C" {
    static hubris_app_table: App;
    static mut __sheap: u8;
    static __eheap: u8;
}

fn init() {
    let scb = unsafe { &*peripheral::SCB::ptr() };
    if scb.vtor.read() != 0xc000 {
        return;
    }
    // let (clkgen, mcuctrl, pwrctrl, cachectrl) = unsafe { (&*CLKGEN::ptr(), &*MCUCTRL::ptr(), &*PWRCTRL::ptr(), &*CACHECTRL::ptr()) };

    // // Set the clock frequency.
    // clkgen.clkkey.write(|w| w.clkkey().key());
    // clkgen.cctrl.write(|w| w.coresel().hfrc());
    // clkgen.clkkey.write(|w| unsafe { w.bits(0) });

    // // Set the default cache configuration
    // // am_hal_cachectrl_config(&am_hal_cachectrl_defaults);
    // // am_hal_cachectrl_enable();

    // if mcuctrl.chiprev.read().revmaj().is_a()
    //     && mcuctrl.chiprev.read().revmin().is_rev1()
    //     && pwrctrl.supplystatus.read().simobuckon().is_on()
    // {
    //     pwrctrl.devpwren.modify(|_, w| w.pwrpdm().en());
    // }
    // // MCUCTRL->SIMOBUCK4_b.SIMOBUCKCOMP2TIMEOUTEN = 0;
    // mcuctrl
    //     .simobuck4
    //     .modify(|r, w| unsafe { w.bits(r.bits() & !(1_u32 << 24)) });

    // cachectrl
    //     .cachecfg
    //     .modify(|_, w| w.dcache_enable().clear_bit().icache_enable().clear_bit());
    // cachectrl.cachecfg.write(|w| {
    //     w.enable()
    //         .clear_bit()
    //         .cache_clkgate()
    //         .set_bit()
    //         .cache_ls()
    //         .clear_bit()
    //         .data_clkgate()
    //         .set_bit()
    //         .enable_monitor()
    //         .clear_bit()
    //         .lru()
    //         .clear_bit()
    //         .config()
    //         .w1_128b_1024e()
    //         .dcache_enable()
    //         .set_bit()
    //         .icache_enable()
    //         .set_bit()
    // });
    // cachectrl.cachecfg.modify(|_, w| w.enable().set_bit());
    let (gpio, cachectrl, clkgen) =
        unsafe { (&*GPIO::ptr(), &*CACHECTRL::ptr(), &*CLKGEN::ptr()) };
    let pwrctrl = unsafe { &*PWRCTRL::ptr() };
    let mcuctrl = unsafe { &*MCUCTRL::ptr() };

    clkgen.clkkey.write(|w| w.clkkey().key());
    clkgen.cctrl.write(|w| w.coresel().hfrc());
    clkgen.clkkey.write(|w| {
        *w = clkkey::W::reset_value();
        w
    });

    cachectrl.cachecfg.modify(|_, w| {
        w.dcache_enable().clear_bit().icache_enable().clear_bit()
    });
    cachectrl.cachecfg.write(|w| {
        w.enable()
            .clear_bit()
            .cache_clkgate()
            .set_bit()
            .cache_ls()
            .clear_bit()
            .data_clkgate()
            .set_bit()
            .enable_monitor()
            .clear_bit()
            .lru()
            .clear_bit()
            .config()
            .w1_128b_1024e()
            .dcache_enable()
            .set_bit()
            .icache_enable()
            .set_bit()
    });
    cachectrl.cachecfg.modify(|_, w| w.enable().set_bit());

    if mcuctrl.chiprev.read().revmaj().is_a()
        && mcuctrl.chiprev.read().revmin().is_rev1()
        && pwrctrl.supplystatus.read().simobuckon().is_on()
    {
        pwrctrl.devpwren.modify(|_, w| w.pwrpdm().en());
    }

    mcuctrl
        .simobuck4
        .modify(|r, w| unsafe { w.bits(r.bits() | 1 << 24) });

    clkgen.octrl.modify(|_, w| w.stopxt().stop());

    gpio.encb.write(|w| unsafe {
        w.encb().bits(1 << 14 | 1 << 5 | 1 << 15 | 1 << 12)
    });

    gpio.padkey.write(|w| w.padkey().key());

    gpio.padregj
        .write(|w| w.pad37fncsel().gpio37().pad37strng().low());

    gpio.padregl.write(|w| {
        w.pad46fncsel()
            .gpio46()
            .pad46strng()
            .low()
            .pad47fncsel()
            .gpio47()
            .pad47strng()
            .low()
            .pad44fncsel()
            .gpio44()
            .pad44strng()
            .low()
    });

    gpio.cfge.write(|w| w.gpio37outcfg().pushpull());
    gpio.cfgf.write(|w| {
        w.gpio46outcfg()
            .pushpull()
            .gpio47outcfg()
            .pushpull()
            .gpio44outcfg()
            .pushpull()
    });

    gpio.padkey.write(|w| unsafe { w.bits(0) });

    gpio.wtsb
        .write(|w| unsafe { w.bits(1 << 14 | 1 << 5 | 1 << 15 | 1 << 12) });
}

#[entry]
fn main() -> ! {
    // Default boot speed, until we bother raising it:
    #[cfg(feature = "stm32f3")]
    const CYCLES_PER_MS: u32 = 8_000;
    #[cfg(feature = "stm32f4")]
    const CYCLES_PER_MS: u32 = 16_000;
    #[cfg(feature = "ambiq-apollo3-pac")]
    const CYCLES_PER_MS: u32 = 48_000;

    init();
    unsafe {
        let heap_size =
            (&__eheap as *const _ as usize) - (&__sheap as *const _ as usize);
        kern::startup::start_kernel(
            &hubris_app_table,
            (&mut __sheap) as *mut _,
            heap_size,
            CYCLES_PER_MS,
        )
    }
}
