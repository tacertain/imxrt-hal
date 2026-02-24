//! HID keyboard input example using RTIC v2.
//!
//! Enumerates a USB HID keyboard on the USB2 host port, allocates an
//! interrupt IN pipe on its first interrupt endpoint, and logs each raw
//! HID report received. When an alphanumeric key is pressed, the
//! on-board LED flashes its morse code equivalent.
//!
//! This example does **not** support USB hubs. If a hub is connected it
//! will panic with a descriptive message. See [`rtic_usb_hub`] for an
//! example with hub support.
//!
//! # What it does
//!
//! 1. Initialises USB2 host controller (same as `rtic_usb_enumerate`).
//! 2. Waits for a device connect event via `UsbBus::device_events_no_hubs()`.
//! 3. On `DeviceEvent::Connect`, panics if the device is a hub (class 9).
//! 4. Uses `UsbBus::get_configuration()` with a `DescriptorVisitor` to find
//!    the first interrupt IN endpoint.
//! 5. Calls `UsbBus::configure()` which issues SET_CONFIGURATION.
//! 6. Calls `UsbBus::interrupt_endpoint_in()` to open an interrupt IN stream.
//! 7. Polls the stream in a loop, logging each decoded keypress and flashing
//!    the LED in morse code for alphanumeric keys.
//!
//! # Expected output (with a USB keyboard plugged in)
//!
//! ```text
//! === imxrt-usbh: HID Keyboard Example ===
//! USB2 PLL locked
//! VBUS power enabled
//! USB host controller initialised
//! USB_OTG2 ISR installed (NVIC priority 0xE0)
//! Entering device event loop...
//! DeviceEvent::Connect  addr=1  VID=045e PID=00db class=0 subclass=0
//! Found HID interface: iface=0 ep=1 mps=8 interval=10
//! Opening interrupt IN stream...
//! key: A
//! key: Shift+A
//! key: Ctrl+C
//! key: Enter
//! key: F1
//! key: A B        <- A and B simultaneously
//! ```
//!
//! Idle reports (no keys pressed) are suppressed.
//! Up to 3 simultaneous keycodes are shown per report (full boot protocol has 6).
//!
//! # Build and flash
//!
//! ```sh
//! cargo build --release --target thumbv7em-none-eabihf --example rtic_usb_hid_keyboard
//! rust-objcopy -O ihex target/thumbv7em-none-eabihf/release/examples/rtic_usb_hid_keyboard hid_keyboard.hex
//! teensy_loader_cli --mcu=TEENSY41 -w -v hid_keyboard.hex
//! ```

#![no_std]
#![no_main]

#[rtic::app(device = board, peripherals = false, dispatchers = [BOARD_SWTASK0])]
mod app {
    use core::pin::pin;
    use futures::StreamExt;
    use imxrt_hal as hal;
    use imxrt_ral as ral;
    use imxrt_usbh::host::{Imxrt1062HostController, UsbShared, UsbStatics};
    use imxrt_usbh::usb_bus::{DeviceEvent, UsbBus};
    use imxrt_usbh::wire::{
        ConfigurationDescriptor, DescriptorVisitor, EndpointDescriptor, InterfaceDescriptor,
    };

    // -----------------------------------------------------------------------
    // Configuration
    // -----------------------------------------------------------------------

    const FRONTEND: board::logging::Frontend = board::logging::Frontend::Log;
    const BACKEND: board::logging::Backend = board::logging::Backend::Usbd;

    const USB2_BASE: *const () = 0x402E_0200usize as *const ();
    const USB2_NVIC_PRIORITY: u8 = 0xE0;

    // -----------------------------------------------------------------------
    // HID keycode decoder (USB HID Usage Tables 1.12, section 10)
    // -----------------------------------------------------------------------

    /// Map a USB HID boot-protocol keycode to a printable name.
    ///
    /// Returns `""` for keycode 0 (no key), `"?"` for unknown codes.
    fn keycode_name(code: u8) -> &'static str {
        match code {
            0x00 => "",
            0x04 => "A",
            0x05 => "B",
            0x06 => "C",
            0x07 => "D",
            0x08 => "E",
            0x09 => "F",
            0x0A => "G",
            0x0B => "H",
            0x0C => "I",
            0x0D => "J",
            0x0E => "K",
            0x0F => "L",
            0x10 => "M",
            0x11 => "N",
            0x12 => "O",
            0x13 => "P",
            0x14 => "Q",
            0x15 => "R",
            0x16 => "S",
            0x17 => "T",
            0x18 => "U",
            0x19 => "V",
            0x1A => "W",
            0x1B => "X",
            0x1C => "Y",
            0x1D => "Z",
            0x1E => "1",
            0x1F => "2",
            0x20 => "3",
            0x21 => "4",
            0x22 => "5",
            0x23 => "6",
            0x24 => "7",
            0x25 => "8",
            0x26 => "9",
            0x27 => "0",
            0x28 => "Enter",
            0x29 => "Esc",
            0x2A => "Backspace",
            0x2B => "Tab",
            0x2C => "Space",
            0x2D => "-",
            0x2E => "=",
            0x2F => "[",
            0x30 => "]",
            0x31 => "\\",
            0x33 => ";",
            0x34 => "'",
            0x35 => "`",
            0x36 => ",",
            0x37 => ".",
            0x38 => "/",
            0x39 => "CapsLock",
            0x3A => "F1",
            0x3B => "F2",
            0x3C => "F3",
            0x3D => "F4",
            0x3E => "F5",
            0x3F => "F6",
            0x40 => "F7",
            0x41 => "F8",
            0x42 => "F9",
            0x43 => "F10",
            0x44 => "F11",
            0x45 => "F12",
            0x46 => "PrtSc",
            0x47 => "ScrollLock",
            0x48 => "Pause",
            0x49 => "Insert",
            0x4A => "Home",
            0x4B => "PageUp",
            0x4C => "Delete",
            0x4D => "End",
            0x4E => "PageDown",
            0x4F => "Right",
            0x50 => "Left",
            0x51 => "Down",
            0x52 => "Up",
            0x53 => "NumLock",
            _ => "?",
        }
    }

    // -----------------------------------------------------------------------
    // Morse code
    // -----------------------------------------------------------------------

    /// Return the morse pattern for an alphanumeric HID keycode.
    ///
    /// Keycodes 0x04–0x1D map to A–Z; 0x1E–0x27 map to 1–0.
    /// Each element: `true` = dah (long), `false` = dit (short).
    /// Returns `None` for non-alphanumeric keycodes.
    fn morse_pattern(code: u8) -> Option<&'static [bool]> {
        match code {
            // A–Z (0x04–0x1D)
            0x04 => Some(&[false, true]),                // A: .-
            0x05 => Some(&[true, false, false, false]),  // B: -...
            0x06 => Some(&[true, false, true, false]),   // C: -.-.
            0x07 => Some(&[true, false, false]),         // D: -..
            0x08 => Some(&[false]),                      // E: .
            0x09 => Some(&[false, false, true, false]),  // F: ..-.
            0x0A => Some(&[true, true, false]),          // G: --.
            0x0B => Some(&[false, false, false, false]), // H: ....
            0x0C => Some(&[false, false]),               // I: ..
            0x0D => Some(&[false, true, true, true]),    // J: .---
            0x0E => Some(&[true, false, true]),          // K: -.-
            0x0F => Some(&[false, true, false, false]),  // L: .-..
            0x10 => Some(&[true, true]),                 // M: --
            0x11 => Some(&[true, false]),                // N: -.
            0x12 => Some(&[true, true, true]),           // O: ---
            0x13 => Some(&[false, true, true, false]),   // P: .--.
            0x14 => Some(&[true, true, false, true]),    // Q: --.-
            0x15 => Some(&[false, true, false]),         // R: .-.
            0x16 => Some(&[false, false, false]),        // S: ...
            0x17 => Some(&[true]),                       // T: -
            0x18 => Some(&[false, false, true]),         // U: ..-
            0x19 => Some(&[false, false, false, true]),  // V: ...-
            0x1A => Some(&[false, true, true]),          // W: .--
            0x1B => Some(&[true, false, false, true]),   // X: -..-
            0x1C => Some(&[true, false, true, true]),    // Y: -.--
            0x1D => Some(&[true, true, false, false]),   // Z: --..
            // 1–9, 0 (0x1E–0x27)
            0x1E => Some(&[false, true, true, true, true]), // 1: .----
            0x1F => Some(&[false, false, true, true, true]), // 2: ..---
            0x20 => Some(&[false, false, false, true, true]), // 3: ...--
            0x21 => Some(&[false, false, false, false, true]), // 4: ....-
            0x22 => Some(&[false, false, false, false, false]), // 5: .....
            0x23 => Some(&[true, false, false, false, false]), // 6: -....
            0x24 => Some(&[true, true, false, false, false]), // 7: --...
            0x25 => Some(&[true, true, true, false, false]), // 8: ---..
            0x26 => Some(&[true, true, true, true, false]), // 9: ----.
            0x27 => Some(&[true, true, true, true, true]),  // 0: -----
            _ => None,
        }
    }

    /// Flash the LED in morse code for the given pattern.
    ///
    /// - Dit (false): LED on 150 ms, off 150 ms
    /// - Dah (true):  LED on 450 ms, off 150 ms
    /// - End of character: extra 300 ms off
    ///
    /// Uses `cortex_m::asm::delay` busy-wait at 600 MHz.
    fn flash_morse(led: &mut board::Led, pattern: &[bool]) {
        const TICKS_PER_MS: u32 = 600_000;
        for &is_dah in pattern {
            let on_ms: u32 = if is_dah { 450 } else { 150 };
            led.set();
            cortex_m::asm::delay(TICKS_PER_MS * on_ms);
            led.clear();
            cortex_m::asm::delay(TICKS_PER_MS * 150);
        }
        // End-of-character gap (300 ms extra, total inter-char = 150 + 300 = 450 ms)
        cortex_m::asm::delay(TICKS_PER_MS * 300);
    }

    // -----------------------------------------------------------------------
    // Descriptor visitor: finds first interrupt IN endpoint
    // -----------------------------------------------------------------------

    /// Walks a configuration descriptor set and records the first interrupt
    /// IN endpoint found, along with the interface number and config value.
    struct HidFinder {
        config_value: u8,
        ep_num: Option<u8>,
        ep_mps: u16,
        ep_interval: u8,
        iface_num: u8,
    }

    impl Default for HidFinder {
        fn default() -> Self {
            Self {
                config_value: 1,
                ep_num: None,
                ep_mps: 8,
                ep_interval: 10,
                iface_num: 0,
            }
        }
    }

    impl DescriptorVisitor for HidFinder {
        fn on_configuration(&mut self, c: &ConfigurationDescriptor) {
            self.config_value = c.bConfigurationValue;
        }

        fn on_interface(&mut self, i: &InterfaceDescriptor) {
            if self.ep_num.is_none() {
                self.iface_num = i.bInterfaceNumber;
            }
        }

        fn on_endpoint(&mut self, e: &EndpointDescriptor) {
            // First interrupt IN endpoint: direction bit = 1, transfer type bits [1:0] = 3
            if self.ep_num.is_none()
                && (e.bEndpointAddress & 0x80) != 0
                && (e.bmAttributes & 0x03) == 0x03
            {
                self.ep_num = Some(e.bEndpointAddress & 0x0F);
                self.ep_mps = u16::from_le_bytes(e.wMaxPacketSize);
                self.ep_interval = e.bInterval;
            }
        }
    }

    // -----------------------------------------------------------------------
    // PLL_USB2 setup
    // -----------------------------------------------------------------------

    fn enable_usb2_pll() {
        let ccm_analog = unsafe { ral::ccm_analog::CCM_ANALOG::instance() };
        loop {
            if ral::read_reg!(ral::ccm_analog, ccm_analog, PLL_USB2, DIV_SELECT == 1) {
                ral::write_reg!(ral::ccm_analog, ccm_analog, PLL_USB2_SET, BYPASS: 1);
                ral::write_reg!(ral::ccm_analog, ccm_analog, PLL_USB2_CLR,
                    POWER: 1, DIV_SELECT: 1, ENABLE: 1, EN_USB_CLKS: 1);
                continue;
            }
            if ral::read_reg!(ral::ccm_analog, ccm_analog, PLL_USB2, ENABLE == 0) {
                ral::write_reg!(ral::ccm_analog, ccm_analog, PLL_USB2_SET, ENABLE: 1);
                continue;
            }
            if ral::read_reg!(ral::ccm_analog, ccm_analog, PLL_USB2, POWER == 0) {
                ral::write_reg!(ral::ccm_analog, ccm_analog, PLL_USB2_SET, POWER: 1);
                continue;
            }
            if ral::read_reg!(ral::ccm_analog, ccm_analog, PLL_USB2, LOCK == 0) {
                continue;
            }
            if ral::read_reg!(ral::ccm_analog, ccm_analog, PLL_USB2, BYPASS == 1) {
                ral::write_reg!(ral::ccm_analog, ccm_analog, PLL_USB2_CLR, BYPASS: 1);
                continue;
            }
            if ral::read_reg!(ral::ccm_analog, ccm_analog, PLL_USB2, EN_USB_CLKS == 0) {
                ral::write_reg!(ral::ccm_analog, ccm_analog, PLL_USB2_SET, EN_USB_CLKS: 1);
                continue;
            }
            break;
        }
    }

    // -----------------------------------------------------------------------
    // VBUS power enable
    // -----------------------------------------------------------------------

    fn enable_vbus_power() {
        let iomuxc = unsafe { ral::iomuxc::IOMUXC::instance() };
        ral::write_reg!(ral::iomuxc, iomuxc, SW_MUX_CTL_PAD_GPIO_EMC_40, 5);
        ral::write_reg!(ral::iomuxc, iomuxc, SW_PAD_CTL_PAD_GPIO_EMC_40, 0x0008);

        // Route GPIO_EMC_40 (GPIO3_IO26) to fast GPIO8_IO26.
        // The Teensy Arduino startup sets GPR26-29 = 0xFFFFFFFF; our bare-metal
        // runtime doesn't, so GPR28[26] defaults to 0 (pin driven by GPIO3).
        // Without this, writes to GPIO8 registers update the register file but
        // don't actually drive the pin.
        let iomuxc_gpr = unsafe { ral::iomuxc_gpr::IOMUXC_GPR::instance() };
        ral::modify_reg!(ral::iomuxc_gpr, iomuxc_gpr, GPR28, |v| v | (1 << 26));

        let gpio8 = unsafe { ral::gpio::GPIO8::instance() };
        ral::modify_reg!(ral::gpio, gpio8, GDIR, |v| v | (1 << 26));
        ral::write_reg!(ral::gpio, gpio8, DR_SET, 1 << 26);
    }

    // -----------------------------------------------------------------------
    // Delay helper
    // -----------------------------------------------------------------------

    fn delay_ms(ms: usize) -> impl core::future::Future<Output = ()> {
        cortex_m::asm::delay((ms as u32) * 600_000);
        core::future::ready(())
    }

    // -----------------------------------------------------------------------
    // Static resources
    // -----------------------------------------------------------------------

    static SHARED: UsbShared = UsbShared::new();
    static mut STATICS: UsbStatics = UsbStatics::new();

    // -----------------------------------------------------------------------
    // RTIC resources
    // -----------------------------------------------------------------------

    #[local]
    struct Local {}

    #[shared]
    struct Shared {
        poller: board::logging::Poller,
    }

    // -----------------------------------------------------------------------
    // Init
    // -----------------------------------------------------------------------

    #[init]
    fn init(_cx: init::Context) -> (Shared, Local) {
        let (
            board::Common {
                usb1,
                usbnc1,
                usbphy1,
                mut dma,
                ..
            },
            board::Specifics { led, console, .. },
        ) = board::new();

        let usbd = hal::usbd::Instances {
            usb: usb1,
            usbnc: usbnc1,
            usbphy: usbphy1,
        };
        let dma_a = dma[board::BOARD_DMA_A_INDEX].take().unwrap();
        let poller = board::logging::init(FRONTEND, BACKEND, console, dma_a, usbd);
        // Filter out TRACE-level messages to avoid overflowing the 1024-byte log buffer
        // during rapid USB transfer sequences.
        log::set_max_level(log::LevelFilter::Debug);

        hid_keyboard::spawn(led).ok();

        (Shared { poller }, Local {})
    }

    // -----------------------------------------------------------------------
    // USB_OTG2 ISR
    // -----------------------------------------------------------------------

    unsafe extern "C" fn usb2_isr() {
        SHARED.on_usb_irq(USB2_BASE);
    }

    // -----------------------------------------------------------------------
    // USB1 ISR (logging) — priority 2 so log flushing preempts USB task
    // -----------------------------------------------------------------------
    //
    // The USB host task runs at priority 1 via the BOARD_SWTASK0 dispatcher.
    // At priority 2, USB1 and DMA interrupts preempt the USB task to flush
    // logs promptly. We poll the logger directly in the ISR, avoiding the
    // need for a second RTIC dispatcher.

    #[task(binds = BOARD_USB1, shared = [poller], priority = 2)]
    fn usb1_interrupt(mut cx: usb1_interrupt::Context) {
        cx.shared.poller.lock(|poller| poller.poll());
    }

    #[task(binds = BOARD_DMA_A, shared = [poller], priority = 2)]
    fn dma_interrupt(mut cx: dma_interrupt::Context) {
        cx.shared.poller.lock(|poller| poller.poll());
    }

    // -----------------------------------------------------------------------
    // HID keyboard task
    // -----------------------------------------------------------------------

    /// Async task: enumerate device, find HID interrupt endpoint, poll for reports.
    ///
    /// Uses the cotton-usb-host high-level API:
    ///  - `UsbBus::device_events_no_hubs()` for non-hub enumeration
    ///  - `UsbBus::get_configuration()` to parse descriptors via `HidFinder`
    ///  - `UsbBus::configure()` to issue SET_CONFIGURATION
    ///  - `UsbBus::interrupt_endpoint_in()` to open the interrupt IN stream
    #[task(priority = 1)]
    async fn hid_keyboard(_cx: hid_keyboard::Context, mut led: board::Led) {
        // Delay to let logging settle before starting USB host.
        cortex_m::asm::delay(600_000 * 5_000);

        log::info!("=== imxrt-usbh: HID Keyboard Example ===");

        enable_usb2_pll();
        log::info!("USB2 PLL locked");

        enable_vbus_power();
        log::info!("VBUS power enabled");

        let usb2 = unsafe { ral::usb::USB2::instance() };
        let usbphy2 = unsafe { ral::usbphy::USBPHY2::instance() };

        let statics: &'static UsbStatics = unsafe { &*core::ptr::addr_of!(STATICS) };
        let mut host = Imxrt1062HostController::new(usb2, usbphy2, &SHARED, statics);
        unsafe { host.init() };
        log::info!("USB host controller initialised");

        unsafe {
            let irq_num = ral::interrupt::USB_OTG2 as u32;
            core::ptr::write_volatile((0xE000_E400 + irq_num) as *mut u8, USB2_NVIC_PRIORITY);

            extern "C" {
                static __INTERRUPTS: [core::cell::UnsafeCell<unsafe extern "C" fn()>; 240];
            }
            let usb_otg2_irq = ral::interrupt::USB_OTG2 as usize;
            __INTERRUPTS[usb_otg2_irq].get().write_volatile(usb2_isr);

            cortex_m::asm::dsb();
            cortex_m::asm::isb();

            cortex_m::peripheral::NVIC::unmask(ral::interrupt::USB_OTG2);
        }
        log::info!(
            "USB_OTG2 ISR installed (NVIC priority 0x{:02X})",
            USB2_NVIC_PRIORITY
        );

        log::info!("Entering device event loop...");

        let bus = UsbBus::new(host);
        let mut events = pin!(bus.device_events_no_hubs(delay_ms));

        loop {
            match events.next().await {
                Some(DeviceEvent::Connect(device, info)) => {
                    // Panic if a hub is connected — this example doesn't support hubs.
                    if info.class == 9 {
                        panic!(
                            "Hub detected (class=9) but hub-support feature is not enabled. \
                             Rebuild with --features hub-support and use the rtic_usb_hub example."
                        );
                    }

                    log::info!(
                        "DeviceEvent::Connect  addr={}  VID={:04x} PID={:04x} class={} subclass={}",
                        device.address(),
                        info.vid,
                        info.pid,
                        info.class,
                        info.subclass,
                    );

                    // Walk configuration descriptors to find first interrupt IN endpoint.
                    let mut finder = HidFinder::default();
                    if let Err(_e) = bus.get_configuration(&device, &mut finder).await {
                        log::warn!("get_configuration failed");
                        continue;
                    }

                    let (ep, ep_mps, ep_interval) = match finder.ep_num {
                        Some(n) => (n, finder.ep_mps, finder.ep_interval),
                        None => {
                            log::warn!("No interrupt IN endpoint found");
                            continue;
                        }
                    };

                    log::info!(
                        "Found HID interface: iface={} ep={} mps={} interval={}",
                        finder.iface_num,
                        ep,
                        ep_mps,
                        ep_interval,
                    );

                    // Issue SET_CONFIGURATION and transition to Configured state.
                    let usb_device = match bus.configure(device, finder.config_value).await {
                        Ok(d) => d,
                        Err(_e) => {
                            log::warn!("configure failed");
                            continue;
                        }
                    };

                    log::info!("Opening interrupt IN stream...");

                    // Allocate interrupt IN pipe and poll for HID reports.
                    // alloc_interrupt_pipe() is awaited lazily on first poll_next().
                    let mut pipe =
                        pin!(bus.interrupt_endpoint_in(&usb_device, ep, ep_mps, ep_interval));

                    loop {
                        match pipe.next().await {
                            Some(pkt) => {
                                // HID boot protocol (8 bytes):
                                //   byte 0: modifier bitmap
                                //   byte 1: reserved
                                //   bytes 2-7: up to 6 simultaneous keycodes
                                if pkt.size < 8 {
                                    continue; // unexpected short report
                                }
                                let mods = pkt.data[0];
                                // Suppress idle reports (no keys, no modifiers).
                                if mods == 0 && pkt.data[2] == 0 {
                                    continue;
                                }
                                // Modifier byte bits: LCtrl=0 LShift=1 LAlt=2 LGUI=3
                                //                    RCtrl=4 RShift=5 RAlt=6 RGUI=7
                                log::info!(
                                    "key: {}{}{}{}{}{}{}{}{}",
                                    if mods & 0x02 != 0 || mods & 0x20 != 0 {
                                        "Shift+"
                                    } else {
                                        ""
                                    },
                                    if mods & 0x01 != 0 || mods & 0x10 != 0 {
                                        "Ctrl+"
                                    } else {
                                        ""
                                    },
                                    if mods & 0x04 != 0 || mods & 0x40 != 0 {
                                        "Alt+"
                                    } else {
                                        ""
                                    },
                                    if mods & 0x08 != 0 || mods & 0x80 != 0 {
                                        "GUI+"
                                    } else {
                                        ""
                                    },
                                    keycode_name(pkt.data[2]),
                                    if pkt.data[3] != 0 { " " } else { "" },
                                    keycode_name(pkt.data[3]),
                                    if pkt.data[4] != 0 { " " } else { "" },
                                    keycode_name(pkt.data[4]),
                                );

                                // Flash morse code for the first keycode if alphanumeric.
                                if let Some(pattern) = morse_pattern(pkt.data[2]) {
                                    flash_morse(&mut led, pattern);
                                }
                            }
                            None => {
                                log::warn!("Interrupt pipe stream ended");
                                break;
                            }
                        }
                    }
                }
                Some(DeviceEvent::Disconnect(_)) => {
                    log::info!("DeviceEvent::Disconnect");
                }
                Some(DeviceEvent::EnumerationError(hub, port, _err)) => {
                    log::warn!("DeviceEvent::EnumerationError  hub={} port={}", hub, port);
                }
                Some(DeviceEvent::HubConnect(_)) => {}
                Some(DeviceEvent::None) => {}
                None => {
                    log::warn!("Device event stream ended");
                    break;
                }
            }
        }
    }
}
