#![no_std]
#![no_main]

use teensy4_panic as _;

mod stepper;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [KPP])]
mod app {
    use board::t40 as my_board;
    use bsp::board;
    use embedded_hal::blocking::serial::Write as SerialWrite;
    use embedded_hal::serial::Read as SerialRead;
    use imxrt_log as logging;
    use rtic_monotonics::systick::{Systick, *};
    use teensy4_bsp as bsp;

    const STEPS_PER_MM: f32 = 80.0;
    const MAX_FRAME_SIZE: usize = 256;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led: board::Led,
        uart: board::Lpuart6,
        step: bsp::hal::gpio::Output<bsp::pins::t40::P2>,
        dir: bsp::hal::gpio::Output<bsp::pins::t40::P3>,
        usb_log: logging::Poller,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let board::Resources {
            mut gpio2,
            mut gpio4,
            pins,
            usb,
            lpuart6,
            ..
        } = my_board(cx.device);

        let usb_log = logging::log::usbd(usb, logging::Interrupts::Enabled).unwrap();
        let led = board::led(&mut gpio2, pins.p13);
        let step = gpio4.output(pins.p2);
        let dir = gpio4.output(pins.p3);
        let uart: board::Lpuart6 = board::lpuart(lpuart6, pins.p1, pins.p0, 115200);

        Systick::start(
            cx.core.SYST,
            board::ARM_FREQUENCY,
            rtic_monotonics::create_systick_token!(),
        );

        motor_control::spawn().unwrap();

        log::info!("motion controller online");

        (
            Shared {},
            Local {
                led,
                uart,
                step,
                dir,
                usb_log,
            },
        )
    }

    #[task(local = [uart, step, dir, led])]
    async fn motor_control(cx: motor_control::Context) {
        let uart = cx.local.uart;
        let step = cx.local.step;
        let dir = cx.local.dir;
        let led = cx.local.led;

        let setup_frame: [u8; 8] = [0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x29];
        let _ = uart.bwrite_all(&setup_frame);

        dir.set();

        loop {
            let mut len_buf = [0u8; 4];
            if read_exact(uart, &mut len_buf).is_err() {
                Systick::delay(10.millis()).await;
                continue;
            }

            let payload_len = u32::from_le_bytes(len_buf) as usize;
            if payload_len == 0 || payload_len > MAX_FRAME_SIZE {
                log::warn!("bad frame length: {}", payload_len);
                continue;
            }

            let mut payload = [0u8; MAX_FRAME_SIZE];
            if read_exact(uart, &mut payload[..payload_len]).is_err() {
                log::warn!("payload read failed");
                continue;
            }

            let phases = decode_phases(&payload[..payload_len]);
            log::info!("running {} phases", phases.len());
            led.toggle();

            for (duration, jerk) in phases.iter().copied() {
                execute_phase(step, duration, jerk).await;
            }

            led.toggle();
            log::info!("move complete");
        }
    }

    fn read_exact(uart: &mut board::Lpuart6, buf: &mut [u8]) -> Result<(), ()> {
        for byte in buf.iter_mut() {
            *byte = nb::block!(uart.read()).map_err(|_| ())?;
        }
        Ok(())
    }

    fn decode_phases(data: &[u8]) -> heapless::Vec<(f32, f32), 7> {
        let mut phases = heapless::Vec::new();
        let mut offset = 0;

        while offset < data.len() {
            let (tag_wire, n) = match decode_varint(&data[offset..]) {
                Some(v) => v,
                None => break,
            };
            offset += n;

            let field_number = tag_wire >> 3;
            let wire_type = tag_wire & 0x7;

            if field_number != 1 || wire_type != 2 {
                break;
            }

            let (msg_len, m) = match decode_varint(&data[offset..]) {
                Some(v) => v,
                None => break,
            };
            offset += m;

            let end = (offset + msg_len as usize).min(data.len());
            let (duration, jerk) = decode_phase_message(&data[offset..end]);
            let _ = phases.push((duration, jerk));
            offset = end;
        }

        phases
    }

    fn decode_phase_message(data: &[u8]) -> (f32, f32) {
        let mut duration = 0.0f32;
        let mut jerk = 0.0f32;
        let mut offset = 0;

        while offset < data.len() {
            let (tag_wire, n) = match decode_varint(&data[offset..]) {
                Some(v) => v,
                None => break,
            };
            offset += n;

            let field_number = tag_wire >> 3;
            let wire_type = tag_wire & 0x7;

            match wire_type {
                5 => {
                    if offset + 4 > data.len() {
                        break;
                    }

                    let mut bytes = [0u8; 4];
                    bytes.copy_from_slice(&data[offset..offset + 4]);
                    let value = f32::from_le_bytes(bytes);
                    offset += 4;

                    match field_number {
                        1 => duration = value,
                        2 => jerk = value,
                        _ => {}
                    }
                }
                0 => {
                    let (_, used) = match decode_varint(&data[offset..]) {
                        Some(v) => v,
                        None => break,
                    };
                    offset += used;
                }
                2 => {
                    let (len, used) = match decode_varint(&data[offset..]) {
                        Some(v) => v,
                        None => break,
                    };
                    offset += used + len as usize;
                    if offset > data.len() {
                        break;
                    }
                }
                _ => break,
            }
        }

        (duration, jerk)
    }

    fn decode_varint(data: &[u8]) -> Option<(u64, usize)> {
        let mut result: u64 = 0;
        let mut shift = 0;

        for (i, &byte) in data.iter().enumerate() {
            result |= ((byte & 0x7F) as u64) << shift;
            if byte & 0x80 == 0 {
                return Some((result, i + 1));
            }

            shift += 7;
            if shift >= 64 {
                break;
            }
        }

        None
    }

    async fn execute_phase(
        step: &mut bsp::hal::gpio::Output<bsp::pins::t40::P2>,
        duration_secs: f32,
        jerk: f32,
    ) {
        if duration_secs <= 0.0 {
            return;
        }

        const DT: f32 = 0.0001;
        let ticks = (duration_secs / DT) as u32;

        let mut velocity = 0.0f32;
        let mut accel = 0.0f32;
        let mut step_accumulator = 0.0f32;

        for _ in 0..ticks {
            accel += jerk * DT;
            velocity += accel * DT;
            step_accumulator += velocity.abs() * DT * STEPS_PER_MM;

            while step_accumulator >= 1.0 {
                step_accumulator -= 1.0;
                step.set();
                Systick::delay(2.micros()).await;
                step.clear();
                Systick::delay(8.micros()).await;
            }

            Systick::delay(100.micros()).await;
        }
    }

    #[task(binds = USB_OTG1, local = [usb_log])]
    fn log_over_usb(cx: log_over_usb::Context) {
        cx.local.usb_log.poll();
    }
}
