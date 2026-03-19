use std::io::{self, Write as IoWrite};

use prost::Message;
use serial2::SerialPort;

mod data {
    use prost::Message;

    #[derive(Clone, PartialEq, Message)]
    pub struct MotionProfile {
        #[prost(message, repeated, tag = "1")]
        pub phases: Vec<Phase>,
    }

    #[derive(Clone, PartialEq, Message)]
    pub struct Phase {
        #[prost(float, tag = "1")]
        pub duration: f32,
        #[prost(float, tag = "2")]
        pub jerk: f32,
        #[prost(float, tag = "3")]
        pub target_velocity: f32,
        #[prost(float, tag = "4")]
        pub target_position: f32,
    }
}

use data::{MotionProfile, Phase};

const DEVICE_PATH: &str = "/dev/ttyACM0";
const BAUD_RATE: u32 = 115200;

const MAX_SPEED: f32 = 300.0;
const MAX_ACCEL: f32 = 15000.0;
const JERK: f32 = 10.0;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let distance = prompt_distance()?;
    let phase_data = motion_planner(distance, MAX_SPEED, MAX_ACCEL, JERK);
    send_phases(phase_data)?;
    Ok(())
}

fn prompt_distance() -> Result<f32, Box<dyn std::error::Error>> {
    print!("Enter distance (mm): ");
    io::stdout().flush()?;

    let mut input = String::new();
    io::stdin().read_line(&mut input)?;

    let distance: f32 = input.trim().parse()?;
    Ok(distance)
}

fn motion_planner(distance: f32, velocity: f32, accel: f32, jerk: f32) -> [[f32; 7]; 2] {
    let mut t_j = accel / jerk;
    let mut t_a = (velocity / accel) - t_j;

    if t_a < 0.0 {
        t_j = (velocity / jerk).sqrt();
        t_a = 0.0;
    }

    let v_accel = if t_a > 0.0 {
        jerk * t_j * t_j + accel * t_a
    } else {
        jerk * t_j * t_j
    };

    let d = v_accel * (2.0 * t_j + t_a);

    let mut t_v = 0.0;

    if distance > d {
        t_v = (distance - d) / velocity;
    } else {
        let short_path_j = (distance / (2.0 * jerk)).powf(1.0 / 3.0);

        if short_path_j < t_j {
            t_j = short_path_j;
            t_a = 0.0;
        } else {
            let a = accel;
            let v_peak =
                ((a.powi(4) / jerk.powi(2) + 4.0 * a * distance).sqrt() - (a.powi(2) / jerk))
                    / 2.0;
            t_j = a / jerk;
            t_a = (v_peak / a) - t_j;
        }
    }

    [
        [t_j, t_a, t_j, t_v, t_j, t_a, t_j],
        [jerk, 0.0, -jerk, 0.0, -jerk, 0.0, jerk],
    ]
}

fn send_phases(phase_data: [[f32; 7]; 2]) -> Result<(), Box<dyn std::error::Error>> {
    let phases: Vec<Phase> = (0..7)
        .map(|i| Phase {
            duration: phase_data[0][i],
            jerk: phase_data[1][i],
            target_velocity: 0.0,
            target_position: 0.0,
        })
        .collect();

    let msg = MotionProfile { phases };

    let mut payload = Vec::new();
    payload.reserve(msg.encoded_len());
    msg.encode(&mut payload)?;

    let length_prefix = (payload.len() as u32).to_le_bytes();

    let mut frame = Vec::with_capacity(4 + payload.len());
    frame.extend_from_slice(&length_prefix);
    frame.extend_from_slice(&payload);

    println!("Connecting to {} at {} baud...", DEVICE_PATH, BAUD_RATE);
    let port = SerialPort::open(DEVICE_PATH, BAUD_RATE)?;

    println!(
        "Sending {} phases ({} bytes payload)...",
        msg.phases.len(),
        payload.len()
    );
    port.write_all(&frame)?;
    println!("Done.");

    Ok(())
}
