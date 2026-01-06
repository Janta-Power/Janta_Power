use accel_stepper::{Driver, OperatingSystemClock, StepAndDirection};
use astronav::coords::noaa_sun::NOAASun;
use clock::Clock;
use std::time::{Duration, Instant};
use esp_idf_svc::hal::gpio::{Gpio15, Gpio16, Gpio17, Gpio14, Gpio21, Gpio47, Input, Output, PinDriver};
use esp_idf_svc::nvs::*;
use network::mqtt::Mqtt;
use wifi::wifi::{Wifi, WifiState};
use ota::OtaUpdater;
use semver::Version;
use std::thread;

// Import state definitions from states module
use crate::states::TrackingState;

// ============================================================================
// ENCODER CONSTANTS (from C++ implementation)
// ============================================================================
// Encoder counts per one full 360° rotation of the output shaft
const ENCODER_COUNTS_PER_REV: i64 = 348323;

// Stall detection constants
const STALL_TIME_MS: u64 = 250;      // How long encoder can be "stuck" before stall detected
const STALL_MIN_DENC: i64 = 1;       // Minimum encoder count change considered "movement"

// Output steps per revolution (motor steps * microsteps * gear ratio)
const OUT_STEPS_PER_REV: f64 = 25600.0 * 50.0 * 84.0;  // 107520000 steps per output rev

pub fn calculate_steps(offset: f32) -> i64 {
    return ((offset / 360.0) * (25600.0 * 50.0 * 84.0)) as i64;
}

pub struct Motion<'a> {
    location: f32,
    tracking_state: TrackingState,
    speed: f32,
    acceleration: u16,
    motor: Driver,
    motor_device:
        StepAndDirection<PinDriver<'a, Gpio15, Output>, PinDriver<'a, Gpio16, Output>>,
    motor_clock: OperatingSystemClock,
    prev_balance: i32,
    relay: PinDriver<'a, Gpio17, Output>,
    lmsw: PinDriver<'a, Gpio14, Input>,
    // Encoder support for L2 state
    encoder_a: PinDriver<'a, Gpio21, Input>,  // Encoder channel A (GPIO 21)
    encoder_b: PinDriver<'a, Gpio47, Input>,  // Encoder channel B (GPIO 47)
    encoder_count: i64,
    encoder_target: Option<i64>,
    encoder_stop_armed: bool,
    last_encoder_for_stall: i64,
    stall_timer_start: Option<Instant>,
    // Quadrature encoder state tracking
    encoder_last_state: u8,  // Last state of A and B pins (0-3)
    // Encoder speed/acceleration tracking
    encoder_last_count: i64,
    encoder_last_time: Option<Instant>,
    encoder_last_speed_dps: f64,  // Last calculated encoder speed in degrees per second
    encoder_last_speed_time: Option<Instant>,
    log_counter: u32,  // Counter for periodic logging
}

// CW: direction
// CCW: step
impl Motion<'_> {
    pub fn new<'a>(p10: Gpio15, p11: Gpio16, p7: Gpio17, p6: Gpio14, enc_a: Gpio21, enc_b: Gpio47) -> Motion<'a> {
        let step = PinDriver::output(p10).unwrap();
        let direction = PinDriver::output(p11).unwrap();
        let relay = PinDriver::output(p7).unwrap();
        let mut lmsw = PinDriver::input(p6).unwrap();
        lmsw.set_pull(esp_idf_svc::hal::gpio::Pull::Down)
            .unwrap_or_default();
        
        // Initialize encoder pins with pull-up resistors
        let mut encoder_a = PinDriver::input(enc_a).unwrap();
        encoder_a.set_pull(esp_idf_svc::hal::gpio::Pull::Up)
            .unwrap_or_default();
        let mut encoder_b = PinDriver::input(enc_b).unwrap();
        encoder_b.set_pull(esp_idf_svc::hal::gpio::Pull::Up)
            .unwrap_or_default();
        
        // Read initial encoder state
        let a_init = encoder_a.is_high();
        let b_init = encoder_b.is_high();
        let initial_state = ((a_init as u8) << 1) | (b_init as u8);
        log::info!("Encoder initialized: GPIO21 (A)={}, GPIO47 (B)={}, initial_state={:02b}", 
                   a_init, b_init, initial_state);

        Motion {
            location: 0.0,
            tracking_state: TrackingState::L1,
            speed: 43000.0,
            acceleration: 20000,
            motor: Driver::new(),
            motor_device: StepAndDirection::new(step, direction),
            motor_clock: OperatingSystemClock::new(),
            prev_balance: 0,
            relay,
            lmsw,
            // Encoder pin drivers
            encoder_a,
            encoder_b,
            // Encoder state initialization
            encoder_count: 0,
            encoder_target: None,
            encoder_stop_armed: false,
            last_encoder_for_stall: 0,
            stall_timer_start: None,
            encoder_last_state: initial_state,
            // Speed/acceleration tracking initialization
            encoder_last_count: 0,
            encoder_last_time: None,
            encoder_last_speed_dps: 0.0,
            encoder_last_speed_time: None,
            log_counter: 0,
        }
    }

    pub fn update_position(&mut self, location: f32) {
        self.location = location;
    }

    pub fn location(&mut self) -> f32 {
        self.location
    }

    pub fn switch_pressed(&mut self) -> bool {
        self.lmsw.is_low()
    }

    pub fn init(&mut self) {
        self.motor.set_max_speed(self.speed);
        self.motor.set_speed(self.speed);
        self.motor.set_acceleration(self.acceleration.into());
    }

    pub fn move_by_angle(&mut self, offset: f32) {
        self.move_by(calculate_steps(offset));
        //self.run();
    }

    pub fn move_by(&mut self, location: i64) {
        self.motor.move_by(location);
        self.run();
    }
    
            /// Moves the tracker to 90 degrees (home), enabling relay before moving and disabling it after.
    /// Uses the shortest path (CW or CCW) based on current position.
    pub fn move_to_60(&mut self) {
        let current = self.location();
        let offset = 60.0 - current;
        log::info!("Moving from {:.2}° to 150°, offset = {:.2}°", current, offset);

        // Turn ON relay to enable motor movement
        self.relay.set_high().unwrap_or_default();

        // Move by calculated angle
        self.move_by_angle(offset);
        self.run();

        // Update internal position
        self.update_position(60.0);

        // Turn OFF relay after movement for safety/power savings
        self.relay.set_low().unwrap_or_default();

        log::info!("Now at 150°");
    }

    pub fn move_test(&mut self, location: i64) {
        self.relay.set_high().unwrap_or_default();  // Turn on relay 
        self.tracking_state = TrackingState::L1;   //  Change tracking state

        match self.tracking_state {
            TrackingState::L1 => {
                let steps = (location / 360) * (25600 * 50 * 84);
                log::info!("Steps Needed: {}", steps);
                log::info!("Steps Needed: {}", steps as i64);
                self.move_by(steps as i64);
                self.run();
                self.relay.set_low().unwrap_or_default();  // Turn on relay 
                   // self.update_position((location as f64 + angle_offset) as f32);
                //return false;
                
            }
            TrackingState::L2 => {
                log::info!("L2");
            }

            TrackingState::L3 => (),
        }

    }

    pub fn run(&mut self) {
        // Reset logging counter for this movement
        self.log_counter = 0;
        if self.encoder_last_time.is_none() {
            self.encoder_last_time = Some(Instant::now());
            self.encoder_last_count = self.encoder_count;
        }
        
        loop {
            if self.motor.is_running() {
                let _ = self.motor.poll(&mut self.motor_device, &self.motor_clock);
                
                // Update encoder position from GPIO pins during movement
                self.update_encoder_from_pins();
                
                // Log detailed status every 10 iterations
                self.log_counter += 1;
                if self.log_counter % 10 == 0 {
                    self.log_detailed_status();
                }
            } else {
                break;
            }
        }
    }

    pub fn flip_relay(&mut self) {
        self.relay.toggle().unwrap_or_default();
    }

    // ========================================================================
    // ENCODER SUPPORT FUNCTIONS (for L2 state)
    // ========================================================================
    // These functions implement encoder-based position tracking similar to the C++ code
    // Currently uses stepper position as encoder proxy, but structured for real encoder
    
    /// Reset encoder count to zero
    fn reset_encoder(&mut self) {
        let old_count = self.encoder_count;
        self.encoder_count = 0;
        self.last_encoder_for_stall = 0;
        self.stall_timer_start = None;
        // Read current encoder state to initialize last_state
        self.encoder_last_state = ((self.encoder_a.is_high() as u8) << 1) | (self.encoder_b.is_high() as u8);
        // Reset speed/acceleration tracking
        self.encoder_last_count = 0;
        self.encoder_last_time = Some(Instant::now());
        self.encoder_last_speed_dps = 0.0;
        self.encoder_last_speed_time = None;
        self.log_counter = 0;
        log::info!("Encoder reset: count {} -> 0, initial state: {:02b}", old_count, self.encoder_last_state);
    }
    
    /// Convert encoder count to degrees
    fn encoder_count_to_degrees(&self, count: i64) -> f64 {
        (count as f64 / ENCODER_COUNTS_PER_REV as f64) * 360.0
    }
    
    /// Convert stepper position to degrees
    fn stepper_position_to_degrees(&self, steps: i64) -> f64 {
        (steps as f64 / OUT_STEPS_PER_REV) * 360.0
    }
    
    /// Calculate encoder speed in degrees per second
    fn calculate_encoder_speed_dps(&mut self) -> f64 {
        let now = Instant::now();
        let current_count = self.encoder_count;
        
        if let Some(last_time) = self.encoder_last_time {
            let dt = now.duration_since(last_time).as_secs_f64();
            if dt > 0.0 {
                let d_count = current_count - self.encoder_last_count;
                let d_degrees = self.encoder_count_to_degrees(d_count);
                let speed_dps = d_degrees / dt;
                
                self.encoder_last_count = current_count;
                self.encoder_last_time = Some(now);
                self.encoder_last_speed_dps = speed_dps;
                
                return speed_dps;
            }
        } else {
            self.encoder_last_time = Some(now);
            self.encoder_last_count = current_count;
        }
        
        self.encoder_last_speed_dps
    }
    
    /// Calculate encoder acceleration in degrees per second squared
    fn calculate_encoder_accel_dps2(&mut self) -> f64 {
        let now = Instant::now();
        let current_speed = self.calculate_encoder_speed_dps();
        let last_speed = self.encoder_last_speed_dps;
        
        if let Some(last_speed_time) = self.encoder_last_speed_time {
            let dt = now.duration_since(last_speed_time).as_secs_f64();
            if dt > 0.0 && dt < 1.0 {  // Only calculate if time difference is reasonable
                let d_speed = current_speed - last_speed;
                let accel_dps2 = d_speed / dt;
                
                self.encoder_last_speed_time = Some(now);
                
                return accel_dps2;
            }
        } else {
            self.encoder_last_speed_time = Some(now);
        }
        
        0.0
    }
    
    /// Log detailed encoder/stepper status in the format requested
    fn log_detailed_status(&mut self) {
        let stepper_pos = self.motor.current_position();
        let encoder_count = self.encoder_count;
        
        // Calculate speeds and accelerations
        let enc_speed_dps = self.calculate_encoder_speed_dps();
        let enc_accel_dps2 = self.calculate_encoder_accel_dps2();
        
        // Convert stepper speed from steps/sec to degrees/sec
        let stepper_speed_steps_s = self.motor.speed();
        let given_speed_dps = (stepper_speed_steps_s as f64 / OUT_STEPS_PER_REV) * 360.0;
        
        // Set acceleration in degrees per second squared
        // Acceleration is in steps/sec^2, convert to degrees/sec^2
        let set_accel_dps2 = (self.acceleration as f64 / OUT_STEPS_PER_REV) * 360.0;
        
        log::info!(
            "Stepper={} | Encoder={} | EncSpeed(dps)={:.3} | GivenSpeed(dps)={:.3} | EncAccel(dps2)={:.3} | SetAccel={:.3}",
            stepper_pos,
            encoder_count,
            enc_speed_dps,
            given_speed_dps,
            enc_accel_dps2,
            set_accel_dps2
        );
    }
    
    /// Read quadrature encoder and update count
    /// Implements quadrature decoding: tracks state transitions to determine direction
    /// State encoding: A=MSB, B=LSB (00, 01, 10, 11)
    fn update_encoder_from_pins(&mut self) {
        // Read current encoder pin states
        let a_state = self.encoder_a.is_high();
        let b_state = self.encoder_b.is_high();
        let current_state = ((a_state as u8) << 1) | (b_state as u8);
        
        // Quadrature decoder state machine
        // Valid transitions: 00->01->11->10->00 (CW) or 00->10->11->01->00 (CCW)
        let last_state = self.encoder_last_state;
        
        // Check for valid state transition
        if current_state != last_state {
            let old_count = self.encoder_count;
            // Determine direction based on state transition
            // CW: 00->01->11->10->00 (increment)
            // CCW: 00->10->11->01->00 (decrement)
            match (last_state, current_state) {
                // Clockwise transitions
                (0b00, 0b01) | (0b01, 0b11) | (0b11, 0b10) | (0b10, 0b00) => {
                    self.encoder_count += 1;
                    log::debug!("Encoder: CW transition {:02b}->{:02b}, count: {}->{}", 
                               last_state, current_state, old_count, self.encoder_count);
                }
                // Counter-clockwise transitions
                (0b00, 0b10) | (0b10, 0b11) | (0b11, 0b01) | (0b01, 0b00) => {
                    self.encoder_count -= 1;
                    log::debug!("Encoder: CCW transition {:02b}->{:02b}, count: {}->{}", 
                               last_state, current_state, old_count, self.encoder_count);
                }
                // Invalid transitions (noise or missed state) - log and ignore
                _ => {
                    log::warn!("Encoder: Invalid transition {:02b}->{:02b}, ignoring", 
                              last_state, current_state);
                }
            }
            
            self.encoder_last_state = current_state;
        }
    }
    
    /// Read current encoder position from GPIO pins
    fn read_encoder(&mut self) -> i64 {
        self.update_encoder_from_pins();
        self.encoder_count
    }
    
    /// Get current encoder state for debugging (returns A, B, state, count)
    pub fn get_encoder_status(&mut self) -> (bool, bool, u8, i64) {
        let a = self.encoder_a.is_high();
        let b = self.encoder_b.is_high();
        let state = ((a as u8) << 1) | (b as u8);
        (a, b, state, self.encoder_count)
    }
    
    /// Calculate encoder target count for a given angle in degrees
    fn calculate_encoder_target(&self, angle_deg: f64) -> i64 {
        ((angle_deg / 360.0) * ENCODER_COUNTS_PER_REV as f64) as i64
    }
    
    /// Check for stall condition (encoder not changing while stepper is moving)
    fn check_stall(&mut self) -> bool {
        let enc_now = self.read_encoder();
        let step_speed = self.motor.speed(); // steps/sec (actual ramped speed)
        let stepper_trying_to_move = step_speed.abs() > 1.0;
        
        if stepper_trying_to_move {
            let d = enc_now - self.last_encoder_for_stall;
            
            if d.abs() >= STALL_MIN_DENC {
                // Encoder is moving => reset stall timer
                self.stall_timer_start = None;
                self.last_encoder_for_stall = enc_now;
                false
            } else {
                // Encoder not moving
                if self.stall_timer_start.is_none() {
                    self.stall_timer_start = Some(Instant::now());
                    self.last_encoder_for_stall = enc_now;
                    false
                } else if self.stall_timer_start.unwrap().elapsed().as_millis() as u64 > STALL_TIME_MS {
                    log::warn!("STALL DETECTED (encoder not changing) -> STOP");
                    true
                } else {
                    false
                }
            }
        } else {
            self.stall_timer_start = None;
            self.last_encoder_for_stall = enc_now;
            false
        }
    }
    
    /// Move to target angle using encoder-based control (L2 state)
    /// Implements overshoot steps and encoder-based termination like the C++ code
    fn move_to_angle_encoder_based(&mut self, target_angle_deg: f64, max_speed_steps_per_sec: f64) -> bool {
        log::info!("===== L2 ENCODER-BASED MOVE START =====");
        log::info!("TargetAngle_deg: {:.2}", target_angle_deg);
        
        // Log current encoder status before reset
        let (a, b, state, count) = self.get_encoder_status();
        log::info!("Encoder before reset: A={}, B={}, state={:02b}, count={}", a, b, state, count);
        
        // Reset encoder
        self.reset_encoder();
        
        // Calculate encoder target
        self.encoder_target = Some(self.calculate_encoder_target(target_angle_deg));
        log::info!("EncoderTarget_cnt: {}", self.encoder_target.unwrap());
        log::info!("MaxSpeed_steps_s: {:.2}", max_speed_steps_per_sec);
        
        // Calculate overshoot steps (2 full revolutions as in C++ code)
        let overshoot_steps = (OUT_STEPS_PER_REV * 2.0) as i64;
        
        // Enable motor and configure
        self.relay.set_high().unwrap_or_default();
        self.motor.set_current_position(0);
        self.motor.set_max_speed(max_speed_steps_per_sec as f32);
        self.motor.set_acceleration(self.acceleration as f32);
        self.motor.move_to(overshoot_steps);
        
        self.encoder_stop_armed = true;
        self.stall_timer_start = None;
        self.last_encoder_for_stall = 0;
        
        log::info!("Starting encoder-based movement with overshoot");
        
        // Main movement loop with encoder monitoring
        loop {
            // Poll stepper motor
            if self.motor.is_running() {
                let _ = self.motor.poll(&mut self.motor_device, &self.motor_clock);
            }
            
            // Update encoder position from GPIO pins
            self.update_encoder_from_pins();
            
            // Log detailed status every 10 iterations (adjust frequency as needed)
            self.log_counter += 1;
            if self.log_counter % 10 == 0 {
                self.log_detailed_status();
            }
            
            // Check encoder-based termination
            if self.encoder_stop_armed {
                if let Some(target) = self.encoder_target {
                    if self.encoder_count.abs() >= target.abs() {
                        log::info!("ENCODER TARGET REACHED -> STOP");
                        self.motor.stop();
                        self.encoder_stop_armed = false;
                        break;
                    }
                }
            }
            
            // Check for stall
            if self.check_stall() {
                self.motor.stop();
                self.encoder_stop_armed = false;
                log::warn!("Movement stopped due to stall detection");
                break;
            }
            
            // Break if motor finished (shouldn't happen with overshoot, but safety check)
            if !self.motor.is_running() && self.motor.distance_to_go() == 0 {
                break;
            }
            
            // Small delay to prevent busy waiting
            thread::sleep(Duration::from_millis(1));
        }
        
        // Wait for motor to fully stop
        while self.motor.is_running() || self.motor.distance_to_go() != 0 {
            let _ = self.motor.poll(&mut self.motor_device, &self.motor_clock);
            thread::sleep(Duration::from_millis(1));
        }
        
        // Disable motor
        self.relay.set_low().unwrap_or_default();
        self.encoder_stop_armed = false;
        
        log::info!("===== L2 MOVE COMPLETE =====");
        log::info!("FinalStepper_steps: {}", self.motor.current_position());
        log::info!("FinalEncoder_cnt: {}", self.encoder_count);
        
        // Log final encoder pin states
        let (a, b, state, _) = self.get_encoder_status();
        log::info!("Final encoder pins: A={}, B={}, state={:02b}", a, b, state);
        
        // Update location based on encoder position
        let final_angle = (self.encoder_count as f64 / ENCODER_COUNTS_PER_REV as f64) * 360.0;
        self.update_position(final_angle as f32);
        log::info!("Final angle from encoder: {:.2}°", final_angle);
        
        true
    }

    pub fn find_limit_switch_cw(&mut self) -> bool {
       
        if self.lmsw.is_low() {
            log::info!("Found Limit Switch, Heading : 90");
            self.update_position(90.0);
            return true;
        }

        log::info!("Move 15 Degress clockwise first");
        self.relay.set_high().unwrap_or_default();

        let correction_factor = 1.231;
        let steps = (15.0 / 360.0) * (25600.0 * 50.0 * 84.0); //* correction_factor;
        log::info!("Steps Needed: {}", steps);
        log::info!("Steps Needed: {}", steps as i64);
        self.move_by(steps as i64);
        self.run();    // Blocking 
        log::info!("Done moving 15 Degress clockwise");
        
        log::info!("Now, looking for the limit switch");

        let mut max_steps = calculate_steps(-360.0);
        while (max_steps < 0 && self.lmsw.is_high()) {
            let step_movement = calculate_steps(-1.0);
            self.move_by(step_movement);
            max_steps -= step_movement;
        }

        self.relay.set_low().unwrap_or_default();
        if max_steps < 0 {
            log::info!("Found Limit Switch, Heading : 90");
            self.update_position(90.0);
            self.relay.set_low().unwrap_or_default();
            return true;
        }
        log::error!("Limit Switch was not found!");
        return false;
    }


    pub fn find_limit_switch_ccw(&mut self) -> bool {
        
        if self.lmsw.is_low() {
            self.update_position(90.0);
            return true;
        }
        
        log::info!("Move 15 Degress clockwise first");
        self.relay.set_high().unwrap_or_default();

        let correction_factor = 1.231;
        let steps = (15.0 / -360.0) * (25600.0 * 50.0 * 84.0) * correction_factor;
        log::info!("Steps Needed: {}", steps);
        log::info!("Steps Needed: {}", steps as i64);
        self.move_by(steps as i64);
        self.run();    // Blocking 
        log::info!("Done moving 15 Degress clockwise");
        log::info!("Now, looking for the limit switch");

        let mut max_steps = calculate_steps(360.0); // full CW
        while (max_steps > 0 && self.lmsw.is_high()) {
            let step_movement = calculate_steps(1.0); // Move 1 deg at a time
            self.move_by(step_movement);
            max_steps -= step_movement;
        }

        self.relay.set_low().unwrap_or_default();

        if max_steps > 0 {
            self.update_position(90.0);
            self.relay.set_low().unwrap_or_default();
            return true;
        }
        false
    }



    pub fn set_tower_position<I2C: embedded_hal::i2c::I2c, T: NvsPartitionId>(
        &mut self,
        clock: &mut Clock<I2C>,
        location: f32,
        balance: i32,
        mqtt: &mut Mqtt,
        current_version: Version,
        nvs: &mut EspNvs<T>,
        wifi: &mut Wifi<'_>,
        formatted_time: String,
    ) -> bool {
        self.update_position(location);
        log::info!("{},", clock.after_sunrise());
        if clock.after_sunrise() && !clock.after_sunset() {
            let sun = NOAASun {
                year: clock.get_year(),
                doy: clock.get_day() as u16,
                long: clock.get_longitude() as f32,
                lat: clock.get_latitude() as f32,
                timezone: -5.0,
                hour: clock.get_hour(),
                min: clock.get_minutes(),
                sec: clock.get_seconds(),
            };
            log::info!("Tracking in progress");
            let angle_offset = sun.azimuth_in_deg() - (location as f64);
            log::info!("Actual Location: {}", location);
            log::info!("Angle Offset: {}", angle_offset);
            log::info!("Sun Angle: {}", sun.azimuth_in_deg());
            
            // Log encoder status periodically
            let (enc_a, enc_b, enc_state, enc_count) = self.get_encoder_status();
            log::info!("Encoder status: A={}, B={}, state={:02b}, count={}", 
                      enc_a, enc_b, enc_state, enc_count);
            if angle_offset.abs() > 5.0 {
                self.relay.set_high().unwrap_or_default();
                self.tracking_state = TrackingState::L1;
            }
            if angle_offset.abs() <= 5.0 && self.tracking_state == TrackingState::L1 {
                let _ = self.relay.set_low().unwrap_or_default();
                return true; // New line
                //self.tracking_state = TrackingState::L2;
            }
            match self.tracking_state {
                TrackingState::L1 => {
                    let correction_factor = 1.3;
                    log::info!("Tracking state L1");
                    let steps = (angle_offset / 360.0) * (25600.0 * 50.0 * 84.0); // * correction_factor; // Change to -360 for waco 
                    log::info!("Steps Needed: {}", steps as i64);
                    self.move_by(steps as i64);
                    self.run();    // Blocking 
                    // log::info!("Angle Offset: {}", angle_offset);
                    self.update_position((location as f64 + angle_offset) as f32);
                    log::info!("Exiting Tracking state L1");
                    self.relay.set_low().unwrap_or_default(); // New line

                    //Publish message
                    let payload = format!(
                        "Current datetime: {}, and current tower angle: {}",
                        formatted_time, 
                        location as f64 + angle_offset
                    );
                    match mqtt.publish("device1A/data", payload.as_bytes()){
                        Ok(_) => log::info!("Published data payload successfully"),
                        Err(e) => log::error!("Failed to publish data payload: {:?}", e),
                    }
                    return false;
                }
                TrackingState::L2 => {
                    // ========================================================================
                    // L2 STATE: ENCODER-BASED PRECISION TRACKING
                    // ========================================================================
                    // Implements encoder-based control similar to C++ implementation
                    // Uses encoder feedback for precise positioning with overshoot and stall detection
                    
                    log::info!("Tracking state L2 - Encoder-based precision tracking");
                    
                    // If angle offset is too large, fall back to L1
                    if angle_offset.abs() > 5.0 {
                        log::info!("Angle offset > 5°, falling back to L1 state");
                        self.prev_balance = 0;
                        self.tracking_state = TrackingState::L1;
                        return false;
                    }
                    
                    // Calculate target angle for encoder-based movement
                    let target_angle = location as f64 + angle_offset;
                    log::info!("L2 Target angle: {:.2}° (current: {:.2}°, offset: {:.2}°)", 
                               target_angle, location, angle_offset);
                    
                    // Use encoder-based movement with overshoot
                    // Max speed in steps/sec (using current speed setting)
                    let max_speed_steps_s = self.speed as f64;
                    
                    // Perform encoder-based movement
                    let move_success = self.move_to_angle_encoder_based(target_angle, max_speed_steps_s);
                    
                    if move_success {
                        // Movement completed successfully
                        let new_location = self.location();
                        log::info!("L2 movement complete. New location: {:.2}°", new_location);
                        
                        // Check if we're close enough to target
                        let final_offset = target_angle - new_location as f64;
                        if final_offset.abs() <= 1.0 {
                            // Close enough, return to L1 for next cycle
                            log::info!("L2 target reached within tolerance, returning to L1");
                            self.prev_balance = 0;
                            self.tracking_state = TrackingState::L1;
                            return true;
                        } else {
                            // Still need fine-tuning, stay in L2
                            log::info!("L2 fine-tuning needed, offset: {:.2}°", final_offset);
                            return false;
                        }
                    } else {
                        // Movement failed or stalled
                        log::warn!("L2 movement failed or stalled, staying in L2");
                        return false;
                    }
                }
                TrackingState::L3 => (), // Future tracking 
            }
        } 
        else {// Sunset Operation 
            if location == 90.0 {
                log::info!("Already reached sleep position");

                // Track start time
                let mut last_check = Instant::now();
                let check_interval = Duration::from_secs(2 * 60 * 60); // 2 hours

                // Wait here until sunrise
                while clock.after_sunset() || !clock.after_sunrise() {
                    if clock.after_sunrise() && !clock.after_sunset() {
                        log::info!("Sunrise detected, exiting sleep loop");
                        break;
                    }
                    if last_check.elapsed() >= check_interval {
                        log::info!("2 hours elapsed, checking for OTA");

                        // Check to see if wifi is disconnected before OTA try
                        log::info!("Current wifi state: {:?}", wifi.state());
                        if wifi.state() == WifiState::Disconnected{
                            wifi.reconnect_if_disconnected();
                        }

                        // Creates an instance of OTA crate and runs version compare
                        thread::sleep(Duration::from_secs(3));
                        let mut updater = OtaUpdater::new_ota(current_version.clone(), mqtt, Some("device1A"), Some("device1A")).expect("Failed to create OTA udater instance");

                        thread::sleep(Duration::from_secs(3));
                        let run_compare = updater.run_version_compare(nvs);

                        match run_compare {
                            Ok(_) => log::info!("Version compare succeeded"),
                            Err(e) => {
                                log::error!("Version compare failed: {:?}", e);
                            }
                        } 

                        last_check = Instant::now(); // reset the timer
                        //break;
                    }
                    log::info!("Still waiting for sunrise...");
                    std::thread::sleep(std::time::Duration::from_secs(600)); // Prevent busy waiting
                }

                return true;
            } else {
                log::info!("Moving to sleep position...");
                let limit_sw_status = self.find_limit_switch_cw(); // change to ccw for waco
                match limit_sw_status{
                    true => log::info!("Limit switch has returned true"),
                    false => {
                        log::error!("Limit switch has returned false, limit switch could not be found");
                        loop{
                            if let Err(e) = mqtt.publish("device1A/tower/status", b"Critical failure: Limit switch failure!") {
                                log::error!("Failed to publish critical error message: {:?}", e);
                            }
                            thread::sleep(Duration::from_secs(900));// Loop every 15 minutes
                        }
                    }
                }
                log::info!("Tower has reached sleep position");
                return false;
            }
        }
        true

        /*else if clock.after_sunset() {
             if self.tracking_state != TrackingState::L3 {
                let angle_offset = 90.0 - location;
                let steps = (angle_offset / 360.0) * (20000.0 * 50.0 * 84.0);
                log::info!("Steps Needed: {}", steps);
                log::info!("Steps Needed: {}", steps as i64);
                self.move_by(steps as i64);
                self.relay.set_high().unwrap_or_default();
                self.run();
                self.relay.set_low().unwrap_or_default();
                self.update_position(90.0);
                self.tracking_state = TrackingState::L3;
                return true; 

                
            }
        }
        true
        */
    }
}

