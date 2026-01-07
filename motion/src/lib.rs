pub mod motion {
    use accel_stepper::{Driver, OperatingSystemClock, StepAndDirection};
    use astronav::coords::noaa_sun::NOAASun;
    use clock::Clock;
    use esp_idf_svc::hal::gpio::{
        Gpio14, Gpio15, Gpio16, Gpio17, Gpio21, Gpio47, Input, Output, PinDriver,
    };
    use esp_idf_svc::nvs::*;
    use network::mqtt::Mqtt;
    use ota::OtaUpdater;
    use semver::Version;
    use std::time::{Duration, Instant};
    use std::thread;
    use wifi::wifi::{Wifi, WifiState};

    // Constants
    const ENCODER_COUNTS_PER_REV: i64 = 348_323;
    const NVS_KEY_LAST_ENCODER: &str = "last_enc_cnt";

    // Quadrature decode table (robust, no branching)
    static QUAD_TABLE: [i8; 16] = [
        0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0,
    ];

    #[derive(PartialEq)]
    enum TrackingState {
        L1,
        L2,
        L3,
    }

    pub fn calculate_steps(offset: f32) -> i64 {
        ((offset / 360.0) * (25600.0 * 5.0 * 84.0)) as i64
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

        // Encoder inputs + state
        enc_a: PinDriver<'a, Gpio47, Input>,
        enc_b: PinDriver<'a, Gpio21, Input>,
        encoder_count: i64,
        last_ab: u8,

        // Encoder NVS persistence
        encoder_loaded_from_nvs: bool,
        last_encoder_persist: Instant,
    }

    impl Motion<'_> {
        /// Convert encoder ticks to motor steps
        /// Ratio: ~308.7 motor steps per encoder tick
        fn encoder_ticks_to_motor_steps(encoder_ticks: i64) -> i64 {
            const MOTOR_STEPS_PER_REV: f64 = 25600.0 * 5.0 * 84.0; // 107,520,000
            const ENCODER_TICKS_PER_REV: f64 = ENCODER_COUNTS_PER_REV as f64; // 348,323
            ((encoder_ticks as f64 * MOTOR_STEPS_PER_REV / ENCODER_TICKS_PER_REV).round()) as i64
        }

        /// Convert angle offset to encoder ticks
        fn angle_to_encoder_ticks(angle_offset: f64) -> i64 {
            ((angle_offset / 360.0) * ENCODER_COUNTS_PER_REV as f64).round() as i64
        }

        // CW: direction, CCW: step
        pub fn new<'a>(
            p10: Gpio15,
            p11: Gpio16,
            p7: Gpio17,
            p6: Gpio14,
            enc_a: Gpio47,
            enc_b: Gpio21,
        ) -> Motion<'a> {
            let step = PinDriver::output(p10).unwrap();
            let direction = PinDriver::output(p11).unwrap();
            let relay = PinDriver::output(p7).unwrap();
            let mut lmsw = PinDriver::input(p6).unwrap();
            lmsw.set_pull(esp_idf_svc::hal::gpio::Pull::Down)
                .unwrap_or_default();

            // Encoder pins (use pull-ups like Arduino INPUT_PULLUP)
            let mut enc_a = PinDriver::input(enc_a).unwrap();
            let mut enc_b = PinDriver::input(enc_b).unwrap();
            enc_a.set_pull(esp_idf_svc::hal::gpio::Pull::Up)
                .unwrap_or_default();
            enc_b.set_pull(esp_idf_svc::hal::gpio::Pull::Up)
                .unwrap_or_default();

            let mut m = Motion {
                location: 0.0,
                // Default tracking state: L1 = stepper-only (legacy), L2 = encoder-driven
                // Change to L1 for legacy stepper-only mode, L2 for encoder-driven mode
                tracking_state: TrackingState::L2,
                speed: 43000.0,
                acceleration: 3000,
                motor: Driver::new(),
                motor_device: StepAndDirection::new(step, direction),
                motor_clock: OperatingSystemClock::new(),
                prev_balance: 0,
                relay,
                lmsw,
                enc_a,
                enc_b,
                encoder_count: 0,
                last_ab: 0,
                encoder_loaded_from_nvs: false,
                last_encoder_persist: Instant::now(),
            };

            // Initialize last_ab from current encoder pin states
            m.last_ab = ((m.enc_a.is_high() as u8) << 1) | (m.enc_b.is_high() as u8);

            m
        }

        // Encoder helpers
        fn read_encoder(&mut self) {
            // A in bit1, B in bit0 (same layout as Arduino sketch)
            let ab: u8 = ((self.enc_a.is_high() as u8) << 1) | (self.enc_b.is_high() as u8);
            if ab != self.last_ab {
                let idx = ((self.last_ab << 2) | ab) as usize;
                let delta = QUAD_TABLE[idx] as i64;
                self.encoder_count += delta;
                self.last_ab = ab;
            }
        }

        // Diagnostic: Check if encoder pins are actually changing
        pub fn encoder_pin_states(&self) -> (bool, bool) {
            (self.enc_a.is_high(), self.enc_b.is_high())
        }

        pub fn encoder_count(&self) -> i64 {
            self.encoder_count
        }

        pub fn encoder_degrees(&self) -> f32 {
            (self.encoder_count as f32) * (360.0 / ENCODER_COUNTS_PER_REV as f32)
        }

        pub fn reset_encoder(&mut self) {
            self.encoder_count = 0;
            self.last_ab = ((self.enc_a.is_high() as u8) << 1) | (self.enc_b.is_high() as u8);
        }

        /// Set encoder to 90 degrees position (limit switch position)
        /// This initializes the encoder count to 87080 when limit switch is found
        fn set_encoder_to_limit_switch_position(&mut self) {
            const LIMIT_SWITCH_ENCODER_COUNT: i64 = 87080;
            self.encoder_count = LIMIT_SWITCH_ENCODER_COUNT;
            log::info!(
                "Encoder initialized to limit switch position: count={} (90.0°)",
                LIMIT_SWITCH_ENCODER_COUNT
            );
        }

        fn load_encoder_from_nvs<T: NvsPartitionId>(&mut self, nvs: &mut EspNvs<T>) {
            if self.encoder_loaded_from_nvs {
                return;
            }

            match nvs.get_i64(NVS_KEY_LAST_ENCODER) {
                Ok(Some(v)) => {
                    self.encoder_count = v;
                    log::info!("Loaded encoder_count={} from NVS", v);
                }
                Ok(None) => {
                    log::info!("No stored encoder_count in NVS yet");
                }
                Err(e) => {
                    log::error!("Failed to read encoder_count from NVS: {:?}", e);
                }
            }

            self.encoder_loaded_from_nvs = true;
            self.last_encoder_persist = Instant::now();
        }

        fn persist_encoder_to_nvs<T: NvsPartitionId>(&mut self, nvs: &mut EspNvs<T>) {
            // Rate-limit writes to reduce flash wear
            if self.last_encoder_persist.elapsed() < Duration::from_secs(1) {
                return;
            }

            let v = self.encoder_count;
            if let Err(e) = nvs.set_i64(NVS_KEY_LAST_ENCODER, v) {
                log::error!("Failed to write encoder_count to NVS: {:?}", e);
                return;
            }

            self.last_encoder_persist = Instant::now();
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
        }

        pub fn move_by(&mut self, location: i64) {
            self.motor.move_by(location);
            self.run();
        }

        /// Move by encoder ticks (true closed-loop encoder-driven movement for L2)
        /// Continuously adjusts motor movement based on encoder feedback until target reached
        fn move_by_encoder_ticks(&mut self, encoder_ticks: i64, tolerance: i64) {
            let start_encoder = self.encoder_count;
            let target_encoder = start_encoder + encoder_ticks;
            
            log::info!(
                "Encoder-driven move (closed-loop): {} ticks (from {} to {})",
                encoder_ticks,
                start_encoder,
                target_encoder
            );

            // Constants for closed-loop control
            const MAX_CHUNK_STEPS: i64 = 1000; // Maximum steps per iteration
            const MIN_CHUNK_STEPS: i64 = 10;   // Minimum steps to avoid jitter
            const TICKS_TO_STEPS: f64 = (25600.0 * 5.0 * 84.0) / (ENCODER_COUNTS_PER_REV as f64); // ~308.7

            let mut t0 = Instant::now();
            let mut encoder_reads = 0u64;
            let mut encoder_changes = 0u64;
            let mut iteration = 0u64;

            // True closed-loop: keep moving in chunks until encoder reaches target
            loop {
                iteration += 1;

                // Read encoder multiple times for high-frequency polling
                for _ in 0..20 {
                    let old_count = self.encoder_count;
                    self.read_encoder();
                    encoder_reads += 1;
                    if self.encoder_count != old_count {
                        encoder_changes += 1;
                    }
                }

                // Calculate remaining encoder ticks to target
                let encoder_remaining = target_encoder - self.encoder_count;
                let encoder_error = encoder_remaining.abs();

                // Check if we've reached target (primary completion check)
                if encoder_error <= tolerance {
                    log::info!(
                        "Encoder target reached: {} (target: {}, error: {}) after {} iterations",
                        self.encoder_count,
                        target_encoder,
                        encoder_error,
                        iteration
                    );
                    // Stop motor if it's still running
                    if self.motor.is_running() {
                        self.motor.stop();
                    }
                    break;
                }

                // Calculate how much more to move (in motor steps) based on encoder feedback
                let motor_steps_needed = (encoder_remaining as f64 * TICKS_TO_STEPS).round() as i64;
                
                // Clamp to reasonable chunk size to avoid overshoot
                let motor_steps_chunk = motor_steps_needed.clamp(-MAX_CHUNK_STEPS, MAX_CHUNK_STEPS);
                
                // Only move if chunk is significant enough
                if motor_steps_chunk.abs() >= MIN_CHUNK_STEPS {
                    // Move this chunk
                    self.motor.move_by(motor_steps_chunk);
                    log::debug!(
                        "Iteration {}: Enc remaining: {} ticks, Moving {} motor steps",
                        iteration,
                        encoder_remaining,
                        motor_steps_chunk
                    );
                } else if motor_steps_chunk.abs() > 0 {
                    // Very small remaining movement, do it anyway
                    self.motor.move_by(motor_steps_chunk);
                }

                // Poll motor to execute the movement
                let _ = self.motor.poll(&mut self.motor_device, &self.motor_clock);

                // Read encoder again after motor poll
                for _ in 0..20 {
                    let old_count = self.encoder_count;
                    self.read_encoder();
                    encoder_reads += 1;
                    if self.encoder_count != old_count {
                        encoder_changes += 1;
                    }
                }

                // Print debug every 100ms
                if t0.elapsed() >= Duration::from_millis(100) {
                    let (enc_a_state, enc_b_state) = self.encoder_pin_states();
                    let step_pos = self.motor.current_position();
                    let step_rem = self.motor.distance_to_go();

                    log::info!(
                        "Encoder-driven (closed-loop): Iter: {} | Step pos: {} | Step rem: {} | Enc cnt: {} (target: {}, rem: {}) | Enc deg: {:.2} | Enc reads: {} | Enc changes: {} | Pins: A={} B={}",
                        iteration,
                        step_pos,
                        step_rem,
                        self.encoder_count(),
                        target_encoder,
                        encoder_remaining,
                        self.encoder_degrees(),
                        encoder_reads,
                        encoder_changes,
                        enc_a_state,
                        enc_b_state
                    );

                    encoder_reads = 0;
                    encoder_changes = 0;
                    t0 = Instant::now();
                }

                // Safety: prevent infinite loops (shouldn't happen, but just in case)
                if iteration > 10000 {
                    log::error!(
                        "Encoder-driven move exceeded max iterations ({}), stopping. Encoder at: {} (target: {})",
                        iteration,
                        self.encoder_count,
                        target_encoder
                    );
                    if self.motor.is_running() {
                        self.motor.stop();
                    }
                    break;
                }
            }

            // Final summary
            let step_pos = self.motor.current_position();
            let step_rem = self.motor.distance_to_go();
            let final_error = (self.encoder_count - target_encoder).abs();

            log::info!(
                "ENCODER-DRIVEN MOVE COMPLETE (closed-loop) | Iterations: {} | Stepper pos: {} | Step rem: {} | Enc cnt: {} (target: {}) | Enc error: {} | Enc deg: {:.2}",
                iteration,
                step_pos,
                step_rem,
                self.encoder_count(),
                target_encoder,
                final_error,
                self.encoder_degrees()
            );
        }

        /// Moves the tracker to 60 degrees, enabling relay before moving and disabling it after.
        pub fn move_to_60(&mut self) {
            let current = self.location();
            let offset = 60.0 - current;
            log::info!("Moving from {:.2}° to 60°, offset = {:.2}°", current, offset);

            // Turn ON relay to enable motor movement
            self.relay.set_high().unwrap_or_default();

            // Move by calculated angle
            self.move_by_angle(offset);
            self.run();

            // Update internal position
            self.update_position(60.0);

            // Turn OFF relay after movement for safety/power savings
            self.relay.set_low().unwrap_or_default();

            log::info!("Now at 60°");
        }

        pub fn move_test(&mut self, location: i64) {
            self.relay.set_high().unwrap_or_default();
            self.update_position(15.0);
            self.tracking_state = TrackingState::L2;

            match self.tracking_state {
                TrackingState::L1 => {
                    let steps = (location / 360) * (25600 * 5 * 84);
                    log::info!("Steps Needed: {}", steps as i64);
                    self.move_by(steps as i64);
                    self.run();
                    self.relay.set_low().unwrap_or_default();
                }
                TrackingState::L2 => {
                    log::info!("L2: The encoder based movement test");
                    let required_ticks = (location / 360) * ENCODER_COUNTS_PER_REV;
                    log::info!("Ticks Needed: {}", required_ticks);
                    self.move_by(required_ticks as i64);
                    self.run();
                    self.relay.set_low().unwrap_or_default();
                }
                TrackingState::L3 => (),
            }
        }

        pub fn run(&mut self) {
            let mut t0 = Instant::now();
            let mut encoder_reads = 0u64;
            let mut encoder_changes = 0u64;
            let testing = true;

            loop {
                if testing{
                    if self.motor.is_running() {
                        // ULTRA-TIGHT LOOP: Read encoder as the primary activity
                        // For 348k counts/rev encoder, we need maximum polling frequency
                        // Read encoder multiple times before each motor poll
                        for _ in 0..20 {
                            let old_count = self.encoder_count;
                            self.read_encoder();
                            encoder_reads += 1;
                            if self.encoder_count != old_count {
                                encoder_changes += 1;
                            }
                        }

                        // Poll motor (must be called frequently, but encoder is priority)
                        let _ = self.motor.poll(&mut self.motor_device, &self.motor_clock);

                        // Read encoder again after motor poll
                        for _ in 0..20 {
                            let old_count = self.encoder_count;
                            self.read_encoder();
                            encoder_reads += 1;
                            if self.encoder_count != old_count {
                                encoder_changes += 1;
                            }
                        }

                        // Print debug every 100ms
                        if t0.elapsed() >= Duration::from_millis(100) {
                            let (enc_a_state, enc_b_state) = self.encoder_pin_states();
                            let step_pos = self.motor.current_position();
                            let step_rem = self.motor.distance_to_go();

                            log::info!(
                                "Stepper pos: {} | Step rem: {} | Enc cnt: {} | Enc deg: {:.2} | Enc reads: {} | Enc changes: {} | Pins: A={} B={}",
                                step_pos,
                                step_rem,
                                self.encoder_count(),
                                self.encoder_degrees(),
                                encoder_reads,
                                encoder_changes,
                                enc_a_state,
                                enc_b_state
                            );

                            // Reset counters for next interval
                            encoder_reads = 0;
                            encoder_changes = 0;
                            t0 = Instant::now();
                        }
                    } else {
                        break;
                    }
                    
                    
                }
                else{
                    if self.motor.is_running() {
                        // ULTRA-TIGHT LOOP: Read encoder as the primary activity
                        // For 348k counts/rev encoder, we need maximum polling frequency
                        // Read encoder multiple times before each motor poll
                        for _ in 0..20 {
                            let old_count = self.encoder_count;
                            self.read_encoder();
                            encoder_reads += 1;
                            if self.encoder_count != old_count {
                                encoder_changes += 1;
                            }
                        }

                        // Poll motor (must be called frequently, but encoder is priority)
                        let _ = self.motor.poll(&mut self.motor_device, &self.motor_clock);

                        // Read encoder again after motor poll
                        for _ in 0..20 {
                            let old_count = self.encoder_count;
                            self.read_encoder();
                            encoder_reads += 1;
                            if self.encoder_count != old_count {
                                encoder_changes += 1;
                            }
                        }

                        // Print debug every 100ms
                        if t0.elapsed() >= Duration::from_millis(100) {
                            let (enc_a_state, enc_b_state) = self.encoder_pin_states();
                            let step_pos = self.motor.current_position();
                            let step_rem = self.motor.distance_to_go();

                            log::info!(
                                "Stepper pos: {} | Step rem: {} | Enc cnt: {} | Enc deg: {:.2} | Enc reads: {} | Enc changes: {} | Pins: A={} B={}",
                                step_pos,
                                step_rem,
                                self.encoder_count(),
                                self.encoder_degrees(),
                                encoder_reads,
                                encoder_changes,
                                enc_a_state,
                                enc_b_state
                            );

                            // Reset counters for next interval
                            encoder_reads = 0;
                            encoder_changes = 0;
                            t0 = Instant::now();
                        }
                    } else {
                        break;
                    }
                }
            }

                // Final summary after move completes
                let step_pos = self.motor.current_position();
                let step_rem = self.motor.distance_to_go();

                log::info!(
                    "MOVE COMPLETE | Stepper pos: {} | Step rem: {} | Enc cnt: {} | Enc deg: {:.2}",
                    step_pos,
                    step_rem,
                    self.encoder_count(),
                    self.encoder_degrees()
                );
        }

        pub fn flip_relay(&mut self) {
            self.relay.toggle().unwrap_or_default();
        }

        pub fn find_limit_switch_cw(&mut self) -> bool {
            if self.lmsw.is_low() {
                log::info!("Found Limit Switch, Heading: 90");
                self.update_position(90.0);
                self.set_encoder_to_limit_switch_position();
                return true;
            }

            log::info!("Move 15 degrees clockwise first");
            self.relay.set_high().unwrap_or_default();

            // steps = (angle offset / 360.0) * (microstepping * gear ratio)
            let steps = (15.0 / 360.0) * (25600.0 * 5.0 * 84.0);
            log::info!("Steps Needed: {}", steps as i64);
            self.move_by(steps as i64);
            self.run();
            log::info!("Done moving 15 degrees clockwise");

            log::info!("Now, looking for the limit switch");

            let mut max_steps = calculate_steps(-360.0);
            while max_steps < 0 && self.lmsw.is_high() {
                let step_movement = calculate_steps(-1.0);
                self.move_by(step_movement);
                max_steps -= step_movement;
            }

            self.relay.set_low().unwrap_or_default();
            if max_steps < 0 {
                log::info!("Found Limit Switch, Heading: 90");
                self.update_position(90.0);
                self.set_encoder_to_limit_switch_position();
                return true;
            }
            log::error!("Limit Switch was not found!");
            false
        }

        pub fn find_limit_switch_ccw(&mut self) -> bool {
            if self.lmsw.is_low() {
                self.update_position(90.0);
                self.set_encoder_to_limit_switch_position();
                return true;
            }

            log::info!("Move 15 degrees counter-clockwise first");
            self.relay.set_high().unwrap_or_default();

            let steps = (15.0 / -360.0) * (25600.0 * 5.0 * 84.0);
            log::info!("Steps Needed: {}", steps as i64);
            self.move_by(steps as i64);
            self.run();
            log::info!("Done moving 15 degrees counter-clockwise");
            log::info!("Now, looking for the limit switch");

            let mut max_steps = calculate_steps(360.0);
            while max_steps > 0 && self.lmsw.is_high() {
                let step_movement = calculate_steps(1.0);
                self.move_by(step_movement);
                max_steps -= step_movement;
            }

            self.relay.set_low().unwrap_or_default();

            if max_steps > 0 {
                self.update_position(90.0);
                self.set_encoder_to_limit_switch_position();
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
        ) -> bool {
            // Load encoder count once per boot
            self.load_encoder_from_nvs(nvs);

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

                // Independent L1 and L2 implementations - no state transitions between them
                match self.tracking_state {
                    TrackingState::L1 => {
                        // L1: Pure stepper movement (legacy mode)
                        log::info!("Tracking state L1 (stepper-only mode)");
                        
                        if angle_offset.abs() <= 0.1 {
                            // Close enough, no movement needed
                            log::info!("Angle offset within tolerance, no movement needed");
                            return true;
                        }

                        self.relay.set_high().unwrap_or_default();
                        let steps = (angle_offset / 360.0) * (25600.0 * 50.0 * 84.0);
                        log::info!("Steps Needed: {}", steps as i64);
                        self.move_by(steps as i64);
                        self.update_position((location as f64 + angle_offset) as f32);
                        log::info!("Exiting Tracking state L1");
                        self.relay.set_low().unwrap_or_default();

                        // Persist encoder after motion (rate-limited)
                        self.persist_encoder_to_nvs(nvs);

                        // Publish message (now includes encoder values)
                        let payload = format!(
                            "L1: Steps needed: {}, current tower angle: {}, encoder_count: {}, encoder_deg: {}",
                            steps,
                            location as f64 + angle_offset,
                            self.encoder_count(),
                            self.encoder_degrees()
                        );
                        match mqtt.publish("device1A/data", payload.as_bytes()) {
                            Ok(_) => log::info!("Published data payload successfully"),
                            Err(e) => log::error!("Failed to publish data payload: {:?}", e),
                        }
                        return false;
                    }
                    TrackingState::L2 => {
                        // L2: Encoder-driven movement (encoder-based mode)
                        log::info!("Tracking state L2 (encoder-driven mode)");
                        
                        if angle_offset.abs() <= 0.1 {
                            // Close enough, no movement needed
                            log::info!("Angle offset within tolerance, no movement needed");
                            return true;
                        }

                        self.relay.set_high().unwrap_or_default();
                        
                        // Convert angle offset to encoder ticks
                        let encoder_ticks = Self::angle_to_encoder_ticks(angle_offset);
                        log::info!("Encoder ticks needed: {} (for {:.2}° offset)", encoder_ticks, angle_offset);
                        
                        // Move using encoder-driven movement (tolerance: ~10 encoder ticks ≈ 0.01°)
                        const ENCODER_TOLERANCE: i64 = 10;
                        self.move_by_encoder_ticks(encoder_ticks, ENCODER_TOLERANCE);
                        
                        // Update position based on actual encoder movement
                        let actual_encoder_movement = self.encoder_degrees() - (location as f32);
                        self.update_position((location as f64 + angle_offset) as f32);
                        
                        log::info!("Exiting Tracking state L2");
                        self.relay.set_low().unwrap_or_default();

                        // Persist encoder after motion (rate-limited)
                        self.persist_encoder_to_nvs(nvs);

                        // Publish message (now includes encoder values)
                        let payload = format!(
                            "L2: Encoder ticks: {}, current tower angle: {}, encoder_count: {}, encoder_deg: {:.2}",
                            encoder_ticks,
                            location as f64 + angle_offset,
                            self.encoder_count(),
                            self.encoder_degrees()
                        );
                        match mqtt.publish("device1A/data", payload.as_bytes()) {
                            Ok(_) => log::info!("Published data payload successfully"),
                            Err(e) => log::error!("Failed to publish data payload: {:?}", e),
                        }
                        return false;
                    }
                    TrackingState::L3 => {
                        log::warn!("Tracking state L3 not implemented");
                        return true;
                    }
                }
            } else {
                // Sunset Operation
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
                            if wifi.state() == WifiState::Disconnected {
                                wifi.reconnect_if_disconnected();
                            }

                            // Creates an instance of OTA crate and runs version compare
                            thread::sleep(Duration::from_secs(3));
                            let mut updater = OtaUpdater::new_ota(
                                current_version.clone(),
                                mqtt,
                                Some("device1A"),
                                Some("device1A"),
                            )
                            .expect("Failed to create OTA udater instance");

                            thread::sleep(Duration::from_secs(3));
                            let run_compare = updater.run_version_compare(nvs);

                            match run_compare {
                                Ok(_) => log::info!("Version compare succeeded"),
                                Err(e) => {
                                    log::error!("Version compare failed: {:?}", e);
                                }
                            }

                            last_check = Instant::now();
                        }
                        log::info!("Still waiting for sunrise...");
                        thread::sleep(Duration::from_secs(600));
                    }

                    return true;
                } else {
                    log::info!("Moving to sleep position...");
                    let limit_sw_status = self.find_limit_switch_cw();
                    match limit_sw_status {
                        true => log::info!("Limit switch has returned true"),
                        false => {
                            log::error!(
                                "Limit switch has returned false, limit switch could not be found"
                            );
                            loop {
                                if let Err(e) = mqtt.publish(
                                    "device1A/tower/status",
                                    b"Critical failure: Limit switch failure!",
                                ) {
                                    log::error!("Failed to publish critical error message: {:?}", e);
                                }
                                thread::sleep(Duration::from_secs(900));
                            }
                        }
                    }

                    // Persist encoder after reaching sleep move attempt
                    self.persist_encoder_to_nvs(nvs);

                    log::info!("Tower has reached sleep position");
                    return false;
                }
            }
            true
        }
    }
}

pub use motion::Motion;