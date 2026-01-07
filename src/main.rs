use std::{
    time::{Duration, SystemTime},
};
use chrono::{DateTime, FixedOffset, Utc};
use clock::Clock;
use log::*;
use std::thread;
//use esp32_nimble::{enums::*, uuid128, BLEAdvertisedDevice, BLEDevice, BLEScan};
use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    hal::{
        gpio::PinDriver,
        i2c::{I2cConfig, I2cDriver},
        peripherals::Peripherals,
        prelude::*,
        //        task::block_on,
    },
    log::EspLogger,
    nvs::{EspDefaultNvsPartition, EspNvs},
    ota::EspOta,
    sntp::{EspSntp, SyncStatus},
};
use motion::Motion;
use rgb_led::Led;
use network::mqtt::Mqtt;
use ota::OtaUpdater;
use semver::Version;
use wifi::wifi::{Wifi, WifiState};

#[no_mangle]
pub extern "C" fn __pender() {
    // For ESP-IDF with FreeRTOS, this is typically a no-op when using
    // the embassy-time-driver feature, as the time driver handles wake-ups
    // This stub satisfies the linker requirement
}

fn main() -> anyhow::Result<()> {
    // Required for ESP-IDF patches
    esp_idf_svc::sys::link_patches();
    //esp_app_desc!(); // FIND PURPOSE ELSE DELETE

    // Initialize logger and system loop
    EspLogger::initialize_default();
    let sysloop = EspSystemEventLoop::take()?;

    // Initialize peripherals and nvs
    let peripherals = Peripherals::take().unwrap();
    let nvs_default = EspDefaultNvsPartition::take()?;
    let mut nvs = match EspNvs::new(nvs_default.clone(), "storage", true) {
        Ok(nvs) => {
            info!("Got namespace {:?} from default partition", "storage");
            nvs
        }
        Err(e) => panic!("Could't get namespace {:?}", e),
    };

    // Setting of sda and scl gpio pins as well as i2c
    let sda = peripherals.pins.gpio8;
    let scl = peripherals.pins.gpio9;

    // I2C configuration
    let config = I2cConfig::new().baudrate(10_u32.kHz().into());
    let i2c = I2cDriver::new(peripherals.i2c0, sda, scl, &config).unwrap();

    // Setting up i2c bus driver
    let bus: &'static _ = shared_bus::new_std!(I2cDriver = i2c).unwrap();
    // let mut buttons = buttons::Buttons::new(
    //     peripherals.pins.gpio5,
    //     peripherals.pins.gpio4,
    //     peripherals.pins.gpio6,
    // );

    // ======== Wifi: Initialization ========
    let mut wifi = Wifi::new(peripherals.modem, sysloop.clone(), nvs_default)?;
	wifi.connect("Power2", "@Powerfuture22").expect("Wi-Fi connection failed");
	info!("Current wifi state: {:?}", wifi.state());
    if wifi.state() == WifiState::Disconnected{
        wifi.reconnect_if_disconnected()?;
    }

    // Initializing ntp and local time
    let ntp = EspSntp::new_default().unwrap();
    info!("Synchronizing with NTP Server");
    while ntp.get_sync_status() != SyncStatus::Completed {}
    info!("Time Sync Completed");

    let st_now = SystemTime::now();
    let dt_now_utc: DateTime<Utc> = st_now.clone().into();
    let timezone_offset_hours: i32 = -5; 
    let local_time: DateTime<FixedOffset> = DateTime::from_naive_utc_and_offset(
        dt_now_utc.naive_utc(),
        //FixedOffset::west_opt(5 * 3600).unwrap(),              // 
        FixedOffset::east_opt(timezone_offset_hours * 3600).unwrap(),  
    );

    let formatted_time = format!("{}", local_time.format("%d/%m/%Y %H:%M:%S"));
    info!("{}", formatted_time);

    // ======== Mqtt: Initialization ========
    // Create MQTT client using the Wi-Fi TCP/IP stack
    let mut mqtt = Box::new(Mqtt::new_mqtt(
        "mqttS://mqtt.jantaus.com:9443",
        "device1A_pub",
        "",
        "",
    )?);

    // ======== Boot Validation ========    
    let first_boot = nvs.get_u8("first_boot")?.unwrap_or(1);

    // Run boot_diagnostic check
    let boot_diagnostic_result = boot_diagnostic(&mut wifi, &mut mqtt);

    if first_boot == 1 {
        info!("First boot, now performing boot diagnostics");
        let mut valid_ota = EspOta::new().expect("Failed to get OTA instance");// Minimal OTA instance for validation

        let running_slot = valid_ota.get_running_slot();
        info!("This is the running boot slot {:?}", running_slot);

        if running_slot.unwrap().label == "factory" {
            info!("Running from factory partition -> skipping OTA validity marking");
            nvs.set_u8("first_boot", 0)?;
        } else{
            // Mark firmware valid or rollback
            if boot_diagnostic_result {
                info!("Boot validation passed, now marking firmware as valid");
                valid_ota.mark_running_slot_valid()?;
                nvs.set_u8("first_boot", 0)?;
                
            } else {
                error!("Boot validation failed, rolling back firmware");
                valid_ota.mark_running_slot_invalid_and_reboot(); // reboots immediately
            }
        }
    }else {
        info!("Normal boot firmware already validated");
    }

    // ======== OTA: Initialization ========
    // Create a version buffer large enough for the version string
    let mut version_buf = [0u8; 32]; // Adjust size as needed

    info!("Setting the firmware version..."); // RESETTING THE FIRMWARE VERSION
    nvs.set_str("version", "1.0.0")?; // RESETTING THE FIRMWARE VERSION

    // Default version if nothing is stored
    const DEFAULT_VERSION: &str = "1.0.0";
    // Read the current version from NVS
    let current_version: Version = nvs
        .get_str("version", &mut version_buf)?
        .map(|s| s.trim().parse::<Version>())
        .transpose()? // converts Option<Result<..>> into Result<Option<..>>
        .unwrap_or_else(|| Version::parse(DEFAULT_VERSION).unwrap());

    // Publishing a message about the current running version
    info!("The current firmware version is: {}", current_version.to_string());
    let mut payload = format!("The current firmware version is: {}", current_version.to_string());
    mqtt.publish("device1A/firmware/version", payload.as_bytes())?;

    // Creates an instance of OTA crate
    let mut updater = OtaUpdater::new_ota(current_version.clone(), &mut mqtt, Some("device1A"), Some("device1A")).expect("Failed to create OTA updater instance");

    // Run version compare
    info!("Checking for new OTA update in 3 seconds...");
    thread::sleep(Duration::from_secs(3));
    updater.run_version_compare(&mut nvs)?;
    
    // Load tower configuration values
    let tower_id: u32 = 1;
    let latitude: f64 = 32.797868;
    let longitude: f64 = -96.835597;
    let altitude: f64 = 0.0; 
    
    info!("Tower id: {}, Lat: {}, Lon: {}, Alt: {}", tower_id, latitude, longitude, altitude);
    
    // Set new instance of clock crate 
    let mut calculation = Clock::new(bus.acquire_i2c(), latitude, longitude, altitude); // Create a new clock object 
    calculation.set_date_time(&local_time.naive_local()); // Set the current date and time 
    
    //let mut relay = PinDriver::output(peripherals.pins.gpio15).unwrap();
    //let mut lmsw = PinDriver::input(peripherals.pins.gpio6).unwrap();
    //lmsw.set_pull(esp_idf_hal::gpio::Pull::Down);
    let mut led = Led::new(peripherals.pins.gpio7, peripherals.rmt.channel0).unwrap();

    // Set motion pins
    let mut motion = Motion::new(
        peripherals.pins.gpio15,   // CCW Motor
        peripherals.pins.gpio16,   // CW Motor
        peripherals.pins.gpio17,   // Relay 
        peripherals.pins.gpio14,   // Limit Switch 
        peripherals.pins.gpio47,   // Encoder A
        peripherals.pins.gpio21,   // Encoder B
    );
   
    motion.init();          // Initialize motor driver parameters
    led.display_healthy();  // Show healthy LED status
    motion.run();           // Ensure motor driver is in a ready state

    // Initialize and store the actual tracker heading in NVS
    let heading_tag = "heading";
    let mut actual_heading: f32 = 90.0;

      // Write initial heading to NVS (as raw bits)
    match nvs.set_u32(heading_tag, actual_heading.to_bits()) {
         Ok(_) => info!("heading updated"),
         // You can find the meaning of the error codes in the output of the error branch in:
         // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/error-codes.html
         Err(e) => error!("heading not updated {:?}", e),
    }; 

    // Load heading from NVS if present, otherwise keep default
    match nvs.get_u32(heading_tag).unwrap() {
        Some(v) => {
            info!("{:?} = {:?}", heading_tag, v);
            actual_heading = f32::from_bits(v);
        }
        None => info!("{:?} not found", heading_tag),
    }; 

    let mut mb = PinDriver::input(peripherals.pins.gpio5).unwrap();  // Maintenance 
    let mut eb = PinDriver::input(peripherals.pins.gpio4).unwrap();  // East Button
    let mut wb = PinDriver::input(peripherals.pins.gpio6).unwrap();  // West Button

    // --- ONE-TIME HOMING: Ensure tracker is at physical reference before tracking begins ---
    let limit_sw_status = motion.find_limit_switch_cw();
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
    // Find_limit_switch_cw
    thread::sleep(Duration::from_secs(5)); // 

    loop {

        info!("Actual Heading: {}", motion.location());
        //std::thread::sleep(Duration::from_secs(10)); // 5-minute cycle */

        let now = std::time::Instant::now();  // Timer to measure how long this tracking loop iteration takes

        // Perform solar tracking
        let tracking_done = motion.set_tower_position(&mut calculation, actual_heading, 0, &mut mqtt, current_version.clone(), &mut nvs, &mut wifi);

        // Update heading if movement occurred
        if !tracking_done {
            actual_heading = motion.location();
            match nvs.set_u32(heading_tag, actual_heading.to_bits()) {
                Ok(_) => info!("Stored stable heading in NVS: {}", actual_heading),
                Err(e) => warn!("Failed to store heading in NVS: {:?}", e),
            }
        } else {
            info!("True return from set tower position");
            info!("Angle offset is less then 5");
        }

        info!("Tracking loop duration (v1.0.0): {:?}", now.elapsed());
        if wifi.state() == WifiState::Disconnected{
            warn!("Wifi disconnected, attempting to reconnect...");
            wifi.reconnect_if_disconnected()?;
        }
        payload = format!("The current firmware version is: {}", current_version.to_string());
        mqtt.publish("device1A/firmware/version", payload.as_bytes())?;
        
        std::thread::sleep(Duration::from_secs(300)); // 5-minute cycle  

    }
    //loop {
        //log::info!("Limit Switch On: {}", motion.switch_pressed());
        //motion.find_limit_switch();
        //let now = std::time::Instant::now();                    // Starts a high-resolution timer to measure elapsed time
        //std::thread::sleep(Duration::from_millis(1000));        // Pauses the thread for 1000 milliseconds (1 second)
        //std::thread::sleep(Duration::from_nanos(50));

/*         log::info!("Time Gap: {:?}", now.elapsed());            // Logs how much time passed since now
        log::info!("Maintenance Pressed: {:?}", mb.is_high());  // Reads GPIO input states
        log::info!("East Pressed: {:?}", eb.is_high());
        log::info!("West Pressed: {:?}", wb.is_high()); */

        // esp_idf_svc::hal::delay::FreeRtos::delay_ms(5000);    // Pause the thread for 5 seconds
        // motion.flip_relay();                                 // Toggle motor 
        // motion.move_by(((20.0 / 360.0) * (20000.0 * 50.0 * 84.0)) as i64);
        // motion.flip_relay();
        // esp_idf_svc::hal::delay::FreeRtos::delay_ms(10000);
        // log::info!("Limit Switch On: {}", lmsw.is_high());
        // log::info!("Limit Switch Level: {:?}", lmsw.get_level());
        // motion.flip_relay();
        // motion.move_by(((-20.0 / 360.0) * (20000.0 * 50.0 * 84.0)) as i64);
        // motion.flip_relay();
        // esp_idf_svc::hal::delay::FreeRtos::delay_ms(10000);

        // let val = motion.set_tower_position(&mut calculation, actual_heading as f32, 0);
        // let val = motion.set_tower_position(&mut calculation, actual_heading as f32, 0);
        // actual_heading = motion.location();
        // log::info!("{},", actual_heading);

        // match nvs.set_u32(heading_tag, actual_heading.to_bits()) {
        //     Ok(_) => log::info!("heading updated"),
        //     // You can find the meaning of the error codes in the output of the error branch in:
        //     // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/error-codes.html
        //     Err(e) => log::info!("heading not updated {:?}", e),
        // };
        // esp_idf_svc::hal::delay::FreeRtos::delay_ms(180000);
    //}
 
/*     loop {
        block_on(async {
            let ble_device = BLEDevice::take();
            ble_device
                .set_power(PowerType::Default, PowerLevel::P9)
                .unwrap_or_default();
            let device_name = b"Axum-0.1";
            let mut ble_scan = BLEScan::new();
            loop {
                esp_idf_svc::hal::delay::FreeRtos::delay_ms(1500);
                let device = ble_scan
                    .active_scan(true)
                    .interval(100)
                    .window(99)
                    .start(ble_device, 5000, |device, data| {
                        if let Some(name) = data.name() {
                            if **name == *device_name {
                                //log::info!("{:?}", data.service_data());
                                //log::info!("RSSI: {}", device.rssi());
                                return Some(*device);
                            }
                        }
                        None::<BLEAdvertisedDevice>
                    })
                    .await;

                if let Ok(Some(device)) = device {
                    let mut client = ble_device.new_client();
                    client.on_connect(|client| {
                        log::info!(" ");
                        client
                            .update_conn_params(120, 120, 0, 65535)
                            .unwrap_or_default();
                    });
                    client.connect(&device.addr()).await.unwrap_or_default();
                    loop {
                        let mut actual_balance = 0;
                        let service = client
                            .get_service(uuid128!("00001812-0000-1000-8000-00805f9b34fb"))
                            .await;
                        if let Ok(service) = service {
                            let mut characteristics =
                                service.get_characteristics().await.unwrap_or_default();
                            let ldre = characteristics.next();
                            if let Some(ldre) = ldre {
                                let ldr = ldre.read_value().await;
                                if let Ok(ldr) = ldr {
                                    let actual = <[u8; 4]>::try_from(ldr).unwrap();
                                    //actual_balance = i32::from_ne_bytes(actual);
                                    //log::info!("LDRE: {}", i32::from_ne_bytes(actual));
                                }
                            }

                            let ldrw = characteristics.next();
                            if let Some(ldrw) = ldrw {
                                let ldr = ldrw.read_value().await;
                                if let Ok(ldr) = ldr {
                                    let actual = <[u8; 4]>::try_from(ldr).unwrap();
                                    //actual_balance = i32::from_ne_bytes(actual);
                                    //log::info!("LDRW: {}", i32::from_ne_bytes(actual));
                                }
                            }

                            let bal = characteristics.next();
                            if let Some(bal) = bal {
                                let balance = bal.read_value().await;
                                if let Ok(balance) = balance {
                                    let actual = <[u8; 4]>::try_from(balance).unwrap();
                                    actual_balance = i32::from_ne_bytes(actual);
                                    //log::info!("Balance: {}", i32::from_ne_bytes(actual));
                                }
                            }
                        } else {
                            break;
                        }
                        log::info!("Heading: {}", actual_heading);
                        log::info!("Balance: {}", actual_balance);

                        if actual_heading == 0.0 {
                            break;
                        }
                        let val = motion.set_tower_position(
                            &mut calculation,
                            actual_heading as f32,
                            actual_balance,
                        );
                        actual_heading = motion.location();
                        match nvs.set_u32(heading_tag, actual_heading.to_bits()) {
                            Ok(_) => log::info!("heading updated"),
                            // You can find the meaning of the error codes in the output of the error branch in:
                            // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/error-codes.html
                            Err(e) => log::info!("heading not updated {:?}", e),
                        };
                        if !val {
                            continue;
                        }
                        esp_idf_svc::hal::delay::FreeRtos::delay_ms(120000);
                    }
                    client.disconnect().unwrap_or_default();
                }
                //let data = deserialize(device_data.1.service_data().unwrap().service_data);
                //log::info!("{:?}", data);
                //let val = motion.set_tower_position(&mut calculation, data.loc, data.acc, data.bgp);}
            }
            //anyhow::Ok(())
        })
    } */
}

fn boot_diagnostic(wifi: &mut Wifi, mqtt: &mut Mqtt) -> bool {
    // Let system settle
    info!("Starting boot validation in 5 seconds...");
    thread::sleep(Duration::from_secs(5));

    // Wifi check
    match wifi.state() {
        WifiState::Connected(ip) => {
            info!("Wi-Fi connected with IP: {}", ip);
        }
        WifiState::Connecting => {
            warn!("Wi-Fi still connecting during validation...");
            return false;
        }
        WifiState::Disconnected => {
            error!("Wi-Fi disconnected, validation failed");
            return false;
        }
    }

    // MQTT check
    const MAX_RETRIES: u8 = 3;

    for attempt in 1..=MAX_RETRIES {
        info!("Boot diagnostic MQTT attempt {}/{}", attempt, MAX_RETRIES);

        // Wait until the MQTT client reports connected
        let mut waited = 0;
        while !mqtt.is_connected() && waited < 12000 {
            thread::sleep(Duration::from_millis(3000));
            waited += 3000;
        }

        if !mqtt.is_connected() {
            warn!("MQTT not connected yet, retrying...");
            continue; // next attempt
        }

        // Try publishing a test message
        match mqtt.publish("device1A/boot", b"Boot check...") {
            Ok(_) => {
                info!("MQTT boot diagnostic publish succeeded...");
                return true;
            }
            Err(e) => {
                error!("MQTT publish failed immediately: {:?}", e);
                if attempt == MAX_RETRIES {
                    error!("All MQTT boot diagnostic attempts failed...");
                    return false; // give up after max retries
                }
                thread::sleep(Duration::from_millis(1000)); // backoff
                continue;
            }
        }
    }
    return false; 
    //return true;
}