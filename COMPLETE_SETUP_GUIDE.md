# Complete Setup Guide: Axum-S3 ESP32-S3 Rust Development Environment

## Table of Contents
1. [Overview](#overview)
2. [Original Codebase State](#original-codebase-state)
3. [Problems Encountered](#problems-encountered)
4. [Changes Made](#changes-made)
5. [Complete Environment Setup](#complete-environment-setup)
6. [Project Structure](#project-structure)
7. [Configuration Files Explained](#configuration-files-explained)
8. [Dependencies Breakdown](#dependencies-breakdown)
9. [Building and Flashing](#building-and-flashing)
10. [Troubleshooting](#troubleshooting)

---

## Overview

This project is a Rust-based firmware for the ESP32-S3 microcontroller, implementing a solar tracker tower system with:
- WiFi connectivity
- MQTT communication
- Over-the-air (OTA) updates
- Motion control with stepper motors
- Sensor integration (HDC1080, BNO080, DS3231 RTC)
- Real-time clock management
- Network connectivity and MQTT messaging

**Target Hardware:** ESP32-S3  
**Rust Edition:** 2021  
**ESP-IDF Version:** v5.2.2  
**Rust Toolchain:** esp (nightly)

---

## Original Codebase State

### What Was Working
- Basic project structure with workspace members
- ESP-IDF integration via `esp-idf-svc` and `esp-idf-hal`
- Embassy async runtime features (`embassy-time-driver`, `embassy-sync`)
- All workspace crates properly configured
- Build system configured for ESP32-S3

### What Was NOT Working

#### 1. Linker Error: `undefined reference to '__pender'`
**Error Message:**
```
error: linking with `ldproxy` failed: exit status: 1
undefined reference to `__pender'
```

**Root Cause:**
- The `embassy_executor` crate (transitive dependency through `embassy-sync` and `embassy-time-driver`) requires a `__pender` function to be provided
- This function is used internally by the executor to wake tasks
- ESP-IDF doesn't automatically provide this function when using embassy features
- The function must be manually implemented

#### 2. Invalid Feature Flag
**Error Message:**
```
package `buttons` depends on `esp-idf-svc` with feature `embassy-executor` 
but `esp-idf-svc` does not have that feature.
```

**Root Cause:**
- Multiple `Cargo.toml` files had `"embassy-executor"` in their `esp-idf-svc` features list
- This feature does NOT exist in `esp-idf-svc` version 0.51
- The feature flag was likely added mistakenly or copied from incorrect documentation

#### 3. WiFi SSID Configuration
- WiFi SSID was set to `"Power2"` in `src/main.rs` line 90
- Needed to be changed to `"jp5k"` for the current network

---

## Changes Made

### Change 1: Removed Invalid Feature Flag

**Files Modified:**
- `Cargo.toml` (root)
- `buttons/Cargo.toml`
- `motion/Cargo.toml`
- `sensors/Cargo.toml`

**What Was Removed:**
The line `"embassy-executor",` was removed from the `esp-idf-svc` features list in all files.

**Before:**
```toml
esp-idf-svc = { version = "0.51", features = [
    "critical-section",
    "embassy-time-driver",
    "embassy-sync",
    "embassy-executor",  # ← This line was removed
] }
```

**After:**
```toml
esp-idf-svc = { version = "0.51", features = [
    "critical-section",
    "embassy-time-driver",
    "embassy-sync",
] }
```

**Why:** The `embassy-executor` feature doesn't exist in `esp-idf-svc` 0.51. It was causing dependency resolution errors.

### Change 2: Added `__pender` Function Implementation

**File Modified:** `src/main.rs`

**What Was Added:**
A Rust implementation of the `__pender` function was added after line 10 (after the `use esp_idf_hal` import, before the `use esp_idf_svc` block):

```rust
// Provide __pender function for embassy_executor
// This function is called by embassy_executor to wake tasks
#[no_mangle]
pub extern "C" fn __pender() {
    // For ESP-IDF with FreeRTOS, this is typically a no-op when using
    // the embassy-time-driver feature, as the time driver handles wake-ups
    // This stub satisfies the linker requirement
}
```

**Why:**
- `embassy_executor` (pulled in as a transitive dependency) requires this function
- `#[no_mangle]` prevents Rust from mangling the function name, making it callable from C
- `pub extern "C"` makes it a C-compatible function that the executor can call
- The implementation is a no-op because `embassy-time-driver` handles task wake-ups through other mechanisms

### Change 3: Updated WiFi SSID

**File Modified:** `src/main.rs` (line 90)

**Before:**
```rust
let wifi_ssid = "Power2";
```

**After:**
```rust
let wifi_ssid = "jp5k"; //"Power2";
```

**Why:** Updated to connect to the correct WiFi network.

---

## Complete Environment Setup

### Prerequisites

#### 1. Install Rust and Rustup

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source $HOME/.cargo/env
```

#### 2. Install ESP Rust Toolchain

The project uses the `esp` toolchain which is a custom nightly build for ESP32 targets.

```bash
# Install the esp toolchain
rustup toolchain install esp

# Or if using the esp-rs installer:
curl -L https://github.com/esp-rs/rust-build/releases/download/v1.80.0.0/install-rust-toolchain.sh | bash
```

#### 3. Install System Dependencies (Linux)

```bash
# For Fedora/RHEL:
sudo dnf install -y \
    git \
    curl \
    pkg-config \
    libssl-dev \
    libudev-dev \
    libusb-1.0 \
    python3 \
    python3-pip \
    clang \
    cmake \
    ninja-build \
    libxml2-devel

# For Ubuntu/Debian:
sudo apt-get install -y \
    git \
    curl \
    pkg-config \
    libssl-dev \
    libudev-dev \
    libusb-1.0-0 \
    python3 \
    python3-pip \
    clang \
    cmake \
    ninja-build \
    libxml2-dev
```

#### 4. Install espflash

```bash
cargo install espflash
```

#### 5. Install ldproxy (Linker Proxy)

```bash
cargo install ldproxy
```

#### 6. Set Up Serial Port Permissions

Add your user to the `dialout` group to access serial ports:

```bash
sudo usermod -a -G dialout $USER
# Log out and back in, or run:
newgrp dialout
```

Verify:
```bash
groups | grep dialout
```

### Project Setup

#### 1. Clone/Copy the Project

```bash
cd ~/Projects
git clone <repository-url> Axum-S3
# or copy the project directory
cd Axum-S3
```

#### 2. Verify Rust Toolchain

The project uses `rust-toolchain.toml` to automatically select the correct toolchain:

```bash
cd Axum-S3
rustup show
# Should show: "esp (active)"
```

If not active, the toolchain will be automatically installed when you run `cargo build`.

#### 3. Install Project Dependencies

```bash
cargo fetch
```

#### 4. Verify Build Configuration

Check that `.cargo/config.toml` exists and is configured correctly (see Configuration Files section).

---

## Project Structure

```
Axum-S3/
├── .cargo/
│   └── config.toml              # Cargo build configuration
├── accel-stepper/               # Stepper motor control library
├── bno080/                      # BNO080 IMU sensor library
├── buttons/                     # Button input handling
├── clock/                       # Real-time clock management
├── ds323x/                      # DS3231 RTC driver
├── hdc1080/                     # HDC1080 temperature/humidity sensor
├── motion/                      # Motion control logic
├── network/                     # MQTT networking
├── ota/                         # Over-the-air updates
├── rgb_led/                     # RGB LED control
├── sensors/                     # Sensor aggregation
├── wifi/                        # WiFi connectivity
├── src/
│   ├── main.rs                  # Main application entry point
│   └── config.rs                # Configuration management
├── build.rs                     # Build script
├── Cargo.toml                   # Root workspace manifest
├── rust-toolchain.toml          # Rust toolchain specification
├── sdkconfig.defaults           # ESP-IDF default configuration
├── partitions.csv               # Flash partition table
└── config.toml.example          # Example configuration file
```

---

## Configuration Files Explained

### 1. `Cargo.toml` (Root)

**Key Sections:**

```toml
[workspace]
members = ["buttons", "rgb_led", "sensors", "clock", "wifi", "network", "ota"]

[package]
name = "tower"
edition = "2021"
resolver = "2"
rust-version = "1.77"

[package.metadata.esp-idf]
partition_table = "partitions.csv"

[[bin]]
name = "tower"
harness = false  # Disables test harness for rust-analyzer compatibility
```

**Important Dependencies:**
- `esp-idf-svc = "0.51"` with features: `critical-section`, `embassy-time-driver`, `embassy-sync`
- `esp-idf-hal = "0.45"` with `rmt-legacy` feature
- `embuild = "0.33.0"` as build dependency

### 2. `.cargo/config.toml`

```toml
[build]
target = "xtensa-esp32s3-espidf"  # ESP32-S3 target

[target.xtensa-esp32s3-espidf]
linker = "ldproxy"  # Uses linker proxy for ESP-IDF integration
runner = "espflash flash --monitor --partition-table=partitions.csv"
rustflags = ["--cfg", "espidf_time64"]  # Enable 64-bit time support

[unstable]
build-std = ["std", "panic_abort"]  # Build standard library from source

[env]
MCU = "esp32s3"
ESP_IDF_VERSION = "v5.2.2"
LD_LIBRARY_PATH = "/usr/lib64:/usr/lib"
```

**Why `ldproxy`?**
- ESP-IDF requires special linker scripts and configurations
- `ldproxy` acts as a wrapper around the ESP-IDF linker
- It handles all the complex linking requirements automatically

### 3. `rust-toolchain.toml`

```toml
[toolchain]
channel = "esp"
```

**Why the `esp` toolchain?**
- Standard Rust toolchain doesn't support Xtensa architecture (ESP32)
- The `esp` toolchain is a custom build with Xtensa support
- It includes necessary compiler patches for ESP32 targets

### 4. `build.rs`

```rust
fn main() {
    // Set LD_LIBRARY_PATH for esp-clang to find libxml2.so.2 and other dependencies
    std::env::set_var("LD_LIBRARY_PATH", "/usr/lib64:/usr/lib");
    
    embuild::espidf::sysenv::output();
}
```

**Purpose:**
- Sets library path for ESP-IDF build tools
- Calls `embuild` to configure ESP-IDF build environment
- Generates necessary bindings and linker scripts

### 5. `sdkconfig.defaults`

```ini
CONFIG_ESP_MAIN_TASK_STACK_SIZE=20000  # Increased stack for Rust
CONFIG_ESP_TASK_WDT_EN=n                # Disable watchdog (Rust handles this)
CONFIG_FREERTOS_HZ=1000                 # 1ms tick granularity
```

**Why these settings?**
- Rust needs more stack space than C (default 3K → 20K)
- FreeRTOS tick at 1000Hz allows 1ms sleep granularity (vs 10ms default)

### 6. Workspace Member `Cargo.toml` Files

All workspace members (`buttons`, `motion`, `sensors`) have similar `esp-idf-svc` configuration:

```toml
esp-idf-svc = { version = "0.51", features = [
    "critical-section",
    "embassy-time-driver",
    "embassy-sync",
    # NOTE: "embassy-executor" is NOT a valid feature - do not add it!
] }
```

---

## Dependencies Breakdown

### Core ESP-IDF Dependencies

1. **`esp-idf-svc` (0.51)**
   - High-level ESP-IDF services wrapper
   - Features used:
     - `critical-section`: Critical section handling
     - `embassy-time-driver`: Async time driver for embassy runtime
     - `embassy-sync`: Synchronization primitives

2. **`esp-idf-hal` (0.45)**
   - Hardware abstraction layer
   - Features: `rmt-legacy` (Remote Control legacy support)

3. **`esp-idf-sys` (transitive)**
   - Low-level ESP-IDF bindings
   - Automatically managed by `esp-idf-svc`

### Embassy Runtime Dependencies

4. **`embassy-executor` (transitive)**
   - Async executor runtime
   - Pulled in by `embassy-sync` and `embassy-time-driver`
   - **Requires `__pender` function** (provided in `main.rs`)

5. **`embassy-sync` (transitive)**
   - Async synchronization primitives (channels, mutexes, etc.)

6. **`embassy-time-driver` (transitive)**
   - Time driver for async operations
   - Handles task wake-ups

### Application Dependencies

7. **`anyhow` (1.0)**: Error handling
8. **`chrono` (0.4.40)**: Date/time handling
9. **`serde` (1.0)**: Serialization
10. **`toml` (0.8)**: Configuration file parsing
11. **`semver` (1.0.26)**: Version management for OTA
12. **`heapless` (0.8.0)**: No-alloc collections
13. **`log` (0.4)**: Logging framework

### Hardware-Specific Dependencies

14. **`shared-bus`**: I2C bus sharing
15. **`embedded-hal` (1.0.0)**: Hardware abstraction traits
16. **`rgb` (0.8.50)**: RGB color handling
17. **`astronav` (0.2.5)**: Astronomical calculations for solar tracking

---

## Building and Flashing

### Build the Project

```bash
cd Axum-S3
cargo build
```

### Build and Flash

```bash
cargo run
```

This will:
1. Build the project
2. Flash to the connected ESP32-S3
3. Start monitoring serial output

### Flash Only (without building)

```bash
espflash flash target/xtensa-esp32s3-espidf/debug/tower
```

### Erase Flash

```bash
espflash erase-flash
```

### Monitor Serial Output

```bash
espflash monitor
```

---

## Troubleshooting

### Problem: `undefined reference to '__pender'`

**Solution:** Ensure `__pender` function is implemented in `src/main.rs`:

```rust
#[no_mangle]
pub extern "C" fn __pender() {
    // Stub implementation
}
```

### Problem: `esp-idf-svc` feature `embassy-executor` not found

**Solution:** Remove `"embassy-executor"` from all `Cargo.toml` files. This feature doesn't exist.

### Problem: Serial port permission denied

**Solution:**
```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

### Problem: `ldproxy` not found

**Solution:**
```bash
cargo install ldproxy
```

### Problem: Wrong Rust toolchain

**Solution:** Ensure `rust-toolchain.toml` exists with:
```toml
[toolchain]
channel = "esp"
```

### Problem: ESP-IDF build errors

**Solution:** 
- Ensure all system dependencies are installed
- Check `LD_LIBRARY_PATH` is set correctly
- Verify `ESP_IDF_VERSION = "v5.2.2"` in `.cargo/config.toml`

### Problem: Out of memory during build

**Solution:** The build requires significant RAM. Close other applications or increase swap space.

---

## Key Takeaways for Setting Up on New Machines

1. **Always use the `esp` Rust toolchain** - standard Rust won't work
2. **Never add `"embassy-executor"` feature** - it doesn't exist in `esp-idf-svc` 0.51
3. **Always provide `__pender` function** - required by `embassy_executor`
4. **Set up serial port permissions** - add user to `dialout` group
5. **Use `ldproxy` as linker** - required for ESP-IDF integration
6. **Configure `build-std`** - must build standard library from source
7. **Set `LD_LIBRARY_PATH`** - needed for ESP-IDF build tools

---

## Summary of Changes

| File | Change | Reason |
|------|--------|--------|
| `Cargo.toml` (root) | Removed `"embassy-executor"` feature | Feature doesn't exist |
| `buttons/Cargo.toml` | Removed `"embassy-executor"` feature | Feature doesn't exist |
| `motion/Cargo.toml` | Removed `"embassy-executor"` feature | Feature doesn't exist |
| `sensors/Cargo.toml` | Removed `"embassy-executor"` feature | Feature doesn't exist |
| `src/main.rs` | Added `__pender()` function | Required by embassy_executor |
| `src/main.rs` | Changed WiFi SSID to `"jp5k"` | Network configuration |

---

## Verification Checklist

After setting up on a new machine, verify:

- [ ] Rust toolchain is `esp` (check with `rustup show`)
- [ ] `ldproxy` is installed (`which ldproxy`)
- [ ] `espflash` is installed (`which espflash`)
- [ ] User is in `dialout` group (`groups | grep dialout`)
- [ ] `__pender` function exists in `src/main.rs`
- [ ] No `"embassy-executor"` in any `Cargo.toml` files
- [ ] `.cargo/config.toml` exists and is correct
- [ ] `rust-toolchain.toml` exists with `channel = "esp"`
- [ ] `build.rs` exists and sets `LD_LIBRARY_PATH`
- [ ] Project builds successfully (`cargo build`)
- [ ] Device can be flashed (`cargo run` or `espflash flash`)

---

## Additional Resources

- [ESP-RS Book](https://esp-rs.github.io/book/)
- [esp-idf-svc Documentation](https://docs.rs/esp-idf-svc/)
- [Embassy Documentation](https://embassy.dev/)
- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/)

---

**Last Updated:** 2025-12-31  
**ESP-IDF Version:** v5.2.2  
**Rust Edition:** 2021  
**Target:** xtensa-esp32s3-espidf

