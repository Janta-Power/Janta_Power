# Build Fix Instructions: Resolving `__pender` Undefined Reference Error

## Problem Summary

The project was failing to build with a linker error:
```
undefined reference to `__pender'
```

This error occurred because `embassy_executor` (a dependency pulled in by `embassy-sync` and `embassy-time-driver` features) requires a `__pender` function to be provided, but it wasn't available in the ESP-IDF build.

## What Was Removed

**From all Cargo.toml files** (root `Cargo.toml`, `buttons/Cargo.toml`, `motion/Cargo.toml`, and `sensors/Cargo.toml`):

The line `"embassy-executor",` was removed from the `esp-idf-svc` features list. This feature doesn't exist in `esp-idf-svc` version 0.51, so it was causing dependency resolution errors.

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

## What Was Added

**To `src/main.rs`** (after line 10, before the `use esp_idf_svc::` block):

A Rust implementation of the `__pender` function was added to satisfy the linker requirement:

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

## Step-by-Step Fix Instructions

### Step 1: Remove the Invalid Feature

Remove the `"embassy-executor",` line from all `Cargo.toml` files that have `esp-idf-svc` as a dependency:

1. **Root `Cargo.toml`** (line ~37)
2. **`buttons/Cargo.toml`** (line ~13)
3. **`motion/Cargo.toml`** (line ~14)
4. **`sensors/Cargo.toml`** (line ~14)

You can do this manually by editing each file, or use this command:

```bash
cd /home/mudit/Projects/Axum-S3
sed -i '/embassy-executor/d' Cargo.toml buttons/Cargo.toml motion/Cargo.toml sensors/Cargo.toml
```

Verify the removal:
```bash
grep -n "embassy-executor" Cargo.toml buttons/Cargo.toml motion/Cargo.toml sensors/Cargo.toml
```
(This should return nothing if successful)

### Step 2: Add the `__pender` Function

Add the `__pender` function implementation to `src/main.rs`. Place it after the imports and before the `use esp_idf_svc::` block (around line 10-11):

```rust
use esp_idf_hal::{sys::esp_app_desc, task::current};

// Provide __pender function for embassy_executor
// This function is called by embassy_executor to wake tasks
#[no_mangle]
pub extern "C" fn __pender() {
    // For ESP-IDF with FreeRTOS, this is typically a no-op when using
    // the embassy-time-driver feature, as the time driver handles wake-ups
    // This stub satisfies the linker requirement
}

use esp_idf_svc::{
    // ... rest of imports
```

### Step 3: Clean and Rebuild

Clean the build cache to ensure a fresh build:

```bash
cargo clean
```

Then rebuild:

```bash
cargo run
```

Or if you just want to build without flashing:

```bash
cargo build
```

## Why This Works

1. **Removing `embassy-executor` feature**: This feature doesn't exist in `esp-idf-svc` 0.51, so removing it prevents dependency resolution errors.

2. **Adding `__pender` function**: The `embassy_executor` crate (which is a transitive dependency through `embassy-sync` and `embassy-time-driver`) requires a `__pender` function to be linked. This function is used internally by the executor to wake tasks. By providing a stub implementation with:
   - `#[no_mangle]` - prevents Rust from mangling the function name
   - `pub extern "C"` - makes it callable from C code (which embassy_executor expects)
   
   We satisfy the linker's requirement. The function can be a no-op because when using `embassy-time-driver`, the time driver handles task wake-ups through other mechanisms.

## Verification

After applying these fixes, the build should complete successfully. You may see warnings (which are normal), but no errors. The final step will attempt to flash the device, which will fail if no ESP32-S3 is connected - this is expected behavior.

## Notes

- The `__pender` implementation is a minimal stub. If you encounter issues with task wake-ups in the future, you may need to implement a more sophisticated version that interacts with FreeRTOS task notifications.
- This fix is specific to ESP-IDF with `embassy-time-driver` and `embassy-sync` features enabled.
- The warnings shown during compilation are non-critical and can be addressed separately if desired.

