# Bolt ⚡

> ⚠️ **Work in Progress (WIP)**: This project is under active development. APIs are experimental and subject to breaking changes.

High-performance [Jolt Physics](https://github.com/jrouwe/JoltPhysics) integration for the [Bevy](https://bevyengine.org) game engine.

---

## 📁 Workspace Structure

- `bolt-core`: Engine-agnostic core physics abstractions and Jolt wrapper types.
- `bolt-bevy`: Bevy ECS plugin, components, resources, and synchronization systems.
- `bolt-examples`: Interactive and visual example applications.

---

## 🧪 Testing & Development Guide

Use the following commands to run tests, linting, and verification across the workspace.

### 1. Running Tests

#### Run all tests across the entire workspace
```bash
cargo test --workspace
```

#### Run unit tests for `bolt-bevy` only
```bash
cargo test -p bolt-bevy --lib
```

#### Run the integration test suite (`tests/physics_loop.rs`)
```bash
cargo test -p bolt-bevy --test physics_loop
```

#### Run a single test by name
```bash
cargo test -p bolt-bevy test_get_transform_invalid_and_destroyed_body
```

#### Run tests with full log/console output (`--nocapture`)
Useful for viewing `info!`, `error!`, and `warn!` tracing output:
```bash
cargo test -p bolt-bevy -- --nocapture
```

---

### 2. Code Quality & Linting

#### Run Clippy across all targets and features
```bash
cargo clippy --workspace --all-targets --all-features
```

#### Check code formatting
```bash
cargo fmt --all -- --check
```

#### Format the entire codebase
```bash
cargo fmt --all
```

---

### 3. Running Examples

#### Run the 3D Hello World physics simulation
```bash
cargo run --bin hello_world -p bolt-examples
```

---

## 🛠️ Prerequisites

- **Rust Toolchain**: 2024 edition compatible (`nightly` or stable $\ge$ 1.85).
- **C++ Compiler / Standard Library**:
  - **Linux**: `g++` or `clang++` with `libstdc++`
  - **macOS**: Xcode Command Line Tools (`libc++`)
  - **Windows**: MSVC build tools or MinGW (`libstdc++`)
