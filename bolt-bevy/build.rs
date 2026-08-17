fn main() {
    // macOS uses libc++
    #[cfg(target_os = "macos")]
    println!("cargo:rustc-link-lib=dylib=c++");

    // Linux and Windows (GNU toolchain) use libstdc++
    #[cfg(all(unix, not(target_os = "macos")))]
    println!("cargo:rustc-link-lib=dylib=stdc++");

    #[cfg(all(target_os = "windows", target_env = "gnu"))]
    println!("cargo:rustc-link-lib=dylib=stdc++");

    // For Windows MSVC, the C++ standard library is linked automatically
    // by the Visual Studio toolchain, so we don't need to specify anything.
}
