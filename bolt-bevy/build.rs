fn main() {
    // Link the C++ standard library required by Jolt.
    // macOS uses libc++, while Linux typically uses libstdc++.
    #[cfg(target_os = "macos")]
    println!("cargo:rustc-link-lib=dylib=c++");

    #[cfg(not(target_os = "macos"))]
    println!("cargo:rustc-link-lib=dylib=stdc++");
}
