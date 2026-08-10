fn main() {
    // Link the C++ standard library required by Jolt
    println!("cargo:rustc-link-lib=dylib=stdc++");
}
