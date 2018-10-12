echo "Updating rust..."
rustup update

echo "Adding SDL..."
copy C:\SDL2-2.0.8\lib\x64\*.lib %USERPROFILE%\.multirust\toolchains\%RUST_CHANNEL%-%RUST_TARGET%\lib\rustlib\%RUST_TARGET%\lib
copy C:\SDL2-2.0.8\lib\x64\*.dll .

echo "Running tests..."
$env:RUSTFLAGS = "-D warnings"
cargo +%RUST_CHANNEL% test --all
cargo +%RUST_CHANNEL% test --all --features profiler
cargo +%RUST_CHANNEL% test --all --features sdl_controller
