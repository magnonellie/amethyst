echo "Updating rust..."
rustup update || exit /b 1

echo "Adding SDL..."
copy /Y C:\SDL2-2.0.8\lib\x64\*.lib %USERPROFILE%\.multirust\toolchains\%RUST_CHANNEL%-%RUST_TARGET%\lib\rustlib\%RUST_TARGET%\lib || exit /b 1
copy /Y C:\SDL2-2.0.8\lib\x64\*.dll || exit /b 1

echo "Running tests..."
set RUSTFLAGS=-D warnings || exit /b 1
cargo +%RUST_CHANNEL% test --all || exit /b 1
cargo +%RUST_CHANNEL% test --all --features profiler || exit /b 1
cargo +%RUST_CHANNEL% test --all --features sdl_controller || exit /b 1
