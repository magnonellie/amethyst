export RUSTFLAGS="-D warnings"

echo "Build and test"
cargo test --all || exit 1

if [ ${CHANNEL} = "stable" ]
then
    echo "Build and test with sdl_controller"
    cargo check --all --features sdl_controller,profiler

    if [ ${TRAVIS_OS_NAME} = "linux" ]
    then
        mdbook test book -L target/debug/deps
    fi
fi
