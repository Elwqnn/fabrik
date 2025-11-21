# fabrik

FABRIK (Forward And Backward Reaching Inverse Kinematics) implementation in Rust.

## Library

Zero-dependency IK solver in `src/lib.rs`:

```rust
use fabrik::{Chain, ChainConfig, Vec2};

let mut chain = Chain::new(Vec2::new(0.0, 0.0), &ChainConfig::default());
chain.solve(Vec2::new(100.0, 200.0));
```

## Visualizers

```bash
cargo run --release --bin fabrik-egui    # native GUI
cargo run --release --bin fabrik-minifb  # raw framebuffer
cargo run --release --bin fabrik-tui     # terminal
```

**Controls:** `↑/↓` segment count, `←/→` segment length, `R` reset, mouse for target.

## License

MIT
