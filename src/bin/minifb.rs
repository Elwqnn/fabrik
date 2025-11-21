use fabrik::{Chain, ChainConfig, Vec2};
use minifb::{Key, MouseMode, Window, WindowOptions};

const WIDTH: usize = 1024;
const HEIGHT: usize = 768;

fn main() {
    let mut buffer: Vec<u32> = vec![0; WIDTH * HEIGHT];

    let mut window = Window::new(
        "FABRIK IK - minifb",
        WIDTH,
        HEIGHT,
        WindowOptions::default(),
    )
    .expect("Failed to create window");

    // ~60fps
    window.set_target_fps(60);

    let mut config = ChainConfig::default();
    let origin = Vec2::new(WIDTH as f32 / 2.0, HEIGHT as f32 * 0.75);
    let mut chain = Chain::new(origin, &config);
    let mut target = Vec2::new(WIDTH as f32 / 2.0, HEIGHT as f32 / 2.0);

    while window.is_open() && !window.is_key_down(Key::Escape) {
        // Handle input
        if window.is_key_pressed(Key::Up, minifb::KeyRepeat::Yes) {
            config.segment_count += 1;
            chain.rebuild(&config);
        }
        if window.is_key_pressed(Key::Down, minifb::KeyRepeat::Yes) && config.segment_count > 1 {
            config.segment_count -= 1;
            chain.rebuild(&config);
        }
        if window.is_key_pressed(Key::Right, minifb::KeyRepeat::Yes) {
            config.segment_length += 5.0;
            chain.rebuild(&config);
        }
        if window.is_key_pressed(Key::Left, minifb::KeyRepeat::Yes) && config.segment_length > 10.0
        {
            config.segment_length -= 5.0;
            chain.rebuild(&config);
        }
        if window.is_key_pressed(Key::R, minifb::KeyRepeat::No) {
            config = ChainConfig::default();
            chain.rebuild(&config);
        }

        // Update target from mouse
        if let Some((mx, my)) = window.get_mouse_pos(MouseMode::Clamp) {
            target = Vec2::new(mx, my);
        }

        // Solve IK
        chain.solve(target);

        // Clear buffer
        buffer.fill(rgb(25, 25, 38));

        // Draw target
        draw_circle(&mut buffer, target.x as i32, target.y as i32, 10, rgb(255, 75, 75));

        // Draw chain
        let joints = &chain.joints;
        let n = joints.len();

        // Segments
        for i in 0..n - 1 {
            let t = i as f32 / (n - 1) as f32;
            let color = rgb(
                (50.0 + 150.0 * t) as u8,
                (150.0 - 75.0 * t) as u8,
                (230.0 - 130.0 * t) as u8,
            );
            draw_line(
                &mut buffer,
                joints[i].x as i32,
                joints[i].y as i32,
                joints[i + 1].x as i32,
                joints[i + 1].y as i32,
                color,
            );
        }

        // Joints
        for (i, joint) in joints.iter().enumerate() {
            let t = i as f32 / (n - 1) as f32;
            let color = rgb(
                (75.0 + 180.0 * t) as u8,
                (180.0 - 100.0 * t) as u8,
                (255.0 - 150.0 * t) as u8,
            );
            draw_circle(&mut buffer, joint.x as i32, joint.y as i32, 6, color);
        }

        window.update_with_buffer(&buffer, WIDTH, HEIGHT).unwrap();
    }
}

#[inline]
fn rgb(r: u8, g: u8, b: u8) -> u32 {
    ((r as u32) << 16) | ((g as u32) << 8) | (b as u32)
}

fn set_pixel(buffer: &mut [u32], x: i32, y: i32, color: u32) {
    if x >= 0 && x < WIDTH as i32 && y >= 0 && y < HEIGHT as i32 {
        buffer[y as usize * WIDTH + x as usize] = color;
    }
}

fn draw_circle(buffer: &mut [u32], cx: i32, cy: i32, radius: i32, color: u32) {
    for dy in -radius..=radius {
        for dx in -radius..=radius {
            if dx * dx + dy * dy <= radius * radius {
                set_pixel(buffer, cx + dx, cy + dy, color);
            }
        }
    }
}

fn draw_line(buffer: &mut [u32], x0: i32, y0: i32, x1: i32, y1: i32, color: u32) {
    let dx = (x1 - x0).abs();
    let dy = -(y1 - y0).abs();
    let sx = if x0 < x1 { 1 } else { -1 };
    let sy = if y0 < y1 { 1 } else { -1 };
    let mut err = dx + dy;
    let mut x = x0;
    let mut y = y0;

    loop {
        // Draw thick line (3px)
        for ty in -1..=1 {
            for tx in -1..=1 {
                set_pixel(buffer, x + tx, y + ty, color);
            }
        }

        if x == x1 && y == y1 {
            break;
        }

        let e2 = 2 * err;
        if e2 >= dy {
            err += dy;
            x += sx;
        }
        if e2 <= dx {
            err += dx;
            y += sy;
        }
    }
}
