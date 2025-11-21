use std::io::{self, stdout};
use std::time::Duration;

use crossterm::{
    event::{self, DisableMouseCapture, EnableMouseCapture, Event, KeyCode, MouseEventKind},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};
use fabrik::{Chain, ChainConfig, Vec2};
use ratatui::{
    prelude::*,
    widgets::{
        canvas::{Canvas, Line, Circle},
        Block, Paragraph,
    },
};

fn main() -> io::Result<()> {
    enable_raw_mode()?;
    execute!(stdout(), EnterAlternateScreen, EnableMouseCapture)?;

    let backend = CrosstermBackend::new(stdout());
    let mut terminal = Terminal::new(backend)?;

    let result = run(&mut terminal);

    disable_raw_mode()?;
    execute!(stdout(), LeaveAlternateScreen, DisableMouseCapture)?;

    result
}

fn run(terminal: &mut Terminal<CrosstermBackend<io::Stdout>>) -> io::Result<()> {
    let mut config = ChainConfig {
        segment_length: 8.0,
        ..Default::default()
    };

    let mut canvas_bounds;
    let mut origin = Vec2::new(50.0, 10.0);
    let mut chain = Chain::new(origin, &config);
    let mut target = Vec2::new(50.0, 30.0);
    let mut needs_rebuild = false;

    loop {
        // Get terminal size to compute canvas bounds
        let size = terminal.size()?;
        canvas_bounds = (0.0, 0.0, size.width as f64, size.height as f64 - 3.0);
        origin = Vec2::new(canvas_bounds.2 as f32 / 2.0, 5.0);

        if needs_rebuild {
            chain = Chain::new(origin, &config);
            needs_rebuild = false;
        } else {
            chain.set_origin(origin);
        }

        chain.solve(target);

        terminal.draw(|f| {
            let chunks = Layout::default()
                .direction(Direction::Vertical)
                .constraints([Constraint::Min(1), Constraint::Length(3)])
                .split(f.area());

            // Canvas
            let canvas = Canvas::default()
                .block(Block::bordered().title(" FABRIK IK - TUI "))
                .x_bounds([canvas_bounds.0, canvas_bounds.2])
                .y_bounds([canvas_bounds.1, canvas_bounds.3])
                .paint(|ctx| {
                    // Draw target
                    ctx.draw(&Circle {
                        x: target.x as f64,
                        y: target.y as f64,
                        radius: 1.5,
                        color: Color::Red,
                    });

                    // Draw segments
                    let joints = &chain.joints;
                    for i in 0..joints.len() - 1 {
                        let t = i as f32 / (joints.len() - 1) as f32;
                        let color = Color::Rgb(
                            (50.0 + 150.0 * t) as u8,
                            (150.0 - 75.0 * t) as u8,
                            (230.0 - 130.0 * t) as u8,
                        );
                        ctx.draw(&Line {
                            x1: joints[i].x as f64,
                            y1: joints[i].y as f64,
                            x2: joints[i + 1].x as f64,
                            y2: joints[i + 1].y as f64,
                            color,
                        });
                    }

                    // Draw joints
                    for (i, joint) in joints.iter().enumerate() {
                        let t = i as f32 / (joints.len() - 1) as f32;
                        let color = Color::Rgb(
                            (75.0 + 180.0 * t) as u8,
                            (180.0 - 100.0 * t) as u8,
                            (255.0 - 150.0 * t) as u8,
                        );
                        ctx.draw(&Circle {
                            x: joint.x as f64,
                            y: joint.y as f64,
                            radius: 0.8,
                            color,
                        });
                    }
                });

            f.render_widget(canvas, chunks[0]);

            // Status bar
            let status = Paragraph::new(format!(
                " Segments: {} (↑/↓)  Length: {:.0} (←/→)  [R] Reset  [Q] Quit  |  Move mouse to control target",
                config.segment_count, config.segment_length
            ))
            .style(Style::default().fg(Color::Gray))
            .block(Block::bordered());

            f.render_widget(status, chunks[1]);
        })?;

        // Poll events
        if event::poll(Duration::from_millis(16))? {
            match event::read()? {
                Event::Key(key) => match key.code {
                    KeyCode::Char('q') | KeyCode::Esc => break,
                    KeyCode::Up => {
                        config.segment_count += 1;
                        needs_rebuild = true;
                    }
                    KeyCode::Down if config.segment_count > 1 => {
                        config.segment_count -= 1;
                        needs_rebuild = true;
                    }
                    KeyCode::Right => {
                        config.segment_length += 1.0;
                        needs_rebuild = true;
                    }
                    KeyCode::Left if config.segment_length > 2.0 => {
                        config.segment_length -= 1.0;
                        needs_rebuild = true;
                    }
                    KeyCode::Char('r') => {
                        config = ChainConfig {
                            segment_length: 8.0,
                            ..Default::default()
                        };
                        needs_rebuild = true;
                    }
                    _ => {}
                },
                Event::Mouse(mouse) => {
                    if matches!(mouse.kind, MouseEventKind::Moved | MouseEventKind::Drag(_)) {
                        // Convert terminal coords to canvas coords (y is flipped)
                        let term_size = terminal.size()?;
                        let canvas_height = term_size.height.saturating_sub(5) as f32;
                        target = Vec2::new(
                            mouse.column as f32,
                            canvas_height - mouse.row as f32 + 3.0,
                        );
                    }
                }
                Event::Resize(_, _) => {
                    needs_rebuild = true;
                }
                _ => {}
            }
        }
    }

    Ok(())
}
