use eframe::egui::{self, Color32, Pos2, Stroke};
use fabrik::{Chain, ChainConfig, Vec2};

fn main() -> eframe::Result<()> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([1024.0, 768.0]),
        ..Default::default()
    };

    eframe::run_native(
        "FABRIK IK - egui",
        options,
        Box::new(|_cc| Ok(Box::new(App::new()))),
    )
}

struct App {
    config: ChainConfig,
    chain: Chain,
    target: Vec2,
}

impl App {
    fn new() -> Self {
        let config = ChainConfig::default();
        let origin = Vec2::new(512.0, 576.0);
        let chain = Chain::new(origin, &config);

        Self {
            config,
            chain,
            target: Vec2::new(512.0, 300.0),
        }
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ctx, |ui| {
            // Controls panel
            ui.horizontal(|ui| {
                ui.label("Segments:");
                if ui.button("-").clicked() && self.config.segment_count > 1 {
                    self.config.segment_count -= 1;
                    self.chain.rebuild(&self.config);
                }
                ui.label(format!("{}", self.config.segment_count));
                if ui.button("+").clicked() {
                    self.config.segment_count += 1;
                    self.chain.rebuild(&self.config);
                }

                ui.separator();

                ui.label("Length:");
                if ui.button("-").clicked() && self.config.segment_length > 10.0 {
                    self.config.segment_length -= 5.0;
                    self.chain.rebuild(&self.config);
                }
                ui.label(format!("{:.0}", self.config.segment_length));
                if ui.button("+").clicked() {
                    self.config.segment_length += 5.0;
                    self.chain.rebuild(&self.config);
                }

                ui.separator();

                if ui.button("Reset").clicked() {
                    self.config = ChainConfig::default();
                    self.chain.rebuild(&self.config);
                }
            });

            ui.separator();

            // Canvas area
            let available = ui.available_size();
            let (response, painter) =
                ui.allocate_painter(available, egui::Sense::hover());

            let rect = response.rect;

            // Update origin to center-bottom of canvas
            let origin = Vec2::new(rect.center().x, rect.bottom() - 100.0);
            self.chain.set_origin(origin);

            // Update target from mouse position
            if let Some(pos) = response.hover_pos() {
                self.target = Vec2::new(pos.x, pos.y);
            }

            // Solve IK
            self.chain.solve(self.target);

            // Draw background
            painter.rect_filled(rect, 0.0, Color32::from_rgb(25, 25, 38));

            // Draw target
            painter.circle_filled(
                Pos2::new(self.target.x, self.target.y),
                10.0,
                Color32::from_rgba_unmultiplied(255, 75, 75, 200),
            );
            painter.circle_stroke(
                Pos2::new(self.target.x, self.target.y),
                15.0,
                Stroke::new(2.0, Color32::from_rgba_unmultiplied(255, 125, 125, 125)),
            );

            // Draw chain
            let joints = &self.chain.joints;
            let n = joints.len();

            // Segments
            for i in 0..n - 1 {
                let t = i as f32 / (n - 1) as f32;
                let color = Color32::from_rgb(
                    (50.0 + 150.0 * t) as u8,
                    (150.0 - 75.0 * t) as u8,
                    (230.0 - 130.0 * t) as u8,
                );
                painter.line_segment(
                    [
                        Pos2::new(joints[i].x, joints[i].y),
                        Pos2::new(joints[i + 1].x, joints[i + 1].y),
                    ],
                    Stroke::new(3.0, color),
                );
            }

            // Joints
            for (i, joint) in joints.iter().enumerate() {
                let t = i as f32 / (n - 1) as f32;
                let color = Color32::from_rgb(
                    (75.0 + 180.0 * t) as u8,
                    (180.0 - 100.0 * t) as u8,
                    (255.0 - 150.0 * t) as u8,
                );
                painter.circle_filled(Pos2::new(joint.x, joint.y), 6.0, color);
            }
        });

        // Request continuous repaints for smooth updates
        ctx.request_repaint();
    }
}
