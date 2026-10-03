use bevy::diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin};
use bevy::prelude::*;
use bolt_bevy::prelude::*;

pub struct ExampleStatsPlugin;

impl Plugin for ExampleStatsPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(FrameTimeDiagnosticsPlugin::default())
            .add_systems(Startup, setup_stats_ui)
            .add_systems(Update, update_stats);
    }
}

#[derive(Component)]
struct StatsText;

fn setup_stats_ui(mut commands: Commands) {
    commands.spawn((
        Text::new("FPS: --\nBodies: --"),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(10.0),
            left: Val::Px(10.0),
            ..default()
        },
        StatsText,
    ));
}

fn update_stats(
    diagnostics: Res<DiagnosticsStore>,
    body_query: Query<(), With<JoltBody>>,
    mut text_query: Query<&mut Text, With<StatsText>>,
) {
    let mut fps = 0.0;
    if let Some(fps_diag) = diagnostics.get(&FrameTimeDiagnosticsPlugin::FPS)
        && let Some(value) = fps_diag.smoothed()
    {
        fps = value;
    }

    let body_count = body_query.iter().count();

    for mut text in text_query.iter_mut() {
        text.0 = format!("FPS: {:.1}\nPhysics Bodies: {}", fps, body_count);
    }
}
