use bevy::prelude::*;
use bolt_bevy::prelude::*;

fn main() {
    App::new()
        .add_plugins(MinimalPlugins)
        .add_plugins(BoltPlugin::default())
        .add_systems(Startup, bolt_init)
        .add_systems(Update, print_delta_time)
        .run();
}

fn bolt_init() {
    println!("Bolt Physics Plugin initialized!");
}

fn print_delta_time(time: Res<Time>) {
    println!("Delta time: {}", time.delta_secs())
}
