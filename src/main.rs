mod utils;

use crate::utils::convex_collider;
use avian2d::PhysicsPlugins;
use avian2d::prelude::{
    ColliderOf, ExternalForce, ExternalImpulse, Gravity, LinearVelocity, Mass, NarrowPhaseSet,
    PhysicsDebugPlugin, PhysicsInterpolationPlugin, PhysicsSchedule, Position, RigidBody, Rotation,
};
use avian2d::prepare::PrepareSet;
use bevy::prelude::*;
use bevy_inspector_egui::bevy_egui::EguiPlugin;
use bevy_inspector_egui::quick::WorldInspectorPlugin;

fn main() {
    let mut app = App::new();
    app.add_plugins(DefaultPlugins);
    app.add_plugins(EguiPlugin {
        enable_multipass_for_primary_context: true,
    });
    app.add_plugins(WorldInspectorPlugin::new());
    app.add_plugins(PhysicsPlugins::default().set(PhysicsInterpolationPlugin::extrapolate_all()));
    app.add_plugins(PhysicsDebugPlugin::default());
    app.insert_resource(Gravity::ZERO);
    app.add_systems(Startup, setup);
    app.add_systems(
        PhysicsSchedule,
        update_transforms
            .after(PrepareSet::PropagateTransforms)
            .before(PrepareSet::InitTransforms)
            .ambiguous_with_all(),
    );
    app.run();
}

#[derive(Component)]
struct Offset(Vec3);

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    commands.spawn(Camera2d);

    let default_material = ColorMaterial::from_color(Color::WHITE);
    let material_handle = materials.add(default_material);

    let ball_mesh = Circle::new(50.0).mesh().build();
    let ball_collider = convex_collider(&ball_mesh).expect("Failed to create ball collider");
    let ball_handle = meshes.add(ball_mesh);

    for (name, translation, velocity) in [
        ("Right", Vec3::X * 100.0, Vec2::X * 10.0),
        ("Left", Vec3::X * 300.0, Vec2::NEG_X * 20.0),
        ("Up", Vec3::Y * 100.0, Vec2::Y * 30.0),
    ] {
        commands.spawn((
            Name::new(name),
            Mesh2d(ball_handle.clone()),
            MeshMaterial2d(material_handle.clone()),
            Transform::from_translation(translation),
            ball_collider.clone(),
            RigidBody::Dynamic,
            ExternalImpulse::new(velocity).with_persistence(true),
            Mass(75.0),
        ));
    }

    let box_mesh = Rectangle::new(50.0, 50.0).mesh().build();
    let box_collider = convex_collider(&box_mesh).expect("Failed to create box collider");
    let box_handle = meshes.add(box_mesh);

    const DISTANCE: f32 = 200.0;
    let target = commands
        .spawn((
            Mesh2d(box_handle.clone()),
            MeshMaterial2d(material_handle.clone()),
            Transform::from_translation(Vec3::X * DISTANCE),
            box_collider.clone(),
            RigidBody::Dynamic,
            Mass(100.0),
        ))
        .id();

    commands.spawn((
        ColliderOf { body: target },
        Offset(Vec3::Y * DISTANCE + Vec3::NEG_X * DISTANCE),
        Mesh2d(box_handle.clone()),
        MeshMaterial2d(material_handle.clone()),
        Transform::from_translation(Vec3::Y * DISTANCE),
        box_collider.clone(),
        Mass(100.0),
    ));
}

fn update_transforms(
    mut query: Query<
        (
            &ColliderOf,
            &Offset,
            &mut Transform,
            &mut Position,
            &mut Rotation,
        ),
        With<Offset>,
    >,
    target_transforms: Query<&Transform, Without<Offset>>,
    target_positions: Query<&Position, Without<Offset>>,
    target_rotations: Query<&Rotation, Without<Offset>>,
) {
    for (&ColliderOf { body }, Offset(offset), mut transform, mut position, mut rotation) in
        query.iter_mut()
    {
        let target_transform = target_transforms
            .get(body)
            .expect("Failed to get target transform");
        let target_position = target_positions
            .get(body)
            .expect("Failed to get target position");
        let target_rotation = target_rotations
            .get(body)
            .expect("Failed to get target rotation");

        *transform = target_transform.with_translation(target_transform.translation + offset);
        position.0 = target_position.0 + offset.truncate();
        *rotation = *target_rotation;
    }
}
