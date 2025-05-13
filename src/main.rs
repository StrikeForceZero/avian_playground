mod utils;

use crate::utils::convex_collider;
use avian2d::PhysicsPlugins;
use avian2d::prelude::{
    Collider, ColliderOf, ExternalForce, ExternalImpulse, Gravity, LinearVelocity, Mass,
    NarrowPhaseSet, PhysicsDebugPlugin, PhysicsInterpolationPlugin, PhysicsSchedule, Position,
    RigidBody, Rotation,
};
use avian2d::prepare::PrepareSet;
use bevy::ecs::query::QueryData;
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
    app.register_type::<RemoteRigidBodyOffsets>();
    app.register_type::<RemoteInstances>();
    app.register_type::<RemoteInstance>();
    app.register_type::<Offset>();
    app.add_systems(Startup, setup);
    app.add_observer(on_remote_rigid_body_offsets_added);
    app.add_systems(
        PhysicsSchedule,
        update_transforms
            .after(PrepareSet::PropagateTransforms)
            .before(PrepareSet::InitTransforms)
            .ambiguous_with_all(),
    );
    app.run();
}

#[derive(Component, Debug, Default, Clone, PartialEq, Reflect)]
#[require(RigidBody::Dynamic)]
#[require(RemoteInstances)]
#[reflect(Component)]
struct RemoteRigidBodyOffsets(Vec<Vec3>);

#[derive(Component, Debug, Default, Copy, Clone, PartialEq, Reflect)]
#[reflect(Component)]
struct Offset(Vec3);

#[derive(Component, Debug, Default, Clone, Reflect)]
#[reflect(Component)]
#[relationship_target(relationship = RemoteInstance, linked_spawn)]
pub struct RemoteInstances(Vec<Entity>);

#[derive(Component, Debug, Copy, Clone, Reflect)]
#[reflect(Component)]
#[require(Offset)]
#[relationship(relationship_target = RemoteInstances)]
pub struct RemoteInstance {
    #[relationship]
    target: Entity,
}

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
    let collider = convex_collider(&box_mesh).expect("Failed to create box collider");
    let box_handle = meshes.add(box_mesh);

    const DISTANCE: f32 = 200.0;
    commands.spawn((
        Name::new("Main Box"),
        RemoteRigidBodyOffsets(vec![Vec3::Y * DISTANCE + Vec3::NEG_X * DISTANCE]),
        // collider,
        Mesh2d(box_handle.clone()),
        MeshMaterial2d(material_handle.clone()),
        Transform::from_translation(Vec3::X * DISTANCE),
        Mass(100.0),
        children![(
            Name::new("debug child"),
            // Mesh2d(box_handle.clone()),
            // MeshMaterial2d(material_handle.clone()),
            // Transform::from_translation(Vec3::Z * 10.0).with_scale(Vec2::splat(0.5).extend(0.0)),
        )],
    ));
}

#[derive(QueryData)]
struct RemoteRigidBodyOffsetsAddedQueryData {
    entity: Entity,
    transform: &'static Transform,
    offsets: &'static RemoteRigidBodyOffsets,
    collider: Option<&'static Collider>,
    mass: Option<&'static Mass>,
    mesh2d: Option<&'static Mesh2d>,
    mesh_material2d: Option<&'static MeshMaterial2d<ColorMaterial>>,
}

fn on_remote_rigid_body_offsets_added(
    trigger: Trigger<OnAdd, RemoteRigidBodyOffsets>,
    mut commands: Commands,
    target_q: Query<RemoteRigidBodyOffsetsAddedQueryData, Added<RemoteRigidBodyOffsets>>,
    meshes: Res<Assets<Mesh>>,
) {
    info!("Trigger<OnAdd, RemoteRigidBodyOffsets>");
    let target_entity = trigger.target();
    let target = target_q.get(target_entity).expect("Failed to get target");
    let (collider, mesh2d_opt) = match (target.collider, target.mesh2d) {
        (Some(collider), Some(mesh2d)) => {
            info!("Using existing collider and mesh2d for remote rigid body");
            (collider.clone(), Some(mesh2d.clone()))
        }
        (Some(collider), None) => {
            info!("No mesh2d found for remote rigid body (invisible), using existing collider");
            (collider.clone(), None)
        }
        (None, Some(mesh2d)) => {
            info!("No collider found for remote rigid body, creating one");
            let mesh = meshes.get(mesh2d.id()).expect("Failed to get mesh");
            let collider = convex_collider(mesh).expect("Failed to create collider");
            (collider, Some(mesh2d.clone()))
        }
        (None, None) => {
            error!("No collider or mesh2d found for remote rigid body");
            return;
        }
    };
    let mut entity_cmds = commands.entity(target_entity);
    if target.collider.is_none() {
        info!("Adding collider for Main box {target_entity}");
        entity_cmds.insert(collider.clone());
    }
    if target.mesh2d.is_none() {
        if let Some(mesh2d) = &mesh2d_opt {
            info!("Adding mesh for Main box {target_entity}");
            entity_cmds.insert(mesh2d.clone());
        }
    }
    if target.offsets.0.is_empty() {
        warn!("No offsets found for remote rigid body {target_entity}");
        return;
    }
    for &offset in target.offsets.0.iter() {
        info!("Adding offset: {offset} for {target_entity}");
        let mut entity_cmds = commands.spawn((
            Name::new("Main Box (Remote)"),
            ColliderOf {
                body: target_entity,
            },
            collider.clone(),
            RemoteInstance {
                target: target_entity,
            },
            Offset(offset),
            target
                .transform
                .with_translation(target.transform.translation + offset),
        ));
        let offset_entity = entity_cmds.id();
        if let Some(mesh2d) = &mesh2d_opt {
            info!("Offset {offset_entity} using mesh from {target_entity}");
            entity_cmds.insert(mesh2d.clone());
        } else {
            warn!("No mesh2d found for remote rigid body (invisible)");
        }
        if let Some(material2d) = target.mesh_material2d {
            info!("Offset {offset_entity} using mesh material from {target_entity}");
            entity_cmds.insert(material2d.clone());
        } else {
            warn!("No mesh material found for remote rigid body (invisible)");
        }
        if let Some(mass) = target.mass {
            info!("Offset {offset_entity} using mass from {target_entity}");
            entity_cmds.insert(mass.clone());
        } else {
            warn!("No mass found for remote rigid body");
        }
    }
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
    if query.is_empty() {
        warn_once!("No offsets to update");
    }
    for (&ColliderOf { body }, Offset(offset), mut transform, mut position, mut rotation) in
        query.iter_mut()
    {
        info_once!("Updating transform for {body} Remote");
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
