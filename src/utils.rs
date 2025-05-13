use avian2d::prelude::Collider;
use bevy::platform::collections::HashMap;
use bevy::prelude::*;
use bevy::render::mesh::Indices;

pub fn get_positions(mesh: &Mesh) -> Option<Vec<Vec2>> {
    Some(
        mesh.attribute(Mesh::ATTRIBUTE_POSITION)?
            .as_float3()?
            .into_iter()
            .map(|&[x, y, _]| Vec2::new(x, y))
            .collect::<Vec<_>>(),
    )
}

pub fn get_indices_as_u32(mesh: &Mesh) -> Option<Vec<u32>> {
    // Normalize into a Vec<u32>
    let indices: Vec<u32> = match mesh.indices()? {
        Indices::U16(v) => v.iter().map(|&i| i as u32).collect(),
        Indices::U32(v) => v.clone(),
    };

    Some(indices)
}

pub fn mesh_to_polyline_indices(mesh: &Mesh) -> Option<Vec<[u32; 2]>> {
    let indices = get_indices_as_u32(mesh)?;

    // Count each undirected edge
    let mut edge_count = HashMap::new();
    for tri in indices.chunks(3) {
        for &(a, b) in &[(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])] {
            let key = if a < b { (a, b) } else { (b, a) };
            *edge_count.entry(key).or_insert(0) += 1;
        }
    }

    // Keep only edges occurring exactly once (the “boundary”)
    let boundary: Vec<[u32; 2]> = edge_count
        .into_iter()
        .filter_map(
            |((a, b), count)| {
                if count == 1 { Some([a, b]) } else { None }
            },
        )
        .collect();

    if boundary.is_empty() {
        None
    } else {
        Some(boundary)
    }
}

pub fn convex_collider(mesh: &Mesh) -> Option<Collider> {
    let positions = get_positions(mesh)?;
    let indices = mesh_to_polyline_indices(mesh)?;
    Some(Collider::convex_decomposition(positions, indices))
}
