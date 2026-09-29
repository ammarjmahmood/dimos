// Scratch harness: replay packed lidar+stereo world clouds through Mapper, write the global map.
use dimos_voxel_ray_tracing::mapper::{Mapper, Pose};
use dimos_voxel_ray_tracing::voxel_ray_tracer::Config;
use std::io::{Read, Write};

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let (input, output) = (&a[1], &a[2]);
    let coeff: f32 = a[3].parse().unwrap();
    let exponent: f32 = a[4].parse().unwrap();
    let mode = a.get(5).map(|s| s.as_str()).unwrap_or("fused");
    let cfg = Config {
        voxel_size: 0.05,
        fine_divisor: 0,
        emit_fine: false,
        max_range: 10.0,
        ray_subsample: 10,
        shadow_depth: 0.1,
        grace_depth: 0.2,
        min_health: -1,
        max_health: 5,
        range_error_coeff: coeff,
        range_error_exponent: exponent,
        range_error_frame_ids: vec!["camera".into()],
        graze_cos: 0.7,
        support_min: 4,
        emit_every: 0,
        global_emit_every: 0,
        region_percentile: 95.0,
        world_frame: "odom".into(),
        tf_match_tolerance_s: 0.25,
        worker_threads: 8,
    };
    let mut mapper = Mapper::new(cfg);
    let mut buf = Vec::new();
    std::fs::File::open(input)
        .unwrap()
        .read_to_end(&mut buf)
        .unwrap();
    let f32at = |b: &[u8], i: usize| f32::from_le_bytes(b[i..i + 4].try_into().unwrap());
    let mut i = 0;
    while i < buf.len() {
        let kind = buf[i];
        let n = u32::from_le_bytes(buf[i + 1..i + 5].try_into().unwrap()) as usize;
        let o = (f32at(&buf, i + 5), f32at(&buf, i + 9), f32at(&buf, i + 13));
        i += 17;
        let pts: Vec<(f32, f32, f32)> = (0..n)
            .map(|k| {
                let j = i + k * 12;
                (
                    f32at(&buf, j) - o.0,
                    f32at(&buf, j + 4) - o.1,
                    f32at(&buf, j + 8) - o.2,
                )
            })
            .collect();
        i += n * 12;
        let use_it = match mode {
            "lidar" => kind == 0,
            "stereo" => kind == 1,
            _ => true,
        };
        if use_it {
            let frame = if kind == 1 { "camera" } else { "lidar" };
            mapper.add_frame(
                pts,
                Pose {
                    position: o,
                    orientation: (0.0, 0.0, 0.0, 1.0),
                },
                frame,
            );
        }
    }
    let g = mapper.global_points();
    let mut f = std::fs::File::create(output).unwrap();
    for v in &g {
        f.write_all(&v.to_le_bytes()).unwrap();
    }
    eprintln!("{} voxels", g.len() / 3);
}
