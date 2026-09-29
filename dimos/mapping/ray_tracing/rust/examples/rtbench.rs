// Scratch bench: replay packed R1 clouds (sensor frame + odom pose) through Mapper with r1pro-nav-lio's config, timing each stage.
use dimos_voxel_ray_tracing::mapper::{Mapper, Pose};
use dimos_voxel_ray_tracing::voxel_ray_tracer::Config;
use std::io::Read;
use std::time::{Duration, Instant};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let input = &args[1];
    let threads: u32 = args.get(2).map_or(3, |s| s.parse().unwrap());
    let only: Option<usize> = std::env::var("RT_ONLY").ok().map(|v| v.parse().unwrap());
    let stereo_stride: usize = args.get(3).map_or(1, |s| s.parse().unwrap());
    let ray_subsample: u32 = args.get(4).map_or(10, |s| s.parse().unwrap());
    let config = Config {
        voxel_size: 0.05,
        fine_divisor: 0,
        emit_fine: false,
        max_range: 10.0,
        ray_subsample,
        shadow_depth: 0.1,
        grace_depth: 0.2,
        min_health: -1,
        max_health: 5,
        range_error_coeff: std::env::var("RT_COEFF")
            .map_or(1.1 / (253.0 * 0.12), |v| v.parse().unwrap()),
        range_error_exponent: 2.0,
        range_error_frame_ids: vec!["camera_head_left_link".into()],
        graze_cos: 0.7,
        support_min: 4,
        emit_every: 1,
        global_emit_every: 50,
        region_percentile: 95.0,
        world_frame: "odom".into(),
        tf_match_tolerance_s: 0.25,
        worker_threads: threads,
    };
    let mut mapper = Mapper::new(config);
    let mut buf = Vec::new();
    std::fs::File::open(input)
        .unwrap()
        .read_to_end(&mut buf)
        .unwrap();
    let f32_at = |i: usize| f32::from_le_bytes(buf[i..i + 4].try_into().unwrap());

    let mut add = [Duration::ZERO; 2];
    let mut add_count = [0usize; 2];
    let mut add_points = [0usize; 2];
    let (mut local, mut global) = (Duration::ZERO, Duration::ZERO);
    let (mut local_count, mut global_count, mut local_out) = (0usize, 0usize, 0usize);
    let mut stereo_seen = 0usize;
    let started = Instant::now();
    let mut i = 0;
    while i < buf.len() {
        let kind = buf[i] as usize;
        let n = u32::from_le_bytes(buf[i + 1..i + 5].try_into().unwrap()) as usize;
        let position = (f32_at(i + 5), f32_at(i + 9), f32_at(i + 13));
        let orientation = (
            f32_at(i + 17),
            f32_at(i + 21),
            f32_at(i + 25),
            f32_at(i + 29),
        );
        i += 33;
        let points: Vec<(f32, f32, f32)> = (0..n)
            .map(|k| {
                (
                    f32_at(i + k * 12),
                    f32_at(i + k * 12 + 4),
                    f32_at(i + k * 12 + 8),
                )
            })
            .filter(|p| p.0.is_finite())
            .collect();
        i += n * 12;
        if kind == 1 {
            stereo_seen += 1;
        }
        // Thin stereo like a coarser pixel decimation would: keep every Nth point.
        let points: Vec<_> = if kind == 1 && stereo_stride > 1 {
            points.into_iter().step_by(stereo_stride).collect()
        } else {
            points
        };
        if only.is_some_and(|k| k != kind) {
            continue;
        }
        let frame = if kind == 1 {
            "camera_head_left_link"
        } else {
            "lidar_pointlio_link"
        };
        let t = Instant::now();
        add_points[kind] += points.len();
        mapper.add_frame(
            points,
            Pose {
                position,
                orientation,
            },
            frame,
        );
        add[kind] += t.elapsed();
        add_count[kind] += 1;
        if mapper.local_due() {
            let t = Instant::now();
            let bounds = mapper.take_local_bounds().bounds();
            local_out += mapper.local_points(&bounds).len() / 3;
            local += t.elapsed();
            local_count += 1;
        }
        if mapper.global_due() {
            let t = Instant::now();
            let _ = mapper.global_points();
            global += t.elapsed();
            global_count += 1;
        }
    }
    if let Some(path) = args.get(5) {
        let bytes: Vec<u8> = mapper
            .global_points()
            .iter()
            .flat_map(|v| v.to_le_bytes())
            .collect();
        std::fs::write(path, bytes).unwrap();
    }
    let per = |d: Duration, n: usize| d.as_secs_f64() * 1e3 / n.max(1) as f64;
    let total = started.elapsed().as_secs_f64();
    let frames = add_count[0] + add_count[1];
    println!(
        "threads={threads} stereo_stride={stereo_stride} ray_subsample={ray_subsample} stereo_frames={stereo_seen}"
    );
    println!(
        "lidar  add_frame {:.2} ms avg over {} ({} pts avg)",
        per(add[0], add_count[0]),
        add_count[0],
        add_points[0] / add_count[0].max(1)
    );
    println!(
        "stereo add_frame {:.2} ms avg over {} ({} pts avg)",
        per(add[1], add_count[1]),
        add_count[1],
        add_points[1] / add_count[1].max(1)
    );
    println!(
        "local_points     {:.2} ms avg over {} ({} pts out avg)",
        per(local, local_count),
        local_count,
        local_out / local_count.max(1)
    );
    println!(
        "global_points    {:.2} ms avg over {}",
        per(global, global_count),
        global_count
    );
    println!(
        "total {:.1} s => {:.1} clouds/s sustained",
        total,
        frames as f64 / total
    );
}
