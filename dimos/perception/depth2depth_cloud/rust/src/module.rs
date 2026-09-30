// Copyright 2026 Dimensional Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! `depth2depth_cloud`: a camera-frame point cloud from one colour camera, Depth
//! Anything calibrated per pixel to the recent lidar scans (see the depth2depth crate).
//!
//! Images arrive faster than the model runs, so the handler only keeps the newest
//! frame and a worker thread processes whichever is newest when it is free. Lidar
//! scans are kept for `lidar_history_s` in the world frame, so the anchor set
//! includes floor the lidar saw a moment ago and the camera sees now.

use std::collections::VecDeque;
use std::sync::mpsc::{sync_channel, Receiver, SyncSender};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use depth2depth::{
    Calibration, CalibrationConfig, CloudOptions, Config as ModelConfig, Depth2Depth,
};
use dimos_module::pointcloud::extract_xyz;
use dimos_module::{native_config, warn_throttled, Input, Module, Output, Tf};
use lcm_msgs::sensor_msgs::{CameraInfo, CompressedImage, PointCloud2, PointField};
use lcm_msgs::std_msgs::{Header, Time};
use nalgebra::{Isometry3, Point3};
use tracing::info;

use crate::undistort::{Lens, UndistortMap};

const TIMING_REPORT_EVERY: Duration = Duration::from_secs(5);
const TF_WAIT: Duration = Duration::from_millis(50);
/// A frame can arrive before odometry has covered its stamp (Point-LIO publishes a scan's pose after the scan);
/// the worker is on its own thread, so it can wait this long for it.
const FRAME_TF_WAIT: Duration = Duration::from_millis(400);

#[native_config]
#[derive(Clone)]
pub struct Config {
    /// Directory holding `dinov2_vits14.safetensors` and `da2_head_vits.safetensors`.
    weights_dir: String,
    /// The model as an ONNX export (in `weights_dir`) for TensorRT on a Jetson, and where built engines are cached.
    onnx_file: String,
    engine_cache_dir: String,
    /// Model input size for candle (a Mac, or CPU); both multiples of 14, smaller is faster. TensorRT uses the ONNX's.
    #[validate(range(min = 56, max = 1036))]
    model_height: i64,
    #[validate(range(min = 56, max = 1036))]
    model_width: i64,
    /// JPEG decoded at 1/`decode_scale` of full size (1, 2, 4 or 8), before undistorting.
    #[validate(range(min = 1, max = 8))]
    decode_scale: i64,
    /// The pinhole image Depth Anything sees: size and focal length in pixels, centred.
    #[validate(range(min = 16, max = 4096))]
    undistorted_width: i64,
    #[validate(range(min = 16, max = 4096))]
    undistorted_height: i64,
    #[validate(range(min = 1.0, max = 10000.0))]
    undistorted_focal_px: f64,
    /// Frame the lidar history is kept in; must be fixed while the robot moves.
    world_frame: String,
    /// Seconds of lidar scans used as anchors.
    #[validate(range(min = 0.0, max = 30.0))]
    lidar_history_s: f64,
    /// Lidar points farther than this from the camera are not anchors.
    #[validate(range(min = 0.1, max = 200.0))]
    max_anchor_range_m: f64,
    /// Largest gap between a stamp and the transform used for it.
    #[validate(range(min = 0.0, max = 5.0))]
    tf_tolerance_s: f64,
    /// Calibration, see `depth2depth::CalibrationConfig`.
    #[validate(range(min = 1.0, max = 1000.0))]
    sigma_px: f64,
    #[validate(range(min = 0.001, max = 10.0))]
    sigma_log_depth: f64,
    #[validate(range(min = 1, max = 256))]
    neighbours: i64,
    #[validate(range(min = 1, max = 64))]
    grid_step: i64,
    #[validate(range(min = 0.01, max = 100.0))]
    reach: f64,
    #[validate(range(min = 0.0, max = 1.0))]
    shape_ema: f64,
    #[validate(range(min = 1, max = 1000000))]
    min_anchors: i64,
    /// Pixels leaning on nearby lidar less than this (0..1) are left out of the cloud; 0 keeps all.
    #[validate(range(min = 0.0, max = 1.0))]
    min_support: f64,
    /// Cloud crop, then decimation (one pixel per block), then a point budget (0 = none).
    #[validate(range(min = 0.0, max = 1000.0))]
    min_range_m: f64,
    #[validate(range(min = 0.0, max = 1000.0))]
    max_range_m: f64,
    #[validate(range(min = 1, max = 64))]
    decimation: i64,
    #[validate(range(min = 0, max = 10000000))]
    max_points: i64,
    /// Frame of the cloud; empty takes the CameraInfo's, then the image's.
    frame_id: String,
    /// Keep only points whose height in `height_frame` is within [min, max]; empty frame keeps all.
    height_frame: String,
    min_height_m: f64,
    max_height_m: f64,
}

#[derive(Module)]
#[module(name = "depth2depth_cloud", setup = start)]
pub struct Depth2DepthCloud {
    #[input(decode = CompressedImage::decode, handler = on_image)]
    image: Input<CompressedImage>,

    #[input(decode = CameraInfo::decode, handler = on_camera_info)]
    camera_info: Input<CameraInfo>,

    #[input(decode = PointCloud2::decode, handler = on_lidar)]
    lidar: Input<PointCloud2>,

    #[tf]
    tf: Tf,

    #[output(encode = PointCloud2::encode)]
    depth_cloud: Output<PointCloud2>,

    #[config]
    config: Config,

    shared: Arc<Shared>,
    wake: Option<SyncSender<()>>,
}

/// What the handlers hand the worker.
#[derive(Default)]
struct Shared {
    image: Mutex<Option<CompressedImage>>,
    camera_info: Mutex<Option<CameraInfo>>,
    history: Mutex<VecDeque<Scan>>,
}

/// One lidar scan, already in the world frame.
struct Scan {
    stamp: f64,
    points: Vec<[f32; 3]>,
}

impl Depth2DepthCloud {
    async fn start(&mut self) {
        let (wake, woken) = sync_channel(1);
        self.wake = Some(wake);
        let worker = Worker {
            shared: self.shared.clone(),
            tf: self.tf.clone(),
            output: self.depth_cloud.clone(),
            runtime: tokio::runtime::Handle::current(),
            config: self.config.clone(),
        };
        std::thread::spawn(move || worker.run(woken));
    }

    async fn on_image(&mut self, msg: CompressedImage) {
        *self.shared.image.lock().unwrap() = Some(msg);
        if let Some(wake) = &self.wake {
            let _ = wake.try_send(());
        }
    }

    async fn on_camera_info(&mut self, msg: CameraInfo) {
        *self.shared.camera_info.lock().unwrap() = Some(msg);
    }

    async fn on_lidar(&mut self, msg: PointCloud2) {
        // A blueprint may feed this cloud back on the lidar's port (one mapper input for both); it is no anchor.
        if msg.header.frame_id == self.output_frame() {
            return;
        }
        let stamp = seconds(&msg.header.stamp);
        let Some(world_from_lidar) = self
            .tf
            .lookup(&self.config.world_frame, &msg.header.frame_id)
            .at(stamp)
            .tolerance(self.config.tf_tolerance_s)
            .within(TF_WAIT)
            .await
        else {
            warn_throttled!(Duration::from_secs(5), frame = %msg.header.frame_id, "No transform for a lidar scan, dropped it.");
            return;
        };
        let points = match extract_xyz(&msg) {
            Ok(points) => points,
            Err(error) => {
                warn_throttled!(Duration::from_secs(5), %error, "Unreadable lidar scan, dropped it.");
                return;
            }
        };
        let pose = isometry(&world_from_lidar);
        let points = points
            .into_iter()
            .filter(|p| p.iter().all(|v| v.is_finite()))
            .map(|[x, y, z]| {
                let p = pose * Point3::new(x as f64, y as f64, z as f64);
                [p.x as f32, p.y as f32, p.z as f32]
            })
            .collect();
        let mut history = self.shared.history.lock().unwrap();
        history.push_back(Scan { stamp, points });
        // Keep a little past the window, since images may be stamped slightly behind the newest scan.
        let oldest = stamp - self.config.lidar_history_s - 1.0;
        while history.front().is_some_and(|scan| scan.stamp < oldest) {
            history.pop_front();
        }
    }
}

impl Depth2DepthCloud {
    fn output_frame(&self) -> String {
        let info = self.shared.camera_info.lock().unwrap();
        let info_frame = info.as_ref().map_or("", |i| i.header.frame_id.as_str());
        resolve_frame_id(&self.config.frame_id, info_frame, "").to_string()
    }
}

struct Worker {
    shared: Arc<Shared>,
    tf: Tf,
    output: Output<PointCloud2>,
    runtime: tokio::runtime::Handle,
    config: Config,
}

impl Worker {
    fn run(self, woken: Receiver<()>) {
        let cfg = &self.config;
        let model = match load_model(cfg) {
            Ok(model) => model,
            Err(error) => {
                tracing::error!(%error, weights_dir = %cfg.weights_dir, "Could not load the depth model; no clouds will be published.");
                return;
            }
        };
        let mut calibration = Calibration::new(CalibrationConfig {
            sigma_px: cfg.sigma_px as f32,
            sigma_log_depth: cfg.sigma_log_depth as f32,
            neighbours: cfg.neighbours as usize,
            grid_step: cfg.grid_step as usize,
            reach: cfg.reach as f32,
            shape_ema: cfg.shape_ema as f32,
            min_anchors: cfg.min_anchors as usize,
        });
        let mut undistort: Option<(CameraInfo, usize, UndistortMap)> = None;
        let mut timing = Timing::default();
        while woken.recv().is_ok() {
            let Some(image) = self.shared.image.lock().unwrap().take() else {
                continue;
            };
            let Some(info) = self.shared.camera_info.lock().unwrap().clone() else {
                warn_throttled!(
                    Duration::from_secs(5),
                    "No CameraInfo yet, skipped a frame."
                );
                continue;
            };
            let started = Instant::now();
            let Some((rgb, width, height)) = decode_rgb(&image, cfg.decode_scale as usize) else {
                warn_throttled!(Duration::from_secs(5), format = %image.format, "Could not decode a frame, skipped it.");
                continue;
            };
            let decoded = Instant::now();
            if !undistort.as_ref().is_some_and(|(held, held_width, _)| {
                held.K == info.K && held.D == info.D && *held_width == width
            }) {
                let scale = if info.width > 0 {
                    width as f64 / info.width as f64
                } else {
                    1.0 / cfg.decode_scale as f64
                };
                let Some(lens) = Lens::from_info(&info, scale) else {
                    warn_throttled!(
                        Duration::from_secs(5),
                        "CameraInfo has no usable intrinsics, skipped a frame."
                    );
                    continue;
                };
                let map = UndistortMap::new(
                    &lens,
                    cfg.undistorted_width as usize,
                    cfg.undistorted_height as usize,
                    cfg.undistorted_focal_px,
                );
                undistort = Some((info.clone(), width, map));
            }
            let (_, _, map) = undistort.as_ref().unwrap();
            let pinhole_rgb = map.apply(&rgb, width, height);
            let undistorted = Instant::now();

            let frame_id =
                resolve_frame_id(&cfg.frame_id, &info.header.frame_id, &image.header.frame_id)
                    .to_string();
            let stamp = seconds(&image.header.stamp);
            let lookup = self
                .tf
                .lookup(&cfg.world_frame, &frame_id)
                .at(stamp)
                .tolerance(cfg.tf_tolerance_s);
            let Some(world_from_camera) = self.runtime.block_on(lookup.within(FRAME_TF_WAIT))
            else {
                warn_throttled!(Duration::from_secs(5), frame = %frame_id, "No transform for a frame, skipped it.");
                continue;
            };
            let anchors = self.anchors(&isometry(&world_from_camera).inverse(), stamp);
            let anchored = Instant::now();

            let pred = match model.predict(&pinhole_rgb, map.height, map.width) {
                Ok(pred) => pred,
                Err(error) => {
                    warn_throttled!(Duration::from_secs(5), %error, "Depth model failed on a frame.");
                    continue;
                }
            };
            let predicted = Instant::now();
            let visible =
                depth2depth::cloud::visible_anchors(&anchors, &map.camera, map.height, map.width);
            let mut calibrated = calibration.apply(&pred, map.height, map.width, &visible);
            for (depth, support) in calibrated.depth.iter_mut().zip(&calibrated.support) {
                if (*support as f64) < cfg.min_support {
                    *depth = 0.0;
                }
            }
            let options = CloudOptions {
                min_range_m: cfg.min_range_m as f32,
                max_range_m: cfg.max_range_m as f32,
                decimation: cfg.decimation as usize,
                max_points: (cfg.max_points > 0).then_some(cfg.max_points as usize),
                ..CloudOptions::default()
            };
            let mut points = calibrated.points(map.height, map.width, &map.camera, &options);
            if !cfg.height_frame.is_empty() {
                let Some(base_from_camera) = self
                    .tf
                    .lookup(&cfg.height_frame, &frame_id)
                    .at(stamp)
                    .tolerance(cfg.tf_tolerance_s)
                    .get()
                else {
                    warn_throttled!(Duration::from_secs(5), frame = %cfg.height_frame, "No transform to the height frame, skipped a frame.");
                    continue;
                };
                let pose = isometry(&base_from_camera);
                points.retain(|&[x, y, z]| {
                    let height = (pose * Point3::new(x as f64, y as f64, z as f64)).z;
                    (cfg.min_height_m..=cfg.max_height_m).contains(&height)
                });
            }
            let calibrated_at = Instant::now();
            let cloud = make_cloud(&points, &image.header, frame_id);
            if let Err(error) = self.runtime.block_on(self.output.publish(&cloud)) {
                warn_throttled!(Duration::from_secs(5), %error, "Could not publish a cloud.");
            }
            timing.record(
                [
                    decoded - started,
                    undistorted - decoded,
                    anchored - undistorted,
                    predicted - anchored,
                    calibrated_at - predicted,
                ],
                visible.len(),
                points.len(),
            );
        }
    }

    /// The history's points in the camera frame, within range and in front of it.
    fn anchors(&self, camera_from_world: &Isometry3<f64>, stamp: f64) -> Vec<[f32; 3]> {
        let history = self.shared.history.lock().unwrap();
        let max_range = self.config.max_anchor_range_m as f32;
        history
            .iter()
            .filter(|scan| {
                scan.stamp >= stamp - self.config.lidar_history_s && scan.stamp <= stamp + 0.5
            })
            .flat_map(|scan| scan.points.iter())
            .map(|&[x, y, z]| {
                let p = camera_from_world * Point3::new(x as f64, y as f64, z as f64);
                [p.x as f32, p.y as f32, p.z as f32]
            })
            .filter(|p| {
                p[2] > 0.0 && p[0] * p[0] + p[1] * p[1] + p[2] * p[2] < max_range * max_range
            })
            .collect()
    }
}

/// On a Jetson, TensorRT: candle's CUDA path is bound by kernel launches there (190 ms a frame
/// against TensorRT's 17). The engine is built from the ONNX once and cached, which takes minutes.
#[cfg(all(target_os = "linux", target_arch = "aarch64"))]
fn load_model(cfg: &Config) -> Result<Depth2Depth, String> {
    let onnx = format!("{}/{}", cfg.weights_dir, cfg.onnx_file);
    std::fs::create_dir_all(&cfg.engine_cache_dir).map_err(|e| e.to_string())?;
    let stem = cfg.onnx_file.trim_end_matches(".onnx");
    let engine = format!("{}/{stem}.engine", cfg.engine_cache_dir);
    info!(%engine, "Loading the TensorRT engine (building it first if it is not cached).");
    let model = Depth2Depth::new_tensorrt(&onnx, &engine, ModelConfig::default())
        .map_err(|e| e.to_string())?;
    info!("Depth model loaded on TensorRT.");
    Ok(model)
}

/// Elsewhere candle: Metal on a Mac, else CPU.
#[cfg(not(all(target_os = "linux", target_arch = "aarch64")))]
fn load_model(cfg: &Config) -> Result<Depth2Depth, String> {
    use depth2depth::candle::{DType, Device};
    #[cfg(target_os = "macos")]
    let device = Device::new_metal(0).unwrap_or(Device::Cpu);
    #[cfg(not(target_os = "macos"))]
    let device = Device::Cpu;
    let weights = |name: &str| format!("{}/{name}", cfg.weights_dir);
    let model_config = ModelConfig {
        model_h: cfg.model_height as usize,
        model_w: cfg.model_width as usize,
        ..ModelConfig::default()
    };
    let dtype = if device.is_cpu() {
        DType::F32
    } else {
        DType::F16
    };
    let model = Depth2Depth::new(
        &weights("dinov2_vits14.safetensors"),
        &weights("da2_head_vits.safetensors"),
        device.clone(),
        dtype,
        model_config,
    )
    .map_err(|e| e.to_string())?;
    info!(?device, "Depth model loaded.");
    Ok(model)
}

/// Decode a JPEG as RGB at 1/`scale` of its size; None for anything that is not one.
fn decode_rgb(image: &CompressedImage, scale: usize) -> Option<(Vec<u8>, usize, usize)> {
    let format = image.format.to_ascii_lowercase();
    if !(format.contains("jpeg") || format.contains("jpg") || format.is_empty()) {
        return None;
    }
    let scaling = match scale {
        8 => turbojpeg::ScalingFactor::ONE_EIGHTH,
        4 => turbojpeg::ScalingFactor::ONE_QUARTER,
        2 => turbojpeg::ScalingFactor::ONE_HALF,
        _ => turbojpeg::ScalingFactor::ONE,
    };
    let mut decompressor = turbojpeg::Decompressor::new().ok()?;
    let header = decompressor.read_header(&image.data).ok()?;
    decompressor.set_scaling_factor(scaling).ok()?;
    let (width, height) = (scaling.scale(header.width), scaling.scale(header.height));
    let mut pixels = vec![0u8; width * height * 3];
    decompressor
        .decompress(
            &image.data,
            turbojpeg::Image {
                pixels: &mut pixels[..],
                width,
                pitch: width * 3,
                height,
                format: turbojpeg::PixelFormat::RGB,
            },
        )
        .ok()?;
    Some((pixels, width, height))
}

fn make_cloud(points: &[[f32; 3]], source: &Header, frame_id: String) -> PointCloud2 {
    let field = |name: &str, offset: i32| PointField {
        name: name.into(),
        offset,
        datatype: PointField::FLOAT32 as u8,
        count: 1,
    };
    let data: Vec<u8> = points
        .iter()
        .flatten()
        .flat_map(|v| v.to_le_bytes())
        .collect();
    PointCloud2 {
        header: Header {
            seq: source.seq,
            stamp: source.stamp.clone(),
            frame_id,
        },
        height: 1,
        width: points.len() as i32,
        fields: vec![field("x", 0), field("y", 4), field("z", 8)],
        is_bigendian: false,
        point_step: 12,
        row_step: 12 * points.len() as i32,
        data,
        is_dense: true,
    }
}

fn isometry(transform: &dimos_module::tf::Transform) -> Isometry3<f64> {
    Isometry3::from_parts(transform.translation().into(), transform.rotation())
}

fn seconds(stamp: &Time) -> f64 {
    stamp.sec as f64 + stamp.nsec as f64 * 1e-9
}

/// Config, then calibration, then image frame, since drivers stamp frames nobody publishes a transform for.
fn resolve_frame_id<'a>(config: &'a str, info: &'a str, image: &'a str) -> &'a str {
    [config, info, image]
        .into_iter()
        .find(|f| !f.is_empty())
        .unwrap_or("")
}

/// Rolling per-stage timing, logged every few seconds so throughput is visible live.
#[derive(Default)]
struct Timing {
    frames: Vec<[Duration; 5]>,
    anchors: usize,
    points: usize,
    since: Option<Instant>,
}

impl Timing {
    fn record(&mut self, stages: [Duration; 5], anchors: usize, points: usize) {
        let since = *self.since.get_or_insert_with(Instant::now);
        self.frames.push(stages);
        (self.anchors, self.points) = (anchors, points);
        let elapsed = since.elapsed();
        if elapsed < TIMING_REPORT_EVERY {
            return;
        }
        let median_ms = |stage: usize| {
            let mut ms: Vec<f64> = self
                .frames
                .iter()
                .map(|f| f[stage].as_secs_f64() * 1000.0)
                .collect();
            ms.sort_by(f64::total_cmp);
            ms[ms.len() / 2]
        };
        info!(
            fps = self.frames.len() as f64 / elapsed.as_secs_f64(),
            decode_ms = median_ms(0),
            undistort_ms = median_ms(1),
            anchors_ms = median_ms(2),
            predict_ms = median_ms(3),
            calibrate_ms = median_ms(4),
            anchors = self.anchors,
            points = self.points,
            "depth2depth_cloud timing (median ms per stage over the window)",
        );
        self.frames.clear();
        self.since = Some(Instant::now());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frame_id_precedence_is_config_then_calibration_then_image() {
        assert_eq!(resolve_frame_id("a", "b", "c"), "a");
        assert_eq!(resolve_frame_id("", "b", "c"), "b");
        assert_eq!(resolve_frame_id("", "", "c"), "c");
    }

    #[test]
    fn a_non_jpeg_format_is_refused_rather_than_fed_to_the_decoder() {
        let png = CompressedImage {
            format: "png".into(),
            ..Default::default()
        };
        assert!(decode_rgb(&png, 1).is_none());
    }

    #[test]
    fn the_cloud_carries_the_points_and_the_source_stamp() {
        let header = Header {
            seq: 7,
            stamp: Time { sec: 3, nsec: 4 },
            frame_id: "image".into(),
        };
        let cloud = make_cloud(
            &[[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]],
            &header,
            "camera".into(),
        );
        assert_eq!(
            (
                cloud.width,
                cloud.header.frame_id.as_str(),
                cloud.header.stamp.nsec
            ),
            (2, "camera", 4)
        );
        assert_eq!(
            extract_xyz(&cloud).unwrap(),
            vec![[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]
        );
    }
}
