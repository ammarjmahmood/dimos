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

//! `stereo_cloud`: depth and a point cloud from a calibrated stereo RGB pair, via [`crate::unproject`].

use std::collections::VecDeque;
use std::time::Duration;

use dimos_module::{native_config, warn_throttled, Input, Module, Output};
use lcm_msgs::sensor_msgs::{CameraInfo, CompressedImage, Image, PointCloud2};
use lcm_msgs::std_msgs::Header;
use serde::{Deserialize, Serialize};

use crate::denoise::Chain;
use crate::module::make_cloud;
use crate::stereo::{
    disparity_to_depth, downsample, horizontal_fov, match_stereo, rectify_pair_rotated, to_gray,
    Camera, Gray, MatchParams, Rotation, StereoRectification,
};
use crate::timing::{Stopwatch, Window, STAGES};
use crate::unproject::{unproject, HeightGate, Params};

/// Python's `None` as a JSON null, since native_config forbids `Option` (the Python side lists these in `base_fields`).
#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(transparent)]
pub struct Nullable<T>(pub Option<T>);

const TIMING_REPORT_EVERY: Duration = Duration::from_secs(5);

#[native_config]
pub struct Config {
    /// Distance between the two cameras in metres; depth scales linearly with it.
    #[validate(range(min = 0.001, max = 2.0))]
    baseline_m: f64,

    /// Integer downscale before matching, since full-resolution matching costs far more than navigation needs.
    #[validate(range(min = 1, max = 16))]
    downscale: i64,

    /// Number of disparities searched, which sets the nearest depth that can be matched.
    #[validate(range(min = 8, max = 512))]
    disparity_range: i64,

    /// SGM smoothness penalties: a one-step disparity change costs `p1`, a larger jump `p2` (must exceed `p1`).
    #[validate(range(min = 0, max = 255))]
    p1: i64,
    #[validate(range(min = 1, max = 4096))]
    p2: i64,

    /// Fraction by which the winning disparity must beat the runner-up; higher drops more invented textureless surfaces.
    #[validate(range(min = 0.0, max = 1.0))]
    uniqueness: f64,

    /// Largest left/right disparity disagreement kept, in pixels; negative disables the check (tests only).
    max_lr_difference: f64,

    /// Disparity regions smaller than this are speckle a map would see as floating obstacles; 0 or 1 disables.
    #[validate(range(min = 0, max = 100000))]
    min_region: i64,

    /// Disparity step below which neighbouring pixels count as one surface for the speckle flood fill.
    #[validate(range(min = 0.0, max = 64.0))]
    speckle_max_step: f64,

    /// Also aggregate along the diagonals: doubles matcher cost for better support on textureless surfaces like floors.
    diagonal_paths: bool,

    /// Disparities at or below this are dropped, since near-zero disparity turns small noise into far-away obstacles.
    #[validate(range(min = 0.0, max = 64.0))]
    min_disparity_px: f64,

    /// Keep every Nth point when building the cloud.
    #[validate(range(min = 1, max = 64))]
    decimation: i64,

    #[validate(range(min = 0.0, max = 1000.0))]
    min_range_m: f64,
    #[validate(range(min = 0.0, max = 1000.0))]
    max_range_m: f64,

    /// Frame the cloud is published in; empty defers to the left CameraInfo, then the left image.
    frame_id: String,

    /// Largest timestamp gap (seconds) still paired; unsynchronised cameras skew, but too much smears moving objects.
    #[validate(range(min = 0.0, max = 1.0))]
    max_pair_skew_s: f64,

    /// Right camera's rotation relative to the left in radians, the stereo calibration two monocular CameraInfos lack.
    #[validate(range(min = -0.2, max = 0.2))]
    right_roll_rad: f64,
    #[validate(range(min = -0.2, max = 0.2))]
    right_pitch_rad: f64,
    #[validate(range(min = -0.2, max = 0.2))]
    right_yaw_rad: f64,

    /// Depth denoise chain applied before unprojection, filters joined by `+` (see [`crate::denoise`]); `none` disables.
    denoise: String,

    /// Keep cloud points with base-frame height in `[min_height_m, max_height_m]` (null = unbounded).
    min_height_m: Nullable<f64>,
    max_height_m: Nullable<f64>,

    /// Optical-frame pose in base (`R(rpy) * p + xyz`, extrinsic XYZ like a URDF `rpy`), for the height gate.
    base_from_camera_xyz_m: [f64; 3],
    base_from_camera_rpy_rad: [f64; 3],
}

#[derive(Module)]
#[module(name = "stereo_cloud", setup = prepare)]
pub struct StereoCloud {
    #[input(decode = CompressedImage::decode, handler = on_left)]
    left: Input<CompressedImage>,

    #[input(decode = CompressedImage::decode, handler = on_right)]
    right: Input<CompressedImage>,

    #[input(decode = CameraInfo::decode, handler = on_left_info)]
    left_info: Input<CameraInfo>,

    #[input(decode = CameraInfo::decode, handler = on_right_info)]
    right_info: Input<CameraInfo>,

    #[output(encode = PointCloud2::encode)]
    cloud: Output<PointCloud2>,

    /// The rectified depth map behind the cloud, published because a bad rectification is visible there and not in 3D.
    #[output(encode = Image::encode)]
    depth: Output<Image>,

    /// The rectified intrinsics of `depth`, so a viewer can project it into 3D.
    #[output(encode = CameraInfo::encode)]
    depth_info: Output<CameraInfo>,

    #[config]
    config: Config,

    pending_left: VecDeque<CompressedImage>,
    pending_right: VecDeque<CompressedImage>,
    left_camera: Option<CameraInfo>,
    right_camera: Option<CameraInfo>,

    /// Built once both CameraInfos have arrived and the first frame has fixed the resolution.
    rectify: Option<StereoRectification>,

    /// Empty when `config.denoise` was invalid, so the module runs unfiltered instead of crash-looping.
    denoise: Chain,

    /// Per-stage time of recent frames, logged every few seconds.
    timing: Window,

    /// A steadily growing count means one camera runs ahead or `max_pair_skew_s` is too tight.
    frames_dropped_unpaired: u64,
}

impl StereoCloud {
    async fn prepare(&mut self) {
        self.denoise = parse_denoise_or_none(&self.config.denoise);
        tracing::info!(
            denoise = %self.denoise.name(),
            min_height_m = ?self.config.min_height_m.0,
            max_height_m = ?self.config.max_height_m.0,
            "stereo_cloud ready",
        );
    }

    async fn on_left_info(&mut self, msg: CameraInfo) {
        if !same_intrinsics(self.left_camera.as_ref(), &msg) {
            self.rectify = None;
        }
        self.left_camera = Some(msg);
    }

    async fn on_right_info(&mut self, msg: CameraInfo) {
        if !same_intrinsics(self.right_camera.as_ref(), &msg) {
            self.rectify = None;
        }
        self.right_camera = Some(msg);
    }

    async fn on_left(&mut self, msg: CompressedImage) {
        Self::hold(
            &mut self.pending_left,
            msg,
            &mut self.frames_dropped_unpaired,
        );
        self.try_pair().await;
    }

    async fn on_right(&mut self, msg: CompressedImage) {
        Self::hold(
            &mut self.pending_right,
            msg,
            &mut self.frames_dropped_unpaired,
        );
        self.try_pair().await;
    }

    /// Queue a few frames per camera so a frame can wait for a partner still in flight instead of being evicted.
    fn hold(pending: &mut VecDeque<CompressedImage>, msg: CompressedImage, dropped: &mut u64) {
        pending.push_back(msg);
        while pending.len() > PAIR_QUEUE {
            pending.pop_front();
            *dropped += 1;
        }
    }

    /// Process the closest-stamped pair within the skew, dropping anything older since it could only pair with staler frames.
    async fn try_pair(&mut self) {
        let Some((i, j)) = closest_pair(
            &self.pending_left,
            &self.pending_right,
            self.config.max_pair_skew_s,
        ) else {
            return;
        };
        let left = self
            .pending_left
            .remove(i)
            .expect("indexed within the queue");
        let right = self
            .pending_right
            .remove(j)
            .expect("indexed within the queue");
        self.frames_dropped_unpaired += (i + j) as u64;
        self.pending_left.drain(..i);
        self.pending_right.drain(..j);
        self.process(left, right).await;
    }

    async fn process(&mut self, left: CompressedImage, right: CompressedImage) {
        let (Some(left_info), Some(right_info)) =
            (self.left_camera.clone(), self.right_camera.clone())
        else {
            // No intrinsics yet, so no geometry.
            return;
        };

        let mut stopwatch = Stopwatch::start();
        let factor = self.config.downscale.max(1) as usize;
        let (Some(left_decoded), Some(right_decoded)) = decode_pair_scaled(&left, &right, factor)
        else {
            warn_throttled!(
                Duration::from_secs(1),
                format = %left.format,
                "Could not decode a camera frame, dropped the pair.",
            );
            return;
        };
        let (left_gray, right_gray) = (&left_decoded.gray, &right_decoded.gray);
        if left_gray.width != right_gray.width || left_gray.height != right_gray.height {
            warn_throttled!(
                Duration::from_secs(5),
                left = left_gray.width,
                right = right_gray.width,
                "Left and right cameras disagree on resolution, dropped the pair.",
            );
            return;
        }
        let decode_ms = stopwatch.lap_ms();

        // Whatever downscale libjpeg could not do in the IDCT.
        let left_small = downsample(left_gray, left_decoded.remaining);
        let right_small = downsample(right_gray, right_decoded.remaining);

        if self.rectify.is_none() {
            self.rectify = build_rectification(
                &left_info,
                &right_info,
                left_decoded.full_width,
                left_decoded.full_height,
                factor,
                Rotation {
                    roll_rad: self.config.right_roll_rad,
                    pitch_rad: self.config.right_pitch_rad,
                    yaw_rad: self.config.right_yaw_rad,
                },
            );
            if let Some(built) = self.rectify.as_ref() {
                tracing::info!(
                    width = built.width,
                    height = built.height,
                    fx = built.fx,
                    baseline_m = self.config.baseline_m,
                    horizontal_fov_deg = horizontal_fov(built.fx as f64, built.width),
                    "stereo_cloud rectification ready",
                );
            }
        }
        let Some(rectification) = self.rectify.as_ref() else {
            warn_throttled!(
                Duration::from_secs(5),
                "A CameraInfo has no usable intrinsics, dropped the pair.",
            );
            return;
        };

        let left_rect = rectification.left.apply(&left_small);
        let right_rect = rectification.right.apply(&right_small);
        let rectify_ms = stopwatch.lap_ms();

        let params = MatchParams {
            min_disparity: 0,
            disparity_range: self.config.disparity_range as usize,
            p1: self.config.p1 as u16,
            p2: self.config.p2 as u16,
            uniqueness: self.config.uniqueness as f32,
            max_lr_difference: self.config.max_lr_difference as f32,
            min_region: self.config.min_region as usize,
            speckle_max_step: self.config.speckle_max_step as f32,
            diagonal_paths: self.config.diagonal_paths,
        };
        let disparity = match_stereo(&left_rect, &right_rect, &params);
        let depth_metres = disparity_to_depth(
            &disparity,
            rectification.fx,
            self.config.baseline_m as f32,
            self.config.min_disparity_px as f32,
        );
        let match_ms = stopwatch.lap_ms();

        // Denoise before building the image so the published depth and the cloud match.
        let depth_metres = self.denoise.apply(
            &depth_metres,
            rectification.width,
            rectification.height,
            rectification.fx,
        );
        let denoise_ms = stopwatch.lap_ms();

        let frame_id = resolve_frame_id(
            &self.config.frame_id,
            &left_info.header.frame_id,
            &left.header.frame_id,
        )
        .to_owned();

        let depth_image = depth_to_image(
            &depth_metres,
            rectification.width,
            rectification.height,
            &left.header,
            &frame_id,
        );

        // The depth map is in the rectified frame, so unproject through the rectified intrinsics.
        let mut rectified_info = rectified_camera_info(rectification, &left_info);
        rectified_info.header = depth_image.header.clone();
        let cloud_params = Params {
            decimation: self.config.decimation as usize,
            min_range_m: self.config.min_range_m as f32,
            max_range_m: self.config.max_range_m as f32,
            depth_scale: 1.0,
            height_gate: HeightGate::new(
                self.config.base_from_camera_xyz_m,
                self.config.base_from_camera_rpy_rad,
                self.config.min_height_m.0,
                self.config.max_height_m.0,
            ),
        };
        match unproject(&depth_image, &rectified_info, &cloud_params) {
            Ok((data, count)) => {
                let cloud = make_cloud(data, count, frame_id, left.header.clone());
                self.cloud.publish(&cloud).await.ok();
            }
            Err(error) => {
                warn_throttled!(
                    Duration::from_secs(1),
                    error = %error,
                    "Could not unproject the stereo depth map.",
                );
            }
        }
        self.depth.publish(&depth_image).await.ok();
        self.depth_info.publish(&rectified_info).await.ok();
        let unproject_ms = stopwatch.lap_ms();

        // In STAGES order.
        debug_assert_eq!(
            STAGES,
            ["decode", "rectify", "match", "denoise", "unproject"]
        );
        self.timing
            .push([decode_ms, rectify_ms, match_ms, denoise_ms, unproject_ms]);
        if let Some(summary) = self.timing.report_due(TIMING_REPORT_EVERY) {
            let [decode, rectify, matching, denoise, unproject] = summary.median_ms;
            let tenths = |ms: f32| (ms * 10.0).round() / 10.0;
            tracing::info!(
                frames = summary.frames,
                fps = (summary.fps * 100.0).round() / 100.0,
                decode_ms = tenths(decode),
                rectify_ms = tenths(rectify),
                match_ms = tenths(matching),
                denoise_ms = tenths(denoise),
                unproject_ms = tenths(unproject),
                frames_dropped_unpaired = self.frames_dropped_unpaired,
                "stereo_cloud timing (median ms per stage over the window)",
            );
        }
    }
}

/// The configured chain, or none (logged as an error) if it does not parse, so the module still publishes.
pub fn parse_denoise_or_none(text: &str) -> Chain {
    match Chain::parse(text) {
        Ok(chain) => chain,
        Err(error) => {
            tracing::error!(
                denoise = text,
                error = %error,
                "Invalid denoise chain, running with none.",
            );
            Chain(Vec::new())
        }
    }
}

/// Frames a camera may hold while waiting for its partner, ample reordering room at 30 Hz.
const PAIR_QUEUE: usize = 8;

/// The fields the rectification depends on, so a republished identical CameraInfo does not trigger a rebuild.
fn same_intrinsics(held: Option<&CameraInfo>, incoming: &CameraInfo) -> bool {
    held.is_some_and(|h| {
        h.width == incoming.width
            && h.height == incoming.height
            && h.K == incoming.K
            && h.D == incoming.D
            && h.distortion_model == incoming.distortion_model
    })
}

/// Queue positions of the closest-stamped pair within `max_skew_s`, if any.
fn closest_pair(
    left: &VecDeque<CompressedImage>,
    right: &VecDeque<CompressedImage>,
    max_skew_s: f64,
) -> Option<(usize, usize)> {
    let mut best: Option<(usize, usize, f64)> = None;
    for (i, l) in left.iter().enumerate() {
        for (j, r) in right.iter().enumerate() {
            let skew = (stamp_seconds(&l.header) - stamp_seconds(&r.header)).abs();
            if skew <= max_skew_s && best.is_none_or(|(_, _, s)| skew < s) {
                best = Some((i, j, skew));
            }
        }
    }
    best.map(|(i, j, _)| (i, j))
}

fn stamp_seconds(header: &Header) -> f64 {
    header.stamp.sec as f64 + header.stamp.nsec as f64 * 1e-9
}

/// A scaled decode: the image, the camera's full size, and the downscale still left for the caller.
pub struct Decoded {
    pub gray: Gray,
    pub full_width: usize,
    pub full_height: usize,
    pub remaining: usize,
}

/// Decode both cameras' frames in parallel, since decode is two equal independent jobs.
pub fn decode_pair_scaled(
    left: &CompressedImage,
    right: &CompressedImage,
    factor: usize,
) -> (Option<Decoded>, Option<Decoded>) {
    rayon::join(
        || decode_scaled(left, factor),
        || decode_scaled(right, factor),
    )
}

/// Decode to grayscale with libjpeg doing what downscale it can in the IDCT, which is nearly free versus a full decode.
pub fn decode_scaled(image: &CompressedImage, factor: usize) -> Option<Decoded> {
    let format = image.format.to_ascii_lowercase();
    if !(format.contains("jpeg") || format.contains("jpg") || format.is_empty()) {
        return None;
    }
    // Only powers of two dividing the factor, or the image would not match the rectification.
    let (scaling, by) = match factor {
        f if f % 8 == 0 => (turbojpeg::ScalingFactor::ONE_EIGHTH, 8),
        f if f % 4 == 0 => (turbojpeg::ScalingFactor::ONE_QUARTER, 4),
        f if f % 2 == 0 => (turbojpeg::ScalingFactor::ONE_HALF, 2),
        _ => (turbojpeg::ScalingFactor::ONE, 1),
    };

    let mut decompressor = turbojpeg::Decompressor::new().ok()?;
    let header = decompressor.read_header(&image.data).ok()?;
    let (full_width, full_height) = (header.width, header.height);
    decompressor.set_scaling_factor(scaling).ok()?;
    let width = scaling.scale(full_width);
    let height = scaling.scale(full_height);
    if width == 0 || height == 0 {
        return None;
    }

    let mut pixels = vec![0u8; width * height];
    decompressor
        .decompress(
            &image.data,
            turbojpeg::Image {
                pixels: &mut pixels[..],
                width,
                pitch: width,
                height,
                format: turbojpeg::PixelFormat::GRAY,
            },
        )
        .ok()?;
    Some(Decoded {
        gray: to_gray(&pixels, width, height, 1)?,
        full_width,
        full_height,
        remaining: factor / by,
    })
}

/// Rectification at the downscaled resolution, via the shared `stereo` geometry so offline tools and the module agree.
pub fn build_rectification(
    left_info: &CameraInfo,
    right_info: &CameraInfo,
    full_width: usize,
    full_height: usize,
    factor: usize,
    right_rotation: Rotation,
) -> Option<StereoRectification> {
    let factor = factor.max(1);
    let width = full_width / factor;
    let height = full_height / factor;
    if width == 0 || height == 0 {
        return None;
    }
    let left_camera = camera_from(left_info, full_width, full_height, factor)?;
    let right_camera = camera_from(right_info, full_width, full_height, factor)?;
    Some(rectify_pair_rotated(
        &left_camera,
        &right_camera,
        width,
        height,
        right_rotation,
    ))
}

/// Pull intrinsics out of a `CameraInfo`, rescaled to the working resolution.
fn camera_from(
    info: &CameraInfo,
    full_width: usize,
    full_height: usize,
    factor: usize,
) -> Option<Camera> {
    let (fx, fy) = (info.K[0], info.K[4]);
    let (cx, cy) = (info.K[2], info.K[5]);
    if fx == 0.0 || fy == 0.0 || !fx.is_finite() || !fy.is_finite() {
        return None;
    }
    // Fold in a calibration taken at a different resolution than the frames.
    let (calibration_scale_x, calibration_scale_y) = if info.width > 0 && info.height > 0 {
        (
            full_width as f64 / info.width as f64,
            full_height as f64 / info.height as f64,
        )
    } else {
        (1.0, 1.0)
    };
    let sx = calibration_scale_x / factor as f64;
    let sy = calibration_scale_y / factor as f64;

    // Eight coefficients mean the rational model whatever `distortion_model` claims; some drivers mislabel it `plumb_bob`.
    let mut distortion = [0.0f64; 8];
    for (slot, value) in distortion.iter_mut().zip(info.D.iter()) {
        *slot = *value;
    }

    Some(Camera {
        fx: fx * sx,
        fy: fy * sy,
        cx: cx * sx,
        cy: cy * sy,
        distortion,
    })
}

/// A `CameraInfo` describing the rectified image the depth map lives in.
pub fn rectified_camera_info(
    rectification: &StereoRectification,
    source: &CameraInfo,
) -> CameraInfo {
    let mut info = source.clone();
    info.width = rectification.width as i32;
    info.height = rectification.height as i32;
    info.K = [
        rectification.fx as f64,
        0.0,
        rectification.cx as f64,
        0.0,
        rectification.fy as f64,
        rectification.cy as f64,
        0.0,
        0.0,
        1.0,
    ];
    info.D = vec![0.0; 5];
    info.distortion_model = "plumb_bob".into();
    info
}

pub fn depth_to_image(
    depth: &[f32],
    width: usize,
    height: usize,
    source: &Header,
    frame_id: &str,
) -> Image {
    let mut data = Vec::with_capacity(depth.len() * 4);
    for value in depth {
        data.extend_from_slice(&value.to_le_bytes());
    }
    Image {
        header: Header {
            seq: source.seq,
            stamp: source.stamp.clone(),
            frame_id: frame_id.to_owned(),
        },
        height: height as i32,
        width: width as i32,
        encoding: "32FC1".into(),
        is_bigendian: 0,
        step: (width * 4) as i32,
        data,
    }
}

/// Config, then calibration, then image frame, like `DepthCloud`, since drivers stamp frames nobody publishes a transform for.
fn resolve_frame_id<'a>(config: &'a str, info: &'a str, image: &'a str) -> &'a str {
    [config, info, image]
        .into_iter()
        .find(|candidate| !candidate.is_empty())
        .unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn info(width: i32, height: i32, fx: f64, coefficients: usize) -> CameraInfo {
        CameraInfo {
            width,
            height,
            distortion_model: "plumb_bob".into(),
            D: vec![0.0; coefficients],
            K: [
                fx,
                0.0,
                width as f64 / 2.0,
                0.0,
                fx,
                height as f64 / 2.0,
                0.0,
                0.0,
                1.0,
            ],
            ..Default::default()
        }
    }

    #[test]
    fn downscaling_scales_the_intrinsics_with_it() {
        let camera = camera_from(&info(1920, 1536, 1012.6, 8), 1920, 1536, 4)
            .expect("valid intrinsics should convert");
        assert!((camera.fx - 253.15).abs() < 0.01, "fx was {}", camera.fx);
        assert!((camera.cx - 240.0).abs() < 0.01, "cx was {}", camera.cx);
    }

    #[test]
    fn a_calibration_taken_at_another_resolution_is_folded_in() {
        // Calibrated at half resolution, so the focal length must double.
        let camera = camera_from(&info(960, 768, 506.3, 8), 1920, 1536, 1)
            .expect("valid intrinsics should convert");
        assert!((camera.fx - 1012.6).abs() < 0.01, "fx was {}", camera.fx);
    }

    #[test]
    fn zero_focal_length_is_rejected_rather_than_dividing_by_zero() {
        assert!(camera_from(&info(1920, 1536, 0.0, 8), 1920, 1536, 4).is_none());
    }

    #[test]
    fn a_five_coefficient_calibration_still_works() {
        let camera =
            camera_from(&info(640, 480, 500.0, 5), 640, 480, 1).expect("plumb_bob should convert");
        // Zero rational terms collapse the model back to plumb_bob.
        assert_eq!(camera.distortion[5], 0.0);
        assert_eq!(camera.distortion[6], 0.0);
        assert_eq!(camera.distortion[7], 0.0);
    }

    #[test]
    fn both_cameras_get_the_same_rectified_intrinsics() {
        // Disparity needs a shared focal length and principal point.
        let left = info(1920, 1536, 1012.59, 8);
        let right = info(1920, 1536, 1013.80, 8);
        let rectification = build_rectification(&left, &right, 1920, 1536, 4, Rotation::IDENTITY)
            .expect("should build");
        assert_eq!(rectification.width, 480);
        assert_eq!(rectification.height, 384);
        let fx = rectification.fx as f64;
        assert!(fx > 1012.59 / 4.0 && fx < 1013.80 / 4.0, "fx was {fx}");
        assert!((rectification.cx - 240.0).abs() < 0.01);
        assert!((rectification.cy - 192.0).abs() < 0.01);
    }

    #[test]
    fn the_rectified_camera_info_describes_the_depth_map_not_the_raw_frame() {
        let source = info(1920, 1536, 1012.59, 8);
        let rectification =
            build_rectification(&source, &source, 1920, 1536, 4, Rotation::IDENTITY)
                .expect("should build");
        let rectified = rectified_camera_info(&rectification, &source);
        assert_eq!(rectified.width, 480);
        assert_eq!(rectified.height, 384);
        assert!((rectified.K[0] - rectification.fx as f64).abs() < 1e-6);
        assert!(
            rectified.D.iter().all(|&d| d == 0.0),
            "rectified image must not be undistorted a second time"
        );
    }

    #[test]
    fn depth_image_is_32fc1_with_a_tight_row_stride() {
        let depth = vec![1.0f32, 2.0, 3.0, 4.0, 5.0, 6.0];
        let image = depth_to_image(&depth, 3, 2, &Header::default(), "camera_left_link");
        assert_eq!(image.encoding, "32FC1");
        assert_eq!(image.width, 3);
        assert_eq!(image.height, 2);
        assert_eq!(image.step, 12);
        assert_eq!(image.data.len(), 24);
        assert_eq!(image.header.frame_id, "camera_left_link");
        assert_eq!(
            f32::from_le_bytes(image.data[0..4].try_into().unwrap()),
            1.0
        );
    }

    #[test]
    fn frame_id_precedence_is_config_then_calibration_then_image() {
        assert_eq!(resolve_frame_id("config", "info", "image"), "config");
        assert_eq!(resolve_frame_id("", "info", "image"), "info");
        assert_eq!(resolve_frame_id("", "", "image"), "image");
        assert_eq!(resolve_frame_id("", "", ""), "");
    }

    #[test]
    fn stamp_seconds_combines_both_halves() {
        let mut header = Header::default();
        header.stamp.sec = 1000;
        header.stamp.nsec = 500_000_000;
        assert!((stamp_seconds(&header) - 1000.5).abs() < 1e-9);
    }

    #[test]
    fn a_non_jpeg_format_is_refused_rather_than_fed_to_the_decoder() {
        let image = CompressedImage {
            format: "png".into(),
            data: vec![0; 16],
            ..Default::default()
        };
        assert!(decode_scaled(&image, 4).is_none());
    }

    /// A real JPEG, so tests catch libjpeg's scaled size disagreeing with `full / factor`.
    fn jpeg_of(width: usize, height: usize) -> CompressedImage {
        let mut pixels = vec![0u8; width * height];
        for row in 0..height {
            for column in 0..width {
                // Structured, so the DCT has real coefficients.
                pixels[row * width + column] = ((row * 7 + column * 3) % 256) as u8;
            }
        }
        let data = turbojpeg::compress(
            turbojpeg::Image {
                pixels: &pixels[..],
                width,
                pitch: width,
                height,
                format: turbojpeg::PixelFormat::GRAY,
            },
            90,
            turbojpeg::Subsamp::Gray,
        )
        .expect("compress")
        .to_vec();
        CompressedImage {
            format: "jpeg".into(),
            data,
            ..Default::default()
        }
    }

    #[test]
    fn a_real_jpeg_decodes_to_exactly_the_size_the_rectification_assumes() {
        for factor in [1usize, 2, 4, 8] {
            let image = jpeg_of(1920, 1536);
            let decoded = decode_scaled(&image, factor).expect("decodes");
            assert_eq!(decoded.full_width, 1920, "factor {factor}");
            assert_eq!(decoded.full_height, 1536, "factor {factor}");
            // Size after the leftover box pass.
            let final_width = decoded.gray.width / decoded.remaining;
            let final_height = decoded.gray.height / decoded.remaining;
            assert_eq!(final_width, 1920 / factor, "factor {factor} width");
            assert_eq!(final_height, 1536 / factor, "factor {factor} height");
        }
    }

    #[test]
    fn a_scaled_decode_carries_the_same_picture_as_a_full_one() {
        // A truncated IDCT is a different low-pass than a box average, but must be the same image.
        let image = jpeg_of(640, 512);
        let scaled = decode_scaled(&image, 4).expect("scaled");
        let full = decode_scaled(&image, 1).expect("full");
        let boxed = downsample(&full.gray, 4);
        assert_eq!(scaled.gray.width, boxed.width);
        assert_eq!(scaled.gray.height, boxed.height);

        let error: f64 = scaled
            .gray
            .data
            .iter()
            .zip(&boxed.data)
            .map(|(a, b)| (*a as f64 - *b as f64).abs())
            .sum::<f64>()
            / boxed.data.len() as f64;
        assert!(error < 12.0, "mean |difference| was {error} grey levels");
    }

    #[test]
    fn libjpeg_is_asked_for_the_largest_scale_that_divides_the_downscale() {
        // libjpeg's scale must divide the factor so the box downsample finishes the job exactly.
        for (factor, expected_remaining) in
            [(1, 1), (2, 1), (3, 3), (4, 1), (6, 3), (8, 1), (12, 3)]
        {
            let image = CompressedImage {
                format: "jpeg".into(),
                data: vec![0; 16],
                ..Default::default()
            };
            // Not a real JPEG; the table checks the mirrored divisor arithmetic below.
            assert!(decode_scaled(&image, factor).is_none());
            let by = match factor {
                f if f % 8 == 0 => 8,
                f if f % 4 == 0 => 4,
                f if f % 2 == 0 => 2,
                _ => 1,
            };
            assert_eq!(factor / by, expected_remaining, "factor {factor}");
        }
    }

    #[test]
    fn a_bad_denoise_chain_becomes_none_rather_than_a_panic() {
        // A panic would crash-loop under the supervisor; unfiltered depth is better.
        let chain = parse_denoise_or_none("median:8+nonsense:3");
        assert_eq!(chain.name(), "none");
        assert_eq!(parse_denoise_or_none("median:8+fill:8").0.len(), 2);
        assert_eq!(parse_denoise_or_none("none").0.len(), 0);
    }

    #[test]
    fn a_null_height_bound_deserialises_to_none() {
        // The wire shape the Python side sends for `min_height_m=None`.
        let bound: Nullable<f64> = serde_json::from_str("null").expect("null parses");
        assert_eq!(bound.0, None);
        let bound: Nullable<f64> = serde_json::from_str("0.05").expect("number parses");
        assert_eq!(bound.0, Some(0.05));
    }

    #[test]
    fn the_configured_rotation_reaches_the_right_eyes_map() {
        // A rotation read but never applied would look exactly like a parallel rig.
        let source = info(1920, 1536, 1012.59, 8);
        let parallel = build_rectification(&source, &source, 1920, 1536, 4, Rotation::IDENTITY)
            .expect("built");
        let turned = build_rectification(
            &source,
            &source,
            1920,
            1536,
            4,
            Rotation {
                roll_rad: 0.0,
                pitch_rad: 0.0,
                yaw_rad: 0.012,
            },
        )
        .expect("built");
        let index = (192 * 480 + 240) * 2;
        assert_eq!(
            parallel.left.coords()[index],
            turned.left.coords()[index],
            "the left eye defines the rectified frame and must not move"
        );
        assert!(
            (parallel.right.coords()[index] - turned.right.coords()[index]).abs() > 1.0,
            "the right eye's map did not move with the configured yaw"
        );
    }

    fn stamped(seconds: f64) -> CompressedImage {
        let mut image = CompressedImage::default();
        image.header.stamp.sec = seconds.floor() as i32;
        image.header.stamp.nsec = ((seconds - seconds.floor()) * 1e9).round() as i32;
        image
    }

    /// The next left arrives before the right that belongs to the previous one.
    #[test]
    fn a_frame_waits_for_its_partner_instead_of_being_evicted() {
        let left: VecDeque<_> = [stamped(1.000), stamped(1.033)].into();
        let right: VecDeque<_> = [stamped(1.001)].into();
        assert_eq!(closest_pair(&left, &right, 0.05), Some((0, 0)));
    }

    #[test]
    fn the_closest_stamps_pair_not_the_newest_frames() {
        let left: VecDeque<_> = [stamped(1.000), stamped(1.033), stamped(1.066)].into();
        let right: VecDeque<_> = [stamped(1.030), stamped(1.070)].into();
        assert_eq!(closest_pair(&left, &right, 0.05), Some((1, 0)));
    }

    #[test]
    fn nothing_pairs_across_more_than_the_skew() {
        let left: VecDeque<_> = [stamped(1.000)].into();
        let right: VecDeque<_> = [stamped(1.100)].into();
        assert_eq!(closest_pair(&left, &right, 0.05), None);
        assert_eq!(closest_pair(&left, &right, 0.15), Some((0, 0)));
    }

    #[test]
    fn a_repeated_camera_info_is_not_a_new_calibration() {
        let a = info(640, 480, 500.0, 5);
        let mut b = info(640, 480, 500.0, 5);
        assert!(same_intrinsics(Some(&a), &b));
        assert!(!same_intrinsics(None, &b));
        b.K[0] += 1.0;
        assert!(!same_intrinsics(Some(&a), &b));
    }
}
