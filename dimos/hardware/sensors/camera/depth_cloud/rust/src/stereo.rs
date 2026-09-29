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

//! Dense depth from a stereo pair: rectify through each lens, then semi-global matching.

use rayon::prelude::*;
use std::f32::consts::PI;

/// Census window: 5x9 = 45 bits fits a u64, and wide suits a horizontal search.
const CENSUS_WIDTH: usize = 9;
const CENSUS_HEIGHT: usize = 5;
const CENSUS_HALF_W: usize = CENSUS_WIDTH / 2;
const CENSUS_HALF_H: usize = CENSUS_HEIGHT / 2;

/// Marks a pixel that failed a consistency check.
pub const INVALID_DISPARITY: f32 = f32::NAN;

/// A single-channel 8-bit image.
#[derive(Clone)]
pub struct Gray {
    pub width: usize,
    pub height: usize,
    pub data: Vec<u8>,
}

impl Gray {
    pub fn new(width: usize, height: usize) -> Self {
        Self {
            width,
            height,
            data: vec![0; width * height],
        }
    }

    #[inline]
    pub fn at(&self, x: usize, y: usize) -> u8 {
        self.data[y * self.width + x]
    }
}

/// Pinhole + distortion, as it arrives in a `CameraInfo`.
#[derive(Clone, Copy, Debug)]
pub struct Camera {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    /// `[k1, k2, p1, p2, k3, k4, k5, k6]`, zero-extended so a 5-coefficient plumb_bob works unchanged.
    pub distortion: [f64; 8],
}

impl Camera {
    /// Project a normalised ray through the 8-coefficient rational model (the coefficient count decides the model, not a `plumb_bob` label).
    pub fn distort(&self, x: f64, y: f64) -> (f64, f64) {
        let [k1, k2, p1, p2, k3, k4, k5, k6] = self.distortion;
        let r2 = x * x + y * y;
        let r4 = r2 * r2;
        let r6 = r4 * r2;
        let numerator = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
        let denominator = 1.0 + k4 * r2 + k5 * r4 + k6 * r6;
        // Past the denominator's root the ray would be mirrored through the principal point, so drop it.
        if denominator <= 1e-12 {
            return (f64::NAN, f64::NAN);
        }
        let radial = numerator / denominator;
        let x_tangential = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
        let y_tangential = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
        let xd = x * radial + x_tangential;
        let yd = y * radial + y_tangential;
        (self.fx * xd + self.cx, self.fy * yd + self.cy)
    }
}

/// One eye's roll-pitch-yaw relative to the rectified frame, applied exactly rather than linearised.
#[derive(Clone, Copy, Debug, Default)]
pub struct Rotation {
    pub roll_rad: f64,
    pub pitch_rad: f64,
    pub yaw_rad: f64,
}

impl Rotation {
    pub const IDENTITY: Self = Self {
        roll_rad: 0.0,
        pitch_rad: 0.0,
        yaw_rad: 0.0,
    };

    pub fn is_identity(&self) -> bool {
        self.roll_rad == 0.0 && self.pitch_rad == 0.0 && self.yaw_rad == 0.0
    }

    /// Rotate a rectified ray into the eye's frame (yaw, then pitch, then roll); `None` if it ends up behind the lens.
    pub fn apply(&self, x: f64, y: f64) -> Option<(f64, f64)> {
        if self.is_identity() {
            return Some((x, y));
        }
        let (sy, cy) = self.yaw_rad.sin_cos();
        let (sp, cp) = self.pitch_rad.sin_cos();
        let (sr, cr) = self.roll_rad.sin_cos();
        let (x1, y1, z1) = (cy * x + sy, y, -sy * x + cy);
        let (x2, y2, z2) = (x1, cp * y1 - sp * z1, sp * y1 + cp * z1);
        let (x3, y3, z3) = (cr * x2 - sr * y2, sr * x2 + cr * y2, z2);
        if z3 <= 1e-9 {
            return None;
        }
        Some((x3 / z3, y3 / z3))
    }
}

/// Per-output-pixel source coordinates in the distorted source image.
pub struct RectifyMap {
    pub width: usize,
    pub height: usize,
    /// Interleaved `(x, y)` source coordinates, `NaN` where unmapped.
    coords: Vec<f32>,
}

impl RectifyMap {
    /// Map a distorted source image to a pinhole image with intrinsics `fx, fy, cx, cy`.
    pub fn new(
        source: &Camera,
        width: usize,
        height: usize,
        fx: f64,
        fy: f64,
        cx: f64,
        cy: f64,
    ) -> Self {
        Self::rotated(source, width, height, fx, fy, cx, cy, Rotation::IDENTITY)
    }

    /// The same, with the eye rotated relative to the rectified frame; this is what turns two monocular calibrations into a stereo one.
    #[allow(clippy::too_many_arguments)]
    pub fn rotated(
        source: &Camera,
        width: usize,
        height: usize,
        fx: f64,
        fy: f64,
        cx: f64,
        cy: f64,
        rotation: Rotation,
    ) -> Self {
        let mut coords = vec![f32::NAN; width * height * 2];
        for row in 0..height {
            let y_rectified = (row as f64 - cy) / fy;
            for column in 0..width {
                let x_rectified = (column as f64 - cx) / fx;
                let Some((x_normalised, y_normalised)) = rotation.apply(x_rectified, y_rectified)
                else {
                    continue;
                };
                let (sx, sy) = source.distort(x_normalised, y_normalised);
                let index = (row * width + column) * 2;
                if sx.is_finite() && sy.is_finite() {
                    coords[index] = sx as f32;
                    coords[index + 1] = sy as f32;
                }
            }
        }
        Self {
            width,
            height,
            coords,
        }
    }

    /// The interleaved `(x, y)` source coordinates, for tests that check where a pixel is taken from.
    pub fn coords(&self) -> &[f32] {
        &self.coords
    }

    /// Bilinearly resample `source` through this map.
    pub fn apply(&self, source: &Gray) -> Gray {
        let mut out = Gray::new(self.width, self.height);
        let (sw, sh) = (source.width, source.height);
        out.data
            .par_chunks_mut(self.width)
            .enumerate()
            .for_each(|(row, line)| {
                for (column, pixel) in line.iter_mut().enumerate() {
                    let index = (row * self.width + column) * 2;
                    let sx = self.coords[index];
                    let sy = self.coords[index + 1];
                    if !sx.is_finite() || !sy.is_finite() {
                        continue;
                    }
                    // The bilinear tap needs x0+1 and y0+1 in bounds.
                    if sx < 0.0 || sy < 0.0 {
                        continue;
                    }
                    let x0 = sx.floor() as usize;
                    let y0 = sy.floor() as usize;
                    if x0 + 1 >= sw || y0 + 1 >= sh {
                        continue;
                    }
                    let tx = sx - x0 as f32;
                    let ty = sy - y0 as f32;
                    let base = y0 * sw + x0;
                    let p00 = source.data[base] as f32;
                    let p01 = source.data[base + 1] as f32;
                    let p10 = source.data[base + sw] as f32;
                    let p11 = source.data[base + sw + 1] as f32;
                    let top = p00 + (p01 - p00) * tx;
                    let bottom = p10 + (p11 - p10) * tx;
                    *pixel = (top + (bottom - top) * ty).round().clamp(0.0, 255.0) as u8;
                }
            });
        out
    }
}

/// Everything about a pair's geometry that does not change frame to frame.
pub struct StereoRectification {
    pub left: RectifyMap,
    pub right: RectifyMap,
    /// The intrinsics of the rectified image both maps produce.
    pub fx: f32,
    pub fy: f32,
    pub cx: f32,
    pub cy: f32,
    pub width: usize,
    pub height: usize,
}

/// Map a distorted, parallel pair onto one shared pinhole geometry, without which disparity does not mean `fx * baseline / Z`.
pub fn rectify_pair(
    left: &Camera,
    right: &Camera,
    width: usize,
    height: usize,
) -> StereoRectification {
    rectify_pair_rotated(left, right, width, height, Rotation::IDENTITY)
}

/// The same, with the right eye rotated relative to the left (a yaw is indistinguishable from a depth offset in the images).
pub fn rectify_pair_rotated(
    left: &Camera,
    right: &Camera,
    width: usize,
    height: usize,
    right_rotation: Rotation,
) -> StereoRectification {
    // Averaged so a disagreement between the two calibrations is split rather than landing on one.
    let fx = (left.fx + right.fx) / 2.0;
    let fy = (left.fy + right.fy) / 2.0;
    let cx = width as f64 / 2.0;
    let cy = height as f64 / 2.0;
    StereoRectification {
        left: RectifyMap::new(left, width, height, fx, fy, cx, cy),
        right: RectifyMap::rotated(right, width, height, fx, fy, cx, cy, right_rotation),
        fx: fx as f32,
        fy: fy as f32,
        cx: cx as f32,
        cy: cy as f32,
        width,
        height,
    }
}

/// Drop connected regions (neighbours within `max_step`) smaller than `min_region`, since floating blobs become phantom obstacles.
fn remove_speckle(
    disparity: &mut [f32],
    width: usize,
    height: usize,
    min_region: usize,
    max_step: f32,
) {
    if min_region <= 1 {
        return;
    }
    let mut label = vec![u32::MAX; width * height];
    let mut stack: Vec<usize> = Vec::new();
    let mut region: Vec<usize> = Vec::new();

    for start in 0..width * height {
        if label[start] != u32::MAX || !disparity[start].is_finite() {
            continue;
        }
        region.clear();
        stack.clear();
        stack.push(start);
        label[start] = 0;
        while let Some(index) = stack.pop() {
            region.push(index);
            let (x, y) = (index % width, index / width);
            let here = disparity[index];
            let visit = |nx: usize, ny: usize, stack: &mut Vec<usize>, label: &mut Vec<u32>| {
                let n = ny * width + nx;
                if label[n] != u32::MAX || !disparity[n].is_finite() {
                    return;
                }
                if (disparity[n] - here).abs() > max_step {
                    return;
                }
                label[n] = 0;
                stack.push(n);
            };
            if x > 0 {
                visit(x - 1, y, &mut stack, &mut label);
            }
            if x + 1 < width {
                visit(x + 1, y, &mut stack, &mut label);
            }
            if y > 0 {
                visit(x, y - 1, &mut stack, &mut label);
            }
            if y + 1 < height {
                visit(x, y + 1, &mut stack, &mut label);
            }
        }
        if region.len() < min_region {
            for &index in &region {
                disparity[index] = INVALID_DISPARITY;
            }
        }
    }
}

/// Knobs for the matcher.
#[derive(Clone, Copy, Debug)]
pub struct MatchParams {
    /// Smallest disparity searched, in pixels. Usually 0.
    pub min_disparity: usize,
    /// Number of disparities searched, starting at `min_disparity`.
    pub disparity_range: usize,
    /// SGM penalty for a one-step disparity change.
    pub p1: u16,
    /// SGM penalty for any larger jump. Must exceed `p1`.
    pub p2: u16,
    /// Winner must beat the runner-up (outside its neighbourhood) by this fraction; 0.0 disables.
    pub uniqueness: f32,
    /// Largest left/right disagreement tolerated, in pixels. Negative disables.
    pub max_lr_difference: f32,
    /// Connected regions smaller than this are dropped as speckle; 0 or 1 disables.
    pub min_region: usize,
    /// Largest neighbour disparity step still counted as one surface by the speckle fill.
    pub speckle_max_step: f32,

    /// Also aggregate along the diagonals: doubles aggregation cost, helps textureless surfaces like floors.
    pub diagonal_paths: bool,
}

impl Default for MatchParams {
    fn default() -> Self {
        Self {
            min_disparity: 0,
            disparity_range: 96,
            p1: 8,
            p2: 120,
            uniqueness: 0.10,
            max_lr_difference: 1.5,
            // ~0.2% of a 480x384 frame: clears blobs on textureless walls, keeps a chair leg.
            min_region: 350,
            speckle_max_step: 1.5,
            diagonal_paths: false,
        }
    }
}

/// Census-transform `image` into u64 codes; immune to per-camera gain and bias, unlike an intensity difference.
pub fn census(image: &Gray) -> Vec<u64> {
    let (width, height) = (image.width, image.height);
    let mut out = vec![0u64; width * height];
    out.par_chunks_mut(width)
        .enumerate()
        .for_each(|(row, line)| {
            if row < CENSUS_HALF_H || row + CENSUS_HALF_H >= height {
                return;
            }
            let last = width.saturating_sub(CENSUS_HALF_W);
            for (column, code) in line.iter_mut().enumerate().take(last).skip(CENSUS_HALF_W) {
                let centre = image.at(column, row);
                let mut bits = 0u64;
                for dy in 0..CENSUS_HEIGHT {
                    for dx in 0..CENSUS_WIDTH {
                        let y = row + dy - CENSUS_HALF_H;
                        let x = column + dx - CENSUS_HALF_W;
                        bits = (bits << 1) | u64::from(image.at(x, y) < centre);
                    }
                }
                *code = bits;
            }
        });
    out
}

/// Semi-global matching, returning sub-pixel disparity per left pixel (`NaN` where no confident match).
pub fn match_stereo(left: &Gray, right: &Gray, params: &MatchParams) -> Vec<f32> {
    assert_eq!(
        left.width, right.width,
        "rectified pair must agree in width"
    );
    assert_eq!(
        left.height, right.height,
        "rectified pair must agree in height"
    );
    let (width, height) = (left.width, left.height);
    let range = params.disparity_range.max(1);

    let left_census = census(left);
    let right_census = census(right);

    let costs = cost_volume(&left_census, &right_census, width, height, params);
    let aggregated = aggregate(&costs, width, height, range, params);
    let disparity = winner_take_all(&aggregated, width, height, range, params);

    let mut disparity = if params.max_lr_difference < 0.0 {
        disparity
    } else {
        let right_disparity = right_winner(&aggregated, width, height, range, params);
        consistency_filter(disparity, &right_disparity, width, height, params)
    };
    // Last, so it sees what survived the other filters.
    remove_speckle(
        &mut disparity,
        width,
        height,
        params.min_region,
        params.speckle_max_step,
    );
    disparity
}

/// Hamming distance between census codes for every (pixel, disparity).
fn cost_volume(
    left: &[u64],
    right: &[u64],
    width: usize,
    height: usize,
    params: &MatchParams,
) -> Vec<u16> {
    let range = params.disparity_range.max(1);
    let mut costs = vec![u16::MAX; width * height * range];
    costs
        .par_chunks_mut(width * range)
        .enumerate()
        .for_each(|(row, line)| {
            for column in 0..width {
                let left_code = left[row * width + column];
                for d in 0..range {
                    let shift = params.min_disparity + d;
                    if shift > column {
                        break;
                    }
                    let right_code = right[row * width + column - shift];
                    line[column * range + d] = (left_code ^ right_code).count_ones() as u16;
                }
            }
            let _ = height;
        });
    costs
}

/// Accumulate `costs` along each path direction and sum; paths of one direction are independent.
fn aggregate(
    costs: &[u16],
    width: usize,
    height: usize,
    range: usize,
    params: &MatchParams,
) -> Vec<u32> {
    let mut total = vec![0u32; width * height * range];
    let p1 = u32::from(params.p1);
    let p2 = u32::from(params.p2.max(params.p1 + 1));
    // Horizontal: one path per row, rows independent.
    for dx in [1i32, -1] {
        scan_rows(costs, &mut total, width, height, range, dx, p1, p2);
    }
    // Vertical: transpose so columns are contiguous and reuse the row scan (far faster than walking columns in place).
    if height > 0 && width > 0 {
        let mut turned = vec![0u16; width * height * range];
        transpose(costs, &mut turned, width, height, range);
        let mut turned_total = vec![0u32; width * height * range];
        for dy in [1i32, -1] {
            scan_rows(&turned, &mut turned_total, height, width, range, dy, p1, p2);
        }
        add_transposed(&turned_total, &mut total, width, height, range);
    }
    if params.diagonal_paths {
        for (dx, dy) in [(1i32, 1i32), (-1, -1), (1, -1), (-1, 1)] {
            scan_across_rows(costs, &mut total, width, height, range, dx, dy, p1, p2);
        }
    }
    total
}

/// Padding cost that never wins, halved so adding `p1` cannot wrap it into a cheap one.
const NEVER: u32 = u32::MAX / 2;

/// One SGM step; `previous` is `NEVER`-padded at both ends so the loop is branch-free and vectorises.
#[inline]
fn step(previous: &[u32], here: &[u16], current: &mut [u32], p1: u32, p2: u32) {
    let range = current.len();
    let minimum = previous[1..=range].iter().copied().min().unwrap_or(0);
    let jump = minimum + p2;
    for d in 0..range {
        let same = previous[d + 1];
        let lower = previous[d] + p1;
        let upper = previous[d + 2] + p1;
        let best = same.min(lower).min(upper).min(jump);
        current[d] = u32::from(here[d]) + best - minimum;
    }
}

/// A path's two buffers: `range + 2` wide, padded at both ends.
struct Guarded {
    previous: Vec<u32>,
    current: Vec<u32>,
}

impl Guarded {
    fn new(range: usize) -> Self {
        let mut previous = vec![NEVER; range + 2];
        let mut current = vec![NEVER; range + 2];
        previous[1..=range].fill(0);
        current[1..=range].fill(0);
        Self { previous, current }
    }

    /// Start a path: the first pixel's accumulator is its own cost.
    fn start(&mut self, here: &[u16]) {
        for (slot, cost) in self.current[1..=here.len()].iter_mut().zip(here) {
            *slot = u32::from(*cost);
        }
    }

    fn advance(&mut self, here: &[u16], p1: u32, p2: u32) {
        let range = here.len();
        step(&self.previous, here, &mut self.current[1..=range], p1, p2);
    }

    fn values(&self, range: usize) -> &[u32] {
        &self.current[1..=range]
    }

    fn swap(&mut self) {
        std::mem::swap(&mut self.previous, &mut self.current);
    }
}

/// Horizontal directions: each row is one path and owns its slice of `total`.
#[allow(clippy::too_many_arguments)]
fn scan_rows(
    costs: &[u16],
    total: &mut [u32],
    width: usize,
    height: usize,
    range: usize,
    dx: i32,
    p1: u32,
    p2: u32,
) {
    let _ = height;
    let stride = width * range;
    total
        .par_chunks_mut(stride)
        .enumerate()
        .for_each(|(row, accumulator)| {
            let costs_row = &costs[row * stride..row * stride + stride];
            // Two swapped buffers per row rather than an allocation per pixel.
            let mut buffers = Guarded::new(range);
            let mut first = true;
            for index in 0..width {
                let column = if dx > 0 { index } else { width - 1 - index };
                let at = column * range;
                let here = &costs_row[at..at + range];
                if first {
                    buffers.start(here);
                    first = false;
                } else {
                    buffers.advance(here, p1, p2);
                }
                for (slot, value) in accumulator[at..at + range]
                    .iter_mut()
                    .zip(buffers.values(range))
                {
                    *slot += *value;
                }
                buffers.swap();
            }
        });
}

/// Transpose the cost volume so image columns become contiguous for the row scan.
fn transpose(costs: &[u16], out: &mut [u16], width: usize, height: usize, range: usize) {
    out.par_chunks_mut(height * range)
        .enumerate()
        .for_each(|(column, line)| {
            for row in 0..height {
                let from = (row * width + column) * range;
                line[row * range..row * range + range].copy_from_slice(&costs[from..from + range]);
            }
        });
}

/// Add a turned accumulator back onto an upright one.
fn add_transposed(turned: &[u32], total: &mut [u32], width: usize, height: usize, range: usize) {
    total
        .par_chunks_mut(width * range)
        .enumerate()
        .for_each(|(row, line)| {
            for column in 0..width {
                let from = (column * height + row) * range;
                for (slot, value) in line[column * range..column * range + range]
                    .iter_mut()
                    .zip(&turned[from..from + range])
                {
                    *slot += *value;
                }
            }
        });
}

/// Diagonal directions: rows are walked in order and each row's width is parallelised.
#[allow(clippy::too_many_arguments)]
fn scan_across_rows(
    costs: &[u16],
    total: &mut [u32],
    width: usize,
    height: usize,
    range: usize,
    dx: i32,
    dy: i32,
    p1: u32,
    p2: u32,
) {
    let stride = width * range;
    // Row buffers are padded per pixel, as `step` expects.
    let guarded = range + 2;
    let padded = width * guarded;
    let mut previous_row = vec![NEVER; padded];
    let mut current_row = vec![NEVER; padded];
    let block = (width / rayon::current_num_threads().max(1)).max(16);
    let mut first_row = true;
    for index in 0..height {
        let row = if dy > 0 { index } else { height - 1 - index };
        let costs_row = &costs[row * stride..row * stride + stride];
        let previous = &previous_row;
        current_row
            .par_chunks_mut(block * guarded)
            .zip(total[row * stride..row * stride + stride].par_chunks_mut(block * range))
            .enumerate()
            .for_each(|(chunk, (current_block, accumulator))| {
                for (offset, current) in current_block.chunks_mut(guarded).enumerate() {
                    let column = chunk * block + offset;
                    let here = &costs_row[column * range..column * range + range];
                    let source = column as i32 - dx;
                    // A pixel whose predecessor is off the image starts the path at its raw cost.
                    if first_row || source < 0 || source >= width as i32 {
                        for (slot, cost) in current[1..=range].iter_mut().zip(here) {
                            *slot = u32::from(*cost);
                        }
                    } else {
                        let from = source as usize * guarded;
                        step(
                            &previous[from..from + guarded],
                            here,
                            &mut current[1..=range],
                            p1,
                            p2,
                        );
                    }
                    let at = offset * range;
                    for (slot, value) in accumulator[at..at + range]
                        .iter_mut()
                        .zip(&current[1..=range])
                    {
                        *slot += *value;
                    }
                }
            });
        std::mem::swap(&mut previous_row, &mut current_row);
        first_row = false;
    }
}

/// Pick the best disparity per left pixel, refine to sub-pixel, apply the uniqueness test.
fn winner_take_all(
    aggregated: &[u32],
    width: usize,
    height: usize,
    range: usize,
    params: &MatchParams,
) -> Vec<f32> {
    let mut out = vec![INVALID_DISPARITY; width * height];
    out.par_chunks_mut(width)
        .enumerate()
        .for_each(|(row, line)| {
            for (column, pixel) in line.iter_mut().enumerate() {
                let base = (row * width + column) * range;
                let window = &aggregated[base..base + range];
                let (best, &best_cost) =
                    match window.iter().enumerate().min_by_key(|&(_, cost)| *cost) {
                        Some(found) => found,
                        None => continue,
                    };
                if best_cost == 0 && window.iter().all(|&c| c == 0) {
                    // Never written (outside the census border).
                    continue;
                }
                if params.uniqueness > 0.0 {
                    let runner_up = window
                        .iter()
                        .enumerate()
                        .filter(|&(d, _)| d + 1 < best || d > best + 1)
                        .map(|(_, &cost)| cost)
                        .min();
                    if let Some(second) = runner_up {
                        // `+ 1.0` keeps the test meaningful when the winner scores 0, as on repeating patterns.
                        let threshold = best_cost as f32 * (1.0 + params.uniqueness) + 1.0;
                        if (second as f32) < threshold {
                            continue;
                        }
                    }
                }
                *pixel = params.min_disparity as f32 + best as f32 + subpixel(window, best);
            }
        });
    out
}

/// Equiangular sub-pixel fit in [-0.5, 0.5] px (a parabola biases towards integer disparities).
fn subpixel(window: &[u32], best: usize) -> f32 {
    if best == 0 || best + 1 >= window.len() {
        return 0.0;
    }
    let lower = window[best - 1] as f32;
    let centre = window[best] as f32;
    let upper = window[best + 1] as f32;
    let denominator = if upper > lower {
        upper - centre
    } else {
        lower - centre
    };
    if denominator.abs() < 1e-6 {
        return 0.0;
    }
    let offset = 0.5 * (lower - upper) / denominator;
    if offset.is_finite() {
        offset.clamp(-0.5, 0.5)
    } else {
        0.0
    }
}

/// Best disparity per *right* pixel, read out of the same aggregated volume.
fn right_winner(
    aggregated: &[u32],
    width: usize,
    height: usize,
    range: usize,
    params: &MatchParams,
) -> Vec<f32> {
    let mut out = vec![INVALID_DISPARITY; width * height];
    out.par_chunks_mut(width)
        .enumerate()
        .for_each(|(row, line)| {
            for (column, pixel) in line.iter_mut().enumerate() {
                let mut best: Option<(usize, u32)> = None;
                for d in 0..range {
                    let left_column = column + params.min_disparity + d;
                    if left_column >= width {
                        break;
                    }
                    let cost = aggregated[(row * width + left_column) * range + d];
                    if best.is_none_or(|(_, previous)| cost < previous) {
                        best = Some((d, cost));
                    }
                }
                if let Some((d, _)) = best {
                    *pixel = params.min_disparity as f32 + d as f32;
                }
            }
        });
    out
}

/// Drop left pixels whose right-image match disagrees, which removes occlusions.
fn consistency_filter(
    mut left: Vec<f32>,
    right: &[f32],
    width: usize,
    height: usize,
    params: &MatchParams,
) -> Vec<f32> {
    for row in 0..height {
        for column in 0..width {
            let index = row * width + column;
            let d = left[index];
            if !d.is_finite() {
                continue;
            }
            let right_column = column as f32 - d;
            if right_column < 0.0 {
                left[index] = INVALID_DISPARITY;
                continue;
            }
            let matched = right[row * width + right_column.round() as usize];
            if !matched.is_finite() || (matched - d).abs() > params.max_lr_difference {
                left[index] = INVALID_DISPARITY;
            }
        }
    }
    left
}

/// `Z = fx * baseline / disparity`, dropping disparities at or below `min_disparity_px` where depth error explodes.
pub fn disparity_to_depth(
    disparity: &[f32],
    fx: f32,
    baseline_m: f32,
    min_disparity_px: f32,
) -> Vec<f32> {
    let numerator = fx * baseline_m;
    disparity
        .iter()
        .map(|&d| {
            if d.is_finite() && d > min_disparity_px {
                numerator / d
            } else {
                f32::NAN
            }
        })
        .collect()
}

/// Convert packed RGB8/BGR8/RGBA8/BGRA8/mono8 bytes to grayscale.
pub fn to_gray(data: &[u8], width: usize, height: usize, channels: usize) -> Option<Gray> {
    if channels == 0 || data.len() < width * height * channels {
        return None;
    }
    let mut out = Gray::new(width, height);
    if channels == 1 {
        out.data.copy_from_slice(&data[..width * height]);
        return Some(out);
    }
    out.data
        .par_iter_mut()
        .enumerate()
        .for_each(|(index, pixel)| {
            let base = index * channels;
            let r = data[base] as f32;
            let g = data[base + 1] as f32;
            let b = data[base + 2] as f32;
            // Rec. 601 luma; RGB vs BGR only swaps the r and b weights, which census matching barely notices.
            *pixel = (0.299 * r + 0.587 * g + 0.114 * b).clamp(0.0, 255.0) as u8;
        });
    Some(out)
}

/// Box-downsample by an integer factor; point-sampling would alias the texture census depends on.
pub fn downsample(image: &Gray, factor: usize) -> Gray {
    let factor = factor.max(1);
    if factor == 1 {
        return image.clone();
    }
    let width = image.width / factor;
    let height = image.height / factor;
    let mut out = Gray::new(width, height);
    let area = (factor * factor) as u32;
    out.data
        .par_chunks_mut(width.max(1))
        .enumerate()
        .for_each(|(row, line)| {
            for (column, pixel) in line.iter_mut().enumerate() {
                let mut sum = 0u32;
                for dy in 0..factor {
                    let source_row = row * factor + dy;
                    let base = source_row * image.width + column * factor;
                    for dx in 0..factor {
                        sum += u32::from(image.data[base + dx]);
                    }
                }
                *pixel = (sum / area) as u8;
            }
        });
    out
}

/// Horizontal field of view of a pinhole camera in degrees, a quick sanity check on intrinsics vs resolution.
pub fn horizontal_fov(fx: f64, width: usize) -> f64 {
    2.0 * ((width as f64 / 2.0) / fx).atan() * 180.0 / PI as f64
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A real wide-angle 8-coefficient rational calibration.
    fn rational_lens_left() -> Camera {
        Camera {
            fx: 1012.5909853991253,
            fy: 1012.1256967046519,
            cx: 962.2128602781531,
            cy: 765.6731353273801,
            distortion: [
                -0.679226381939497,
                -0.6379916958825279,
                0.00017985045656597644,
                -0.0002169542130367524,
                -0.030239680416038896,
                -0.2867998467046839,
                -0.9987437961677965,
                -0.1752277996679942,
            ],
        }
    }

    fn pinhole() -> Camera {
        Camera {
            fx: 200.0,
            fy: 200.0,
            cx: 64.0,
            cy: 48.0,
            distortion: [0.0; 8],
        }
    }

    #[test]
    fn an_undistorted_camera_maps_the_principal_ray_to_the_principal_point() {
        let (x, y) = pinhole().distort(0.0, 0.0);
        assert!((x - 64.0).abs() < 1e-9);
        assert!((y - 48.0).abs() < 1e-9);
    }

    #[test]
    fn distortion_leaves_the_optical_axis_alone() {
        // Every term is multiplied by x, y, or r2, so the centre ray cannot move.
        let (x, y) = rational_lens_left().distort(0.0, 0.0);
        assert!((x - 962.2128602781531).abs() < 1e-9);
        assert!((y - 765.6731353273801).abs() < 1e-9);
    }

    #[test]
    fn the_last_three_coefficients_actually_change_the_answer() {
        // Guards against eight coefficients labelled plumb_bob being truncated to five.
        let full = rational_lens_left();
        let mut truncated = full;
        truncated.distortion[5] = 0.0;
        truncated.distortion[6] = 0.0;
        truncated.distortion[7] = 0.0;
        let (x_full, _) = full.distort(0.3, 0.2);
        let (x_truncated, _) = truncated.distort(0.3, 0.2);
        assert!(
            (x_full - x_truncated).abs() > 1.0,
            "k4..k6 moved the pixel by {} px, expected a visible difference",
            (x_full - x_truncated).abs()
        );
    }

    #[test]
    fn rectifying_an_undistorted_camera_is_the_identity() {
        let camera = pinhole();
        let map = RectifyMap::new(&camera, 128, 96, camera.fx, camera.fy, camera.cx, camera.cy);
        let mut source = Gray::new(128, 96);
        for (index, pixel) in source.data.iter_mut().enumerate() {
            *pixel = (index % 251) as u8;
        }
        let out = map.apply(&source);
        // Edges are dropped by the bilinear bounds check.
        for row in 1..95 {
            for column in 1..127 {
                assert_eq!(
                    out.at(column, row),
                    source.at(column, row),
                    "pixel ({column}, {row}) changed under an identity rectification"
                );
            }
        }
    }

    #[test]
    fn census_of_a_flat_image_is_all_zeroes() {
        let image = Gray {
            width: 32,
            height: 32,
            data: vec![128; 32 * 32],
        };
        assert!(census(&image).iter().all(|&code| code == 0));
    }

    #[test]
    fn census_is_blind_to_a_uniform_brightness_offset() {
        // The two cameras run their own exposure; census must not read that as depth.
        let mut a = Gray::new(32, 32);
        for (index, pixel) in a.data.iter_mut().enumerate() {
            *pixel = ((index * 7) % 100) as u8;
        }
        let mut b = a.clone();
        for pixel in b.data.iter_mut() {
            *pixel += 50;
        }
        assert_eq!(census(&a), census(&b));
    }

    /// Deterministic and aperiodic along a row, so a shifted pair has one right answer.
    fn texture(row: usize, column: usize) -> u8 {
        let mut hash = (row as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15)
            ^ (column as u64).wrapping_mul(0xC2B2_AE3D_27D4_EB4F);
        hash ^= hash >> 33;
        hash = hash.wrapping_mul(0xFF51_AFD7_ED55_8CCD);
        hash ^= hash >> 29;
        (hash & 0xFF) as u8
    }

    /// `right` is `left` shifted by `shift` px, as a fronto-parallel plane produces.
    fn shifted_pair(width: usize, height: usize, shift: usize) -> (Gray, Gray) {
        let mut left = Gray::new(width, height);
        for row in 0..height {
            for column in 0..width {
                left.data[row * width + column] = texture(row, column);
            }
        }
        let mut right = Gray::new(width, height);
        for row in 0..height {
            for column in 0..width {
                let source = column + shift;
                if source < width {
                    right.data[row * width + column] = left.data[row * width + source];
                }
            }
        }
        (left, right)
    }

    #[test]
    fn the_test_texture_is_not_accidentally_periodic() {
        // A periodic texture would make the matching tests ambiguous.
        let (width, height) = (96, 64);
        let mut image = Gray::new(width, height);
        for row in 0..height {
            for column in 0..width {
                image.data[row * width + column] = texture(row, column);
            }
        }
        let codes = census(&image);
        let (row, column) = (9usize, 32usize);
        let reference = codes[row * width + column];
        for d in 1..24usize {
            let other = codes[row * width + column - d];
            assert!(
                (reference ^ other).count_ones() > 4,
                "census code nearly repeats at offset {d}: a shifted-pair test would be ambiguous"
            );
        }
    }

    #[test]
    fn a_plane_shifted_by_a_known_amount_comes_back_at_that_disparity() {
        let shift = 7;
        let (left, right) = shifted_pair(96, 64, shift);
        let params = MatchParams {
            disparity_range: 24,
            uniqueness: 0.0,
            max_lr_difference: -1.0,
            ..Default::default()
        };
        let disparity = match_stereo(&left, &right, &params);

        // Skip the census border and the left edge, where the search runs off the right image.
        let mut checked = 0;
        for row in 8..56 {
            for column in 32..88 {
                let d = disparity[row * 96 + column];
                assert!(d.is_finite(), "no disparity at ({column}, {row})");
                assert!(
                    (d - shift as f32).abs() < 1.0,
                    "({column}, {row}) gave disparity {d}, expected {shift}"
                );
                checked += 1;
            }
        }
        assert!(checked > 1000, "only checked {checked} pixels");
    }

    #[test]
    fn an_identical_pair_reads_as_zero_disparity() {
        let (left, _) = shifted_pair(64, 48, 0);
        let params = MatchParams {
            disparity_range: 16,
            uniqueness: 0.0,
            max_lr_difference: -1.0,
            ..Default::default()
        };
        let disparity = match_stereo(&left, &left.clone(), &params);
        for row in 8..40 {
            for column in 20..60 {
                let d = disparity[row * 64 + column];
                assert!(d.is_finite());
                assert!(d.abs() < 0.6, "({column}, {row}) gave {d}, expected ~0");
            }
        }
    }

    #[test]
    fn depth_follows_the_stereo_formula() {
        // fx is rounded because f32 cannot hold more digits and clippy rejects writing them.
        let fx = 1012.591_f32;
        let baseline = 0.120195_f32;
        let depth = disparity_to_depth(&[30.4, 60.8, 0.0, f32::NAN], fx, baseline, 0.5);
        assert!((depth[0] - 4.0).abs() < 0.01, "got {}", depth[0]);
        assert!((depth[1] - 2.0).abs() < 0.01, "got {}", depth[1]);
        assert!(depth[2].is_nan(), "zero disparity must not become a point");
        assert!(depth[3].is_nan(), "NaN disparity must stay NaN");
    }

    #[test]
    fn depth_drops_disparities_below_the_floor() {
        let depth = disparity_to_depth(&[0.4, 0.6], 100.0, 0.1, 0.5);
        assert!(depth[0].is_nan());
        assert!(depth[1].is_finite());
    }

    #[test]
    fn downsampling_averages_rather_than_drops() {
        let image = Gray {
            width: 4,
            height: 2,
            data: vec![0, 100, 0, 100, 0, 100, 0, 100],
        };
        let small = downsample(&image, 2);
        assert_eq!(small.width, 2);
        assert_eq!(small.height, 1);
        // A point sample would give 0.
        assert_eq!(small.data, vec![50, 50]);
    }

    #[test]
    fn grayscale_rejects_a_buffer_that_is_too_short() {
        assert!(to_gray(&[0; 10], 4, 4, 3).is_none());
    }

    #[test]
    fn grayscale_passes_mono_through_untouched() {
        let mono = vec![1, 2, 3, 4];
        let gray = to_gray(&mono, 2, 2, 1).expect("mono8 should convert");
        assert_eq!(gray.data, mono);
    }

    #[test]
    fn a_wide_angle_field_of_view_is_sane() {
        // ~87 degrees across 1920 px at fx 1012.6.
        let fov = horizontal_fov(1012.5909853991253, 1920);
        assert!((fov - 87.0).abs() < 2.0, "got {fov} degrees");
    }

    #[test]
    fn speckle_removal_drops_a_floating_blob_and_keeps_the_surface() {
        // A blob off the true surface would be a phantom obstacle in a voxel map.
        let (width, height) = (64, 64);
        let mut disparity = vec![10.0f32; width * height];
        for row in 30..34 {
            for column in 30..34 {
                disparity[row * width + column] = 40.0;
            }
        }
        remove_speckle(&mut disparity, width, height, 350, 1.5);
        for row in 30..34 {
            for column in 30..34 {
                assert!(
                    disparity[row * width + column].is_nan(),
                    "the blob at ({column}, {row}) survived"
                );
            }
        }
        assert_eq!(disparity[0], 10.0);
        assert_eq!(disparity[width * height - 1], 10.0);
    }

    #[test]
    fn speckle_removal_keeps_a_small_but_connected_feature() {
        // A thin feature connected by a gradual disparity change is absorbed, not deleted.
        let (width, height) = (64, 64);
        let mut disparity = vec![10.0f32; width * height];
        for row in 20..60 {
            disparity[row * width + 32] = 11.0; // one step away, still connected
        }
        remove_speckle(&mut disparity, width, height, 350, 1.5);
        assert_eq!(disparity[40 * width + 32], 11.0);
    }

    #[test]
    fn speckle_removal_is_off_when_min_region_is_trivial() {
        let mut disparity = vec![5.0f32; 16];
        disparity[0] = 40.0;
        remove_speckle(&mut disparity, 4, 4, 1, 1.5);
        assert_eq!(disparity[0], 40.0);
    }

    #[test]
    fn a_whole_frame_of_one_surface_survives_speckle_removal() {
        // Guards against a min_region so large it eats everything.
        let (width, height) = (96, 64);
        let mut disparity = vec![7.0f32; width * height];
        remove_speckle(&mut disparity, width, height, 350, 1.5);
        assert!(disparity.iter().all(|d| *d == 7.0));
    }

    #[test]
    fn the_uniqueness_test_rejects_a_tie() {
        // Two equal disparities cannot be chosen between, so the pixel must be dropped.
        let range = 8;
        let mut aggregated = vec![50u32; range];
        aggregated[2] = 10;
        aggregated[6] = 10;
        let params = MatchParams {
            disparity_range: range,
            uniqueness: 0.10,
            max_lr_difference: -1.0,
            ..Default::default()
        };
        let disparity = winner_take_all(&aggregated, 1, 1, range, &params);
        assert!(
            disparity[0].is_nan(),
            "a tie between d=2 and d=6 was resolved to {}",
            disparity[0]
        );
    }

    #[test]
    fn the_uniqueness_test_keeps_a_clear_winner() {
        let range = 8;
        let mut aggregated = vec![500u32; range];
        aggregated[3] = 10;
        let params = MatchParams {
            disparity_range: range,
            uniqueness: 0.10,
            max_lr_difference: -1.0,
            ..Default::default()
        };
        let disparity = winner_take_all(&aggregated, 1, 1, range, &params);
        assert!((disparity[0] - 3.0).abs() < 0.6, "got {}", disparity[0]);
    }

    #[test]
    fn a_zero_cost_tie_is_still_rejected() {
        // A pure ratio test compares `0 < 0 * (1 + u)` and would keep the pixel.
        let range = 8;
        let aggregated = vec![0u32; range];
        let params = MatchParams {
            disparity_range: range,
            uniqueness: 0.10,
            max_lr_difference: -1.0,
            ..Default::default()
        };
        // An all-zero window means "never written", so make one entry non-zero.
        let mut real = aggregated.clone();
        real[7] = 3;
        let disparity = winner_take_all(&real, 1, 1, range, &params);
        assert!(
            disparity[0].is_nan(),
            "a zero-cost tie was resolved to {}",
            disparity[0]
        );
    }

    /// A real wide-angle rig's two rational-lens calibrations.
    fn rational_lens_pair() -> (Camera, Camera) {
        (
            Camera {
                fx: 1012.591,
                fy: 1012.1257,
                cx: 962.2129,
                cy: 765.6731,
                distortion: [
                    -0.679226, -0.637992, 0.00018, -0.000217, -0.03024, -0.2868, -0.998744,
                    -0.175228,
                ],
            },
            Camera {
                fx: 1013.7956,
                fy: 1013.3652,
                cx: 958.7372,
                cy: 768.2332,
                distortion: [
                    -0.251719, -0.427926, 5.8e-05, 6e-06, -0.02187, 0.140643, -0.620741, -0.123978,
                ],
            },
        )
    }

    #[test]
    fn both_eyes_land_on_one_shared_pinhole_geometry() {
        // Disparity only means `fx * baseline / Z` when the pair shares fx and principal point.
        let (left, right) = rational_lens_pair();
        let rectification = rectify_pair(&left, &right, 480, 384);
        assert!((rectification.cx - 240.0).abs() < 1e-6);
        assert!((rectification.cy - 192.0).abs() < 1e-6);
        let fx = f64::from(rectification.fx);
        assert!(
            fx > left.fx.min(right.fx) && fx < left.fx.max(right.fx),
            "fx was {fx}"
        );
    }

    #[test]
    fn each_eye_is_undistorted_through_its_own_lens() {
        // The lenses differ (k1 -0.68 vs -0.25); using one for both would read as depth error.
        let (left, right) = rational_lens_pair();
        let scale = 1.0 / 4.0;
        let scaled = |c: &Camera| Camera {
            fx: c.fx * scale,
            fy: c.fy * scale,
            cx: c.cx * scale,
            cy: c.cy * scale,
            distortion: c.distortion,
        };
        let rectification = rectify_pair(&scaled(&left), &scaled(&right), 480, 384);
        // A row near the bottom, where the lenses bend hardest.
        let index = (340 * 480 + 240) * 2;
        let left_x = rectification.left.coords[index];
        let right_x = rectification.right.coords[index];
        assert!(left_x.is_finite() && right_x.is_finite());
        assert!(
            (left_x - right_x).abs() > 0.05,
            "the two maps came out identical ({left_x} vs {right_x}), so one lens is being \
             used for both"
        );
    }

    #[test]
    fn rectifying_pulls_the_floor_row_a_long_way() {
        // Near the bottom of a wide-angle frame the lens moves a pixel by a large fraction of the image.
        let (left, _) = rational_lens_pair();
        let map = RectifyMap::new(&left, 1920, 1536, left.fx, left.fy, 960.0, 768.0);
        let index = (1400 * 1920 + 960) * 2;
        let source_y = map.coords[index + 1];
        assert!(source_y.is_finite());
        assert!(
            (f64::from(source_y) - 1400.0).abs() > 50.0,
            "the map moved the floor row by only {} pixels",
            f64::from(source_y) - 1400.0
        );
    }

    #[test]
    fn no_rotation_leaves_a_ray_exactly_where_it_was() {
        let (x, y) = Rotation::IDENTITY.apply(0.3, -0.2).expect("in front");
        assert_eq!((x, y), (0.3, -0.2));
    }

    #[test]
    fn a_yaw_shifts_every_disparity_by_the_same_amount() {
        // A relative yaw shifts the whole image sideways, which reads as a depth error.
        let yaw = Rotation {
            yaw_rad: 0.01,
            ..Rotation::IDENTITY
        };
        let near = yaw.apply(0.0, 0.0).expect("in front").0;
        let far = yaw.apply(0.4, 0.0).expect("in front").0 - 0.4;
        // Not exactly equal, but well within what the matcher can resolve.
        assert!((near - far).abs() < 2e-3, "{near} vs {far}");
        assert!(near > 0.0);
    }

    #[test]
    fn a_pitch_moves_the_image_off_its_row() {
        // A relative pitch puts the match off the searched row, losing recall.
        let pitch = Rotation {
            pitch_rad: 0.01,
            ..Rotation::IDENTITY
        };
        let (x, y) = pitch.apply(0.0, 0.0).expect("in front");
        assert!(x.abs() < 1e-12);
        assert!((y + 0.01).abs() < 1e-4, "y was {y}");
    }

    #[test]
    fn a_ray_rotated_past_the_horizon_is_dropped_not_wrapped() {
        // At z <= 0 the division by z would land the ray on the opposite side of the image.
        let sideways = Rotation {
            yaw_rad: std::f64::consts::FRAC_PI_2,
            ..Rotation::IDENTITY
        };
        assert!(sideways.apply(0.0, 0.0).is_none());
    }

    #[test]
    fn the_right_map_moves_with_the_rotation_and_the_left_does_not() {
        let (left, right) = rational_lens_pair();
        let plain = rectify_pair(&left, &right, 480, 384);
        let turned = rectify_pair_rotated(
            &left,
            &right,
            480,
            384,
            Rotation {
                yaw_rad: 0.012,
                ..Rotation::IDENTITY
            },
        );
        let index = (192 * 480 + 240) * 2;
        assert_eq!(plain.left.coords[index], turned.left.coords[index]);
        assert!(
            (plain.right.coords[index] - turned.right.coords[index]).abs() > 1.0,
            "a 0.012 rad yaw should move the right map by several pixels"
        );
    }

    #[test]
    fn every_pixel_is_reached_by_a_diagonal_pass() {
        // A wrong diagonal entry edge would leave a wedge of the image never aggregated.
        let params = MatchParams {
            diagonal_paths: true,
            disparity_range: 4,
            ..MatchParams::default()
        };
        let (width, height) = (9usize, 7usize);
        let costs = vec![3u16; width * height * params.disparity_range];
        let total = aggregate(&costs, width, height, params.disparity_range, &params);
        for row in 0..height {
            for column in 0..width {
                let value = total[(row * width + column) * params.disparity_range];
                assert!(
                    value > 0,
                    "({column}, {row}) was never aggregated over any of the eight paths"
                );
            }
        }
    }

    #[test]
    fn the_diagonals_are_off_unless_asked_for() {
        // They double the matcher's cost.
        assert!(!MatchParams::default().diagonal_paths);
        let (width, height) = (9usize, 7usize);
        let range = 4;
        let costs = vec![3u16; width * height * range];
        let four = aggregate(
            &costs,
            width,
            height,
            range,
            &MatchParams {
                disparity_range: range,
                ..MatchParams::default()
            },
        );
        let eight = aggregate(
            &costs,
            width,
            height,
            range,
            &MatchParams {
                disparity_range: range,
                diagonal_paths: true,
                ..MatchParams::default()
            },
        );
        let centre = ((height / 2) * width + width / 2) * range;
        assert!(eight[centre] > four[centre], "the diagonals added nothing");
    }
}

#[cfg(test)]
mod pole_tests {
    use super::*;

    /// A real wide-angle 8-coefficient rational calibration.
    fn rational_lens_left() -> Camera {
        Camera {
            fx: 1012.591,
            fy: 1012.126,
            cx: 962.213,
            cy: 765.673,
            distortion: [
                -0.679226, -0.637992, 0.000180, -0.000217, -0.030240, -0.286800, -0.998744,
                -0.175228,
            ],
        }
    }

    #[test]
    fn a_ray_past_the_denominators_root_is_refused_rather_than_mirrored() {
        let camera = rational_lens_left();
        // The denominator changes sign at r ~= 0.95 for this lens; beyond it the ray would be mirrored.
        let (inside_x, inside_y) = camera.distort(0.90, 0.0);
        assert!(inside_x.is_finite() && inside_y.is_finite());
        let (outside_x, outside_y) = camera.distort(1.05, 0.0);
        assert!(
            outside_x.is_nan() && outside_y.is_nan(),
            "a ray beyond the pole came back as ({outside_x}, {outside_y}) instead of being dropped"
        );
    }

    #[test]
    fn the_frames_corner_is_past_that_root_so_this_is_not_a_corner_case() {
        let camera = rational_lens_left();
        // The frame's corner sits at r ~= 1.21, outside the valid radius, so the guard is load-bearing.
        let corner = ((960.0f64 / camera.fx).powi(2) + (768.0f64 / camera.fy).powi(2)).sqrt();
        assert!(corner > 1.15, "corner at r = {corner}");
        let (x, y) = camera.distort(corner, 0.0);
        assert!(x.is_nan() && y.is_nan());
    }

    #[test]
    fn a_ray_down_the_axis_is_untouched() {
        let camera = rational_lens_left();
        let (x, y) = camera.distort(0.0, 0.0);
        assert!((x - camera.cx).abs() < 1e-9);
        assert!((y - camera.cy).abs() < 1e-9);
    }
}
