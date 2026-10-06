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

//! `twist_smoother`: chases the latest input twist on its own clock, low-passed and
//! acceleration limited, so a 10 Hz staircase leaves as a ramp.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use dimos_module::{native_config, run_with_transport, Input, Module, Output};
use lcm_msgs::geometry_msgs::{Twist, Vector3};

#[native_config]
#[derive(Clone)]
struct Config {
    /// Output rate (Hz).
    #[validate(range(exclusive_min = 0.0))]
    rate_hz: f64,
    /// Smoothing strength: the low-pass time constant (s), roughly the time to cover 63% of a step. Zero passes through.
    #[validate(range(min = 0.0))]
    time_constant_s: f64,
    /// Per-axis acceleration caps (m/s², rad/s²), zero for none.
    #[validate(range(min = 0.0))]
    max_linear_accel: f64,
    #[validate(range(min = 0.0))]
    max_angular_accel: f64,
    /// Input silent this long (s) counts as a zero target.
    #[validate(range(exclusive_min = 0.0))]
    timeout_s: f64,
}

type V6 = [f64; 6];

struct Filter {
    time_constant_s: f64,
    max_accel: V6,
    out: V6,
}

impl Filter {
    fn new(c: &Config) -> Self {
        let (l, a) = (c.max_linear_accel, c.max_angular_accel);
        Self {
            time_constant_s: c.time_constant_s,
            max_accel: [l, l, l, a, a, a],
            out: [0.0; 6],
        }
    }

    fn step(&mut self, target: &V6, dt: f64) -> V6 {
        let alpha = if self.time_constant_s > 0.0 {
            1.0 - (-dt / self.time_constant_s).exp()
        } else {
            1.0
        };
        for i in 0..6 {
            let mut d = (target[i] - self.out[i]) * alpha;
            if self.max_accel[i] > 0.0 {
                let lim = self.max_accel[i] * dt;
                d = d.clamp(-lim, lim);
            }
            self.out[i] += d;
        }
        self.out
    }

    fn at_rest(&self) -> bool {
        self.out.iter().all(|v| v.abs() < 1e-3)
    }
}

fn to_v6(t: &Twist) -> V6 {
    [
        t.linear.x,
        t.linear.y,
        t.linear.z,
        t.angular.x,
        t.angular.y,
        t.angular.z,
    ]
}

fn to_twist(v: &V6) -> Twist {
    Twist {
        linear: Vector3 {
            x: v[0],
            y: v[1],
            z: v[2],
        },
        angular: Vector3 {
            x: v[3],
            y: v[4],
            z: v[5],
        },
    }
}

#[derive(Module)]
#[module(setup = spawn_worker, teardown = stop_worker)]
struct TwistSmoother {
    #[input(decode = Twist::decode, handler = on_twist)]
    cmd_vel_in: Input<Twist>,

    #[output(encode = Twist::encode)]
    cmd_vel_out: Output<Twist>,

    #[config]
    config: Config,

    latest: Arc<Mutex<Option<(V6, Instant)>>>,
    worker: Option<tokio::task::JoinHandle<()>>,
}

impl TwistSmoother {
    async fn on_twist(&mut self, msg: Twist) {
        *self.latest.lock().expect("latest mutex") = Some((to_v6(&msg), Instant::now()));
    }

    async fn spawn_worker(&mut self) {
        let latest = Arc::clone(&self.latest);
        let out = self.cmd_vel_out.clone();
        let c = self.config.clone();
        self.worker = Some(tokio::spawn(async move {
            let dt = 1.0 / c.rate_hz;
            let timeout = Duration::from_secs_f64(c.timeout_s);
            let mut filter = Filter::new(&c);
            let mut ticker = tokio::time::interval(Duration::from_secs_f64(dt));
            loop {
                ticker.tick().await;
                let target = match *latest.lock().expect("latest mutex") {
                    Some((t, at)) if at.elapsed() < timeout => Some(t),
                    _ => None,
                };
                // Idle once the input went quiet and the output has wound down,
                // so a downstream mux sees silence, not a stream of zeros.
                if target.is_none() && filter.at_rest() {
                    if filter.out != [0.0; 6] {
                        filter.out = [0.0; 6];
                        out.publish(&to_twist(&filter.out)).await.ok();
                    }
                    continue;
                }
                let v = filter.step(&target.unwrap_or([0.0; 6]), dt);
                out.publish(&to_twist(&v)).await.ok();
            }
        }));
    }

    async fn stop_worker(&mut self) {
        if let Some(handle) = self.worker.take() {
            handle.abort();
        }
        self.cmd_vel_out.publish(&to_twist(&[0.0; 6])).await.ok();
    }
}

#[tokio::main]
async fn main() {
    run_with_transport::<TwistSmoother>().await;
}

#[cfg(test)]
mod tests {
    use super::*;

    fn filter(time_constant_s: f64, lin: f64, ang: f64) -> Filter {
        Filter {
            time_constant_s,
            max_accel: [lin, lin, lin, ang, ang, ang],
            out: [0.0; 6],
        }
    }

    #[test]
    fn accel_cap_ramps_a_step() {
        let mut f = filter(0.0, 1.0, 0.0);
        let target = [1.0, 0.0, 0.0, 0.0, 0.0, 2.0];
        let v = f.step(&target, 0.1);
        assert!((v[0] - 0.1).abs() < 1e-9, "capped at 1 m/s² * 0.1 s");
        assert_eq!(v[5], 2.0, "angular uncapped");
        for _ in 0..20 {
            f.step(&target, 0.1);
        }
        assert!((f.out[0] - 1.0).abs() < 1e-9, "converges");
    }

    #[test]
    fn low_pass_takes_a_time_constant() {
        let mut f = filter(0.2, 0.0, 0.0);
        let target = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        for _ in 0..20 {
            f.step(&target, 0.01);
        }
        assert!(
            (f.out[0] - (1.0 - (-1.0f64).exp())).abs() < 1e-9,
            "63% after one tau"
        );
    }
}
