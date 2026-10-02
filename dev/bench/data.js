window.BENCHMARK_DATA = {
  "lastUpdate": 1790961070569,
  "repoUrl": "https://github.com/dimensionalOS/dimos",
  "entries": {
    "go2 replay realtime (arm64)": [
      {
        "commit": {
          "author": {
            "email": "git@sambull.org",
            "name": "Sam Bull",
            "username": "Dreamsorcerer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "51e97981bf339c029b8c1e179c845f714782185e",
          "message": "Use Otava for benchmark analysis (#4298)",
          "timestamp": "2026-10-02T16:57:32+01:00",
          "tree_id": "e7189f416bef7dd6b0820649feab088e70b6afb6",
          "url": "https://github.com/dimensionalOS/dimos/commit/51e97981bf339c029b8c1e179c845f714782185e"
        },
        "date": 1790956831103,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2027.047,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 394,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2737.508,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.352,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.259,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "bogwi@tutamail.com",
            "name": "Dan Vi",
            "username": "bogwi"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "d05c743aa98c7ebfec14988c7f0fec216e908215",
          "message": "fix mem concurrency (#4405)",
          "timestamp": "2026-10-02T16:22:10Z",
          "tree_id": "9437fca6ac8ba9597fa9ba6632a322a52d772f65",
          "url": "https://github.com/dimensionalOS/dimos/commit/d05c743aa98c7ebfec14988c7f0fec216e908215"
        },
        "date": 1790958305535,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2027.203,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 399,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2734.559,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.191,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.326,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 854/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "69774903+aclauer@users.noreply.github.com",
            "name": "Andrew Lauer",
            "username": "aclauer"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "d5b47cb56cf0af86d9ffe6de5b6a09fc57cc9fbe",
          "message": "fix: bump zenoh crate to 1.10.1 (#4386)",
          "timestamp": "2026-10-02T16:53:44Z",
          "tree_id": "b861fc4bb0ea22884dd9d370e7eeb9076e10efa0",
          "url": "https://github.com/dimensionalOS/dimos/commit/d5b47cb56cf0af86d9ffe6de5b6a09fc57cc9fbe"
        },
        "date": 1790960208283,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2000.301,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 403,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2737.952,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.176,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.23,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "hvent90@gmail.com",
            "name": "Henry Ventura",
            "username": "hvent90"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "e06d97554f54977b80ff0304f4e68a1eb8e5d79d",
          "message": "feat(agent): add planner-based go_to(x,y) skill (#4380)",
          "timestamp": "2026-10-02T10:08:08-07:00",
          "tree_id": "ffafcf0f77777fb795961dd3a4c0201cc30729bc",
          "url": "https://github.com/dimensionalOS/dimos/commit/e06d97554f54977b80ff0304f4e68a1eb8e5d79d"
        },
        "date": 1790961069377,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2025.395,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 402,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2731.812,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.18,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.652,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      }
    ]
  }
}