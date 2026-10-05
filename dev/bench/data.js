window.BENCHMARK_DATA = {
  "lastUpdate": 1791229215890,
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
      },
      {
        "commit": {
          "author": {
            "email": "39084056+mustafab0@users.noreply.github.com",
            "name": "Mustafa Bhadsorawala",
            "username": "mustafab0"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5677b6d46c1e3c15a445793586afa1da39c88892",
          "message": "feat(control): operator hold task (#4402)",
          "timestamp": "2026-10-02T19:22:48Z",
          "tree_id": "d20d0cf28fb0c1b75ee67b18398a8f1f54b275db",
          "url": "https://github.com/dimensionalOS/dimos/commit/5677b6d46c1e3c15a445793586afa1da39c88892"
        },
        "date": 1790969138260,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2033.699,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 404,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2730.586,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.184,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.32,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "55869557+TomCC7@users.noreply.github.com",
            "name": "cc",
            "username": "TomCC7"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "943ce13c67e679e75452557f9457f1ba58541091",
          "message": "feat(memory): record explicit JSON streams with source timestamps (#4392)",
          "timestamp": "2026-10-02T15:12:04-07:00",
          "tree_id": "5a1d5b3f6095c1b6ceb917e361f488e4cb42a640",
          "url": "https://github.com/dimensionalOS/dimos/commit/943ce13c67e679e75452557f9457f1ba58541091"
        },
        "date": 1790979300466,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2035.031,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 397,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2729.585,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.184,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.136,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "39084056+mustafab0@users.noreply.github.com",
            "name": "Mustafa Bhadsorawala",
            "username": "mustafab0"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "e9a1ec0498a203187c6ea3e782fba83cb5779535",
          "message": "refactor(manipulation): skills report facts instead of error codes (#4401)",
          "timestamp": "2026-10-03T00:16:58-07:00",
          "tree_id": "258cd2a97575f3575fa0e5a9bda1dca0d5af4937",
          "url": "https://github.com/dimensionalOS/dimos/commit/e9a1ec0498a203187c6ea3e782fba83cb5779535"
        },
        "date": 1791011987648,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2035.688,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 400,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2739.751,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.43,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.652,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jeff.hykin@gmail.com",
            "name": "Jeff Hykin",
            "username": "jeff-hykin"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "60aee3c05e86ae15c9e38df960918414cf1904c1",
          "message": "r1pro: head cameras straight off V4L2, intrinsics from the factory calibration (#4388)\n\nCo-authored-by: Mustafa <mustafa@dimensionalos.com>",
          "timestamp": "2026-10-03T00:45:21-07:00",
          "tree_id": "5406c0e83bffd810cece15c9bba0687315ae65b0",
          "url": "https://github.com/dimensionalOS/dimos/commit/60aee3c05e86ae15c9e38df960918414cf1904c1"
        },
        "date": 1791013699144,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2024.738,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 403,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2729.595,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.16,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.151,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "63036454+ruthwikdasyam@users.noreply.github.com",
            "name": "ruthwikdasyam",
            "username": "ruthwikdasyam"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "db3d0ca9f31d726bfea3a78e8a57662292372d50",
          "message": "fix(sim): reject orphaned MuJoCo shared-memory buffers (#4337)",
          "timestamp": "2026-10-03T17:51:09-07:00",
          "tree_id": "91e640f27eba6853ab057c8c66ad6a529bf5263a",
          "url": "https://github.com/dimensionalOS/dimos/commit/db3d0ca9f31d726bfea3a78e8a57662292372d50"
        },
        "date": 1791075237313,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2016.594,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 392,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2737.46,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.141,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 149.056,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 855/855; perf counted 100.0%"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jeff.hykin@gmail.com",
            "name": "Jeff Hykin",
            "username": "jeff-hykin"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "c2189a6ea5f431bf6c9bd2111acc0914d375121d",
          "message": "r1pro: hardware-sync the two head cameras (FSYNC) (#4383)\n\nCo-authored-by: Mustafa <mustafa@dimensionalos.com>",
          "timestamp": "2026-10-05T12:27:10-07:00",
          "tree_id": "ce77c018b749a7b395221b4a3e0c2a7aa0c9a497",
          "url": "https://github.com/dimensionalOS/dimos/commit/c2189a6ea5f431bf6c9bd2111acc0914d375121d"
        },
        "date": 1791229214708,
        "tool": "customSmallerIsBetter",
        "benches": [
          {
            "name": "peak memory",
            "value": 2038.367,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "peak threads",
            "value": 398,
            "unit": "threads",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "network (transport)",
            "value": 2729.633,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "disk write",
            "value": 2.371,
            "unit": "MB",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          },
          {
            "name": "instructions",
            "value": 148.971,
            "unit": "G",
            "extra": "cpu: Neoverse-N2; delivered: odom 1122/1122, lidar 461/461, color_image 852/855; perf counted 100.0%"
          }
        ]
      }
    ]
  }
}