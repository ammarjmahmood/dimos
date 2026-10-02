window.BENCHMARK_DATA = {
  "lastUpdate": 1790956832404,
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
      }
    ]
  }
}
