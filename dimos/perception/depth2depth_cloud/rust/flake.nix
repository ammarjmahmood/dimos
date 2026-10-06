{
  description = "depth2depth_cloud native module for DimOS: the depth2depth crate behind an LCM wrapper";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    # Relative path: resolves against the flake, not the cwd (nix#12281). Only reachable when
    # entered as git+file:<dimos>?dir=..., which is how the module builds it.
    dimos-repo = { url = "path:../../../.."; flake = false; };
    crate2nix.url = "github:nix-community/crate2nix";
    crate2nix.inputs.nixpkgs.follows = "nixpkgs";
    # Same rev as Cargo.toml's depth2depth: its flake fetches the model the crate embeds (pinned in its model.json).
    depth2depth.url = "github:jeff-hykin/depth2depth/418511703eea73e1def3b984f0190fb56e286ff7";
    depth2depth.inputs.nixpkgs.follows = "nixpkgs";
  };

  outputs = { self, nixpkgs, flake-utils, dimos-repo, crate2nix, depth2depth }:
    flake-utils.lib.eachSystem [ "aarch64-darwin" "aarch64-linux" "x86_64-linux" ] (system:
      let
        isJetson = system == "aarch64-linux";
        pkgs = import nixpkgs {
          inherit system;
          # TensorRT from nixpkgs (the build sandbox can't see JetPack's). Its CVE flag is about
          # malicious engine files; this one only loads engines it built itself.
          config = nixpkgs.lib.optionalAttrs isJetson {
            allowUnfree = true;
            cudaCapabilities = [ "8.7" ];
            allowInsecurePredicate = pkg: nixpkgs.lib.hasPrefix "cuda12.6-tensorrt-" (pkg.name or "");
          };
        };

        src = pkgs.runCommand "depth2depth-cloud-src" {} ''
          mkdir -p $out/dimos/perception/depth2depth_cloud/rust
          cp -r ${./src} $out/dimos/perception/depth2depth_cloud/rust/src
          cp ${./Cargo.toml} $out/dimos/perception/depth2depth_cloud/rust/Cargo.toml
          cp ${./Cargo.lock} $out/dimos/perception/depth2depth_cloud/rust/Cargo.lock

          mkdir -p $out/native/rust
          cp -r ${dimos-repo}/native/rust/dimos-module $out/native/rust/dimos-module
          cp -r ${dimos-repo}/native/rust/dimos-module-macros $out/native/rust/dimos-module-macros
        '';

        generatedCargoNix = crate2nix.tools.${system}.generatedCargoNix {
          name = "depth2depth-cloud";
          inherit src;
          cargoToml = "dimos/perception/depth2depth_cloud/rust/Cargo.toml";
        };

        depth2depth-cloud = (import generatedCargoNix {
          inherit pkgs;
          buildRustCrateForPkgs = cratePkgs: cratePkgs.buildRustCrate.override {
            defaultCrateOverrides = cratePkgs.defaultCrateOverrides // {
              # Builds libjpeg-turbo from source (the `cmake` feature).
              turbojpeg-sys = attrs: { nativeBuildInputs = (attrs.nativeBuildInputs or []) ++ [ pkgs.cmake pkgs.nasm ]; };
              # TensorRT and cuDLA reference JetPack's driver libraries (libcuda, libnvdla_compiler), which the
              # sandbox lacks; they resolve at runtime from the host (see the wrapper below).
              dimos-depth2depth-cloud = attrs: pkgs.lib.optionalAttrs isJetson {
                extraRustcOpts = (attrs.extraRustcOpts or []) ++ [ "-C" "link-arg=-Wl,--allow-shlib-undefined" ];
              };
              # The model it embeds, and on a Jetson CUDA 12.6 + TensorRT.
              depth2depth = depth2depth.lib.crateOverride { inherit pkgs; cudaPackages = pkgs.cudaPackages_12_6; };
            };
          };
        }).rootCrate.build;
      in {
        # nix's glibc doesn't read ld.so.cache, so name JetPack's driver (libcuda) for it.
        packages.default = if !isJetson then depth2depth-cloud else pkgs.runCommand "depth2depth-cloud" {
          nativeBuildInputs = [ pkgs.makeWrapper ];
        } ''
          makeWrapper ${depth2depth-cloud}/bin/depth2depth_cloud $out/bin/depth2depth_cloud \
            --prefix LD_LIBRARY_PATH : /usr/lib/aarch64-linux-gnu/nvidia:/usr/lib/aarch64-linux-gnu/tegra
        '';
      });
}
