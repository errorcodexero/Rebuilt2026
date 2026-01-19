{
  description = "A very basic flake";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs?ref=nixos-unstable";
  };

  outputs =
    {
      self,
      nixpkgs,
    }:
    let
      system = "x86_64-linux";
      pkgs = nixpkgs.legacyPackages.${system};
      version = "2026.2.1";

      wpilibJdk =
        pkgs.runCommand "wpilib-jdk"
          {
            src = pkgs.fetchzip {
              url = "https://packages.wpilib.workers.dev/installer/v${version}/Linux/WPILib_Linux-${version}.tar.gz";
              hash = "sha256-yly96Zzj0jnvudZhwE30GqvK3Ny2Tqot84ZbPS0RrFM=";
            };
          }
          ''
            mkdir -p $out

            tar -xzf $src/WPILib_Linux-${version}-artifacts.tar.gz jdk
            cp -r jdk/* $out/
          '';
    in
    {
      devShells.x86_64-linux.default = pkgs.mkShell {
        name = "nix-simgui";

        packages = [ wpilibJdk pkgs.libGL ];

        JAVA_HOME = wpilibJdk;
      };
    };
}
