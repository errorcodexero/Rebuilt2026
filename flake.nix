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

      wpilibJdk =
        pkgs.runCommand "wpilib-jdk"
          {
            src = pkgs.fetchzip {
              url = "https://packages.wpilib.workers.dev/installer/v2026.1.1/Linux/WPILib_Linux-2026.1.1.tar.gz";
              hash = "sha256-lZNTm4X0ueLYnhBBZWGVL4gUjZApEEproS4vJiPJycw=";
            };
          }
          ''
            mkdir -p $out

            tar -xzf $src/WPILib_Linux-2026.1.1-artifacts.tar.gz jdk
            cp -r jdk/* $out/
          '';
    in
    {
      devShells.x86_64-linux.default = pkgs.mkShell {
        packages = [ wpilibJdk ];

        JAVA_HOME = wpilibJdk;
      };
    };
}
