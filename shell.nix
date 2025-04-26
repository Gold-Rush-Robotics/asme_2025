{
  nixpkgs ? import <nixpkgs> {},
}:

let
  pkgs = import (nixpkgs.fetchzip {
    name = "rospkgs";
    url = "https://github.com/lopsided98/nix-ros-overlay/archive/master.tar.gz";
    hash = "sha256-kXtMDUh/cCU05lnajBkD1qCuqnkKpMdD3B94P1P7UhE=";
  }) { nixpkgs = <nixpkgs>; };
in

pkgs.mkShell {
  name = "ASME 2025 Dev Shell";

  packages = with pkgs; [
    colcon
    (with rosPackages.jazzy; buildEnv {
      paths = [ ros-core ament-cmake-core python-cmake-module ];
    })
    python312
  ];
  shellHook = ''
    source .venv/bin/activate
    export PYTHONPATH=.venv/lib/python3.12/site-packages
    source install/setup.sh
  '';
}
