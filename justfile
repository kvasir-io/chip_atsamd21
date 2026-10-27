# Upkeep helpers for the atsamd21 chip package.
#
# There is nothing to build here. This repository has no CMakeLists.txt at any
# level: cmake/chip.cmake is an include()-fragment that the Kvasir SDK pulls in
# through CHIP_ROOT (Kvasir_SDK/cmake/kvasir.cmake), and it references
# svd_convert() and kvasir_chip(), which only exist once kvasir.cmake has
# run. src/chip/*.hpp is hand-written; the peripheral headers are generated from
# chip.svd into the *consumer's* binary directory, never into this tree.
#
# So the way to find out whether this package still works is to build something
# that uses it:
#
#     cd <a firmware on this chip> && just chip_root=$PWD/../chip_atsamd21 build-all
#
# TWO PARTS: the ATSAMD21G17L (the default) and the ATSAMD21G18A, which has the
# USB. A project picks with set(KVASIR_ATSAMD21_MPU ATSAMD21G18A) before
# include(kvasir.cmake); cmake/variants/ has what differs (memory, linker script,
# SVD), src/chip/Variant.hpp what the headers ask. A firmware that sets
# KVASIR_ATSAMD21_MPU ATSAMD21G18A builds the G18A.
#
# The G18A's SVD is made at configure by scripts/make_g18a_svd.py, into the build tree (svd/README.md).
#
# What is here is upkeep: submodules, formatting, the container image.
#
# core/ (and core/'s src/cortex_common) are normally symlinks made by the
# workspace's link step; `git submodule status` refuses a symlinked path, which
# is why `just submodules` below reads .gitmodules.
#
#   just --list           # everything
#   just submodules       # after a clone, or when core/ looks empty
#   just format-check     # would clang-format change anything?
#   just docker           # build kvasirio/atsamd21:latest

set shell := ["bash", "-euo", "pipefail", "-c"]

# chip.just must never define root: source_directory() there is Kvasir_SDK/just, not this package
root := source_directory()
chip := "atsamd21"
# the --build-arg this repository's atsamd21-docker-build.yml passes to scripts/Dockerfile.chip
docker_args := ""
# the Kvasir_SDK checkout next to this one: the shared recipes, the Dockerfile and the lint scanner come from it
sdk := root / ".." / "Kvasir_SDK"

# What this package is, and where to build it from
default:
    @echo "chip_{{ chip }} - a CMake include()-fragment, consumed through CHIP_ROOT."
    @echo ""
    @echo "  Nothing builds here. To exercise this package:"
    @echo "      cd <a firmware on this chip> && just chip_root={{ root }} build-all"
    @[ -f "{{ sdk }}/just/chip.just" ] || echo "  (no {{ sdk }}/just/chip.just: check out Kvasir_SDK next to this repo for the other recipes)"
    @echo ""
    @just --justfile "{{ source_file() }}" --list --unsorted

# submodules, format-check, format, docker, lint
import? '../Kvasir_SDK/just/chip.just'
