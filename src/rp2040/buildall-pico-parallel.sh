#!/bin/bash

set -e

mkdir -p buildall
rm -rf buildall/*

# Funktion zum Bauen eines Boards
build_target() {
    local board=$1
    local sdk_path=$2
    local output_name=$3
    local build_dir="build_${board}"

    # SDK-Pfade setzen
    export PICO_SDK_PATH="$sdk_path"
    export PICOTOOL_FETCH_FROM_GIT_PATH="$sdk_path"

    # Separates Build-Verzeichnis nutzen
    cmake -S . -B "$build_dir" -DBOARD="$board"
    
    # -j nutzt alle verfügbaren Kerne deines 16-Kerners für diesen Build
    cmake --build "$build_dir" -j$(nproc)
    
    # Ergebnis verschieben und temporären Build-Ordner löschen
    mv "${build_dir}/fpga_companion.uf2" "buildall/${output_name}.uf2"
    rm -rf "$build_dir"
}

# --- GRUPPE 1: pico-sdk (Läuft komplett parallel im Hintergrund) ---
build_target "MSP20K" "$HOME/pico-sdk" "fpga_companion_MSP20K" &
pid1=$!

build_target "PICO" "$HOME/pico-sdk" "fpga_companion_PICO" &
pid2=$!

build_target "WS2040ZERO" "$HOME/pico-sdk" "fpga_companion_WS2040ZERO" &
pid3=$!

# Warten, bis alle drei Builds der ersten Gruppe fertig sind
wait $pid1 $pid2 $pid3

# --- GRUPPE 2: pico2-sdk (Läuft parallel im Hintergrund) ---
build_target "MSP2_20K" "$HOME/pico2-sdk" "fpga_companion_MSP2_20K" &
pid4=$!

build_target "PICO2" "$HOME/pico2-sdk" "fpga_companion_PICO2" &
pid5=$!

# Warten, bis auch die zweite Gruppe fertig ist
wait $pid4 $pid5

echo "Contents of buildall directory:"
ls -1 buildall/
