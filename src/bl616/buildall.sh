#!/bin/bash

set -e

# Create buildall directory if it doesn't exist
mkdir -p buildall
rm -f buildall/*

# List of boards to build
boards=("m0sdock" "nano20k" "console60k" "mega60k" "mega138kpro" "primer25k")
# boards=("m0sdock" "nano20k" "console60k" "mega60k" "mega138kpro" "primer25k")

for b in "${boards[@]}"; do
    echo "Building for board: $b"

    # Clean previous build
    make clean

    # Set board environment variable and build
    export TANG_BOARD="$b"
    make

    if [ $? -eq 0 ]; then
        echo "Build successful for $b"

        # Copy and rename the binary files
        cp -f build/build_out/fpga_companion_bl616.bin "buildall/fpga_companion_${b}.bin"

        # Partner binaries depending on board
        if [ "$b" = "console60k" ]; then
            cp -f bl616_fpga_partner/bl616_fpga_partner_Console.bin "buildall/bl616_fpga_partner_${b}.bin"
        elif [ "$b" = "mega60k" ]; then
            cp -f bl616_fpga_partner/bl616_fpga_partner_NeoDock.bin "buildall/bl616_fpga_partner_${b}.bin"
        elif [ "$b" = "mega138kpro" ]; then
            cp -f bl616_fpga_partner/bl616_fpga_partner_138kproDock.bin "buildall/bl616_fpga_partner_${b}.bin"
        elif [ "$b" = "primer25k" ]; then
            cp -f bl616_fpga_partner/bl616_fpga_partner_25kDock.bin "buildall/bl616_fpga_partner_${b}.bin"
        elif [ "$b" = "nano20k" ]; then
            cp -f bl616_fpga_partner/bl616_fpga_partner_20kNano.bin "buildall/bl616_fpga_partner_${b}.bin"
        fi

        # Flash INI handling
        if [ "$b" = "m0sdock" ]; then
            cp -f flash_m0sdock_cfg.ini buildall/flash_m0sdock_cfg.ini
        else
            sed \
                -e "s/bl616_fpga_partner.bin/bl616_fpga_partner_${b}.bin/g" \
                -e "s/fpga_companion.bin/fpga_companion_${b}.bin/g" \
                flash.ini > "buildall/flash_${b}.ini"
        fi

    else
        echo "Build failed for $b"
    fi

done

echo
echo "Contents of buildall directory:"
ls -1 buildall/

