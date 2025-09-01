@echo off
setlocal enabledelayedexpansion

REM Create buildall directory if it doesn't exist
if not exist buildall mkdir buildall
del /Q buildall\*

REM List of boards to build
set "boards=m0sdock nano20k console60k primer25k"
@REM set "boards=m0sdock nano20k console60k mega60k mega138kpro primer25k"

for %%b in (%boards%) do (
    echo Building for board: %%b
    
    REM Clean previous build
    make clean
    
    REM Set board environment variable and build
    set "TANG_BOARD=%%b"
    make ninja
    
    if !errorlevel! equ 0 (
        echo Build successful for %%b
        
        REM Copy and rename the binary files
        copy /Y build\build_out\fpga_companion_bl616.bin buildall\fpga_companion_%%b.bin
        if "%%b"=="console60k" (
            copy /Y bl616_fpga_partner\bl616_fpga_partner_Console.bin buildall\bl616_fpga_partner_%%b.bin
        ) else if "%%b"=="console138k" (
            copy /Y bl616_fpga_partner\bl616_fpga_partner_Console.bin buildall\bl616_fpga_partner_%%b.bin
        ) else if "%%b"=="mega60k" (
            copy /Y bl616_fpga_partner\bl616_fpga_partner_NeoDock.bin buildall\bl616_fpga_partner_%%b.bin
        ) else if "%%b"=="mega138kpro" (
            copy /Y bl616_fpga_partner\bl616_fpga_partner_138kproDock.bin buildall\bl616_fpga_partner_%%b.bin
        ) else if "%%b"=="primer25k" (
            copy /Y bl616_fpga_partner\bl616_fpga_partner_25kDock.bin buildall\bl616_fpga_partner_%%b.bin
        ) else if "%%b"=="nano20k" (
            copy /Y bl616_fpga_partner\bl616_fpga_partner_20kNano.bin buildall\bl616_fpga_partner_%%b.bin
            REM Copy unfused files
            copy /Y bl616_fpga_partner\friend_20k_bl616.bin buildall\friend_20k_bl616.bin
            copy /Y bl616_fpga_partner\friend_20k_cfg.ini buildall\friend_20k_cfg.ini
            copy /Y bl616_fpga_partner\flash_nano20k_unfused_cfg.ini buildall\flash_nano20k_unfused_cfg.ini
        )
        if "%%b"=="m0sdock" ( 
        copy /Y flash_m0sdock_cfg.ini buildall\flash_m0sdock_cfg.ini
        ) else (
        powershell -Command "(Get-Content flash.ini) -replace 'bl616_fpga_partner.bin', 'bl616_fpga_partner_%%b.bin' -replace 'fpga_companion.bin', 'fpga_companion_%%b.bin' | Set-Content buildall\flash_%%b.ini"
        )
    ) else (
        echo Build failed for %%b
    )
)

REM List contents of buildall directory
echo.
echo Contents of buildall directory:
dir /B buildall\

endlocal

