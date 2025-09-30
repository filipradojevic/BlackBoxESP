@echo off
REM Start OpenOCD for ESP32-S3 USB-JTAG

REM Postavi OpenOCD putanju
set OPENOCD_BIN=C:\Espressif\tools\openocd-esp32\v0.12.0-esp32-20250422\openocd-esp32\bin\openocd.exe

REM Postavi folder gde se nalaze konfiguracioni fajlovi
set CFG_PATH=%~dp0

REM Pokreni OpenOCD sa search path -s i konfiguracionim fajlom
"%OPENOCD_BIN%" -s "%CFG_PATH%" -f "esp32s3_usb_jtag.cfg"

pause
