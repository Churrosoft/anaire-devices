esptool.exe --chip esp32 --port COM18 --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size detect 0xe000 boot_app0.bin 0x1000 bootloader_qio_80m.bin 0x10000 anaire.PiCO2.ino.esp32.bin
pause