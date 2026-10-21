# Извлечь секцию .rodata из библиотеки
objcopy -O binary -j .rodata libps2000.so.2.0.0 rodata.bin

# Или извлечь конкретный массив по адресу (например, byte_F8B80)
dd if=libps2000.so.2.0.0 of=init_script.bin bs=1 skip=$((0xF8B80)) count=4096