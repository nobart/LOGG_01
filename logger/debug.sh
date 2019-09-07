cgdb -d /media/sf__VIRTUAL_/buildtools/gcc-arm-none-eabi-4_8-131228/bin/arm-none-eabi-gdb --command=init.gdb -iex "target remote 192.168.0.10:3333" -iex "set confirm off" -iex "file build/logger.elf"
