This part entailed flashing our own pre-compiled ELF image file onto the ESP32-C3 and examining it with gdb. To do that, first run the following commands:
```
$ idf.py set-target esp32c3
$ idf.py build
```
Then convert the ELF image to a binary executable by running `esptool.py --chip ESP32-C3 elf2image XXX.elf` (the --chip option is critical) and then place the binary into the `build/` folder as `lab2_1.bin`. Then run `idf.py flash monitor` to flash the binary onto the esp32 and monitor the results. You can then invoke gdb to examine the image by running `idf.py openocd &` in the background, which listens on port 3333 for gdb connections, which can be established by running `riscv32-esp-elf-gdb build/XXX.elf` and then running the following in the gdb console
```
(gdb) target remote :3333 # connect to the remote target
(gdb) set remote hardware-watchpoint-limit 2 # set limit of 2 hardware breakpoints
(gdb) monitor reset halt # reset the program being run
(gdb) flushregs # flush the register cache
(gdb) thb app_main # set temporary hardware breakpoint at method `app_main`
(gdb) continue # continue execution until breakpoint reached
```
