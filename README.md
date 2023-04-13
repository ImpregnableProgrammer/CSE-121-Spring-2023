# CSE-121-Spring-2023
Code repository for CSE 121 taken with Jose Renau in spring 2023

# Setup
To set up the esp32 build environment, run the following:
```
$ . esp-idf/export.sh
```
Then you can use the `idf.py` command in the project directory to set the target board, build the project, flash the board, and monitor its output by running the following:
```
$ idf.py set-target esp32c3
$ idf.py flash monitor
```
Learn more [here](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/linux-macos-setup.html) 
