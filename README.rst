Build this with zephyr SDK using west.

1. Install tools required to build zephyr (https://github.com/zephyrproject-rtos/zephyr ):
   Instructions are here https://docs.zephyrproject.org/latest/getting_started/index.html
   You need zephyr sdk, ninja, west, python dependencies, ....
2. Prepare for building
     go to project dir and source env.sh
3. Build
     run "west build -d build --board=<your board>"