# SPDX-License-Identifier: Apache-2.0

board_runner_args(pyocd "--target=nrf52840")
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
