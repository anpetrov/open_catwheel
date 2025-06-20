# SPDX-License-Identifier: Apache-2.0

board_runner_args(jlink "--device=nrf52840" "--speed=4000")
board_runner_args(pyocd "--target=nrf52840" "--frequency=8000000")
include(${ZEPHYR_BASE}/boards/common/nrfjprog.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/openocd-nrf5.board.cmake)
