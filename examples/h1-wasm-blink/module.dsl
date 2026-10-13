# h1-wasm-blink - A4c proof-of-life DSL source.
#
# The compiled module is in module_wasm.h and in main.c (g_wasm_blob_flash),
# hand-assembled from module.wat (the dsl-to-wasm script these files used to
# name no longer exists). Studio compiles DSL to Wasm in the browser and
# flashes it with its own harness (web apps/studio/src/lib/wasmHarness.ts).
# After changing the module, update both byte arrays, then rebuild with: make.

import Core.LED.heartbeat(period_ms: int, on_ms: int)

# Set once: core_led_heartbeat runs from SysTick, and each call restarts
# the cycle, so calling it on every loop pass would hold the LED on.
on start {
  Core.LED.heartbeat(1000, 2)
}
