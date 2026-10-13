;; Studio A4c proof-of-life module. Matches what the DSL compiler
;; emits for module.dsl:
;;
;;   import Core.LED.heartbeat(period_ms: int, on_ms: int)
;;   on start { Core.LED.heartbeat(1000, 2) }
;;
;; studio_start runs once at boot and starts the heartbeat: a 2 ms blip
;; once a second, timed by the SDK from SysTick (core_led_heartbeat). It
;; is set once, not from studio_loop: each call restarts the cycle, so a
;; call on every loop pass would hold the LED on. studio_loop is called
;; by the host in a tight outer loop and is empty here.

(module
  (import "env" "core_led_heartbeat" (func $heartbeat (param i32 i32)))
  (func (export "studio_start")
    (call $heartbeat (i32.const 1000) (i32.const 2)))
  (func (export "studio_loop")))
