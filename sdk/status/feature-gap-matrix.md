# Cores SDK Feature Gap Matrix

> **Superseded (kept for history).** This April 2026 report is mostly stale and partly wrong
> (it claims SAI on the L4 and H5, and AES on the H5). The current findings are the portal page
> "Core.ST SDK audit and priority list, September 2026" (SDK project), and the live per-Core
> status is `core-*.json` in this folder.

> Generated 2026-04-06 from RM0377 (L0), RM0394 (L4), RM0481 (H5), RM0493 (WBA)
> Cross-referenced against `sdk/status/core-{l,u,h,w}.json`

## Legend

| Symbol | Meaning |
|--------|---------|
| **HW** | IC has the peripheral |
| **--** | IC does not have it (N/A) |
| V | SDK verified on hardware |
| C | Code compiles, not hw-tested |
| P | Partial (bugs / incomplete) |
| . | Not started (empty in status) |
| none | Explicitly marked not implemented |

**Usefulness** (general platform): 5 = broadly essential, 4 = very useful, 3 = useful for specific cases, 2 = niche, 1 = rarely needed
**Effort**: 1 = trivial (<1 day), 2 = small (1-3 days), 3 = medium (3-7 days), 4 = large (1-2 weeks), 5 = major (weeks+)

---

## 1. FEATURES ALREADY TRACKED — Current Gaps

These are features in `features.json` that exist on the hardware but are **not yet verified** (compile-only, partial, blank, or "none").

### 1a. Peripheral Communication — Highest Impact

| Feature | L | U | W | H | Useful | Effort | Notes |
|---------|---|---|---|---|--------|--------|-------|
| **SPI master (polling)** | -- | C | P | C | 5 | 3 | W: SPI v2 CSTART bug blocks all SPI tiles. U/H: compile but untested. Critical for memory tiles, flash, displays. |
| **SPI coregen auto-init** | -- | C | C | C | 5 | 2 | Needed before SPI tiles are usable in projects. I2C got full treatment; SPI needs same. |
| **UART IRQ-driven** | C | C | C | C | 4 | 2 | Compile on all. Needed for non-blocking serial (debug, GPS, RS-485 tiles). |
| **UART DMA** | . | . | . | . | 3 | 3 | Not started anywhere. Useful for high-throughput serial (GPS NMEA, sensor streams). |
| **LPUART** | C | C | C | C | 4 | 2 | Compile on all. Key for ultra-low-power wake-on-serial and sub-mA idle current. |
| **I2C IRQ-driven** | . | . | . | . | 4 | 3 | Not started. Needed for non-blocking tile communication; currently all I2C is polled. |
| **I2C DMA** | . | . | . | . | 3 | 3 | Not started. Useful for bulk reads (large sensor buffers, EEPROM). Driver arch has `TILE_FLAG_DMA_COMPLETE` reserved. |
| **I2C Fast-mode Plus** | -- | C | -- | C | 3 | 1 | Compile on U/H. Just needs 20mA GPIO + faster timing. Small win for high-speed tiles. |
| **I2C 10-bit addressing** | . | . | . | . | 1 | 1 | Not started. Very few I2C devices use 10-bit addresses. |
| **I2C SMBus** | . | . | . | . | 2 | 3 | Not started. Needed only for PMBus battery/power ICs. |
| **I2C slave mode** | . | . | . | . | 2 | 3 | Not started. Would let Core act as an I2C peripheral to another master. |
| **SPI IRQ-driven** | . | . | . | . | 3 | 3 | Not started. Needed for non-blocking SPI transfers. |
| **SPI DMA** | . | . | . | . | 4 | 3 | Not started. Essential for display tiles, large flash transfers. |
| **SPI slave mode** | . | . | . | . | 2 | 3 | Not started. Niche: Core as SPI peripheral. |
| **UART flow control** | . | . | . | . | 2 | 1 | Not started. CTS/RTS. Simple register config, useful for BT modules. |
| **UART coregen auto-init** | C | C | C | C | 4 | 2 | Compile on all. Needed for config.json-driven UART setup. |

### 1b. Analog

| Feature | L | U | W | H | Useful | Effort | Notes |
|---------|---|---|---|---|--------|--------|-------|
| **ADC analog watchdog** | . | . | . | . | 3 | 2 | Not started anywhere. Threshold-triggered interrupt without polling. Good for battery monitoring. |
| **ADC continuous+DMA** | C | V | V | C | 4 | 2 | Verified on U/W. H needs testing. L compile-only. Important for multi-channel sampling. |
| **DAC** | -- | -- | -- | V | 3 | -- | Verified on H only (the only core with DAC). Done for H. |
| **Comparator (COMP)** | C | C | . | -- | 3 | 2 | L/U have COMP1/COMP2, W has COMP1/2. H has none. Not started on W. Useful for voltage threshold wake, zero-crossing. |

### 1c. Timers

| Feature | L | U | W | H | Useful | Effort | Notes |
|---------|---|---|---|---|--------|--------|-------|
| **Timer one-shot** | . | . | . | . | 3 | 1 | Not started. Simple OPM bit. Useful for precise pulse generation (haptics, trigger signals). |
| **Timer encoder mode** | . | . | . | . | 2 | 2 | Not started. Quadrature decoder in hardware. Niche but zero-CPU for rotary encoders. |
| **LPTIM** | . | . | . | . | 4 | 3 | Not started anywhere. Runs in Stop mode. Essential for periodic low-power wakeup without RTC. |
| **IWDG (watchdog)** | V | V | C | V | 5 | 1 | Verified on L/U/H. W: compile only. Just needs hw test on W. |

### 1d. Power Management

| Feature | L | U | W | H | Useful | Effort | Notes |
|---------|---|---|---|---|--------|--------|-------|
| **Stop mode** | C* | V | C | V | 5 | 3 | L: hangs (RTC wakeup bug). W: compile only. U/H verified. L and W are the gaps. |
| **Standby** | C | V | C | C | 4 | 3 | U verified. H/W/L compile only. H needs testing (Stop 0 works, Standby not yet). |
| **EXTI wakeup** | C | V | . | C | 4 | 2 | U verified. H/L compile. W not started. GPIO-edge wake from Stop. |
| **Wakeup pin (Standby)** | . | V | . | . | 3 | 2 | U verified. Dedicated WKUP pins for Standby wake. |

### 1e. Connectivity

| Feature | L | U | W | H | Useful | Effort | Notes |
|---------|---|---|---|---|--------|--------|-------|
| **USB HID** | -- | . | -- | V | 3 | 2 | H verified. U has USB but HID not implemented. Useful for custom input/sensor devices. |
| **USB MSC** | -- | . | -- | . | 3 | 4 | Not started on U or H. Drag-and-drop firmware, datalogging. Needs SCSI/FAT layer. |
| **BLE (full stack)** | -- | -- | V* | -- | 5 | 5 | W: basic BLE verified (adv, GATT) but stashed on `pre-ble` branch. Needs ST middleware integration. |
| **FDCAN** | -- | -- | -- | . | 2 | 4 | H has FDCAN1/2. Not started. Industrial/automotive use case. |
| **I3C controller** | -- | -- | -- | . | 3 | 4 | H has I3C1 (+ I3C2). Not started. Next-gen sensor bus, backwards-compatible with I2C. |
| **I3C target** | -- | -- | -- | . | 2 | 3 | H only. Core as I3C secondary device. |

### 1f. Security

| Feature | L | U | W | H | Useful | Effort | Notes |
|---------|---|---|---|---|--------|--------|-------|
| **Hardware RNG** | -- | V | V | HW | 4 | 2 | W+U verified. H has it but marked "none". U: HSI48+CCIPR mux, 6/6 tests pass (tests/val-backup-rng-alarm-u). |
| **AES accelerator** | HW* | HW | V | HW | 4 | 3 | W verified. L has AES-128 only. U has AES-128/256. H has AES+SAES. High value for secure comms. |
| **PKA/ECC** | -- | -- | HW | HW* | 3 | 4 | W has full PKA (none status). H has limited PKA (ECDSA verify only). For secure BLE pairing, cert validation. |
| **HASH (SHA-256)** | -- | -- | -- | HW | 3 | 2 | H only (also W has HASH per RM but marked na in status?). Firmware integrity, HMAC. |

---

## 2. FEATURES NOT YET IN features.json — Hardware Available But Untracked

These are IC capabilities discovered in the reference manuals that don't appear in the current feature tracking at all.

### 2a. High Value (Usefulness 4-5)

| Feature | L | U | W | H | Useful | Effort | Notes |
|---------|---|---|---|---|--------|--------|-------|
| **GPDMA linked-list mode** | -- | -- | HW | HW | 5 | 4 | W/H have advanced GPDMA with linked-list chaining, triggers, and autonomous mode in Stop. Enables complex DMA sequences without CPU. Current DMA is basic channel mode. |
| **DMA autonomous mode (Stop)** | -- | -- | HW | HW | 4 | 3 | W/H GPDMA can run transfers in Stop mode with trigger wakeup. Key for ultra-low-power sensor polling. |
| **Peripheral autonomous mode** | -- | -- | HW | -- | 4 | 3 | W: ADC4, SPI1/3, I2C1, USART1/2, LPUART1, LPTIM1/2 can operate autonomously in Stop modes. Game-changer for low-power sensing. |
| **OctoSPI / QUADSPI** | -- | HW | -- | HW | 4 | 4 | U: QUADSPI. H: OCTOSPI1. Memory-mapped external flash. For NOR flash tiles, PSRAM. Not tracked at all. |
| **RTC alarms (A+B)** | HW | HW | HW | HW | 4 | 2 | All cores have dual alarms. Currently only wakeup timer is used. Alarms allow calendar-based scheduling (wake at specific time-of-day). |
| **RTC timestamp** | HW | HW | HW | HW | 3 | 1 | Hardware-captured time on external event. Zero overhead event timestamping. |
| **Flash ECC / error detection** | -- | HW | HW | HW | 4 | 2 | U/W/H have flash ECC with reporting. Important for data integrity in field-deployed devices. Not tracked. |
| **ICACHE management** | -- | -- | HW | HW | 4 | 2 | W: 8KB, H: 16KB instruction cache. Currently not explicitly managed. Proper invalidation after flash writes, performance monitoring. |
| **TIM1/TIM8 advanced features** | -- | HW | HW | HW | 4 | 3 | Complementary PWM with dead-time, break inputs (fault protection), repetition counter, hall sensor interface. U has TIM1, W has TIM1, H has TIM1+TIM8. Key for motor control, H-bridge, power converter tiles. |

### 2b. Medium Value (Usefulness 3)

| Feature | L | U | W | H | Useful | Effort | Notes |
|---------|---|---|---|---|--------|--------|-------|
| **USB Host mode** | -- | -- | -- | HW | 3 | 4 | H has USB DRD (Device+Host). Currently only Device. Host would allow connecting USB peripherals (keyboards, flash drives). |
| **USB BCD (battery charging)** | -- | HW | -- | HW | 3 | 1 | U/H USB peripherals support BC 1.2 detection. Simple register config to detect charger type. |
| **USB LPM (link power mgmt)** | -- | HW | -- | HW | 3 | 2 | USB suspend/resume with L1 sleep. Reduces USB idle power. |
| **SDMMC** | -- | -- | -- | HW | 3 | 4 | H has SDMMC1. SD card interface for datalogging. Needs FAT filesystem. |
| **SAI (serial audio)** | -- | -- | HW | -- | 3 | 3 | W has SAI with I2S/TDM/PDM. Digital microphone interface, audio output. |
| **I2S mode (via SPI)** | HW | HW | -- | HW | 3 | 3 | L: SPI1 has I2S. U: SPI1/2 have I2S. H: SPI1-3 have I2S. Audio data streaming via SPI peripheral. |
| **WWDG (window watchdog)** | HW | HW | HW | HW | 3 | 1 | All cores. More precise timing enforcement than IWDG. Simple to add alongside existing IWDG. |
| **CRC hardware** | HW | HW | HW | HW | 3 | 1 | All cores. Programmable polynomial CRC. Useful for protocol checksums, flash integrity. Trivial to implement. |
| **PVD (voltage detector)** | HW | HW | -- | -- | 3 | 1 | L/U have programmable voltage detector. Brown-out warning before reset. Good for battery-powered tiles. |
| **ADC injected channels** | -- | HW | -- | HW | 3 | 3 | U/H only. Priority ADC conversion that preempts regular sequence. For time-critical measurements (motor control). |
| **ADC dual mode** | -- | HW | -- | HW | 3 | 3 | U has ADC1+ADC2, H has ADC1+ADC2. Simultaneous/interleaved sampling. Double throughput. |
| **DAC noise/triangle gen** | -- | -- | -- | HW | 3 | 1 | H DAC can generate noise and triangle waveforms in hardware. Signal injection, test tones. |
| **DAC sample-and-hold** | -- | -- | -- | HW | 3 | 2 | H DAC operates in Stop mode using LSI/LSE. Hold analog output while sleeping. |
| **RTC calibration** | HW | HW | HW | HW | 3 | 2 | All cores. Digital smooth calibration (~0.95 ppm). Improves RTC accuracy in field. |
| **COMP window mode** | -- | -- | HW | -- | 3 | 2 | W: COMP1+COMP2 combined as window comparator. Voltage-in-range detection. |
| **Low-power run mode** | HW | HW | -- | -- | 3 | 3 | L/U can run at ultra-low clock (<2MHz) with low-power regulator. Sub-100uA active current. |
| **Shutdown mode** | -- | HW | -- | -- | 3 | 2 | U: lowest power mode (even LSI off, only LSE). Lower than Standby. |
| **HSEM (HW semaphore)** | -- | -- | HW | -- | 3 | 2 | W: 16 semaphores for CPU/radio synchronization. Important for proper BLE coexistence. |
| **Tamper detection** | HW | HW | HW | HW | 3 | 3 | All cores. External tamper pins + backup register erase. Physical security for sensitive keys. |
| **Backup registers** | HW | HW | HW | HW | 3 | 1 | All cores (5-32 registers). Persist through reset and Standby. User data, boot flags, crash counters. |
| **OPAMP** | -- | HW | -- | -- | 3 | 2 | U has OPAMP1 (PGA mode, follower, standalone). Signal conditioning without external parts. |
| **TSC (touch sensing)** | -- | HW | HW | -- | 3 | 3 | U/W have touch sensing controller. Capacitive button/slider tiles. |

### 2c. Lower Value (Usefulness 1-2)

| Feature | L | U | W | H | Useful | Effort | Notes |
|---------|---|---|---|---|--------|--------|-------|
| **DCMI / PSSI** | -- | -- | -- | HW | 2 | 4 | H: Digital camera / parallel slave interface. Niche. |
| **HDMI-CEC** | -- | -- | -- | HW | 1 | 2 | H: Consumer electronics control. Very niche. |
| **FMC (ext. memory)** | -- | -- | -- | HW | 2 | 4 | H: NOR/PSRAM/NAND/SDRAM controller. For external RAM/storage. |
| **DTS (digital temp sensor)** | -- | -- | -- | HW | 2 | 1 | H: Separate from ADC temp. Has watchdog thresholds. |
| **VREFBUF** | -- | HW | -- | HW | 2 | 1 | U/H: Internal precision voltage reference buffer. Improves ADC accuracy. |
| **IRTIM (infrared)** | -- | HW | HW | -- | 1 | 1 | U/W: IR modulation via timer interconnect. Remote control. |
| **Firewall** | -- | HW | -- | -- | 2 | 3 | U: Code region isolation. Superseded by TrustZone on M33 cores. |
| **Bit-banding** | -- | HW | -- | -- | 1 | 1 | U: Cortex-M4 atomic bit manipulation. Minor convenience. |
| **Data EEPROM** | HW | -- | -- | -- | 2 | -- | L only. Already verified. |
| **SWO trace** | -- | HW | -- | HW | 2 | 2 | U/H: ITM trace output. Debug aid, not runtime feature. |
| **802.15.4 (Thread/Zigbee)** | -- | -- | HW | -- | 2 | 5 | W: Full 802.15.4 MAC. Major effort, niche use case for tiles. |
| **BLE AoA/AoD** | -- | -- | HW | -- | 2 | 4 | W: Direction finding. Specialized indoor positioning. |
| **BLE isochronous (audio)** | -- | -- | HW | -- | 2 | 5 | W: LE Audio CIS/BIS. Major effort for audio streaming. |
| **Active tamper** | -- | -- | HW | HW | 1 | 3 | W/H: Active mesh tamper detection. Physical security. |
| **SAES (side-channel resistant)** | -- | -- | HW | HW | 2 | 3 | W/H: DPA-resistant AES. For certified security products. |
| **TrustZone configuration** | -- | -- | HW | HW | 2 | 4 | W/H: Full GTZC setup. For secure/nonsecure partitioning. |
| **MPU configuration** | -- | HW | HW | HW | 2 | 3 | U/W/H: Memory protection regions. For robustness, RTOS prep. |

---

## 3. PRIORITIZED IMPLEMENTATION RECOMMENDATIONS

### Tier 1 — High Impact, Reasonable Effort (Do Next)

| # | Feature | Cores | Effort | Why |
|---|---------|-------|--------|-----|
| 1 | **SPI master (fix W, test U/H)** | W,U,H | 3 | Blocks ALL SPI-only tiles. W's SPI v2 CSTART bug is the #1 blocker. |
| 2 | **SPI coregen auto-init** | U,W,H | 2 | Prerequisite for SPI tiles in config.json. |
| 3 | **LPTIM** | all | 3 | Runs in Stop mode. Enables periodic low-power wakeup without RTC complexity. |
| 4 | **UART IRQ + coregen** | all | 2+2 | Non-blocking serial. Compile-only everywhere; just needs testing and coregen hookup. |
| 5 | **Hardware RNG** | ~~U~~,H | 2 | **U done.** 6/6 tests pass (tests/val-backup-rng-alarm-u). H still needs testing. |
| 6 | **Stop mode (L, W)** | L,W | 3 | L: debug RTC wakeup hang. W: needs hw test. Verified on U/H. |
| 7 | **RTC alarms** | all | 2 | API + LL implemented. U: time set/get verified, but ALRMAR writes read back 0 — hw investigation needed. L4 alarm flag bug fixed (was reading TSTR instead of ICSR). |
| 8 | **Backup registers** | all | 1 | API implemented. U: BKPxR writes read back 0 — same root cause as alarm (hw investigation). L0/L4 clock-enable bug fixed in core_backup.h. |

### Tier 2 — Valuable, Moderate Effort

| # | Feature | Cores | Effort | Why |
|---|---------|-------|--------|-----|
| 9 | **I2C IRQ-driven** | all | 3 | Non-blocking tile communication. Foundation for real-time multi-tile systems. |
| 10 | **SPI DMA** | U,W,H | 3 | Display tiles, flash bulk transfers. Needs I2C DMA architecture alignment. |
| 11 | **LPUART (low-power UART)** | all | 2 | Wake-on-serial from Stop. Compile-only everywhere, needs hw verification. |
| 12 | **CRC hardware** | all | 1 | Trivial. Protocol checksums, flash integrity verification. |
| 13 | **WWDG** | all | 1 | Window watchdog alongside existing IWDG. Better timing enforcement. |
| 14 | **AES accelerator** | U,H | 3 | Secure comms. W done. U has AES-128/256, H has AES+SAES. |
| 15 | **QUADSPI / OctoSPI** | U,H | 4 | Memory-mapped external flash. Unlocks NOR flash and PSRAM tiles. |
| 16 | **TIM1 advanced (complementary PWM)** | U,W,H | 3 | Motor control, H-bridge driving, power converter tiles. |
| 17 | **Comparators** | L,U,W | 2 | Hardware voltage threshold detection. Wake from Stop on voltage event. |
| 18 | **Flash ECC reporting** | U,W,H | 2 | Field reliability. Detect flash bit errors before they cause crashes. |

### Tier 3 — Specialized / Large Effort

| # | Feature | Cores | Effort | Why |
|---|---------|-------|--------|-----|
| 19 | **GPDMA linked-list** | W,H | 4 | Complex DMA sequences. High value but significant arch work. |
| 20 | **Peripheral autonomous mode** | W | 3 | Sensor polling in Stop mode. Depends on GPDMA work. |
| 21 | **FDCAN** | H | 4 | Industrial bus. Niche but H has two instances. |
| 22 | **I3C controller** | H | 4 | Next-gen sensor bus. Forward-looking but few I3C sensors exist today. |
| 23 | **USB MSC** | U,H | 4 | USB mass storage. Needs FAT/SCSI layer. |
| 24 | **USB Host** | H | 4 | USB host mode. Connect peripherals to Core.ST.H5. |
| 25 | **SDMMC** | H | 4 | SD card. Needs FAT filesystem. Good for datalogging. |
| 26 | **SAI / I2S** | W,U,H | 3 | Audio interface. Microphone/speaker tiles. |
| 27 | **PKA/ECC** | W,H | 4 | Secure BLE pairing, certificate validation. |
| 28 | **HASH (SHA-256)** | H,(W?) | 2 | Firmware integrity, HMAC. Simple if only wrapping register access. |
| 29 | **TSC (touch sensing)** | U,W | 3 | Capacitive touch tiles. |
| 30 | **TrustZone / MPU** | W,H | 4 | Security partitioning. Long-term platform hardening. |
| 31 | **802.15.4 / Thread** | W | 5 | Zigbee/Thread networking. Major effort. |

---

## 4. QUICK REFERENCE: IC PERIPHERAL AVAILABILITY

| Peripheral | L011E4 | L422TB | WBA55HGF6 | H523HE |
|-----------|--------|--------|-----------|--------|
| **Core** | M0+ 32MHz | M4+FPU 80MHz | M33+TZ 100MHz | M33+TZ 250MHz |
| **Flash** | 16KB | 128KB | 1MB | 512KB |
| **SRAM** | 2KB | 40KB | 128KB | 274KB |
| **DMA** | 5ch basic | 14ch (DMA1+2) | 8ch GPDMA | 16ch GPDMA1+2 |
| **ADC** | 1x 12-bit | 2x 12-bit | 1x 12-bit (ADC4) | 2x 12-bit |
| **DAC** | -- | -- | -- | 2-ch |
| **COMP** | COMP1+2 | COMP1 | COMP1+2 | -- |
| **OPAMP** | -- | OPAMP1 | -- | -- |
| **Timers** | TIM2,21,LPTIM1 | TIM1,2,6,15,16,LPTIM1/2 | TIM1,2,3,16,17,LPTIM1/2 | TIM1,2,3,4,5,6,7,8,12,15,LPTIM1/2 |
| **I2C** | I2C1 | I2C1/2/3 | I2C1,I2C3 | I2C1/2/3 |
| **SPI** | SPI1 (v1) | SPI1/2 (v1) | SPI1,SPI3 (v2) | SPI1/2/3/4 (v2) |
| **USART** | USART2(limited) | USART1/2/3 | USART1,USART2(basic) | USART1/2/3/6,UART4/5 |
| **LPUART** | LPUART1 | LPUART1 | LPUART1 | LPUART1 |
| **USB** | -- | FS Device | -- | FS DRD (Dev+Host) |
| **BLE** | -- | -- | BLE 5.4 + 802.15.4 | -- |
| **FDCAN** | -- | -- | -- | FDCAN1/2 |
| **I3C** | -- | -- | -- | I3C1/2 |
| **QUADSPI/OCTO** | -- | QUADSPI | -- | OCTOSPI1 |
| **SDMMC** | -- | -- | -- | SDMMC1 |
| **SAI** | -- | -- | SAI1 | -- |
| **RNG** | -- | yes | yes | yes |
| **AES** | 128-bit only | 128/256 | AES+SAES | AES+SAES |
| **PKA** | -- | -- | Full | Limited (verify only) |
| **HASH** | -- | -- | SHA-1/224/256,MD5 | SHA-1/224/256/384/512 |
| **TSC** | -- | yes | yes | -- |
| **ICACHE** | -- | I+D cache | 8KB | 16KB I + 16KB D |
| **TrustZone** | -- | -- | yes | yes |
| **RTC** | full | full | full+binary | full+binary |
| **Backup regs** | 5 | 32 | 32 | 32 |
| **EEPROM** | 512B | -- | -- | -- |
| **Tamper** | 2 ext | 2 ext | 6 ext + active | 11 ext + active |
| **CRC** | full | full | full | full |
| **WWDG** | yes | yes | yes | yes |
| **PVD** | yes | yes | -- | -- |
