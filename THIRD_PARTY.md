# Third-party code in tiles

Most of tiles is Bergsonne's own code. The exceptions are STMicroelectronics
components (and the Synopsys and OpenThread headers ST ships with them) used
by the Core.ST.W5 (STM32WBA55) radio support. They live in two folders:

- `sdk/wpan/`: the 2.4 GHz radio plumbing any ST link-layer protocol needs
  (link layer, sequencer, timer server, flash manager, memory managers).
- `sdk/ble/`: the Bluetooth LE host stack and its glue.

They keep their original copyright headers (files changed for tiles are
marked `tiles:` where they differ), and are distributed under the terms below,
**for use with STMicroelectronics microcontrollers only.** They are not
covered by any license Bergsonne applies to the rest of this repository.

Source: STM32CubeWBA V1.6.1 (`STM32Cube_FW_WBA_V1.6.1`; both libraries are
byte-identical to that release's), whose `Package_license.md` assigns each
folder its license. ST's STM32_WPAN license file (its bill of materials and
the full SLA0044 text, which applies to the component outside the Cube
package) is reproduced verbatim next to the code, as
[`sdk/wpan/LICENSE-ST.txt`](sdk/wpan/LICENSE-ST.txt) and
[`sdk/ble/LICENSE-ST.txt`](sdk/ble/LICENSE-ST.txt): the "LICENSE file" the
ST file headers refer to.

| Component (CubeWBA origin) | Files in tiles | License |
|---|---|---|
| STM32_WPAN: BLE stack (`Middlewares/ST/STM32_WPAN/ble`) | `sdk/ble/lib/stm32wba_ble_stack_basic.a`; `sdk/ble/include/ble*.h`, `sdk/ble/include/auto/ble_*.h`, `bleplat.h`, `blestack.h`, `svc_ctl.h`; `sdk/ble/ble_wrap.c`, `svc_ctl.c` | ST SLA (below) |
| STM32_WPAN: Link Layer (`Middlewares/ST/STM32_WPAN/link_layer`) | `sdk/wpan/lib/LinkLayer_BLE_Basic_lib.a`; `sdk/wpan/include/linklayer_plat.h`, `ll_sys.h`, `ll_sys_if.h`, `stm32_wpan_common.h`; `sdk/wpan/ll_sys_startup.c` | ST SLA (below) |
| Link Layer public headers (Synopsys, Inc.) | `sdk/wpan/link_layer/inc/*.h`, `sdk/wpan/link_layer/inc/_40nm_reg_files/*.h` | Synopsys MIT notice in each file; listed under ST SLA in ST's bill of materials |
| OpenThread headers (The OpenThread Authors) | `sdk/wpan/link_layer/inc/ot_inc/*.h` | BSD-3-Clause, notice in each file |
| Application modules and glue (`Projects/Common/WPAN`, `Projects/*/Applications`) | `sdk/wpan/advanced_memory_manager.{c,h}`, `stm32_mm.{c,h}`, `flash_manager.{c,h}`, `flash_driver.{c,h}`, `rf_timing_synchro.{c,h}`, `ll_sys_if.c`, `linklayer_plat.c`, `power_table.c`; `sdk/ble/host_stack_if.{c,h}`, `sdk/ble/include/bpka.h`, `sdk/ble/include/ble_timer.h` | ST SLA (below) |
| Adapted from ST sources | `sdk/ble/include/ble_common.h` | ST SLA (below) |
| Utilities: sequencer and timer server (`Utilities/sequencer`, `Utilities/tim_serv`) | `sdk/wpan/stm32_seq.{c,h}`, `sdk/wpan/stm32_timer.{c,h}` | BSD-3-Clause (below) |

(`.h` files listed with a `.c` sit in that folder's `include/`.) Everything
else in `sdk/wpan/` and `sdk/ble/` (for example `hw_aes.c`, `hw_rng.c`,
`stm32_timer_if.c`, `ble_app.c`, `ble_glue.c`, `ble_flash.c`, `nvm_stub.c`)
is Bergsonne's.

---

## STMicroelectronics Software License Agreement (SLA)

Reproduced from STM32CubeWBA `Package_license.md`, section "SLA – Software
License Agreement".

### Software license agreement

#### SOFTWARE PACKAGE LICENSE AGREEMENT

BY INSTALLING COPYING, DOWNLOADING, ACCESSING OR OTHERWISE USING THIS SOFTWARE PACKAGE OR ANY
PART THEREOF (AND THE RELATED DOCUMENTATION) FROM STMICROELECTRONICS INTERNATIONAL N.V, SWISS
BRANCH AND/OR ITS AFFILIATED COMPANIES (STMICROELECTRONICS), THE RECIPIENT, ON BEHALF OF HIMSELF
OR HERSELF, OR ON BEHALF OF ANY ENTITY BY WHICH SUCH RECIPIENT IS EMPLOYED AND/OR ENGAGED
AGREES TO BE BOUND BY THIS SOFTWARE PACKAGE LICENSE AGREEMENT.

Under STMicroelectronics’ intellectual property rights and subject to applicable licensing terms for any third-party software
incorporated in this software package and applicable Open Source Terms (as defined here below), the redistribution,
reproduction and use in source and binary forms of the software package or any part thereof, with or without modification, are
permitted provided that the following conditions are met:

1. Redistribution of source code (modified or not) must retain any copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form, except as embedded into microcontroller or microprocessor device manufactured by or for
STMicroelectronics or a software update for such device, must reproduce the above copyright notice, this list of conditions
and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of STMicroelectronics nor the names of other contributors to this software package may be used to
endorse or promote products derived from this software package or part thereof without specific written permission.

4. This software package or any part thereof, including modifications and/or derivative works of this software package, must
be used and execute solely and exclusively on or in combination with a microcontroller or a microprocessor devices
manufactured by or for STMicroelectronics.

5. No use, reproduction or redistribution of this software package partially or totally may be done in any manner that would
subject this software package to any Open Source Terms (as defined below).

6. Some portion of the software package may contain software subject to Open Source Terms (as defined below) applicable
for each such portion (“Open Source Software”), as further specified in the software package. Such Open Source Software
is supplied under the applicable Open Source Terms and is not subject to the terms and conditions of license hereunder.
“Open Source Terms” shall mean any open source license which requires as part of distribution of software that the source
code of such software is distributed therewith or otherwise made available, or open source license that substantially
complies with the Open Source definition specified at www.opensource.org and any other comparable open source license
such as for example GNU General Public License (GPL), Eclipse Public License (EPL), Apache Software License, BSD
license and MIT license.

7. This software package may also include third party software as expressly specified in the software package subject to
specific license terms from such third parties. Such third party software is supplied under such specific license terms and is
not subject to the terms and conditions of license hereunder. By installing copying, downloading, accessing or otherwise
using this software package, the recipient agrees to be bound by such license terms with regard to such third party
software.

8. STMicroelectronics has no obligation to provide any maintenance, support or updates for the software package.

9. The software package is and will remain the exclusive property of STMicroelectronics and its licensors. The recipient will
not take any action that jeopardizes STMicroelectronics and its licensors' proprietary rights or acquire any rights in the
software package, except the limited rights specified hereunder.

10. The recipient shall comply with all applicable laws and regulations affecting the use of the software package or any part
thereof including any applicable export control law or regulation.

11. Redistribution and use of this software package partially or any part thereof other than as permitted under this license is
void and will automatically terminate your rights under this license.

THIS SOFTWARE PACKAGE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY
INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE PACKAGE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

EXCEPT AS EXPRESSLY PERMITTED HEREUNDER AND SUBJECT TO THE APPLICABLE LICENSING TERMS FOR ANY
THIRD-PARTY SOFTWARE INCORPORATED IN THE SOFTWARE PACKAGE AND OPEN SOURCE TERMS AS
APPLICABLE, NO LICENSE OR OTHER RIGHTS, WHETHER EXPRESS OR IMPLIED, ARE GRANTED UNDER ANY
PATENT OR OTHER INTELLECTUAL PROPERTY RIGHTS OF STMICROELECTRONICS OR ANY THIRD PARTY.



---

## BSD-3-Clause (STM32 Utilities)

Copyright (c) STMicroelectronics. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its contributors
   may be used to endorse or promote products derived from this software
   without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
