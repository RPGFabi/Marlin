/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../../../inc/MarlinConfigPre.h"

#if ANY(HAS_PRUSA_MMU2, HAS_RPGFABI_MFU)

#include "../../gcode.h"

#if HAS_PRUSA_MMU2
  #include "../../../feature/mmu/mmu2.h"
#elif HAS_RPGFABI_MFU
  #include "../../../feature/mfu/mfu.h"
#endif

/**
 * M403: Set filament type for MMU2 or MFU
 *
 * Valid filament type values:
 *
 * For MMU2
 *  0   Default
 *  1   Flexible
 *  2   PVA
 *
 * For MFU
 * -1                     No Filament
 * 0 to Extrudercount-1   Same Filament gets same number
 */
void GcodeSuite::M403() {
  int8_t index = parser.intval('E', -1),
         type = parser.intval('F', -1);

  #if HAS_PRUSA_MMU2
    if (WITHIN(index, 0, EXTRUDERS - 1) && WITHIN(type, 0, 2))
      mmu2.set_filament_type(index, type);
    else
      SERIAL_ECHO_MSG("M403 - bad arguments.");
  #elif HAS_RPGFABI_MFU
    if (!parser.seen_any()){
      mfu.print_filament_type();
      return;
    }

    if (WITHIN(index, 0, EXTRUDERS - 1) && WITHIN(type, -1, EXTRUDERS - 1)){
      mfu.set_filament_type(index, type);
      SERIAL_ECHOLN_P("Filamenttype set.");
      return;
    }
    else if (index >= EXTRUDERS){
      SERIAL_ECHOLN_P("The given Extruder is higher than the Extrudercount.");
    }
    else if (type >= EXTRUDERS){
      SERIAL_ECHOLN_P("Given Filamenttype is higher than max Extrudercount. You don't need that");
    }
  #endif
}

#endif // HAS_PRUSA_MMU2 || HAS_RPGFABI_MFU