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


/**
 * Thanks to Dust for his help on getting me started with SerialCommunication and Marlin
*/
#pragma once
#include "../../MarlinCore.h"

#if HAS_FILAMENT_SENSOR
  #include "../runout.h"
#endif

#define MFU_RX_BUFFERSIZE 16
#define MFU_TX_BUFFERSIZE 16
#define MFU_NO_TOOL -1

#define MFU_CMD_TIMEOUT 45000UL   // Delay between the wait for ok checks

#define MFU_CMD_NOCMD 0x00
#define MFU_CMD_UNLOADTOOL 0x01
#define MFU_CMD_LOADTOOL 0x02
#define MFU_CMD_FIRSTTOOL 0x20

#define MFU_SEND(S) tx_str(F(S "\n"))
#define MFU_RECV(S) rx_str(F(S "\n"))

inline void mfu_e_move(const float &dist, const feedRate_t fr_mm_s, const bool sync =true);

struct MFU_Filament{
  int8_t types[EXTRUDERS];
} ;

class MFU{
  private:
    static bool ready;
    static bool pausedDueToFilamentShortage;
    static bool filamentAvailable[];

  #ifdef MFU_USE_BUZZER_IF_FILAMENT_EMPTY
    static uint32_t nextBuzz;
  #endif

    static uint8_t cmd, cmd_arg, last_cmd, extruder;
    static int8_t state;
    static volatile bool finda_runout_valid;
    static millis_t prev_request, prev_P0_request;
    static celsius_t hotendTemp_BeforeRunout;

    static bool _enabled, toolLoaded;
    static char rx_buffer[MFU_RX_BUFFERSIZE], tx_buffer[MFU_TX_BUFFERSIZE];
    static void tx_str(FSTR_P fstr);
    static void tx_printf(FSTR_P format, int argument);
    static void tx_printf(FSTR_P format, int argument1, int argument2);
    static bool rx_str(FSTR_P fstr);
    static void clear_rx_buffer();
    static bool get_response();
    static bool rx_start();

    static void home();

    static void manage_response(const bool move_axes, const bool turn_off_nozzle);

    static void handle_MFU_FilamentRunout();

  public:
    MFU();
    static MFU_Filament filamentTypes;

    static void init();
    static void loop();
    static bool enabled() {return _enabled; }

    static void tool_change(const uint8_t index, const bool forceChange);

    static bool unload();

    static void setCommand(const uint8_t newCommand);
    static void set_runout_valid(const bool valid);

    static void set_filament_type(int8_t extruder, int8_t type);
    static void print_filament_type();
};

extern MFU mfu;