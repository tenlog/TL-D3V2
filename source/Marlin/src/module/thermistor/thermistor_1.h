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
#pragma once

// R25 = 100 kOhm, beta25 = 4092 K, 4.7 kOhm pull-up, bed thermistor
const temp_entry_t temptable_1[] PROGMEM = {
  { OV(   0 ), 362 },
  { OV(   3 ), 355 },
  { OV(   5 ), 348 },
  { OV(  10 ), 339 },
  { OV(  12 ), 334 },
  { OV(  14 ), 329 },
  { OV(  15 ), 324 },
  { OV(  16 ), 319 },
  { OV(  17 ), 314 },
  { OV(  19 ), 307 },
  { OV(  21 ), 300 },
  { OV(  23 ), 293 },
  { OV(  25 ), 289 },
  { OV(  27 ), 284 },
  { OV(  28 ), 280 },
  { OV(  31 ), 274 },
  { OV(  33 ), 270 },
  { OV(  35 ), 265 },
  { OV(  38 ), 261 },
  { OV(  41 ), 256 },
  { OV(  44 ), 252 },
  { OV(  48 ), 247 },
  { OV(  52 ), 242 },
  { OV(  56 ), 238 },
  { OV(  61 ), 234 },
  { OV(  66 ), 229 },
  { OV(  71 ), 225 },
  { OV(  78 ), 220 },
  { OV(  84 ), 215 },
  { OV(  92 ), 210 },
  { OV( 100 ), 205 },
  { OV( 109 ), 200 },
  { OV( 120 ), 195 },
  { OV( 131 ), 190 },
  { OV( 143 ), 185 },
  { OV( 156 ), 180 },
  { OV( 171 ), 175 },
  { OV( 187 ), 170 },
  { OV( 205 ), 165 },
  { OV( 224 ), 160 },
  { OV( 245 ), 155 },
  { OV( 268 ), 150 },
  { OV( 293 ), 145 },
  { OV( 320 ), 140 },
  { OV( 348 ), 135 },
  { OV( 379 ), 130 },
  { OV( 411 ), 125 },
  { OV( 445 ), 120 },
  { OV( 480 ), 115 },
  { OV( 516 ), 110 },
  { OV( 553 ), 105 },
  { OV( 591 ), 100 },
  { OV( 628 ),  95 },
  { OV( 665 ),  90 },
  { OV( 702 ),  85 },
  { OV( 737 ),  80 },
  { OV( 770 ),  75 },
  { OV( 801 ),  70 },
  { OV( 830 ),  65 },
  { OV( 857 ),  60 },
  { OV( 881 ),  55 },
  { OV( 903 ),  50 },
  { OV( 922 ),  45 },
  { OV( 939 ),  40 },
  { OV( 954 ),  35 },
  { OV( 966 ),  30 },
  { OV( 977 ),  25 },
  { OV( 985 ),  20 },
  { OV( 993 ),  15 },
  { OV( 999 ),  10 },
  { OV( 1004 ),   5 },
  { OV( 1008 ),   0 },
  { OV( 1012 ),   0 },
  { OV( 1016 ),   0 },
  { OV( 1017 ),   0 },
  { OV( 1024 ),   0 }
};
