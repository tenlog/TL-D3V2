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

#include "../inc/MarlinConfig.h"

#include "SdInfo.h"
#include "disk_io_driver.h"

bool SDIO_Init();
bool SDIO_ReadBlock(uint32_t block, uint8_t *dst);
bool SDIO_WriteBlock(uint32_t block, const uint8_t *src);

class DiskIODriver_SDIO : public DiskIODriver {
  public:
    virtual bool init(const uint8_t sckRateID=0, const pin_t chipSelectPin=0) override { return SDIO_Init(); }

    virtual bool readCSD(csd_t *csd)                              override { return false; }

    virtual bool readStart(const uint32_t block)                  override { return false; }
    virtual bool readData(uint8_t *dst)                           override { return false; }
    virtual bool readStop()                                       override { return false; }

    virtual bool writeStart(const uint32_t block, const uint32_t) override { return false; }
    virtual bool writeData(const uint8_t *src)                    override { return false; }
    virtual bool writeStop()                                      override { return false; }

    virtual bool readBlock(uint32_t block, uint8_t *dst)          override { return SDIO_ReadBlock(block, dst); }
    virtual bool writeBlock(uint32_t block, const uint8_t *src)   override { return SDIO_WriteBlock(block, src); }

    virtual uint32_t cardSize()                                   override { return 0; }

    virtual bool isReady()                                        override { return true; }

    virtual void idle()                                           override {}
};
