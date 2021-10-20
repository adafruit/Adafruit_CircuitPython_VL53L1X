# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2021 Carter Nelson for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""
`adafruit_vl53l1x`
================================================================================

CircuitPython module for interacting with the VL53L1X distance sensor.


* Author(s): Carter Nelson

Implementation Notes
--------------------

**Hardware:**

* Adafruit `VL53L1X Time of Flight Distance Sensor - ~30 to 4000mm <https://www.adafruit.com/product/3967>`_

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://github.com/adafruit/circuitpython/releases
* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
"""

import time
import struct
from adafruit_bus_device import i2c_device
from micropython import const

# imports__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_VL53L1X.git"

_VL53L1X_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND = const(0x0008)
_GPIO_HV_MUX__CTRL = const(0x0030)
_GPIO__TIO_HV_STATUS = const(0x0031)
_SYSTEM__INTERRUPT_CLEAR = const(0x0086)
_SYSTEM__MODE_START = const(0x0087)
_VL53L1X_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 = const(0x0096)
_VL53L1X_IDENTIFICATION__MODEL_ID = const(0x010F)


class VL53L1X:
    """Driver for the VL53L1X distance sensor."""

    def __init__(self, i2c, address=41):
        self.i2c_device = i2c_device.I2CDevice(i2c, address)
        model_id, module_type, mask_rev = self.model_info
        if model_id != 0xEA or module_type != 0xCC or mask_rev != 0x10:
            raise RuntimeError("Wrong sensor ID or type!")
        self._sensor_init()

    def _sensor_init(self):
        init_seq = bytes(
            [
                0x00,  # 0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C), else don't touch
                0x00,  # 0x2e : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD)
                0x00,  # 0x2f : bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD)
                0x01,  # 0x30 : set bit 4 to 0 for active high interrupt and 1 for active low (bits 3:0 must be 0x1), use SetInterruptPolarity()
                0x02,  # 0x31 : bit 1 = interrupt depending on the polarity
                0x00,  # 0x32 : not user-modifiable
                0x02,  # 0x33 : not user-modifiable
                0x08,  # 0x34 : not user-modifiable
                0x00,  # 0x35 : not user-modifiable
                0x08,  # 0x36 : not user-modifiable
                0x10,  # 0x37 : not user-modifiable
                0x01,  # 0x38 : not user-modifiable
                0x01,  # 0x39 : not user-modifiable
                0x00,  # 0x3a : not user-modifiable
                0x00,  # 0x3b : not user-modifiable
                0x00,  # 0x3c : not user-modifiable
                0x00,  # 0x3d : not user-modifiable
                0xFF,  # 0x3e : not user-modifiable
                0x00,  # 0x3f : not user-modifiable
                0x0F,  # 0x40 : not user-modifiable
                0x00,  # 0x41 : not user-modifiable
                0x00,  # 0x42 : not user-modifiable
                0x00,  # 0x43 : not user-modifiable
                0x00,  # 0x44 : not user-modifiable
                0x00,  # 0x45 : not user-modifiable
                0x20,  # 0x46 : interrupt configuration 0->level low detection, 1-> level high, 2-> Out of window, 3->In window, 0x20-> New sample ready , TBC
                0x0B,  # 0x47 : not user-modifiable
                0x00,  # 0x48 : not user-modifiable
                0x00,  # 0x49 : not user-modifiable
                0x02,  # 0x4a : not user-modifiable
                0x0A,  # 0x4b : not user-modifiable
                0x21,  # 0x4c : not user-modifiable
                0x00,  # 0x4d : not user-modifiable
                0x00,  # 0x4e : not user-modifiable
                0x05,  # 0x4f : not user-modifiable
                0x00,  # 0x50 : not user-modifiable
                0x00,  # 0x51 : not user-modifiable
                0x00,  # 0x52 : not user-modifiable
                0x00,  # 0x53 : not user-modifiable
                0xC8,  # 0x54 : not user-modifiable
                0x00,  # 0x55 : not user-modifiable
                0x00,  # 0x56 : not user-modifiable
                0x38,  # 0x57 : not user-modifiable
                0xFF,  # 0x58 : not user-modifiable
                0x01,  # 0x59 : not user-modifiable
                0x00,  # 0x5a : not user-modifiable
                0x08,  # 0x5b : not user-modifiable
                0x00,  # 0x5c : not user-modifiable
                0x00,  # 0x5d : not user-modifiable
                0x01,  # 0x5e : not user-modifiable
                0xCC,  # 0x5f : not user-modifiable
                0x0F,  # 0x60 : not user-modifiable
                0x01,  # 0x61 : not user-modifiable
                0xF1,  # 0x62 : not user-modifiable
                0x0D,  # 0x63 : not user-modifiable
                0x01,  # 0x64 : Sigma threshold MSB (mm in 14.2 format for MSB+LSB), default value 90 mm
                0x68,  # 0x65 : Sigma threshold LSB
                0x00,  # 0x66 : Min count Rate MSB (MCPS in 9.7 format for MSB+LSB)
                0x80,  # 0x67 : Min count Rate LSB
                0x08,  # 0x68 : not user-modifiable
                0xB8,  # 0x69 : not user-modifiable
                0x00,  # 0x6a : not user-modifiable
                0x00,  # 0x6b : not user-modifiable
                0x00,  # 0x6c : Intermeasurement period MSB, 32 bits register
                0x00,  # 0x6d : Intermeasurement period
                0x0F,  # 0x6e : Intermeasurement period
                0x89,  # 0x6f : Intermeasurement period LSB
                0x00,  # 0x70 : not user-modifiable
                0x00,  # 0x71 : not user-modifiable
                0x00,  # 0x72 : distance threshold high MSB (in mm, MSB+LSB)
                0x00,  # 0x73 : distance threshold high LSB
                0x00,  # 0x74 : distance threshold low MSB ( in mm, MSB+LSB)
                0x00,  # 0x75 : distance threshold low LSB
                0x00,  # 0x76 : not user-modifiable
                0x01,  # 0x77 : not user-modifiable
                0x0F,  # 0x78 : not user-modifiable
                0x0D,  # 0x79 : not user-modifiable
                0x0E,  # 0x7a : not user-modifiable
                0x0E,  # 0x7b : not user-modifiable
                0x00,  # 0x7c : not user-modifiable
                0x00,  # 0x7d : not user-modifiable
                0x02,  # 0x7e : not user-modifiable
                0xC7,  # 0x7f : ROI center
                0xFF,  # 0x80 : XY ROI (X=Width, Y=Height)
                0x9B,  # 0x81 : not user-modifiable
                0x00,  # 0x82 : not user-modifiable
                0x00,  # 0x83 : not user-modifiable
                0x00,  # 0x84 : not user-modifiable
                0x01,  # 0x85 : not user-modifiable
                0x00,  # 0x86 : clear interrupt
                0x00,  # 0x87 : start ranging
            ]
        )
        self._write_registerN(0x002D, init_seq)
        self.start_ranging()
        while not self.data_ready:
            time.sleep(0.01)
        self.clear_interrupt()
        self.stop_ranging()
        # not sure what these next two are, but driver has them
        self._write_registerN(_VL53L1X_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, b"\x09")
        self._write_registerN(0x0B, b"\x00")

    @property
    def model_info(self):
        """A 3 tuple of Model ID, Module Type, and Mask Revision."""
        info = self._read_registerN(_VL53L1X_IDENTIFICATION__MODEL_ID, 3)
        return (info[0], info[1], info[2])  # Model ID, Module Type, Mask Rev

    @property
    def distance(self):
        """The distance in units of centimeters."""
        dist = self._read_registerN(
            _VL53L1X_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, 2
        )
        dist = struct.unpack(">H", dist)[0]
        return dist / 10

    def start_ranging(self):
        """Starts ranging operation."""
        self._write_registerN(_SYSTEM__MODE_START, b"\x40")

    def stop_ranging(self):
        """Stopss ranging operation."""
        self._write_registerN(_SYSTEM__MODE_START, b"\x00")

    def clear_interrupt(self):
        """Clears new data interrupt."""
        self._write_registerN(_SYSTEM__INTERRUPT_CLEAR, b"\x01")

    @property
    def data_ready(self):
        """Returns true if new data is ready, otherwise false."""
        if (
            self._read_registerN(_GPIO__TIO_HV_STATUS)[0] & 0x01
            == self._interrupt_polarity
        ):
            return True
        return False

    @property
    def _interrupt_polarity(self):
        int_pol = self._read_registerN(_GPIO_HV_MUX__CTRL)[0] & 0x10
        int_pol = (int_pol >> 4) & 0x01
        return 0 if int_pol else 1

    def _write_registerN(self, address, data, length=None):
        if length is None:
            length = len(data)
        with self.i2c_device as i2c:
            i2c.write(struct.pack(">H", address) + data[:length])

    def _read_registerN(self, address, length=1):
        data = bytearray(length)
        with self.i2c_device as i2c:
            i2c.write(struct.pack(">H", address))
            i2c.readinto(data)
        return data
