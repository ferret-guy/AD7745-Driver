from typing import List, Optional
from dataclasses import dataclass
from enum import Enum, auto
import logging
import bisect
from statistics import mean
import time

from binhoHostAdapter import binhoHostAdapter
import binhoUtilities

logger = logging.getLogger(__name__)


# noinspection PyCallByClass
class AD7745:
    I2C_INDEX = 0

    WRITE_ADDR = 0x90
    READ_ADDR = 0x91

    STATUS_ADDR = 0x00
    STATUS_EXCERR = 1 << 3
    STATUS_RDY = 1 << 2
    STATUS_RDYVT = 1 << 1
    STATUS_RDYCAP = 1 << 0

    CAP_DATA_H_ADDR = 0x01
    CAP_DATA_M_ADDR = 0x02
    CAP_DATA_L_ADDR = 0x03

    VT_DATA_H_ADDR = 0x04
    VT_DATA_M_ADDR = 0x05
    VT_DATA_L_ADDR = 0x06

    CAP_SETUP_ADDR = 0x07
    CAP_SETUP_CAPEN = 1 << 7
    CAP_SETUP_CIN2 = 1 << 6
    CAP_SETUP_CAPDIFF = 1 << 5
    CAP_SETUP_CAPCHOP = 1

    VT_SETUP_ADDR = 0x08
    VT_SETUP_VTEN = 1 << 7
    VT_SETUP_VTMD1 = 1 << 6
    VT_SETUP_VTMD0 = 1 << 5
    VT_SETUP_EXTREF = 1 << 4
    VT_SETUP_VTSHORT = 1 << 1
    VT_SETUP_VTCHOP = 1

    EXC_SETUP_ADDR = 0x09
    EXC_SETUP_CLKCTRL = 1 << 7
    EXC_SETUP_EXCON = 1 << 6
    EXC_SETUP_EXCB = 1 << 5
    EXC_SETUP_UEXCB = 1 << 4
    EXC_SETUP_EXCA = 1 << 3
    EXC_SETUP_UEXCA = 1 << 2
    EXC_SETUP_EXCLVL1 = 1 << 1
    EXC_SETUP_EXCLVL0 = 1

    CONFIG_ADDR = 0x0A
    CONFIG_VTFS1 = 1 << 7
    CONFIG_VTFS0 = 1 << 6
    CONFIG_CAPFS2 = 1 << 5
    CONFIG_CAPFS1 = 1 << 4
    CONFIG_CAPFS0 = 1 << 3
    CONFIG_MD2 = 1 << 2
    CONFIG_MD1 = 1 << 1
    CONFIG_MD0 = 1

    CAP_DAC_A_ADDR = 0x0B
    CAP_DAC_A_DACAENA = 1 << 7
    CAP_DAC_A_DACA7bit = 0x7F

    CAP_DAC_B_ADDR = 0x0C
    CAP_DAC_B_DACBENB = 1 << 7
    CAP_DAC_B_DACB7bit = 0x7F

    CAP_OFFSET_H_ADDR = 0x0D
    CAP_OFFSET_L_ADDR = 0x0E

    CAP_GRAIN_H_ADDR = 0x0F
    CAP_GRAIN_L_ADDR = 0x10

    VOLT_GRAIN_H_ADDR = 0x11
    VOLT_GRAIN_L_ADDR = 0x12

    class CapInput(Enum):
        CAP1_EXC1 = auto()
        CAP2_EXC2 = auto()
        CAP_DIFF_CIN1 = auto()
        CAP_DIFF_CIN2 = auto()

    class CapFilter(Enum):
        # Note frequencies listed as -3dB point
        CONV87Hz = auto()
        CONV79Hz = auto()
        CONV44Hz = auto()
        CONV22Hz = auto()
        CONV14Hz = auto()
        CONV11Hz = auto()
        CONV9Hz = auto()
        CONV8Hz = auto()

    @dataclass
    class vt_setup_reg:
        VTEN: bool = False
        VTMD1: bool = False
        VTMD0: bool = False
        EXTREF: bool = False
        VTSHORT: bool = False
        VTCHOP: bool = False

    @dataclass
    class cap_setup_reg:
        CAPEN: bool = False
        CIN2: bool = False
        CAPDIFF: bool = False
        CAPCHOP: bool = False

    @dataclass
    class exc_setup_reg:
        CLKCTRL: bool = False
        EXCON: bool = False
        EXCB: bool = False
        UEXCB: bool = False
        EXCA: bool = False
        UEXCA: bool = False
        EXCLVL1: bool = False
        EXCLVL0: bool = False

    @dataclass
    class configuration_reg:
        VTFS1: bool = False
        VTFS0: bool = False
        CAPFS2: bool = False
        CAPFS1: bool = False
        CAPFS0: bool = False
        MD2: bool = False
        MD1: bool = False
        MD0: bool = False

    @dataclass
    class cap_daca_reg:
        DACAENA: bool = False
        DACA7bit: int = 0

    @dataclass
    class cap_dacb_reg:
        DACBENB: bool = False
        DACB7bit: int = 0

    @dataclass
    class status_reg:
        EXCERR: bool = False
        RDY: bool = False
        RDYVT: bool = False
        RDYCAP: bool = False

    def __init__(self, binho: binhoHostAdapter.binhoHostAdapter) -> None:
        self.i2cbus = binho
        self.i2cbus.setOperationMode(AD7745.I2C_INDEX, "I2C")
        self.i2cbus.setPullUpStateI2C(0, "EN")
        self.set_clock(400000)
        self.cont_conv = True

        self._cap_dac_lsb: Optional[float] = None

        # Disable voltage channel on startup
        self.vt_setup = AD7745.vt_setup_reg(VTEN=False)
        # Configure for idle, slow read time
        self.configuration = AD7745.configuration_reg(
            VTFS1=True,
            VTFS0=True,
            CAPFS0=True,
            CAPFS1=True,
            CAPFS2=True,
            MD0=False,
            MD1=False,
            MD2=False,
        )

    @staticmethod
    def _check_device_success(ret_str: str) -> None:
        """
        Check whether the operation is successful or not.
        '-OK' -> success
        '-NG' -> fail

        :param ret_str: Binho return string
        :return: None
        """
        if ret_str == "-OK":
            return
        elif ret_str == "-NG":
            raise RuntimeError("Binho returned NG!")
        else:
            raise RuntimeError("Binho returned Unknown command!")

    def write_byte(self, register: int, value: int) -> None:
        """
        Write byte to AD7745

        :param register: Register address to write to
        :param value: Value to write to register
        :return: None
        """
        self._check_device_success(
            self.i2cbus.startI2C(AD7745.I2C_INDEX, AD7745.WRITE_ADDR)
        )
        self._check_device_success(self.i2cbus.writeByteI2C(AD7745.I2C_INDEX, register))
        self._check_device_success(self.i2cbus.writeByteI2C(AD7745.I2C_INDEX, value))
        self._check_device_success(self.i2cbus.endI2C(0, False))

    def set_clock(self, freq: int) -> None:
        """
        Set Binho I2C frequency

        :param freq: I2C Frequency, 0 to 400kHz
        :return: None
        """
        if not 0 <= freq <= 400000:
            raise ValueError(f"Invalid Frequency {freq}, allowed range 0-400khz")
        self._check_device_success(self.i2cbus.setClockI2C(AD7745.I2C_INDEX, freq))

    def get_clock(self) -> int:
        """
        Get the Binho I2C frequency

        :return: current Binho I2C frequency
        """
        read_string = self.i2cbus.getCLockI2C(AD7745.I2C_INDEX)
        if read_string[1:3] == "NG":
            raise RuntimeError("Binho returned NG!")
        return int(read_string[12:], 16)

    def read_byte(self, addr: int, byte_len: int = 1) -> List[int]:
        """
        Read byte from register on Binho

        :param addr: Register address to read from
        :param byte_len: bytes to read, from sequential address
        :return:
        """
        self._check_device_success(
            self.i2cbus.startI2C(AD7745.I2C_INDEX, AD7745.WRITE_ADDR)
        )
        self._check_device_success(self.i2cbus.writeByteI2C(AD7745.I2C_INDEX, addr))
        self._check_device_success(self.i2cbus.endI2C(AD7745.I2C_INDEX, True))

        read_string = self.i2cbus.readBytesI2C(
            AD7745.I2C_INDEX, AD7745.READ_ADDR, byte_len
        )
        if read_string[1:3] == "NG":
            raise RuntimeError("Binho returned NG!")
        return [int(i, 0) for i in read_string[10:].split(" ")]

    @property
    def status(self) -> "AD7745.status_reg":
        """
        STATUS REGISTER: Address Pointer 0x00, Read Only, Default Value 0x07
            EXCERR: EXCERR = 1 indicates that the excitation output cannot be driven properly.
                The possible reason can be a short circuit or too high capacitance between the
                excitation pin and ground.
            RDY: RDY = 0 indicates that conversion on the enabled channel(s) has been finished
                and new unread data is available. If both capacitive and voltage/temperature
                channels are enabled, the RDY bit is changed to 0 after conversion on both
                channels is finished. The RDY bit returns to 1 either when data is read or
                prior to finishing the next conversion. If, for example, only the capacitive
                channel is enabled, then the RDY bit reflects the RDYCAP bit.
            RDYVT: RDYVT = 0 indicates that a conversion on the voltage/temperature channel has
                been finished and new unread data is available
            RDYCAP: RDYCAP = 0 indicates that a conversion on the capacitive channel has been
                finished and new unread data is available.

        :return: AD7745.status_reg
        """
        read_string = self.read_byte(AD7745.STATUS_ADDR, byte_len=1)[0]

        return self.status_reg(
            EXCERR=bool(read_string & AD7745.STATUS_EXCERR),
            RDY=bool(read_string & AD7745.STATUS_RDY),
            RDYVT=bool(read_string & AD7745.STATUS_RDYVT),
            RDYCAP=bool(read_string & AD7745.STATUS_RDYCAP),
        )

    @property
    def cap_data(self) -> int:
        """
        CAP DATA REGISTER: 24 Bits, Address Pointer 0x01, 0x02, 0x03, Read-Only, Default Value 0x000000

        :return: 24-Bit Cap Data value
        """
        read_data = self.read_byte(AD7745.CAP_DATA_H_ADDR, byte_len=3)
        cap_h = read_data[0]
        cap_m = read_data[1]
        cap_l = read_data[2]

        return (cap_h << 16) + (cap_m << 8) + cap_l

    @property
    def vt_data(self) -> int:
        """
        VT DATA REGISTER: 24 Bits, Address Pointer 0x04, 0x05, 0x06, Read-Only, Default Value 0x000000

        :return: 24-Bit Voltage Data value
        """
        read_data = self.read_byte(AD7745.CAP_DATA_H_ADDR, byte_len=3)
        vt_h = read_data[0]
        vt_m = read_data[1]
        vt_l = read_data[2]
        return (vt_h << 16) + (vt_m << 8) + vt_l

    @property
    def cap_setup(self) -> "AD7745.cap_setup_reg":
        """
        CAP SET-UP REGISTER : Address Pointer 0x07, Default Value 0x00
            CAPEN: CAPEN = 1 enables capacitive channel for single conversion, continuous conversion, or calibration.
            CIN2: CIN2 = 1 switches the internal multiplexer to the second capacitive input on the AD7746.
            CAPDIFF: CAPDIFF = 1 sets differential mode on the selected capacitive input.
            CAPCHOP: CAPCHOP: The CAPCHOP bit should be set to 0 for the specified capacitive channel performance.
                CAPCHOP = 1 approximately doubles the capacitive channel conversion times and slightly improves the
                capacitive channel noise performance for the longest conversion times.

        :return: AD7745.cap_setup_reg
        """
        read_data = self.read_byte(AD7745.CAP_SETUP_ADDR, byte_len=1)[0]
        return self.cap_setup_reg(
            CAPEN=bool(read_data & AD7745.CAP_SETUP_CAPEN),
            CIN2=bool(read_data & AD7745.CAP_SETUP_CIN2),
            CAPDIFF=bool(read_data & AD7745.CAP_SETUP_CAPDIFF),
            CAPCHOP=bool(read_data & AD7745.CAP_SETUP_CAPCHOP),
        )

    @cap_setup.setter
    def cap_setup(self, val: "AD7745.cap_setup_reg") -> None:
        """
        CAP SET-UP REGISTER : Address Pointer 0x07, Default Value 0x00
            CAPEN: CAPEN = 1 enables capacitive channel for single conversion, continuous conversion, or calibration.
            CIN2: CIN2 = 1 switches the internal multiplexer to the second capacitive input on the AD7746.
            CAPDIFF: CAPDIFF = 1 sets differential mode on the selected capacitive input.
            CAPCHOP: CAPCHOP: The CAPCHOP bit should be set to 0 for the specified capacitive channel performance.
                CAPCHOP = 1 approximately doubles the capacitive channel conversion times and slightly improves the
                capacitive channel noise performance for the longest conversion times.

        :param val: AD7745.cap_setup_reg to write
        :return: None
        """
        value = 0
        if val.CAPEN:
            value += AD7745.CAP_SETUP_CAPEN
        if val.CIN2:
            value += AD7745.CAP_SETUP_CIN2
        if val.CAPDIFF:
            value += AD7745.CAP_SETUP_CAPDIFF
        if val.CAPCHOP:
            value += AD7745.CAP_SETUP_CAPCHOP

        self.write_byte(AD7745.CAP_SETUP_ADDR, value)

    @property
    def vt_setup(self) -> "AD7745.vt_setup_reg":
        """
        VT SET-UP REGISTER: Address Pointer 0x08, Default Value 0x00
            VTEN: VTEN = 1 enables voltage/temperature channel for single conversion, continuous conversion,
                or calibration.
            VTMD1, VTDM0: Voltage/temperature channel input configuration.

                    ====== ====== ====================================
                    VTMD1  VTMD0               Channel Input
                    ====== ====== ====================================
                    0       0       Internal temperature sensor
                    0       1       External temperature sensor diode
                    1       0       VDD monitor
                    1       1       External voltage input (VIN)
                    ====== ====== ====================================

            EXTREF: EXTREF = 1 selects an external reference voltage connected to REFIN(+), REFIN(–) for the voltage
                input or the VDD monitor.
            EXTREF: EXTREF = 0 selects the on-chip internal reference. The internal reference must be used with the
                internal temperature sensor for proper operation.
            VTSHORT: VTSHORT = 1 internally shorts the voltage/temperature channel input for test purposes.
            VTCHOP: VTCHOP = 1 sets internal chopping on the voltage/temperature channel. The VTCHOP bit must be set to
                1 for the specified voltage/temperature channel performance.

        :return: AD7745.vt_setup_reg
        """
        read_string = self.read_byte(AD7745.VT_SETUP_ADDR, byte_len=1)[0]
        return AD7745.vt_setup_reg(
            VTEN=bool(read_string & AD7745.VT_SETUP_VTEN),
            VTMD1=bool(read_string & AD7745.VT_SETUP_VTMD1),
            VTMD0=bool(read_string & AD7745.VT_SETUP_VTMD0),
            EXTREF=bool(read_string & AD7745.VT_SETUP_EXTREF),
            VTSHORT=bool(read_string & AD7745.VT_SETUP_VTSHORT),
            VTCHOP=bool(read_string & AD7745.VT_SETUP_VTCHOP),
        )

    @vt_setup.setter
    def vt_setup(self, val: "AD7745.vt_setup_reg") -> None:
        """
        VT SET-UP REGISTER: Address Pointer 0x08, Default Value 0x00
        VTEN: VTEN = 1 enables voltage/temperature channel for single conversion, continuous conversion, or calibration.
        VTMD1, VTDM0: Voltage/temperature channel input configuration.

                    ====== ====== ====================================
                    VTMD1  VTMD0     Channel Input
                    ====== ====== ====================================
                    0      0       Internal temperature sensor
                    0      1       External temperature sensor diode
                    1      0       VDD monitor
                    1      1       External voltage input (VIN)
                    ====== ====== ====================================

        EXTREF: EXTREF = 1 selects an external reference voltage connected to REFIN(+), REFIN(–) for the voltage input
            or the VDD monitor.
        EXTREF: EXTREF = 0 selects the on-chip internal reference. The internal reference must be used with the internal
            temperature sensor for proper operation.
        VTSHORT: VTSHORT = 1 internally shorts the voltage/temperature channel input for test purposes.
        VTCHOP: VTCHOP = 1 sets internal chopping on the voltage/temperature channel. The VTCHOP bit must be set to 1
            for the specified voltage/temperature channel performance.

        :param val: AD7745.vt_setup_reg to set
        :return: None
        """
        reg_value = 0
        if val.VTEN:
            reg_value += AD7745.VT_SETUP_VTEN
        if val.VTMD1:
            reg_value += AD7745.VT_SETUP_VTMD1
        if val.VTMD0:
            reg_value += AD7745.VT_SETUP_VTMD0
        if val.EXTREF:
            reg_value += AD7745.VT_SETUP_EXTREF
        if val.VTSHORT:
            reg_value += AD7745.VT_SETUP_VTSHORT
        if val.VTCHOP:
            reg_value += AD7745.VT_SETUP_VTCHOP
        self.write_byte(AD7745.VT_SETUP_ADDR, reg_value)

    @property
    def exc_setup(self) -> "AD7745.exc_setup_reg":
        """
        EXC SET-UP REGISTER: Address Pointer 0x09, Default Value 0x03
        CLKCTRL: The CLKCTRL bit should be set to 0 for the specified AD7745/AD7746 performance. CLKCTRL = 1 decreases
            the excitation signal frequency and the modulator clock frequency by factor of 2. This also increases the
             conversion time on all channels (capacitive, voltage, and temperature) by a factor of 2.
        EXCON: When EXCON = 0, the excitation signal is present on the output only during capacitance channel
            conversion. When EXCON = 1, the excitation signal is present on the output during both capacitance and
            voltage/temperature conversion.
        EXCB: EXCB = 1 enables EXCB pin as the excitation output.
        UEXCB: UEXCB = 1 enables EXCB pin as the inverted excitation output. Only one of the EXCB or the EXCB bits
            should be set for proper operation.
        EXCA: EXCA = 1 enables EXCA pin as the excitation output.
        UEXCA: EXCA = 1 enables EXCA pin as the inverted excitation output. Only one of the EXCA or the EXCA bits should
            be set for proper operation.
        EXCLVL1, EXCLVL0 : Excitation Voltage Level.

            ======= ======= ============== ================= ==================
            EXCLVL1 EXCLVL0 Voltage on Cap EXC Pin Low Level EXC Pin High Level
            ======= ======= ============== ================= ==================
               0    0          ±VDD/8          VDD × 3/8           VDD × 5/8
               0    1          ±VDD/4          VDD × 1/4           VDD × 3/4
               1    0          ±VDD × 3/8      VDD × 1/8           VDD × 7/8
               1    1          ±VDD/2          0                   VDD
            ======= ======= ============== ================= ==================

        :param: None
        :return: AD7745.exc_setup_reg
        """
        read_string = self.read_byte(AD7745.EXC_SETUP_ADDR, byte_len=1)[0]
        return AD7745.exc_setup_reg(
            CLKCTRL=bool(read_string & AD7745.EXC_SETUP_CLKCTRL),
            EXCON=bool(read_string & AD7745.EXC_SETUP_EXCON),
            EXCB=bool(read_string & AD7745.EXC_SETUP_EXCB),
            UEXCB=bool(read_string & AD7745.EXC_SETUP_UEXCB),
            EXCA=bool(read_string & AD7745.EXC_SETUP_EXCA),
            UEXCA=bool(read_string & AD7745.EXC_SETUP_UEXCA),
            EXCLVL1=bool(read_string & AD7745.EXC_SETUP_EXCLVL1),
            EXCLVL0=bool(read_string & AD7745.EXC_SETUP_EXCLVL0),
        )

    @exc_setup.setter
    def exc_setup(self, val: "AD7745.exc_setup_reg") -> None:
        """
        EXC SET-UP REGISTER: Address Pointer 0x09, Default Value 0x03
        CLKCTRL: The CLKCTRL bit should be set to 0 for the specified AD7745/AD7746 performance. CLKCTRL = 1 decreases
            the excitation signal frequency and the modulator clock frequency by factor of 2. This also increases the
             conversion time on all channels (capacitive, voltage, and temperature) by a factor of 2.
        EXCON: When EXCON = 0, the excitation signal is present on the output only during capacitance channel
            conversion. When EXCON = 1, the excitation signal is present on the output during both capacitance and
            voltage/temperature conversion.
        EXCB: EXCB = 1 enables EXCB pin as the excitation output.
        UEXCB: UEXCB = 1 enables EXCB pin as the inverted excitation output. Only one of the EXCB or the EXCB bits
            should be set for proper operation.
        EXCA: EXCA = 1 enables EXCA pin as the excitation output.
        UEXCA: EXCA = 1 enables EXCA pin as the inverted excitation output. Only one of the EXCA or the EXCA bits should
            be set for proper operation.
        EXCLVL1, EXCLVL0 : Excitation Voltage Level.

            ======= ======= ============== ================= ==================
            EXCLVL1 EXCLVL0 Voltage on Cap EXC Pin Low Level EXC Pin High Level
            ======= ======= ============== ================= ==================
               0    0          ±VDD/8          VDD × 3/8           VDD × 5/8
               0    1          ±VDD/4          VDD × 1/4           VDD × 3/4
               1    0          ±VDD × 3/8      VDD × 1/8           VDD × 7/8
               1    1          ±VDD/2          0                   VDD
            ======= ======= ============== ================= ==================

        :param val: AD7745.exc_setup_reg to set
        :return: None
        """
        value = 0
        if val.CLKCTRL:
            value += AD7745.EXC_SETUP_CLKCTRL
        if val.EXCON:
            value += AD7745.EXC_SETUP_EXCON
        if val.EXCB:
            value += AD7745.EXC_SETUP_EXCB
        if val.UEXCB:
            value += AD7745.EXC_SETUP_UEXCB
        if val.EXCA:
            value += AD7745.EXC_SETUP_EXCA
        if val.UEXCA:
            value += AD7745.EXC_SETUP_UEXCA
        if val.EXCLVL1:
            value += AD7745.EXC_SETUP_EXCLVL1
        if val.EXCLVL0:
            value += AD7745.EXC_SETUP_EXCLVL0
        self.write_byte(AD7745.EXC_SETUP_ADDR, value)

    @property
    def configuration(self) -> "AD7745.configuration_reg":
        """
        CONFIGURATION REGISTER: Address Pointer 0x0A, Default Value 0xA0
        VTF1, VTF0: Voltage/temperature channel digital filter setup—conversion time/update rate setup. The conversion
            times in this table are valid for the CLKCTRL = 0 in the EXC SETUP register. The conversion times are
            longer by a factor of two for the CLKCTRL = 1.

        +-----+------+---------------------+------------------+----------------------+
        |     |      |                      VTCHOP = 1                               |
        +-----+------+---------------------+------------------+----------------------+
        |VTF1 | VTF0 |Conversion Time (ms) | Update Rate (Hz) |  –3 dB Frequency (Hz)|
        +=====+======+=====================+==================+======================+
        |0    |  0   |   20.1              |     49.8         |      26.4            |
        +-----+------+---------------------+------------------+----------------------+
        |0    |  1   |   32.1              |     31.2         |      15.9            |
        +-----+------+---------------------+------------------+----------------------+
        |1    |  0   |   62.1              |     16.1         |      8.0             |
        +-----+------+---------------------+------------------+----------------------+
        |1    |  1   |   122.1             |     8.2          |      4.0             |
        +-----+------+---------------------+------------------+----------------------+

        CAPF2, CAPF1, CAPF0: Capacitive channel digital filter setup—conversion time/update rate setup. The conversion
            times in this table are valid for the CLKCTRL = 0 in the EXC SETUP register. The conversion times are longer
            by factor of two for the CLKCTRL = 1.

        ====== ======= ======= ======================  ================ ======================
                                                     CAP CHOP = 0
        ------ ------- ------- ---------------------------------------------------------------
        CAPF2   CAPF1   CAPF0   Conversion Time (ms)    Update Rate     –3 dB Frequency (Hz)
        ====== ======= ======= ======================  ================ ======================
        0       0       0       11.0                    90.9            87.2
        0       0       1       11.9                    83.8            79.0
        0       1       0       20.0                    50.0            43.6
        0       1       1       38.0                    26.3            21.8
        1       0       0       62.0                    16.1            13.1
        1       0       1       77.0                    13.0            10.5
        1       1       0       92.0                    10.9            8.9
        1       1       1       109.6                   9.1             8.0
        ====== ======= ======= ======================  ================ ======================

        MD2, MD1, MD0: Converter mode of operation setup.

        === === === ===============================================
        MD2 MD1 MD0 Mode
        === === === ===============================================
        0   0   0   Idle
        0   0   1   Continuous conversion
        0   1   0   Single conversion
        0   1   1   Power-Down
        1   0   0   \-
        1   0   1   Capacitance system offset calibration
        1   1   0   Capacitance or voltage system gain calibration
        1   1   1
        === === === ===============================================

        :param: None
        :return: AD7745.exc_setup_reg
        """
        read_string = self.read_byte(AD7745.CONFIG_ADDR, byte_len=3)[0]
        return AD7745.configuration_reg(
            VTFS1=bool(read_string & AD7745.CONFIG_VTFS1),
            VTFS0=bool(read_string & AD7745.CONFIG_VTFS0),
            CAPFS2=bool(read_string & AD7745.CONFIG_CAPFS2),
            CAPFS1=bool(read_string & AD7745.CONFIG_CAPFS1),
            CAPFS0=bool(read_string & AD7745.CONFIG_CAPFS0),
            MD2=bool(read_string & AD7745.CONFIG_MD2),
            MD1=bool(read_string & AD7745.CONFIG_MD1),
            MD0=bool(read_string & AD7745.CONFIG_MD0),
        )

    @configuration.setter
    def configuration(self, val: "AD7745.configuration_reg") -> None:
        """
        CONFIGURATION REGISTER: Address Pointer 0x0A, Default Value 0xA0
        VTF1, VTF0: Voltage/temperature channel digital filter setup—conversion time/update rate setup. The conversion
            times in this table are valid for the CLKCTRL = 0 in the EXC SETUP register. The conversion times are
            longer by a factor of two for the CLKCTRL = 1.

                                                VTCHOP = 1
            VTF1    VTF0    Conversion Time (ms)    Update Rate (Hz)    –3 dB Frequency (Hz)
             0       0       20.1                    49.8                26.4
             0       1       32.1                    31.2                15.9
             1       0       62.1                    16.1                8.0
             1       1       122.1                   8.2                 4.0

        CAPF2, CAPF1, CAPF0: Capacitive channel digital filter setup—conversion time/update rate setup. The conversion
            times in this table are valid for the CLKCTRL = 0 in the EXC SETUP register. The conversion times are
            longer by factor of two for the CLKCTRL = 1.

                                                CAP CHOP = 0
            CAPF2   CAPF1   CAPF0   Conversion Time (ms)    Update Rate     –3 dB Frequency (Hz)
              0       0       0       11.0                    90.9            87.2
              0       0       1       11.9                    83.8            79.0
              0       1       0       20.0                    50.0            43.6
              0       1       1       38.0                    26.3            21.8
              1       0       0       62.0                    16.1            13.1
              1       0       1       77.0                    13.0            10.5
              1       1       0       92.0                    10.9            8.9
              1       1       1       109.6                   9.1             8.0

        MD2, MD1, MD0: Converter mode of operation setup.

            MD2 MD1 MD0 Mode
             0   0   0   Idle
             0   0   1   Continuous conversion
             0   1   0   Single conversion
             0   1   1   Power-Down
             1   0   0   -
             1   0   1   Capacitance system offset calibration
             1   1   0   Capacitance or voltage system gain calibration
             1   1   1

        :param val: AD7745.exc_setup_reg to set
        :return: None
        """
        value = 0
        if val.VTFS1:
            value += AD7745.CONFIG_VTFS1
        if val.VTFS0:
            value += AD7745.CONFIG_VTFS0
        if val.CAPFS2:
            value += AD7745.CONFIG_CAPFS2
        if val.CAPFS1:
            value += AD7745.CONFIG_CAPFS1
        if val.CAPFS0:
            value += AD7745.CONFIG_CAPFS0
        if val.MD2:
            value += AD7745.CONFIG_MD2
        if val.MD1:
            value += AD7745.CONFIG_MD1
        if val.MD0:
            value += AD7745.CONFIG_MD0
        self.write_byte(AD7745.CONFIG_ADDR, value)

    @property
    def cap_daca(self) -> "AD7745.cap_daca_reg":
        """
        CAP DAC A REGISTER: Address Pointer 0x0B, Default Value 0x00
        DACAENA: DACAENA = 1 connects capacitive DACA to the positive capacitance input.
        DACA: DACA value, Code 0x00 ≈ 0 pF, Code 0x7F ≈ full range.

        :param: None
        :return: AD7745.cap_daca_reg
        """
        read_string = self.read_byte(AD7745.CAP_DAC_A_ADDR, byte_len=1)[0]
        return AD7745.cap_daca_reg(
            DACAENA=bool(read_string & AD7745.CAP_DAC_A_DACAENA),
            DACA7bit=int(read_string & AD7745.CAP_DAC_A_DACA7bit),
        )

    @cap_daca.setter
    def cap_daca(self, val: "AD7745.cap_daca_reg") -> None:
        """
        CAP DAC A REGISTER: Address Pointer 0x0B, Default Value 0x00
        DACAENA: DACAENA = 1 connects capacitive DACA to the positive capacitance input.
        DACA: DACA value, Code 0x00 ≈ 0 pF, Code 0x7F ≈ full range.

        :param val: AD7745.cap_daca_reg to set
        :return: None
        """
        if val.DACA7bit > 127 or val.DACA7bit < 0:
            raise ValueError(
                f"Provided DACA7bit ({val.DACA7bit}) is out of bounds, must be less than 127!"
            )
        if val.DACAENA:
            self.write_byte(AD7745.CAP_DAC_A_ADDR, (1 << 7) | val.DACA7bit)
        else:
            self.write_byte(AD7745.CAP_DAC_A_ADDR, val.DACA7bit)

    @property
    def cap_dacb(self) -> "AD7745.cap_dacb_reg":
        """
        CAP DAC B REGISTER: Address Pointer 0x0C, Default Value 0x00
        DACBENB: DACBENB = 1 connects capacitive DACB to the negative capacitance input.
        DACB: DACB value, Code 0x00 ≈ 0 pF, Code 0x7F ≈ full range.

        :param: None
        :return: AD7745.cap_dacb_reg
        """
        read_string = self.read_byte(self.CAP_DAC_B_ADDR, byte_len=1)[0]
        return AD7745.cap_dacb_reg(
            DACBENB=bool(read_string & AD7745.CAP_DAC_B_DACBENB),
            DACB7bit=int(read_string & AD7745.CAP_DAC_B_DACB7bit),
        )

    @cap_dacb.setter
    def cap_dacb(self, val: "AD7745.cap_dacb_reg") -> None:
        """
        CAP DAC B REGISTER: Address Pointer 0x0C, Default Value 0x00
        DACBENB: DACBENB = 1 connects capacitive DACB to the negative capacitance input.
        DACB: DACB value, Code 0x00 ≈ 0 pF, Code 0x7F ≈ full range.

        :param val: AD7745.cap_dacb_reg to set
        :return: None
        """
        if val.DACB7bit > 177 or val.DACB7bit < 0:
            raise ValueError(
                f"Provided DACA7bit ({val.DACB7bit}) is out of bounds, buts be less than 177!"
            )
        if val.DACBENB:
            self.write_byte(AD7745.CAP_DAC_B_ADDR, (1 << 7) | val.DACB7bit)
        else:
            self.write_byte(AD7745.CAP_DAC_B_ADDR, val.DACB7bit)

    @property
    def cap_offset(self) -> int:
        """
        CAP OFFSET CALIBRATION REGISTER: 16 Bits, Address Pointer 0x0D, 0x0E, Default Value 0x8000

        :param: None
        :return: (off_h << 8) + off_l, 16 bits int value
        """
        reg_val = self.read_byte(AD7745.CAP_OFFSET_H_ADDR, byte_len=2)
        off_h = reg_val[0]
        off_l = reg_val[1]

        return (off_h << 8) + off_l

    @cap_offset.setter
    def cap_offset(self, val: int) -> None:
        """
        CAP OFFSET CALIBRATION REGISTER: 16 Bits, Address Pointer 0x0D, 0x0E, Default Value 0x8000

        :param val: 16 bits int value to set
        :return: None
        """
        off_h = val >> 8
        off_l = val & 0xFF

        self.write_byte(AD7745.CAP_OFFSET_H_ADDR, off_h)
        self.write_byte(AD7745.CAP_OFFSET_L_ADDR, off_l)

    @property
    def cap_gain(self) -> int:
        """
        CAP GAIN CALIBRATION REGISTER: 16 Bits, Address Pointer 0x0F, 0x10, Default Value 0xXXXX

        :param: None
        :return: (grain_h << 8) + grain_l, 16 bits int value
        """
        reg_val = self.read_byte(AD7745.CAP_OFFSET_H_ADDR, byte_len=2)
        grain_h = reg_val[0]
        grain_l = reg_val[1]

        return (grain_h << 8) + grain_l

    @cap_gain.setter
    def cap_gain(self, val: int) -> None:
        """
        CAP GAIN CALIBRATION REGISTER: 16 Bits, Address Pointer 0x0F, 0x10, Default Value 0xXXXX

        :param val: 16 bits int value to set
        :return: None
        """
        grain_h = val >> 8
        grain_l = val & 0xFF

        self.write_byte(AD7745.CAP_GRAIN_H_ADDR, grain_h)
        self.write_byte(AD7745.CAP_GRAIN_L_ADDR, grain_l)

    @property
    def volt_gain(self) -> int:
        """
        VOLT GAIN CALIBRATION REGISTER: 16 Bits, Address Pointer 0x11,0x12, Default Value 0xXXXX

        :param: None
        :return: 16 bit int gain val
        """
        reg_val = self.read_byte(AD7745.VOLT_GRAIN_H_ADDR, byte_len=2)
        grain_h = reg_val[0]
        grain_l = reg_val[1]

        return (grain_h << 8) + grain_l

    @volt_gain.setter
    def volt_gain(self, val: int) -> None:
        """
        VOLT GAIN CALIBRATION REGISTER: 16 Bits, Address Pointer 0x11,0x12, Default Value 0xXXXX

        :param val: 16 bits int value to set
        :return: None
        """
        grain_h = val >> 8
        grain_l = val & 0xFF

        self.write_byte(AD7745.VOLT_GRAIN_H_ADDR, grain_h)
        self.write_byte(AD7745.VOLT_GRAIN_L_ADDR, grain_l)

    def __del__(self) -> None:
        """
        Close binho on del.

        :return: None
        """
        self.close()

    def close(self) -> None:
        """
        Close the binho adaptor.

        :return: None
        :rtype: None
        """
        self.i2cbus.close()

    @staticmethod
    def cap_val_to_pf(cap_val: int) -> float:
        """Convert full scale dac value to 0-8.1920pF val"""
        return cap_val * (4.096 / 0x800000)

    def get_cap_val(self, timeout: float = 10) -> float:
        """
        Waits until the sensor has fresh data then reads it

        Raises an exception if the sensor's EXCERR bit is set or it takes longer than the timeout

        :param timeout: Timeout in sec
        :return: Capacitance float
        """
        return AD7745.cap_val_to_pf(self.get_raw_cap_val(timeout=timeout))

    def get_raw_cap_val(self, timeout: float = 10) -> int:
        """
        Waits until the sensor has fresh data then reads it

        Raises an exception if the sensor's EXCERR bit is set or it takes longer than the timeout

        :param timeout: Timeout in sec
        :return: Raw Capacitance value
        """
        if not self.cont_conv:
            conf = self.configuration
            conf.MD0 = False
            conf.MD1 = True
            conf.MD2 = False
            self.configuration = conf
        start_time = time.time()
        while True:
            stat_reg = self.status
            if stat_reg.RDYCAP is False:
                return self.cap_data
            if stat_reg.EXCERR:
                self.i2cbus.setLEDRGB(255, 0, 0)
                raise IOError(
                    "Excitation output cannot be driven properly!\n"
                    "A possible reason is a short circuit or too high capacitance between the "
                    "excitation pin and ground "
                )
            if time.time() - start_time > timeout:
                self.i2cbus.setLEDRGB(255, 0, 0)
                raise IOError("Sensor was not ready in time!")

    def cap_input(self, input_setting: "AD7745.CapInput") -> None:
        """
        Configure the cap input setting, nominally CIN1/CIN2

        :param AD7745.CapInput input_setting: Input settin to use
        :return: None
        :raises ValueError: If the valus is invalid
        """
        # Read config and change only the specified values:
        conf_val = self.configuration

        if input_setting == AD7745.CapInput.CAP1_EXC1:
            self.cap_setup = AD7745.cap_setup_reg(
                CAPEN=True,  # Enable cap channels
                CIN2=False,  # Set input to CIN1
                CAPDIFF=False,  # Set single ended input
                CAPCHOP=False,  # Required for specified performance
            )
            self.exc_setup = AD7745.exc_setup_reg(
                CLKCTRL=False,  # Required for specified performance
                EXCON=False,  # Signal is present for cap/vin conversion
                EXCB=False,  # Disable EXCB
                UEXCB=False,  # Disable EXCB
                EXCA=True,  # Enable EXCA
                UEXCA=False,  # Normal not inverted operation
                EXCLVL0=True,  # Excitation Voltage Level set to hi
                EXCLVL1=True,  # Excitation Voltage Level
            )
            # Configure for Continuous conversion
            conf_val.MD0 = True
            conf_val.MD1 = False
            conf_val.MD2 = False
            self.configuration = conf_val
        elif input_setting == AD7745.CapInput.CAP2_EXC2:
            self.cap_setup = AD7745.cap_setup_reg(
                CAPEN=True,  # Enable cap channels
                CIN2=True,  # Set input to CIN2
                CAPDIFF=False,  # Set single ended input
                CAPCHOP=False,  # Required for specified performance
            )
            self.exc_setup = AD7745.exc_setup_reg(
                CLKCTRL=False,  # Required for specified performance
                EXCON=False,  # Signal is present for cap/vin conversion
                EXCB=True,  # Enable EXCB
                UEXCB=False,  # Normal not inverted operation
                EXCA=False,  # Enable EXCA
                UEXCA=False,  # Normal not inverted operation
                EXCLVL0=True,  # Excitation Voltage Level set to hi
                EXCLVL1=True,  # Excitation Voltage Level
            )
            # Configure for Continuous conversion
            conf_val.MD0 = True
            conf_val.MD1 = False
            conf_val.MD2 = False
            self.configuration = conf_val
        elif input_setting == AD7745.CapInput.CAP_DIFF_CIN1:
            self.cap_setup = AD7745.cap_setup_reg(
                CAPEN=True,  # Enable cap channels
                CIN2=False,  # Set input to CIN1
                CAPDIFF=False,  # Set single ended input
                CAPCHOP=False,  # Required for specified performance
            )
            self.exc_setup = AD7745.exc_setup_reg(
                CLKCTRL=False,  # Required for specified performance
                EXCON=False,  # Signal is present for cap/vin conversion
                EXCB=False,  # Disable B output
                UEXCB=True,  # Inverted EXCB operation
                EXCA=True,  # Enable EXCA
                UEXCA=False,  # Normal not inverted operation
                EXCLVL0=True,  # Excitation Voltage Level set to hi
                EXCLVL1=True,  # Excitation Voltage Level
            )
            # Configure for Continuous conversion
            conf_val.MD0 = True
            conf_val.MD1 = False
            conf_val.MD2 = False
            self.configuration = conf_val
        elif input_setting == AD7745.CapInput.CAP_DIFF_CIN2:
            self.cap_setup = AD7745.cap_setup_reg(
                CAPEN=True,  # Enable cap channels
                CIN2=True,  # Set input to CIN2
                CAPDIFF=False,  # Set single ended input
                CAPCHOP=False,  # Required for specified performance
            )
            self.exc_setup = AD7745.exc_setup_reg(
                CLKCTRL=False,  # Required for specified performance
                EXCON=False,  # Signal is present for cap/vin conversion
                EXCB=False,  # Disable B output
                UEXCB=True,  # Inverted EXCB operation
                EXCA=True,  # Enable EXCA
                UEXCA=False,  # Normal not inverted operation
                EXCLVL0=True,  # Excitation Voltage Level set to hi
                EXCLVL1=True,  # Excitation Voltage Level
            )
            if self.cont_conv:
                # Configure for Continuous conversion
                conf_val.MD0 = True
                conf_val.MD1 = False
                conf_val.MD2 = False
            else:
                # Configure for single shot conversion
                conf_val.MD0 = False
                conf_val.MD1 = True
                conf_val.MD2 = False
            self.configuration = conf_val
        else:
            raise ValueError(
                f"The provided input_setting {input_setting} is not a valid AD7745.CapInput value!"
            )

    def cap_filter(self, filter_setting: "AD7745.CapFilter") -> None:
        """
        Set the input filter setting, preserving the config reg.

        :param AD7745.CapFilter filter_setting: Filter setting, specified in readout Hz
        :return: None
        :raises ValueError: If the valus is invalid
        """
        # Read config and change only the specified values:
        conf_val = self.configuration
        if filter_setting == AD7745.CapFilter.CONV87Hz:
            conf_val.CAPFS2, conf_val.CAPFS1, conf_val.CAPFS0 = False, False, False
        elif filter_setting == AD7745.CapFilter.CONV79Hz:
            conf_val.CAPFS2, conf_val.CAPFS1, conf_val.CAPFS0 = False, False, True
        elif filter_setting == AD7745.CapFilter.CONV44Hz:
            conf_val.CAPFS2, conf_val.CAPFS1, conf_val.CAPFS0 = False, True, False
        elif filter_setting == AD7745.CapFilter.CONV22Hz:
            conf_val.CAPFS2, conf_val.CAPFS1, conf_val.CAPFS0 = False, True, True
        elif filter_setting == AD7745.CapFilter.CONV14Hz:
            conf_val.CAPFS2, conf_val.CAPFS1, conf_val.CAPFS0 = True, False, False
        elif filter_setting == AD7745.CapFilter.CONV11Hz:
            conf_val.CAPFS2, conf_val.CAPFS1, conf_val.CAPFS0 = True, False, True
        elif filter_setting == AD7745.CapFilter.CONV9Hz:
            conf_val.CAPFS2, conf_val.CAPFS1, conf_val.CAPFS0 = True, True, False
        elif filter_setting == AD7745.CapFilter.CONV8Hz:
            conf_val.CAPFS2, conf_val.CAPFS1, conf_val.CAPFS0 = True, True, True
        else:
            raise ValueError(
                f"The provided filter_setting {filter_setting} is not a valid AD7745.CapFilter value!"
            )
        self.configuration = conf_val

    def cal_low_val(self, st_val=0) -> int:
        """
        Function that sets the DAC value so the sensor has as much headroom as possible.
        This function returns the value that the capdac was set to.

        :return: Identified cap dat val
        """
        # Reset cap reg
        self.cap_daca_reg(DACAENA=True, DACA7bit=0)
        # Flush stale cap data values
        self.get_raw_cap_val()
        self.get_raw_cap_val()
        # Binary search
        start, end = st_val, 127
        while start <= 127:
            print(f"Cal {start}")
            self.cap_daca = self.cap_daca_reg(DACAENA=True, DACA7bit=start)
            self.get_raw_cap_val()
            # Check if consecutive readings are 0, and not full scale
            if self.get_raw_cap_val() > 0:
                start += 1
                continue
            else:
                break
        self.cap_daca = self.cap_daca_reg(DACAENA=True, DACA7bit=start - 1)
        self.get_raw_cap_val()
        if self.get_raw_cap_val() != 0 and self.get_raw_cap_val() != 0xFFFFFF:
            return start
        self.i2cbus.setLEDRGB(255, 0, 0)
        raise IOError(
            f"Failed to find a valid CDAC value!, last attempt was {start} @ {hex(self.get_raw_cap_val())}"
        )

    @property
    def cap_dac_lsb(self) -> float:
        """
        Calculates and returns the size of the capdac LSB based on AN-1585.

        This can range from 0.134pF to 0.173pF. (technically this is the min to typ variation,
        however the datasheet indicates that this parameter typically varies low)

        :return: LSB size of the CAPDAC in pF
        """
        if self._cap_dac_lsb is None:
            f_gain_cal = (2**16 + self.cap_gain) / 2**16
            c_ref = 4.096 * f_gain_cal
            c_dac_range = c_ref * 3.2
            c_dac_lsb = c_dac_range / 127
            self._cap_dac_lsb = c_dac_lsb
        return self._cap_dac_lsb


if __name__ == "__main__":
    adaptor = binhoHostAdapter.binhoHostAdapter("COM1")
    sensor = AD7745(adaptor)

    sensor.cap_input(AD7745.CapInput.CAP1_EXC1)

    print(sensor.get_cap_val())

    sensor.close()
