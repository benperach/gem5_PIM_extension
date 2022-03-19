# Copyright (c) 2012-2018 ARM Limited
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
# Copyright (c) 2013 Amin Farmahini-Farahani
# Copyright (c) 2015 University of Kaiserslautern
# Copyright (c) 2015 The University of Bologna
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Authors: Andreas Hansson
#          Ani Udipi
#          Omar Naji
#          Matthias Jung
#          Erfan Azarkhish

from m5.params import *
from m5.proxy import *
from m5.objects.AbstractMemory import *
from m5.objects.QoSMemCtrl import *

# DRAMCtrl is a single-channel single-ported DRAM controller model
# that aims to model the most important system-level performance
# effects of a DRAM without getting into too much detail of the DRAM
# itself.
class PIMMediaCtrl(QoSMemCtrl):
    type = 'PIMMediaCtrl'
    cxx_header = "mem/pim_ctrl.hh"

    # single-ported on the system interface side, instantiate with a
    # bus in front of the controller for multiple ports
    port = SlavePort("Slave port")

    # the PIM is not part of the address space, so don't try to map
    # it to there. During the simulation we assosiate pages with the
    # PIM using special instructions.
    # in_addr_map is a member from AbstractMemory
    in_addr_map = False

    # the basic configuration of the controller architecture, note
    # that each entry corresponds to a burst for the specific DRAM
    # configuration (e.g. x32 with burst length 8 is 32 bytes) and not
    # the cacheline size or request/packet size
    write_buffer_size = Param.Unsigned(64, "Number of write queue entries")
    read_buffer_size = Param.Unsigned(32, "Number of read queue entries")

    # threshold in percent for when to forcefully trigger writes and
    # start emptying the write buffer
    write_high_thresh_perc = Param.Percent(85, "Threshold to force writes")

    # threshold in percentage for when to start writes if the read
    # queue is empty
    write_low_thresh_perc = Param.Percent(20, "Threshold to start writes")

    # minimum write bursts to schedule before switching back to reads
    min_writes_per_switch = Param.Unsigned(8, "Minimum write bursts before "
                                           "switching to reads")

    # enforce a limit on the number of accesses per row
    max_accesses_per_row = Param.Unsigned(16, "Max accesses per row before "
                                          "closing");

    # size of DRAM Chip in Bytes
    # device_size = Param.MemorySize("Size of DRAM chip")

    # pipeline latency of the controller and PHY, split into a
    # frontend part and a backend part, with reads and writes serviced
    # by the queues only seeing the frontend contribution, and reads
    # serviced by the memory seeing the sum of the two
    static_frontend_latency = Param.Latency("4ns", "Static frontend latency")
    static_backend_latency = Param.Latency("4ns", "Static backend latency")

    ###### Added by BenP ###########
    # throughput of OpenCAPI is 25GBps, so 1 byte is one over
    pim_Channel_Byte_Latency = Param.Latency("40ps",
                                "Static latency for the pim channle")
    power_Bin_Count = Param.Unsigned(1024,\
        "the number of bins for the power histogram for each PIM rank")
    #power_window_duration = Param.Latency("10us",\
    power_window_duration = Param.Latency("100ns",\
                "a periodic window to count energy&power.")
    ###### End Added Section #######

    # the physical organisation of the DRAM
    device_bus_width = Param.Unsigned(8,\
                        "data bus width in bits for each DRAM device/chip")
    burst_length = Param.Unsigned(8,"Burst length (BL) in beats")
    devices_per_rank = Param.Unsigned(8,"Number of devices/chips per rank")
    #ranks_per_channel = Param.Unsigned("Number of ranks per channel")

    # default to 0 bank groups per rank, indicating bank group architecture
    # is not used
    # update per memory class when bank group architecture is supported
    bank_groups_per_rank = Param.Unsigned(0, "Number of bank groups per rank")
    # banks_per_rank = Param.Unsigned(8, "Number of banks per rank")
    # only used for the address mapping as the controller by
    # construction is a single channel and multiple controllers have
    # to be instantiated for a multi-channel configuration
    channels = Param.Unsigned(8, "Number of channels")

    # rank id so we know where we are in the ranks
    rank_id = Param.Unsigned("Rank index");

    # Enable DRAM powerdown states if True. This is False by default due to
    # performance being lower when enabled
    enable_dram_powerdown = Param.Bool(False, "Enable powerdown states")

    # For power modelling we need to know if the DRAM has a DLL or not
    dll = Param.Bool(True, "DRAM has DLL or not")

    # DRAMPower provides in addition to the core power, the possibility to
    # include RD/WR termination and IO power. This calculation assumes some
    # default values. The integration of DRAMPower with gem5 does not include
    # IO and RD/WR termination power by default. This might be added as an
    # additional feature in the future.

    # timing behaviour and constraints - all in nanoseconds
    # timing parameters are taken from Nishil's CONCEPT paper
    # first, if they are not available in the paper then they
    # are taken from the DDR4_2400_8x8 data in DRAMCtrl.py.
    # We take from DDR4_2400_8x8 because it is the closest
    # to our configuration.

    # the base clock period of the DRAM
    tCK = Param.Latency("0.833ns","Clock period")

    # the time it takes to decode the address sent to the PIM
    # taken from Nishil's CONCEPT paper
    tDEC = Param.Latency("0.833ns","Address description time")

    # the time it takes to charge the MAT
    # taken from Nishil's CONCEPT paper
    tCHARGE = Param.Latency("0.833ns","MAT charge time")

    # the time it takes to read from a the MAT
    # taken from Nishil's CONCEPT paper
    tREAD = Param.Latency("9.163ns","RRAM read time")

    # the time it takes to precharge, which is biasing
    # the MAT to ground in RRAM
    # taken from Nishil's CONCEPT paper
    tPRE = Param.Latency("0.833ns", "RRAM precharge (bias to zero) time")

    # the time it takes to do a MAGIC operation
    # taken from Nishil's CONCEPT paper
    tLOGIC = Param.Latency("29.155ns","RRAM logic operation time")

    # the time it takes to do a MAGIC operation
    # taken from Nishil's CONCEPT paper
    tSET = Param.Latency("22.491ns","RRAM set time")

    # the time it takes to do a MAGIC operation
    # taken from Nishil's CONCEPT paper
    tRESET = Param.Latency("22.491ns","RRAM reset time")

    # the amount of time in nanoseconds from issuing an activate command
    # to the data being available in the row buffer for a read/write
    tRCD = Param.Latency("14.16ns","RAS to CAS delay")

    # the time from issuing a read/write command to seeing the actual data
    # taken from Nishil's CONCEPT paper
    tCL = Param.Latency("14.16ns","CAS latency")

    # minimum time between a precharge and subsequent activate
    tRP = Param.Latency("14.16ns","Row precharge time")

    # time to complete a burst transfer, typically the burst length
    # divided by two due to the DDR bus, but by making it a parameter
    # it is easier to also evaluate SDR memories like WideIO.
    # This parameter has to account for burst length.
    # Read/Write requests with data size larger than one full burst are broken
    # down into multiple requests in the controller
    # tBURST is equivalent to the CAS-to-CAS delay (tCCD)
    # With bank group architectures, tBURST represents the CAS-to-CAS
    # delay for bursts to different bank groups (tCCD_S)
    # taken from Nishil's CONCEPT paper
    tBURST = Param.Latency("3.332ns",
                "Burst duration (for DDR burst length / 2 cycles)")

    # CAS-to-CAS delay for bursts to the same bank group
    # only utilized with bank group architectures; set to 0 for default case
    # tBURST is equivalent to tCCD_S; no explicit parameter required
    # for CAS-to-CAS delay for bursts to different bank groups
    tCCD_L = Param.Latency("5ns", "Same bank group CAS to CAS delay")

    # Write-to-Write delay for bursts to the same bank group
    # only utilized with bank group architectures; set to 0 for default case
    # This will be used to enable different same bank group delays
    # for writes versus reads
    tCCD_L_WR = Param.Latency(Self.tCCD_L,
        "Same bank group Write to Write delay")

    # write-to-read, same rank turnaround penalty
    # According to Nishil's CONCEPT paper tWTR = 0
    tWTR = Param.Latency("0ns","Write to read, same rank switching time")

    # read-to-write, same rank turnaround penalty
    # According to Nishil's CONCEPT paper tRTW = 0
    tRTW = Param.Latency("0ns", "Read to write, same rank switching time")

    # minimum row activate to row activate delay time
    tRRD = Param.Latency("3.332ns","ACT to ACT delay")

    # only utilized with bank group architectures; set to 0 for default case
    tRRD_L = Param.Latency("4.9ns", "Same bank group ACT to ACT delay")

    # time window in which a maximum number of activates are allowed
    # to take place, set to 0 to disable
    # taken from Nishil's CONCEPT paper (tFAW)
    tXAW = Param.Latency("13.328ns","X activation window")
    activation_limit = Param.Unsigned(4,"Max number of activates in window")

    # Currently rolled into other params
    ######################################################################

    # tRC  - assumed to be tRAS + tRP

    # Power Behaviour and Constraints
    # DRAMs like LPDDR and WideIO have 2 external voltage domains. These are
    # defined as VDD and VDD2. Each current is defined for each voltage domain
    # separately. For example, current IDD0 is active-precharge current for
    # voltage domain VDD and current IDD02 is active-precharge current for
    # voltage domain VDD2.
    # By default all currents are set to 0mA. Users who are only interested in
    # the performance of DRAMs can leave them at 0.

    # Operating 1 Bank Active-Precharge current
    IDD0 = Param.Current("48mA", "Active precharge current")

    # Operating 1 Bank Active-Precharge current multiple voltage Range
    IDD02 = Param.Current("3mA", "Active precharge current VDD2")

    # Precharge Power-down Current: Slow exit
    IDD2P0 = Param.Current("0mA", "Precharge Powerdown slow")

    # Precharge Power-down Current: Slow exit multiple voltage Range
    IDD2P02 = Param.Current("0mA", "Precharge Powerdown slow VDD2")

    # Precharge Power-down Current: Fast exit
    IDD2P1 = Param.Current("25mA", "Precharge Powerdown fast")

    # Precharge Power-down Current: Fast exit multiple voltage Range
    IDD2P12 = Param.Current("0mA", "Precharge Powerdown fast VDD2")

    # Precharge Standby current
    IDD2N = Param.Current("34mA", "Precharge Standby current")

    # Precharge Standby current multiple voltage range
    IDD2N2 = Param.Current("0mA", "Precharge Standby current VDD2")

    # Active Power-down current: slow exit
    IDD3P0 = Param.Current("0mA", "Active Powerdown slow")

    # Active Power-down current: slow exit multiple voltage range
    IDD3P02 = Param.Current("0mA", "Active Powerdown slow VDD2")

    # Active Power-down current : fast exit
    IDD3P1 = Param.Current("37mA", "Active Powerdown fast")

    # Active Power-down current : fast exit multiple voltage range
    IDD3P12 = Param.Current("0mA", "Active Powerdown fast VDD2")

    # Active Standby current
    IDD3N = Param.Current("43mA", "Active Standby current")

    # Active Standby current multiple voltage range
    IDD3N2 = Param.Current("3mA", "Active Standby current VDD2")

    # Burst Read Operating Current
    IDD4R = Param.Current("135mA", "READ current")

    # Burst Read Operating Current multiple voltage range
    IDD4R2 = Param.Current("0mA", "READ current VDD2")

    # Burst Write Operating Current
    IDD4W = Param.Current("123mA", "WRITE current")

    # Burst Write Operating Current multiple voltage range
    IDD4W2 = Param.Current("0mA", "WRITE current VDD2")

    # Refresh Current
    IDD5 = Param.Current("250mA", "Refresh current")

    # Refresh Current multiple voltage range
    IDD52 = Param.Current("0mA", "Refresh current VDD2")

    # Self-Refresh Current
    IDD6 = Param.Current("30mA", "Self-refresh Current")

    # Self-Refresh Current multiple voltage range
    IDD62 = Param.Current("0mA", "Self-refresh Current VDD2")

    # Main voltage range of the DRAM
    VDD = Param.Voltage("1.2V", "Main Voltage Range")

    # Second voltage range defined by some DRAMs
    VDD2 = Param.Voltage("2.5V", "2nd Voltage Range")
