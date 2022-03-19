/*
 * Copyright (c) 2014 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Omar Naji
 */

#include "mem/drampower.hh"

#include "base/intmath.hh"
#include "mem/pim_lib.hh"
#include "sim/core.hh"

DRAMPower::DRAMPower(const DRAMCtrlParams* p, bool include_io) :
    powerlib(libDRAMPower(getMemSpec(p), include_io))
{
}

DRAMPower::DRAMPower(const PIMMediaCtrlParams* p, bool include_io) :
    powerlib(libDRAMPower(getMemSpec(p), include_io))
{
}

Data::MemArchitectureSpec
DRAMPower::getArchParams(const DRAMCtrlParams* p)
{
    Data::MemArchitectureSpec archSpec;
    archSpec.burstLength = p->burst_length;
    archSpec.nbrOfBanks = p->banks_per_rank;
    // One DRAMPower instance per rank, hence set this to 1
    archSpec.nbrOfRanks = 1;
    archSpec.dataRate = getDataRate(p);
    // For now we can ignore the number of columns and rows as they
    // are not used in the power calculation.
    archSpec.nbrOfColumns = 0;
    archSpec.nbrOfRows = 0;
    archSpec.width = p->device_bus_width;
    archSpec.nbrOfBankGroups = p->bank_groups_per_rank;
    archSpec.dll = p->dll;
    archSpec.twoVoltageDomains = hasTwoVDD(p);
    // Keep this disabled for now until the model is firmed up.
    archSpec.termination = false;
    return archSpec;
}

Data::MemArchitectureSpec
DRAMPower::getArchParams(const PIMMediaCtrlParams* p)
{
    Data::MemArchitectureSpec archSpec;
    archSpec.burstLength = p->burst_length;
    archSpec.nbrOfBanks = pim::BANKS_PER_RANK;
    // One DRAMPower instance per rank, hence set this to 1
    archSpec.nbrOfRanks = 1;
    archSpec.dataRate = getDataRate(p);
    // For now we can ignore the number of columns and rows as they
    // are not used in the power calculation.
    archSpec.nbrOfColumns = 0;
    archSpec.nbrOfRows = 0;
    archSpec.width = p->device_bus_width;
    archSpec.nbrOfBankGroups = p->bank_groups_per_rank;
    archSpec.dll = p->dll;
    archSpec.twoVoltageDomains = hasTwoVDD(p);
    // Keep this disabled for now until the model is firmed up.
    archSpec.termination = false;
    return archSpec;
}

Data::MemTimingSpec
DRAMPower::getTimingParams(const DRAMCtrlParams* p)
{
    // Set the values that are used for power calculations and ignore
    // the ones only used by the controller functionality in DRAMPower

    // All DRAMPower timings are in clock cycles
    Data::MemTimingSpec timingSpec;
    timingSpec.RC = divCeil((p->tRAS + p->tRP), p->tCK);
    timingSpec.RCD = divCeil(p->tRCD, p->tCK);
    timingSpec.RL = divCeil(p->tCL, p->tCK);
    timingSpec.RP = divCeil(p->tRP, p->tCK);
    timingSpec.RFC = divCeil(p->tRFC, p->tCK);
    timingSpec.RAS = divCeil(p->tRAS, p->tCK);
    // Write latency is read latency - 1 cycle
    // Source: B.Jacob Memory Systems Cache, DRAM, Disk
    timingSpec.WL = timingSpec.RL - 1;
    timingSpec.DQSCK = 0; // ignore for now
    timingSpec.RTP = divCeil(p->tRTP, p->tCK);
    timingSpec.WR = divCeil(p->tWR, p->tCK);
    timingSpec.XP = divCeil(p->tXP, p->tCK);
    timingSpec.XPDLL = divCeil(p->tXPDLL, p->tCK);
    timingSpec.XS = divCeil(p->tXS, p->tCK);
    timingSpec.XSDLL = divCeil(p->tXSDLL, p->tCK);

    // Clock period in ns
    timingSpec.clkPeriod = (p->tCK / (double)(SimClock::Int::ns));
    assert(timingSpec.clkPeriod != 0);
    timingSpec.clkMhz = (1 / timingSpec.clkPeriod) * 1000;
    return timingSpec;
}

Data::MemTimingSpec
DRAMPower::getTimingParams(const PIMMediaCtrlParams* p)
{
    // Set the values that are used for power calculations and ignore
    // the ones only used by the controller functionality in DRAMPower

    // All DRAMPower timings are in clock cycles
    // BenP: there are timing parameters not relevant
    // for the PIM, so we set them as 0.
    Data::MemTimingSpec timingSpec;
    timingSpec.RC = divCeil(0, p->tCK);
    timingSpec.RCD = divCeil(p->tRCD, p->tCK);
    timingSpec.RL = divCeil(p->tCL, p->tCK);
    timingSpec.RP = divCeil(p->tRP, p->tCK);
    timingSpec.RFC = divCeil(0, p->tCK);
    timingSpec.RAS = divCeil(0, p->tCK);
    // Write latency is read latency - 1 cycle
    // Source: B.Jacob Memory Systems Cache, DRAM, Disk
    timingSpec.WL = timingSpec.RL - 1;
    timingSpec.DQSCK = 0; // ignore for now
    timingSpec.RTP = divCeil(0, p->tCK);
    timingSpec.WR = divCeil(0, p->tCK);
    timingSpec.XP = divCeil(0, p->tCK);
    timingSpec.XPDLL = divCeil(0, p->tCK);
    timingSpec.XS = divCeil(0, p->tCK);
    timingSpec.XSDLL = divCeil(0, p->tCK);

    // Clock period in ns
    timingSpec.clkPeriod = (p->tCK / (double)(SimClock::Int::ns));
    assert(timingSpec.clkPeriod != 0);
    timingSpec.clkMhz = (1 / timingSpec.clkPeriod) * 1000;
    return timingSpec;
}

Data::MemPowerSpec
DRAMPower::getPowerParams(const DRAMCtrlParams* p)
{
    // All DRAMPower currents are in mA
    Data::MemPowerSpec powerSpec;
    powerSpec.idd0 = p->IDD0 * 1000;
    powerSpec.idd02 = p->IDD02 * 1000;
    powerSpec.idd2p0 = p->IDD2P0 * 1000;
    powerSpec.idd2p02 = p->IDD2P02 * 1000;
    powerSpec.idd2p1 = p->IDD2P1 * 1000;
    powerSpec.idd2p12 = p->IDD2P12 * 1000;
    powerSpec.idd2n = p->IDD2N * 1000;
    powerSpec.idd2n2 = p->IDD2N2 * 1000;
    powerSpec.idd3p0 = p->IDD3P0 * 1000;
    powerSpec.idd3p02 = p->IDD3P02 * 1000;
    powerSpec.idd3p1 = p->IDD3P1 * 1000;
    powerSpec.idd3p12 = p->IDD3P12 * 1000;
    powerSpec.idd3n = p->IDD3N * 1000;
    powerSpec.idd3n2 = p->IDD3N2 * 1000;
    powerSpec.idd4r = p->IDD4R * 1000;
    powerSpec.idd4r2 = p->IDD4R2 * 1000;
    powerSpec.idd4w = p->IDD4W * 1000;
    powerSpec.idd4w2 = p->IDD4W2 * 1000;
    powerSpec.idd5 = p->IDD5 * 1000;
    powerSpec.idd52 = p->IDD52 * 1000;
    powerSpec.idd6 = p->IDD6 * 1000;
    powerSpec.idd62 = p->IDD62 * 1000;
    powerSpec.vdd = p->VDD;
    powerSpec.vdd2 = p->VDD2;
    return powerSpec;
}

void
DRAMPower::getPowerParams(const PIMMediaCtrlParams* p, Data::MemorySpecification* memSpec)
{
    // All DRAMPower currents are in mA
    memSpec->memPowerSpec.idd0 = p->IDD0 * 1000;
    memSpec->memPowerSpec.idd02 = p->IDD02 * 1000;
    memSpec->memPowerSpec.idd2p0 = p->IDD2P0 * 1000;
    memSpec->memPowerSpec.idd2p02 = p->IDD2P02 * 1000;
    memSpec->memPowerSpec.idd2p1 = p->IDD2P1 * 1000;
    memSpec->memPowerSpec.idd2p12 = p->IDD2P12 * 1000;
    memSpec->memPowerSpec.idd2n = p->IDD2N * 1000;
    memSpec->memPowerSpec.idd2n2 = p->IDD2N2 * 1000;
    memSpec->memPowerSpec.idd3p0 = p->IDD3P0 * 1000;
    memSpec->memPowerSpec.idd3p02 = p->IDD3P02 * 1000;
    memSpec->memPowerSpec.idd3p1 = p->IDD3P1 * 1000;
    memSpec->memPowerSpec.idd3p12 = p->IDD3P12 * 1000;
    memSpec->memPowerSpec.idd3n = p->IDD3N * 1000;
    memSpec->memPowerSpec.idd3n2 = p->IDD3N2 * 1000;
    memSpec->memPowerSpec.idd4r = p->IDD4R * 1000;
    memSpec->memPowerSpec.idd4r2 = p->IDD4R2 * 1000;
    memSpec->memPowerSpec.idd4w = p->IDD4W * 1000;
    memSpec->memPowerSpec.idd4w2 = p->IDD4W2 * 1000;
    memSpec->memPowerSpec.idd5 = p->IDD5 * 1000;
    memSpec->memPowerSpec.idd52 = p->IDD52 * 1000;
    memSpec->memPowerSpec.idd6 = p->IDD6 * 1000;
    memSpec->memPowerSpec.idd62 = p->IDD62 * 1000;
    memSpec->memPowerSpec.vdd = p->VDD;
    memSpec->memPowerSpec.vdd2 = p->VDD2;
    memSpec->memPowerSpec.capacitance = memSpec->memoryType.getCapacitance();
    memSpec->memPowerSpec.ioPower = memSpec->memoryType.getIoPower();
    memSpec->memPowerSpec.wrOdtPower = memSpec->memoryType.getWrOdtPower();
    memSpec->memPowerSpec.termRdPower = memSpec->memoryType.getTermRdPower();
    memSpec->memPowerSpec.termWrPower = memSpec->memoryType.getTermWrPower();
}

Data::MemorySpecification
DRAMPower::getMemSpec(const DRAMCtrlParams* p)
{
    Data::MemorySpecification* memSpec = new Data::MemorySpecification("DDR4");
    memSpec->memArchSpec = getArchParams(p);
    memSpec->memTimingSpec = getTimingParams(p);
    memSpec->memPowerSpec = getPowerParams(p);
    return *memSpec;
}

Data::MemorySpecification
DRAMPower::getMemSpec(const PIMMediaCtrlParams* p)
{ // BenP: since we use DDR4 we set it here hardcoded, this is not good coding...
    Data::MemorySpecification* memSpec = new Data::MemorySpecification("DDR4");
    memSpec->memArchSpec = getArchParams(p);
    memSpec->memTimingSpec = getTimingParams(p);
    getPowerParams(p,memSpec);
    return *memSpec;
}


bool
DRAMPower::hasTwoVDD(const DRAMCtrlParams* p)
{
    return p->VDD2 == 0 ? false : true;
}

bool
DRAMPower::hasTwoVDD(const PIMMediaCtrlParams* p)
{
    return p->VDD2 == 0 ? false : true;
}

uint8_t
DRAMPower::getDataRate(const DRAMCtrlParams* p)
{
    uint32_t burst_cycles = divCeil(p->tBURST, p->tCK);
    uint8_t data_rate = p->burst_length / burst_cycles;
    // 4 for GDDR5
    if (data_rate != 1 && data_rate != 2 && data_rate != 4)
        fatal("Got unexpected data rate %d, should be 1 or 2 or 4\n");
    return data_rate;
}

uint8_t
DRAMPower::getDataRate(const PIMMediaCtrlParams* p)
{
    uint32_t burst_cycles = divCeil(p->tBURST, p->tCK);
    uint8_t data_rate = p->burst_length / burst_cycles;
    // 4 for GDDR5
    if (data_rate != 1 && data_rate != 2 && data_rate != 4)
        fatal("Got unexpected data rate %d, should be 1 or 2 or 4\n");
    return data_rate;
}
