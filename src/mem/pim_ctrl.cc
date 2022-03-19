/*
 * Copyright (c) 2010-2018 ARM Limited
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
 * Copyright (c) 2013 Amin Farmahini-Farahani
 * All rights reserved.
 *
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
 * Authors: Ben Perach
 *          Andreas Hansson
 *          Ani Udipi
 *          Neha Agarwal
 *          Omar Naji
 *          Wendy Elsasser
 *          Radhika Jagtap
 *  This code is based on the dram_ctrl.cc, so the authors of the
 *  dram_ctrl.cc are also the authors of this file.
 */
#include "mem/pim_ctrl.hh"

#include "base/bitfield.hh"
#include "base/trace.hh"
#include "debug/PIM.hh"
#include "debug/PIMPower.hh"
#include "debug/PIMop.hh"
#include "debug/QOS.hh"
#include "sim/system.hh"

using namespace std;
using namespace Data;

PIMMediaCtrl::PIMMediaCtrl(const PIMMediaCtrlParams* p) :
    QoS::MemCtrl(p),
    port(name() + ".port", *this),
    isTimingMode(false),
    retryRdReq(false),
    retryWrReq(false),
    immediateSwitch(false),
    tmp_hasReadyPacket(false),
    tmp_nextServTick(MaxTick),
    nextReqEvent([this]{ processNextReqEvent(); }, name()),
    respondEvent([this]{ processRespondEvent(); }, name()),
    ADDR_MAT_MASK(
        getAddrMask(pim::ADDR_MAT_IDX,pim::LOG_MAT_PER_SUBARRY)),
    ADDR_CHIP_MASK(
        getAddrMask(pim::ADDR_CHIP_IDX,pim::LOG_CHIPS_PER_RANK)),
    ADDR_ROW_MASK(
        getAddrMask(pim::ADDR_ROW_IDX,pim::LOG_MAT_ROWS)),
    ADDR_COL_MASK(
        getAddrMask(pim::ADDR_COL_IDX,pim::LOG_MAT_COLS
                                        -pim::LOG_BYTE_SIZE)),
    ADDR_SUBARRAY_MASK(
        getAddrMask(pim::ADDR_SUBARRAY_IDX,
        pim::LOG_SUBARRY_PER_BANK_PER_CHIP)),
    ADDR_SUBARRYS_PER_PIMCTRL_MASK(
        getAddrMask(pim::ADDR_SUBARRAY_IDX,
        pim::LOG_SUBARRAYS_PER_PIMCTRL)),
    ADDR_BANK_MASK
        (getAddrMask(pim::ADDR_BANK_IDX,pim::LOG_BANKS_PER_RANK)),
    PIMRANKOFFSET_MASK(ADDR_MAT_MASK | ADDR_CHIP_MASK |
                        ADDR_ROW_MASK | ADDR_COL_MASK |
                        ADDR_SUBARRAY_MASK | ADDR_BANK_MASK),
    rank(new Rank(*this, p)),
    rankID(p->rank_id),
    //deviceSize(p->device_size),
    deviceBusWidth(p->device_bus_width),
    burstLength(p->burst_length),
    burstSize((devicesPerRank * burstLength * deviceBusWidth) / 8),
    bankGroupsPerRank(p->bank_groups_per_rank),
    bankGroupArch(p->bank_groups_per_rank > 0),
    readBufferSize(p->read_buffer_size),
    writeBufferSize(p->write_buffer_size),
    writeHighThreshold(writeBufferSize * p->write_high_thresh_perc / 100.0),
    writeLowThreshold(writeBufferSize * p->write_low_thresh_perc / 100.0),
    minWritesPerSwitch(p->min_writes_per_switch),
    opsThisTime(0),
    writesThisTime(0),
    readsThisTime(0),
    tDEC(p->tDEC), tCHARGE(p->tCHARGE), tREAD(p->tREAD), tPRE(p->tPRE),
    tLOGIC(p->tLOGIC), tSET(p->tSET), tRESET(p->tRESET),
    tCK(p->tCK), tRTW(p->tRTW), tBURST(p->tBURST),
    tCCD_L_WR(p->tCCD_L_WR),
    tCCD_L(p->tCCD_L), tRCD(p->tRCD), tCL(p->tCL), tRP(p->tRP),
    tRRD(p->tRRD),
    tRRD_L(p->tRRD_L), tXAW(p->tXAW),
    activationLimit(p->activation_limit),
    wrToRdDly(tDEC + tBURST + p->tWTR), rdToWrDly(tRTW + tBURST),
    thresholdScheduleCheckTime(10*tBURST),
    maxAccessesPerRow(p->max_accesses_per_row),
    frontendLatency(p->static_frontend_latency),
    backendLatency(p->static_backend_latency),
    nextBurstAt(0), prevArrival(0),
    nextReqTime(0),
    pimChannelByteLatency(p->pim_Channel_Byte_Latency),
    timeStampOffset(0),
    lastStatsResetTick(0)
{
    // sanity check
    fatal_if(!isPowerOf2(burstSize), "PIM burst size %d is not allowed, "
             "must be a power of two\n", burstSize);
    readQueue.resize(p->qos_priorities);
    writeQueue.resize(p->qos_priorities);

    // initialize arrays for the implementation of the PIM operation.
    // The arrays contain constant value, which are pre-calculated here.

    generateConstFieldAddr(ADDR_MAT_NUM_ARRAY,
              pim::ADDR_MAT_IDX,pim::LOG_MAT_PER_SUBARRY);
    generateConstFieldAddr(ADDR_CHIP_NUM_ARRAY,
              pim::ADDR_CHIP_IDX,pim::LOG_CHIPS_PER_RANK);
    generateConstFieldAddr(ADDR_ROW_NUM_ARRAY,
              pim::ADDR_ROW_IDX,pim::LOG_MAT_ROWS);
    generateConstFieldAddr(ADDR_COL_NUM_ARRAY,
              pim::ADDR_COL_IDX,pim::LOG_MAT_COLS-pim::LOG_BYTE_SIZE);
    generateConstFieldAddr(ADDR_SUBARRYS_PER_PIMCTRL_NUM_ARRAY,
              pim::ADDR_SUBARRAY_IDX, pim::LOG_SUBARRAYS_PER_PIMCTRL);

    // the page offset must include all the relevant bits for the PIMCtrl
    assert(pim::PAGEOFFSET_MASK == (ADDR_COL_MASK | ADDR_MAT_MASK |
                                    ADDR_CHIP_MASK | ADDR_ROW_MASK |
                                    ADDR_SUBARRYS_PER_PIMCTRL_MASK));

    // perform a basic check of the write thresholds
    if (p->write_low_thresh_perc >= p->write_high_thresh_perc)
        fatal("Write buffer low threshold %d must be smaller than the "
              "high threshold %d\n", p->write_low_thresh_perc,
              p->write_high_thresh_perc);

    // determine the rows per bank by looking at the total capacity
    uint64_t capacity = ULL(1) << ceilLog2(AbstractMemory::size());
/*
    // determine the dram actual capacity from the DRAM config in Mbytes
    uint64_t deviceCapacity = deviceSize / (1024 * 1024) * devicesPerRank *
        pim::RANK_PER_CHANNEL;

    // if actual DRAM size does not match memory capacity in system warn!
    if (deviceCapacity != capacity / (1024 * 1024))
        warn("DRAM device capacity (%d Mbytes) does not match the "
             "address range assigned (%d Mbytes)\n", deviceCapacity,
             capacity / (1024 * 1024));
*/
    DPRINTF(PIM, "Memory capacity %lld (%lld) bytes\n", capacity,
            AbstractMemory::size());
    // basic bank group architecture checks ->
    if (bankGroupArch) {
        // must have at least one bank per bank group
        if (bankGroupsPerRank > banksPerRank) {
            fatal("banks per rank (%d) must be equal to or larger than "\
                  "banks groups per rank (%d)\n",
                  banksPerRank, bankGroupsPerRank);
        }
        // must have same number of banks in each bank group
        if ((banksPerRank % bankGroupsPerRank) != 0) {
            fatal("Banks per rank (%d) must be evenly divisible by bank "\
                  "groups per rank (%d) for equal banks per bank group\n",
                  banksPerRank, bankGroupsPerRank);
        }
        // tCCD_L should be greater than minimal, back-to-back burst delay
        if (tCCD_L <= tBURST) {
            fatal("tCCD_L (%d) should be larger than tBURST (%d) when "\
                  "bank groups per rank (%d) is greater than 1\n",
                  tCCD_L, tBURST, bankGroupsPerRank);
        }
        // tCCD_L_WR should be greater than minimal, back-to-back burst delay
        if (tCCD_L_WR <= tBURST) {
            fatal("tCCD_L_WR (%d) should be larger than tBURST (%d) when "\
                  "bank groups per rank (%d) is greater than 1\n",
                  tCCD_L_WR, tBURST, bankGroupsPerRank);
        }
        // tRRD_L is greater than minimal, same bank group ACT-to-ACT delay
        // some datasheets might specify it equal to tRRD
        if (tRRD_L < tRRD) {
            fatal("tRRD_L (%d) should be larger than tRRD (%d) when "\
                  "bank groups per rank (%d) is greater than 1\n",
                  tRRD_L, tRRD, bankGroupsPerRank);
        }
    }

}

void
PIMMediaCtrl::init()
{
    MemCtrl::init();

    if (!port.isConnected()) {
        fatal("PIMMediaCtrl %s is unconnected!\n", name());
    }
}

void
PIMMediaCtrl::startup()
{
    // remember the memory system mode of operation
    isTimingMode = system()->isTimingMode();

    if (isTimingMode) {
        // timestamp offset should be in clock cycles for DRAMPower
        timeStampOffset = divCeil(curTick(), tCK);

        // shift the bus busy time sufficiently far ahead that we never
        // have to worry about negative values when computing the time for
        // the next request, this will add an insignificant bubble at the
        // start of simulation
        nextBurstAt = curTick() + tRP + tRCD;
    }
}

Tick
PIMMediaCtrl::recvAtomic(PacketPtr pkt)
{
    DPRINTF(PIM, "recvAtomic: %s 0x%x\n", pkt->cmdString(), pkt->getAddr());

    panic_if(pkt->cacheResponding(), "Should not see packets where cache "
             "is responding");

    // do the actual memory access and turn the packet into a response
    access(pkt);

    Tick latency = 0;
    if (pkt->hasData()) {
        // this value is not supposed to be accurate, just enough to
        // keep things going, mimic a closed page
        latency = tRP + tRCD + tCL;
    }
    return latency;
}

bool
PIMMediaCtrl::readQueueFull(unsigned int neededEntries) const
{
    /*DPRINTF(PIM, "Read queue limit %d, current size %d, "
            "entries needed %d\n",
            readBufferSize, totalReadQueueSize + respQueue.size(),
            neededEntries);*/

    auto rdsize_new = totalReadQueueSize + respQueue.size() + neededEntries;
    return rdsize_new > readBufferSize;
}

bool
PIMMediaCtrl::writeQueueFull(unsigned int neededEntries) const
{
    DPRINTF(PIM, "Write queue limit %d, current size %d, entries needed %d\n",
            writeBufferSize, totalWriteQueueSize, neededEntries);

    auto wrsize_new = (totalWriteQueueSize + neededEntries);
    return  wrsize_new > writeBufferSize;
}

PIMMediaCtrl::PIMPacket*
PIMMediaCtrl::decodeAddr(const PacketPtr pkt, Addr dramPktAddr, unsigned size,
                     bool isRead, bool isOp) const
{
    // decode the address based on the address mapping scheme
    uint8_t bank;
    // use a 64-bit unsigned during the computations as the row is
    // always the top bits, and check before creating the PIMPacket
    uint32_t subarray;
    uint32_t pimCtrl;
    uint32_t row;
    uint32_t col;

    // truncate the address to a DRAM burst, which makes it unique to
    // a specific column, row, bank, rank and channel.
    // however, if this is
    // Addr addr = dramPktAddr / burstSize;

    bank        = (uint8_t)pim::getAddrField(dramPktAddr,
                        pim::ADDR_BANK_IDX, pim::LOG_BANKS_PER_RANK);
    subarray    = (uint32_t)pim::getAddrField(dramPktAddr,
                                pim::ADDR_SUBARRAY_IDX,
                                pim::LOG_SUBARRY_PER_BANK_PER_CHIP);
    // since the pimCtrl is responsible for a single page, it's
    // index is outside of the pageoffset, and so the pimCtrl
    // index is in regular and virtual addresses in the same place.
    pimCtrl    = (uint32_t)pim::getPIMCtrlBankIdex(dramPktAddr);
    row         = (uint32_t)pim::getAddrField(dramPktAddr, pim::ADDR_ROW_IDX,
                                                        pim::LOG_MAT_ROWS);
    // this is the column of the MAT
    col         = (uint32_t)pim::getAddrField(dramPktAddr, pim::ADDR_COL_IDX,
                                pim::LOG_MAT_COLS-pim::LOG_BYTE_SIZE);

    assert(bank < banksPerRank);
    assert(row < pim::MAT_ROWS);
    assert(row < Bank::NO_ROW);
    assert(col < pim::MAT_COLS/pim::BYTE_SIZE);

    DPRINTF(PIM, "Address: 0x%X Bank %d Subarray %d Row %d Col %d\n",
            dramPktAddr, bank, subarray, row, col);

    // create the corresponding DRAM packet with the entry time and
    // ready time set to the current tick, the latter will be updated
    // later
    uint16_t bank_id = banksPerRank * rankID + bank;
    return new PIMPacket(pkt, isRead, isOp, bank, subarray, pimCtrl,
                            row, col, bank_id, dramPktAddr,
                            size, rank->banks[bank]);
}

void
PIMMediaCtrl::addToReadQueue(PacketPtr pkt, unsigned int pktCount)
{
    // only add to the read queue here. whenever the request is
    // eventually done, set the readyTime, and call schedule()
    assert(!pkt->isWrite());

    assert(pktCount != 0);

    // if the request size is larger than burst size, the pkt is split into
    // multiple DRAM packets
    // Note if the pkt starting address is not aligened to burst size, the
    // address of first DRAM packet is kept unaliged. Subsequent DRAM packets
    // are aligned to burst size boundaries. This is to ensure we accurately
    // check read packets against packets in write queue.
    Addr addr = pkt->getAddr();
    unsigned pktsServicedByWrQ = 0;
    BurstHelper* burst_helper = NULL;
    for (int cnt = 0; cnt < pktCount; ++cnt) {
        unsigned size = std::min((addr | (burstSize - 1)) + 1,
                        pkt->getAddr() + pkt->getSize()) - addr;
        readPktSize[ceilLog2(size)]++;
        readBursts++;
        masterReadAccesses[pkt->masterId()]++;

        // First check write buffer to see if the data is already at
        // the controller
        bool foundInWrQ = false;
        bool dependedOn = false;
        PIMPacket* dependedOnPIMPacket;
        Addr burst_addr = burstAlign(addr);
        Addr page_addr = addr & pim::PAGE_MASK;
        // if the burst address is not present then there is no need
        // looking any further
        if (isInWriteQueue.find(burst_addr) != isInWriteQueue.end() ||
            isPIMOpInWriteQueue.find(page_addr) !=
            isPIMOpInWriteQueue.end()) {
            // we go through all the write queue by the reverse order
            // of execute, i.e. start from the last priority queue,
            // and start from the last packet in each queue.
            for (auto vec = writeQueue.begin();
                     vec != writeQueue.end(); vec++) {
                for (auto p = vec->rbegin(); p != vec->rend(); p++) {
                    // check if the read is subsumed in the write queue
                    // packet we are looking.
                    // If this is an PIM op then check the entire page
                    // for the address. If it is subsumed in an PIM op then
                    // enqueue this packet and save the dependency.
                    // If we found a dependency in a write first, than we don't
                    // need to enqueue this read.
                    if ((*p)->isOp() &&
                        (((*p)->getAddr() & pim::PAGE_MASK) ==
                            (addr & pim::PAGE_MASK))
                    ){
                        foundInWrQ = false; // this is just for readability
                        readDependOnPIM++;
                        DPRINTF(PIM,
                                "Read to addr 0x%llX depended on "
                                "PIM op in write queue\n",
                                addr);
                        dependedOn = true;
                        dependedOnPIMPacket = *p;
                        break;
                    } else if ((*p)->getAddr() <= addr &&
                       ((addr + size) <= ((*p)->getAddr() + (*p)->size))) {
                        foundInWrQ = true;
                        servicedByWrQ++;
                        pktsServicedByWrQ++;
                        DPRINTF(PIM,
                                "Read to addr 0x%llX with size %d serviced by "
                                "write queue\n",
                                addr, size);
                        bytesReadWrQ += burstSize;
                        break;
                    }
                }
            }
        }

        // If not found in the write q, make a DRAM packet and
        // push it onto the read queue
        if (!foundInWrQ) {

            // Make the burst helper for split packets
            if (pktCount > 1 && burst_helper == NULL) {
                DPRINTF(PIM, "Read to addr 0x%llX translates to %d "
                        "dram requests\n", pkt->getAddr(), pktCount);
                burst_helper = new BurstHelper(pktCount);
            }

            PIMPacket* pim_pkt = decodeAddr(pkt, addr, size, true, false);
            pim_pkt->burstHelper = burst_helper;

            // assigning the dependencies, but only if there are any
            if (dependedOn){
                pim_pkt->dependedOn = true; // since we are here it is true
                dependedOnPIMPacket->dependeds.insert(pim_pkt);
            }

            assert(!readQueueFull(1));
            rdQLenPdf[totalReadQueueSize + respQueue.size()]++;

            DPRINTF(PIM, "Adding to read queue\n");

            readQueue[pim_pkt->qosValue()].push_back(pim_pkt);
            // add the address for the lists for future comparisons
            isInReadQueue.insert(burst_addr);
            is2MPageInReadQueue.insert(page_addr);

            ++rank->readEntries;

            // log packet
            logRequest(MemCtrl::READ, pkt->masterId(), pkt->qosValue(),
                       pim_pkt->addr, 1);

            // Update stats
            avgRdQLen = totalReadQueueSize + respQueue.size();

            // update the next request event according to the packet details
            if (updateNextReqEvent(*pim_pkt)) {
                DPRINTF(PIM, "Read request scheduled on input for tick"
                                " %lld\n", nextReqEvent.when());
            }
        }

        // Starting address of next dram pkt (aligend to burstSize boundary)
        addr = (addr | (burstSize - 1)) + 1;
    }

    // do the access now, to take care of coherency
    access(pkt);

    // If all packets are serviced by write queue, we send the response back
    if (pktsServicedByWrQ == pktCount) {
        Tick response_time = curTick() + frontendLatency +
            pim::OPENCAPI_READRESPPKT_SIZE*pimChannelByteLatency;
        // Here we reset the timing of the packet before sending it out.
        pkt->headerDelay = pkt->payloadDelay = 0;
        // queue the packet in the response queue to be sent out after
        // the static latency has passed
        port.schedTimingResp(pkt, response_time);
        return;
    }

    // Update how many split packets are serviced by write queue
    if (burst_helper != NULL)
        burst_helper->burstsServiced = pktsServicedByWrQ;
}

void
PIMMediaCtrl::addToWriteQueue(PacketPtr pkt, unsigned int pktCount)
{
    // only add to the write queue here. whenever the request is
    // eventually done, set the readyTime, and call schedule()
    assert(pkt->isWrite());

    // if the request size is larger than burst size, the pkt is split into
    // multiple DRAM packets
    Addr addr = pkt->getAddr();
    for (int cnt = 0; cnt < pktCount; ++cnt) {
        bool isPIM = pkt->isPIM();
        unsigned size = std::min((addr | (burstSize - 1)) + 1,
                        pkt->getAddr() + pkt->getSize()) - addr;

        if (isPIM){
            opBursts++;
            masterOpAccesses[pkt->masterId()]++;
        } else {
            writePktSize[ceilLog2(size)]++;
            writeBursts++;
            masterWriteAccesses[pkt->masterId()]++;
        }

        // see if we can merge with an existing item in the write
        // queue and keep track of whether we have merged or not
        bool merged = (!isPIM) &
            (isInWriteQueue.find(burstAlign(addr)) !=
            isInWriteQueue.end());
        // if merge is true (then this is a write packet) and
        // there is an PIM op that is to the same page than
        // we need to find if the write to merge with is before
        // or after the PIM op. If after, than we can merge.
        // But if the write to merge is before than we need to
        // insert the write packet to the write queue.
        if (merged &&
            isPIMOpInWriteQueue.find(addr & pim::PAGE_MASK) !=
            isPIMOpInWriteQueue.end()){
            // we go through all the write queue by the reverse order
            // of execute, i.e. start from the last priority queue,
            // and start from the last packet in in each queue.
            for (auto vec = writeQueue.begin();
                            vec != writeQueue.end(); vec++) {
                for (auto p = vec->rbegin(); p != vec->rend(); p++) {
                    if ((*p)->isOp() &&
                        (((*p)->addr & pim::PAGE_MASK) ==
                            (addr & pim::PAGE_MASK))){
                        // if we are here then the first dependency
                        // is on an PIM op, so we need to queue the
                        // packet.
                        merged = false;
                        break;
                    } else if ((*p)->addr <= addr &&
                       ((addr + size) <= ((*p)->addr + (*p)->size))) {
                        // if we are here then the first dependency
                        // is on a write, so we can merge the packet.
                        break;
                    }
                }
            }
        }

        // if the item cannot be merged we need to create a new write
        // and enqueue it
        if (!merged) {
            PIMPacket* pim_pkt = decodeAddr(pkt, addr, size, false, isPIM);

            // find in the readQueue if there is some dependency.
            // we go through all the read queues by the reverse order
            // of execute, i.e. start from the last priority queue,
            // and start from the last packet in each queue.
            // This feature does not appear in the regular DRAM module,
            // I don't know why since it appears to me that writes might
            // go over reads.
            if ((isPIM & (is2MPageInReadQueue.find(addr & pim::PAGE_MASK) !=
                        is2MPageInReadQueue.end()) ) |
                (!isPIM & (isInReadQueue.find(burstAlign(addr)) !=
                            isInReadQueue.end()))){
                for (auto vec = readQueue.begin() ;
                            vec != readQueue.end() ; vec++) {
                    for (auto p = vec->rbegin() ; p != vec->rend() ; p++) {
                        pim_pkt->dependedOn = true;
                        (*p)->dependeds.insert(pim_pkt);
                        break;
                    }
                }
            }

            assert(totalWriteQueueSize < writeBufferSize);
            wrQLenPdf[totalWriteQueueSize]++;

            DPRINTF(PIM, "Adding to write queue\n");

            writeQueue[pim_pkt->qosValue()].push_back(pim_pkt);
            // add this operation to the correct list for future comparison
            if (isPIM){
                isPIMOpInWriteQueue.insert(addr & pim::PAGE_MASK);
            } else {
                isInWriteQueue.insert(burstAlign(addr));
            }

            // log packet
            logRequest(MemCtrl::WRITE, pkt->masterId(), pkt->qosValue(),
                       pim_pkt->addr, 1);

            assert(totalWriteQueueSize ==
                isInWriteQueue.size() + isPIMOpInWriteQueue.size());

            // Update stats
            avgWrQLen = totalWriteQueueSize;

            // increment write entries of the rank
            ++rank->writeEntries;

            // update the next request event according to the packet details
            if (updateNextReqEvent(*pim_pkt)) {
                DPRINTF(PIM, "Write request scheduled on input\n");
            }
        } else {
            DPRINTF(PIM, "Merging write burst with existing queue entry\n");

            // keep track of the fact that this burst effectively
            // disappeared as it was merged with an existing one
            mergedWrBursts++;
        }

        // Starting address of next dram pkt (aligend to burstSize boundary)
        addr = (addr | (burstSize - 1)) + 1;
    }

    // we do not wait for the writes to be send to the actual memory,
    // but instead take responsibility for the consistency here and
    // snoop the write queue for any upcoming reads.
    if (pkt->isPIM()){
        // use this function to access the PIM with PIM ops.
        // The function assumes that pmemAddr is a pointer to the
        // host memory containing a PIMRankSize memory chunk.
        if (pim::PERFORM_PIM_OPS){
            accessPIM(pkt, pmemAddr);
        }
        if (pkt->needsResponse()) {
            pkt->makeResponse();
        }
    } else {
        access(pkt);
    };
}

void
PIMMediaCtrl::printQs() const
{
#if TRACING_ON
    DPRINTF(PIM, "===READ QUEUE===\n\n");
    for (const auto& queue : readQueue) {
        for (const auto& packet : queue) {
            DPRINTF(PIM, "Read %lu\n", packet->addr);
        }
    }

    DPRINTF(PIM, "\n===RESP QUEUE===\n\n");
    for (const auto& packet : respQueue) {
        DPRINTF(PIM, "Response %lu\n", packet->addr);
    }

    DPRINTF(PIM, "\n===WRITE QUEUE===\n\n");
    for (const auto& queue : writeQueue) {
        for (const auto& packet : queue) {
            DPRINTF(PIM, "Write %lu\n", packet->addr);
        }
    }
#endif // TRACING_ON
}

bool
PIMMediaCtrl::recvTimingReq(PacketPtr pkt)
{
    // This is where we enter from the outside world
    DPRINTF(PIM, "recvTimingReq: request %s addr 0x%llX size %d\n",
            pkt->cmdString(), pkt->getAddr(), pkt->getSize());

    // start the power event scheduling. We do this now because starting
    // the power event in the init of the rank cause errors when starting
    // from a checkpoint. We reach this point only when the first message
    // to the PIM is sent, so we need the power only from now.
    // Additionally, the many events that do nothing are affecting the
    // running time of gem5.
    // The start up only apply on the first time it is called, any other
    // time it does nothing.
    rank->startUpPowerStatsEvent();

    // Calc avg gap between requests
    if (prevArrival != 0) {
        totGap += curTick() - prevArrival;
    }
    prevArrival = curTick();

    // Find out how many dram packets a pkt translates to
    // If the burst size is equal or larger than the pkt size, then a pkt
    // translates to only one dram packet. Otherwise, a pkt translates to
    // multiple dram packets
    // BenP: the size of the pim packet payload is 8Byte, so set it that way
    // for calculating the pim_pkt_num. Also, the offset is zero, since
    // an PIM operation packet is not for a specific address.
    unsigned size = pim::PIM_PKT_SIZE;
    unsigned offset = 0;
    if (!pkt->isPIM()){
        size = pkt->getSize();
        offset = pkt->getAddr() & (burstSize - 1);
    }
    unsigned int pim_pkt_count = divCeil(offset + size, burstSize);
    assert(pim_pkt_count > 0);
    // run the QoS scheduler and assign a QoS priority value to the packet
    // BenP: this line doest seem to do anything. All parameters associate
    // with QoS are turned off always.
    qosSchedule( { &readQueue, &writeQueue }, burstSize, pkt);

    // check local buffers and do not accept if full
    if (pkt->isWrite()) {
        assert(size != 0);
        if (writeQueueFull(pim_pkt_count)) {
            DPRINTF(PIM, "Write queue full, not accepting\n");
            // remember that we have to retry this port
            retryWrReq = true;
            numWrRetry++;
            return false;
        } else {
            addToWriteQueue(pkt, pim_pkt_count);
            if (pkt->isPIM()){
                numOpReqs++;
            } else {
                writeReqs++;
                bytesWrittenSys += size;
            }
        }
    } else {
        assert(pkt->isRead());
        assert(size != 0);
        if (readQueueFull(pim_pkt_count)) {
            DPRINTF(PIM, "Read queue full, not accepting\n");
            // remember that we have to retry this port
            retryRdReq = true;
            numRdRetry++;
            return false;
        } else {
            addToReadQueue(pkt, pim_pkt_count);
            readReqs++;
            bytesReadSys += size;
        }
    }

    return true;
}

void
PIMMediaCtrl::processRespondEvent()
{
    DPRINTF(PIM,
            "processRespondEvent(): Some req has reached its readyTime\n");
    assert(!respQueue.empty());
    PIMPacket* pim_pkt = respQueue.front();

    // if a read has reached its ready-time, decrement the number of reads
    if (pim_pkt->isRead()) {
        --rank->readEntries;
        // 20 is to be on the safe side
        char data_s [20*pim::CACHEBLOCK_SIZE];
        int cx = 0;
        for (int i = 0 ; i < pim::CACHEBLOCK_SIZE ; i++){
            cx += snprintf(data_s + cx ,20," 0x%X",
            *(pim_pkt->pkt->getPtr<char>()+i));
        }
        DPRINTF(PIM,"Sending a read response, address 0x%X \n\t data: %s\n",
                pim_pkt->getAddr(),data_s);
        DPRINTF(PIM, "number of read entries for rank %d is %d\n",
                rankID, rank->readEntries);
    } else {
        --rank->writeEntries;
        DPRINTF(PIM,"Sending a write response, address 0x%X\n",
                pim_pkt->getAddr());
        DPRINTF(PIM, "number of write entries for rank %d is %d\n",
                rankID, rank->writeEntries);
    }

    // Note that OpenCAPI enable to bundle the headers
    // of several packets together, so they are sent all
    // at once and with a single DataLink layer header.
    // We do not implement that bundling. We send each packet at
    // its own, without regarding the basic flit size (64bytes).
    // In OpenCAPI, if the flit (called data carriers) is not filled
    // then the extra space is wasted. We do not take that into account,
    // However, since we do not bundle the headers together this create
    // an effective bundling (since the total time of sending is as if
    // we bundle headers together). However, we add a DataLink header
    // to each packet, so this is an overhead not necessarily
    // add in OpenCAPI.
    Tick response_time = curTick() + frontendLatency +
        pimChannelByteLatency *
        (pim_pkt->isRead() ?
            pim::OPENCAPI_READRESPPKT_SIZE :
            pim::OPENCAPI_WRITERESPPKT_SIZE);

    if (pim_pkt->burstHelper) {
        // it is a split packet
        pim_pkt->burstHelper->burstsServiced++;
        if (pim_pkt->burstHelper->burstsServiced ==
            pim_pkt->burstHelper->burstCount) {
            // we have now serviced all children packets of a system packet
            // so we can now respond to the requester
            // @todo we probably want to have a different front end and back
            // end latency for split packets
            // Here we reset the timing of the packet before sending it out.
            pim_pkt->pkt->headerDelay = pim_pkt->pkt->payloadDelay = 0;
            // queue the packet in the response queue to be sent out after
            // the static latency has passed
            port.schedTimingResp(pim_pkt->pkt, response_time);
            delete pim_pkt->burstHelper;
            pim_pkt->burstHelper = NULL;
        }
    } else {
        // it is not a split packet
        // Here we reset the timing of the packet before sending it out.
        pim_pkt->pkt->headerDelay = pim_pkt->pkt->payloadDelay = 0;
        //make the packet a response packet
        DPRINTF(PIM,"type %s , isResponse %d, isPIM %d\n"
                , pim_pkt->pkt->cmdString(), pim_pkt->pkt->isResponse()
                , pim_pkt->isOp());
        // queue the packet in the response queue to be sent out after
        // the static latency has passed
        port.schedTimingResp(pim_pkt->pkt, response_time);
    }

    delete respQueue.front();
    respQueue.pop_front();

    if (!respQueue.empty()) {
        assert(respQueue.front()->readyTime >= curTick());
        assert(!respondEvent.scheduled());
        schedule(respondEvent, respQueue.front()->readyTime);
    }
    // We have made a location in the queue available at this point,
    // so if there is a read that was forced to wait, retry now
    if (retryRdReq) {
        retryRdReq = false;
        port.sendRetryReq();
    }
}

pair<PIMMediaCtrl::PIMPacketQueue::iterator, Tick>
PIMMediaCtrl::chooseNext(PIMPacketQueue& queue, Tick extra_col_delay)
{
    // This method does the arbitration between requests.
    // and return the evaluated time for the found packet
    Tick pkt_time = curTick() + thresholdScheduleCheckTime;

    PIMMediaCtrl::PIMPacketQueue::iterator ret = queue.end();

    if (!queue.empty()) {
        if (queue.size() == 1) {
            PIMPacket* pim_pkt = *(queue.begin());
            if (!pim_pkt->dependedOn) {
                ret = queue.begin();
                DPRINTF(PIM, "Single request, going to a rank\n");
            } else {
                DPRINTF(PIM, "Single request, but it is depended\n");
            }
        } else {
            std::tie(ret,pkt_time) = chooseNextFRFCFS(queue, extra_col_delay);
        }
    }
    return make_pair(ret,pkt_time);
}

pair<PIMMediaCtrl::PIMPacketQueue::iterator,Tick>
PIMMediaCtrl::chooseNextFRFCFS(PIMPacketQueue& queue, Tick extra_col_delay)
{
    auto selected_pkt_it = queue.end();

    // time we need to issue a column command to be seamless
    const Tick min_col_at = std::max(nextBurstAt + extra_col_delay, curTick());

    // check if this is a read queue
    bool read = (*queue.begin())->isRead();

    // if the time is too long then we need to wait
    // since we don't want to set the media controller
    // with a packet that can start a long time from now
    // but it is the closest packet (what if new packets arrive
    // in the meantime that can be issued?)
    Tick min_time = MaxTick;

    for (auto i = queue.begin(); i != queue.end() ; ++i) {
        PIMPacket* pim_pkt = *i;
        const Bank& bank = pim_pkt->bankRef;
        const PIMCtrl* pimCtrl = bank.pimCtrls[pim_pkt->pimCtrl];
        Tick col_allowed_at = read ? bank.rdAllowedAt :
                                           bank.wrAllowedAt;
        col_allowed_at = std::max(pimCtrl->opAllowedAt,col_allowed_at);

        DPRINTF(PIM, "%s checking packet in bank %d\n",
                __func__, pim_pkt->bankRef.bank);

        // check if the packet is depended on another packet, if it is,
        // jump to the next packet
        if (!pim_pkt->dependedOn) {

            //TBD: change print
            DPRINTF(PIM,
                    "%s bank %d available\n", __func__,
                    pim_pkt->bankRef.bank);

            // check if it is a row hit, this is only valid for reads
            // in the PIM
            if (read &&
                bank.openRow == pim_pkt->row &&
               (bank.openCol <= pim_pkt->col &&
                pim_pkt->col < bank.openCol +
                                pim::NUM_OF_SA_PER_MAT/pim::BYTE_SIZE)
             )
                {
                // no additional rank-to-rank or same bank-group
                // delays, or we switched read/write and might as well
                // go for the row hit
                if (col_allowed_at <= min_col_at) {
                    // FCFS within the hits, giving priority to
                    // commands that can issue seamlessly, without
                    // additional delay, such as same rank accesses
                    // and/or different bank-group accesses
                    DPRINTF(PIM, "%s Seamless row buffer hit\n", __func__);
                    selected_pkt_it = i;
                    min_time = min_col_at;
                    // no need to look through the remaining queue entries
                    break;
                } else {
                    min_time = col_allowed_at;
                    selected_pkt_it = i;
                    DPRINTF(PIM, "%s buffer hit, set new min time %d\n",
                        __func__,min_time);
                }
            } else {
                // check if this is the first packet to issue
                DPRINTF(PIM, "%s buffer miss\n", __func__);
                if (col_allowed_at < min_time) {
                    min_time = col_allowed_at;
                    selected_pkt_it = i;
                    DPRINTF(PIM, "%s set new min time %d\n",
                        __func__,min_time);
                }
            }
        } else {
            //TBD: change print
            DPRINTF(PIM, "%s bank %d not available\n", __func__,
                    pim_pkt->bankRef.bank);
        }
    }

    //TBD: change print
    if (selected_pkt_it == queue.end()) {
        DPRINTF(PIM, "%s no available ranks found\n", __func__);
    }

    return make_pair(selected_pkt_it, min_time);
}

inline bool
PIMMediaCtrl::updateNextReqEvent(Tick pkt_at)
{
    // if there is no event then set one.
    // if there is an event, then check if
    // this is earlier then the suggested packet
    bool hasChanged = false;
    pkt_at = std::max(curTick(),pkt_at);
    if (!nextReqEvent.scheduled()) {
        DPRINTF(PIM, "restart the nextReqEvent\n");
        schedule(nextReqEvent, pkt_at);
        hasChanged = true;
        DPRINTF(PIM,"updateNextReqEvent: pkt_at = %lld\n", pkt_at);
    } else if (pkt_at < nextReqEvent.when()) {
        reschedule(nextReqEvent, pkt_at);
        hasChanged = true;
        DPRINTF(PIM,"updateNextReqEvent: pkt_at = %lld\n", pkt_at);
    }
    return hasChanged;
}

inline bool
PIMMediaCtrl::updateNextReqEvent(PIMPacket& pim_pkt)
{
    // get the estimated timing of the packet
    Tick pkt_at = pim_pkt.isRead() ? pim_pkt.bankRef.rdAllowedAt:
                                        pim_pkt.bankRef.wrAllowedAt;
    pkt_at = std::max(pkt_at,
            pim_pkt.bankRef.pimCtrls[pim_pkt.pimCtrl]->opAllowedAt);
    // see if we can change the timing of the NextReqEvent.
    // We don't need to check for dependencies, since if there is a
    // dependence then there is a packet that is due sometime to the
    // same location, and the event should be scheduled to at least the
    // time of this packet.
    bool hasChanged = updateNextReqEvent(pkt_at);
    // change the bus if we actually updated
    if (hasChanged)
        busStateNext = pim_pkt.isRead() ? BusState::READ : BusState::WRITE;
        DPRINTF(PIM,"updateNextReqEvent: rdAllowedAt = %lld "
             "wrAllowedAt = %lld "
             "opAllowedAt = %lld\n",
             pim_pkt.bankRef.rdAllowedAt,
             pim_pkt.bankRef.wrAllowedAt,
             pim_pkt.bankRef.pimCtrls[pim_pkt.pimCtrl]->opAllowedAt);
    return hasChanged;
}

void
PIMMediaCtrl::doDRAMAccess(PIMPacket* pim_pkt)
{
    DPRINTF(PIM, "Timing access to addr 0x%X, rank/bank/row %d %d %d\n",
            pim_pkt->addr, rankID, pim_pkt->bank, pim_pkt->row);

    // instead of multiple access save the results
    bool read = pim_pkt->isRead();
    bool op = pim_pkt->isOp();

    // get the bank
    Bank& bank = pim_pkt->bankRef;
    // get the PIM ctrl
    assert(pim_pkt->row <= pim::SUBARRY_PER_BANK_PER_CHIP*pim::MAT_ROWS);
    PIMCtrl* pimCtrl = bank.pimCtrls[pim_pkt->pimCtrl];

    // A closed page policy is assumed.
    // According to Nishil's paper (CONCEPT) there is no precharge event
    // as in DRAM, there is another precharge that happen after every
    // operation, but it is simply biasing the MATs WL and BL to ground.

    // for the state we need to track if it is a row hit or not
    bool row_hit = true;
    if (    (!op && bank.openRow == pim_pkt->row &&
            (bank.openCol <= pim_pkt->col &&
            pim_pkt->col < bank.openCol +
                            pim::NUM_OF_SA_PER_MAT/pim::BYTE_SIZE)
            ) ||
            ( op && (pim_pkt->pimCtrl == bank.openRowPIM))
        )
    {
            // if this is a write or and op to the opened data,
            // then we have to close stuff up for consistency issues
            if (!read || op){
                bank.openRow = Bank::NO_ROW;
                bank.openRow = Bank::NO_COL;
                bank.openRowPIM = Bank::NO_ROW;
                if (op) { opRowHits++; }
                else { writeRowHits++; }
                row_hit = false;
            } else {
                readRowHits++;
                DPRINTF(PIM,"Row hit on read request\n");
            }
    } else {
        row_hit = false;
        // if there is no row hit on a read then update the buffed data info
        if (read){
            bank.openRow = pim_pkt->row;
            bank.openCol = pim_pkt->col;
            bank.openRowPIM = pim_pkt->pimCtrl;
        }
    }

    // respect any constraints on the command (e.g. tRCD or tCCD)
    Tick col_allowed_at = read ?
                                bank.rdAllowedAt : bank.wrAllowedAt;
    // account the fact that if an PIM is during an operation then
    // we can't send until it is finished.
    col_allowed_at = std::max(pimCtrl->opAllowedAt,col_allowed_at);


    // we need to wait until the bus is available before we can issue
    // the command; need minimum of tBURST between commands
    Tick cmd_at = std::max({col_allowed_at, nextBurstAt, curTick()});

    // packet data pointer
    const uint64_t* dataP = pim_pkt->pkt->getConstPtr<uint64_t>();

    commandType command;
    // update the packet ready time according to Nishil's paper
    // and set the command type for the power estimation
    if (read) {
        // if there is a row hit we can get the data from there.
        if (row_hit){
            command = commandType::READ_H;
            pim_pkt->readyTime = cmd_at + tCL + tBURST;
            nextBurstAt = cmd_at + tBURST;
        } else {
            command = commandType::READ_M;
            // there is no tPRE because it is included in the tCL and tBURST
            pim_pkt->readyTime = cmd_at + tDEC + tCHARGE +
                                    tREAD + tCL + tBURST;
            nextBurstAt = cmd_at +
                            std::max(tDEC + tCHARGE,tREAD + tCL + tBURST);
        }
        // Update bus state to reflect when previous command was issued
    } else if (op) {
        // the packet is ready when we subbmit it to the PIMCtrl
        // but the operation in the PIM will take longer according
        // to the operation
        pim_pkt->readyTime = cmd_at + tDEC + tBURST;
        pimCtrl->opAllowedAt = pim_pkt->readyTime +
            pim::pimOpCycles(dataP)*(tCHARGE+tLOGIC) + tPRE;
        pimCtrl->totalOpCycles += pim::pimOpCycles(dataP);
        DPRINTF(PIMop,"op Exe: cmd_at = %lld, opAllowedAt = %lld,"
                        " pimOpCycles = %lld, tCHARGE = %lld,"
                        " tLOGIC = %lld\n",
                        cmd_at,
                        pimCtrl->opAllowedAt,pim::pimOpCycles(dataP),
                        tCHARGE,tLOGIC);
        DPRINTF(PIMop,"op details: opCode = 0x%X, src1len = 0x%X,"
                        " src2len = 0x%X, imm = 0x%X, imm1 = 0x%X,"
                        " imm2 = 0x%X\n",
                pim::getFiled(*dataP,
                            pim::OPCODE_MASK,pim::OPCODE_BIT_START),
                pim::getFiled(*dataP,
                            pim::SRC1LEN_MASK,pim::SRC1LEN_BIT_START)+1,
                pim::getFiled(*dataP,
                            pim::SRC2LEN_MASK,pim::SRC2LEN_BIT_START)+1,
                pim::getFiled(*dataP,
                            pim::IMM_MASK,pim::IMM_BIT_START),
                pim::getFiled(*dataP,
                            pim::IMM1_MASK,pim::IMM1_BIT_START),
                pim::getFiled(*dataP,
                            pim::IMM2_MASK,pim::IMM2_BIT_START)
                );
        // set the command for power calculation according to
        // whether it is for a single cell at a time or full
        // parallel
        command = (pimFullParallel(dataP)) ?
            commandType::OP_FULL_N : commandType::OP_SNGL_N;
        nextBurstAt = cmd_at + tBURST;
    } else { //this is a write
        pim_pkt->readyTime = cmd_at + tBURST + tDEC + 2*tCHARGE +
        tSET + tRESET + tPRE;
        command = commandType::WRITE;
        nextBurstAt = pim_pkt->readyTime;
    }

    // update the time for the next read/write burst for each
    // bank (add a max with tCCD/tCCD_L/tCCD_L_WR here)
    Tick dly_to_rd_cmd;
    Tick dly_to_wr_cmd;
    for (int i = 0; i < banksPerRank; i++) {
        // next burst to same bank group in this rank must not happen
        // before tCCD_L.  Different bank group timing requirement is
        // tBURST;
        if (bankGroupArch &&
           (bank.bankgr == rank->banks[i].bankgr)) {
            // bank group architecture requires longer delays between
            // RD/WR burst commands to the same bank group.
            // tCCD_L is default requirement for same BG timing
            // tCCD_L_WR is required for write-to-write
            // Need to also take bus turnaround delays into account
            dly_to_rd_cmd = read ?
                 std::max(tCCD_L,tRRD) : std::max(tCCD_L, wrToRdDly);
            dly_to_wr_cmd = read ?
                            std::max(tCCD_L, rdToWrDly) : tCCD_L_WR;
        } else {
            // tBURST is default requirement for diff BG timing
            // Need to also take bus turnaround delays into account
            dly_to_rd_cmd = read ? std::max(tBURST,tRRD) : wrToRdDly;
            dly_to_wr_cmd = read ? rdToWrDly : tBURST;
        }
        rank->banks[i].rdAllowedAt = std::max(cmd_at + dly_to_rd_cmd,
                                         rank->banks[i].rdAllowedAt);
        rank->banks[i].wrAllowedAt = std::max(cmd_at + dly_to_wr_cmd,
                                         rank->banks[i].wrAllowedAt);
    }

    // increment the bytes accessed and the accesses per row
    if (op){
        numTypeOpReqs[pim::getFiled(*dataP,pim::OPCODE_MASK,
                                    pim::OPCODE_BIT_START)]++;
    } else {
        bank.bytesAccessed += burstSize;
        ++bank.rowAccesses;
    }

    // DRAMPower trace command to be written
    std::string mem_cmd = read ? "RD" : "WR";

    DPRINTF(PIM, "Access to %llX, ready at %lld next burst at %lld"
                " (row_hit = %d).\n",
            pim_pkt->addr, pim_pkt->readyTime, nextBurstAt,row_hit);

    if (op){
        opEnergyAssignment(dataP,rank,pim_pkt->bank,
                            pim_pkt->readyTime,
                            pimCtrl->opAllowedAt,
                            (tCHARGE+tLOGIC));
    } else {
        rank->cmdList.push_back(Command(command, pim_pkt->bank,
                                   cmd_at,
                                   pimCtrl->opAllowedAt));
    }
    DPRINTF(PIMPower, "%llu,%s,%d\n", divCeil(cmd_at, tCK) -
            timeStampOffset, mem_cmd, pim_pkt->bank);

    // Update the minimum timing between the requests, this is a
    // conservative estimate of when we have to schedule the next
    // request to not introduce any unecessary bubbles. In most cases
    // we will wake up sooner than we have to.
    // BenP: I don't think this estimate is correct for the PIM, since the
    // PIM dose not have these timing parameters and the next packet can't
    // be scheduled before the next burst anyway.
    // The +1 for the curTick is to make sure we give the current packet
    // time to get out before we try scheduling the next packet
    nextReqTime = std::max(nextBurstAt - (tRP + tRCD),curTick()+1);
    //DPRINTF(PIM, "nextReqTime %lld\n", nextReqTime);
    // Update the stats and schedule the next request
    if (read) {
        ++readsThisTime;
        bytesReadDRAM += burstSize;
        perBankRdBursts[pim_pkt->bankId]++;

        // Update latency stats
        totMemAccLat += pim_pkt->readyTime - pim_pkt->entryTime;
        masterReadTotalLat[pim_pkt->masterId()] +=
            pim_pkt->readyTime - pim_pkt->entryTime;

        totBusLat += tBURST;
        totQLat += cmd_at - pim_pkt->entryTime;
        masterReadBytes[pim_pkt->masterId()] += pim_pkt->size;
    } else if (op) {
        ++opsThisTime;
        perBankOpBursts[pim_pkt->bankId]++;
        masterOpTotalLat[pim_pkt->masterId()] +=
            pim_pkt->readyTime - pim_pkt->entryTime;
    } else { //write
        ++writesThisTime;
        bytesWritten += burstSize;
        perBankWrBursts[pim_pkt->bankId]++;
        masterWriteBytes[pim_pkt->masterId()] += pim_pkt->size;
        masterWriteTotalLat[pim_pkt->masterId()] +=
            pim_pkt->readyTime - pim_pkt->entryTime;
    }
}

void
PIMMediaCtrl::processNextReqEvent()
{
    // transition is handled by QoS algorithm if enabled
    // BenP: turnPolicy is NULL by default, no DRAM controller
    // available in gem5 uses this.
    //if (turnPolicy) {
        // select bus state - only done if QoS algorithms are in use
        //busStateNext = selectNextBusState();
    //}
    DPRINTF(PIM,"processNextRequestEvent initiated\n");
    // detect bus state change
    bool switched_cmd_type = (busState != busStateNext);
    // record stats
    recordTurnaroundStats();

    DPRINTF(PIM, "QoS Turnarounds selected state %s %s\n",
            (busState==BusState::READ)?"READ":"WRITE",
            switched_cmd_type?"[turnaround triggered]":"");

    if (switched_cmd_type) {
        if (busState == BusState::READ) {
            DPRINTF(PIM,
                    "Switching to writes after %d reads with %d reads "
                    "waiting\n", readsThisTime, totalReadQueueSize);
            rdPerTurnAround.sample(readsThisTime);
            readsThisTime = 0;
        } else {
            DPRINTF(PIM,
                    "Switching to reads after %d writes and %d ops "
                    "with %d writes waiting\n",
                    writesThisTime, opsThisTime, totalWriteQueueSize);
            wrPerTurnAround.sample(writesThisTime+opsThisTime);
            writesThisTime = 0;
            opsThisTime = 0;
        }
    }

    // updates current state
    busState = busStateNext;
    Tick found_pkt_time;

    // when we get here it is either a read or a write
    if (busState == BusState::READ) {

        // track if we should switch or not
        bool switch_to_writes = false;

        if (totalReadQueueSize == 0) {
            // In the case there is no read request to go next,
            // trigger writes if we have passed the low threshold (or
            // if we are draining)
            // BenP: this is peculiar. if totalWriteQueueSize != 0 is true
            // then drainState() == DrainState::Draining is also true
            // , see drainState(). So this "if" statement is true if
            // totalWriteQueueSize != 0.
            if (!(totalWriteQueueSize == 0)){

                // if we just switched from writes, then if we found
                // a packet there schedule it. If there is no packet
                // then reset the variables and return, no packet to
                // schedule
                if (immediateSwitch){
                    assert(tmp_hasReadyPacket);
                    updateNextReqEvent(tmp_nextServTick);
                    busStateNext = BusState::WRITE;
                    tmp_hasReadyPacket = false;
                    immediateSwitch = false;
                } else {
                    DPRINTF(PIM,
                        "Switching to writes due to read queue empty\n");
                    tmp_hasReadyPacket = false;
                    busStateNext = BusState::WRITE;
                    immediateSwitch = true;
                    updateNextReqEvent(curTick());
                }
                return;
            } else {
                // sanity check that we did not came from the writes right now
                assert(!immediateSwitch);
                // nothing to do, not even any point in scheduling an
                // event for the next request
                return;
            }
        } else {

            bool read_found = false;
            PIMPacketQueue::iterator to_read;
            // numPriorities() is the number of QoS levels
            uint8_t prio = numPriorities();

            for (auto queue = readQueue.rbegin();
                 queue != readQueue.rend(); ++queue) {

                prio--;

                DPRINTF(QOS,
                        "DRAM controller checking READ queue"\
                        "[%d] priority [%d elements]\n",
                        prio, queue->size());

                // Figure out which read request goes next
                // If we are changing command type, incorporate the minimum
                // bus turnaround delay (0 in PIM, since we have separate IOs)
                std::tie(to_read,found_pkt_time) = chooseNext((*queue), 0);
                if (to_read != queue->end()) {
                    // candidate read found
                    read_found = true;
                    break;
                }
            }

            // if no read to an available rank is found then return
            // at this point. There could be writes, so go check them
            if (!read_found) {

                DPRINTF(PIM, "No Reads Found - exiting\n");
                // if this is after a switch then take what we have found
                // in the write
                if (immediateSwitch) {
                    // if we came from writes,
                    assert(tmp_hasReadyPacket);
                    updateNextReqEvent(tmp_nextServTick);
                    busStateNext = BusState::WRITE;
                    immediateSwitch = false;
                } else {
                    // if this is not after a switch, then check the writes.
                    // They should not be empty, since if they were the read
                    //  queue will have at least one packet ready
                    // (we checked it is not empty)
                    assert(totalWriteQueueSize);
                    updateNextReqEvent(curTick());
                    busStateNext = BusState::WRITE;
                    immediateSwitch = true;
                    tmp_hasReadyPacket = false;
                }
                // either way exit, since we don't have a read packet to do.
                return;
            }
            // check if the packet is too far in the future.
            // If so then check the writes if we haven't just came from there,
            // if we came from checking the writes then schedule the next event
            // whichever is the closest one
            if ((found_pkt_time > curTick()) &&
            (found_pkt_time - curTick() > thresholdScheduleCheckTime)) {
                DPRINTF(PIM, "Packet it too long into the future\n");
                if (immediateSwitch) {
                    // if the write queue has a ready earlier
                    // packet then use that
                    if (tmp_hasReadyPacket &&
                            (found_pkt_time > tmp_nextServTick)){
                        DPRINTF(PIM, "Switching to previously found "
                                        "write packet\n");
                        updateNextReqEvent(tmp_nextServTick);
                        busStateNext = BusState::WRITE;
                    } else {
                        DPRINTF(PIM, "No previously found write packet, just"
                                    " re-check in predetermined time\n");
                        updateNextReqEvent(curTick()+
                                            thresholdScheduleCheckTime);
                        busStateNext = BusState::READ;
                    }
                    tmp_hasReadyPacket = false;
                    immediateSwitch = false;
                } else {
                    if (totalWriteQueueSize) {
                        DPRINTF(PIM, "There are packets in the write queue,"
                                    "check them out\n");
                        // go check the write queue if it is not empty
                        updateNextReqEvent(curTick());
                        busStateNext = BusState::WRITE;
                        tmp_nextServTick = found_pkt_time;
                        tmp_hasReadyPacket = true;
                        immediateSwitch = true;
                    } else {
                        DPRINTF(PIM, "No packet in the write queue, just"
                                    " re-check in predetermined time\n");
                        updateNextReqEvent(curTick()+
                                            thresholdScheduleCheckTime);
                        busStateNext = BusState::READ;
                        tmp_hasReadyPacket = false;
                        immediateSwitch = false;
                    }
                }
                return;
            }

            auto pim_pkt = *to_read;

            doDRAMAccess(pim_pkt);

            // sanity check
            assert(pim_pkt->size <= burstSize);
            assert(pim_pkt->readyTime >= curTick());

            // log the response
            logResponse(MemCtrl::READ, (*to_read)->masterId(),
                        pim_pkt->qosValue(), pim_pkt->getAddr(), 1,
                        pim_pkt->readyTime - pim_pkt->entryTime);


            // Insert into response queue. It will be sent back to the
            // requester at its readyTime
            if (respQueue.empty()) {
                assert(!respondEvent.scheduled());
                schedule(respondEvent, pim_pkt->readyTime);
                respQueue.push_back(pim_pkt);
                DPRINTF(PIM,"insert packet to respQueue, addr 0x%X,"
                            " ready time %ld\n",
                            pim_pkt->addr,pim_pkt->readyTime);
            // if the queue is not empty then find the location to insert the
            // new response
            } else {
                // there is a packet in the queue, so it needs to be scheduled
                assert(respondEvent.scheduled());
                for (auto it = respQueue.rbegin();it != respQueue.rend();it++){
                    if ((*it)->readyTime <= pim_pkt->readyTime){
                        respQueue.insert(it.base(),pim_pkt);
                        DPRINTF(PIM,"insert packet to respQueue,"
                                " addr 0x%X, ready time %ld\n"
                                "\tafter packet to addr 0x%d at"
                                " ready time %ld\n",
                            pim_pkt->addr,pim_pkt->readyTime,
                            (*it)->addr, (*it)->readyTime);

                        break;
                    // we need to check explicitly if we are at the front
                    // of the queue since the for-loop does not cover this
                    // automatically.
                    } else if (it == (respQueue.rend()-1)) {
                        respQueue.push_front(pim_pkt);
                        DPRINTF(PIM,"insert packet at front of respQueue,"
                            " addr 0x%X, ready time %ld\n"
                            "\tafter packet to addr 0x%d at ready time %ld\n",
                            pim_pkt->addr,pim_pkt->readyTime,
                            (*it)->addr, (*it)->readyTime);
                        reschedule(respondEvent, pim_pkt->readyTime);
                        break;
                    }
                }
            }

            // we have so many writes that we have to transition
            if (totalWriteQueueSize > writeHighThreshold) {
                switch_to_writes = true;
            }

            // erase the packet from the multisets for the searches
            Addr addr = pim_pkt->addr;
            auto pageRef = is2MPageInReadQueue.find(addr & pim::PAGE_MASK);
            auto addrRef = isInReadQueue.find(burstAlign(addr));
            assert(pageRef != is2MPageInReadQueue.end());
            assert(addrRef != isInReadQueue.end());
            is2MPageInReadQueue.erase(pageRef);
            isInReadQueue.erase(addrRef);
            // mark all packets depended on this packet as not depended
            for (auto& p : pim_pkt->dependeds){
                p->dependedOn = false;
            }
            // remove the request from the queue -
            // the iterator is no longer valid.
            readQueue[pim_pkt->qosValue()].erase(to_read);
        }

        // switching to writes, because the writes hit the hight threshold
        if (switch_to_writes) {
            // transition to writing
            busStateNext = BusState::WRITE;
        }
    } else {
        // note that if we are here then the write queue is
        // not empty
        assert(totalWriteQueueSize);

        bool write_found = false;
        PIMPacketQueue::iterator to_write;
        uint8_t prio = numPriorities();

        for (auto queue = writeQueue.rbegin();
             queue != writeQueue.rend(); ++queue) {

            prio--;

            DPRINTF(QOS,
                    "DRAM controller checking WRITE queue"\
                        "[%d] priority [%d elements]\n",
                    prio, queue->size());

            // If we are changing command type, incorporate the minimum
            // bus turnaround delay (0 in PIM, since we have separate IOs)
            std::tie(to_write,found_pkt_time) = chooseNext((*queue), 0);
            if (to_write != queue->end()) {
                write_found = true;
                break;
            }
        }

        // if there are no writes to a rank that is available to service
        // requests (i.e. rank is in refresh idle state) are found then
        // return. There could be reads to the available ranks. However, to
        // avoid adding more complexity to the code, return at this point and
        // wait for a refresh event to kick things into action again.
        // find a read! do not wait to a refresh, there isn't one here.
        if (!write_found) {
            DPRINTF(PIM, "No Writes Found - exiting\n");

            // if we came from the read queue then take what we found there.
            // it should be impossible to have a non-empty write queue
            // with no ready packets AND that the read queue does not
            // have ready packets for any reason
            if (immediateSwitch) {
                assert(tmp_hasReadyPacket);
                updateNextReqEvent(tmp_nextServTick);
                busStateNext = BusState::READ;
                immediateSwitch = false;
            } else {
                // if we did not found a write then it is because all of the
                // writes are depended on the reads, the the reads must not
                // be empty
                assert(totalReadQueueSize);
                updateNextReqEvent(curTick());
                busStateNext = BusState::READ;
                immediateSwitch = true;
            }
            tmp_hasReadyPacket = false;
            return;
        }

        // check if the packet is too far in the future.
        // If so then check the reads if we haven't just came from there,
        // if we came from checking the reads then schedule the next event
        // whichever is the closest one
        if ((found_pkt_time > curTick()) &&
        (found_pkt_time - curTick() > thresholdScheduleCheckTime)) {
            if (immediateSwitch) {
                // if the read queue has a ready earlier packet then use that
                if (tmp_hasReadyPacket && (found_pkt_time > tmp_nextServTick)){
                    updateNextReqEvent(tmp_nextServTick);
                    busStateNext = BusState::READ;
                } else {
                    updateNextReqEvent(found_pkt_time);
                    busStateNext = BusState::WRITE;
                }
                tmp_hasReadyPacket = false;
                immediateSwitch = false;
            } else {
                // go check the read queue
                updateNextReqEvent(curTick());
                DPRINTF(PIM,"immediate to READs due"
                        " to long write ready time %d\n",
                        found_pkt_time - curTick());
                busStateNext = BusState::READ;
                tmp_nextServTick = found_pkt_time;
                tmp_hasReadyPacket = true;
                immediateSwitch = true;
            }
            return;
        }

        auto pim_pkt = *to_write;

        // sanity check
        assert(pim_pkt->size <= burstSize);

        doDRAMAccess(pim_pkt);

        // erase the packet from the multisets for the searches
        Addr addr = pim_pkt->addr;
        if (pim_pkt->isOp()){
            auto pageRef = isPIMOpInWriteQueue.find(addr & pim::PAGE_MASK);
            assert(pageRef != isPIMOpInWriteQueue.end());
            isPIMOpInWriteQueue.erase(pageRef);
        } else {
            auto addrRef = isInWriteQueue.find(burstAlign(addr));
            assert(addrRef != isInWriteQueue.end());
            isInWriteQueue.erase(addrRef);
        }
        // mark all packets depended on this packet as not depended
        for (auto& p : pim_pkt->dependeds){
            p->dependedOn = false;
        }

        // log the response
        logResponse(MemCtrl::WRITE, pim_pkt->masterId(),
                    pim_pkt->qosValue(), pim_pkt->getAddr(), 1,
                    pim_pkt->readyTime - pim_pkt->entryTime);


        // remove the request from the queue - the iterator is no longer valid
        writeQueue[pim_pkt->qosValue()].erase(to_write);

        //delete pim_pkt;
        // instead of deleting the packet we return in to the
        // memory controller at the CPU die. This is because we
        // are using a OpenCAPI style connection which require a
        // replay
        // Insert into response queue. It will be sent back to the
        // requester at its readyTime
        if (respQueue.empty()) {
            assert(!respondEvent.scheduled());
            schedule(respondEvent, pim_pkt->readyTime);
            respQueue.push_back(pim_pkt);
            DPRINTF(PIM,"insert packet to respQueue, addr 0x%X,"
                            " ready time %ld\n",
                        pim_pkt->addr,pim_pkt->readyTime);
        // if the queue is not empty then find the location to insert the
        // new response
        } else {
            // there is a packet in the queue, so it needs to be scheduled
            assert(respondEvent.scheduled());
            for ( auto it = respQueue.rbegin();it != respQueue.rend();it++){
                if ((*it)->readyTime <= pim_pkt->readyTime){
                    respQueue.insert(it.base(),pim_pkt);
                    DPRINTF(PIM,"insert packet to respQueue, addr 0x%X,"
                            " ready time %ld\n"
                            "\tafter packet to addr 0x%d at ready time %ld\n",
                        pim_pkt->addr,pim_pkt->readyTime,
                        (*it)->addr, (*it)->readyTime);

                    break;
                // we need to check explicitly if we are at the front
                // of the queue since the for-loop does not cover this
                // automatically.
                } else if (it == (respQueue.rend()-1)) {
                    respQueue.push_front(pim_pkt);
                    DPRINTF(PIM,"insert packet at front of respQueue,"
                            " addr 0x%X, ready time %ld\n"
                            "\tafter packet to addr 0x%d at ready time %ld\n",
                        pim_pkt->addr,pim_pkt->readyTime,
                        (*it)->addr, (*it)->readyTime);
                    reschedule(respondEvent, pim_pkt->readyTime);
                    break;
                }
            }
        }
        // If we emptied the write queue, or got sufficiently below the
        // threshold (using the minWritesPerSwitch as the hysteresis) and
        // are not draining, or we have reads waiting and have done enough
        // writes, then switch to reads.
        bool below_threshold =
            (totalWriteQueueSize + minWritesPerSwitch) < writeLowThreshold;

        if (totalWriteQueueSize == 0 ||
            (below_threshold && (drainState() != DrainState::Draining)) ||
            (totalReadQueueSize && (writesThisTime >= minWritesPerSwitch))) {

            // turn the bus back around for reads again
            busStateNext = BusState::READ;

            // note that the we switch back to reads also in the idle
            // case, which eventually will check for any draining and
            // also pause any further scheduling if there is really
            // nothing to do
        }
    }

    //schedule the next pkt
    immediateSwitch = false;
    updateNextReqEvent(nextReqTime);

    // If there is space available and we have writes waiting then let
    // them retry. This is done here to ensure that the retry does not
    // cause a nextReqEvent to be scheduled before we do so as part of
    // the next request processing
    if (retryWrReq && (totalWriteQueueSize < writeBufferSize)) {
        retryWrReq = false;
        port.sendRetryReq();
    }
}

PIMMediaCtrl::Rank::Rank(PIMMediaCtrl& _memory,
                        const PIMMediaCtrlParams* _p)
    : EventManager(&_memory), memory(_memory),
      powerBinCount(_p->power_Bin_Count),
      isPowerEventStartedUp(false),
      readEntries(0), writeEntries(0),
      power(_p, true), banks(pim::BANKS_PER_RANK),
      periodPowerStatsEvent([this]{ processPeriodPowerStatsEvent(); }, name()),
      lastWindowEnd(0),windowDuration(_p->power_window_duration)
{
    for (int b = 0; b < pim::BANKS_PER_RANK; b++) {
        banks[b].bank = b;
        char buffer[20];
        for (int i = 0; i < pim::PIMCTRL_PER_BANK ; i++){
            banks[b].pimCtrls[i]->opCycleTime = _p->tCHARGE + _p->tLOGIC;
            sprintf(buffer,"bank%d_ctrl%d",b,i);
            banks[b].pimCtrls[i]->name = name()+buffer;
        }
        // GDDR addressing of banks to BG is linear.
        // Here we assume that all DRAM generations address bank groups as
        // follows:
        if (_p->bank_groups_per_rank > 0) {
            // Simply assign lower bits to bank group in order to
            // rotate across bank groups as banks are incremented
            // e.g. with 4 banks per bank group and 16 banks total:
            //    banks 0,4,8,12  are in bank group 0
            //    banks 1,5,9,13  are in bank group 1
            //    banks 2,6,10,14 are in bank group 2
            //    banks 3,7,11,15 are in bank group 3
            banks[b].bankgr = b % _p->bank_groups_per_rank;
        } else {
            // No bank groups; simply assign to bank number
            banks[b].bankgr = b;
        }
    }
}

void
PIMMediaCtrl::Rank::startUpPowerStatsEvent()
{
    // starting the power scheduling loop
    if (!isPowerEventStartedUp){
        schedule(periodPowerStatsEvent, curTick());
        isPowerEventStartedUp = true;
    }
}

void
PIMMediaCtrl::Rank::updatePowerStats()
{
    // The DRAMPower library works with pJ and mW
    //(see LibDRAMPower.h, line 87), so that is what we
    // work with as well

    double windowReadEnergy = 0;
    double windowWriteEnergy = 0;
    double windowOpEnergy = 0;
    double windowPIMCtrlsSwitchEnergy = 0;
    double windowPIMCtrlsLeakageEnergy = 0;
    double windowTotalEnergy;

    std::vector<Command> cmdList_leftover;

    // flush cmdList to DRAMPower
    // at the moment sort the list of commands and update the counters
    // for DRAMPower libray when doing a refresh
    sort(cmdList.begin(), cmdList.end(), PIMMediaCtrl::sortTime);

    auto next_iter = cmdList.begin();
    // push to commands to DRAMPower
    for ( ; next_iter != cmdList.end() ; ++next_iter) {
        Command cmd = *next_iter;
        if (cmd.timeStamp <= curTick()) {
            // Move all commands at or before curTick to DRAMPower
            // if the command is issued for the first time here.
            // we use the DRAMPower only for the data movement energy
            // on the RDDR bus.
            if (cmd.type != PIMMediaCtrl::commandType::OP_FULL_C &&
               cmd.type != PIMMediaCtrl::commandType::OP_SNGL_C){
               Data::MemCommand::cmds type =
                 cmd.type == PIMMediaCtrl::commandType::READ_H ||
                 cmd.type == PIMMediaCtrl::commandType::READ_M ?
                 Data::MemCommand::cmds::RD : Data::MemCommand::cmds::WR;
               power.powerlib.doCommand(type, cmd.bank,
                                         divCeil(cmd.timeStamp, memory.tCK) -
                                         memory.timeStampOffset);
            }
            // add the PIM energy if needed.
            // for read hit in the buffer no need to add anything
            switch(cmd.type){
                case PIMMediaCtrl::commandType::READ_H :
                    break;
                case PIMMediaCtrl::commandType::READ_M :
                    windowReadEnergy +=
                        pim::SA_READ_ENERGY*pim::NUM_OF_SA_PER_MAT*
                        pim::MATS_IN_READ*pim::CHIPS_PER_RANK;
                    break;
                case PIMMediaCtrl::commandType::WRITE :
                // for simplicity assume half the bits are high and half low
                    windowWriteEnergy +=
                        (pim::SET_BIT_ENERGY + pim::RESET_BIT_ENERGY)*
                        (pim::CACHEBLOCK_SIZE/2)*pim::BYTE_SIZE;
                    break;
                case PIMMediaCtrl::commandType::OP_FULL_N :
                case PIMMediaCtrl::commandType::OP_FULL_C :
                case PIMMediaCtrl::commandType::OP_SNGL_N :
                case PIMMediaCtrl::commandType::OP_SNGL_C :
                    // find the end time
                    Tick endOp = std::min(cmd.endTime,curTick());

                    unsigned magicInst = (
                                    (cmd.type ==
                                        PIMMediaCtrl::commandType::OP_FULL_N)
                                    ||
                                    (cmd.type ==
                                        PIMMediaCtrl::commandType::OP_FULL_C))
                                        ?
                                        pim::MAT_ROWS : 1;

                    // find the energy due to switching of the PIM controller
                    // the last factor is because the energy is given in pJ and
                    // the power in W
                    windowPIMCtrlsSwitchEnergy +=
                        pim::PIMCNTRL_SWITCH_POWER*
                        (endOp-cmd.timeStamp)/SimClock::Frequency*
                        pim::CHIPS_PER_RANK*
                        pim::PIMCTRL_PER_1GBPAGE_PER_CHIP*1e12;
                    windowOpEnergy += pim::MAGIC_BIT_ENERGY*magicInst*
                        pim::MATS_PER_PIMCTRL_1GB*pim::PIMCTRL_PER_1GBPAGE_PER_CHIP*
                        pim::CHIPS_PER_RANK*
                        (endOp-cmd.timeStamp)/memory.tLOGIC;
                        //(SimClock::Frequency / 1000000000.0);
                    // Compute the leakage for the PIM controllers.
                    // We operate only when there is an PIM operation,
                    // otherwise we do power gating
                    // the last factor is because the energy is given in pJ and
                    // and the power in W
                    windowPIMCtrlsLeakageEnergy +=
                            pim::PIMCNTRL_LEAKAGE_POWER*
                            windowDuration/SimClock::Frequency*
                            pim::PIMCTRL_PER_1GBPAGE_PER_CHIP*
                            pim::CHIPS_PER_RANK*1e12;
                    // transfer the remaining operation to the next window
                    if (endOp < cmd.endTime){
                        commandType type =
                            ((cmd.type ==
                                PIMMediaCtrl::commandType::OP_FULL_N)
                            ||
                            (cmd.type ==
                                PIMMediaCtrl::commandType::OP_FULL_C)) ?
                                PIMMediaCtrl::commandType::OP_FULL_C :
                                PIMMediaCtrl::commandType::OP_SNGL_C ;
                        cmdList_leftover.push_back(Command(type,cmd.bank,
                            curTick()+1,cmd.endTime));
                    }
                    break;
            }
        } else {
            // done - found all commands at or before curTick()
            // next_iter references the 1st command after curTick
            break;
        }
    }
    // reset cmdList to only contain commands after curTick
    // if there are no commands after curTick, updated cmdList will be empty
    // in this case, next_iter is cmdList.end()
    cmdList.assign(next_iter, cmdList.end());
    cmdList.insert(cmdList.end(),
        cmdList_leftover.begin(),cmdList_leftover.end());


    // Call the function that calculates window energy at intermediate update
    // events like at refresh, stats dump as well as at simulation exit.
    // Window starts at the last time the calcWindowEnergy function was called
    // and is upto current time.
    power.powerlib.calcWindowEnergy(divCeil(curTick(), memory.tCK) -
                                    memory.timeStampOffset);

    // Get the energy from DRAMPower
    Data::MemoryPowerModel::Energy energy = power.powerlib.getEnergy();

    // The energy components inside the power lib are calculated over
    // the window so accumulate into the corresponding gem5 stat
    periphReadEnergy += energy.read_energy * memory.devicesPerRank;
    arrayReadEnergy += windowReadEnergy;
    windowReadEnergy += energy.read_energy * memory.devicesPerRank;
    readEnergy += windowReadEnergy;
    periphWriteEnergy += energy.write_energy * memory.devicesPerRank;
    arrayWriteEnergy += windowWriteEnergy;
    windowWriteEnergy += energy.write_energy * memory.devicesPerRank;
    writeEnergy += windowWriteEnergy;

    arrayOpEnergy += windowOpEnergy;

    pimCtrlsSwitchEnergy += windowPIMCtrlsSwitchEnergy;
    pimCtrlsLeakageEnergy += windowPIMCtrlsLeakageEnergy;

    ioTermEnergy += energy.io_term_energy;

    windowTotalEnergy = windowReadEnergy + windowWriteEnergy + windowOpEnergy +
            energy.io_term_energy +
            windowPIMCtrlsSwitchEnergy + windowPIMCtrlsLeakageEnergy;

    // Accumulate window energy into the total energy.
    totalEnergy += windowTotalEnergy;
    // Average power must not be accumulated but calculated over the time
    // since last stats reset. SimClock::Frequency is tick period not tick
    // frequency.
    //              energy (pJ)     1e-9
    // power (mW) = ----------- * ----------
    //              time (tick)   tick_frequency
    averagePower = (totalEnergy.value() /
                    (curTick() - memory.lastStatsResetTick)) *
                    (SimClock::Frequency / 1000000000.0);
    windowAveragePower.sample( (windowTotalEnergy /
                    (curTick() - lastWindowEnd)) *
                    (SimClock::Frequency / 1000000000.0) );
}

void
PIMMediaCtrl::Rank::processPeriodPowerStatsEvent()
{
    DPRINTF(PIM,"processPeriodPowerStatsEvent initiated\n");
    updatePowerStats();
    lastWindowEnd = curTick() + 1;
    schedule(periodPowerStatsEvent, curTick()+windowDuration);
}


void
PIMMediaCtrl::Rank::computeStats()
{
    DPRINTF(PIM,"Computing stats due to a dump callback\n");

    // Update the stats
    updatePowerStats();
    lastWindowEnd = curTick() + 1;

}

void
PIMMediaCtrl::Rank::resetStats() {
    // The only way to clear the counters in DRAMPower is to call
    // calcWindowEnergy function as that then calls clearCounters. The
    // clearCounters method itself is private.
    power.powerlib.calcWindowEnergy(divCeil(curTick(), memory.tCK) -
                                    memory.timeStampOffset);

}
void
PIMMediaCtrl::PIMCtrl::regStats()
{
    totalOpCycles
        .name(name + ".totalOpCycles")
        .desc("number of Op cycles performed in this PIMCtrl");
    totalOpTime
        .name(name + ".totalOpTime")
        .desc("Time of Op cycles performed in this PIMCtrl [ps]");
    totalOpTime = totalOpCycles*opCycleTime;
}
void
PIMMediaCtrl::Rank::regStats()
{
    for (auto bank_it = banks.begin() ; bank_it != banks.end() ; bank_it++){
        for (int i = 0 ; i < pim::PIMCTRL_PER_BANK ; i++){
            bank_it->pimCtrls[i]->regStats();
        }
    }
    readEnergy
        .name(name() + ".readEnergy")
        .desc("Energy for read commands(pJ)");

    periphReadEnergy
        .name(name() + ".periphReadEnergy")
        .desc("Energy for peripherals on read commands(pJ)");

    arrayReadEnergy
        .name(name() + ".arrayReadEnergy")
        .desc("Energy for array on read commands(pJ)");

    writeEnergy
        .name(name() + ".writeEnergy")
        .desc("Energy for write commands(pJ)");

    periphWriteEnergy
        .name(name() + ".periphWriteEnergy")
        .desc("Energy for peripherals on write commands(pJ)");

    arrayWriteEnergy
        .name(name() + ".arrayWriteEnergy")
        .desc("Energy for array on write commands(pJ)");

    ioTermEnergy
        .name(name() + ".ioTermEnergy")
        .desc("Energy for IO and termination for all operations(pJ)");

    arrayOpEnergy
        .name(name() + ".arrayOpEnergy")
        .desc("Energy for array on PIM operation commands(pJ)");

    pimCtrlsSwitchEnergy
        .name(name() + ".pimCtrlsSwitchEnergy")
        .desc("Total Switching Energy for PIM controllers(pJ)");

    pimCtrlsLeakageEnergy
        .name(name() + ".pimCtrlsLeakageEnergy")
        .desc("Total Leakage Energy for PIM controllers(pJ)");

    totalEnergy
        .name(name() + ".totalEnergy")
        .desc("Total energy(pJ)");

    averagePower
        .name(name() + ".averagePower")
        .desc("Core power(mW)");

    windowAveragePower
        .init(powerBinCount)
        .name(name() + ".windowAveragePower")
        .desc("power of a single measured window")
        .flags(Stats::nozero);

    Stats::registerDumpCallback(new RankDumpCallback(this));
    Stats::registerResetCallback(new RankResetCallback(this));
}
void
PIMMediaCtrl::regStats()
{
    using namespace Stats;

    MemCtrl::regStats();

    rank->regStats();

    registerResetCallback(new MemResetCallback(this));

    readReqs
        .name(name() + ".readReqs")
        .desc("Number of read requests accepted");

    writeReqs
        .name(name() + ".writeReqs")
        .desc("Number of write requests accepted");

    numOpReqs
        .name(name() + ".numOpReqs")
        .desc("Number of pim op requests accepted");

    numTypeOpReqs
        .init(1U << pim::OPCODE_LEN)
        .name(name() + ".numTypeOpReqs")
        .desc("number of operations from each type at"
              " the PIM controller")
        .flags(nozero | nonan);

    readBursts
        .name(name() + ".readBursts")
        .desc("Number of DRAM read bursts, "
              "including those serviced by the write queue");

    writeBursts
        .name(name() + ".writeBursts")
        .desc("Number of DRAM write bursts, "
              "including those merged in the write queue");

    opBursts
        .name(name() + ".opBursts")
        .desc("Number of PIM op bursts");

    servicedByWrQ
        .name(name() + ".servicedByWrQ")
        .desc("Number of DRAM read bursts serviced by the write queue");

    readDependOnPIM
        .name(name() + ".readDependOnPIM")
        .desc("Number of reads that are depended on a write or op.");

    mergedWrBursts
        .name(name() + ".mergedWrBursts")
        .desc("Number of DRAM write bursts merged with an existing one");

    neitherReadNorWrite
        .name(name() + ".neitherReadNorWriteReqs")
        .desc("Number of requests that are neither read nor write");

    perBankRdBursts
        .init(banksPerRank)
        .name(name() + ".perBankRdBursts")
        .desc("Per bank write bursts");

    perBankWrBursts
        .init(banksPerRank)
        .name(name() + ".perBankWrBursts")
        .desc("Per bank write bursts");

    perBankOpBursts
        .init(banksPerRank)
        .name(name() + ".perBankOpBursts")
        .desc("Per bank PIM op bursts");

    avgRdQLen
        .name(name() + ".avgRdQLen")
        .desc("Average read queue length when enqueuing")
        .precision(2);

    avgWrQLen
        .name(name() + ".avgWrQLen")
        .desc("Average write queue length when enqueuing")
        .precision(2);

    totQLat
        .name(name() + ".totQLat")
        .desc("Total ticks spent queuing");

    totBusLat
        .name(name() + ".totBusLat")
        .desc("Total ticks spent in databus transfers");

    totMemAccLat
        .name(name() + ".totMemAccLat")
        .desc("Total ticks spent from burst creation until serviced "
              "by the DRAM");

    avgQLat
        .name(name() + ".avgQLat")
        .desc("Average queueing delay per DRAM burst")
        .precision(2);

    avgQLat = totQLat / (readBursts - servicedByWrQ);

    avgBusLat
        .name(name() + ".avgBusLat")
        .desc("Average bus latency per DRAM burst")
        .precision(2);

    avgBusLat = totBusLat / (readBursts - servicedByWrQ);

    avgMemAccLat
        .name(name() + ".avgMemAccLat")
        .desc("Average memory access latency per DRAM burst")
        .precision(2);

    avgMemAccLat = totMemAccLat / (readBursts - servicedByWrQ);

    numRdRetry
        .name(name() + ".numRdRetry")
        .desc("Number of times read queue was full causing retry");

    numWrRetry
        .name(name() + ".numWrRetry")
        .desc("Number of times write queue was full causing retry");

    readRowHits
        .name(name() + ".readRowHits")
        .desc("Number of row buffer hits during reads");

    writeRowHits
        .name(name() + ".writeRowHits")
        .desc("Number of row buffer hits during writes");

    opRowHits
        .name(name() + ".opRowHits")
        .desc("Number of row buffer hits during PIM operations");

    readRowHitRate
        .name(name() + ".readRowHitRate")
        .desc("Row buffer hit rate for reads")
        .precision(2);

    readRowHitRate = (readRowHits / (readBursts - servicedByWrQ)) * 100;

    writeRowHitRate
        .name(name() + ".writeRowHitRate")
        .desc("Row buffer hit rate for writes")
        .precision(2);

    writeRowHitRate = (writeRowHits / (writeBursts - mergedWrBursts)) * 100;

    opRowHitRate
        .name(name() + ".opRowHitRate")
        .desc("Row buffer hit rate for PIM ops")
        .precision(2);

    opRowHitRate = (opRowHits / opBursts) * 100;

    readPktSize
        .init(ceilLog2(burstSize) + 1)
        .name(name() + ".readPktSize")
        .desc("Read request sizes (log2)");

     writePktSize
        .init(ceilLog2(burstSize) + 1)
        .name(name() + ".writePktSize")
        .desc("Write request sizes (log2)");

     rdQLenPdf
        .init(readBufferSize)
        .name(name() + ".rdQLenPdf")
        .desc("What read queue length does an incoming req see");

     wrQLenPdf
        .init(writeBufferSize)
        .name(name() + ".wrQLenPdf")
        .desc("What write queue length does an incoming req see");

     rdPerTurnAround
         .init(readBufferSize)
         .name(name() + ".rdPerTurnAround")
         .desc("Reads before turning the bus around for writes")
         .flags(nozero);

     wrPerTurnAround
         .init(writeBufferSize)
         .name(name() + ".wrPerTurnAround")
         .desc("Writes before turning the bus around for reads")
         .flags(nozero);

    bytesReadDRAM
        .name(name() + ".bytesReadDRAM")
        .desc("Total number of bytes read from DRAM");

    bytesReadWrQ
        .name(name() + ".bytesReadWrQ")
        .desc("Total number of bytes read from write queue");

    bytesWritten
        .name(name() + ".bytesWritten")
        .desc("Total number of bytes written to DRAM");

    bytesReadSys
        .name(name() + ".bytesReadSys")
        .desc("Total read bytes from the system interface side");

    bytesWrittenSys
        .name(name() + ".bytesWrittenSys")
        .desc("Total written bytes from the system interface side");

    avgRdBW
        .name(name() + ".avgRdBW")
        .desc("Average DRAM read bandwidth in MiByte/s")
        .precision(2);

    avgRdBW = (bytesReadDRAM / 1000000) / simSeconds;

    avgWrBW
        .name(name() + ".avgWrBW")
        .desc("Average achieved write bandwidth in MiByte/s")
        .precision(2);

    avgWrBW = (bytesWritten / 1000000) / simSeconds;

    avgRdBWSys
        .name(name() + ".avgRdBWSys")
        .desc("Average system read bandwidth in MiByte/s")
        .precision(2);

    avgRdBWSys = (bytesReadSys / 1000000) / simSeconds;

    avgWrBWSys
        .name(name() + ".avgWrBWSys")
        .desc("Average system write bandwidth in MiByte/s")
        .precision(2);

    avgWrBWSys = (bytesWrittenSys / 1000000) / simSeconds;

    peakBW
        .name(name() + ".peakBW")
        .desc("Theoretical peak bandwidth in MiByte/s")
        .precision(2);

    peakBW = (SimClock::Frequency / tBURST) * burstSize / 1000000;

    busUtil
        .name(name() + ".busUtil")
        .desc("Data bus utilization in percentage")
        .precision(2);
    busUtil = (avgRdBW + avgWrBW) / peakBW * 100;

    totGap
        .name(name() + ".totGap")
        .desc("Total gap between requests");

    avgGap
        .name(name() + ".avgGap")
        .desc("Average gap between requests")
        .precision(2);

    avgGap = totGap / (readReqs + writeReqs);

    // Stats for DRAM Power calculation based on Micron datasheet
    busUtilRead
        .name(name() + ".busUtilRead")
        .desc("Data bus utilization in percentage for reads")
        .precision(2);

    busUtilRead = avgRdBW / peakBW * 100;

    busUtilWrite
        .name(name() + ".busUtilWrite")
        .desc("Data bus utilization in percentage for writes")
        .precision(2);

    busUtilWrite = avgWrBW / peakBW * 100;

    pageHitRate
        .name(name() + ".pageHitRate")
        .desc("Row buffer hit rate, read and write combined")
        .precision(2);

    pageHitRate = (writeRowHits + readRowHits) /
        (writeBursts - mergedWrBursts + readBursts
        - servicedByWrQ) * 100;

    // per-master bytes read and written to memory
    masterReadBytes
        .init(_system->maxMasters())
        .name(name() + ".masterReadBytes")
        .desc("Per-master bytes read from memory")
        .flags(nozero | nonan);

    masterWriteBytes
        .init(_system->maxMasters())
        .name(name() + ".masterWriteBytes")
        .desc("Per-master bytes write to memory")
        .flags(nozero | nonan);

    // per-master bytes read and written to memory rate
    masterReadRate.name(name() + ".masterReadRate")
        .desc("Per-master bytes read from memory rate (Bytes/sec)")
        .flags(nozero | nonan)
        .precision(12);

    masterReadRate = masterReadBytes/simSeconds;

    masterWriteRate
        .name(name() + ".masterWriteRate")
        .desc("Per-master bytes write to memory rate (Bytes/sec)")
        .flags(nozero | nonan)
        .precision(12);

    masterWriteRate = masterWriteBytes/simSeconds;

    masterReadAccesses
        .init(_system->maxMasters())
        .name(name() + ".masterReadAccesses")
        .desc("Per-master read serviced memory accesses")
        .flags(nozero);

    masterWriteAccesses
        .init(_system->maxMasters())
        .name(name() + ".masterWriteAccesses")
        .desc("Per-master write serviced memory accesses")
        .flags(nozero);

    masterOpAccesses
        .init(_system->maxMasters())
        .name(name() + ".masterOpAccesses")
        .desc("Per-master PIM op serviced memory accesses")
        .flags(nozero);

    masterReadTotalLat
        .init(_system->maxMasters())
        .name(name() + ".masterReadTotalLat")
        .desc("Per-master read total memory access latency")
        .flags(nozero | nonan);

    masterReadAvgLat.name(name() + ".masterReadAvgLat")
        .desc("Per-master read average memory access latency")
        .flags(nonan)
        .precision(2);

    masterReadAvgLat = masterReadTotalLat/masterReadAccesses;

    masterWriteTotalLat
        .init(_system->maxMasters())
        .name(name() + ".masterWriteTotalLat")
        .desc("Per-master write total memory access latency")
        .flags(nozero | nonan);

    masterOpTotalLat
        .init(_system->maxMasters())
        .name(name() + ".masterOpTotalLat")
        .desc("Per-master PIM op total memory access latency")
        .flags(nozero | nonan);

    masterWriteAvgLat.name(name() + ".masterWriteAvgLat")
        .desc("Per-master write average memory access latency")
        .flags(nonan)
        .precision(2);

    masterWriteAvgLat = masterWriteTotalLat/masterWriteAccesses;

    masterOpAvgLat.name(name() + ".masterOpAvgLat")
        .desc("Per-master PIM op average memory access latency")
        .flags(nonan)
        .precision(2);

    masterOpAvgLat = masterOpTotalLat/masterOpAccesses;

    for (int i = 0; i < _system->maxMasters(); i++) {
        const std::string master = _system->getMasterName(i);
        masterReadBytes.subname(i, master);
        masterReadRate.subname(i, master);
        masterWriteBytes.subname(i, master);
        masterWriteRate.subname(i, master);
        masterReadAccesses.subname(i, master);
        masterWriteAccesses.subname(i, master);
        masterOpAccesses.subname(i, master);
        masterReadTotalLat.subname(i, master);
        masterReadAvgLat.subname(i, master);
        masterWriteTotalLat.subname(i, master);
        masterOpTotalLat.subname(i, master);
        masterWriteAvgLat.subname(i, master);
        masterOpAvgLat.subname(i, master);
    }
}

void
PIMMediaCtrl::recvFunctional(PacketPtr pkt)
{
    // rely on the abstract memory
    functionalAccess(pkt);
}

Port &
PIMMediaCtrl::getPort(const string &if_name, PortID idx)
{
    if (if_name != "port") {
        return QoS::MemCtrl::getPort(if_name, idx);
    } else {
        return port;
    }
}

PIMMediaCtrl::MemoryPort::MemoryPort(const std::string& name,
                                            PIMMediaCtrl& _memory)
    : QueuedSlavePort(name, &_memory, queue), queue(_memory, *this, true),
      memory(_memory)
{ }

AddrRangeList
PIMMediaCtrl::MemoryPort::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(memory.getAddrRange());
    return ranges;
}

void
PIMMediaCtrl::MemoryPort::recvFunctional(PacketPtr pkt)
{
    pkt->pushLabel(memory.name());

    // comment by BenP: trySatisfyFunctional look into the queue
    // and try to satisfy the functional request from there
    if (!queue.trySatisfyFunctional(pkt)) {
        // Default implementation of SimpleTimingPort::recvFunctional()
        // calls recvAtomic() and throws away the latency; we can save a
        // little here by just not calculating the latency.
        memory.recvFunctional(pkt);
    }

    pkt->popLabel();
}

Tick
PIMMediaCtrl::MemoryPort::recvAtomic(PacketPtr pkt)
{
    return memory.recvAtomic(pkt);
}

bool
PIMMediaCtrl::MemoryPort::recvTimingReq(PacketPtr pkt)
{
    // pass it to the memory controller
    return memory.recvTimingReq(pkt);
}

PIMMediaCtrl*
PIMMediaCtrlParams::create()
{
    return new PIMMediaCtrl(this);
}

inline void PIMMediaCtrl::scanArgsSubarrayMats(const Addr& subarrayRowAddr,
                                    const uint8_t* pmemAddr,
                                    uint64_t value[],
                                    const unsigned firstCol,
                                    const unsigned NumOfCols,
                                    const unsigned firstColOffset,
                                    const unsigned lastColLen){
    unsigned value_idx;
    uint8_t tmpCol;
    Addr addrCol;
    // reset the variables for the column value
    std::fill(value, value + pim::CHIPS_PER_RANK*pim::MAT_PER_SUBARRY, 0);
    // go over all the columns of value
    for (int col = 0 ; col < NumOfCols ; col++) {
        addrCol = (subarrayRowAddr & (~ADDR_COL_MASK)) |
                    ADDR_COL_NUM_ARRAY[col+firstCol];
        // go through all mats in a subarray
        // (relevant bits in the cache offset)
        for (Addr chip = 0 ; chip < pim::CHIPS_PER_RANK; chip++){
            addrCol = (addrCol & (~ADDR_CHIP_MASK))
                                    | ADDR_CHIP_NUM_ARRAY[chip];
            for (Addr mat = 0 ; mat < pim::MAT_PER_SUBARRY; mat++){
                addrCol = (addrCol & (~ADDR_MAT_MASK)) |
                            ADDR_MAT_NUM_ARRAY[mat];
                tmpCol = *(pmemAddr+addrCol-range.start());
                // move tmpCol to the appropriate location.
                value_idx = mat+chip*pim::MAT_PER_SUBARRY;
                if (col == NumOfCols-1){ // deal with nonaligned operands
                    if (col == 0){
                        value[value_idx] =
                                    ((uint64_t)(tmpCol &
                                            pim::LAST_COL_MASK[lastColLen]))
                                    >> (firstColOffset);
                    } else {
                        value[value_idx] >>= firstColOffset;
                        // last column is tricky since it can overflow 64 bits
                        value[value_idx] |=
                                    ((uint64_t)(tmpCol &
                                            pim::LAST_COL_MASK[lastColLen]))
                                    << (col*pim::BYTE_SIZE - firstColOffset);
                    }
                } else {
                    value[value_idx] |=
                            ((uint64_t)tmpCol << col*pim::BYTE_SIZE);
                }
            }
        }
    }
}

void PIMMediaCtrl::accessPIM(PacketPtr pkt, uint8_t* pmemAddr){
    //IMPORTENT: This function adjusted on the address view
    Addr addr = pkt->getAddr();
    const uint64_t* data = pkt->getConstPtr<uint64_t>();
    pim::pimOpcode op = (pim::pimOpcode)pim::getFiled(*data,
                            pim::OPCODE_MASK,pim::OPCODE_BIT_START);
    unsigned src1len    = pim::getFiled(*data,
                            pim::SRC1LEN_MASK,pim::SRC1LEN_BIT_START)+1;
    unsigned src1offset = pim::getFiled(*data,
                            pim::SRC1OFFSET_MASK,
                            pim::SRC1OFFSET_BIT_START);
    unsigned src2len    = pim::getFiled(*data,
                            pim::SRC2LEN_MASK,pim::SRC2LEN_BIT_START)+1;
    unsigned src2offset = pim::getFiled(*data,
                            pim::SRC2OFFSET_MASK,
                            pim::SRC2OFFSET_BIT_START);
    uint64_t imm        = pim::getFiled(*data,
                            pim::IMM_MASK,pim::IMM_BIT_START);
    uint64_t imm1       = pim::getFiled(*data,
                            pim::IMM1_MASK,pim::IMM1_BIT_START);
    uint64_t imm2       = pim::getFiled(*data,
                            pim::IMM2_MASK,pim::IMM2_BIT_START);
    unsigned dstCol     = pim::getAddrField(addr,
                            pim::ADDR_COL_IDX,
                            pim::LOG_MAT_COLS-pim::LOG_BYTE_SIZE);
    unsigned dstRow     = pim::getAddrField(addr,
                            pim::ADDR_ROW_IDX,
                            pim::LOG_MAT_ROWS);
    unsigned dstOffset  = pim::getFiled(*data,
                            pim::DSTOFFSET_MASK,pim::DSTOFFSET_BIT_START);

    // dstOffset refer to a MAT read,
    // not a MAT row, so we need to adjust
    dstCol += (dstOffset >> pim::LOG_BYTE_SIZE);
    unsigned dstColOffset       = dstOffset % pim::BYTE_SIZE;
    unsigned dstlen             = pim::getDestLen(op,src1len,src2len,imm);
    unsigned dstlastColLen      = (dstlen + dstOffset) % pim::BYTE_SIZE;
    unsigned dstNumOfCol        = ((dstlen + dstColOffset)>>
                                    pim::LOG_BYTE_SIZE)
                                    + (((dstlen + dstColOffset)%
                                        pim::BYTE_SIZE) != 0);
    unsigned src1Col           = src1offset >> pim::LOG_BYTE_SIZE;
    unsigned src1ColOffset     = src1offset % pim::BYTE_SIZE;
    unsigned src1lastColLen    = (src1len + src1offset) % pim::BYTE_SIZE;
    unsigned src1NumOfCol      = ((src1len + src1ColOffset)>>
                                    pim::LOG_BYTE_SIZE)
                                    + (((src1len + src1ColOffset)%
                                        pim::BYTE_SIZE) != 0);
    unsigned src2Col           = src2offset >> pim::LOG_BYTE_SIZE;
    unsigned src2ColOffset     = src2offset % pim::BYTE_SIZE;
    unsigned src2lastColLen    = (src2len + src2offset) % pim::BYTE_SIZE;
    unsigned src2NumOfCol      = ((src2len + src2ColOffset)>>
                                    pim::LOG_BYTE_SIZE)
                                    + (((src2len + src2ColOffset)%
                                        pim::BYTE_SIZE) != 0);
    uint64_t immMask = (-1) >> (64 - src1len);

    DPRINTF(PIMop,"Op on addr 0x%lX with opcode 0x%X\n",addr,op);
    DPRINTF(PIMop,"range.size() 0x%lX range.start() 0x%lX range.end() 0x%X\n",
                    range.size(),range.start(),range.end());
    DPRINTF(PIMop,"src1len 0x%X src1Col 0x%X src1ColOffset 0x%X"
                    " src1lastColLen 0x%X src1NumOfCol 0x%X\n"
                ,src1len,src1Col,src1ColOffset,src1lastColLen,src1NumOfCol);
    DPRINTF(PIMop,"src2len 0x%X src2Col 0x%X src2ColOffset 0x%X"
                    " src2lastColLen 0x%X src2NumOfCol 0x%X\n"
                ,src2len,src2Col,src2ColOffset,src2lastColLen,src2NumOfCol);
    DPRINTF(PIMop,"dstlen 0x%X dstCol 0x%X dstColOffset 0x%X"
                    " dstlastColLen 0x%X dstNumOfCol 0x%X dstOffset 0x%X\n"
                ,dstlen,dstCol,dstColOffset,dstlastColLen,
                dstNumOfCol,dstOffset);
    DPRINTF(PIMop,"imm 0x%X, imm1 0x%X, imm2 0x%X\n",imm,imm1,imm2);

    // if we just want to sent the intermediate offset then do nothing
    if (op == pim::pimOpcode::SET_INTR)
        return;

    assert(dstCol+dstNumOfCol - 1 <= pim::MAT_COLS/pim::BYTE_SIZE);
    assert(src1Col+src1NumOfCol - 1 <= pim::MAT_COLS/pim::BYTE_SIZE);
    // check src2 only if it is used (other wise it can has unexpected values)
    if ((op == pim::COL_AND) ||
        (op == pim::COL_OR) ||
        (op == pim::COL_MUL) ||
        (op == pim::COL_EQ) ||
        (op == pim::COL_NEQ) ||
        (op == pim::COL_LT) ||
        (op == pim::COL_LTE))
        assert(src2Col+src2NumOfCol - 1 <= pim::MAT_COLS/pim::BYTE_SIZE);

    // we go through all the rows of all the mats in all the subarrays
    // in all the chips that corresponds to the relevant page.
    // we try to save locality by saving the column for all mats in all chips
    // for that subarray and that row (because that is how the address is).
    uint64_t src1[pim::CHIPS_PER_RANK*pim::MAT_PER_SUBARRY]   = {};
    uint64_t src2[pim::CHIPS_PER_RANK*pim::MAT_PER_SUBARRY]   = {};
    uint64_t result[pim::CHIPS_PER_RANK*pim::MAT_PER_SUBARRY] = {};

    // set the pageoffset to be 0 so we are pointing to the
    // first row of the page.
    Addr currMatRowPtr = addr & (~pim::PAGEOFFSET_MASK);

    uint64_t tmpCol;
    Addr addrCol;
    bool writeResult;
    // if the instruction need a second argument
    bool src2read = (op == pim::pimOpcode::COL_EQ) ||
                    (op == pim::pimOpcode::COL_NEQ) ||
                    (op == pim::pimOpcode::COL_LT) ||
                    (op == pim::pimOpcode::COL_LTE) ||
                    (op == pim::pimOpcode::COL_AND) ||
                    (op == pim::pimOpcode::COL_OR) ||
                    (op == pim::pimOpcode::COL_MUL);
    // if instruction is reduce
    bool reduceOp = (op == pim::pimOpcode::REDU_SUM) ||
                    (op == pim::pimOpcode::REDU_MAX) ||
                    (op == pim::pimOpcode::REDU_MIN);
    // if result is not written in every row
    bool notAllRows = reduceOp || (op == pim::pimOpcode::TRNS_COL);

    // check if this is a MUL operation, and if so
    // if the simulation support it.
    // there is no real issue here except that this
    // code does not go the extra mile to support multiplication
    // with the result greater then 64bit.
    assert((op != pim::pimOpcode::COL_MUL) || (src1len+src2len <= 64));

    // go over all the subarrays, rows, chips, and mats for that
    // PIM controller. i.e. that page. We go over with this order
    // to get as much spatial locality in the host as possible.
    // The locality is depended on the address sectioning.
    // Since our mat and chip indexing is mostly lower then the
    // column indexes when we look at the address, then we loop
    // on the mats and chips for every new column.
    for (Addr subarray = 0 ; subarray < pim::SUBARRAYS_PER_PIMCTRL ;
                                                        subarray ++) {
        currMatRowPtr = (currMatRowPtr & (~ADDR_SUBARRYS_PER_PIMCTRL_MASK)) |
                            ADDR_SUBARRYS_PER_PIMCTRL_NUM_ARRAY[subarray];
        for (Addr row = 0 ; row < pim::MAT_ROWS ; row ++) {
            currMatRowPtr = (currMatRowPtr & (~ADDR_ROW_MASK)) |
                                ADDR_ROW_NUM_ARRAY[row];
            // read sources only when needed to
            if (op != pim::pimOpcode::COLIMM_SET){
                // set the address for the row and subarray
                scanArgsSubarrayMats(currMatRowPtr, pmemAddr, src1,
                                    src1Col, src1NumOfCol,
                                    src1ColOffset, src1lastColLen);
                // read src2 only when needed to
                if (src2read){
                    // set the address for the row and subarray
                    scanArgsSubarrayMats(currMatRowPtr, pmemAddr,
                                        src2, src2Col, src2NumOfCol,
                                        src2ColOffset, src2lastColLen);
                }
            }
            //DPRINTF(PIMop,"subarray 0x%X row 0x%X\n", subarray, row);
            //int const mat_chip_print_idx = 6;
            for (int i=0;i<pim::CHIPS_PER_RANK*pim::MAT_PER_SUBARRY;i++){
                switch(op){
                    case pim::pimOpcode::SCAN_EQ:
                        result[i] = (src1[i] == (imm & immMask));
                        break;
                    case pim::pimOpcode::SCAN_NEQ:
                        result[i] = (src1[i] != (imm & immMask)); break;
                    case pim::pimOpcode::SCAN_LT:
                        result[i] = (src1[i] < (imm & immMask)); break;
                    case pim::pimOpcode::SCAN_LTEQ:
                        result[i] = (src1[i] <= (imm & immMask)); break;
                    case pim::pimOpcode::SCAN_GT:
                        result[i] = (src1[i] > (imm & immMask)); break;
                    case pim::pimOpcode::SCAN_GTEQ:
                        result[i] = (src1[i] >= (imm & immMask)); break;
                    case pim::pimOpcode::SCAN_BTWN:
                        result[i] = (((imm1 & immMask) < src1[i])
                                & (src1[i] < (imm2 & immMask))); break;
                    case pim::pimOpcode::REDU_SUM:
                        /*if ((subarray == 0) && (i == mat_chip_print_idx)){
                            DPRINTF(PIMop,"row %d : value in REDU_SUM %ld\n",
                            row,src1[i]);
                        }*/
                        result[i] += src1[i];
                        break;
                    case pim::pimOpcode::REDU_MIN:
                        // since result is init to 0 we
                        // need set it to src1 at the first row
                        result[i] = (row==0) ?
                            src1[i] : std::min(result[i],src1[i]);
                        break;
                    case pim::pimOpcode::REDU_MAX:
                        result[i] = std::max(result[i],src1[i]);
                        break;
                    case pim::pimOpcode::COLIMM_ADD:
                        result[i] = src1[i] + imm; break;
                    case pim::pimOpcode::COLIMM_SUB:
                        result[i] = imm - src1[i];
                        /*if ((subarray == 0) && (i == mat_chip_print_idx)){
                            DPRINTF(PIMop,
                            "row %d : value in COLIMM_SUB src1 %ld \t
                             src2 %ld \t res %ld\n"
                            ,row,src1[i],src2[i],result[i]);
                        }*/
                        break;
                    case pim::pimOpcode::COLIMM_SET:
                        result[i] = imm ? ((1U << src1len) - 1) : 0;
                        break;
                    case pim::pimOpcode::COL_NOT:
                        result[i] = ~src1[i]; break;
                    case pim::pimOpcode::COL_AND:
                        /*if ((subarray == 0) && (i == mat_chip_print_idx)){
                            DPRINTF(PIMop,"row %d : value in AND src1 %ld \t"\
                            "src2 %ld\n",
                            row,src1[i],src2[i]);
                        }*/
                        result[i] = src1[i] & src2[i]; break;
                    case pim::pimOpcode::COL_OR:
                        result[i] = src1[i] | src2[i]; break;
                    case pim::pimOpcode::COL_MUL:
                        /*if ((subarray == 0) && (i == mat_chip_print_idx)){
                            DPRINTF(PIMop,"row %d : value in MUL src1 %ld \t
                            src2 %ld\n",
                            row,src1[i],src2[i]);
                        }*/
                        result[i] = src1[i] * src2[i]; break;
                    case pim::pimOpcode::COL_EQ:
                        result[i] = (src1[i] == src2[i]); break;
                    case pim::pimOpcode::COL_NEQ:
                        result[i] = (src1[i] != src2[i]); break;
                    case pim::pimOpcode::COL_LT:
                        result[i] = (src1[i] < src2[i]); break;
                    case pim::pimOpcode::COL_LTE:
                        result[i] = (src1[i] <= src2[i]); break;
                    case pim::pimOpcode::TRNS_COL:
                        result[i] |=
                            (src1[i] & 1U) <<
                            (row % pim::MAT_READ_SIZE);
                        break;
                    default:
                        printf("ERROR: PIM try to execute an unknown"\
                                "opcode, terminating the simulation.");
                        std::abort();
                }
            }

            // check if we need to write actual data in this row.
            // otherwise we just write 0, as if the result bits
            //were used in the operation.
            writeResult = !notAllRows ||
                        (reduceOp &&
                        ((row % pim::MAT_ROWS) == (pim::MAT_ROWS - 1))
                        ) ||
                        ((op == pim::TRNS_COL) &&
                        ((row % pim::MAT_READ_SIZE)
                        == (pim::MAT_READ_SIZE - 1))
                        );
            // go over all the columns of value
            for (int col = 0 ; col < dstNumOfCol ; col++) {
                addrCol = (currMatRowPtr & (~ADDR_COL_MASK)) |
                            ADDR_COL_NUM_ARRAY[col+dstCol];
                // go through all mats in a subarray
                // (only relevant bits in the cache offset)
                for (Addr chip = 0 ; chip < pim::CHIPS_PER_RANK; chip++){
                    addrCol = (addrCol & (~ADDR_CHIP_MASK)) |
                                ADDR_CHIP_NUM_ARRAY[chip];
                    for (Addr mat = 0 ; mat < pim::MAT_PER_SUBARRY; mat++){
                        addrCol = (addrCol & (~ADDR_MAT_MASK)) |
                                    ADDR_MAT_NUM_ARRAY[mat];
                        unsigned value_idx = mat+chip*pim::MAT_PER_SUBARRY;
                        // shiftright with negative is not shiftleft
                        if (col == 0){
                            tmpCol = (writeResult && !reduceOp) ?
                                result[value_idx] << dstColOffset : 0;
                            // adding the non-touched data
                            uint8_t mask =
                                pim::FIRST_COL_ANTYMASK[dstColOffset];
                            // if this is the last col then we need
                            // to take that into account
                            mask |= (col == dstNumOfCol - 1)*
                                        (~pim::LAST_COL_MASK[dstlastColLen]);
                            tmpCol = (*(pmemAddr+addrCol-range.start())
                                            & mask) |
                                        (tmpCol & ~mask);
                        } else {
                            // we need the shiftright like this because of
                            // a possible under/overflow
                            tmpCol = (writeResult && !reduceOp) ?
                                    result[value_idx] >>
                                    (col*pim::BYTE_SIZE - dstColOffset) :
                                    0;
                            if (col == dstNumOfCol - 1){
                                // adding the non-touched data
                                tmpCol &= pim::LAST_COL_MASK[dstlastColLen];
                                tmpCol |= ~pim::LAST_COL_MASK[dstlastColLen]
                                         & *(pmemAddr+addrCol-range.start());
                            }
                        }
                        /*if (writeResult){
                            DPRINTF(PIMop,"set: print in row 0x%X,"
                                " set in row : 0x%X col 0x%X,"
                                " mat 0x%X, chip 0x%X, subarray 0x%X,"
                                " tmpCol = 0x%X\n",
                            row,dstRow,col,mat,chip,subarray,tmpCol);
                        }*/
                        // set the byte
                        *(pmemAddr+addrCol-range.start()) = (uint8_t)tmpCol;
                        // if we use reduce, we can choose the destination row
                        // by the row in the address. We assume that in a
                        // reduce there is only one writeResult in a mat
                        if (reduceOp && writeResult){
                            // change the address to point to the actual
                            // destination row
                            addrCol = (addrCol & (~ADDR_ROW_MASK)) |
                                ADDR_ROW_NUM_ARRAY[dstRow];
                            // do the write to the destination row
                            // shiftright with negative is not shiftleft
                            if (col == 0){
                                tmpCol = result[value_idx] << dstColOffset;
                                // adding the non-touched data
                                uint8_t mask =
                                    pim::FIRST_COL_ANTYMASK[dstColOffset];
                                // if this is the last col then we need
                                // to take that into account
                                mask |= (col == dstNumOfCol - 1)*
                                    (~pim::LAST_COL_MASK[dstlastColLen]);
                                tmpCol = (*(pmemAddr+addrCol-range.start())
                                                & mask) |
                                            (tmpCol & ~mask);
                            } else {
                                // we need the shiftright like this because of
                                // a possible under/overflow
                                tmpCol = result[value_idx] >>
                                        (col*pim::BYTE_SIZE - dstColOffset);
                                if (col == dstNumOfCol - 1){
                                    // adding the non-touched data
                                    tmpCol &=
                                        pim::LAST_COL_MASK[dstlastColLen];
                                    tmpCol |=
                                        ~pim::LAST_COL_MASK[dstlastColLen]
                                        & *(pmemAddr+addrCol-range.start());
                                }
                            }
                            *(pmemAddr+addrCol-range.start()) =
                                            (uint8_t)tmpCol;
                            /*DPRINTF(PIMop,"set: print in row 0x%X,"
                                " set in row : 0x%X col 0x%X,"
                                " mat 0x%X, chip 0x%X, subarray 0x%X,"
                                " tmpCol = 0x%X\n",
                            row,dstRow,col,mat,chip,subarray,tmpCol);*/
                        }
                    }
                }
            }

            // if we have an op that write only to some rows,
            // and we just wrote then reset the result variable
            if (writeResult & notAllRows)
                std::fill(result,
                    result + pim::CHIPS_PER_RANK*pim::MAT_PER_SUBARRY, 0);
        }
    }
}

void PIMMediaCtrl::opEnergyAssignment(const uint64_t* pimOpData,
                                        Rank* rank,
                                        uint8_t bank,
                                        Tick time_stamp,
                                        Tick end_time,
                                        const Tick cycle_time){
    unsigned op = pim::getFiled(*pimOpData,pim::OPCODE_MASK,
                                    pim::OPCODE_BIT_START);
    unsigned src1len    = pim::getFiled(*pimOpData,
                            pim::SRC1LEN_MASK,pim::SRC1LEN_BIT_START)+1;
    Tick start_time = time_stamp;
    Tick finish_time;
    unsigned cycels;
    switch(op){
        case pim::pimOpcode::SCAN_EQ:
        case pim::pimOpcode::SCAN_NEQ:
        case pim::pimOpcode::SCAN_LT:
        case pim::pimOpcode::SCAN_LTEQ:
        case pim::pimOpcode::SCAN_GT:
        case pim::pimOpcode::SCAN_GTEQ:
        case pim::pimOpcode::SCAN_BTWN:
        case pim::pimOpcode::COLIMM_ADD:
        case pim::pimOpcode::COLIMM_SUB:
        case pim::pimOpcode::COLIMM_SET:
        case pim::pimOpcode::COL_NOT:
        case pim::pimOpcode::COL_AND:
        case pim::pimOpcode::COL_OR:
        case pim::pimOpcode::COL_MUL:
        case pim::pimOpcode::COL_EQ:
        case pim::pimOpcode::COL_NEQ:
        case pim::pimOpcode::COL_LT:
        case pim::pimOpcode::COL_LTE:
            cycels = pim::pimOpCycles(pimOpData);
            finish_time = start_time+cycels*cycle_time;
            rank->cmdList.push_back(Command(commandType::OP_FULL_N,
                                    bank,
                                    start_time,
                                    finish_time));
            DPRINTF(PIMop,"command pushed to rank list,"
                    " op: 0x%X, src1len: 0x%X, cycles: %ld,"
                    " start: %ld, finish: %ld, cycle_time: %ld\n"
                ,op,src1len,cycels,start_time,finish_time,
                    cycle_time);
            assert(end_time >= finish_time);
            break;
        case pim::pimOpcode::REDU_SUM:
        case pim::pimOpcode::REDU_MIN:
        case pim::pimOpcode::REDU_MAX:
            finish_time = start_time;
            for (int itr = pim::LOG_MAT_ROWS - 1 ; itr >= 0 ; itr--){
                // the first section is parallel column move, to move the
                // values to the desired column. Number of cycles is twice
                // the the length of the operand
                start_time = finish_time;
                cycels = 2*src1len;
                finish_time = start_time + cycels*cycle_time;
                rank->cmdList.push_back(Command(commandType::OP_FULL_N,
                                    bank,
                                    start_time,
                                    finish_time));
                DPRINTF(PIMop,"command pushed to rank list,"
                    " reduce parallel column move ,"
                    " op: 0x%X, src1len: 0x%X, cycles: %ld,"
                    " start: %ld, finish: %ld, cycle_time: %ld\n"
                ,op,src1len,cycels,start_time,finish_time,
                    cycle_time);
                // the next move is to transfer the values from thier rows
                // to the designated rows. It takes 2 cycles for each bit.
                // The itr (the look counter) is log2 for the number of rows
                // that need to be transfered, while src1len is the length
                // of the current operand
                start_time = finish_time;
                cycels = 2*(1U<<itr)*src1len;
                finish_time = start_time + cycels*cycle_time;
                rank->cmdList.push_back(Command(commandType::OP_SNGL_N,
                                    bank,
                                    start_time,
                                    finish_time));
                DPRINTF(PIMop,"command pushed to rank list,"
                    " reduce serial row move,"
                    " op: 0x%X, src1len: 0x%X, cycles: %ld,"
                    " start: %ld, finish: %ld, cycle_time: %ld\n"
                ,op,src1len,cycels,start_time,finish_time,
                    cycle_time);
                // the next move is to perform the operation on all of the rows
                start_time = finish_time;
                if (op == pim::pimOpcode::REDU_SUM) {
                    cycels = src1len*pim::FA_CYCLE;
                    src1len++; // every iteration increase the operand size
                } else { // MAX and MIN have the same cycle count
                    cycels = (24*src1len+2);
                }
                finish_time = start_time + cycels*cycle_time;
                rank->cmdList.push_back(Command(commandType::OP_FULL_N,
                                    bank,
                                    start_time,
                                    finish_time));
                DPRINTF(PIMop,"command pushed to rank list, reduce op,"
                    " op: 0x%X, src1len: 0x%X, cycles: %ld,"
                    " start: %ld, finish: %ld, cycle_time: %ld\n"
                    ,op,src1len,cycels,start_time,finish_time,
                    cycle_time);
            }
            // after computing the aggregation, it is transferred
            // to the desired row
            start_time = finish_time;
            cycels = 4*src1len;
            finish_time = start_time + cycels*cycle_time;
            rank->cmdList.push_back(Command(commandType::OP_SNGL_N,
                                bank,
                                start_time,
                                finish_time));
            DPRINTF(PIMop,"command pushed to rank list,"
                " reduce end serial row move,"
                " op: 0x%X, src1len: 0x%X, cycles: %ld,"
                " start: %ld, finish: %ld, cycle_time: %ld\n"
            ,op,src1len,cycels,start_time,finish_time,
                cycle_time);
            assert(end_time >= finish_time);
            break;
        case pim::pimOpcode::TRNS_COL:
            // the first stage is parallel transfer of all bits
            cycels = 2*pim::MAT_READ_SIZE+2;
            finish_time = start_time + cycels*cycle_time;
            rank->cmdList.push_back(Command(commandType::OP_FULL_N,
                                    bank,
                                    start_time,
                                    finish_time));
            DPRINTF(PIMop,"command pushed to rank list,"
                    " col-trans parallel part,"
                    " op: 0x%X, src1len: 0x%X, cycles: %ld,"
                    " start: %ld, finish: %ld, cycle_time: %ld\n"
                    ,op,src1len,cycels,start_time,finish_time,
                    cycle_time);
            // the second stage is to transfer all bits to the sesignated
            // rows serially.
            start_time = finish_time;
            cycels = 2*(pim::MAT_ROWS-pim::MAT_READ_SIZE);
            finish_time = start_time + cycels*cycle_time;
            rank->cmdList.push_back(Command(commandType::OP_SNGL_N,
                                    bank,
                                    start_time,
                                    finish_time));
            DPRINTF(PIMop,"command pushed to rank list,"
                    " col-trans serial part,"
                    " op: 0x%X, src1len: 0x%X, cycles: %ld,"
                    " start: %ld, finish: %ld, cycle_time: %ld\n"
                    ,op,src1len,cycels,start_time,finish_time,
                    cycle_time);
            assert(end_time >= finish_time);
            break;
        case pim::pimOpcode::SET_INTR:
            break;
        default:
            printf("ERROR: PIM try to compute energy an unknown"\
                        "opcode, terminating the simulation.");
            std::abort();
    }
}


// For power considerations, this function defines if the
// PIM operation does a single MAGIC operation in each
// cycle (return false) or a full MAT width operation on
// every mat (return true).
// There are operation in between (not fully parallel and not
// fully sequential), like reduce. For simplicity we refer
// to them as fully parallel (because they are closer to
// to that and because it is the pessimist option)
bool PIMMediaCtrl::pimFullParallel(const uint64_t* pimOpData){
    unsigned op = pim::getFiled(*pimOpData,pim::OPCODE_MASK,
                                    pim::OPCODE_BIT_START);
    switch(op){
        case pim::pimOpcode::SCAN_EQ:
        case pim::pimOpcode::SCAN_NEQ:
        case pim::pimOpcode::SCAN_LT:
        case pim::pimOpcode::SCAN_LTEQ:
        case pim::pimOpcode::SCAN_GT:
        case pim::pimOpcode::SCAN_GTEQ:
        case pim::pimOpcode::SCAN_BTWN:
        case pim::pimOpcode::REDU_SUM:
        case pim::pimOpcode::REDU_MIN:
        case pim::pimOpcode::REDU_MAX:
        case pim::pimOpcode::COLIMM_ADD:
        case pim::pimOpcode::COLIMM_SUB:
        case pim::pimOpcode::COLIMM_SET:
        case pim::pimOpcode::COL_NOT:
        case pim::pimOpcode::COL_AND:
        case pim::pimOpcode::COL_OR:
        case pim::pimOpcode::COL_MUL:
        case pim::pimOpcode::COL_EQ:
        case pim::pimOpcode::COL_NEQ:
        case pim::pimOpcode::COL_LT:
        case pim::pimOpcode::COL_LTE:
            return true;
            break;
        case pim::pimOpcode::TRNS_COL:
            return false;
            break;
        case pim::pimOpcode::SET_INTR:
            return false;
            break;
        default:
            printf("ERROR: PIM try to schedule an unknown"\
                        "opcode, terminating the simulation.");
            std::abort();
    }
}

