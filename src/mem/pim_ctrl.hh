/*
 * Copyright (c) 2012-2018 ARM Limited
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
 * Copyright (c) 2020 Ben Perach
 * All rights reserved.
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
 *          Matthias Jung
 *          Wendy Elsasser
 *          Radhika Jagtap
 */

/**
 * @file
 * PIMMediaCtrl declaration
 */

#ifndef __MEM_PIM_CTRL_HH__
#define __MEM_PIM_CTRL_HH__

#include <deque>
#include <string>
#include <unordered_set>
#include <vector>

//#include <math>

#include "base/callback.hh"
#include "base/statistics.hh"
#include "enums/AddrMap.hh"
#include "enums/MemSched.hh"
#include "enums/PageManage.hh"
#include "mem/drampower.hh"
#include "mem/pim_lib.hh"
#include "mem/qos/mem_ctrl.hh"
#include "mem/qport.hh"
#include "params/PIMMediaCtrl.hh"
#include "sim/eventq.hh"

/**
 * The DRAM controller is a single-channel memory controller capturing
 * the most important timing constraints associated with a
 * contemporary DRAM. For multi-channel memory systems, the controller
 * is combined with a crossbar model, with the channel address
 * interleaving taking part in the crossbar.
 *
 * As a basic design principle, this controller
 * model is not cycle callable, but instead uses events to: 1) decide
 * when new decisions can be made, 2) when resources become available,
 * 3) when things are to be considered done, and 4) when to send
 * things back. Through these simple principles, the model delivers
 * high performance, and lots of flexibility, allowing users to
 * evaluate the system impact of a wide range of memory technologies,
 * such as DDR3/4, LPDDR2/3/4, WideIO1/2, HBM and HMC.
 *
 * This is the Media controller for the PIM. Written by BenP.
 * This module is base on the DRAM module of gem5 with heavy
 * modifications. There are no power states, no refreshes,
 * no activations, as this is intended to be RRAM technology.
 */
class PIMMediaCtrl : public QoS::MemCtrl
{

  private:

    // For now, make use of a queued slave port to avoid dealing with
    // flow control for the responses being sent back
    class MemoryPort : public QueuedSlavePort
    {

        RespPacketQueue queue;
        PIMMediaCtrl& memory;

      public:

        MemoryPort(const std::string& name, PIMMediaCtrl& _memory);

      protected:

        Tick recvAtomic(PacketPtr pkt);

        void recvFunctional(PacketPtr pkt);

        bool recvTimingReq(PacketPtr);

        virtual AddrRangeList getAddrRanges() const;

    };

    /**
     * Our incoming port, for a multi-ported controller add a crossbar
     * in front of it
     */
    MemoryPort port;

    /**
     * Remember if the memory system is in timing mode
     */
    bool isTimingMode;

    /**
     * Remember if we have to retry a request when available.
     */
    bool retryRdReq;
    bool retryWrReq;

    // temporarily variables to save the next request update
    // if we search in both read and write queues
    bool immediateSwitch;
    bool tmp_hasReadyPacket;
    Tick tmp_nextServTick;

    /**
     * Simple structure to hold the values needed to keep track of
     * commands for DRAMPower.
     */
    enum commandType{
        READ_H, // buffer hit
        READ_M, // buffer miss
        WRITE,
        OP_FULL_N, // operating on a full column, New
        OP_FULL_C, // operating on a full column, Continued
        OP_SNGL_N, // operating on a single cell each cycle, New
        OP_SNGL_C  // operating on a single cell each cycle, Continued
    };

    struct Command {
        //Data::MemCommand::cmds type;
        commandType type;
        uint8_t bank;
        Tick timeStamp;
        Tick endTime;
        constexpr Command(commandType _type, uint8_t _bank,
                         Tick time_stamp, Tick end_time)
            : type(_type), bank(_bank), timeStamp(time_stamp),
            endTime(end_time)
        { }
    };

    /**
     * A basic class to track the PIM controller state, i.e. what
     * is the first time we can operate on a from the region under
     * the PIM controller.
     * Each bank has many PIM controllers. PIM controllers on
     * different
     */
  public:
    class PIMCtrl
    {
      public:
        Tick opAllowedAt;
        Tick opCycleTime;
        Stats::Scalar totalOpCycles;
        Stats::Formula totalOpTime;
        std::string name;
        // 2<<OPCODE_LEN = pow(2,OPCODE_LEN)
        //uint32_t opCount[2<<pim::OPCODE_LEN];
        PIMCtrl() :
            opAllowedAt(0), // init all zeros
            opCycleTime(0),
            name("")
        { }

        /*
         * Function to register Stats
         */
        void regStats();
    };

  private:
    /**
     * A basic class to track the bank state, i.e. what row is
     * currently open (if any), when is the bank free to accept a new
     * column (read/write) command, when can it be precharged, and
     * when can it be activated.
     *
     * The bank also keeps track of how many bytes have been accessed
     * in the open row since it was opened.
     */
    class Bank
    {

      public:

        static const uint32_t NO_ROW = -1;
        static const uint32_t NO_COL = -1;

        // no open row, assuming closed page
        uint32_t openRow;
        uint32_t openRowPIM;
        uint32_t openCol;
        uint8_t bank;
        uint8_t bankgr;

        Tick rdAllowedAt;
        Tick wrAllowedAt;

        PIMCtrl* pimCtrls[pim::PIMCTRL_PER_BANK];

        uint32_t rowAccesses;
        uint32_t bytesAccessed;

        Bank() :
            openRow(NO_ROW), openRowPIM(NO_ROW), openCol(NO_COL),
            bank(0), bankgr(0),
            rdAllowedAt(0), wrAllowedAt(0),
            rowAccesses(0), bytesAccessed(0)
        {
            for (int i = 0; i < pim::PIMCTRL_PER_BANK ; i++){
                pimCtrls[i] = new PIMCtrl();
            }
        }
    };

    // For power considerations, this function defines if the
    // PIM operation does a single MAGIC operation in each
    // cycle (return false) or a full MAT width operation on
    // every mat (return true).
    // There are operation in between (not fully parallel and not
    // fully sequential), like reduce. For simplicity we refer
    // to them as fully parallel (because they are closer to
    // to that and because it is the pessimist option).
    bool pimFullParallel(const uint64_t* pimOpData);

    /**
     * Rank class includes a vector of banks. Refresh and Power state
     * machines are defined per rank. Events required to change the
     * state of the refresh and power state machine are scheduled per
     * rank. This class allows the implementation of rank-wise refresh
     * and rank-wise power-down.
     */
    class Rank : public EventManager
    {

      private:

        /**
         * A reference to the parent PIMMediaCtrl instance
         */
        PIMMediaCtrl& memory;

        /*
         * Command energies
         */
        Stats::Scalar readEnergy;
        Stats::Scalar periphReadEnergy;
        Stats::Scalar arrayReadEnergy;
        Stats::Scalar writeEnergy;
        Stats::Scalar periphWriteEnergy;
        Stats::Scalar arrayWriteEnergy;
        Stats::Scalar ioTermEnergy;
        Stats::Scalar arrayOpEnergy;
        Stats::Scalar pimCtrlsSwitchEnergy;
        Stats::Scalar pimCtrlsLeakageEnergy;

        unsigned powerBinCount;
        Stats::Scalar totalEnergy;
        Stats::Scalar averagePower;
        Stats::Histogram windowAveragePower;

        /**
         * Function to update Power Stats
         */
        void updatePowerStats();

        bool isPowerEventStartedUp;

      public:
       /**
         * Track number of packets in read queue going to this rank
         */
        uint32_t readEntries;

       /**
         * Track number of packets in write queue going to this rank
         */
        uint32_t writeEntries;

        /**
         * One DRAMPower instance per rank
         */
        DRAMPower power;

        /**
         * List of commands issued, to be sent to DRAMPpower at refresh
         * and stats dump.  Keep commands here since commands to different
         * banks are added out of order.  Will only pass commands up to
         * curTick() to DRAMPower after sorting.
         */
        std::vector<Command> cmdList;

        /**
         * Vector of Banks. Each rank is made of several devices which in
         * term are made from several banks.
         */
        std::vector<Bank> banks;

        Rank(PIMMediaCtrl& _memory, const PIMMediaCtrlParams* _p);

        const std::string name() const
        {
            return csprintf("%s", memory.name());
        }

        /*
         * Function to register Stats
         */
        void regStats();

        /**
         * Computes stats just prior to dump event
         */
        void computeStats();

        /**
         * Reset stats on a stats event
         */
        void resetStats();

        void processPeriodPowerStatsEvent();
        void startUpPowerStatsEvent();
        EventFunctionWrapper periodPowerStatsEvent;
        Tick lastWindowEnd;
        Tick windowDuration;

    };

    // For op energy considerations. For filter and arithmetic
    // it is simple, but for col/row transfer and reduction,
    // there are several stages, each one parallel or serial.
    void opEnergyAssignment(const uint64_t* pimOpData,Rank* rank,
                            uint8_t bank,
                            Tick time_stamp,
                            Tick end_time,
                            const Tick cycle_time);

    /**
     * Define the process to compute stats on a stats dump event, e.g. on
     * simulation exit or intermediate stats dump. This is defined per rank
     * as the per rank stats are based on state transition and periodically
     * updated, requiring re-sync at exit.
     */
    class RankDumpCallback : public Callback
    {
        Rank *ranks;
      public:
        RankDumpCallback(Rank *r) : ranks(r) {}
        virtual void process() { ranks->computeStats(); };
    };

    /** Define a process to clear power lib counters on a stats reset */
    class RankResetCallback : public Callback
    {
      private:
        /** Pointer to the rank, thus we instantiate per rank */
        Rank *rank;

      public:
        RankResetCallback(Rank *r) : rank(r) {}
        virtual void process() { rank->resetStats(); };
    };

    /** Define a process to store the time on a stats reset */
    class MemResetCallback : public Callback
    {
      private:
        /** A reference to the PIMMediaCtrl instance */
        PIMMediaCtrl *mem;

      public:
        MemResetCallback(PIMMediaCtrl *_mem) : mem(_mem) {}
        virtual void process() { mem->lastStatsResetTick = curTick(); };
    };

    /**
     * A burst helper helps organize and manage a packet that is larger than
     * the DRAM burst size. A system packet that is larger than the burst size
     * is split into multiple DRAM packets and all those DRAM packets point to
     * a single burst helper such that we know when the whole packet is served.
     */
    class BurstHelper {

      public:

        /** Number of DRAM bursts requred for a system packet **/
        const unsigned int burstCount;

        /** Number of DRAM bursts serviced so far for a system packet **/
        unsigned int burstsServiced;

        BurstHelper(unsigned int _burstCount)
            : burstCount(_burstCount), burstsServiced(0)
        { }
    };

    /**
     * A DRAM packet stores packets along with the timestamp of when
     * the packet entered the queue, and also the decoded address.
     */
    class PIMPacket {

      public:

        /** When did request enter the controller */
        const Tick entryTime;

        /** When will request leave the controller */
        Tick readyTime;

        /** This comes from the outside world */
        const PacketPtr pkt;

        /** MasterID associated with the packet */
        const MasterID _masterId;

        const bool op;
        const bool read;

        /** Will be populated by address decoder.
        There is no rank here because every PIM entity
        is a unique rank by itself, by arriving to a certain
        PIM the rank is known.
        */
        const uint8_t bank;
        const uint32_t subarray;
        const uint32_t pimCtrl;
        const uint32_t row;
        const uint32_t col;

        /**
         * Bank id is calculated considering banks in all the ranks
         * eg: 2 ranks each with 8 banks, then bankId = 0 --> rank0, bank0 and
         * bankId = 8 --> rank1, bank0
         */
        const uint16_t bankId;

        /**
         * The starting address of the DRAM packet.
         * This address could be unaligned to burst size boundaries. The
         * reason is to keep the address offset so we can accurately check
         * incoming read packets with packets in the write queue.
         */
        Addr addr;

        /**
         * The size of this dram packet in bytes
         * It is always equal or smaller than DRAM burst size
         */
        unsigned int size;

        /**
         * A pointer to the BurstHelper if this PIMPacket is a split packet
         * If not a split packet (common case), this is set to NULL
         */
        BurstHelper* burstHelper;
        Bank& bankRef;

        /**
         * Current Rank index
         */
        //uint8_t rank;

        /**
         * QoS value of the encapsulated packet read at queuing time
         */
        uint8_t _qosValue;

        /**
        * A pointer to the packet that we are depended on.
        * For reads, they can be depended on an PIM op in the writeQueue.
        * However, they might as well be depended only on the latest PIM op,
        * since the dependency between the ops will take care for all the rest.
        * Reads cannot be depended on a write since we can take the data from
        * the middle of the queue. Also, reads do not depended on reads.
        * For writes and PIM ops, they can be depended on other writes or PIM
        * ops, but since they are all in the same queue, and it is a FIFO this
        * dependency is taking care off. Writes and PIM ops can be depended on
        * reads. Here also, it is sufficient to relay on the latest read,
        * since the read queue is also FIFO.
        **/
        bool dependedOn;

        /**
        * A set to include all PIMpackets that are depended
        * on the current packet. When we satisfy the current packet
        * we go to all packets in this set and ture their dependedOn
        * to false.
        **/
        std::unordered_set<PIMPacket*> dependeds;

        /**
         * Set the packet QoS value
         * (interface compatibility with Packet)
         */
        inline void qosValue(const uint8_t qv) { _qosValue = qv; }

        /**
         * Get the packet QoS value
         * (interface compatibility with Packet)
         */
        inline uint8_t qosValue() const { return _qosValue; }

        /**
         * Get the packet MasterID
         * (interface compatibility with Packet)
         */
        inline MasterID masterId() const { return _masterId; }

        /**
         * Get the packet size
         * (interface compatibility with Packet)
         */
        inline unsigned int getSize() const { return size; }

        /**
         * Get the packet address
         * (interface compatibility with Packet)
         */
        inline Addr getAddr() const { return addr; }

        /**
         * Return true if its a read packet
         * (interface compatibility with Packet)
         */
        inline bool isRead() const { return read & !op; }

        /**
         * Return true if its a write packet
         * (interface compatibility with Packet)
         */
        inline bool isWrite() const { return !read & !op; }

        inline bool isOp() const { return op;}

        PIMPacket(PacketPtr _pkt, bool is_read, bool is_op,
                    uint8_t _bank, uint8_t _subarray,
                    uint32_t _pimCtrl, uint32_t _row,
                    uint32_t _col, uint16_t bank_id,
                   Addr _addr, unsigned int _size, Bank& bank_ref)
            : entryTime(curTick()), readyTime(curTick()), pkt(_pkt),
              _masterId(pkt->masterId()), op(is_op), read(is_read),
              bank(_bank), subarray(_subarray),
              pimCtrl(_pimCtrl), row(_row), col(_col),
              bankId(bank_id),
              addr(_addr), size(_size), burstHelper(NULL),
              bankRef(bank_ref), _qosValue(_pkt->qosValue()),
              dependedOn(false)
        { }

    };

    // The DRAM packets are store in a multiple dequeue structure,
    // based on their QoS priority
    typedef std::deque<PIMPacket*> PIMPacketQueue;

    // save the next time a packet is suppose to be ready
    // to be served.
    inline bool updateNextReqEvent(Tick pkt_at);
    inline bool updateNextReqEvent(PIMPacket& pim_pkt);

    /**
     * Bunch of things requires to setup "events" in gem5
     * When event "respondEvent" occurs for example, the method
     * processRespondEvent is called; no parameters are allowed
     * in these methods
     */
    void processNextReqEvent();
    EventFunctionWrapper nextReqEvent;

    void processRespondEvent();
    EventFunctionWrapper respondEvent;

    /**
     * Check if the read queue has room for more entries
     *
     * @param pktCount The number of entries needed in the read queue
     * @return true if read queue is full, false otherwise
     */
    bool readQueueFull(unsigned int pktCount) const;

    /**
     * Check if the write queue has room for more entries
     *
     * @param pktCount The number of entries needed in the write queue
     * @return true if write queue is full, false otherwise
     */
    bool writeQueueFull(unsigned int pktCount) const;

    /**
     * When a new read comes in, first check if the write q has a
     * pending request to the same address.\ If not, decode the
     * address to populate rank/bank/row, create one or mutliple
     * "dram_pkt", and push them to the back of the read queue.\
     * If this is the only
     * read request in the system, schedule an event to start
     * servicing it.
     *
     * @param pkt The request packet from the outside world
     * @param pktCount The number of DRAM bursts the pkt
     * translate to. If pkt size is larger then one full burst,
     * then pktCount is greater than one.
     */
    void addToReadQueue(PacketPtr pkt, unsigned int pktCount);

    /**
     * Decode the incoming pkt, create a dram_pkt and push to the
     * back of the write queue. \If the write q length is more than
     * the threshold specified by the user, ie the queue is beginning
     * to get full, stop reads, and start draining writes.
     *
     * @param pkt The request packet from the outside world
     * @param pktCount The number of DRAM bursts the pkt
     * translate to. If pkt size is larger then one full burst,
     * then pktCount is greater than one.
     */
    void addToWriteQueue(PacketPtr pkt, unsigned int pktCount);

    void accessPIM(PacketPtr pkt, uint8_t* pmemAddr);
    inline void scanArgsSubarrayMats(const Addr& subarrayRowAddr,
                                    const uint8_t* pmemAddr,
                                    uint64_t value[],
                                    const unsigned firstCol,
                                    const unsigned NumOfCols,
                                    const unsigned firstColOffset,
                                    const unsigned lastColLen);
    // defining arrays to hold constant value for PIM operation
    // implementation. I don't know how to make those arrays
    // be computed at compile time and be constants, so they
    // are computed on generated time and some of them are
    // constants.
    Addr ADDR_MAT_NUM_ARRAY[1U<<pim::LOG_MAT_PER_SUBARRY];
    Addr ADDR_CHIP_NUM_ARRAY[1U<<pim::LOG_CHIPS_PER_RANK];
    Addr ADDR_ROW_NUM_ARRAY[1U<<pim::LOG_MAT_ROWS];
    Addr ADDR_COL_NUM_ARRAY[1U<<(pim::LOG_MAT_COLS-pim::LOG_BYTE_SIZE)];
    Addr ADDR_SUBARRYS_PER_PIMCTRL_NUM_ARRAY
                  [1U<<pim::LOG_SUBARRAYS_PER_PIMCTRL];

    const Addr ADDR_MAT_MASK ;
    const Addr ADDR_CHIP_MASK;
    const Addr ADDR_ROW_MASK ;
    const Addr ADDR_COL_MASK ;
    const Addr ADDR_SUBARRAY_MASK;
    const Addr ADDR_SUBARRYS_PER_PIMCTRL_MASK;
    const Addr ADDR_BANK_MASK;
    const Addr PIMRANKOFFSET_MASK;

    Addr getAddrMask(const unsigned* field, const unsigned len){
        unsigned fieldVal = 0;
        unsigned bitVal;
        for (int i = 0 ; i < len ; i++){
            bitVal = 1U<<field[i];
            fieldVal |= bitVal; // adding the bit to the total result
        }
        return fieldVal;
    }

    // functions to generate constants to for the PIM addressing
    void generateConstFieldAddr(Addr* mask_arr, const unsigned* idx,
                        const unsigned idx_len){
        // the index length must be the log2 of the output array length
        int arr_len = 1U<<idx_len;
        // for each number, go over all the relevant bits of it and set
        // them in the right location.
        for (int num = 0 ; num < arr_len ; num ++) {
            mask_arr[num] = 0;
            for (int i = 0 ; i < idx_len ; i++) {
                mask_arr[num] |= ((num>>i) & 1U)<<idx[i];
            }
        }
    }

    /**
     * Actually do the DRAM access - figure out the latency it
     * will take to service the req based on bank state, channel state etc
     * and then update those states to account for this request.\ Based
     * on this, update the packet's "readyTime" and move it to the
     * response q from where it will eventually go back to the outside
     * world.
     *
     * @param pkt The DRAM packet created from the outside world pkt
     */
    void doDRAMAccess(PIMPacket* dram_pkt);

    // this function defines the number of cycles each PIM op takes.
    // documentation on the actual computation can be found in the PIM
    // project documents.
    //unsigned pimOpCycles(const uint64_t* pimOpData);

    /**
     * When a packet reaches its "readyTime" in the response Q,
     * use the "access()" method in AbstractMemory to actually
     * create the response packet, and send it back to the outside
     * world requestor.
     *
     * @param pkt The packet from the outside world
     * @param static_latency Static latency to add before sending the packet
     */
    void accessAndRespond(PacketPtr pkt, Tick static_latency);

    /**
     * Address decoder to figure out physical mapping onto ranks,
     * banks, and rows. This function is called multiple times on the same
     * system packet if the pakcet is larger than burst of the memory. The
     * dramPktAddr is used for the offset within the packet.
     *
     * @param pkt The packet from the outside world
     * @param dramPktAddr The starting address of the DRAM packet
     * @param size The size of the DRAM packet in bytes
     * @param isRead Is the request for a read or a write to DRAM
     * @return A PIMPacket pointer with the decoded information
     */
    PIMPacket* decodeAddr(const PacketPtr pkt, Addr dramPktAddr,
                           unsigned int size, bool isRead,
                           bool isOp) const;

    /**
     * The memory schduler/arbiter - picks which request needs to
     * go next, based on the specified policy such as FCFS or FR-FCFS
     * and moves it to the head of the queue.
     * Prioritizes accesses to the same rank as previous burst unless
     * controller is switching command type.
     *
     * @param queue Queued requests to consider
     * @param extra_col_delay Any extra delay due to a read/write switch
     * @return an iterator to the selected packet, else queue.end()
     */
    std::pair<PIMMediaCtrl::PIMPacketQueue::iterator, Tick>
    chooseNext(PIMPacketQueue& queue,
        Tick extra_col_delay);

    /**
     * For FR-FCFS policy reorder the read/write queue depending on row buffer
     * hits and earliest bursts available in DRAM
     *
     * @param queue Queued requests to consider
     * @param extra_col_delay Any extra delay due to a read/write switch
     * @return an iterator to the selected packet, else queue.end()
     */
    std::pair<PIMMediaCtrl::PIMPacketQueue::iterator, Tick>
    chooseNextFRFCFS(PIMPacketQueue& queue,
            Tick extra_col_delay);

    /**
     * Used for debugging to observe the contents of the queues.
     */
    void printQs() const;

    /**
     * Burst-align an address.
     *
     * @param addr The potentially unaligned address
     *
     * @return An address aligned to a DRAM burst
     */
    Addr burstAlign(Addr addr) const
        { return (addr & ~(Addr(burstSize - 1))); }

    /**
     * The controller's main read and write queues,
     * with support for QoS reordering
     */
    std::vector<PIMPacketQueue> readQueue;
    std::vector<PIMPacketQueue> writeQueue;

    /**
     * To avoid iterating over the write queue to check for
     * overlapping transactions, maintain a set of burst addresses
     * that are currently queued. In DRAM we merge writes to the same
     * location, so we never had more than one address to the same burst
     * address. However, in the PIM, since write merges cannot be done
     * between writes with PIM op between them, writes with the same
     * burst addresses are possible.
     */
    //std::unordered_set<Addr> isInWriteQueue;
    std::unordered_multiset<Addr> isInWriteQueue;

    // do the same thing to PIM ops
    std::unordered_multiset<Addr> isPIMOpInWriteQueue;

    // and exactly the same for the read queue since we
    // want to track dependencies there as well.
    std::unordered_multiset<Addr> isInReadQueue;
    std::unordered_multiset<Addr> is2MPageInReadQueue;

    /**
     * Response queue where read packets wait after we're done working
     * with them, but it's not time to send the response yet. The
     * responses are stored separately mostly to keep the code clean
     * and help with events scheduling. For all logical purposes such
     * as sizing the read queue, this and the main read queue need to
     * be added together.
     */
    std::deque<PIMPacket*> respQueue;

    /**
     * Instead of Vector of ranks we have a single rank,
     * so the code will be similar to the DRAMCtrl and so
     * we can use the rank class.
     */
    //std::vector<Rank*> ranks;
    Rank* rank;

    // BenP: Add so we know what rank we are
    uint32_t rankID;
    /**
     * The following are basic design parameters of the memory
     * controller, and are initialized based on parameter values.
     * The rowsPerBank is determined based on the capacity, number of
     * ranks and banks, the burst size, and the row buffer size.
     */
    //const uint32_t deviceSize;
    const uint32_t deviceBusWidth;
    const uint32_t burstLength;
    const uint32_t devicesPerRank = pim::CHIPS_PER_RANK;
    const uint32_t burstSize;
    const uint32_t bankGroupsPerRank;
    const bool bankGroupArch;
    const uint32_t banksPerRank = pim::BANKS_PER_RANK;
    uint32_t rowsPerBank;
    const uint32_t readBufferSize;
    const uint32_t writeBufferSize;
    const uint32_t writeHighThreshold;
    const uint32_t writeLowThreshold;
    const uint32_t minWritesPerSwitch;
    uint32_t opsThisTime;
    uint32_t writesThisTime;
    uint32_t readsThisTime;

    /**
    * Basic memory timing parameters initialized based on parameter
    * values.
    */

    // timing parameters for the PIM
    const Tick tDEC;
    const Tick tCHARGE;
    const Tick tREAD;
    const Tick tPRE;
    const Tick tLOGIC;
    const Tick tSET;
    const Tick tRESET;
    //

    const Tick M5_CLASS_VAR_USED tCK;
    const Tick tRTW;
    const Tick tBURST;
    const Tick tCCD_L_WR;
    const Tick tCCD_L;
    const Tick tRCD;
    const Tick tCL;
    const Tick tRP;
    const Tick tRRD;
    const Tick tRRD_L;
    const Tick tXAW;
    const uint32_t activationLimit;
    const Tick wrToRdDly;
    const Tick rdToWrDly;

    // the threshold time that we are willing to select a packet to issued.
    // Once a packet is selected the code is so that the controller will
    // fix on a selected packet, no matter if other packets arrive that
    // can be scheduled before that.
    Tick thresholdScheduleCheckTime;

    /**
     * Max column accesses (read and write) per row, before forcefully
     * closing it.
     */
    const uint32_t maxAccessesPerRow;

    /**
     * Pipeline latency of the controller frontend. The frontend
     * contribution is added to writes (that complete when they are in
     * the write buffer) and reads that are serviced the write buffer.
     */
    const Tick frontendLatency;

    /**
     * Pipeline latency of the backend and PHY. Along with the
     * frontend contribution, this latency is added to reads serviced
     * by the DRAM.
     */
    const Tick backendLatency;

    /**
     * Till when must we wait before issuing next RD/WR burst?
     */
    Tick nextBurstAt;

    Tick prevArrival;

    /**
     * The soonest you have to start thinking about the next request
     * is the longest access time that can occur before
     * nextBurstAt. Assuming you need to precharge, open a new row,
     * and access, it is tRP + tRCD + tCL.
     */
    Tick nextReqTime;

    Tick pimChannelByteLatency;

    // All statistics that the model needs to capture
    Stats::Scalar readReqs;
    Stats::Scalar writeReqs;
    Stats::Scalar numOpReqs;
    Stats::Vector numTypeOpReqs;
    Stats::Scalar readBursts;
    Stats::Scalar writeBursts;
    Stats::Scalar opBursts;
    Stats::Scalar bytesReadDRAM;
    Stats::Scalar bytesReadWrQ;
    Stats::Scalar bytesWritten;
    Stats::Scalar bytesReadSys;
    Stats::Scalar bytesWrittenSys;
    Stats::Scalar servicedByWrQ;
    Stats::Scalar readDependOnPIM;
    Stats::Scalar mergedWrBursts;
    Stats::Scalar neitherReadNorWrite;
    Stats::Vector perBankRdBursts;
    Stats::Vector perBankWrBursts;
    Stats::Vector perBankOpBursts;
    Stats::Scalar numRdRetry;
    Stats::Scalar numWrRetry;
    Stats::Scalar totGap;
    Stats::Vector readPktSize;
    Stats::Vector writePktSize;
    Stats::Vector rdQLenPdf;
    Stats::Vector wrQLenPdf;
    Stats::Histogram rdPerTurnAround;
    Stats::Histogram wrPerTurnAround;

    // per-master bytes read and written to memory
    Stats::Vector masterReadBytes;
    Stats::Vector masterWriteBytes;

    // per-master bytes read and written to memory rate
    Stats::Formula masterReadRate;
    Stats::Formula masterWriteRate;

    // per-master read and write serviced memory accesses
    Stats::Vector masterReadAccesses;
    Stats::Vector masterWriteAccesses;
    Stats::Vector masterOpAccesses;

    // per-master read and write total memory access latency
    Stats::Vector masterReadTotalLat;
    Stats::Vector masterWriteTotalLat;
    Stats::Vector masterOpTotalLat;

    // per-master read and write average memory access latency
    Stats::Formula masterReadAvgLat;
    Stats::Formula masterWriteAvgLat;
    Stats::Formula masterOpAvgLat;

    // Latencies summed over all requests
    Stats::Scalar totQLat;
    Stats::Scalar totMemAccLat;
    Stats::Scalar totBusLat;

    // Average latencies per request
    Stats::Formula avgQLat;
    Stats::Formula avgBusLat;
    Stats::Formula avgMemAccLat;

    // Average bandwidth
    Stats::Formula avgRdBW;
    Stats::Formula avgWrBW;
    Stats::Formula avgRdBWSys;
    Stats::Formula avgWrBWSys;
    Stats::Formula peakBW;
    Stats::Formula busUtil;
    Stats::Formula busUtilRead;
    Stats::Formula busUtilWrite;

    // Average queue lengths
    Stats::Average avgRdQLen;
    Stats::Average avgWrQLen;

    // Row hit count and rate
    Stats::Scalar readRowHits;
    Stats::Scalar writeRowHits;
    Stats::Scalar opRowHits;
    Stats::Formula readRowHitRate;
    Stats::Formula writeRowHitRate;
    Stats::Formula opRowHitRate;
    Stats::Formula avgGap;

    // DRAM Power Calculation
    Stats::Formula pageHitRate;

    // timestamp offset
    uint64_t timeStampOffset;

    /** The time when stats were last reset used to calculate average power */
    Tick lastStatsResetTick;

    /**
     * Upstream caches need this packet until true is returned, so
     * hold it for deletion until a subsequent call
     */
    std::unique_ptr<Packet> pendingDelete;

    /**
     * This function increments the energy when called. If stats are
     * dumped periodically, note accumulated energy values will
     * appear in the stats (even if the stats are reset). This is a
     * result of the energy values coming from DRAMPower, and there
     * is currently no support for resetting the state.
     *
     * @param rank Current rank
     */
    void updatePowerStats(Rank& rank_ref);

    /**
     * Function for sorting Command structures based on timeStamp
     *
     * @param a Memory Command
     * @param next Memory Command
     * @return true if timeStamp of Command 1 < timeStamp of Command 2
     */
    static bool sortTime(const Command& cmd, const Command& cmd_next) {
        return cmd.timeStamp < cmd_next.timeStamp;
    };

  public:

    void regStats() override;
    PIMMediaCtrl(const PIMMediaCtrlParams* p);
    Port &getPort(const std::string &if_name,
                  PortID idx=InvalidPortID) override;
    virtual void init() override;
    virtual void startup() override;
  protected:
    Tick recvAtomic(PacketPtr pkt);
    void recvFunctional(PacketPtr pkt);
    bool recvTimingReq(PacketPtr pkt);
};

#endif //__MEM_PIM_CTRL_HH__
