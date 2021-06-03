extern "C"
{
#include "ti/drivers/rf/RF.h"
#include <ti_radio_config.h>
#include DeviceFamily_constructPath(driverlib / rf_prop_mailbox.h)
#include DeviceFamily_constructPath(driverlib / rf_data_entry.h)
}
#include <cinttypes>

/* -------------------------------------------------------------------------- */
/*                                  Interface                                 */
/* -------------------------------------------------------------------------- */

namespace radio
{

    // A convienient structure for passing buffers around
    struct Bytes;

    // Initialize radio interface
    auto init() -> void;

    // Receive a single packet (timeout=0 means no timeout) (returns empty buffer on failure)
    auto receive(uint32_t timeout_ms) -> Bytes;

    // Transmit a single packet over the radio
    auto transmit(const Bytes bytes) -> void;

    // Checks whether someone else is transmitting
    auto is_channel_busy() -> bool;

}

/* -------------------------------------------------------------------------- */
/*                               Implementation                               */
/* -------------------------------------------------------------------------- */

namespace radio
{

    struct Bytes
    {
        uint8_t *buffer;
        uint32_t length;
    };

    namespace internals
    {
        constexpr auto max_packet_length = 252;

        struct QueueEntry
        {
            rfc_dataEntry_t header;
            uint8_t data[max_packet_length];
        };

        constexpr QueueEntry make_entry(uint8_t *next)
        {
            return {
                {
                    next,
                    DATA_ENTRY_PENDING,
                    {DATA_ENTRY_TYPE_GEN, 0},
                    max_packet_length,
                },
                {},
            };
        }

        static RF_Object rfObject;
        static RF_Handle rfHandle;
        static QueueEntry queue_buffer[2] = {
            make_entry(reinterpret_cast<uint8_t *>(&queue_buffer[1])),
            make_entry(reinterpret_cast<uint8_t *>(&queue_buffer[0])),
        };
        static dataQueue_t data_queue = {
            reinterpret_cast<uint8_t *>(&queue_buffer[0].header),
            NULL,
        };
        static QueueEntry *current_entry = &queue_buffer[0];

        auto get_data() -> Bytes
        {
            Bytes bytes = {
                current_entry->data,
                (uint32_t)current_entry->header.length};
            current_entry->header.status = DATA_ENTRY_PENDING;
            current_entry =
                reinterpret_cast<QueueEntry *>(current_entry->header.pNextEntry);
            return bytes;
        }
    }

    auto init() -> void
    {
        // Initialize radio
        internals::rfHandle = RF_open(
            &internals::rfObject,
            &RF_prop,
            (RF_RadioSetup *)&RF_cmdPropRadioDivSetup,
            NULL);

        // Configure frequency synthesizer
        RF_postCmd(
            internals::rfHandle,
            (RF_Op *)&RF_cmdFs,
            RF_PriorityNormal,
            NULL,
            0);

        // Configure common command settings
        RF_cmdPropRx.pQueue = &internals::data_queue;
        RF_cmdPropRx.rxConf.bAutoFlushIgnored = 1;
        RF_cmdPropRx.rxConf.bAutoFlushCrcErr = 1;
        RF_cmdPropRx.maxPktLen = internals::max_packet_length;
        RF_cmdPropRx.endTrigger.triggerType = TRIG_REL_START;
        RF_cmdPropRx.startTrigger.triggerType = TRIG_NOW;
        RF_cmdPropTx.startTrigger.triggerType = TRIG_NOW;
        RF_cmdPropCs.csConf.busyOp = 1; // End carrier sense on channel busy
        RF_cmdPropCs.csConf.bEnaRssi = 1;
        RF_cmdPropCs.condition.rule = COND_NEVER;
        RF_cmdPropCs.csEndTrigger.triggerType = TRIG_REL_START;
        RF_cmdPropCs.rssiThr = -90;
        RF_cmdPropCs.csEndTime = (50000 + 150) * 4;
    }

    auto is_channel_busy() -> bool
    {
        RF_runCmd(
            internals::rfHandle,
            (RF_Op *)&RF_cmdPropCs,
            RF_PriorityNormal,
            NULL,
            0);
        const auto status = ((volatile RF_Op *)&RF_cmdPropCs)->status;
        return status == PROP_DONE_BUSY;
    }

    auto receive(uint32_t timeout_ms) -> Bytes
    {
        // Setup timeout
        const auto no_timeout = (timeout_ms == 0);
        if (no_timeout)
        {
            RF_cmdPropRx.endTrigger.triggerType = TRIG_NEVER;
        }
        else
        {
            RF_cmdPropRx.endTrigger.triggerType = TRIG_REL_START;
            RF_cmdPropRx.endTime = timeout_ms * 4000; // One ms has 4000 rat ticks
        }

        // Run the command
        RF_runCmd(
            internals::rfHandle,
            (RF_Op *)&RF_cmdPropRx,
            RF_PriorityNormal,
            NULL,
            0);

        // Check status
        const auto status = ((volatile RF_Op *)&RF_cmdPropRx)->status;
        if (status != PROP_DONE_OK)
            return {NULL, 0};

        // Return data from the queue
        return internals::get_data();
    }

    auto transmit(const Bytes bytes) -> void
    {
        // Setup the command
        RF_cmdPropTx.pktLen = bytes.length;
        RF_cmdPropTx.pPkt = bytes.buffer;

        // Execute the command
        RF_runCmd(
            internals::rfHandle,
            (RF_Op *)&RF_cmdPropTx,
            RF_PriorityNormal,
            NULL,
            0);
    }
}
