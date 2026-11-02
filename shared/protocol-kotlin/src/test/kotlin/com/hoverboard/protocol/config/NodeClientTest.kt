package com.hoverboard.protocol.config

import com.hoverboard.protocol.board.BoardStatus
import com.hoverboard.protocol.l3.Identity
import com.hoverboard.protocol.l3.Opcode
import com.hoverboard.protocol.l3.Walk
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Value
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.async
import kotlinx.coroutines.test.advanceTimeBy
import kotlinx.coroutines.test.runTest
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test

/**
 * [NodeClient] (`specs/machine-settings.md` section 6.1): `IDENTIFY` and `STATUS_READ` to an
 * addressed node, each answered by its own reply from that node, on the session's one request slot.
 */
@OptIn(ExperimentalCoroutinesApi::class)
class NodeClientTest {

    private val master = 0x01
    private val slave = 0x02
    private val uid = ByteArray(Identity.UID_LEN) { (0x84 + it).toByte() }

    @Test
    fun identifyAnswersTheNodesOwnIdentity() = runTest {
        val f = FakeConfigBoards(this)
        val identity = Identity(Walk.CLASS_MOTOR, Walk.MCU_F1X0, 0x0102, uid)
        f.identities[slave] = identity.encode()

        assertEquals(Identified(identity), f.node.identify(slave))
        assertEquals(listOf(Opcode.Identify.value), f.requests.map { it.opcode })
        assertEquals(slave, f.requests.single().dst)
    }

    @Test
    fun anIdentityOfTheWrongLengthIsUnreadable() = runTest {
        val f = FakeConfigBoards(this)
        assertEquals(Unreadable(ByteArray(3)), f.node.identify(master))
    }

    @Test
    fun theStatusIsTheBootRecordsEntriesAndEmptyFromABridge() = runTest {
        val f = FakeConfigBoards(this)
        f.statuses[slave] = byteArrayOf(Fields.LINK_TX.id.toByte(), 1, BoardStatus.Reason.NOT_UP.code.toByte(), 0x03)

        assertEquals(
            StatusEntries(listOf(BoardStatus.Entry(Fields.LINK_TX.id, 1, BoardStatus.Reason.NOT_UP.code, 0x03))),
            f.node.status(slave),
        )
        assertEquals(StatusEntries(emptyList()), f.node.status(master))
    }

    @Test
    fun aSilentNodeTimesOut() = runTest {
        val f = FakeConfigBoards(this)
        f.silent = true
        assertEquals(TimedOut, f.node.identify(slave))
        assertEquals(TimedOut, f.node.status(slave))
    }

    /** The node lane shares the slot with the config lane: one request in flight between them. */
    @Test
    fun aNodeRequestWhileAConfigOperationIsInFlightIsBusy() = runTest {
        val f = FakeConfigBoards(this)
        f.hold = true
        val first = async { f.client.write(Fields.MOTOR_CURRENT_LIMIT.key(), Value.U32(5), slave) }
        advanceTimeBy(50)
        assertEquals(Busy, f.node.identify(slave))
        assertEquals(Busy, f.node.status(slave))
        f.hold = false
        assertEquals(WriteVerified(Value.U32(5)), first.await())
    }
}
