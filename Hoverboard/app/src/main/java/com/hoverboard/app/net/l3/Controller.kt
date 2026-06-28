package com.hoverboard.app.net.l3

import com.hoverboard.app.net.store.Key
import com.hoverboard.app.net.store.Value

/** Shared L3 walk wire constants, a mirror of `crates/net/src/walk.rs`'s consts. */
object Walk {
    /** `NODE_HELLO` request kind: a transient controller making first contact (grant a guest address). */
    const val KIND_CONTROLLER = 0x01

    /** `NODE_HELLO` request kind: a board probing a neighbour on the controller's behalf (no grant). */
    const val KIND_PROBE = 0x02

    /** `PORTS` neighbour state: nothing wired to this port. */
    const val NB_EMPTY = 0

    /** `PORTS` neighbour state: a board with no address yet (`node_id == 0x00`). */
    const val NB_UNASSIGNED = 1

    /** `PORTS` neighbour state: a board (or guest) that already has an address. */
    const val NB_ASSIGNED = 2

    /** `PORTS` port medium tag: a UART. */
    const val PORT_UART = 0

    /** `PORTS` port medium tag: the BLE link. */
    const val PORT_BLE = 1

    /** `PORTS` port medium tag: an SWD mailbox. */
    const val PORT_SWD = 2

    /** `ASSIGN` `egress_port` meaning "the addressed board itself" (assign directly, not via a relay). */
    const val EGRESS_SELF = 0xFF

    /** `ASSIGN_ACK` / `CONFIG_RESP` status: success. */
    const val STATUS_OK = 0

    /** `ASSIGN_ACK` / `CONFIG_RESP` status: failure (e.g. a flash error). */
    const val STATUS_ERR = 1

    /** The L3 protocol version a `NODE_HELLO` reply reports. */
    const val PROTO_VER = 1

    /** `CONFIG_RESP` status: success. */
    const val CFG_OK = 0

    /** `CONFIG_RESP` status: a malformed request. */
    const val CFG_BAD = 1

    /** `CONFIG_RESP` status: no field declares this `field_id`. */
    const val CFG_UNKNOWN_KEY = 2

    /** `CONFIG_RESP` status: the value's type did not match the field's registered type. */
    const val CFG_TYPE_MISMATCH = 3

    /** `CONFIG_RESP` status: the store write failed. */
    const val CFG_STORE_ERR = 4
}

/** A parsed `CONFIG_RESP` payload: `[field_id, index, status, type_tag, value...]`. */
data class ConfigResp(
    val fieldId: Int,
    val index: Int,
    val status: Int,
    val typeTag: Int,
    val value: ByteArray,
) {
    /** Decode the value bytes per [typeTag], or null on a bad tag / malformed bytes. */
    fun decodeValue(): Value? =
        com.hoverboard.app.net.store.Type.fromTag(typeTag)?.let { Value.decode(it, value) }

    override fun equals(other: Any?): Boolean =
        other is ConfigResp && fieldId == other.fieldId && index == other.index &&
            status == other.status && typeTag == other.typeTag && value.contentEquals(other.value)

    override fun hashCode(): Int =
        (((fieldId * 31 + index) * 31 + status) * 31 + typeTag) * 31 + value.contentHashCode()

    companion object {
        /** Parse a `CONFIG_RESP` PDU's payload. Returns null if shorter than the 3-byte fixed head. */
        fun parse(pdu: Pdu): ConfigResp? {
            val p = pdu.payload
            if (p.size < 3) return null
            return ConfigResp(
                fieldId = p[0].toInt() and 0xFF,
                index = p[1].toInt() and 0xFF,
                status = p[2].toInt() and 0xFF,
                typeTag = if (p.size >= 4) p[3].toInt() and 0xFF else 0,
                value = if (p.size > 4) p.copyOfRange(4, p.size) else ByteArray(0),
            )
        }
    }
}

/**
 * The transient controller (the host/app side), a mirror of `crates/net/src/walk.rs`'s `Controller`.
 * Sequential request/response: it holds one outstanding request, advances on its reply, and works the
 * queue to quiescence. All requests leave on the single attach port; the gateway forwards them onward
 * by `dst`. The board-side responder lives in the firmware (and, for host tests, in a mock).
 */
class Controller {
    private val one = byteArrayOf(Walk.KIND_CONTROLLER.toByte())

    /** One step of the walk the controller still owes. */
    private sealed class Task {
        object Hello : Task()
        data class AssignGateway(val newAddr: Int) : Task()
        data class Probe(val addr: Int) : Task()
        data class AssignNeighbor(val relay: Int, val egress: Int, val newAddr: Int) : Task()
    }

    /** The controller's (guest) address; provisional 0x80, adopted from the gateway's grant. */
    var guestAddr: Int = 0x80
        private set

    private var nextBoard: Int = 0x01

    // (addr, relay, egress) for each board addressed this walk - its positional identity.
    private val assigned = ArrayList<Triple<Int, Int, Int>>()
    private val queue = ArrayDeque<Task>().apply { addLast(Task.Hello) }
    private var outstanding: Task? = null

    /** The board addresses handed out this walk. */
    fun assignedAddrs(): List<Int> = assigned.map { it.first }

    /** The walk is finished: nothing queued and nothing outstanding. */
    fun isComplete(): Boolean = queue.isEmpty() && outstanding == null

    /** The next request to send out the attach port, or null while a reply is outstanding / complete. */
    fun nextRequest(): ByteArray? {
        if (outstanding != null) return null
        if (queue.isEmpty()) return null
        val task = queue.removeFirst()
        val buf = buildRequest(task)
        outstanding = task
        return buf
    }

    /**
     * Reply to a `NODE_HELLO` probe of the controller's own port (a board probing it on the walk's
     * behalf). Returns the reply PDU to send back, or null if `frame` is not such a probe.
     */
    fun replyToProbe(frame: ByteArray): ByteArray? {
        val pdu = Pdu.decodeOrNull(frame) ?: return null
        if (pdu.known() == Opcode.NodeHello && pdu.payload.size == 1) {
            val reply = byteArrayOf(
                guestAddr.toByte(), Walk.PROTO_VER.toByte(), 0, 0, 0, NO_ADDRESS.toByte(),
            )
            return Pdu.of(Opcode.NodeHello, guestAddr, pdu.src, reply).encode()
        }
        return null
    }

    /** Feed a reply to the outstanding request, advancing the walk. */
    fun onReply(frame: ByteArray) {
        val pdu = Pdu.decodeOrNull(frame) ?: return
        val task = outstanding ?: return
        val op = pdu.known()
        val p = pdu.payload
        when {
            task is Task.Hello && op == Opcode.NodeHello && p.size >= 6 -> {
                outstanding = null
                val nodeId = p[0].toInt() and 0xFF
                val yourAddr = p[5].toInt() and 0xFF
                guestAddr = yourAddr // adopt the granted guest address
                if (nodeId == NO_ADDRESS) {
                    enqueue(Task.AssignGateway(alloc()))
                } else {
                    record(nodeId, 0, Walk.EGRESS_SELF)
                    enqueue(Task.Probe(nodeId))
                }
            }

            task is Task.AssignGateway && op == Opcode.AssignAck && p.size >= 2 -> {
                outstanding = null
                val acked = p[0].toInt() and 0xFF
                record(acked, 0, Walk.EGRESS_SELF)
                enqueue(Task.Probe(acked))
            }

            task is Task.AssignNeighbor && op == Opcode.AssignAck && p.size >= 2 -> {
                outstanding = null
                val acked = p[0].toInt() and 0xFF
                record(acked, task.relay, task.egress)
                enqueue(Task.Probe(acked))
            }

            task is Task.Probe && op == Opcode.Ports -> {
                outstanding = null
                onPorts(task.addr, pdu)
            }

            else -> {
                // An unexpected / wrong reply: keep the task outstanding for a retransmit.
            }
        }
    }

    private fun onPorts(probed: Int, pdu: Pdu) {
        val p = pdu.payload
        if (p.isEmpty()) return
        val parent = parentOf(probed)
        val n = p[0].toInt() and 0xFF
        for (i in 0 until n) {
            val base = 1 + i * 4
            if (base + 3 >= p.size) break
            val port = p[base].toInt() and 0xFF
            val state = p[base + 2].toInt() and 0xFF
            val naddr = p[base + 3].toInt() and 0xFF
            when (state) {
                Walk.NB_UNASSIGNED -> {
                    enqueue(Task.AssignNeighbor(relay = probed, egress = port, newAddr = alloc()))
                }

                Walk.NB_ASSIGNED -> {
                    when {
                        isController(naddr) -> Unit // the controller/guest itself, reached back: ignore
                        naddr == parent -> Unit // the upstream link back to the parent: ignore
                        positionOf(naddr) != null -> {
                            if (positionOf(naddr) != Pair(probed, port)) {
                                // Same address at a different position: a collision. Reassign.
                                enqueue(Task.AssignNeighbor(relay = probed, egress = port, newAddr = alloc()))
                            }
                        }

                        else -> {
                            // A board carrying a stale address: adopt as-is and descend.
                            record(naddr, probed, port)
                            enqueue(Task.Probe(naddr))
                        }
                    }
                }

                else -> Unit // empty
            }
        }
    }

    /** The relay through which `addr` was reached (its parent in the walk tree), or null for the gateway. */
    private fun parentOf(addr: Int): Int? =
        assigned.firstOrNull { it.first == addr }?.second?.takeIf { isUnicast(it) }

    private fun buildRequest(task: Task): ByteArray {
        val pdu = when (task) {
            is Task.Hello -> Pdu.of(Opcode.NodeHello, guestAddr, NO_ADDRESS, one)
            is Task.AssignGateway ->
                Pdu.of(Opcode.Assign, guestAddr, NO_ADDRESS, byteArrayOf(Walk.EGRESS_SELF.toByte(), task.newAddr.toByte()))
            is Task.Probe -> Pdu.of(Opcode.ProbePorts, guestAddr, task.addr, ByteArray(0))
            is Task.AssignNeighbor ->
                Pdu.of(Opcode.Assign, guestAddr, task.relay, byteArrayOf(task.egress.toByte(), task.newAddr.toByte()))
        }
        return pdu.encode()
    }

    /** Allocate the next free board address (0x01..=0x7F), skipping any already handed out. */
    private fun alloc(): Int {
        while (true) {
            val a = nextBoard
            nextBoard = (nextBoard + 1) and 0xFF // wrapping_add(1) on a u8
            if (isBoard(a) && positionOf(a) == null) return a
        }
    }

    private fun record(addr: Int, relay: Int, egress: Int) {
        if (positionOf(addr) == null) assigned.add(Triple(addr, relay, egress))
    }

    private fun positionOf(addr: Int): Pair<Int, Int>? =
        assigned.firstOrNull { it.first == addr }?.let { Pair(it.second, it.third) }

    private fun enqueue(task: Task) = queue.addLast(task)

    // --- CONFIG_* request building / response parsing (the wire face of the store) ---

    /** Build a `CONFIG_WRITE(dst, key, value)` PDU: `[field_id, index, type_tag, value...]`. */
    fun buildConfigWrite(dst: Int, key: Key, value: Value): ByteArray {
        val vb = value.encode()
        val payload = ByteArray(3 + vb.size)
        payload[0] = key.fieldId.toByte()
        payload[1] = key.index.toByte()
        payload[2] = value.kind().tag.toByte()
        System.arraycopy(vb, 0, payload, 3, vb.size)
        return Pdu.of(Opcode.ConfigWrite, guestAddr, dst, payload).encode()
    }

    /** Build a `CONFIG_READ(dst, key)` PDU: `[field_id, index]`. */
    fun buildConfigRead(dst: Int, key: Key): ByteArray =
        Pdu.of(Opcode.ConfigRead, guestAddr, dst, byteArrayOf(key.fieldId.toByte(), key.index.toByte())).encode()
}
