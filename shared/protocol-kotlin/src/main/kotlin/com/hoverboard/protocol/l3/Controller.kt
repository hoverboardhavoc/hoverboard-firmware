package com.hoverboard.protocol.l3

import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Value

/** Shared L3 walk wire constants, a mirror of `shared/net/src/walk.rs`, the wire vocabulary. */
object Walk {
    /** `NODE_HELLO` request kind: a transient controller making first contact (grant a guest address). */
    const val KIND_CONTROLLER = 0x01

    /** `NODE_HELLO` request kind: a board probing a neighbour on the controller's behalf (no grant). */
    const val KIND_PROBE = 0x02

    // The guest range is NOT here. It is the address space's own fact and lives in Pdu.kt as
    // [GUEST_FIRST] / [GUEST_LAST], beside [BROADCAST], [NO_ADDRESS] and [isController], mirroring
    // `shared/net/src/pdu.rs`, the address model. This object mirrors `shared/net/src/walk.rs`, which is a CONSUMER of
    // the range: it decides which guest address is granted next, not what the range is.

    /** `PORTS` neighbour state: nothing wired to this port. */
    const val NB_EMPTY = 0

    /** `PORTS` neighbour state: a board with no address yet (`node_id == 0x00`). */
    const val NB_UNASSIGNED = 1

    /** `PORTS` neighbour state: a board (or guest) that already has an address. */
    const val NB_ASSIGNED = 2

    /**
     * The bits of a `PORTS` `neighbour_state` byte that carry the state. Bit 7 is reserved for
     * "member of the node's link group" (`specs/l3.md`, "Tier-1 wire encoding"); a board does not
     * set it yet, and every decoder masks with this so a reply that does still classifies the port.
     */
    const val NB_STATE_MASK = 0x03

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

    /** `NODE_HELLO` reply `mcu`: a GD32F10x board. The chip family, never the node's class. */
    const val MCU_F10X = 1

    /** `NODE_HELLO` reply `mcu`: a GD32F1x0 board. */
    const val MCU_F1X0 = 2

    /** `NODE_HELLO` reply `mcu`: the hoverboard bridge. */
    const val MCU_ESP32C3 = 3

    /**
     * `node.class` of a bridge (the hoverboard bridge). The same value [Fields.NodeClass.BRIDGE]
     * carries, which is pinned to the registry's doc; the Rust declares it beside the responder.
     */
    const val CLASS_BRIDGE = com.hoverboard.protocol.store.Fields.NodeClass.BRIDGE

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

    /**
     * `CONFIG_RESP` status: the write was rejected because the board is armed (some motor's MOE is
     * allowed; `specs/integration.md`, R4). Reads are unaffected; retry after disarm.
     */
    const val CFG_ARMED = 5

    /**
     * `CONFIG_RESP` status: the field is read-only (`node.class`, [com.hoverboard.protocol.store.Fields.NODE_CLASS]).
     * The board refuses the write and stores nothing.
     */
    const val CFG_READ_ONLY = 6

    /**
     * `CONFIG_RESP` status: the region is genuinely out of space. The board compacted and the
     * value still does not fit, so the live set alone fills a clean page. Nothing is wrong with
     * the flash and retrying will not help; something has to go.
     */
    const val CFG_NO_SPACE = 7

    /**
     * `CONFIG_RESP` status: the board compacted but could not COMMIT the packed page, so it could
     * not reclaim the space. No configuration was lost, but nothing more can be written until the
     * cause is found.
     *
     * It is its own byte because the two states above used to be one: a board whose flash refused
     * the commit answered the same [CFG_STORE_ERR] a board with a genuinely full region would, so
     * a firmware defect looked like a board that was simply out of room.
     */
    const val CFG_COMMIT_FAILED = 8

    /**
     * `CONFIG_RESP` status: a successful `CONFIG_READ` of a key the board does not store
     * (`specs/cli-config.md` section 6.1). The value that follows is the field's default; a key with
     * a record answers [CFG_OK]. Not a refusal: a client reads the value either way.
     */
    const val CFG_DEFAULT = 9

    /**
     * `CONFIG_RESP` status: a `CONFIG_READ` of a stored secret (`store::SECRET`, `wifi.pass`;
     * `specs/bridge-onboarding.md` section 2.2). The key holds a value and none follows: no node
     * reads a secret back over a link. The app reads no secret, so it meets this only as a refusal.
     */
    const val CFG_SECRET = 10

    /**
     * `TUNE_WRITE`: set one live-tunable value in the board's RAM, replying `CONFIG_RESP`
     * (`specs/rider-ui.md` section 4). The payload is `CONFIG_WRITE`'s
     * `[field_id, index, type_tag, value...]`.
     *
     * It is a CONST here and not an [Opcode] entry for the same reason it is one in the Rust: L3
     * declares the opcode but does not serve it. The values live in the board's control loop, not
     * in its store, so the walk layer hands the PDU to the firmware. That is what makes the lane
     * exempt from [CFG_ARMED]: it writes no flash, so the armed refusal never applies to it, and a
     * tune write is answered while armed. Persisting a tuned value is a separate, ordinary,
     * DISARMED `CONFIG_WRITE` of the same key, which is refused while armed like any other.
     */
    const val OP_TUNE_WRITE = 0x34

    /** `TUNE_READ`: read one live value back out of RAM, replying `CONFIG_RESP`. Payload is
     * `CONFIG_READ`'s `[field_id, index]`. See [OP_TUNE_WRITE]. */
    const val OP_TUNE_READ = 0x35
}

/** The `CONFIG_RESP` bytes before the value: `field_id`, `index`, `status`, `type_tag`. */
private const val CONFIG_RESP_HEAD = 4

/**
 * The longest value one `CONFIG_WRITE` can carry and have verified: 57 bytes, of a `STR` or `BLOB`
 * value (every scalar is far shorter).
 *
 * The request is `HEADER_LEN` + `[field_id, index, type_tag]` + the value, so it fits [MAX_PDU] up
 * to `64 - 3 - 3 = 58` bytes. The verifying echo is the tighter bound: `CONFIG_RESP` is
 * `HEADER_LEN` + `[field_id, index, status, type_tag]` + the stored value, so `64 - 3 - 4 = 57`.
 * A 58-byte value is stored, but its echo would be a 65-byte PDU, which the firmware's `emit`
 * in `shared/net/src/walk.rs` cannot encode into its `MAX_PDU` buffer, so NO reply is sent and the
 * client times out. A later `CONFIG_READ` of that key is unanswerable for the same reason, until the
 * value is rewritten at 57 bytes or fewer.
 */
const val CONFIG_VALUE_MAX = MAX_PDU - HEADER_LEN - CONFIG_RESP_HEAD

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
        com.hoverboard.protocol.store.Type.fromTag(typeTag)?.let { Value.decode(it, value) }

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
 * The transient controller (the host/app side), mirroring the WALK LOOP of `shared/net/src/walk.rs`'s
 * `Controller`. Sequential request/response: it holds one outstanding request, advances on its reply,
 * and works the queue to quiescence. All requests leave on the single attach port; the gateway
 * forwards them onward by `dst`. The board-side responder lives in the firmware (and, for host tests,
 * in a mock).
 *
 * Not a mirror of all of it, and the gap is named rather than implied: the Rust `Controller` also
 * derives the attached node from the walk (`shared/net/src/walk.rs`, `HostLink`, recorded in
 * `on_ports` from the port table's `kind` byte). Nothing here reads that byte, and [BleWalk] still
 * takes the entry board as the first board it recorded, which is the gateway half alone. A walk whose
 * gateway is not the attached node is described correctly by the Rust and not by this file.
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

    /** The controller's (guest) address; provisional [GUEST_FIRST], adopted from the gateway's grant. */
    var guestAddr: Int = GUEST_FIRST
        private set

    private var nextBoard: Int = 0x01

    /**
     * One node the walk has placed: its address, its position (the relay it was reached through
     * and that relay's port; `(0, EGRESS_SELF)` for the node on the controller's own link), and
     * whether this controller handed the address out during this walk ([here]) rather than the
     * node answering already holding it. Only a node placed [here] is moved to resolve a collision
     * (mirror of `walk.rs`'s `Placed`).
     */
    private data class Placed(var addr: Int, var relay: Int, val egress: Int, val here: Boolean)

    private val assigned = ArrayList<Placed>()
    private val queue = ArrayDeque<Task>().apply { addLast(Task.Hello) }
    private var outstanding: Task? = null

    /** The board addresses handed out this walk. */
    fun assignedAddrs(): List<Int> = assigned.map { it.addr }

    /**
     * The address of the board this controller attached to: the first board recorded, i.e. the peer
     * on the controller's own link. Null until first contact has produced an address (mirror of
     * `walk.rs`'s `gateway_addr`).
     *
     * It is NOT "address 0x01" and not "the lowest address": a board that persisted `0x02` in a past
     * session answers first contact as `0x02` while a freshly-allocated neighbour gets `0x01`.
     */
    fun gatewayAddr(): Int? = assigned.firstOrNull()?.addr

    /** The walk is finished: nothing queued and nothing outstanding. */
    fun isComplete(): Boolean = queue.isEmpty() && outstanding == null

    /**
     * Walk again from [addrs], after the walk has finished, to find nodes that came up since
     * (`specs/rider-ui.md` 3.1, "Boards that come up after the connect"): each of [addrs] this walk
     * has placed is probed again. A neighbour already known at the port it answers on is passed over
     * ([onPorts]: "already known here"), and a new or unaddressed one is adopted or assigned and
     * descended into as on the first walk. Does nothing while a walk is still running.
     */
    fun rewalk(addrs: List<Int>) {
        if (!isComplete()) return
        for (a in addrs.distinct()) if (positionOf(a) != null) enqueue(Task.Probe(a))
    }

    /**
     * Drop what an abandoned walk left queued and outstanding, so [rewalk] can start: the driver
     * gave up on that request, and its late reply is then no reply ([onReply] answers false).
     */
    fun dropUnfinished() {
        queue.clear()
        outstanding = null
    }

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

    /**
     * Feed a frame that may answer the outstanding request, advancing the walk. Returns whether the
     * frame was that reply and was consumed (mirror of `walk.rs`'s `on_reply`).
     *
     * A reply is bound to the request it answers (`specs/l3.md`, "Discovery and address
     * assignment"): a frame that does not fit the outstanding request ([answers]) is not a reply,
     * the request stays outstanding for its retransmit, and false tells the caller to route the
     * frame on (the telemetry inbox). A duplicate or late `PORTS` read against the next probe is
     * read against the wrong node's position and looks like a collision, which is how the rider
     * renumbered an offroad slave to `0x03` and later the master to `0x05` on 2026-10-25.
     */
    fun onReply(frame: ByteArray): Boolean {
        val pdu = Pdu.decodeOrNull(frame) ?: return false
        val task = outstanding ?: return false
        if (!answers(task, pdu)) return false
        outstanding = null
        val p = pdu.payload
        when (task) {
            is Task.Hello -> {
                val nodeId = p[0].toInt() and 0xFF
                val yourAddr = p[5].toInt() and 0xFF
                guestAddr = yourAddr // adopt the granted guest address
                if (nodeId == NO_ADDRESS) {
                    enqueue(Task.AssignGateway(alloc()))
                } else {
                    record(nodeId, 0, Walk.EGRESS_SELF, here = false)
                    enqueue(Task.Probe(nodeId))
                }
            }

            // A non-OK status (the R4 armed refusal, STATUS_ERR) clears the outstanding task but
            // records nothing and probes nothing: the board did not adopt the address, so acting
            // on it would put a nobody-holds-it address in the map. Mirror of `walk.rs`, the same
            // rule. A reassignment of a node already placed ([reassign]) moved its record when it
            // was decided, so [record] finds it and nothing is probed again.
            is Task.AssignGateway -> {
                if (p[1].toInt() and 0xFF == Walk.STATUS_OK &&
                    record(task.newAddr, 0, Walk.EGRESS_SELF, here = true)
                ) {
                    enqueue(Task.Probe(task.newAddr))
                }
            }

            is Task.AssignNeighbor -> {
                if (p[1].toInt() and 0xFF == Walk.STATUS_OK &&
                    record(task.newAddr, task.relay, task.egress, here = true)
                ) {
                    enqueue(Task.Probe(task.newAddr))
                }
            }

            is Task.Probe -> onPorts(task.addr, pdu)
        }
        return true
    }

    /**
     * Whether [pdu] is the reply to [task], the outstanding request (`specs/l3.md`, "A reply is
     * bound to the request it answers"; mirror of `walk.rs`'s `answers`). Every reply is addressed
     * to this controller's guest address: for the first-contact `NODE_HELLO` that is the address
     * the request was sent from, since the grant is adopted only from the reply that passes this
     * check. A `PORTS` comes from the node probed, and an `ASSIGN_ACK` from the address assigned
     * and names it in its payload.
     */
    private fun answers(task: Task, pdu: Pdu): Boolean {
        if (pdu.dst != guestAddr) return false
        val p = pdu.payload
        val op = pdu.known()
        return when (task) {
            is Task.Hello -> op == Opcode.NodeHello && p.size >= 6
            is Task.Probe -> op == Opcode.Ports && pdu.src == task.addr
            is Task.AssignGateway -> op == Opcode.AssignAck && acks(pdu, task.newAddr)
            is Task.AssignNeighbor -> op == Opcode.AssignAck && acks(pdu, task.newAddr)
        }
    }

    /** An `ASSIGN_ACK` from [newAddr] that names [newAddr]. */
    private fun acks(pdu: Pdu, newAddr: Int): Boolean =
        pdu.payload.size >= 2 && pdu.src == newAddr && (pdu.payload[0].toInt() and 0xFF) == newAddr

    private fun onPorts(probedAt: Int, pdu: Pdu) {
        val p = pdu.payload
        if (p.isEmpty()) return
        val parent = parentOf(probedAt)
        val n = p[0].toInt() and 0xFF
        // (port, state, naddr) per entry, while the payload holds one.
        val entries = (0 until n).map { 1 + it * 4 }.takeWhile { it + 3 < p.size }.map { base ->
            // Bits 0-1 only: bit 7 is the link-group flag, which says nothing about the state.
            Triple(p[base].toInt() and 0xFF, p[base + 2].toInt() and Walk.NB_STATE_MASK, p[base + 3].toInt() and 0xFF)
        }

        // The parent's address on more than one port: the upstream link and a neighbour that
        // already held the address the parent was given this walk. Which port is upstream cannot be
        // told from the report, so the parent moves and this node is probed again once it has.
        if (parent != null && placedHere(parent) &&
            entries.count { (_, state, naddr) -> state == Walk.NB_ASSIGNED && naddr == parent } > 1
        ) {
            reassign(parent)
            enqueue(Task.Probe(probedAt))
            return
        }

        // The probed node's address for this report: it changes if the node itself is moved.
        var probed = probedAt
        for ((port, state, naddr) in entries) {
            when (state) {
                Walk.NB_UNASSIGNED -> {
                    enqueue(Task.AssignNeighbor(relay = probed, egress = port, newAddr = alloc()))
                }

                Walk.NB_ASSIGNED -> {
                    val pos = positionOf(naddr)
                    when {
                        isController(naddr) -> Unit // the controller/guest itself, reached back: ignore
                        naddr == parent -> Unit // the upstream link back to the parent: ignore
                        pos == Pair(probed, port) -> Unit // already known here: a re-walk
                        pos != null && placedHere(naddr) -> {
                            // Two holders of one address: the node this walk assigned it to, and
                            // this neighbour, which held it already. The one this walk assigned
                            // moves; the neighbour keeps its address and is descended into.
                            val moved = reassign(naddr)
                            if (probed == naddr) probed = moved
                            record(naddr, probed, port, here = false)
                            enqueue(Task.Probe(naddr))
                        }

                        pos != null -> {
                            // Two nodes that both held it: the one reached second is reassigned.
                            enqueue(Task.AssignNeighbor(relay = probed, egress = port, newAddr = alloc()))
                        }

                        else -> {
                            // A board carrying a stale address: adopt as-is and descend.
                            record(naddr, probed, port, here = false)
                            enqueue(Task.Probe(naddr))
                        }
                    }
                }

                else -> Unit // empty
            }
        }
    }

    /** Whether `addr` was handed out by this controller during this walk. */
    private fun placedHere(addr: Int): Boolean = assigned.any { it.addr == addr && it.here }

    /**
     * Move the node this walk placed at [old] to a fresh address, returning it (mirror of
     * `walk.rs`'s `reassign`). The map follows at once: the node's record, the records of nodes
     * reached through it, and the queued tasks naming it. Its `ASSIGN` goes to the FRONT of the
     * queue, so nothing addressed to [old] is sent before the node has left it.
     */
    private fun reassign(old: Int): Int {
        val na = alloc()
        var task: Task? = null
        for (pl in assigned) {
            if (pl.addr == old) {
                pl.addr = na
                task = if (pl.egress == Walk.EGRESS_SELF) {
                    Task.AssignGateway(na)
                } else {
                    Task.AssignNeighbor(relay = pl.relay, egress = pl.egress, newAddr = na)
                }
            }
        }
        for (pl in assigned) if (pl.relay == old) pl.relay = na
        for (i in queue.indices) {
            val t = queue[i]
            if (t is Task.Probe && t.addr == old) queue[i] = Task.Probe(na)
            if (t is Task.AssignNeighbor && t.relay == old) queue[i] = t.copy(relay = na)
        }
        task?.let { queue.addFirst(it) }
        return na
    }

    /** The relay through which `addr` was reached (its parent in the walk tree), or null for the gateway. */
    private fun parentOf(addr: Int): Int? =
        assigned.firstOrNull { it.addr == addr }?.relay?.takeIf { isUnicast(it) }

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

    /** Place [addr] at ([relay], [egress]) unless it is already placed; whether it was new. */
    private fun record(addr: Int, relay: Int, egress: Int, here: Boolean): Boolean {
        if (positionOf(addr) != null) return false
        assigned.add(Placed(addr, relay, egress, here))
        return true
    }

    private fun positionOf(addr: Int): Pair<Int, Int>? =
        assigned.firstOrNull { it.addr == addr }?.let { Pair(it.relay, it.egress) }

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

    /**
     * Build a `TUNE_WRITE(dst, key, value)` PDU (`specs/rider-ui.md` section 4): the same payload
     * shape as [buildConfigWrite], under [Walk.OP_TUNE_WRITE], and always an `I16` because the
     * live-tunable fields are.
     *
     * The reply is an ordinary `CONFIG_RESP` and parses with [ConfigResp.parse]: `CFG_OK` echoes
     * the value now live, `CFG_UNKNOWN_KEY` means the key is not live-tunable, `CFG_BAD` means the
     * value is outside the field's range. Never `CFG_ARMED` (see [Walk.OP_TUNE_WRITE]).
     */
    fun buildTuneWrite(dst: Int, key: Key, value: Int): ByteArray {
        val v = Value.I16(value)
        val vb = v.encode()
        val payload = ByteArray(3 + vb.size)
        payload[0] = key.fieldId.toByte()
        payload[1] = key.index.toByte()
        payload[2] = v.kind().tag.toByte()
        System.arraycopy(vb, 0, payload, 3, vb.size)
        return Pdu(Walk.OP_TUNE_WRITE, guestAddr, dst, payload).encode()
    }

    /** Build a `TUNE_READ(dst, key)` PDU: `[field_id, index]`, under [Walk.OP_TUNE_READ]. */
    fun buildTuneRead(dst: Int, key: Key): ByteArray =
        Pdu(Walk.OP_TUNE_READ, guestAddr, dst, byteArrayOf(key.fieldId.toByte(), key.index.toByte())).encode()
}
