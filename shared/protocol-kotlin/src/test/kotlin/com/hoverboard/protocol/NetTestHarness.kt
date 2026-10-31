package com.hoverboard.protocol

import com.hoverboard.protocol.l2.Link
import com.hoverboard.protocol.l2.Transport
import com.hoverboard.protocol.l3.BROADCAST
import com.hoverboard.protocol.l3.Controller
import com.hoverboard.protocol.l3.GUEST_FIRST
import com.hoverboard.protocol.l3.GUEST_LAST
import com.hoverboard.protocol.l3.NO_ADDRESS
import com.hoverboard.protocol.l3.Opcode
import com.hoverboard.protocol.l3.Pdu
import com.hoverboard.protocol.l3.Walk
import com.hoverboard.protocol.l3.isUnicast
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Type
import com.hoverboard.protocol.store.Value

/**
 * Host-test mesh harness: an in-memory network of real [Link]s (over a mock datagram transport)
 * carrying the [Controller] and a [MockBoard] responder. The responder + forwarder mirror the
 * firmware (`shared/net/src/{walk,forward}.rs`); this is the controller-walk round-trip exactly the
 * way `shared/net/src/walk/tests.rs`, the Rust harness, drives it, so the Kotlin controller is
 * cross-checked against the
 * Rust on the same worked topologies.
 */

/** Frame capacity of the mock links (matches the Rust walk test's CAP). */
private const val CAP = 255

/** The `node_address` key (field 0x01, singleton index 0), mirror of `store::NODE_ADDRESS`. */
val NODE_ADDRESS_KEY = Key(0x01, 0)

/** A sentinel "direction unknown" port (mirror of `forward.rs`'s `NO_PORT`). */
private const val NO_PORT = 0xFF

/** The fleet-max local ports (mirror of `walk.rs`'s `MAX_PORTS`). */
private const val MAX_PORTS = 4

/** One thing a board emits: an egress port + an encoded PDU. */
data class Emission(val port: Int, val bytes: ByteArray) {
    override fun equals(other: Any?): Boolean =
        other is Emission && port == other.port && bytes.contentEquals(other.bytes)
    override fun hashCode(): Int = port * 31 + bytes.contentHashCode()
}

private fun emit(out: MutableList<Emission>, port: Int, pdu: Pdu) {
    out.add(Emission(port, pdu.encode()))
}

/** A mock datagram transport: each L2 frame rides one transaction as-is (loopback via crossed queues). */
private class MockPort(private val tx: ArrayDeque<ByteArray>, private val rx: ArrayDeque<ByteArray>) :
    Transport {
    override fun frameCapacity(): Int = CAP
    override fun sendL2Frame(l2: ByteArray) = tx.addLast(l2.copyOf())
    override fun recvL2Frame(): ByteArray? = rx.removeFirstOrNull()
}

/**
 * A tiny registry-checked key/value store (mirror of the firmware store's CONFIG-relevant behavior).
 *
 * [nodeClass] is the node's read-only `node.class` and [name] its `device.name` default: a registry
 * default each, served by a read and never stored, which is how a board answers them unwritten
 * (`shared/store/src/field.rs`, `NODE_CLASS` and `DEVICE_NAME`). A bridge is the same responder
 * with class `0x02` (`specs/hoverboard-bridge.md`, "The bridge as a node").
 */
private class MockStore(nodeClass: Int, name: String) {
    // The fields the walk, the CONFIG round-trip and a controller's node-class read touch.
    private val registry = mapOf(
        0x01 to Type.U8, // NODE_ADDRESS
        0x03 to Type.U8, // NODE_CLASS (read-only)
        0x10 to Type.Str, // DEVICE_NAME
        0x20 to Type.U32, // MOTOR_CURRENT_LIMIT
    )
    private val readOnly = setOf(NODE_CLASS_KEY.fieldId)
    private val defaults = mapOf(NODE_CLASS_KEY to Value.U8(nodeClass), DEVICE_NAME_KEY to Value.Str(name))
    private val map = HashMap<Key, Value>()

    fun getValue(key: Key): Value? = map[key]

    /** What a CONFIG_READ answers: the stored value, else the registry default where there is one. */
    fun readValue(key: Key): Value? = map[key] ?: defaults[key]

    /** Set with registry type-checking; returns a CONFIG status (CFG_OK / UNKNOWN_KEY / TYPE_MISMATCH / READ_ONLY). */
    fun setValue(key: Key, value: Value): Int {
        val regType = registry[key.fieldId] ?: return Walk.CFG_UNKNOWN_KEY
        if (value.kind() != regType) return Walk.CFG_TYPE_MISMATCH
        if (key.fieldId in readOnly) return Walk.CFG_READ_ONLY
        map[key] = value
        return Walk.CFG_OK
    }
}

/** The `node.class` key (field 0x03), mirror of `store::NODE_CLASS`. */
val NODE_CLASS_KEY = Key(0x03, 0)

/** The `device.name` key (field 0x10), mirror of `store::DEVICE_NAME`. */
val DEVICE_NAME_KEY = Key(0x10, 0)

/** Source-learned multi-hop forwarder, a mirror of `shared/net/src/forward.rs`'s `Forwarder`. */
private class Forwarder(var addr: Int, val nPorts: Int) {
    private val table = IntArray(256) { NO_PORT }

    fun portToward(dst: Int): Int? = table[dst].takeIf { it != NO_PORT }
    fun clearRoutes() = table.fill(NO_PORT)

    private fun learn(src: Int, ingress: Int) {
        if (isUnicast(src)) table[src] = ingress
    }

    fun ingest(ingress: Int, pdu: Pdu, deliver: (Pdu) -> Unit, forward: (Int, Pdu) -> Unit) {
        learn(pdu.src, ingress)
        decide(ingress, pdu, deliver, forward)
    }

    fun originate(pdu: Pdu, forward: (Int, Pdu) -> Unit) = decide(null, pdu, {}, forward)

    private fun decide(ingress: Int?, pdu: Pdu, deliver: (Pdu) -> Unit, forward: (Int, Pdu) -> Unit) {
        val dst = pdu.dst
        if (ingress != null && dst == NO_ADDRESS) {
            deliver(pdu); return
        }
        if (dst == addr && addr != NO_ADDRESS) {
            deliver(pdu); return
        }
        if (dst == BROADCAST) {
            if (ingress != null) deliver(pdu)
            flood(ingress, pdu, forward); return
        }
        val p = portToward(dst)
        when {
            p != null && p != ingress -> forward(p, pdu)
            p != null -> Unit // points back at the ingress: drop (split-horizon)
            else -> flood(ingress, pdu, forward)
        }
    }

    private fun flood(ingress: Int?, pdu: Pdu, forward: (Int, Pdu) -> Unit) {
        for (port in 0 until nPorts) if (port != ingress) forward(port, pdu)
    }
}

/** A board's in-flight PROBE_PORTS state. */
private class Probe(val replyTo: Int, val nPorts: Int) {
    val states = Array(MAX_PORTS) { Pair(Walk.NB_EMPTY, 0) } // (neighbour_state, neighbour_addr)
}

/**
 * The board-side walk responder, a mirror of `shared/net/src/walk.rs`'s `Responder` (wrapping the
 * [Forwarder]). It answers NODE_HELLO, probes its ports on PROBE_PORTS and reports PORTS, takes an
 * ASSIGN (persisting node_address) or relays a directed ASSIGN, and serves CONFIG_READ/WRITE. It
 * never reads a hardware id - identity is positional during discovery, the persisted address after.
 */
class MockBoard(
    nPorts: Int,
    private val portKinds: IntArray,
    private val mcu: Int,
    private val fwVer: Int,
    nodeClass: Int = Fields.NodeClass.MOTOR_CONTROLLER,
    name: String = "Hoverboard",
) {
    private val fwd = Forwarder(NO_ADDRESS, nPorts)
    private val store = MockStore(nodeClass, name)
    /** The next grant's offset in this board's guest slice (`shared/net/src/walk.rs`, `guest_slice`). */
    private var guestSlot = 0
    private var probe: Probe? = null

    fun addr(): Int = fwd.addr
    fun probing(): Boolean = probe != null
    fun portToward(dst: Int): Int? = fwd.portToward(dst)

    /** A reboot: clear the (soft) routing table and re-read the persisted node_address. */
    fun reboot() {
        fwd.clearRoutes()
        (store.getValue(NODE_ADDRESS_KEY) as? Value.U8)?.let { fwd.addr = it.v }
    }

    /** Pre-persist a stale node_address and boot there (a past session). */
    fun preassign(addr: Int) {
        store.setValue(NODE_ADDRESS_KEY, Value.U8(addr))
        fwd.addr = addr
    }

    fun persistedAddr(): Int? = (store.getValue(NODE_ADDRESS_KEY) as? Value.U8)?.v

    fun ingest(ingress: Int, frame: ByteArray, out: MutableList<Emission>) {
        val pdu = Pdu.decodeOrNull(frame) ?: return
        var delivered: Pdu? = null
        val forwarded = ArrayList<Emission>()
        fwd.ingest(ingress, pdu, { d -> delivered = d }, { port, f -> emit(forwarded, port, f) })
        out.addAll(forwarded)
        delivered?.let { handleLocal(ingress, it, out) }
    }

    private fun handleLocal(ingress: Int, pdu: Pdu, out: MutableList<Emission>) {
        when (pdu.known()) {
            Opcode.NodeHello -> onHello(ingress, pdu, out)
            Opcode.ProbePorts -> onProbePorts(pdu, out)
            Opcode.Assign -> onAssign(pdu, out)
            Opcode.ConfigRead -> onConfigRead(pdu, out)
            Opcode.ConfigWrite -> onConfigWrite(pdu, out)
            else -> Unit // controller-bound replies to a board are ignored
        }
    }

    private fun onHello(ingress: Int, pdu: Pdu, out: MutableList<Emission>) {
        val payload = pdu.payload
        if (payload.size == 1) {
            val kind = payload[0].toInt() and 0xFF
            val yourAddr = if (kind == Walk.KIND_CONTROLLER) {
                // The 8-address slice at 0x80 + 8 * (addr & 0x0F); the top one stops at GUEST_LAST.
                val first = GUEST_FIRST + 8 * (addr() and 0x0F)
                val len = minOf(8, GUEST_LAST - first + 1)
                val slot = guestSlot % len
                guestSlot = (slot + 1) % len
                first + slot
            } else {
                NO_ADDRESS
            }
            val nodeId = addr()
            val reply = byteArrayOf(
                nodeId.toByte(), Walk.PROTO_VER.toByte(),
                (fwVer and 0xFF).toByte(), ((fwVer ushr 8) and 0xFF).toByte(),
                mcu.toByte(), yourAddr.toByte(),
            )
            emit(out, ingress, Pdu.of(Opcode.NodeHello, nodeId, pdu.src, reply))
        } else {
            val pr = probe ?: return
            if (ingress < MAX_PORTS && payload.isNotEmpty()) {
                val nodeId = payload[0].toInt() and 0xFF
                val state = if (nodeId == NO_ADDRESS) Walk.NB_UNASSIGNED else Walk.NB_ASSIGNED
                pr.states[ingress] = Pair(state, nodeId)
            }
        }
    }

    private fun onProbePorts(pdu: Pdu, out: MutableList<Emission>) {
        val n = fwd.nPorts
        probe = Probe(pdu.src, n)
        val req = byteArrayOf(Walk.KIND_PROBE.toByte())
        for (p in 0 until n) emit(out, p, Pdu.of(Opcode.NodeHello, addr(), NO_ADDRESS, req))
    }

    fun pollProbe(out: MutableList<Emission>) {
        val pr = probe ?: return
        probe = null
        val payload = ArrayList<Byte>()
        payload.add(pr.nPorts.toByte())
        for (p in 0 until pr.nPorts) {
            val (state, a) = pr.states[p]
            payload.add(p.toByte()); payload.add(portKinds[p].toByte())
            payload.add(state.toByte()); payload.add(a.toByte())
        }
        fwd.originate(Pdu.of(Opcode.Ports, addr(), pr.replyTo, payload.toByteArray())) { port, f ->
            emit(out, port, f)
        }
    }

    private fun onAssign(pdu: Pdu, out: MutableList<Emission>) {
        val payload = pdu.payload
        if (payload.size < 2) return
        val egress = payload[0].toInt() and 0xFF
        val newAddr = payload[1].toInt() and 0xFF
        if (egress == Walk.EGRESS_SELF) {
            val cfg = store.setValue(NODE_ADDRESS_KEY, Value.U8(newAddr))
            val status = if (cfg == Walk.CFG_OK) {
                fwd.addr = newAddr; Walk.STATUS_OK
            } else {
                Walk.STATUS_ERR
            }
            val ack = byteArrayOf(newAddr.toByte(), status.toByte())
            fwd.originate(Pdu.of(Opcode.AssignAck, newAddr, pdu.src, ack)) { port, f -> emit(out, port, f) }
        } else {
            // I am the relay: forward out `egress`, dst rewritten to 0x00, src kept = the controller.
            val fwdPayload = byteArrayOf(Walk.EGRESS_SELF.toByte(), newAddr.toByte())
            emit(out, egress, Pdu.of(Opcode.Assign, pdu.src, NO_ADDRESS, fwdPayload))
        }
    }

    private fun onConfigRead(pdu: Pdu, out: MutableList<Emission>) {
        val payload = pdu.payload
        if (payload.size < 2) return
        val key = Key(payload[0].toInt() and 0xFF, payload[1].toInt() and 0xFF)
        val resp = ArrayList<Byte>()
        resp.add(key.fieldId.toByte()); resp.add(key.index.toByte())
        val v = store.readValue(key)
        if (v != null) {
            resp.add(Walk.CFG_OK.toByte()); pushValue(resp, v)
        } else {
            resp.add(Walk.CFG_UNKNOWN_KEY.toByte()); resp.add(0)
        }
        replyConfig(pdu.src, resp.toByteArray(), out)
    }

    private fun onConfigWrite(pdu: Pdu, out: MutableList<Emission>) {
        val payload = pdu.payload
        if (payload.size < 3) return
        val key = Key(payload[0].toInt() and 0xFF, payload[1].toInt() and 0xFF)
        val typeTag = payload[2].toInt() and 0xFF
        val valueBytes = payload.copyOfRange(3, payload.size)
        val status = applyWrite(key, typeTag, valueBytes)
        val resp = ArrayList<Byte>()
        resp.add(key.fieldId.toByte()); resp.add(key.index.toByte()); resp.add(status.toByte())
        if (status == Walk.CFG_OK) {
            store.getValue(key)?.let { pushValue(resp, it) }
        } else {
            resp.add(0)
        }
        replyConfig(pdu.src, resp.toByteArray(), out)
    }

    private fun applyWrite(key: Key, typeTag: Int, bytes: ByteArray): Int {
        val type = Type.fromTag(typeTag) ?: return Walk.CFG_BAD
        val value = Value.decode(type, bytes) ?: return Walk.CFG_BAD
        return store.setValue(key, value)
    }

    private fun pushValue(out: ArrayList<Byte>, v: Value) {
        out.add(v.kind().tag.toByte())
        for (b in v.encode()) out.add(b)
    }

    private fun replyConfig(dst: Int, payload: ByteArray, out: MutableList<Emission>) {
        fwd.originate(Pdu.of(Opcode.ConfigResp, addr(), dst, payload)) { port, f -> emit(out, port, f) }
    }
}

private class BoardNode(val board: MockBoard, val ports: Array<Link?>)
private class CtrlNode(var ctrl: Controller = Controller(), var link: Link? = null) {
    val inbox = ArrayList<ByteArray>()

    /** Every request the controller sent, in order. */
    val sent = ArrayList<ByteArray>()

    /** The frames `onReply` refused, each with the last request sent before it arrived. */
    val refused = ArrayList<Pair<ByteArray, ByteArray?>>()
}

/** An in-memory mesh of boards + a controller over real L2 links (mirror of the Rust walk-test Mesh). */
class Mesh {
    private val boards = ArrayList<BoardNode>()
    private val ctrl = CtrlNode()

    val controller: Controller get() = ctrl.ctrl

    fun addBoard(
        nPorts: Int,
        nodeClass: Int = Fields.NodeClass.MOTOR_CONTROLLER,
        name: String = "Hoverboard",
    ): Int {
        val kinds = IntArray(MAX_PORTS) { Walk.PORT_UART }
        val board = MockBoard(nPorts, kinds, mcu = 0x10, fwVer = 0x0001, nodeClass = nodeClass, name = name)
        boards.add(BoardNode(board, arrayOfNulls(nPorts)))
        return boards.size - 1
    }

    fun preassign(board: Int, addr: Int) = boards[board].board.preassign(addr)

    fun wire(a: Int, pa: Int, b: Int, pb: Int) {
        val w1 = ArrayDeque<ByteArray>()
        val w2 = ArrayDeque<ByteArray>()
        boards[a].ports[pa] = Link(MockPort(tx = w1, rx = w2))
        boards[b].ports[pb] = Link(MockPort(tx = w2, rx = w1))
    }

    fun attachController(gateway: Int, port: Int) {
        val w1 = ArrayDeque<ByteArray>()
        val w2 = ArrayDeque<ByteArray>()
        ctrl.link = Link(MockPort(tx = w1, rx = w2))
        boards[gateway].ports[port] = Link(MockPort(tx = w2, rx = w1))
    }

    fun liveAddr(board: Int): Int = boards[board].board.addr()
    fun persistedAddr(board: Int): Int? = boards[board].board.persistedAddr()
    fun portToward(board: Int, dst: Int): Int? = boards[board].board.portToward(dst)

    /** Swap in a fresh controller (a new session re-walks the same, already-addressed boards). */
    fun resetController() {
        ctrl.ctrl = Controller()
    }

    /** Every request the controller sent, decoded. */
    val sentRequests: List<Pdu> get() = ctrl.sent.map { Pdu.decode(it) }

    /** The frames the controller refused as not its outstanding request's reply, each with the request then outstanding. */
    val refused: List<Pair<Pdu, Pdu?>>
        get() = ctrl.refused.map { (f, req) -> Pair(Pdu.decode(f), req?.let { Pdu.decode(it) }) }

    /** Send [frame] out the controller's attach link, as a retransmitted request does. */
    fun ctrlSendRaw(frame: ByteArray) = ctrl.link!!.send(frame)

    /**
     * Drive the walk to completion, calling [hook] with each request right after it is sent and
     * before the mesh settles (a test injects a retransmitted copy of an earlier request there).
     */
    fun runWalk(hook: (ByteArray) -> Unit = {}) {
        repeat(1000) {
            if (ctrl.ctrl.isComplete()) return
            ctrl.ctrl.nextRequest()?.let {
                ctrl.link!!.send(it)
                ctrl.sent.add(it)
                hook(it)
            }
            settle()
        }
        error("walk did not complete")
    }

    /** Run the mesh to quiescence: drain the controller and every board, route emissions, fire probes. */
    private fun settle() {
        repeat(10_000) {
            var moved = false

            // Controller inbound: answer a probe-of-me inline; capture CONFIG_RESP; feed replies to the walk.
            while (true) {
                val frame = ctrl.link!!.pollRecv() ?: break
                moved = true
                val reply = ctrl.ctrl.replyToProbe(frame)
                if (reply != null) {
                    ctrl.link!!.send(reply)
                } else if (Pdu.decodeOrNull(frame)?.known() == Opcode.ConfigResp) {
                    ctrl.inbox.add(frame)
                } else if (!ctrl.ctrl.onReply(frame)) {
                    ctrl.refused.add(Pair(frame, ctrl.sent.lastOrNull()))
                }
            }

            // Boards: ingest each ready frame, route emissions.
            for (bi in boards.indices) {
                val node = boards[bi]
                for (p in node.ports.indices) {
                    while (true) {
                        val frame = node.ports[p]?.pollRecv() ?: break
                        moved = true
                        val emits = ArrayList<Emission>()
                        node.board.ingest(p, frame, emits)
                        sendEmits(bi, emits)
                    }
                }
            }

            if (moved) return@repeat

            // No L2 movement: fire any pending probe ticks (the firmware poll's "probe window elapsed").
            var ticked = false
            for (bi in boards.indices) {
                if (boards[bi].board.probing()) {
                    val emits = ArrayList<Emission>()
                    boards[bi].board.pollProbe(emits)
                    sendEmits(bi, emits)
                    ticked = true
                }
            }
            if (!ticked) return
        }
        error("settle did not quiesce")
    }

    private fun sendEmits(bi: Int, emits: List<Emission>) {
        for (e in emits) boards[bi].ports.getOrNull(e.port)?.send(e.bytes)
    }

    /** Reboot every board: clear its routing table and re-read its persisted address. */
    fun rebootAll() {
        for (n in boards) n.board.reboot()
    }

    /** Send a PDU from the controller and settle, returning the captured CONFIG_RESP if one came back. */
    private fun controllerSend(bytes: ByteArray): ByteArray? {
        ctrl.link!!.send(bytes)
        settle()
        return ctrl.inbox.removeLastOrNull()
    }

    /** `CONFIG_WRITE(dst, key, value)` -> the CONFIG_RESP PDU bytes. */
    fun configWrite(dst: Int, key: Key, value: Value): ByteArray =
        controllerSend(ctrl.ctrl.buildConfigWrite(dst, key, value)) ?: error("no CONFIG_RESP")

    /** `CONFIG_READ(dst, key)` -> the CONFIG_RESP PDU bytes. */
    fun configRead(dst: Int, key: Key): ByteArray =
        controllerSend(ctrl.ctrl.buildConfigRead(dst, key)) ?: error("no CONFIG_RESP")
}
