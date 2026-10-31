package com.hoverboard.protocol

import com.hoverboard.protocol.l3.ConfigResp
import com.hoverboard.protocol.l3.Controller
import com.hoverboard.protocol.l3.GUEST_FIRST
import com.hoverboard.protocol.l3.NO_ADDRESS
import com.hoverboard.protocol.l3.Opcode
import com.hoverboard.protocol.l3.Pdu
import com.hoverboard.protocol.l3.Walk
import com.hoverboard.protocol.l3.isBoard
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Gains
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Value
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNotEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * The L3 controller walk against an in-memory mock board responder, reproducing `specs/l3.md`'s
 * worked topologies (a)/(b). Mirrors `shared/net/src/walk/tests.rs`, so the Kotlin controller and the
 * Rust controller agree on the same topologies. No hardware id is read anywhere - identity is
 * positional during discovery, the persisted address after.
 */
class WalkTest {

    /** `store::MOTOR_CURRENT_LIMIT` (field 0x20, singleton index 0). */
    private val motorCurrentLimit = Key(0x20, 0)

    @Test
    fun aRefusedAssignRecordsNothing() {
        // The firmware refuses an ASSIGN while the board is armed and ACKs with STATUS_ERR. Taking
        // that as an assignment would put an address nobody holds into the controller's map, and a
        // caller would then address a board that never adopted it.
        val c = Controller()
        assertTrue(c.nextRequest() != null) // NODE_HELLO
        val hello = byteArrayOf(NO_ADDRESS.toByte(), Walk.PROTO_VER.toByte(), 0, 0, 0, 0x80.toByte())
        c.onReply(Pdu.of(Opcode.NodeHello, NO_ADDRESS, 0x80, hello).encode())

        val assign = Pdu.decode(c.nextRequest()!!)
        assertEquals(Opcode.Assign, assign.known())
        val newAddr = assign.payload[1].toInt() and 0xFF

        val refusal = byteArrayOf(newAddr.toByte(), Walk.STATUS_ERR.toByte())
        c.onReply(Pdu.of(Opcode.AssignAck, newAddr, 0x80, refusal).encode())

        assertEquals(emptyList<Int>(), c.assignedAddrs())
        assertEquals(null, c.gatewayAddr())
    }

    /**
     * `PORTS` `neighbour_state` carries the state in bits 0-1 and the link-group flag in bit 7
     * (`specs/l3.md`, "Tier-1 wire encoding"). A reply with bit 7 set on every port still
     * classifies each one: the controller is found on port 0, port 1's unassigned neighbour is
     * assigned, and the empty port 2 queues nothing.
     */
    @Test
    fun theControllerReadsTheNeighbourStateFromBits0And1Only() {
        val group = 0x80
        val c = Controller()
        c.nextRequest()!! // NODE_HELLO
        val hello = byteArrayOf(NO_ADDRESS.toByte(), Walk.PROTO_VER.toByte(), 0, 0, 0x10, 0x80.toByte())
        c.onReply(Pdu.of(Opcode.NodeHello, NO_ADDRESS, 0x80, hello).encode())
        c.nextRequest()!! // ASSIGN the gateway 0x01
        c.onReply(Pdu.of(Opcode.AssignAck, 0x01, 0x80, byteArrayOf(0x01, Walk.STATUS_OK.toByte())).encode())
        assertEquals(Opcode.ProbePorts, Pdu.decode(c.nextRequest()!!).known())

        val ports = intArrayOf(
            3,
            0, Walk.PORT_BLE, Walk.NB_ASSIGNED or group, 0x80,
            1, Walk.PORT_UART, Walk.NB_UNASSIGNED or group, 0x00,
            2, Walk.PORT_UART, Walk.NB_EMPTY or group, 0x00,
        ).map { it.toByte() }.toByteArray()
        c.onReply(Pdu.of(Opcode.Ports, 0x01, 0x80, ports).encode())

        val assign = Pdu.decode(c.nextRequest()!!)
        assertEquals(Opcode.Assign, assign.known())
        assertEquals(0x01, assign.dst)
        assertEquals(1, assign.payload[0].toInt() and 0xFF, "port 1 still reads as unassigned")
        val na = assign.payload[1].toInt() and 0xFF
        c.onReply(Pdu.of(Opcode.AssignAck, na, 0x80, byteArrayOf(na.toByte(), Walk.STATUS_OK.toByte())).encode())
        // The new neighbour is probed next; the controller's own port and the empty one queued nothing.
        assertEquals(Opcode.ProbePorts, Pdu.decode(c.nextRequest()!!).known())
        assertEquals(listOf(0x01, na), c.assignedAddrs())
    }

    // Topology (a): a 12-FET gateway + two attitude sideboards.
    @Test
    fun topologyAAddressesAndPersistsEveryBoard() {
        val m = Mesh()
        val gw = m.addBoard(3) // attach(0) + two sideboard ports(1,2)
        val s1 = m.addBoard(1)
        val s2 = m.addBoard(1)
        m.attachController(gw, 0)
        m.wire(gw, 1, s1, 0)
        m.wire(gw, 2, s2, 0)

        m.runWalk()

        assertEquals(0x01, m.liveAddr(gw))
        val a1 = m.liveAddr(s1)
        val a2 = m.liveAddr(s2)
        assertTrue(isBoard(a1) && isBoard(a2))
        assertNotEquals(a1, a2)
        assertEquals(listOf(0x02, 0x03), listOf(a1, a2).sorted())

        // Every address is PERSISTED (survives a reboot), not just live.
        assertEquals(0x01, m.persistedAddr(gw))
        assertEquals(a1, m.persistedAddr(s1))
        assertEquals(a2, m.persistedAddr(s2))

        // The controller's map holds exactly the three boards.
        assertEquals(listOf(0x01, 0x02, 0x03), m.controller.assignedAddrs().sorted())
    }

    // Topology (b): the master/slave pair.
    @Test
    fun topologyBMasterSlavePair() {
        val m = Mesh()
        val gw = m.addBoard(2) // attach(0) + inter-board(1)
        val slave = m.addBoard(1)
        m.attachController(gw, 0)
        m.wire(gw, 1, slave, 0)

        m.runWalk()

        assertEquals(0x01, m.liveAddr(gw))
        assertEquals(0x02, m.liveAddr(slave))
        assertEquals(0x01, m.persistedAddr(gw))
        assertEquals(0x02, m.persistedAddr(slave))
    }

    @Test
    fun twoIdenticalBoardsProvisionByPositionNoIdRead() {
        // The two sideboards are byte-for-byte identical responders (same mcu, same fw, no device id).
        // They still get distinct addresses, distinguished only by which gateway port they sit on.
        val m = Mesh()
        val gw = m.addBoard(3)
        val s1 = m.addBoard(1)
        val s2 = m.addBoard(1)
        m.attachController(gw, 0)
        m.wire(gw, 1, s1, 0)
        m.wire(gw, 2, s2, 0)
        m.runWalk()
        assertNotEquals(m.liveAddr(s1), m.liveAddr(s2))
    }

    @Test
    fun configWriteReadRoundTripToTheGateway() {
        val m = Mesh()
        val gw = m.addBoard(2)
        val slave = m.addBoard(1)
        m.attachController(gw, 0)
        m.wire(gw, 1, slave, 0)
        m.runWalk()

        // CONFIG_WRITE then CONFIG_READ a U32 field on the gateway (0x01); the response round-trips.
        val wResp = ConfigResp.parse(Pdu.decode(m.configWrite(0x01, motorCurrentLimit, Value.U32(15_000))))!!
        assertEquals(Walk.CFG_OK, wResp.status)
        assertEquals(Value.U32(15_000), wResp.decodeValue())

        val rResp = ConfigResp.parse(Pdu.decode(m.configRead(0x01, motorCurrentLimit)))!!
        assertEquals(Walk.CFG_OK, rResp.status)
        assertEquals(motorCurrentLimit.fieldId, rResp.fieldId)
        assertEquals(Value.U32(15_000), rResp.decodeValue())
    }

    @Test
    fun aNodeAssignedThisWalkMovesOffAnAddressANeighbourAlreadyHeld() {
        // The gateway is fresh and is assigned 0x01; a sideboard boots holding 0x01 from a past
        // session. The walk moves the node IT assigned (the gateway), and the sideboard keeps the
        // address it held (`specs/l3.md`, "Discovery and address assignment", step 3).
        val m = Mesh()
        val gw = m.addBoard(2)
        val s1 = m.addBoard(1)
        m.attachController(gw, 0)
        m.wire(gw, 1, s1, 0)
        m.preassign(s1, 0x01)

        m.runWalk()

        assertEquals(0x01, m.liveAddr(s1))
        assertEquals(0x02, m.liveAddr(gw))
        assertEquals(0x02, m.persistedAddr(gw))
        assertEquals(0x02, m.controller.gatewayAddr())
    }

    @Test
    fun twoNodesThatBothHeldOneAddressReassignTheOneReachedSecond() {
        val m = Mesh()
        val gw = m.addBoard(2)
        val s1 = m.addBoard(1)
        m.attachController(gw, 0)
        m.wire(gw, 1, s1, 0)
        m.preassign(gw, 0x05)
        m.preassign(s1, 0x05)

        m.runWalk()

        assertEquals(0x05, m.liveAddr(gw))
        val a1 = m.liveAddr(s1)
        assertNotEquals(0x05, a1) // reassigned off the collision
        assertTrue(isBoard(a1))
        assertEquals(a1, m.persistedAddr(s1)) // and re-persisted
    }

    @Test
    fun aFreshBridgeInFrontOfAnAddressedPairEndsAt0x03AndTheBoardsKeepTheirs() {
        // `specs/hoverboard-bridge.md`, "A fresh bridge in front of an addressed machine".
        val m = Mesh()
        val esp = m.addBoard(2, nodeClass = Walk.CLASS_BRIDGE)
        val master = m.addBoard(2)
        val slave = m.addBoard(1)
        m.preassign(master, 0x01)
        m.preassign(slave, 0x02)
        m.attachController(esp, 0)
        m.wire(esp, 1, master, 0)
        m.wire(master, 1, slave, 0)

        m.runWalk()

        assertEquals(0x03, m.liveAddr(esp))
        assertEquals(0x03, m.persistedAddr(esp))
        assertEquals(0x01, m.liveAddr(master))
        assertEquals(0x02, m.liveAddr(slave))
        assertEquals(listOf(0x01, 0x02, 0x03), m.controller.assignedAddrs().sorted())
    }

    /**
     * A board switched on after the walk (`specs/remote-ui.md` 3.1, "Boards that come up after the
     * connect"): the same controller re-walks from the gateway, assigns the new board and its peer,
     * and passes over nothing it already knew.
     */
    @Test
    fun aReWalkFromTheGatewayFindsABoardThatCameUpAfterTheWalk() {
        val m = Mesh()
        val bridge = m.addBoard(2, nodeClass = Fields.NodeClass.BRIDGE, name = "offroad-master")
        m.attachController(bridge, 0)
        m.runWalk()
        assertEquals(listOf(0x01), m.controller.assignedAddrs())

        // The boards were unpowered; now the master behind the bridge's port 1, and its slave, are up.
        val master = m.addBoard(2)
        val slave = m.addBoard(1)
        m.wire(bridge, 1, master, 0)
        m.wire(master, 1, slave, 0)
        val before = m.sentRequests.size
        m.controller.rewalk(listOf(0x01, 0x7E))
        m.runWalk()

        assertEquals(listOf(0x01, 0x02, 0x03), m.controller.assignedAddrs().sorted())
        assertEquals(0x02, m.liveAddr(master))
        assertEquals(0x03, m.liveAddr(slave))
        // One probe of the bridge, an ASSIGN and a probe per new board; an unplaced address is not probed.
        val again = m.sentRequests.drop(before)
        assertEquals(listOf(0x01, 0x02, 0x03), again.filter { it.known() == Opcode.ProbePorts }.map { it.dst }.sorted())

        // A second re-walk with nothing new sends probes and assigns nothing.
        val settled = m.sentRequests.size
        m.controller.rewalk(listOf(0x01))
        m.runWalk()
        assertEquals(listOf(Opcode.ProbePorts), m.sentRequests.drop(settled).map { it.known() })
        assertEquals(listOf(0x01, 0x02, 0x03), m.controller.assignedAddrs().sorted())
    }

    @Test
    fun aReWalkAdoptsAssignedBoardsAndAssignsNothingNew() {
        // First walk addresses everyone; a FRESH controller re-walks the SAME (already-addressed)
        // boards. This exercises the stale-adopt path: a board reporting a persisted address the
        // controller has not handed out this session is ADOPTED (recorded + descended), never re-minted.
        val m = Mesh()
        val gw = m.addBoard(3)
        val s1 = m.addBoard(1)
        val s2 = m.addBoard(1)
        m.attachController(gw, 0)
        m.wire(gw, 1, s1, 0)
        m.wire(gw, 2, s2, 0)
        m.runWalk()
        val first = m.controller.assignedAddrs().sorted()

        // A second controller attaches and walks again. Boards keep their persisted addresses.
        m.resetController()
        m.runWalk()
        val second = m.controller.assignedAddrs().sorted()

        assertEquals(listOf(0x01, 0x02, 0x03), first)
        assertEquals(first, second) // same addresses reported, none newly minted (adopted, not reassigned)
        assertEquals(0x01, m.persistedAddr(gw))
        assertNotEquals(m.persistedAddr(s1), m.persistedAddr(s2))
    }

    @Test
    fun twoHopConfigToTheSlaveThroughTheGateway() {
        // Slice-2 preview of the Tier-3 two-hop path (host-tested): a CONFIG to the slave (0x02)
        // reaches it THROUGH the gateway (source-learning routes the request and the reply).
        val m = Mesh()
        val gw = m.addBoard(2)
        val slave = m.addBoard(1)
        m.attachController(gw, 0)
        m.wire(gw, 1, slave, 0)
        m.runWalk()

        val wResp = ConfigResp.parse(Pdu.decode(m.configWrite(0x02, motorCurrentLimit, Value.U32(21_000))))!!
        assertEquals(Walk.CFG_OK, wResp.status)
        val rResp = ConfigResp.parse(Pdu.decode(m.configRead(0x02, motorCurrentLimit)))!!
        assertEquals(Value.U32(21_000), rResp.decodeValue())
    }

    /**
     * The tune lane's two requests, byte for byte (`specs/remote-ui.md` section 4). There is no
     * round-trip against the mock board here on purpose: the board side of this lane is not the
     * walk responder, it is the firmware's control block, so the mock has nothing to answer with.
     * What the mirror owes is the request shape and the reply parse, and those are pinned here.
     */
    @Test
    fun theTuneLaneBuildsConfigShapedRequestsUnderItsOwnOpcodes() {
        // A fresh controller holds the provisional guest address until a gateway grants one.
        val c = Controller()
        val key = Gains.key(Gains.CONTROL_SF_GAINS, Gains.GR)

        val w = Pdu.decode(c.buildTuneWrite(0x02, key, 2500))
        assertEquals(Walk.OP_TUNE_WRITE, w.opcode)
        assertEquals(GUEST_FIRST, w.src)
        assertEquals(0x02, w.dst)
        // The CONFIG_WRITE payload shape: [field_id, index, type_tag, value_le].
        assertEquals(
            listOf(Gains.CONTROL_SF_GAINS, Gains.GR, com.hoverboard.protocol.store.Type.I16.tag, 0xC4, 0x09),
            w.payload.map { it.toInt() and 0xFF },
        )

        val r = Pdu.decode(c.buildTuneRead(0x02, key))
        assertEquals(Walk.OP_TUNE_READ, r.opcode)
        assertEquals(listOf(Gains.CONTROL_SF_GAINS, Gains.GR), r.payload.map { it.toInt() and 0xFF })

        // Neither opcode is one L3 interprets: a board forwards or hands them on by dst, and the
        // Kotlin `Opcode` table must not grow them either (the drift gate reads that table).
        assertEquals(null, w.known())
        assertEquals(null, r.known())

        // The reply is an ordinary CONFIG_RESP and parses as one.
        val resp = ConfigResp.parse(
            Pdu(Opcode.ConfigResp.value, 0x02, 0x80, byteArrayOf(0x79, 1, Walk.CFG_OK.toByte(), 0x05, 0xC4.toByte(), 0x09)),
        )!!
        assertEquals(Walk.CFG_OK, resp.status)
        assertEquals(Value.I16(2500), resp.decodeValue())

        // And the mirrored range is the client-side half of the seam the board enforces.
        assertTrue(Gains.inRange(Gains.GR, 2500))
        assertTrue(!Gains.inRange(Gains.GX, Gains.DEFAULT_MAX[Gains.GX] + 1))
        assertTrue(Gains.inRange(Gains.GX, 12000, max = 12000), "a board's own maximum widens it")
        assertTrue(!Gains.inRange(Gains.GR, 1, max = -5), "a negative maximum reads as 0")
        assertTrue(Gains.inRange(Gains.GR, 0, max = -5))
        assertTrue(!Gains.inRange(4, 0), "no such gain")
        assertTrue(Gains.referenceInRange(Gains.SPEED_SLEW, 1))
        assertTrue(!Gains.referenceInRange(Gains.SPEED_SLEW, 0), "the slew floor is 1")
        assertTrue(!Gains.referenceInRange(Gains.SPEED_MAX, 513))
        assertTrue(!Gains.referenceInRange(2, 0), "no such reference word")
        assertEquals(0, Gains.default(Gains.CONTROL_SF_GAINS, Gains.GR))
        assertEquals(41, Gains.default(Gains.CONTROL_REFERENCE, Gains.SPEED_SLEW))
        assertEquals(null, Gains.default(0x70, 0))
    }

    // A reply is bound to the request it answers (`specs/l3.md`, "Discovery and address
    // assignment"): the two 2026-10-25 renumberings replayed, and each half of the rule on its own.
    // Mirrors `shared/net/src/walk/tests.rs` (a_duplicate_ports_from_the_slave_is_not_the_masters_reply_trace_1,
    // a_duplicate_ports_from_the_master_is_not_the_slaves_reply_trace_2, a_late_ports_from_a_previous_probe_is_ignored,
    // an_assign_ack_naming_another_address_is_ignored, a_hello_reply_to_a_stale_guest_address_is_ignored).

    private fun bytes(vararg v: Int) = v.map { it.toByte() }.toByteArray()

    private fun pdu(op: Opcode, src: Int, dst: Int, payload: ByteArray) = Pdu.of(op, src, dst, payload).encode()

    /** Replay a retransmitted `PROBE_PORTS(first)` once, right after `PROBE_PORTS(second)` goes out. */
    private fun replayProbe(m: Mesh, first: Int, second: Int): (ByteArray) -> Unit {
        var held: ByteArray? = null
        var replayed = false
        return { req ->
            val p = Pdu.decode(req)
            if (p.known() == Opcode.ProbePorts && !replayed) {
                when (p.dst) {
                    first -> held = req
                    second -> {
                        m.ctrlSendRaw(held!!)
                        replayed = true
                    }
                }
            }
        }
    }

    @Test
    fun aDuplicatePortsFromTheSlaveIsNotTheMastersReplyTrace1() {
        // The phone attached through the slave; a retransmitted Probe(0x02) reaches the slave after
        // Probe(0x01) went out, and its second PORTS(0x02) arrives while Probe(0x01) is
        // outstanding. Read as the master's table it renumbered the slave to 0x03.
        val m = Mesh()
        val slave = m.addBoard(3) // first, so its duplicate reaches the controller first
        val master = m.addBoard(3)
        m.preassign(master, 0x01)
        m.preassign(slave, 0x02)
        m.attachController(slave, 2)
        m.wire(slave, 1, master, 1)

        m.runWalk(replayProbe(m, first = 0x02, second = 0x01))

        assertTrue(
            m.refused.any { (f, req) ->
                f.known() == Opcode.Ports && f.src == 0x02 &&
                    req?.known() == Opcode.ProbePorts && req.dst == 0x01
            },
            "the duplicate PORTS(0x02) arrived while Probe(0x01) was outstanding and was refused",
        )
        assertTrue(m.sentRequests.none { it.known() == Opcode.Assign }, "no ASSIGN was sent")
        assertEquals(0x02, m.liveAddr(slave))
        assertEquals(0x02, m.persistedAddr(slave))
        assertEquals(0x01, m.liveAddr(master))
        assertEquals(0x01, m.persistedAddr(master))
        assertEquals(listOf(0x01, 0x02), m.controller.assignedAddrs().sorted())
    }

    @Test
    fun aDuplicatePortsFromTheMasterIsNotTheSlavesReplyTrace2() {
        // The phone attached at bridge 0x03 (ports 0 BLE, 1 mailbox to the master, 2 console); the
        // slave's bridge 0x04 is on the slave's mailbox. A retransmitted Probe(0x01) answers twice,
        // and the duplicate PORTS(0x01) arrives while Probe(0x02) is outstanding. Read as the
        // slave's table it queued ASSIGN(0x02, egress 0, 0x04) and ASSIGN(0x02, egress 1, 0x05),
        // and the master was renumbered to 0x05.
        val m = Mesh()
        val near = m.addBoard(3, nodeClass = Walk.CLASS_BRIDGE)
        val master = m.addBoard(3)
        val slave = m.addBoard(3)
        val far = m.addBoard(3, nodeClass = Walk.CLASS_BRIDGE)
        m.preassign(near, 0x03)
        m.preassign(master, 0x01)
        m.preassign(slave, 0x02)
        m.preassign(far, 0x04)
        m.attachController(near, 0)
        m.wire(near, 1, master, 0)
        m.wire(master, 1, slave, 1)
        m.wire(slave, 0, far, 1)

        m.runWalk(replayProbe(m, first = 0x01, second = 0x02))

        assertTrue(
            m.refused.any { (f, req) ->
                f.known() == Opcode.Ports && f.src == 0x01 &&
                    req?.known() == Opcode.ProbePorts && req.dst == 0x02
            },
            "the duplicate PORTS(0x01) arrived while Probe(0x02) was outstanding and was refused",
        )
        assertTrue(m.sentRequests.none { it.known() == Opcode.Assign }, "no ASSIGN was sent")
        for ((node, addr) in listOf(near to 0x03, master to 0x01, slave to 0x02, far to 0x04)) {
            assertEquals(addr, m.liveAddr(node))
            assertEquals(addr, m.persistedAddr(node))
        }
        assertEquals(listOf(0x01, 0x02, 0x03, 0x04), m.controller.assignedAddrs().sorted())
        assertEquals(0x03, m.controller.gatewayAddr())
    }

    @Test
    fun aLatePortsFromAPreviousProbeIsIgnored() {
        // The first PORTS(0x01) is held up past its retransmit; the walk moves on to assign 0x02
        // and probe it, and then the held-up copy arrives, still showing port 1 unassigned.
        val c = Controller()
        c.nextRequest()!!
        assertTrue(c.onReply(pdu(Opcode.NodeHello, NO_ADDRESS, 0x80, bytes(0, Walk.PROTO_VER, 0, 0, 0x10, 0x80))))
        c.nextRequest()!!
        assertTrue(c.onReply(pdu(Opcode.AssignAck, 0x01, 0x80, bytes(0x01, Walk.STATUS_OK))))
        c.nextRequest()!! // Probe(0x01)
        val gatewayPorts = pdu(
            Opcode.Ports, 0x01, 0x80,
            bytes(2, 0, Walk.PORT_BLE, Walk.NB_ASSIGNED, 0x80, 1, Walk.PORT_UART, Walk.NB_UNASSIGNED, 0),
        )
        assertTrue(c.onReply(gatewayPorts))
        assertEquals(listOf(1, 0x02), Pdu.decode(c.nextRequest()!!).payload.map { it.toInt() })
        assertTrue(c.onReply(pdu(Opcode.AssignAck, 0x02, 0x80, bytes(0x02, Walk.STATUS_OK))))
        assertEquals(0x02, Pdu.decode(c.nextRequest()!!).dst)

        // The late copy is not Probe(0x02)'s reply: refused, still outstanding, nothing queued.
        assertTrue(!c.onReply(gatewayPorts))
        assertEquals(null, c.nextRequest())
        assertTrue(!c.isComplete())

        assertTrue(c.onReply(pdu(Opcode.Ports, 0x02, 0x80, bytes(1, 0, Walk.PORT_UART, Walk.NB_ASSIGNED, 0x01))))
        assertTrue(c.isComplete(), "no ASSIGN was queued")
        assertEquals(listOf(0x01, 0x02), c.assignedAddrs())
    }

    @Test
    fun anAssignAckNamingAnotherAddressIsIgnored() {
        val c = Controller()
        c.nextRequest()!!
        assertTrue(c.onReply(pdu(Opcode.NodeHello, NO_ADDRESS, 0x80, bytes(0, Walk.PROTO_VER, 0, 0, 0x10, 0x80))))
        c.nextRequest()!! // AssignGateway(0x01)
        for ((src, dst, named) in listOf(Triple(0x02, 0x80, 0x02), Triple(0x01, 0x80, 0x02), Triple(0x02, 0x80, 0x01), Triple(0x01, 0x81, 0x01))) {
            assertTrue(!c.onReply(pdu(Opcode.AssignAck, src, dst, bytes(named, Walk.STATUS_OK))), "src $src dst $dst names $named")
            assertEquals(null, c.nextRequest())
            assertEquals(emptyList<Int>(), c.assignedAddrs())
        }
        assertTrue(c.onReply(pdu(Opcode.AssignAck, 0x01, 0x80, bytes(0x01, Walk.STATUS_OK))))

        // The neighbour arm the same way.
        c.nextRequest()!! // Probe(0x01)
        assertTrue(c.onReply(pdu(Opcode.Ports, 0x01, 0x80, bytes(1, 0, Walk.PORT_UART, Walk.NB_UNASSIGNED, 0))))
        c.nextRequest()!! // ASSIGN(dst 0x01, egress 0, 0x02)
        assertTrue(!c.onReply(pdu(Opcode.AssignAck, 0x03, 0x80, bytes(0x03, Walk.STATUS_OK))))
        assertTrue(!c.onReply(pdu(Opcode.AssignAck, 0x02, 0x80, bytes(0x03, Walk.STATUS_OK))))
        assertEquals(listOf(0x01), c.assignedAddrs())
        assertTrue(c.onReply(pdu(Opcode.AssignAck, 0x02, 0x80, bytes(0x02, Walk.STATUS_OK))))
        assertEquals(listOf(0x01, 0x02), c.assignedAddrs())
    }

    @Test
    fun aHelloReplyToAStaleGuestAddressIsIgnored() {
        val c = Controller()
        assertEquals(GUEST_FIRST, Pdu.decode(c.nextRequest()!!).src)
        assertTrue(!c.onReply(pdu(Opcode.NodeHello, 0x01, 0x81, bytes(0x01, Walk.PROTO_VER, 0, 0, 0x10, 0x89))))
        assertEquals(GUEST_FIRST, c.guestAddr)
        assertEquals(null, c.gatewayAddr())
        assertEquals(null, c.nextRequest(), "the Hello is still outstanding")

        assertTrue(c.onReply(pdu(Opcode.NodeHello, 0x01, 0x80, bytes(0x01, Walk.PROTO_VER, 0, 0, 0x10, 0x88))))
        assertEquals(0x88, c.guestAddr)
        assertEquals(0x01, c.gatewayAddr())
    }
}
