package com.hoverboard.app.net

import com.hoverboard.app.net.l3.ConfigResp
import com.hoverboard.app.net.l3.Pdu
import com.hoverboard.app.net.l3.Walk
import com.hoverboard.app.net.l3.isBoard
import com.hoverboard.app.net.store.Key
import com.hoverboard.app.net.store.Value
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNotEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * The L3 controller walk against an in-memory mock board responder, reproducing `specs/l3.md`'s
 * worked topologies (a)/(b). Mirrors `crates/net/src/walk/tests.rs`, so the Kotlin controller and the
 * Rust controller agree on the same topologies. No hardware id is read anywhere - identity is
 * positional during discovery, the persisted address after.
 */
class WalkTest {

    /** `store::MOTOR_CURRENT_LIMIT` (field 0x20, singleton index 0). */
    private val motorCurrentLimit = Key(0x20, 0)

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
    fun anAssignedCollisionIsReassignedAndRePersisted() {
        // The gateway is assigned 0x01; a sideboard boots with a STALE persisted 0x01 (a past session).
        // The walk detects the collision (0x01 at two positions) and reassigns the sideboard,
        // re-persisting. The gateway's own upstream link to the sideboard is correctly excluded from the
        // collision check (the parent link is `naddr == parent`, ignored, not mistaken for a clash).
        val m = Mesh()
        val gw = m.addBoard(2)
        val s1 = m.addBoard(1)
        m.attachController(gw, 0)
        m.wire(gw, 1, s1, 0)
        m.preassign(s1, 0x01) // collides with the gateway's eventual 0x01

        m.runWalk()

        assertEquals(0x01, m.liveAddr(gw))
        val a1 = m.liveAddr(s1)
        assertNotEquals(0x01, a1) // reassigned off the collision
        assertTrue(isBoard(a1))
        assertEquals(a1, m.persistedAddr(s1)) // and re-persisted
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
}
