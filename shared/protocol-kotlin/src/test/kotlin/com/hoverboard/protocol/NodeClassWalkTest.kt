package com.hoverboard.protocol

import com.hoverboard.protocol.board.BoardStatus
import com.hoverboard.protocol.l3.ConfigResp
import com.hoverboard.protocol.l3.Identity
import com.hoverboard.protocol.l3.Opcode
import com.hoverboard.protocol.l3.Pdu
import com.hoverboard.protocol.l3.Walk
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Value
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test

/**
 * What a remote app's connect does (`specs/l3.md`, "How far a controller walks";
 * `specs/machine-settings.md` section 6.1): the WHOLE walk, then an `IDENTIFY` and a `CONFIG_READ`
 * of `device.name` to every address the walk addressed. Run against the in-memory mesh ([Mesh]) on
 * the two attach points a phone meets: a hoverboard bridge with the two motor controllers one and
 * two hops behind it (`specs/hoverboard-bridge.md`, "The bridge as a node"), and a motor
 * controller's own BLE module.
 */
class NodeClassWalkTest {

    /** One node as the reads found it: its class byte and its name. */
    private data class Found(val nodeClass: Int, val name: String)

    /** Walk [m], then identify every address and read its name, in address order. */
    private fun walkThenRead(m: Mesh): Map<Int, Found> {
        m.runWalk()
        return m.controller.assignedAddrs().sorted().associateWith { addr ->
            Found(nodeClass = identify(m, addr).nodeClass, name = (read(m, addr, DEVICE_NAME_KEY) as Value.Str).v)
        }
    }

    private fun identify(m: Mesh, addr: Int): Identity {
        val pdu = Pdu.decode(m.identify(addr))
        assertEquals(Opcode.Identity, pdu.known())
        assertEquals(addr, pdu.src, "answered by the node it was sent to")
        return Identity.decode(pdu.payload)!!
    }

    private fun read(m: Mesh, addr: Int, key: Key): Value {
        val pdu = Pdu.decode(m.configRead(addr, key))
        val resp = ConfigResp.parse(pdu)!!
        assertEquals(addr, pdu.src, "answered by the node it was sent to")
        assertEquals(Walk.CFG_OK, resp.status, "a read of 0x${key.fieldId.toString(16)} from 0x${addr.toString(16)}")
        return resp.decodeValue()!!
    }

    @Test
    fun aBridgeAtTheAttachPointAndTwoMotorControllersBehindIt() {
        val m = Mesh()
        // The bridge: port 0 its BLE link (the controller), port 1 the board's SWD mailbox.
        val esp = m.addBoard(2, nodeClass = Walk.CLASS_BRIDGE, name = "hb-bridge-1a2b")
        // The board under it: port 0 the mailbox, port 1 the inter-board UART.
        val right = m.addBoard(2, name = "rover-r")
        val left = m.addBoard(1, name = "rover-l")
        m.attachController(esp, 0)
        m.wire(esp, 1, right, 0)
        m.wire(right, 1, left, 0)

        val nodes = walkThenRead(m)

        // The bridge is the node on the controller's own link, so it is addressed first; the boards
        // are found one and two hops further and reached through it by address.
        assertEquals(
            mapOf(
                m.liveAddr(esp) to Found(Walk.CLASS_BRIDGE, "hb-bridge-1a2b"),
                m.liveAddr(right) to Found(Walk.CLASS_MOTOR, "rover-r"),
                m.liveAddr(left) to Found(Walk.CLASS_MOTOR, "rover-l"),
            ),
            nodes,
        )
        assertEquals(listOf(0x01, 0x02, 0x03), nodes.keys.toList())
        assertEquals(0x01, m.liveAddr(esp))
    }

    @Test
    fun aMotorControllerAtTheAttachPoint() {
        val m = Mesh()
        // A board's own BLE module: port 0 BLE (the controller), port 1 the inter-board UART.
        val near = m.addBoard(2, name = "rover-r")
        val far = m.addBoard(1, name = "rover-l")
        m.attachController(near, 0)
        m.wire(near, 1, far, 0)

        val nodes = walkThenRead(m)

        assertEquals(
            mapOf(0x01 to Found(Walk.CLASS_MOTOR, "rover-r"), 0x02 to Found(Walk.CLASS_MOTOR, "rover-l")),
            nodes,
        )
    }

    /** Each node reports its own factory id, two hops away as on the controller's own link. */
    @Test
    fun theIdentityCarriesTheChipTheFirmwareAndTheNodesOwnUid() {
        val m = Mesh()
        val esp = m.addBoard(2, nodeClass = Walk.CLASS_BRIDGE)
        val board = m.addBoard(1)
        m.attachController(esp, 0)
        m.wire(esp, 1, board, 0)
        m.runWalk()

        val a = identify(m, 0x01)
        val b = identify(m, 0x02)
        assertEquals(Walk.MCU_F1X0, b.mcu)
        assertEquals(0x0001, b.fwVer)
        assertEquals(Identity.UID_LEN, b.uid.size)
        assertEquals(false, a.uid.contentEquals(b.uid), "two nodes, two ids")
    }

    /** `STATUS_READ` answers the boot record's entries from a board, and nothing from a bridge. */
    @Test
    fun theBootReportIsReadWithStatusRead() {
        val m = Mesh()
        // One entry: link 0 (`links[0].tx`) did not come up, detail the HAL's error code.
        val entry = byteArrayOf(0x0F, 0x00, BoardStatus.Reason.NOT_UP.code.toByte(), 0x04)
        val esp = m.addBoard(2, nodeClass = Walk.CLASS_BRIDGE)
        val board = m.addBoard(1, status = entry)
        m.attachController(esp, 0)
        m.wire(esp, 1, board, 0)
        m.runWalk()

        val fromBoard = Pdu.decode(m.statusRead(0x02))
        assertEquals(Opcode.Status, fromBoard.known())
        assertEquals(listOf(BoardStatus.Entry(0x0F, 0, BoardStatus.Reason.NOT_UP.code, 0x04)), BoardStatus.decode(fromBoard.payload))
        assertEquals(0, Pdu.decode(m.statusRead(0x01)).payload.size, "a bridge reports no entries")
    }
}
