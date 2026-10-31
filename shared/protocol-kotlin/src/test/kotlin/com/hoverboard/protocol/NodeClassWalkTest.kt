package com.hoverboard.protocol

import com.hoverboard.protocol.l3.ConfigResp
import com.hoverboard.protocol.l3.Pdu
import com.hoverboard.protocol.l3.Walk
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Value
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test

/**
 * What a remote app's connect does (`specs/l3.md`, "How far a controller walks"; `specs/remote-ui.md`
 * section 2): the WHOLE walk, then a `CONFIG_READ` of `node.class` and `device.name` from every
 * address the walk addressed. Run against the in-memory mesh ([Mesh]) on the two attach points a
 * phone meets: a hoverboard bridge with the two motor controllers one and two hops behind it
 * (`specs/hoverboard-bridge.md`, "The bridge as a node"), and a motor controller's own BLE module.
 */
class NodeClassWalkTest {

    /** One node as the read found it: its class byte and its name. */
    private data class Found(val nodeClass: Int, val name: String)

    /** Walk [m], then read the class and the name from every address, in address order. */
    private fun walkThenRead(m: Mesh): Map<Int, Found> {
        m.runWalk()
        return m.controller.assignedAddrs().sorted().associateWith { addr ->
            Found(
                nodeClass = (read(m, addr, NODE_CLASS_KEY) as Value.U8).v,
                name = (read(m, addr, DEVICE_NAME_KEY) as Value.Str).v,
            )
        }
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
        val esp = m.addBoard(2, nodeClass = Fields.NodeClass.BRIDGE, name = "hb-bridge-1a2b")
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
                m.liveAddr(esp) to Found(Fields.NodeClass.BRIDGE, "hb-bridge-1a2b"),
                m.liveAddr(right) to Found(Fields.NodeClass.MOTOR_CONTROLLER, "rover-r"),
                m.liveAddr(left) to Found(Fields.NodeClass.MOTOR_CONTROLLER, "rover-l"),
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
            mapOf(
                0x01 to Found(Fields.NodeClass.MOTOR_CONTROLLER, "rover-r"),
                0x02 to Found(Fields.NodeClass.MOTOR_CONTROLLER, "rover-l"),
            ),
            nodes,
        )
    }

    @Test
    fun aWriteOfTheClassIsRefusedReadOnlyAndChangesNothing() {
        val m = Mesh()
        val esp = m.addBoard(2, nodeClass = Fields.NodeClass.BRIDGE)
        val board = m.addBoard(1)
        m.attachController(esp, 0)
        m.wire(esp, 1, board, 0)
        m.runWalk()

        val resp = ConfigResp.parse(Pdu.decode(m.configWrite(0x02, NODE_CLASS_KEY, Value.U8(Fields.NodeClass.BRIDGE))))!!

        assertEquals(Walk.CFG_READ_ONLY, resp.status)
        assertEquals(Value.U8(Fields.NodeClass.MOTOR_CONTROLLER), read(m, 0x02, NODE_CLASS_KEY))
    }
}
