package com.hoverboard.remote.ui.screens

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.ExperimentalLayoutApi
import androidx.compose.foundation.layout.FlowRow
import androidx.compose.material3.FilterChip
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.unit.dp
import com.hoverboard.remote.model.NodeInfo

/** Test tag on the chip for the node at [address], under a selector tagged [prefix]. */
fun nodeChipTag(prefix: String, address: Int): String = "${prefix}_${boardHex(address)}"

/**
 * A node's label everywhere the app names one (`specs/remote-ui.md` section 2): "<device.name>
 * 0x01", or the address alone while its name is unread. Never a role: two motor controllers with the
 * same name are told apart by the address.
 */
fun nodeLabel(node: NodeInfo): String =
    node.name?.let { "$it ${boardHex(node.address)}" } ?: boardHex(node.address)

/**
 * A node selector (`specs/remote-ui.md` 3.3 and 3.4): one chip per node in [nodes], in address order,
 * labelled by [nodeLabel]. Each screen passes the nodes it offers: Tune the motor controllers, Setup
 * every node. [enabled] is false for the whole row on a screen that cannot take a pick now.
 *
 * @param tag the prefix of each chip's test tag ([nodeChipTag]), so each screen's tests find their own.
 */
@OptIn(ExperimentalLayoutApi::class)
@Composable
internal fun NodeChips(
    nodes: List<NodeInfo>,
    selected: Int?,
    onSelect: (Int) -> Unit,
    tag: String,
    enabled: Boolean = true,
) {
    FlowRow(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
        for (node in nodes) {
            FilterChip(
                selected = selected == node.address,
                onClick = { onSelect(node.address) },
                label = { Text(nodeLabel(node)) },
                enabled = enabled,
                modifier = Modifier.testTag(nodeChipTag(tag, node.address)),
            )
        }
    }
}
