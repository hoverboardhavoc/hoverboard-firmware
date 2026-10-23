package com.hoverboard.remote.ui.screens

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Row
import androidx.compose.material3.FilterChip
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.unit.dp
import com.hoverboard.remote.R
import com.hoverboard.remote.model.Node

/**
 * The board selector the per-board screens share (`specs/rider-ui.md` 3.3 and 3.7): one chip for
 * the master the phone is attached to and one for the slave this session's discovery found behind
 * it. Each chip carries the address the session learned for that role, never one assumed from the
 * role: which of the pair holds 0x01 differs from pair to pair, and only the walk knows. The
 * slave's chip is offered once discovery has found one and says so until then.
 *
 * @param slaveTag the test tag on the slave's chip, so each screen's tests find their own.
 */
@Composable
internal fun BoardTargets(
    target: Node,
    master: Int?,
    slave: Int?,
    onSelect: (Node) -> Unit,
    slaveTag: String,
) {
    Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
        FilterChip(
            selected = target == Node.MASTER,
            onClick = { onSelect(Node.MASTER) },
            label = {
                Text(
                    master?.let { stringResource(R.string.board_target_master, boardHex(it)) }
                        ?: stringResource(R.string.board_target_master_none),
                )
            },
        )
        FilterChip(
            selected = target == Node.SLAVE,
            onClick = { onSelect(Node.SLAVE) },
            enabled = slave != null,
            label = {
                Text(
                    slave?.let { stringResource(R.string.board_target_slave, boardHex(it)) }
                        ?: stringResource(R.string.board_target_slave_none),
                )
            },
            modifier = Modifier.testTag(slaveTag),
        )
    }
}
