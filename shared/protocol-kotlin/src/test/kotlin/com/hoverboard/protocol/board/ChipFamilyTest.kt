package com.hoverboard.protocol.board

import com.hoverboard.protocol.linkctl.ChipTag
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Test

/**
 * The seam between the part a board NAMES on the wire and the pin list the layout editor picks
 * from: `ChipTag` in `shared/linkctl/src/lib.rs` on one side, [ChipFamily] on the other, joined by
 * [ChipFamily.forTag].
 *
 * The tags are pinned to the Rust by `RustSourceDriftTest`, and either side can grow a member
 * without the other noticing: a fourth fleet part allocated a tag but given no pin list would read
 * on the wire and offer nothing, and the reverse would be a list no board can ask for.
 */
class ChipFamilyTest {

    @Test
    fun everyTagButTheUnknownOneNamesAPartAndEveryPartIsNamedByATag() {
        val named = ChipTag.entries.filter { it != ChipTag.Unknown }
        assertEquals(
            ChipFamily.entries.toSet(),
            named.mapNotNull { ChipFamily.forTag(it) }.toSet(),
            "a wire tag and a capability table are not matched one for one",
        )
        for (tag in named) {
            assertEquals(tag.name, ChipFamily.forTag(tag)?.name, "the tag and the table name different parts")
        }
    }

    @Test
    fun theUnknownTagNamesNoPart() {
        // The board's boot probe measured a family and an advanced-timer count the firmware has no
        // fleet part for. There is no pin list to pick from, and the nearest part is not an
        // answer, so a client is left with nothing rather than a guess.
        assertNull(ChipFamily.forTag(ChipTag.Unknown))
    }
}
