package com.hoverboard.protocol.board

/**
 * The board's boot status record, as a `STATUS` reply to `STATUS_READ` carries it
 * (`specs/machine-settings.md` section 6.1): which configured peripheral did not come up at boot
 * (`specs/configured-links.md` section 5; `motorcontroller/board/src/status.rs`, which
 * `RustSourceDriftTest` pins this against). The configuration itself is checked before it is
 * written ([com.hoverboard.protocol.checks.Checks]), so the one reason left is a runtime fact.
 *
 * The payload is the record's entries as bytes, [ENTRY_LEN] per entry and at most [MAX_ENTRIES] of
 * them. No entries means everything configured came up. A bridge answers it empty.
 */
object BoardStatus {
    /** The most entries the record holds (`status::MAX_ENTRIES`). */
    const val MAX_ENTRIES = 14

    /** The bytes of one entry: field, index, reason, detail (`status::ENTRY_LEN`). */
    const val ENTRY_LEN = 4

    /** Why a function is absent: byte 2 of an entry (`status::Reason`), with what byte 3 means. */
    enum class Reason(val code: Int) {
        /** The peripheral did not come up. Detail: the HAL's error code, or 0 for a probe that did not answer. */
        NOT_UP(6),
        ;

        companion object {
            /** The reason [code] names, or null for a byte no reason has. */
            fun of(code: Int): Reason? = entries.firstOrNull { it.code == code }
        }
    }

    /**
     * One entry: the function named by [field] (its group's first field id, or its own) at [index]
     * (the motor or the link; 0 for a single field) is absent for [reasonCode], with [detail].
     */
    data class Entry(val field: Int, val index: Int, val reasonCode: Int, val detail: Int) {
        /** The [Reason] this entry carries, or null for a byte no reason has. */
        val reason: Reason? get() = Reason.of(reasonCode)
    }

    /** Decode a `STATUS` payload into its entries. A trailing partial entry is ignored. */
    fun decode(bytes: ByteArray): List<Entry> =
        bytes.toList().chunked(ENTRY_LEN).filter { it.size == ENTRY_LEN }.map { e ->
            Entry(e[0].toInt() and 0xFF, e[1].toInt() and 0xFF, e[2].toInt() and 0xFF, e[3].toInt() and 0xFF)
        }
}
