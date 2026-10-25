package com.hoverboard.remote

import com.hoverboard.protocol.config.Busy
import com.hoverboard.protocol.config.CfgRefusal
import com.hoverboard.protocol.config.ConfigReadResult
import com.hoverboard.protocol.config.ConfigWriteResult
import com.hoverboard.protocol.config.Malformed
import com.hoverboard.protocol.config.ReadValue
import com.hoverboard.protocol.config.Refused
import com.hoverboard.protocol.config.TimedOut
import com.hoverboard.protocol.config.StagedValue
import com.hoverboard.protocol.config.TuneReadResult
import com.hoverboard.protocol.config.TuneVerified
import com.hoverboard.protocol.config.TuneWriteResult
import com.hoverboard.protocol.config.WriteVerified
import com.hoverboard.protocol.l3.ConfigResp
import com.hoverboard.protocol.l3.Walk
import com.hoverboard.protocol.linkctl.ChipTag
import com.hoverboard.protocol.linkctl.CyclicObs
import com.hoverboard.protocol.linkctl.CyclicState
import com.hoverboard.protocol.store.Fields
import com.hoverboard.protocol.store.Gains
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.ble.HoverboardTransport
import com.hoverboard.remote.model.ConnectionState
import com.hoverboard.remote.model.DriveFrame
import com.hoverboard.remote.model.FrameStats
import com.hoverboard.remote.model.RiderCommand
import com.hoverboard.remote.model.TelemetryInbox
import com.hoverboard.remote.model.TelemetryUi
import kotlinx.coroutines.CompletableDeferred
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.test.TestCoroutineScheduler
import kotlinx.coroutines.test.runCurrent

/**
 * In-memory fake transport for unit tests. Records every [RiderCommand] the ViewModel produced and
 * lets tests drive connection state and inject a [CyclicState], with no Android BLE stack.
 *
 * It records commands rather than wire bytes on purpose. What the app *intends* is this seam's
 * business; how that intent is spelled on the wire is [RiderCommand.pdus]', and the tests that care
 * about opcodes call it directly rather than trusting a re-implementation here.
 *
 * @param scheduler the test scheduler driving the ViewModel, so [setConnectionState] can deliver
 *   each change rather than leaving it to a caller who might not.
 */
@OptIn(ExperimentalCoroutinesApi::class)
class FakeHoverboardTransport(
    private val scheduler: TestCoroutineScheduler,
) : HoverboardTransport {

    private val _connectionState = MutableStateFlow(ConnectionState.DISCONNECTED)
    override val connectionState: StateFlow<ConnectionState> = _connectionState

    private val inbox = TelemetryInbox()
    override val telemetry: StateFlow<Map<Int, TelemetryUi>> = inbox.boards
    override val frameStats: StateFlow<Map<Int, FrameStats>> = inbox.frameStats

    /** Every [DriveFrame] the ViewModel produced, in order. */
    val frames: MutableList<DriveFrame> = mutableListOf()

    /**
     * The [RiderCommand] each frame told [ONE_BOARD], in order, skipping a frame that addressed it not
     * at all (one sent before the board's class was read): what a one-board test reads.
     */
    val sent: List<RiderCommand> get() = frames.mapNotNull { it.commands[ONE_BOARD] }

    val last: RiderCommand? get() = sent.lastOrNull()

    var connectCalls: Int = 0
        private set
    var disconnectCalls: Int = 0
        private set

    /**
     * How many commands had been sent by the time [disconnect] was called, or null if it never was.
     *
     * This is what lets a test check ORDER rather than just contents: the guarantee that matters on
     * the disconnect path is that the disarming command went out BEFORE the link was dropped, and a
     * fake that only records the final state cannot tell those two apart.
     */
    var sentAtDisconnect: Int? = null
        private set

    override fun connect() {
        connectCalls++
    }

    override fun disconnect() {
        disconnectCalls++
        sentAtDisconnect = sent.size
    }

    override fun sendCommand(frame: DriveFrame) {
        frames.add(frame)
    }

    private val _attachedBoard = MutableStateFlow<Int?>(null)
    override val attachedBoard: StateFlow<Int?> = _attachedBoard

    private val _nodes = MutableStateFlow<List<Int>>(emptyList())
    override val nodes: StateFlow<List<Int>> = _nodes

    /**
     * The fake board's store, by (target, key). A read of a key it does not hold answers the
     * caller's [defaults] entry, the way a real board answers a registered field's default.
     *
     * Every node starts out answering what every board running this firmware answers unwritten:
     * `node.class` motor controller, its default `device.name`, and `machine.drive_mode` SINGLE. A
     * test about a bridge or a stored mode says so in [store].
     */
    val store: MutableMap<Pair<Int, Key>, Value> = mutableMapOf()
    val defaults: MutableMap<Key, Value> = mutableMapOf(
        Fields.NODE_CLASS.key() to Fields.NODE_CLASS.default,
        Fields.DEVICE_NAME.key() to Fields.DEVICE_NAME.default,
        Fields.MACHINE_DRIVE_MODE.key() to Fields.MACHINE_DRIVE_MODE.default,
    )

    /** Whether the fake board refuses writes with `CFG_ARMED`, as a real one does while armed. */
    var boardArmed: Boolean = false

    /**
     * Answers a write in place of the fake board, or null to let the board store and verify it. It
     * runs before the store is touched, so a hook that writes [store] itself and answers [TimedOut]
     * is a write that landed and whose answer was lost.
     */
    var writeHook: (Key, Value) -> ConfigWriteResult? = { _, _ -> null }

    /**
     * When set, every write waits on it before the board answers: an Apply held mid-write, with the
     * caller's one-operation lock held, until the test completes it.
     */
    var writeGate: CompletableDeferred<Unit>? = null

    /** Keys whose reads time out. */
    val unreadable: MutableSet<Key> = mutableSetOf()

    /**
     * Keys the board ANSWERS `CFG_OK` and whose answer does not decode, which a real board
     * produces for a `STR` record whose bytes are not valid UTF-8: it stopped validating them in
     * shrink round 2 and now stores and serves whatever was written, while the mirror's value
     * model carries text (`specs/storage-layer.md`, the record types). A reply with a type tag no
     * `Type` claims is
     * the cheapest way to reach the same `Malformed`.
     */
    val garbledReads: MutableSet<Key> = mutableSetOf()

    /** A type tag no `Type` claims, so `ConfigResp.decodeValue` answers null. */
    private val undecodableTag = 0x7F

    /** Every config write sent, in order, as (target, key, value). */
    val writes: MutableList<Triple<Int, Key, Value>> = mutableListOf()

    /** Every config read sent, in order, as (target, key). */
    val reads: MutableList<Pair<Int, Key>> = mutableListOf()

    /** How many config reads to answer [Busy] first, as when the Tune model holds the request slot. */
    var configBusy: Int = 0

    /** Every config and tune request sent, reads and writes of both lanes, in order, as (target, key). */
    val requests: MutableList<Pair<Int, Key>> = mutableListOf()

    /**
     * When true, every answered tune read leaves a late duplicate of its reply on the link: the reply
     * to a re-send, arriving after the request was answered. Both lanes answer with byte-identical
     * `CONFIG_RESP`s matched by (board, key), and the exchange drains its inbox before each send, so
     * the duplicate is taken as the answer to the NEXT request exactly when that request names the
     * same board and key; any other next request discards it. The exchange's settle window (which a
     * real session has, `ConfigExchange`) is deliberately not modelled: this is the hazard the
     * model's request order alone must survive.
     */
    var duplicateTuneReplies: Boolean = false
    private var duplicate: Pair<Pair<Int, Key>, Int>? = null

    /** The pending duplicate's value if it answers a request of [key] to [target]; consumed either way. */
    private fun takeDuplicate(target: Int, key: Key): Int? {
        requests.add(target to key)
        val d = duplicate ?: return null
        duplicate = null
        return d.second.takeIf { d.first == target to key }
    }

    override suspend fun readConfig(key: Key, target: Int): ConfigReadResult? {
        if (_attachedBoard.value == null) return null
        if (configBusy > 0) {
            configBusy--
            return Busy
        }
        reads.add(target to key)
        takeDuplicate(target, key)?.let { return ReadValue(Value.I16(it)) }
        val v = store[target to key] ?: defaults[key]
        return when {
            key in unreadable -> TimedOut
            key in garbledReads ->
                Malformed(ConfigResp(key.fieldId, key.index, Walk.CFG_OK, undecodableTag, byteArrayOf(1)))
            v == null -> Refused(CfgRefusal.UNKNOWN_KEY)
            else -> ReadValue(v)
        }
    }

    override suspend fun writeConfig(key: Key, value: Value, target: Int): ConfigWriteResult? {
        if (_attachedBoard.value == null) return null
        check(takeDuplicate(target, key) == null) { "a duplicate answering a write is not modelled" }
        writes.add(Triple(target, key, value))
        writeGate?.await()
        writeHook(key, value)?.let { return it }
        if (boardArmed) return Refused(CfgRefusal.ARMED)
        store[target to key] = value
        return WriteVerified(value)
    }

    /**
     * The fake board's RAM gain shadow (what `TUNE_READ` reports), by (target, key). A key it does not
     * hold answers its store default ([stagedDefault]), the way a fresh board's shadow is its
     * store's defaults.
     */
    val shadow: MutableMap<Pair<Int, Key>, Int> = mutableMapOf()

    /** Boards that do not answer the tune lane at all (firmware without it, or out of reach). */
    val tuneSilent: MutableSet<Int> = mutableSetOf()

    /** When set, every tune write is refused with it. */
    var tuneRefusal: CfgRefusal? = null

    /** Every tune write sent, in order, as (target, key, value). */
    val tuneWrites: MutableList<Triple<Int, Key, Int>> = mutableListOf()

    /** Every tune read sent, in order, as (target, key). */
    val tuneReads: MutableList<Pair<Int, Key>> = mutableListOf()

    /** How many tune reads to answer [Busy] first, as when another operation holds the request slot. */
    var tuneBusy: Int = 0

    override suspend fun readTune(key: Key, target: Int): TuneReadResult? {
        if (_attachedBoard.value == null) return null
        if (tuneBusy > 0) {
            tuneBusy--
            return Busy
        }
        tuneReads.add(target to key)
        takeDuplicate(target, key)?.let { return StagedValue(it) }
        val v = shadow[target to key] ?: stagedDefault(key)
        return when {
            target in tuneSilent -> TimedOut
            v == null -> Refused(CfgRefusal.UNKNOWN_KEY)
            else -> {
                if (duplicateTuneReplies) duplicate = (target to key) to v
                StagedValue(v)
            }
        }
    }

    override suspend fun writeTune(key: Key, value: Int, target: Int): TuneWriteResult? {
        if (_attachedBoard.value == null) return null
        check(takeDuplicate(target, key) == null) { "a duplicate answering a write is not modelled" }
        tuneWrites.add(Triple(target, key, value))
        // The board refuses against ITS maxima (`CONTROL_GAIN_MAX` as its store holds them), the way
        // the firmware's shadow does.
        val max = tuneMax(target, key)
        val refusal = tuneRefusal ?: CfgRefusal.BAD.takeIf { !Gains.inRange(key.index, value, max) }
        return when {
            target in tuneSilent -> TimedOut
            refusal != null -> Refused(refusal)
            else -> {
                shadow[target to key] = value
                TuneVerified(value)
            }
        }
    }

    /**
     * What a board that never staged [key] reports on the tune lane: the store's default, which is
     * the compiled profile for a gain and the disabled speed row (0 at every index) for
     * `CONTROL_SPEED_LOOP` (`crates/store/src/field.rs`, `CONTROL_SPEED_LOOP`). Null for a key the
     * lane does not carry, which the board answers unknown.
     */
    private fun stagedDefault(key: Key): Int? =
        if (key.fieldId == Gains.CONTROL_SPEED_LOOP) {
            (Fields.CONTROL_SPEED_LOOP.defaults.getOrNull(key.index) as? Value.I16)?.v
        } else {
            Gains.default(key.fieldId, key.index)
        }

    /**
     * The maximum this board refuses a tune write against: its stored `CONTROL_GAIN_MAX` for a PID
     * gain, and the COMPILED [Gains.SPEED_MAXIMA] for a speed-loop word, which is the split the
     * firmware's shadow makes (`crates/control/src/config.rs`, `row_maxima`).
     */
    private fun tuneMax(target: Int, key: Key): Int? =
        if (key.fieldId == Gains.CONTROL_SPEED_LOOP) {
            Gains.SPEED_MAXIMA.getOrNull(key.index)
        } else {
            (store[target to Key(Fields.CONTROL_GAIN_MAX.id, key.index)] as? Value.I16)?.v
                ?: Gains.DEFAULT_MAX.getOrNull(key.index)
        }

    // --- Test driving helpers ---

    /**
     * Attach to [board] (or detach with null) AND let the scheduler deliver it. [others] are the
     * other nodes the session's walk found, published with it (before it) the way the real
     * transport does.
     */
    fun setAttachedBoard(board: Int?, vararg others: Int?) {
        _nodes.value = if (board == null) emptyList() else (listOf(board) + others.filterNotNull()).distinct().sorted()
        _attachedBoard.value = board
        scheduler.runCurrent()
    }

    /** Make the node at [address] a hoverboard bridge: its `node.class` reads `0x02`. */
    fun makeBridge(address: Int) {
        store[address to Fields.NODE_CLASS.key()] = Value.U8(Fields.NodeClass.BRIDGE)
    }

    /**
     * Move the link to [state] AND let the scheduler deliver it.
     *
     * The advance is the whole point, and it is here rather than left to each caller because
     * forgetting it produces a silent wrong answer rather than a failure. `_connectionState` is a
     * [MutableStateFlow], which CONFLATES AND DEDUPLICATES: a test that sets CONNECTED and then
     * DISCONNECTED with no scheduler turn in between never lets
     * [com.hoverboard.remote.MainViewModel]'s collector observe CONNECTED at all, so when it does
     * resume the value equals the one it last saw and there is NO EMISSION. The force-disarm that
     * a dropped link is supposed to trigger then simply never runs, and the test reads as if the
     * ViewModel ignored the drop.
     *
     * That cost real time once: it looks exactly like a ViewModel bug, and it is not one. Driving
     * the flow through this helper makes every state change observable by construction, so the
     * hazard cannot come back in the next test someone writes.
     *
     * [TestCoroutineScheduler.runCurrent] rather than `advanceUntilIdle`: it is enough to deliver
     * the emission (the collector resumes with no delay) and it cannot fast-forward virtual time
     * through a pending `delay`, which COULD silently collapse windows like
     * [com.hoverboard.remote.MainViewModel.DISARM_SETTLE_MS]. No test today would fail under
     * `advanceUntilIdle` (the one settle-window test sets its connection state after virtual time
     * has already passed the window), so this is a hazard the choice forecloses rather than a
     * failure it currently prevents. It costs nothing to keep it foreclosed.
     */
    fun setConnectionState(state: ConnectionState) {
        // The real transport reports CONNECTED only once a node is attached and the walk is done,
        // so a session that connects without saying to what is attached to [ONE_BOARD].
        if (state == ConnectionState.CONNECTED && _attachedBoard.value == null) {
            _nodes.value = listOf(ONE_BOARD)
            _attachedBoard.value = ONE_BOARD
        }
        _connectionState.value = state
        scheduler.runCurrent()
    }

    /**
     * Inject a `CYCLIC_STATE` as [from] sent it, folded in under that board's address the way the
     * real transport folds it in. Latest-wins per board.
     *
     * [from] defaults to the attached board, which is what a one-board test means; a test about
     * the pair names the other address and gets a second board on the strip, which is what the
     * attached board's forward of its partner's state looks like to the app. No clock: the fake
     * never ages a board out, so a test that wants a board to go silent says so by not emitting
     * and asking the model what it shows.
     */
    fun emitCyclicState(state: CyclicState, from: Int? = null) {
        inbox.cyclic(from ?: _attachedBoard.value ?: UNATTACHED_SRC, state, nowMs = 0)
    }
}

/**
 * The appended observation block of a board that is sitting still (`crates/linkctl/src/lib.rs`,
 * `CyclicObs`): a zeroed current window, the per-boot identity every board reports, and an
 * odometer that has never counted an edge.
 *
 * Every `CYCLIC_STATE` carries this block, so a test saying nothing about the current window still
 * has to say what the board sent. One fixture serves all of them, and a test that cares about the
 * part, the boot tag or the odometer passes its own.
 *
 * The zero odometer is a reading and not an absence: this is a board that has reported, and what
 * it reports is a wheel that has never turned.
 */
internal val QUIET_OBS = CyclicObs(
    phasePeak = 0,
    phaseMean = 0,
    dutyOn = 0,
    bootTag = 1,
    chip = ChipTag.F103C8,
    odometer = 0L,
)

/**
 * The same block from a board that is pulling: a 1,365-count window mean at two thirds duty, which
 * at the 455 counts per amp of [DRIVING_CAL] is 3.0 A at the motor and 2.0 A from the supply.
 *
 * It exists for the screens that are meant to SHOW a current (`specs/rider-ui.md` 3.6): with the
 * quiet block they render the coasting state, which says nothing about how a figure looks.
 */
internal val DRIVING_OBS = QUIET_OBS.copy(
    phasePeak = 2_100,
    phaseMean = 1_365,
    dutyOn = CyclicObs.DUTY_FULL * 2 / 3,
)

/** The counts per amp [DRIVING_OBS]'s figures are quoted at: the registry default. */
internal const val DRIVING_CAL = 455

/**
 * The pack the screens' voltages are quoted against (`board.pack_cells`): ten cells in series, a
 * stock 36 V hoverboard pack, which runs 30.0 V empty to 42.0 V full.
 */
internal const val PACK_CELLS = 10

/** The board a one-board test is attached to, and whose commands [FakeHoverboardTransport.sent] reads. */
internal const val ONE_BOARD = 0x01

/**
 * The address a `CYCLIC_STATE` is filed under when a test emits one before attaching to anything.
 * Such a test cannot be asking about either board, since the app resolves a board's telemetry by
 * the address it learned for it and has learned none.
 */
private const val UNATTACHED_SRC = 0
