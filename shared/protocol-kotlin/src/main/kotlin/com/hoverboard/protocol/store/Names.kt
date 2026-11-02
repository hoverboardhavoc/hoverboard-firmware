package com.hoverboard.protocol.store

/**
 * The store's name table (`NAMES` in `shared/store/src/field.rs`; specs/cli-config.md
 * section 2): the path each field goes by in a settings file and on the CLI's command line, how it
 * is indexed, and how a file writes its value. A configuration check names the key it refuses by
 * this path, as the CLI does. `RustSourceDriftTest` pins every row against the Rust, both ways.
 */
object Names {
    /** How a named field is indexed (`store::Index`): one value, an array, one per motor, or one per link. */
    enum class IndexKind { SINGLE, ARRAY, MOTOR, LINK }

    /** How a file writes a field's value (`store::Kind`). */
    enum class Kind { PIN, INT, BOOL, ENUM, BITS, STR }

    /**
     * One row: field [id] at [path], indexed as [index], its value written as [kind]. [labels] are
     * an [Kind.ENUM]'s labels by byte, or a [Kind.BITS]'s bit names by bit number.
     */
    data class Name(
        val id: Int,
        val path: String,
        val index: IndexKind,
        val kind: Kind,
        val labels: List<Pair<Int, String>> = emptyList(),
    )

    /** The object arrays a file holds, by index kind (`store::Index::array`). */
    fun arrayOf(index: IndexKind): String? = when (index) {
        IndexKind.MOTOR -> "motors"
        IndexKind.LINK -> "links"
        IndexKind.SINGLE, IndexKind.ARRAY -> null
    }

    private val METHOD = listOf(0 to "six_step", 1 to "sine", 2 to "foc")
    private val MODE = listOf(0 to "throttle", 1 to "balance")
    private val FRAME = listOf(0 to "pivoting", 1 to "rigid")
    private val SIDE = listOf(0 to "left", 1 to "right")
    private val DIRECTION = listOf(0 to "forward", 1 to "reverse")
    private val AXIS = listOf(1 to "x", 2 to "y", 3 to "z")
    private val CHEMISTRY = listOf(0 to "li_ion", 1 to "lev50", 2 to "lto")
    private val ACTIVE_HIGH = listOf(0 to "power.button", 1 to "pads.a", 2 to "pads.b")

    private fun single(f: FieldDef, path: String, kind: Kind = Kind.INT, labels: List<Pair<Int, String>> = emptyList()) =
        Name(f.id, path, IndexKind.SINGLE, kind, labels)

    private fun motor(f: FieldDef, path: String, kind: Kind = Kind.INT, labels: List<Pair<Int, String>> = emptyList()) =
        Name(f.id, path, IndexKind.MOTOR, kind, labels)

    private fun array(f: IndexedFieldDef, path: String, kind: Kind = Kind.INT, labels: List<Pair<Int, String>> = emptyList()) =
        Name(f.id, path, IndexKind.ARRAY, kind, labels)

    private fun link(id: Int, path: String, kind: Kind) = Name(id, path, IndexKind.LINK, kind)

    private fun pin(f: FieldDef, path: String, index: IndexKind = IndexKind.SINGLE) = Name(f.id, path, index, Kind.PIN)

    /** Every row, in the Rust table's order. */
    val ALL: List<Name> = listOf(
        // Node.
        single(Fields.NODE_ADDRESS, "node.address"),
        single(Fields.DEVICE_NAME, "device.name", Kind.STR),
        single(Fields.WIFI_SSID, "wifi.ssid", Kind.STR),
        single(Fields.WIFI_PASS, "wifi.pass", Kind.STR),
        single(Fields.WIFI_TX_POWER, "wifi.tx_power"),
        pin(Fields.BOARD_SELF_HOLD, "power.self_hold"),
        pin(Fields.BOARD_BATTERY_PIN, "battery_sense.pin"),
        pin(Fields.BOARD_BUZZER, "buzzer"),
        pin(Fields.LED_GREEN, "leds.green"),
        pin(Fields.LED_ORANGE, "leds.orange"),
        pin(Fields.LED_RED, "leds.red"),
        pin(Fields.PAD_A, "pads.a"),
        pin(Fields.PAD_B, "pads.b"),
        pin(Fields.BOARD_BUTTON, "power.button"),
        pin(Fields.LINK_TX, "tx", IndexKind.LINK),
        pin(Fields.LINK_RX, "rx", IndexKind.LINK),
        pin(Fields.IMU_SCL_PIN, "imu.scl"),
        pin(Fields.IMU_SDA_PIN, "imu.sda"),
        single(Fields.IMU_I2C, "imu.i2c"),
        array(Fields.IMU_GYRO_BIAS, "imu.gyro_bias"),
        array(Fields.IMU_AXIS_SIGN, "imu.axis_sign"),
        array(Fields.IMU_AXIS_ROLE, "imu.axis_role", Kind.ENUM, AXIS),
        array(Fields.ATTITUDE_LEVEL_TRIM, "attitude.level_trim"),
        array(Fields.BOARD_BATTERY_CAL, "battery_sense.cal"),
        single(Fields.BOARD_INPUT_POLARITY, "active_high", Kind.BITS, ACTIVE_HIGH),
        single(Fields.IMU_REMAP, "imu.remap"),
        link(Fields.LINK_USART.id, "usart", Kind.INT),
        link(Fields.LINK_REMAP.id, "remap", Kind.INT),
        // Machine.
        single(Fields.CONTROL_MODE, "machine.mode", Kind.ENUM, MODE),
        single(Fields.CONTROL_FRAME, "machine.frame", Kind.ENUM, FRAME),
        single(Fields.CONTROL_RIDER_REQUIRED, "machine.rider_required", Kind.BOOL),
        single(Fields.MOTOR_METHOD, "machine.method", Kind.ENUM, METHOD),
        single(Fields.BOARD_BATTERY_CELLS, "machine.battery.cells"),
        single(Fields.BOARD_BATTERY_CHEMISTRY, "machine.battery.chemistry", Kind.ENUM, CHEMISTRY),
        single(Fields.BOARD_BATTERY_FLOOR, "machine.battery.floor"),
        array(Fields.CONTROL_SF_GAINS, "machine.balance.gains"),
        array(Fields.CONTROL_GAIN_MAX, "machine.balance.gain_max"),
        array(Fields.CONTROL_REFERENCE, "machine.balance.reference"),
        single(Fields.CONTROL_POS_CLAMP, "machine.balance.pos_clamp"),
        array(Fields.CONTROL_DRIVE_TURN, "machine.turn"),
        // Wheel.
        pin(Fields.MOTOR_HALL_A, "hall.a", IndexKind.MOTOR),
        pin(Fields.MOTOR_HALL_B, "hall.b", IndexKind.MOTOR),
        pin(Fields.MOTOR_HALL_C, "hall.c", IndexKind.MOTOR),
        pin(Fields.MOTOR_GATE_HI_A, "gate.hi_a", IndexKind.MOTOR),
        pin(Fields.MOTOR_GATE_HI_B, "gate.hi_b", IndexKind.MOTOR),
        pin(Fields.MOTOR_GATE_HI_C, "gate.hi_c", IndexKind.MOTOR),
        pin(Fields.MOTOR_GATE_LO_A, "gate.lo_a", IndexKind.MOTOR),
        pin(Fields.MOTOR_GATE_LO_B, "gate.lo_b", IndexKind.MOTOR),
        pin(Fields.MOTOR_GATE_LO_C, "gate.lo_c", IndexKind.MOTOR),
        pin(Fields.MOTOR_PHASE_A, "current.phase_a", IndexKind.MOTOR),
        pin(Fields.MOTOR_PHASE_B, "current.phase_b", IndexKind.MOTOR),
        motor(Fields.MOTOR_DEAD_TIME, "gate.dead_time"),
        motor(Fields.MOTOR_CURRENT_CAL, "current.cal"),
        motor(Fields.MOTOR_NOISE_FLOOR, "current.noise_floor"),
        motor(Fields.MOTOR_CURRENT_LIMIT, "current.limit"),
        motor(Fields.MOTOR_DIRECTION, "direction", Kind.ENUM, DIRECTION),
        motor(Fields.CONTROL_SIDE, "side", Kind.ENUM, SIDE),
        motor(Fields.MOTOR_ALIGN_OFFSET, "align_offset"),
        motor(Fields.MOTOR_POLE_PAIRS, "pole_pairs"),
        motor(Fields.WHEEL_DIAMETER_MM, "wheel_diameter_mm"),
        motor(Fields.MOTOR_ODOMETER, "odometer"),
        motor(Fields.MOTOR_GATE_TIMER, "gate.timer"),
        motor(Fields.MOTOR_GATE_REMAP, "gate.remap"),
    )

    /** The row for field [id], or null when the table names none. */
    fun byId(id: Int): Name? = ALL.firstOrNull { it.id == id }

    /** The row named [path] among the fields of the object array [array] (`motors`, `links`), or among the others with null. */
    fun find(path: String, array: String?): Name? = ALL.firstOrNull { it.path == path && arrayOf(it.index) == array }

    /**
     * The path [key] prints as, the CLI's `path_of`: `machine.mode`, `battery_sense.cal[1]`,
     * `motors[0].hall.a`, `links[0].tx`, or the raw `0x99.0` for an id the table does not name.
     */
    fun pathOf(key: Key): String {
        val r = byId(key.fieldId) ?: return "0x%02x.%d".format(key.fieldId, key.index)
        return when (r.index) {
            IndexKind.MOTOR, IndexKind.LINK -> "${arrayOf(r.index)}[${key.index}].${r.path}"
            IndexKind.SINGLE -> if (key.index == 0) r.path else "${r.path}[${key.index}]"
            IndexKind.ARRAY -> "${r.path}[${key.index}]"
        }
    }
}
