package com.hoverboard.stress.run

/**
 * One stress run's parameters, parsed from the launch-intent extras by [com.hoverboard.stress.RunnerActivity].
 *
 * @param name advertised BLE name to scan for (`AT+NAME`, default `hb-stress`).
 * @param mode `roundtrip` (Slice 2) or `sustained` (Slice 3).
 * @param n number of frames to bounce when [durSec] <= 0.
 * @param durSec if > 0, run for this many seconds instead of a fixed [n].
 * @param chunk L2 chunk size in bytes (4-byte BE seq + filler); the L2 frame is `1 + chunk` and the
 *   wire frame `SOF + len + (1+chunk) + CRC`. The default 15 makes a 16-byte L2 frame = the firmware's
 *   `BLE_FRAME_CAP`, i.e. exactly one 20-byte ATT write.
 * @param rate cap on offered frames/sec (0 = as fast as round-trips complete).
 * @param priorityHigh request `CONNECTION_PRIORITY_HIGH` after connect.
 * @param out result-JSON filename in the app's external files dir.
 */
data class RunConfig(
    val name: String,
    val mode: String,
    val n: Int,
    val durSec: Int,
    val chunk: Int,
    val rate: Int,
    val priorityHigh: Boolean,
    val writeWithResponse: Boolean,
    val out: String,
)
