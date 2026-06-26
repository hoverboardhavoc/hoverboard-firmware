package com.hoverboard.bletest.codec

/**
 * Minimal hand-rolled JSON for the run result (no JSON library dep, so the codec module stays plain and
 * JVM-testable). Written to the app's external files dir and pulled with `adb pull` (`specs/ble.md`,
 * "Drive mode"). The same string is echoed on the BLE_TPUT logcat tag as the machine-readable channel.
 */
object ResultJson {
    fun encode(config: RunConfig, r: RunResult): String {
        val l = r.latencyNs
        val goodput = r.goodputBytesPerSec(config.payload, config.durSec.toDouble())
        return buildString {
            append("{")
            append("\"device\":\"").append(config.device).append("\",")
            append("\"mode\":\"").append(config.mode).append("\",")
            append("\"payload\":").append(config.payload).append(",")
            append("\"rate\":").append(config.rate).append(",")
            append("\"dur\":").append(config.durSec).append(",")
            append("\"write\":\"").append(config.write).append("\",")
            append("\"mtu\":").append(config.mtu).append(",")
            append("\"prio\":\"").append(config.priority).append("\",")
            append("\"sent\":").append(r.sent).append(",")
            append("\"delivered\":").append(r.delivered).append(",")
            append("\"lost\":").append(r.lost).append(",")
            append("\"loss_fraction\":").append(fmt(r.lossFraction)).append(",")
            append("\"corrupted\":").append(r.corrupted).append(",")
            append("\"resyncs\":").append(r.resyncs).append(",")
            append("\"goodput_bps\":").append(fmt(goodput)).append(",")
            append("\"latency_ns\":{")
            append("\"count\":").append(l.count).append(",")
            append("\"min\":").append(l.minNs).append(",")
            append("\"mean\":").append(l.meanNs).append(",")
            append("\"p50\":").append(l.p50Ns).append(",")
            append("\"p95\":").append(l.p95Ns).append(",")
            append("\"max\":").append(l.maxNs)
            append("}")
            append("}")
        }
    }

    private fun fmt(d: Double): String = String.format("%.4f", d)
}
