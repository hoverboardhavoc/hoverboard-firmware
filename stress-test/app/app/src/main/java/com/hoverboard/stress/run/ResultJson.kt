package com.hoverboard.stress.run

/**
 * Minimal hand-rolled JSON for the run result (no JSON-library dep). Echoed on the `BLE_STRESS` logcat
 * tag (the machine-readable channel the host run script greps) and written to the app's external files
 * dir for `adb pull`.
 */
object ResultJson {
    fun encode(cfg: RunConfig, r: RoundTripResult): String = buildString {
        append("{")
        append("\"name\":\"").append(cfg.name).append("\",")
        append("\"mode\":\"").append(cfg.mode).append("\",")
        append("\"chunk\":").append(cfg.chunk).append(",")
        append("\"requested\":").append(if (cfg.durSec > 0) "\"${cfg.durSec}s\"" else cfg.n.toString()).append(",")
        append("\"rate\":").append(cfg.rate).append(",")
        append("\"connprio\":\"").append(cfg.connPriority).append("\",")
        append("\"bond\":").append(cfg.bond).append(",")
        append("\"autoconnect\":").append(cfg.autoConnect).append(",")
        append("\"sent\":").append(r.sent).append(",")
        append("\"echoed\":").append(r.echoed).append(",")
        append("\"lost\":").append(r.lost).append(",")
        append("\"loss_fraction\":").append(fmt(r.lossFraction)).append(",")
        append("\"dropped\":").append(r.dropped).append(",")
        append("\"frames_before_drop\":").append(r.framesBeforeDrop).append(",")
        append("\"connected_ms\":").append(r.connectedMs).append(",")
        append("\"duration_ms\":").append(r.durationMs).append(",")
        append("\"throughput_fps\":").append(fmt(r.throughputFps())).append(",")
        append("\"rtt_ns\":{")
        append("\"count\":").append(r.rttCount).append(",")
        append("\"min\":").append(r.rttMinNs).append(",")
        append("\"mean\":").append(r.rttMeanNs).append(",")
        append("\"max\":").append(r.rttMaxNs)
        append("}")
        append("}")
    }

    private fun fmt(d: Double): String = String.format("%.4f", d)
}
