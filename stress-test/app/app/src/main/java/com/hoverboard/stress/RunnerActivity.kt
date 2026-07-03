package com.hoverboard.stress

import android.app.Activity
import android.os.Build
import android.os.Bundle
import android.util.Log
import android.view.WindowManager
import com.hoverboard.stress.ble.BleStressTransport
import com.hoverboard.stress.ble.LinkConfig
import com.hoverboard.stress.run.ResultJson
import com.hoverboard.stress.run.RoundTripRunner
import com.hoverboard.stress.run.RunConfig
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.runBlocking
import java.io.File
import kotlin.concurrent.thread

/**
 * Headless intent-driven runner for the BLE link stress test (spec "Android app"). Launched by
 * `run-roundtrip.sh` via `am start -n com.hoverboard.stress/.RunnerActivity --es mode roundtrip ...`,
 * it runs ONE round-trip stress run over the production Nordic transport, emits the metrics on the
 * `BLE_STRESS` logcat tag (`RESULT {json}` then `DONE ...`), and writes the JSON to the app's external
 * files dir. No real UI; the work runs off the main thread.
 *
 * Slice 3 adds the `sustained` mode here; this slice handles `roundtrip` only.
 */
class RunnerActivity : Activity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        // OxygenOS freezes a screen-off process ("Hans ... freeze ... LcdOff"), which would halt the BLE
        // work thread mid-run. Hold the screen on while this foreground activity is visible.
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)

        val cfg = parseIntent()
        thread {
            try {
                runOne(cfg)
            } catch (t: Throwable) {
                Log.e(TAG, "run failed: ${t.message}", t)
                Log.i(TAG, "DONE status=error mode=${cfg.mode}")
            } finally {
                finish()
            }
        }
    }

    private fun parseIntent(): RunConfig {
        val e = intent
        return RunConfig(
            name = e.getStringExtra("name") ?: LinkConfig.DEFAULT_DEVICE_NAME,
            mode = e.getStringExtra("mode") ?: "roundtrip",
            n = e.getIntExtra("n", 200),
            durSec = e.getIntExtra("dur", 0),
            chunk = e.getIntExtra("chunk", DEFAULT_CHUNK),
            rate = e.getIntExtra("rate", 0),
            connPriority = e.getStringExtra("connprio") ?: "none",
            writeWithResponse = (e.getStringExtra("write") ?: "nores") == "res",
            bond = e.getBooleanExtra("bond", false),
            autoConnect = e.getBooleanExtra("autoconnect", true),
            out = e.getStringExtra("out") ?: "run.json",
        )
    }

    private fun runOne(cfg: RunConfig) {
        Log.i(TAG, "START model=${Build.MODEL} cfg=$cfg")
        if (cfg.mode != "roundtrip") {
            // sustained is Slice 3.
            Log.i(TAG, "DONE status=skip reason=mode_${cfg.mode}_not_implemented_in_slice2")
            return
        }
        if (cfg.chunk !in 5..15) {
            Log.i(TAG, "DONE status=error reason=chunk_${cfg.chunk}_out_of_5..15")
            return
        }

        val transport = BleStressTransport(
            context = this,
            config = LinkConfig(deviceName = cfg.name),
            connPriority = cfg.connPriority,
            writeWithResponse = cfg.writeWithResponse,
            bond = cfg.bond,
            autoConnect = cfg.autoConnect,
        )
        try {
            transport.connect()
            runBlocking {
                val pipe = transport.awaitPipe(CONNECT_BUDGET_MS)
                if (pipe == null) {
                    Log.i(TAG, "DONE status=no_connect name=${cfg.name}")
                    return@runBlocking
                }
                val result = coroutineScope { RoundTripRunner(transport, cfg).run(pipe, this) }
                val json = ResultJson.encode(cfg, result)
                Log.i(TAG, "RESULT $json")
                try {
                    File(getExternalFilesDir(null), cfg.out).writeText(json)
                } catch (t: Throwable) {
                    Log.w(TAG, "could not write ${cfg.out}: ${t.message}")
                }
                Log.i(
                    TAG,
                    "DONE status=ok mode=roundtrip sent=${result.sent} echoed=${result.echoed} " +
                        "lost=${result.lost} loss=${"%.2f".format(result.lossFraction * 100)}% " +
                        "dropped=${result.dropped} frames_before_drop=${result.framesBeforeDrop} " +
                        "connected_ms=${result.connectedMs} " +
                        "rtt_ms[min/mean/max]=${ms(result.rttMinNs)}/${ms(result.rttMeanNs)}/${ms(result.rttMaxNs)} " +
                        "tput_fps=${"%.1f".format(result.throughputFps())}",
                )
            }
        } finally {
            transport.shutdown()
        }
    }

    private fun ms(ns: Long): String = "%.1f".format(ns / 1_000_000.0)

    private companion object {
        const val TAG = "BLE_STRESS"

        /** Default L2 chunk: 4-byte seq + 11 filler = 15 -> 16-byte L2 frame -> one 20-byte ATT write. */
        const val DEFAULT_CHUNK = 15

        /** Budget to scan + connect + discover + subscribe before giving up. The ASUS ROG's autoConnect
         *  can take ~133 s on Android 8, so this is generous. */
        const val CONNECT_BUDGET_MS = 180_000L
    }
}
