package com.hoverboard.bletest

import android.app.Activity
import android.os.Build
import android.os.Bundle
import android.util.Log
import com.hoverboard.bletest.codec.ConnPriority
import com.hoverboard.bletest.codec.ResultJson
import com.hoverboard.bletest.codec.RunConfig
import com.hoverboard.bletest.codec.ThroughputRun
import com.hoverboard.bletest.codec.WriteType
import com.hoverboard.bletest.transport.BleTransport
import com.hoverboard.bletest.transport.LoopbackTransport
import com.hoverboard.bletest.transport.Transport
import java.io.File
import kotlin.concurrent.thread

/**
 * Headless intent-driven runner (`specs/ble.md`, "Drive mode"). Launched by the host sweep script via
 * `am start -n com.hoverboard.bletest/.RunnerActivity --es device ... --ei payload ...`, it runs ONE
 * fixed-config throughput run, emits the result on the `BLE_TPUT` logcat tag (the primary machine-readable
 * channel), writes the JSON to the app's external files dir, then prints a `DONE` line the sweep waits on.
 *
 * No UI, no bonding (so no pairing dialog and no UI Automator). The work runs off the main thread.
 */
class RunnerActivity : Activity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        // Keep the screen on while the run is in flight. On OxygenOS (and other aggressive OEM power
        // managers) the process is FROZEN on screen-off ("Hans ... freeze ... scene: LcdOff"), which
        // halts the BLE work thread mid-run. Holding the screen on while this foreground activity is
        // visible prevents that for the (short) duration of a run.
        window.addFlags(android.view.WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        val cfg = parseIntent()
        thread {
            try {
                runOne(cfg)
            } catch (t: Throwable) {
                Log.e(TAG, "run failed: ${t.message}", t)
                Log.i(TAG, "DONE status=error device=${cfg.device}")
            } finally {
                finish()
            }
        }
    }

    private fun parseIntent(): RunConfig {
        val e = intent
        return RunConfig(
            device = e.getStringExtra("device") ?: "OnePlus",
            name = e.getStringExtra("name") ?: BLE_NAME,
            mode = e.getStringExtra("mode") ?: "loopback",
            payload = e.getIntExtra("payload", 64),
            rate = e.getIntExtra("rate", 50),
            durSec = e.getIntExtra("dur", 10),
            write = WriteType.parse(e.getStringExtra("write")),
            mtu = e.getIntExtra("mtu", 247),
            priority = ConnPriority.parse(e.getStringExtra("prio")),
            out = e.getStringExtra("out") ?: "run.json",
        )
    }

    private fun runOne(cfg: RunConfig) {
        // Confirm we are on the phone the host meant (resolved by model, not address). A mismatch is a
        // bench wiring error worth catching loudly.
        val dev = Devices.byName(cfg.device)
        if (dev != null && dev.model != Build.MODEL) {
            Log.w(TAG, "device '${cfg.device}' expects model ${dev.model} but running on ${Build.MODEL}")
        }

        val transport: Transport = when (cfg.mode) {
            "fake" -> LoopbackTransport()
            else -> BleTransport(
                context = this,
                deviceName = cfg.name,
                autoConnect = dev?.autoConnect ?: false,
                requestMtu = cfg.mtu,
                writeWithResponse = cfg.write == WriteType.RESPONSE,
            )
        }

        val run = ThroughputRun(
            transport = transport,
            nowNanos = { System.nanoTime() },
            sleepUntil = { target ->
                val now = System.nanoTime()
                if (target > now) {
                    val ms = (target - now) / 1_000_000
                    val ns = ((target - now) % 1_000_000).toInt()
                    Thread.sleep(ms, ns)
                }
            },
        )
        val result = run.run(cfg)
        val json = ResultJson.encode(cfg, result)

        Log.i(TAG, "RESULT $json")
        try {
            val dir = getExternalFilesDir(null)
            File(dir, cfg.out).writeText(json)
        } catch (t: Throwable) {
            Log.w(TAG, "could not write ${cfg.out}: ${t.message}")
        }
        Log.i(TAG, "DONE status=ok device=${cfg.device} payload=${cfg.payload} rate=${cfg.rate} " +
            "loss=${result.lost}/${result.sent} corrupted=${result.corrupted}")
    }

    companion object {
        const val TAG = "BLE_TPUT"
        /**
         * The advertised name the board's `AT+NAME` sets and the central scans by (the loopback firmware).
         * Kept short (<=10 chars): the CC2541 silently won't advertise an over-long name, so this must
         * match the loopback firmware's `Module::new("hbloop")`.
         */
        const val BLE_NAME = "hbloop"
    }
}
