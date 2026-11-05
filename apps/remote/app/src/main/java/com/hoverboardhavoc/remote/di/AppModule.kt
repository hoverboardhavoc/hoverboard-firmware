package com.hoverboardhavoc.remote.di

import com.hoverboardhavoc.remote.MainViewModel
import com.hoverboardhavoc.remote.ble.BleBoardScanner
import com.hoverboardhavoc.remote.ble.BleHoverboardTransport
import com.hoverboardhavoc.remote.ble.BoardScanner
import com.hoverboardhavoc.remote.ble.HoverboardTransport
import com.hoverboardhavoc.remote.ble.LinkSettings
import org.koin.android.ext.koin.androidContext
import org.koin.core.module.dsl.viewModel
import org.koin.dsl.module

/**
 * App DI graph. The real [BleHoverboardTransport] is bound here; tests inject a fake
 * [HoverboardTransport] instead (SPEC §12.2 layer 1).
 *
 * [LinkSettings] is a singleton on purpose: the transport reads the target from it and the
 * ViewModel writes it, and two instances would mean the Connect screen showing one board while
 * the scan looks for another.
 *
 * [BoardScanner] is the Connect screen's own scan, the one that lists what is nearby rather than
 * reaching a board; it is bound separately from the transport because the two scans answer
 * different questions (see the interface).
 */
val appModule = module {
    single { LinkSettings(androidContext()) }
    single<HoverboardTransport> { BleHoverboardTransport(androidContext(), get()) }
    single<BoardScanner> { BleBoardScanner(androidContext()) }
    viewModel { MainViewModel(get(), get(), get()) }
}
