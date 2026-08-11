package com.hoverboard.remote.di

import com.hoverboard.remote.MainViewModel
import com.hoverboard.remote.ble.BleHoverboardTransport
import com.hoverboard.remote.ble.HoverboardTransport
import com.hoverboard.remote.ble.LinkSettings
import org.koin.android.ext.koin.androidContext
import org.koin.core.module.dsl.viewModel
import org.koin.dsl.module

/**
 * App DI graph. The real [BleHoverboardTransport] is bound here; tests inject a fake
 * [HoverboardTransport] instead (SPEC §12.2 layer 1).
 *
 * [LinkSettings] is a singleton on purpose: the transport reads the target name from it and the
 * ViewModel writes it, and two instances would mean the UI showing one name while the scan looks
 * for another.
 */
val appModule = module {
    single { LinkSettings(androidContext()) }
    single<HoverboardTransport> { BleHoverboardTransport(androidContext(), get()) }
    viewModel { MainViewModel(get(), get()) }
}
