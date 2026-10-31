package com.hoverboard.remote

import android.app.Application
import com.hoverboard.remote.di.appModule
import org.koin.android.ext.koin.androidContext
import org.koin.core.context.startKoin

class HoverboardApp : Application() {
    override fun onCreate() {
        super.onCreate()
        startKoin {
            androidContext(this@HoverboardApp)
            modules(appModule)
        }
    }
}
