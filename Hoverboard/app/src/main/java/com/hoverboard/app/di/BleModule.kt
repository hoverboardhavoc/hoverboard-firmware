package com.hoverboard.app.di

import android.content.Context
import com.hoverboard.app.ble.BleHoverboardTransport
import com.hoverboard.app.ble.HoverboardTransport
import dagger.Module
import dagger.Provides
import dagger.hilt.InstallIn
import dagger.hilt.android.qualifiers.ApplicationContext
import dagger.hilt.components.SingletonComponent
import javax.inject.Singleton

/**
 * App DI graph (Hilt). The real [BleHoverboardTransport] is bound here as the app-wide singleton; a
 * test swaps in a fake [HoverboardTransport] by replacing this module.
 */
@Module
@InstallIn(SingletonComponent::class)
object BleModule {

    @Provides
    @Singleton
    fun provideHoverboardTransport(
        @ApplicationContext context: Context,
    ): HoverboardTransport = BleHoverboardTransport(context)
}
