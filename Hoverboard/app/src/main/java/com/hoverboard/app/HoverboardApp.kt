package com.hoverboard.app

import android.app.Application
import dagger.hilt.android.HiltAndroidApp

/** Hilt application root: triggers component generation and hosts the app-wide DI graph. */
@HiltAndroidApp
class HoverboardApp : Application()
