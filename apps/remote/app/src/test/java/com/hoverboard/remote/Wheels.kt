package com.hoverboard.remote

import com.hoverboard.remote.model.Wheel

/** Motor 0 of each board at [nodes]: the wheels of a machine of one-motor boards. */
internal fun wheels(vararg nodes: Int): List<Wheel> = nodes.map { Wheel(it, 0) }

/** Motor 0 of the board at [node]. */
internal fun wheel0(node: Int): Wheel = Wheel(node, 0)
