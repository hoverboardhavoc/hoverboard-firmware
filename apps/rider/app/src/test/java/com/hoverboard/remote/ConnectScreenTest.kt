package com.hoverboard.remote

import android.app.Application
import androidx.compose.ui.test.assertIsDisplayed
import androidx.compose.ui.test.junit4.createComposeRule
import androidx.compose.ui.test.onNodeWithTag
import androidx.compose.ui.test.onNodeWithText
import androidx.compose.ui.test.performClick
import androidx.compose.ui.test.performScrollTo
import androidx.test.core.app.ApplicationProvider
import androidx.test.ext.junit.runners.AndroidJUnit4
import com.hoverboard.remote.ble.ScanCandidate
import com.hoverboard.remote.ble.SelectedBoard
import com.hoverboard.remote.model.ConnectionState
import com.hoverboard.remote.ui.screens.ConnectScreen
import com.hoverboard.remote.ui.screens.FONT_LICENCE_TAG
import com.hoverboard.remote.ui.theme.HoverboardRemoteTheme
import org.junit.Assert.assertEquals
import org.junit.Rule
import org.junit.Test
import org.junit.rules.RuleChain
import org.junit.runner.RunWith
import org.robolectric.annotation.Config

/**
 * What the connect screen says while the L3 attach is running and after it fails, and what the
 * board picker puts on screen.
 *
 * The attach states are reachable on every connect (the attach runs before anything is drivable),
 * and both were rendered by no test at all: their strings existed and were never shown to
 * anything. A state that falls through to another state's label, or a failure that renders as the
 * idle screen, is a silent connect that leaves the rider with no diagnosis.
 *
 * The picker cases pin the three things a row has to carry, because a row missing any of them is
 * a row the user cannot tell from the board next to it.
 */
@RunWith(AndroidJUnit4::class)
@Config(sdk = [34], application = Application::class, qualifiers = "w411dp-h891dp-xhdpi")
class ConnectScreenTest {

    private val compose = createComposeRule()

    @get:Rule
    val rules: RuleChain = composeHost(compose)

    private val context: Application = ApplicationProvider.getApplicationContext()

    @Test
    fun attachingSaysSoAndOffersToStop() {
        show(ConnectionState.ATTACHING)

        compose.onNodeWithText(context.getString(R.string.connect_status_attaching)).assertIsDisplayed()
        // Attaching is a busy state, not an idle one: the action is to stop, not to scan again.
        compose.onNodeWithText(context.getString(R.string.connect_stop_scan)).assertIsDisplayed()
    }

    @Test
    fun attachFailedSaysWhyAndOffersToScanAgain() {
        show(ConnectionState.ATTACH_FAILED)

        compose.onNodeWithText(context.getString(R.string.connect_status_attach_failed)).assertIsDisplayed()
        // Terminal for this attempt: the app stopped retrying, so the user gets the button back.
        compose.onNodeWithText(context.getString(R.string.connect_scan, DEVICE_NAME)).assertIsDisplayed()
    }

    @Test
    fun aFoundBoardShowsItsNameAddressAndSignal() {
        show(
            ConnectionState.DISCONNECTED,
            findingBoards = true,
            boardsSeen = listOf(candidate("AA:BB:CC:DD:EE:FF", "hb-offroad-m", rssi = -62)),
        )

        compose.onNodeWithText("hb-offroad-m").assertIsDisplayed()
        compose.onNodeWithText("AA:BB:CC:DD:EE:FF").assertIsDisplayed()
        compose.onNodeWithText(context.getString(R.string.connect_rssi, -62)).assertIsDisplayed()
    }

    @Test
    fun aBoardAdvertisingNoNameStillShowsAsSomethingTappable() {
        // Its address is the only thing it has said about itself, and it is the thing the app
        // stores, so the row is still a usable pick rather than a blank line.
        show(
            ConnectionState.DISCONNECTED,
            findingBoards = true,
            boardsSeen = listOf(candidate("11:22:33:44:55:66", null, rssi = null)),
        )

        compose.onNodeWithText(context.getString(R.string.connect_unnamed_board)).assertIsDisplayed()
        compose.onNodeWithText("11:22:33:44:55:66").assertIsDisplayed()
    }

    @Test
    fun tappingARowSelectsThatBoard() {
        val rows = listOf(
            candidate("11:11:11:11:11:11", "Hoverboard", rssi = -40),
            candidate("22:22:22:22:22:22", "Hoverboard", rssi = -70),
        )
        var picked: ScanCandidate? = null
        show(
            ConnectionState.DISCONNECTED,
            findingBoards = true,
            boardsSeen = rows,
            onSelectBoard = { picked = it },
        )

        // Two boards with the SAME advertised name: the row is picked by its address, which is the
        // case the whole picker exists for.
        compose.onNodeWithText("22:22:22:22:22:22").performClick()

        assertEquals(rows[1], picked)
    }

    @Test
    fun aPickedBoardIsWhatTheConnectButtonOffers() {
        show(
            ConnectionState.DISCONNECTED,
            selectedBoard = SelectedBoard("AA:BB:CC:DD:EE:FF", "hb-offroad-m"),
        )

        // Not "Scan for Hoverboard": the app is pointed at one board now, and says which.
        compose.onNodeWithText(context.getString(R.string.connect_connect_to, "hb-offroad-m"))
            .assertIsDisplayed()
    }

    @Test
    fun aPickedBoardWithNoNameIsNamedByItsAddress() {
        show(
            ConnectionState.DISCONNECTED,
            selectedBoard = SelectedBoard("AA:BB:CC:DD:EE:FF", null),
        )

        compose.onNodeWithText(context.getString(R.string.connect_connect_to, "AA:BB:CC:DD:EE:FF"))
            .assertIsDisplayed()
    }

    private fun candidate(address: String, name: String?, rssi: Int?) = ScanCandidate(
        address = address,
        advertisedName = name,
        cachedName = null,
        rssi = rssi,
    )

    /**
     * The font notice at the foot of the screen opens the licence, read from the asset the fonts
     * ship with (`specs/rider-ui.md` 3.10), and Close dismisses it.
     */
    @Test
    fun theFontLicenceLineOpensTheLicence() {
        show(ConnectionState.DISCONNECTED)
        val firstLine = context.assets.open("fonts/OFL.txt").bufferedReader().use { it.readLine() }

        compose.onNodeWithTag(FONT_LICENCE_TAG).performScrollTo().performClick()

        compose.onNodeWithText(context.getString(R.string.font_licence_title)).assertIsDisplayed()
        compose.onNodeWithText(firstLine, substring = true).assertExists()
        compose.onNodeWithText(context.getString(R.string.font_licence_close)).performClick()
        compose.onNodeWithText(context.getString(R.string.font_licence_title)).assertDoesNotExist()
    }

    private fun show(
        state: ConnectionState,

        selectedBoard: SelectedBoard? = null,
        findingBoards: Boolean = false,
        boardsSeen: List<ScanCandidate> = emptyList(),
        onSelectBoard: (ScanCandidate) -> Unit = {},
    ) = compose.setContent {
        HoverboardRemoteTheme {
            ConnectScreen(
                connectionState = state,
                deviceName = DEVICE_NAME,
                selectedBoard = selectedBoard,
                findingBoards = findingBoards,
                boardsSeen = boardsSeen,
                onFindBoards = {},
                onStopFindingBoards = {},
                onSelectBoard = onSelectBoard,
                onDeviceNameChange = {},
                onConnect = {},
                onDisconnect = {},
            )
        }
    }

    private companion object {
        const val DEVICE_NAME = "Hoverboard"
    }
}
