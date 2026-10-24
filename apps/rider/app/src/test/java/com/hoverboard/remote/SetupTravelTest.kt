package com.hoverboard.remote

import android.app.Application
import androidx.compose.ui.test.junit4.createComposeRule
import androidx.compose.ui.test.onNodeWithTag
import androidx.compose.ui.test.onNodeWithText
import androidx.test.core.app.ApplicationProvider
import androidx.test.ext.junit.runners.AndroidJUnit4
import com.hoverboard.protocol.store.Key
import com.hoverboard.protocol.store.Value
import com.hoverboard.remote.model.Node
import com.hoverboard.remote.model.OdometerReading
import com.hoverboard.remote.model.SetupFields
import com.hoverboard.remote.ui.screens.SetupScreen
import com.hoverboard.remote.ui.screens.TRAVEL_SECTION_TAG
import com.hoverboard.remote.ui.theme.HoverboardRemoteTheme
import org.junit.Rule
import org.junit.Test
import org.junit.rules.RuleChain
import org.junit.runner.RunWith
import org.robolectric.annotation.Config

/**
 * What the Setup screen's Travel section says (`specs/rider-ui.md` 3.6c).
 *
 * Its own class rather than a block of [SetupScreenTest], which is at detekt's class-size bar:
 * the section has three row states, two units and two notes, and the cases that pin them belong
 * together.
 */
@RunWith(AndroidJUnit4::class)
@Config(sdk = [34], application = Application::class, qualifiers = "w411dp-h891dp-xhdpi")
class SetupTravelTest {

    private val compose = createComposeRule()

    @get:Rule
    val rules: RuleChain = composeHost(compose)

    private val context: Application = ApplicationProvider.getApplicationContext()
    private fun s(id: Int, vararg args: Any) = context.getString(id, *args)

    /** Every Setup row at its default, so the screen draws whole and the Travel block sits in it. */
    private val stored: Map<Key, Value> = SetupFields.ALL.associate { it.key to it.def.default }

    /** The attached board in the master role, with [other] as the pair's second board if given. */
    private fun setupState(board: Int, other: Int? = null) = SetupState(
        target = Node.MASTER,
        master = board,
        slave = other,
        boards = mapOf(Node.MASTER to BoardSetup(values = stored)),
    )

    private fun show(
        state: SetupState,
        odometer: OdometerReading,
        peerOdometer: OdometerReading? = null,
    ) = compose.setContent {
        HoverboardRemoteTheme {
            SetupScreen(
                state = state,
                armed = false,
                telemetry = null,
                odometer = odometer,
                peerOdometer = peerOdometer,
                actions = Recorder(),
            )
        }
    }

    /**
     * The count, with its thousands separated, and the word that says what it counts. The
     * magnitudes run into the millions and an unseparated one is unreadable.
     */
    @Test
    fun travelShowsSeparatedEdgeCountsAndSaysTheyAreEdges() {
        show(setupState(board = 0x01), odometer = OdometerReading.Edges(1_284_310))

        compose.onNodeWithText(s(R.string.setup_travel_value, 1_284_310L)).assertExists()
    }

    /**
     * Zero edges is a board reporting a wheel that has never turned, so it draws a figure rather
     * than the words for a board that has sent nothing.
     */
    @Test
    fun aWheelThatHasNeverTurnedShowsZeroRatherThanSilence() {
        show(setupState(board = 0x01), odometer = OdometerReading.Edges(0))

        compose.onNodeWithText(s(R.string.setup_travel_value, 0L)).assertExists()
        compose.onNodeWithText(s(R.string.setup_travel_silent)).assertDoesNotExist()
    }

    /** And a board that has sent nothing says so, with no number at all. */
    @Test
    fun aBoardThatHasSentNothingHasNoTravelFigure() {
        show(setupState(board = 0x01), odometer = OdometerReading.Silent)

        compose.onNodeWithText(s(R.string.setup_travel_silent)).assertExists()
        compose.onNodeWithText(s(R.string.setup_travel_value, 0L)).assertDoesNotExist()
    }

    /** One board in the session, one row: the average is for a pair. */
    @Test
    fun aSessionWithOneBoardDrawsOneTravelRowAndNoAverage() {
        show(setupState(board = 0x01), odometer = OdometerReading.Edges(1_284_310))

        compose.onNodeWithText(s(R.string.setup_travel_slave)).assertDoesNotExist()
        compose.onNodeWithText(s(R.string.setup_travel_machine)).assertDoesNotExist()
    }

    /**
     * The pair's figure is the MEAN of the two wheels, not their sum: both wheels cover the same
     * ground, so a sum would report twice the travel.
     */
    @Test
    fun aPairAveragesItsTwoWheelsRatherThanAddingThem() {
        show(
            setupState(board = 0x01, other = 0x02),
            odometer = OdometerReading.Edges(1_000_000),
            peerOdometer = OdometerReading.Edges(1_400_000),
        )

        compose.onNodeWithText(s(R.string.setup_travel_machine_value, 1_200_000L)).assertExists()
        compose.onNodeWithText(s(R.string.setup_travel_machine_value, 2_400_000L)).assertDoesNotExist()
    }

    /** A pair with a board that has not reported gets no average, and says why rather than guessing. */
    @Test
    fun aPairWithOneBoardNotReportingShowsNoAverage() {
        show(
            setupState(board = 0x01, other = 0x02),
            odometer = OdometerReading.Edges(1_000_000),
            peerOdometer = OdometerReading.Silent,
        )

        compose.onNodeWithText(s(R.string.setup_travel_machine_incomplete)).assertExists()
    }

    /**
     * The note for a board that cannot convert: it says what the number is and what is missing,
     * which is the condition of showing a count at all.
     */
    @Test
    fun travelSaysWhatWouldBeNeededToMakeItADistance() {
        show(setupState(board = 0x01), odometer = OdometerReading.Edges(1_284_310))

        compose.onNodeWithTag(TRAVEL_SECTION_TAG).assertExists()
        compose.onNodeWithText(s(R.string.setup_travel_note)).assertExists()
        compose.onNodeWithText(s(R.string.setup_travel_note_converted)).assertDoesNotExist()
    }

    /**
     * A board told its wheel shows the distance, in metres under a kilometre: 518 mm is what 90
     * edges cover on a 165 mm wheel with 15 pole pairs, and 90,000 of them is 518 m.
     */
    @Test
    fun aBoardToldItsWheelShowsMetresUnderAKilometre() {
        show(
            setupState(board = 0x01),
            odometer = OdometerReading.Travelled(edges = 90_000, millimetres = 518_363),
        )

        compose.onNodeWithText(s(R.string.setup_travel_metres, 518L)).assertExists()
        compose.onNodeWithText(s(R.string.setup_travel_value, 90_000L)).assertDoesNotExist()
    }

    /**
     * And kilometres to two decimals from a kilometre up, because a lifetime total in metres is
     * unreadable while fifty metres shown as 0.05 km throws away what the counter knows.
     */
    @Test
    fun aLifetimeTotalShowsKilometresToTwoDecimals() {
        show(
            setupState(board = 0x01),
            odometer = OdometerReading.Travelled(edges = 7_200_000, millimetres = 41_469_023),
        )

        compose.onNodeWithText(s(R.string.setup_travel_kilometres, 41.469023)).assertExists()
    }

    /** The note for a pair where every reporting board converted says the figures are converted. */
    @Test
    fun aConvertedPairSaysTheFigureCameFromTheWheelSizeTheBoardsWereTold() {
        show(
            setupState(board = 0x01, other = 0x02),
            odometer = OdometerReading.Travelled(edges = 90_000, millimetres = 518_363),
            peerOdometer = OdometerReading.Travelled(edges = 90_000, millimetres = 518_363),
        )

        compose.onNodeWithText(s(R.string.setup_travel_note_converted)).assertExists()
        compose.onNodeWithText(s(R.string.setup_travel_note)).assertDoesNotExist()
    }

    /**
     * A pair with one wheel known and one not is reading in two units, so the machine row stays
     * in edges and the note stays the one that says what is missing.
     */
    @Test
    fun aHalfConvertedPairAveragesEdgesAndKeepsTheUnconvertedNote() {
        show(
            setupState(board = 0x01, other = 0x02),
            odometer = OdometerReading.Travelled(edges = 90_000, millimetres = 518_363),
            peerOdometer = OdometerReading.Edges(110_000),
        )

        compose.onNodeWithText(s(R.string.setup_travel_machine_value, 100_000L)).assertExists()
        compose.onNodeWithText(s(R.string.setup_travel_note)).assertExists()
        compose.onNodeWithText(s(R.string.setup_travel_note_converted)).assertDoesNotExist()
    }
}
