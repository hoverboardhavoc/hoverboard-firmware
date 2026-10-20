package com.hoverboard.remote

import com.hoverboard.remote.ble.BoardTarget
import com.hoverboard.remote.ble.LinkSettings
import com.hoverboard.remote.ble.ScanCandidate
import com.hoverboard.remote.ble.SelectedBoard
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.launch
import kotlinx.coroutines.test.StandardTestDispatcher
import kotlinx.coroutines.test.TestScope
import kotlinx.coroutines.test.advanceUntilIdle
import kotlinx.coroutines.test.resetMain
import kotlinx.coroutines.test.runTest
import kotlinx.coroutines.test.setMain
import org.junit.jupiter.api.AfterEach
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test

/**
 * The Connect screen's board picker, as the ViewModel drives it: run a scan, gather what it hears,
 * tap a row, and have that pick persisted and connected to.
 */
@OptIn(ExperimentalCoroutinesApi::class)
class DevicePickerTest {

    private val dispatcher = StandardTestDispatcher()
    private lateinit var transport: FakeHoverboardTransport
    private lateinit var scanner: FakeBoardScanner
    private lateinit var prefs: FakeSharedPreferences
    private lateinit var settings: LinkSettings
    private lateinit var viewModel: MainViewModel

    @BeforeEach
    fun setUp() {
        Dispatchers.setMain(dispatcher)
        transport = FakeHoverboardTransport(dispatcher.scheduler)
        scanner = FakeBoardScanner()
        prefs = FakeSharedPreferences()
        settings = LinkSettings(prefs)
        viewModel = MainViewModel(transport, settings, scanner)
    }

    @AfterEach
    fun tearDown() {
        Dispatchers.resetMain()
    }

    private fun board(address: String, name: String?, rssi: Int? = -60) = ScanCandidate(
        address = address,
        advertisedName = name,
        cachedName = null,
        rssi = rssi,
    )

    /**
     * [MainViewModel.uiState] is a `stateIn(WhileSubscribed)`, so with no collector its upstream
     * never runs and its value stays the initial one. Every test here reads that value.
     */
    private fun TestScope.collectState() {
        backgroundScope.launch { viewModel.uiState.collect {} }
    }

    @Test
    fun `the scan's sightings become the rows the screen renders`() = runTest(dispatcher) {
        collectState()
        viewModel.findBoards()
        advanceUntilIdle()
        assertTrue(viewModel.uiState.value.findingBoards)

        scanner.advertise(board("11:11:11:11:11:11", "Hoverboard", rssi = -55))
        scanner.advertise(board("22:22:22:22:22:22", null, rssi = -88))
        // The same board again, which is what a radio actually delivers.
        scanner.advertise(board("11:11:11:11:11:11", "Hoverboard", rssi = -57))
        advanceUntilIdle()

        val seen = viewModel.uiState.value.boardsSeen
        assertEquals(listOf("11:11:11:11:11:11", "22:22:22:22:22:22"), seen.map { it.address })
        assertEquals(-57, seen.first().rssi)
        assertEquals("Hoverboard", seen.first().displayName)
    }

    @Test
    fun `tapping a row pins that address, stops the scan and connects`() = runTest(dispatcher) {
        collectState()
        viewModel.findBoards()
        advanceUntilIdle()
        val picked = board("AA:BB:CC:DD:EE:FF", "hb-offroad-m")
        scanner.advertise(picked)
        advanceUntilIdle()

        viewModel.selectBoard(picked)
        advanceUntilIdle()

        assertEquals(SelectedBoard("AA:BB:CC:DD:EE:FF", "hb-offroad-m"), settings.selectedBoard.value)
        assertEquals(BoardTarget.Pinned("AA:BB:CC:DD:EE:FF"), settings.target())
        assertEquals(1, transport.connectCalls)
        assertFalse(viewModel.uiState.value.findingBoards)
        // Nothing else started a scan: the transport's own scan must not race the picker's.
        assertEquals(1, scanner.scans)
    }

    @Test
    fun `the pick survives the app being restarted over the same stored prefs`() = runTest(dispatcher) {
        viewModel.selectBoard(board("AA:BB:CC:DD:EE:FF", "hb-offroad-m"))
        advanceUntilIdle()

        // A fresh store over the same preferences is what the next launch builds.
        val nextLaunch = LinkSettings(prefs)
        assertEquals(BoardTarget.Pinned("AA:BB:CC:DD:EE:FF"), nextLaunch.target())
    }

    @Test
    fun `a board advertising no name is pinned by its address`() = runTest(dispatcher) {
        viewModel.selectBoard(board("AA:BB:CC:DD:EE:FF", null))
        advanceUntilIdle()

        assertEquals(SelectedBoard("AA:BB:CC:DD:EE:FF", null), settings.selectedBoard.value)
        assertEquals(BoardTarget.Pinned("AA:BB:CC:DD:EE:FF"), settings.target())
    }

    @Test
    fun `pressing find twice does not start a second radio scan`() = runTest(dispatcher) {
        collectState()
        viewModel.findBoards()
        advanceUntilIdle()
        viewModel.findBoards()
        advanceUntilIdle()
        assertEquals(1, scanner.scans)
    }

    @Test
    fun `stopping keeps the rows, so one can still be tapped afterwards`() = runTest(dispatcher) {
        collectState()
        viewModel.findBoards()
        advanceUntilIdle()
        scanner.advertise(board("AA:BB:CC:DD:EE:FF", "hb-offroad-m"))
        advanceUntilIdle()

        viewModel.stopFindingBoards()
        advanceUntilIdle()

        assertFalse(viewModel.uiState.value.findingBoards)
        assertEquals(1, viewModel.uiState.value.boardsSeen.size)
    }

    @Test
    fun `a fresh find clears what the last one saw`() = runTest(dispatcher) {
        collectState()
        viewModel.findBoards()
        advanceUntilIdle()
        scanner.advertise(board("AA:BB:CC:DD:EE:FF", "hb-offroad-m"))
        advanceUntilIdle()
        viewModel.stopFindingBoards()
        advanceUntilIdle()

        viewModel.findBoards()
        advanceUntilIdle()
        // Rows from minutes ago are boards that may well be gone; a fresh press asks what is here.
        assertTrue(viewModel.uiState.value.boardsSeen.isEmpty())
    }

    @Test
    fun `connecting by name stops the picker's scan`() = runTest(dispatcher) {
        collectState()
        viewModel.findBoards()
        advanceUntilIdle()

        viewModel.connect()
        advanceUntilIdle()

        assertFalse(viewModel.uiState.value.findingBoards)
        assertEquals(1, transport.connectCalls)
    }
}
