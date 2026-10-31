package com.hoverboard.remote

import android.app.Application
import android.content.ComponentName
import androidx.activity.ComponentActivity
import androidx.test.core.app.ApplicationProvider
import org.junit.rules.RuleChain
import org.junit.rules.TestRule
import org.junit.runner.Description
import org.junit.runners.model.Statement
import org.robolectric.Shadows.shadowOf

/**
 * Wraps a Compose test rule so it can launch its host activity in EVERY build variant.
 *
 * A Compose test rule hosts its content in a bare `androidx.activity.ComponentActivity`, which it
 * launches through an `ActivityScenario`, which resolves the intent against the merged manifest.
 * The only thing that puts that activity in a manifest is the `ui-test-manifest` artifact, and that
 * artifact is a `debugImplementation` (it has to be: on `testImplementation` its manifest is not
 * merged at all). So the DEBUG variant resolves the activity and the RELEASE variant does not, and
 * every rule-based test in the release unit-test variant dies in the rule, before the test body,
 * with "Unable to resolve activity for Intent ... ComponentActivity".
 *
 * Rather than shipping a test-only activity in the release manifest, the component is registered
 * with Robolectric's package manager for the duration of the test. It is the same registration the
 * merged manifest performs, done in the only place that is true for both variants and for test
 * code only.
 *
 * Ordering is the whole point of the chain: the rule launches the activity when it runs, so the
 * registration has to be OUTSIDE it.
 */
fun composeHost(compose: TestRule): RuleChain =
    RuleChain.outerRule(ComponentActivityHost()).around(compose)

private class ComponentActivityHost : TestRule {
    override fun apply(base: Statement, description: Description): Statement = object : Statement() {
        override fun evaluate() {
            val context = ApplicationProvider.getApplicationContext<Application>()
            shadowOf(context.packageManager).addActivityIfNotPresent(
                ComponentName(context, ComponentActivity::class.java),
            )
            base.evaluate()
        }
    }
}
