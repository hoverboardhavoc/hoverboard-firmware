# CLAUDE.md

Project conventions for hoverboard-firmware. Global rules live in `~/.claude/CLAUDE.md`.

## Spell out shorthand labels

This project is full of short labels that mean nothing without their definition: `P1`, `R4`,
`D5`, `slice 4`, `Part Zero`, `tier 2`, `A1`, `B1`, opcode names, branch names. Spec sections,
audit findings, decision records and the todo list all number things independently, so the same
token means different things in different files, and a label coined in one session is opaque in
the next.

**On first use in a reply, give the label plus what it is plus where it is defined.** One clause is
enough:

- "P1 (gains as store fields plus a live tune lane, `specs/rider-ui.md` section 4)"
- "R4 (the armed config-write refusal, `specs/link-control.md`)"
- "Part Zero (the unresolved bench findings at the top of `specs/todo.md`)"

Then use the bare label for the rest of the reply. The same applies when writing a goal for a
subagent, which has none of the conversation's context, and when adding an item to a spec or the
todo: a reader months later is in the same position as a fresh agent.

If a label's definition cannot be found, say so rather than guessing at it.

## Running the checks

- **Host test suite: `./tools/host-test.sh`** (it forwards cargo arguments, so `-p store` or a test
  name works). A bare `cargo test` compiles the test binaries for the board and runs NONE of them,
  silently, because `.cargo/config.toml` defaults the target to `thumbv7m-none-eabi`; the script
  resolves the host triple and passes CI's flags, and CI's host-test job runs the same script.
- **Shipping image: `cargo image`**, the build-std release build whose bytes reach flash
  (`.cargo/config.toml` explains why it is an alias and not a config key).

## No backwards compatibility until a version ships

Nothing here owes compatibility with an older anything. The app may break with older firmware, a
master may break with a slave on an older image, a store field may change id, type or meaning, and a
wire format may change shape. There is no released version and no user but the owner, so every board
and the phone are updated together.

So: do not design a migration, a version negotiation, a fallback decode or a "peer on an older
image" case, and do not keep a field alive because something old might still send it. State plainly
what a change breaks, in the commit message and in the spec, and move on. Flash both boards and
reinstall the app when a change crosses them.

The one thing this does not license is silently breaking a SAFETY property to save work: a frame
that arrives truncated for any reason must still fail safe, which is a robustness requirement and
not a compatibility one.
