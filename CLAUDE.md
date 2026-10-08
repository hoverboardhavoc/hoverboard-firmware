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
