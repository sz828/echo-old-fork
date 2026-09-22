# Echo

Competition code for a VEX V5RC **Override** (2026-2027) robot, built with PROS.

Forked from [2654E Echo](https://github.com/alexDickhans/echo) (High Stakes,
2024-2025), whose command framework, particle-filter localization and Ramsete
path following this code still builds on. The High Stakes mechanisms and
routines have been removed; [`CLEANUP.md`](CLEANUP.md) lists what went.

```
pros build      # compile
pros mu         # build + upload
pros terminal   # view output
```

[`CLAUDE.md`](CLAUDE.md) is the working guide to the codebase: layout, ports,
driver bindings, conventions, and what is still outstanding.
