# soda_os integration branch

This module is part of https://github.com/somarobotics/soda_os. Clone that repository with `--recurse-submodules`, then install and run from its root. `src/`, `web/` and `tests/` in the parent compose these version-pinned sources.

The integration branch is `refactor/soda-os-unified-20260908`. Original x16 changes are preserved before the namespace/session migration commits. Existing default branches are not overwritten. Review and merge module updates independently; update the parent Git link and `modules.lock.json` only after verification.

The parent `docs/design-review-20260908.md` documents namespace mappings, protocol changes, retired interfaces, and hardware validation limits. Historical launch instructions in this repository may target the previous layout; the supported entry is `soda-os` from the parent checkout.
