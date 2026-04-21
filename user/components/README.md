# Legacy User Components Directory

`user/components/` is now a legacy transition directory.

Preferred layout:

- `user/<project>/components/`
- `user/<project>/config/mission.json`
- `user/active_project`

The current build flow only scans the active project's `components/` directory.

If a component becomes stable, reusable, and repository-wide, it should usually
move into a canonical framework package such as:

- `framework/include/gnc/forms/`
- `framework/include/gnc/environment/`
- `framework/include/gnc/vehicle/`
- `framework/include/gnc/interactions/`

Do not treat this directory as the long-term extension model.
