# Operations

## Build overlays

```bash
./scripts/fetch_dependencies.sh
./scripts/build_all.sh
source vendor_ws/install/setup.bash
source robot_ws/install/setup.bash
```

Use `--project-only` while developing without Livox or localization sources.

## Profiles

- `bench`: 0.1 m/s, +/-5 degrees, Nav2 disabled.
- `navigation`: 0.2 m/s, +/-22 degrees, forward-only 2WIS.

The bridge starts inactive and disarmed. Activating the lifecycle node does not
arm motion. Use `/romo_b/arm` only after `doctor.sh --hardware` is green.

## Data

Generated data belongs in `data/local/`. Create a manifest with
`scripts/register_artifact.sh`; it calculates size and SHA-256. `uri: pending`
is accepted during software development but must be replaced before the first
real-data acceptance run.
