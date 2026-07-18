# Artifact manifests

Copy `example.yaml` for each real bag, PCD, or map bundle. `sha256` and
`size_bytes` must match the source file.

- Use `repo://path/from/repository/root` when the artifact is tracked in Git.
- Use an externally retrievable URI for ignored large artifacts such as bag or
  localization database files.
- `uri: pending` is allowed during software preparation, but must be replaced
  before the corresponding large artifact is needed on another computer or in
  the first real-data acceptance test.
