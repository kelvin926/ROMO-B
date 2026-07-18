# Local data handoff policy

This directory keeps compact, reproducibility-relevant artifacts in Git:

- generated PCD, Nav2, and Lanelet2 maps;
- mapping metadata and pose graphs;
- screenshots, JSON results, and text logs from validation;
- rosbag metadata without the recording database.

Individual tracked files must remain at or below 10 MiB. Large rosbag and
localization databases (`*.db3`, `*.mcap`, and `*.bag`) remain ignored and are
described by the tracked files in `data/manifests/`. Put their external URI in
the corresponding manifest before transferring the project to another host.
