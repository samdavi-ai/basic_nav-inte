# Contributing to GRaCEmo ViRa

## Engineering principles

- Keep the Rust kernel focused on event contracts, authorization, state, and persistence. Put perception and provider-specific reasoning in adapters.
- Route robot goals through the registered adapter, kernel authorization, ROS 2 coordinator, and Nav2 pipeline. Do not add a second movement-control path.
- Keep E-stop, collision monitoring, the configured footprint, command arbitration, and terrain safety protections enabled.
- Treat AMCL as the primary Nav2 localization source until visual localization passes independent accuracy and availability acceptance.
- Keep physical robot and motor-output testing out of desktop simulation changes. Mark Gazebo fixtures, unit tests, live simulation, and hardware checks distinctly.
- Report measured failures and uncertainty. A passing component test does not establish full-system acceptance.

## Repository workflow

- Develop on the requested feature branch. Keep branch-specific changes off unrelated branches.
- Do not rewrite remote history or force-push without explicit authorization.
- Keep source, tests, concise reports, and reproducible result summaries in Git. Do not commit local secrets, private queue payloads, virtual environments, build outputs, or high-volume raw simulation telemetry.
- Preserve existing work in the shared tree. Review `git status` before staging and inspect the staged diff before committing.
- Prefer focused commits using `<type>(<scope>): <description>`, such as `fix(kernel): persist visual observation provenance`.

## Validation

For Rust kernel changes:

```bash
cargo test --locked --manifest-path kernel/Cargo.toml
```

For perception and ROS depth changes, source ROS 2 Jazzy and the local workspace overlay, then run the relevant tests described in the [README](README.md#run-checks).

For navigation changes, run the change-specific component checks and the applicable Gazebo fixture. Preserve the five-room patrol and idle-zero regressions. Never claim live or hardware acceptance based only on a unit test.

## Change descriptions

Include the behavior changed, why it was needed, test commands and results, relevant evidence links, and any remaining limits. For safety or motion-path changes, explain how the existing safety gates were kept in the path.
