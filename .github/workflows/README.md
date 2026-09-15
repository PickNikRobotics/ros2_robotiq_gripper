# CI overview

| Workflow | Trigger | Blocking | What it covers |
|---|---|---|---|
| [`build_and_test.yaml`](build_and_test.yaml) | PR to `main`, push, nightly, manual | see below | binary + semi-binary tiers across every distro `main` serves |
| [`rolling-source-build.yml`](rolling-source-build.yml) | push, nightly, manual | no | source tier — core ROS from source |
| [`ci-format.yml`](ci-format.yml) | PR, manual | yes | `pre-commit` across all files |
| [`ci-ros-lint.yml`](ci-ros-lint.yml) | PR to `main`, manual | yes | `ament_copyright`, `ament_lint_cmake`, `ament_cpplint` on **lyrical** |
| [`ci-coverage-build.yml`](ci-coverage-build.yml) | PR to `main`, manual | no | coverage — currently broken, see below |
| [`prerelease-check.yml`](prerelease-check.yml) | manual | n/a | `industrial_ci` `PRERELEASE: true` — buildfarm dry-run before tagging |

## The tiers

Two knobs, not one ladder.

**How much is built from source:**

| Tier | Core ROS | Our immediate deps | `.repos` used |
|---|---|---|---|
| binary | deb | **deb** | `ros2_robotiq_gripper-not-released.<distro>.repos` |
| semi-binary | deb | **source** (dev branches) | `ros2_robotiq_gripper.rolling.repos` |
| source | **source** | source | `ros2.repos` + the above |

**Which apt repo the debs come from:** `main` (what users install today) or `testing` (staged for the next sync).

Both `.repos` files carry `serial`, because [wjwwood/serial was never ported to ROS 2](https://github.com/PickNikRobotics/ros2_robotiq_gripper/issues/21) — there is no rosdep key, so it has to be a source checkout even in the binary tier. That is also why `robotiq_driver` is not in the released package set; only `robotiq_controllers` and `robotiq_description` are released as debs.

## Matrix

| Job | apt | Base OS | Blocking |
|---|---|---|---|
| `jazzy-main` | main | noble | ✅ |
| `kilted-main` | main | noble | ✅ |
| **`lyrical-main`** | main | **resolute** | **✅** |
| `rolling-main` | main | resolute | ❌ |
| `jazzy-testing` | testing | noble | ❌ |
| `kilted-testing` | testing | noble | ❌ |
| `lyrical-testing` | testing | resolute | ❌ |
| `rolling-testing` | testing | resolute | ✅ |
| `rolling-main + upstream-source` | main | resolute | ❌ |
| `rolling-testing + upstream-source` | testing | resolute | ❌ |

**lyrical is the Resolute gate.** It is Ubuntu Resolute *and* released, so its `main` apt is populated (`ros2_control` 6.8.0). This repo is released to five distros but CI only ever built rolling — jazzy, kilted and lyrical all shipped untested, and lyrical is the one whose buildfarm is currently failing.

`rolling-main` is non-blocking because Rolling's `main` apt has no Resolute packages yet — the state [#129](https://github.com/PickNikRobotics/ros2_robotiq_gripper/pull/129) established. It will go green on its own once those promote out of `ros2-testing`.

`rolling-testing` stays blocking: it is green today and has been this repo's de-facto Resolute gate.

humble is absent because humble's `hardware_interface` has no `get_optional()`, which `main` requires. humble is served by the [`humble`](https://github.com/PickNikRobotics/ros2_robotiq_gripper/tree/humble) branch. Once `main` carries source-level distro guards per [moveit2#3751](https://github.com/moveit/moveit2/pull/3751), humble returns as one more matrix entry.

## Why the semi-binary jobs are now non-blocking

They used to be blocking, but only because the tier was inert: `ros2_robotiq_gripper.rolling.repos` was byte-identical to the `-not-released` one, so semi-binary was an exact duplicate of binary and could never fail independently.

It now builds `ros2_control` from `master`, which is the difference the tier exists for — `ros2_control` is released, so binary gets it from apt while semi-binary gets the development branch. That means it can go red on upstream's schedule rather than ours, which should not block a PR here.

This is the tier that would have caught `LoanedCommandInterface::get_value()` being removed, instead of it surfacing as a buildfarm release failure ([#109](https://github.com/PickNikRobotics/ros2_robotiq_gripper/issues/109)).

## Known-broken, tracked separately

- **`ci-coverage-build.yml`** runs `ros-tooling/action-ros-ci` directly on a noble runner, outside a container, and has failed since Rolling moved to Resolute. Its own comment proposes the fix: move it inside `industrial_ci` with `OS_CODE_NAME: resolute`.
- **`rolling-source-build.yml`** fetches its `.repos` with the deprecated `?token=` URL syntax and gets an HTTP 404 on every run, so the source tier has never actually executed. One-line fix (drop the token — this is a public repo).

Both are left alone here to keep this change scoped to distro coverage.
