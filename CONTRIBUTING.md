# Contributing to RaidZero-FRC-2026

How we get code from your laptop onto the robot. Read this before your first pull request.

## The short version

1. Pull the latest `main` and make a new branch.
2. Make one small change. Add or update a test for it.
3. Run `./gradlew build` (it formats your code and runs the tests).
4. Push and open a pull request (PR) into `main`.
5. Get CI green and one approving review, then merge.

Nobody pushes directly to `main`. Everything goes through a PR.

## Branches

Start every change from an up-to-date `main`:

```bash
git checkout main
git pull
git checkout -b feature/shooter-sim
```

Name branches `<type>/<short-description>`, lowercase with dashes:

| Type | Use it for | Example |
| --- | --- | --- |
| `feature/` | New behavior or subsystem work | `feature/hood-sim` |
| `fix/` | Fixing a bug | `fix/alliance-flip` |
| `tune/` | Changing constants, PID gains, setpoints | `tune/turret-pid` |
| `test/` | Adding tests only | `test/interpolate-table` |
| `auto/` | Autonomous routines and paths | `auto/left-trench-3` |
| `chore/` | CI, build, docs, cleanup | `chore/contributing-doc` |

Delete your branch after it merges.

## Before you push

Run this from the repo root:

```bash
./gradlew build
```

Locally this also runs `spotlessApply`, so your code gets formatted for you. Commit any files it changes.

CI does **not** auto-format. If you skip this step, the `spotlessCheck` step fails and your PR can't merge. To format without a full build, run `./gradlew spotlessApply`.

If you touched a mechanism, also run it in simulation (`WPILib: Simulate Robot Code` in VS Code) and check that it moves the right direction and stops at its limits.

## Pull requests

- **Keep it small.** One subsystem or one feature per PR. If the description needs the word "also", it's probably two PRs.
- **Write a real description:** what changed, why, and how you tested it (unit test, sim, on the robot).
- **Fill in the checklist** in the PR template.
- **CI must be green.** The Build check runs formatting, compile, tests and checkstyle. Red means don't merge.
- **One approval is required** from a code reviewer (below) who didn't write the code.
- Open a draft PR early if you want feedback before it's finished.

### Code reviewers for the 2026–27 season

- _TBD_
- _TBD_

Reviewers: read every line, pull the branch and run it in sim if it touches a mechanism, and ask the author to explain anything unclear. Approve only when you'd be comfortable deploying it at an event.

## Code conventions

- **Units:** use WPILib units (`Angle`, `Distance`, `AngularVelocity`) in method signatures instead of raw `double`s. When a raw number is unavoidable, put the unit in the name (`distanceMeters`, `angleRotations`) and comment any sign convention (e.g. "positive = counterclockwise viewed from above").
- **Hardware goes behind IO interfaces.** Subsystem logic talks to an `XxxIO` interface, never directly to a motor controller. Each IO has a `Real` and a `Sim` implementation. This is what lets us test and simulate.
- **Constants go in the subsystem's `XxxConstants` class**, not inline in logic.
- **Javadoc public methods.** Checkstyle enforces this in CI.

## Tests

- Tests live in `src/test/java`, mirroring the package of the code they test. Run them with `./gradlew test`.
- New logic comes with a test in the same PR.
- **Every bug found on the robot gets a test that reproduces it** before the fix merges.
- Tests that use WPILib classes need `HAL.initialize(500, 0)` in a `@BeforeEach` method.

## Using AI tools (Claude, Copilot, ChatGPT, etc.)

AI is allowed and useful. These rules keep the code trustworthy:

1. **Spec first.** Write down the behavior you want in plain words, then turn it into tests. A human reviews the tests before any AI writes the implementation.
2. **Tests are protected.** Never let an AI edit a test just to make it pass. A failing test means a person decides whether the code or the requirement is wrong.
3. **Small PRs.** One subsystem or feature each, so reviewers can actually read every AI-generated line.
4. **Say what was AI-generated** in the PR description.
5. **You own it.** You must be able to explain any AI-written code you submit, line by line, before it merges. If you can't explain it, don't submit it.
6. **Check units and signs yourself.** AI frequently mixes up rotations, radians and degrees, or flips a direction.

## At events

- Before each event, confirm `main` builds green and the code deployed on the robot matches `main`.
- Changes made in the pit still go on a branch and get a PR afterwards. Don't leave fixes only on the laptop that deployed them.

## Questions

Ask in the programming channel or grab a reviewer. Nobody expects you to know all of this on day one.
