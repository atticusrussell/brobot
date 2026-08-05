# Contributing

Working conventions for this repository. Structure lives in [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md);
sequencing lives in [`docs/ROADMAP.md`](docs/ROADMAP.md). This file covers how changes get made.

---

## Branching

Short-lived branches off `humble`, merged and deleted. No long-lived milestone branches: navigation
and hardware work run concurrently and both touch URDF, launch, and config files, so parallel
long-running branches would spend more time on conflicts than on work.

**A branch covers one unit of work.** Usually that is one issue. Group several when they edit the same
file or only make sense together — a costmap tuning pass is one branch, not four.

**Naming: `<track>/<slug>`**, lowercase, using the track IDs from the roadmap.

```
nav/annotate-nav2-params
nav/amcl-and-costmap-tuning
hw/jetson-workspace-build
det/synthetic-capture-pipeline
bt/v0-drive-and-return
```

Track prefixes rather than issue numbers: branches stay readable without a lookup, survive issue
renumbering, and group under `git branch --list 'nav/*'`. Reference issues in the pull request body
with `Closes #N` instead.

**Skip the branch entirely** for reading issues, documentation typos, and deleting stale artifacts.
Close the issue and push to `humble`.

---

## Merging

| Change | Merge style | Why |
|---|---|---|
| Code, config, docs | Squash | One issue, one commit on the trunk |
| Tuning campaigns | Merge commit | The commit sequence is the record — "inflation radius 0.3 → 0.55, and what changed" is the point |

Squashing a tuning branch destroys the only durable trace of what was tried. Keep those commits.

---

## Tagging

Tag `humble` when a milestone closes, using the lowercased track ID:

```
git tag nav-2-complete && git push origin nav-2-complete
```

Tagging is independent of branching — a milestone boundary does not require a milestone branch.

---

## Issues and milestones

- Every milestone has a GitHub Milestone named with its track ID, e.g. `NAV-2 — Real Nav`.
- Issues within a milestone are ordered. Work is pulled from the top rather than chosen.
- Reading issues carry the `theory` label **and** the milestone whose work they unblock. They are
  never label-only: reading detached from the work it serves has no natural stopping point.
- Off-roadmap work carries `side-quest` and no milestone. Pull from there between commitments —
  [open side-quests](https://github.com/atticusrussell/ballbot/issues?q=is%3Aopen+is%3Aissue+label%3Aside-quest).

---

## Documentation

Three documents, deliberately non-overlapping, so that no change requires editing two of them:

| Document | Holds | Changes when |
|---|---|---|
| `docs/ARCHITECTURE.md` | Node graph, TF tree, topic contracts, package layout, open questions, decision log | The system's structure changes |
| `docs/ROADMAP.md` | Track structure, dependency edges, goals, exit criteria, reading | A sequencing decision is made |
| `CONTRIBUTING.md` | Branching, merging, tagging, issue conventions | A process decision is made |

Neither `ARCHITECTURE.md` nor `ROADMAP.md` carries work status or issue enumerations — GitHub holds
those, so doing work never obliges a documentation edit.

`ARCHITECTURE.md` does not reference milestone IDs; sequencing belongs to the roadmap. Its decision log
is the exception, being a dated historical record rather than a live reference.

**Architectural decisions get a dated line in the decision log**, with the reason. An open question
moves out of the open-questions section when it is decided, and the rationale goes to the log.
