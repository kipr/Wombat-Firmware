# Contributing

Thanks for helping improve KIPR's software. These guidelines apply to everyone,
whether you write the code yourself or with an AI coding agent. Agents also
read [AGENTS.md](AGENTS.md), which describes this repository in detail.

New to KIPR development? Start with the
[KIPR Development Toolkit](https://github.com/kipr/KIPR-Development-Toolkit)
and its Wombat Developer Manual.

## Branches

Name branches after the person who owns the change, followed by a short topic:

```
<name>/<topic>        e.g. jdoe/servo-pulse-timing
```

This applies to agent-written changes too: the branch names the person
responsible for it. Branch from `master` and open pull requests against
`master`. If you don't have write access, work from a fork.

## Plan first

For anything beyond a small, obvious fix, write a short plan before changing
code: what you'll change, which files it touches, and how you'll verify it. Get
it reviewed before you start, in an issue, a draft pull request, or with the
maintainer you're working with. A wrong approach is much cheaper to catch in a
plan than in code review.

If you're working with an agent, have it present its plan and review the plan
yourself before it edits anything.

## Keep changes focused

- Make one logical change per pull request.
- Work within the existing architecture and conventions described in
  AGENTS.md. If you think the architecture itself should change, propose that
  separately.
- Don't reformat, rename, or reorganize code you aren't otherwise changing; it
  buries the real change in review. Preserve each file's existing formatting
  and line endings.

## Build for what's needed now

Avoid speculative changes: no abstractions, options, or extension points for
features or hardware that don't exist yet. If you spot a future need, open an
issue for it instead.

## Comments

Comment what a reader can't see from the code: hardware quirks, timing
constraints, datasheet references, and the reason behind a non-obvious choice.
Don't describe what the code plainly does, and don't commit commented-out code;
version control keeps the history.

## Documentation

Update documentation in the same pull request as the change that affects it:
README.md, AGENTS.md, CONTRIBUTING.md, and anything under `docs/`.

## Pull requests

Open pull requests as drafts while they're in progress. The description should
cover what changed and why, how you verified it, and anything left undone.
State plainly what wasn't tested rather than implying it was.

## This repository

- CI builds every pull request and uploads `wombat.bin` as an artifact, so
  reviewers can flash it without building. Run the same Docker build locally
  before opening a pull request (see [AGENTS.md](AGENTS.md#build)).
- Register map changes need a paired libwallaby pull request and a passing
  `scripts/check-register-map.sh`.
- Changes to hardware behavior need testing on a Wombat. Describe what you
  tested in the pull request, or say that it's untested.
