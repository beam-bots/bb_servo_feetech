<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# Change Log

All notable changes to this project will be documented in this file.
See [Conventional Commits](Https://conventionalcommits.org) for commit guidelines.

<!-- changelog -->

## [v0.3.3](https://github.com/beam-bots/bb_servo_pca9685/compare/v0.3.2...v0.3.3) (2026-06-29)




### Bug Fixes:

* fetch control table lazily so the bridge survives controller startup ordering (#61) by James Harton

## [v0.3.2](https://github.com/beam-bots/bb_servo_pca9685/compare/v0.3.1...v0.3.2) (2026-06-25)




### Bug Fixes:

* disarm must confirm torque is disabled, not fire-and-forget (#51) (#57) by James Harton

### Improvements:

* support bb 0.20.3 robot_opts/0 child spec (#48) by James Harton

## [v0.3.1](https://github.com/beam-bots/bb_servo_pca9685/compare/v0.3.0...v0.3.1) (2026-05-27)




### Improvements:

* write device default to config.exs, load from app env (#32) by James Harton

## [v0.3.0](https://github.com/beam-bots/bb_servo_pca9685/compare/v0.2.4...v0.3.0) (2026-05-21)




### Features:

* remove `reverse?`, move to motor-space (#28) by James Harton

### Bug Fixes:

* skip `bb_servo_feetech.install` when controller is already present (#27) (#29) by James Harton

## [v0.2.4](https://github.com/beam-bots/bb_servo_pca9685/compare/v0.2.3...v0.2.4) (2026-05-17)




## [v0.2.3](https://github.com/beam-bots/bb_servo_pca9685/compare/v0.2.2...v0.2.3) (2026-05-13)




### Improvements:

* add `bb_servo_feetech.install` igniter task (#25) by James Harton

## [v0.2.2](https://github.com/beam-bots/bb_servo_pca9685/compare/v0.2.1...v0.2.2) (2026-03-22)




### Improvements:

* controller: replace message-based writes with ETS-backed fixed-rate control loop (#20) by James Harton

* actuator: add velocity/duration hints and trajectory command support by James Harton

## [v0.2.1](https://github.com/beam-bots/bb_servo_pca9685/compare/v0.2.0...v0.2.1) (2026-02-08)




### Bug Fixes:

* prevent stale-goal torque impulse when arming (#10) by James Harton

* prevent stale-goal torque impulse when arming by James Harton

## [v0.2.0](https://github.com/beam-bots/bb_servo_pca9685/compare/v0.1.0...v0.2.0) (2026-01-09)




### Features:

* implement Feetech servo driver with controller, actuator, and bridge by James Harton

