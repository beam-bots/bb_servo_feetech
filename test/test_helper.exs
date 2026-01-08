# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

Application.ensure_all_started(:mimic)

ExUnit.start()

Mimic.copy(BB)
Mimic.copy(BB.Process)
Mimic.copy(BB.Robot)
Mimic.copy(BB.Safety)
Mimic.copy(Feetech)
