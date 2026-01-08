# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

[
  tools: [
    {:credo, "mix credo --strict"},
    {:reuse, command: ["pipx", "run", "reuse", "lint", "-q"]}
  ]
]
