#!/usr/bin/env bash
set -euo pipefail

cd "$(git rev-parse --show-toplevel)"

mapfile -d '' -t headers < <(
    git diff --name-only --diff-filter=U -z -- ':(top,glob)src/html/headers/web-*.h'
)

if ((${#headers[@]} == 0)); then
    printf '%s\n' 'Keine konfliktbehafteten Web-Header gefunden.'
    exit 0
fi

git restore --source=origin/master --staged --worktree -- "${headers[@]}"
printf '%s Web-Header aus origin/master uebernommen und als geloest markiert.\n' "${#headers[@]}"
