#!/usr/bin/env bash
# 저장소를 clone한 뒤 한 번만 실행: 팀 공용 git 훅(.githooks/)을 켭니다.
set -e
root=$(git -C "$(dirname "$0")" rev-parse --show-toplevel)
chmod +x "$root/.githooks/"*
git -C "$root" config core.hooksPath .githooks
echo "✅ git 훅 설치 완료 (core.hooksPath = .githooks)"
