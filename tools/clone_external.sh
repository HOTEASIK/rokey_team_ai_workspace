#!/usr/bin/env bash
# 다른 저장소의 코드를 팀 저장소 안으로 가져올 때 쓰는 스크립트.
# clone 후 그 안의 .git 을 ~/git_backups 로 옮겨서, 일반 파일로 커밋되게 만듭니다.
#
# 사용법:
#   bash tools/clone_external.sh <저장소 URL> <넣을 폴더 경로> [브랜치]
# 예:
#   bash tools/clone_external.sh https://github.com/ahnisinc/cobot_rg2.git 협동-1_ROS2_자동화공정/ws_dsr/src
set -e

if [ $# -lt 2 ]; then
  echo "사용법: bash $0 <저장소 URL> <넣을 폴더 경로> [브랜치]"
  exit 1
fi

url=$1
dest=$2
branch=$3

if [ -e "$dest" ] && [ -n "$(ls -A "$dest" 2>/dev/null)" ]; then
  echo "❌ $dest 가 이미 있고 비어 있지 않습니다. 다른 경로를 지정하거나 먼저 정리하세요."
  exit 1
fi

if [ -n "$branch" ]; then
  git clone --depth 1 -b "$branch" "$url" "$dest"
else
  git clone --depth 1 "$url" "$dest"
fi

backup=~/git_backups/$(basename "$dest")_git_$(date +%Y%m%d%H%M%S)
mkdir -p ~/git_backups
mv "$dest/.git" "$backup"

# 그 안에 또 들어 있는 .git (서브모듈 등)도 같이 옮김
find "$dest" -name .git -prune | while read -r g; do
  mv "$g" "${backup}_$(echo "${g#$dest/}" | tr '/' '_')"
done

echo ""
echo "✅ 완료: $dest (원래 .git 은 $backup 로 옮김)"
echo "   원본 저장소 주소를 기록해 두려면 $dest/README.md 등에 적어 두세요: $url"
echo "   이제 평소처럼 git add \"$dest\" 후 커밋하면 됩니다."
