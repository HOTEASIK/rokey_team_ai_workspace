# Git 브랜치 전략 및 사용 방식

## 브랜치 구조 (Git Flow 간소화)

| 브랜치 | 용도 | 규칙 |
|---|---|---|
| `main` | 배포·제출용 안정 버전 | 직접 push 금지, `develop`에서만 병합 |
| `develop` | 개발 통합 브랜치 | 직접 push 금지, `사용자명_기능명`을 PR로 병합 |
| `사용자명_기능명` | 기능 단위 작업 | `develop`에서 분기 → 작업 → `develop`으로 PR |

```
main     ●────────────────────●──────
          \                  /
develop    ●────●──────●────●────────
                 \    / \  /
사용자명_기능명   ●──●   ●●
```

## 브랜치 이름 규칙

- `<사용자명>_<기능명>` 형식 (예: `hong_login`, `kim_yolo-detect`)
- 버그 수정도 같은 형식 (예: `hong_camera-fix`)
- 영어 소문자, 사용자명과 기능명은 `_`, 기능명 안의 단어는 `-`로 연결

## 작업 순서

```bash
# 1. 최신 develop 받기
git checkout develop
git pull origin develop

# 2. 기능 브랜치 만들기
git checkout -b 사용자명_기능명

# 3. 작업 후 커밋
git add .
git commit -m "기능명: 설명"

# 4. 원격에 올리기
git push origin 사용자명_기능명

# 5. GitHub에서 사용자명_기능명 → develop 으로 Pull Request 생성
#    → 팀원 1명 이상 리뷰 후 병합

# 6. 병합 후 로컬 정리
git checkout develop
git pull origin develop
git branch -d 사용자명_기능명
```

## 작업 중 develop 변경사항 반영

```bash
git checkout 사용자명_기능명
git pull origin develop   # 충돌 나면 해결 후 커밋
```

## 커밋 메시지 규칙

`<기능명>: <설명>` 형식

- 기능명: 작업한 기능 (브랜치의 기능명과 맞추기)
- 설명: 무엇을 했는지 한 줄로

예: `login: 로그인 화면 추가`, `yolo-detect: 사람 인식 정확도 개선`

## 릴리스 (develop → main)

- 기능이 모여 안정화되면 `develop` → `main` PR 생성
- 팀 전체 확인 후 병합

## 주의사항

- `main`, `develop`에 직접 push 하지 않기
- 작업 시작 전 항상 `git pull`
- 비밀번호·API 키 커밋 금지 (`.env`는 `.gitignore`에 추가)

# 규칙 예시

## 폴더 생성

프로젝트별 폴더 >>> 기능 폴더 >>> 기능_코드 + 기능_내용_문서

## 기능 수정 시 
기능_내용_문서 바로 아래에 이어서 작성
## 다른 저장소 코드를 가져올 때 (중요)

clone한 폴더 안에 `.git` 이 남아 있으면, 팀 저장소에는 파일 대신 **커밋 번호 하나(서브모듈 링크)** 만 올라가서 GitHub에서 그 폴더를 누르면 404가 납니다.

**처음 clone 후 한 번만** 훅을 켜 두세요. 이런 커밋을 자동으로 막아 줍니다.

```bash
bash tools/setup_hooks.sh
```

외부 코드는 아래 스크립트로 가져오면 `.git` 을 `~/git_backups/` 로 자동으로 옮겨 줍니다.

```bash
bash tools/clone_external.sh <저장소 URL> <넣을 폴더 경로> [브랜치]
```

이미 clone해 둔 폴더라면 직접 옮기면 됩니다.

```bash
git rm --cached -f <폴더>         # 이미 링크로 add 됐을 때만 (파일은 안 지워짐)
mkdir -p ~/git_backups && mv <폴더>/.git ~/git_backups/<이름>_git
git add <폴더>
```

훅을 안 켠 팀원이 올려도 PR에서 `no-nested-git` 검사가 실패해서 알 수 있습니다.
