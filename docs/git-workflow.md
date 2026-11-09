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
