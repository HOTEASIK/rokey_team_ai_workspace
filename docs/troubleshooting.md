
---

## 3. push 할 때 `Permission ... denied to <아이디>` / 403

- 발생: 2026-11, 팀원 `ysm0066` (브랜치 `ysm0066_record-3`)

### 증상

```
remote: Permission to HOTEASIK/rokey_team_ai_workspace.git denied to ysm0066.
fatal: unable to access 'https://github.com/HOTEASIK/rokey_team_ai_workspace.git/': The requested URL returned error: 403
```

### 원인

로그인은 됐지만(아이디가 메시지에 나옴), **그 계정에 이 저장소에 쓸 권한이 없음**. 보통 아래 셋 중 하나:

1. 협업자로 초대되지 않음
2. 초대는 받았지만 아직 수락하지 않음
3. 토큰 종류나 권한이 맞지 않음

### 해결

**1. 저장소 주인(HOTEASIK)이 초대**
- 저장소 → Settings → Collaborators → **Add people** → 팀원 아이디 추가 (Write 이상)
- 이미 목록에 있는데 *Pending*이면 2번으로

**2. 팀원이 초대 수락**
- 이메일의 초대 링크, 또는 `https://github.com/HOTEASIK/rokey_team_ai_workspace/invitations` → **Accept**

**3. 팀원이 토큰 확인**
- Password 자리에는 GitHub 비밀번호가 아니라 **토큰**을 입력
- **Classic 토큰 + `repo` 권한** 추천 (Settings → Developer settings → Personal access tokens → Tokens (classic))
- Fine-grained 토큰은 다른 사람 개인 계정의 저장소에 못 쓰는 경우가 있음. 1, 2번이 다 됐는데도 403이면 Classic 토큰으로 바꿔 보기
- 예전 토큰이 저장돼 있으면 지우고 다시 push
  ```bash
  printf "protocol=https\nhost=github.com\n\n" | git credential reject
  git push origin <브랜치>
  ```

권한만 고치면 되니 커밋을 다시 할 필요는 없고, push만 다시 하면 됩니다.
