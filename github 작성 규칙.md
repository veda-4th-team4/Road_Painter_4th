# 초보자를 위한 GitHub 협업 실전 가이드

Git을 혼자 사용할 때는 파일을 저장하고 GitHub에 올리는 것만으로도 충분하다. 하지만 여러 명이 동시에 개발하면 서로의 코드를 덮어쓰거나, 테스트되지 않은 코드가 기준 브랜치에 들어가는 문제가 발생할 수 있다.

이를 방지하기 위해 일반적으로 다음 흐름을 사용한다.

> **저장소 복제** $\rightarrow$ **최신 main 동기화** $\rightarrow$ **작업 브랜치 생성** $\rightarrow$ **코드 수정** $\rightarrow$ **변경사항 검토** $\rightarrow$ **커밋** $\rightarrow$ **원격 브랜치에 push** $\rightarrow$ **Pull Request 생성** $\rightarrow$ **코드 리뷰** $\rightarrow$ **merge**

### 💡 핵심 원칙
*   `main` 브랜치에서 직접 개발하지 않고, 작업별 브랜치를 만든 뒤 **Pull Request(PR)**로 검토받는다.

---

## 1. Git과 GitHub의 역할

### Git
Git은 코드의 변경 이력을 로컬 컴퓨터에서 관리하는 도구다.
*   어떤 파일이 변경됐는지 확인
*   변경사항 저장
*   과거 버전 확인
*   여러 작업 브랜치 관리
*   다른 사람의 변경사항과 병합

### GitHub
GitHub는 Git 저장소를 인터넷에서 공유하고 협업할 수 있게 해주는 서비스다.
*   원격 저장소 관리
*   Pull Request & 코드 리뷰
*   Issue 관리
*   브랜치 보호 규칙 설정
*   GitHub Actions를 통한 자동 빌드·테스트

> [!NOTE]
> *   **Git**: 로컬 버전 관리 도구
> *   **GitHub**: Git 저장소를 공유하고 협업하는 웹 서비스

---

## 2. 기본 용어

*   **Repository**: 프로젝트 코드와 변경 이력을 관리하는 저장소.
*   **Commit**: 특정 시점의 변경사항을 하나의 기록으로 저장한 것.
*   **Branch**: 기준 코드에서 분리된 독립적인 작업 공간.
*   **main**: 팀에서 사용하는 기준 브랜치. 항상 빌드 가능하고 검증된 상태를 유지해야 함.
*   **origin**: 로컬 저장소가 연결된 기본 원격 저장소의 별칭.
*   **Push**: 로컬 커밋을 GitHub 원격 저장소에 업로드.
*   **Pull**: 원격 저장소의 변경사항을 로컬에 가져와 반영.
*   **Fetch**: 원격 저장소의 최신 정보만 가져옴 (현재 작업 파일에 바로 반영하지는 않음).
*   **Pull Request (PR)**: 기능 브랜치의 코드를 main에 반영하기 전에 팀원에게 검토를 요청하는 절차.
*   **Merge**: 기능 브랜치의 변경사항을 main에 최종 반영하는 작업.

---

## 3. 처음 프로젝트를 받는 방법

GitHub 저장소를 처음 로컬에 내려받을 때:
```bash
git clone "https://github.com/조직명/저장소명.git"
```

생성된 프로젝트 폴더로 이동합니다:
```bash
cd "저장소명"
```

*   **저장소 위치 확인**: `git rev-parse --show-toplevel`
*   **원격 저장소 확인**: `git remote -v`
*   **현재 브랜치 확인**: `git branch --show-current`
*   **전체 상태 확인**: `git status`

---

## 4. 작업 시작 전 main 동기화

새로운 기능을 개발하기 전에는 항상 최신 main에서 시작해야 합니다.

```bash
# main으로 이동
git switch main

# 원격 main을 안전하게 가져오기 (Fast-Forward만 허용)
git pull --ff-only origin main
```
*   `--ff-only`는 불필요한 merge commit이 자동으로 만들어지는 것을 방지합니다.

---

## 5. 작업 브랜치 생성

main에서 직접 수정하지 않고 작업 전용 브랜치를 생성하여 이동합니다.
```bash
git switch -c feature/login
```

### 🏷️ 올바른 브랜치 네이밍 규칙

*   **기능 추가**: `feature/user-login`, `feature/uart-protocol`
*   **버그 수정**: `fix/login-timeout`, `fix/crc-validation`
*   **문서 수정**: `docs/setup-guide`, `docs/api-specification`
*   **리팩토링**: `refactor/database-layer`, `refactor/motor-controller`
*   **긴급 수정**: `hotfix/server-crash`

> [!WARNING]
> *   **좋지 않은 이름**: `test`, `work`, `new`, `my-branch` (직관적이지 않음)

---

## 6. 이미 main에서 수정했다면?

실수로 main에서 파일을 수정했더라도 아직 커밋하지 않았다면, 변경사항을 유지한 채 새 브랜치로 옮겨갈 수 있습니다.
```bash
# 변경사항을 유지하며 새 브랜치 생성 및 이동
git switch -c feature/작업명
```

---

## 7. 변경사항 확인

```bash
# 간단한 상태 확인 (Short format)
git status --short
```
*   `M`: 수정된 파일 (Modified)
*   `D`: 삭제된 파일 (Deleted)
*   `??`: Git이 추적하지 않는 새 파일 (Untracked)
*   `A`: 스테이징된 새 파일 (Added)

```bash
# 실제 코드 차이 확인
git diff

# 특정 파일만 비교 확인
git diff -- "src/main.c"
```

---

## 8. 스테이징(Staging)이란?

Git은 변경된 모든 파일을 자동으로 커밋하지 않습니다. 커밋에 포함할 대상을 먼저 선택해야 하는데, 이 과정을 **스테이징**이라고 합니다.

```bash
# 특정 파일만 추가
git add -- "src/main.c"

# 여러 파일 명시적 추가
git add -- "src/main.c" "src/motor.c" "include/motor.h"

# 특정 디렉터리의 추가·수정·삭제 전체 반영
git add -A -- "src"
```

---

## 9. `git add .`을 조심해야 하는 이유

`git add .` 명령은 현재 경로 아래의 모든 변경사항을 한 번에 스테이징 영역에 넣습니다. 이 경우 다음과 같은 불필요한 파일들이 실수로 포함될 수 있습니다.
*   비밀번호 및 개인 키 파일 (`.env`, `.key`)
*   빌드 결과물 (`Debug/`, `Release/`, `*.o`, `*.elf`)
*   개인 IDE 설정 파일 및 로그 파일
*   테스트용 대용량 파일

> [!IMPORTANT]
> 협업 프로젝트에서는 필요한 파일만 명시적으로 지정하여 추가하는 습관이 안전합니다.
> `git add -- "실제로 커밋할 파일 경로"`

---

## 10. 커밋 전 검토

```bash
# 스테이징된 파일 목록 확인
git diff --cached --name-only

# 변경량 요약 확인
git diff --cached --stat

# 실제 스테이징된 변경 코드 상세 확인
git diff --cached

# 공백 오류 및 스타일 검사
git diff --cached --check
```
*   `git diff` 결과 화면을 종료하려면 키보드의 **`q`**를 누릅니다.

---

## 11. 잘못 스테이징한 파일 제외

```bash
# 로컬 수정은 보존하고, 커밋 대상(Staging)에서만 제외할 때
git restore --staged -- "파일 경로"

# 작업 파일의 로컬 수정사항 자체를 완전히 버릴 때 (주의!)
git restore -- "파일 경로"
```

---

## 12. 커밋 메시지 작성 규격 (중요)

커밋 메시지는 변경 목적을 명확히 설명할 수 있도록 **접두사(Prefix)**를 붙여 작성합니다.

*   `Init`: 프로젝트 최초 구성 및 초기화
*   `Add`: 새로운 기능 추가
*   `Fix`: 버그 수정
*   `Update`: 기존 기능 개선/수정 (기능 변화 있음)
*   `Refactor`: 동작 변화가 없는 코드 구조 개선
*   `Doc`: 문서 생성 및 수정
*   `Test`: 테스트 코드 추가
*   `Remove`: 불필요한 코드 및 파일 삭제

**예시**:
```bash
git commit -m "Add: Implement binary UART protocol"
git commit -m "Fix: Prevent watchdog timeout race condition"
git commit -m "Doc: Add hardware wiring guide"
```

---

## 13. 커밋 확인

```bash
# 최근 1개 커밋 한 줄로 확인
git log -1 --oneline

# 최근 10개 커밋 확인
git log -10 --oneline
```

---

## 14. Push 전 최신 main 반영 (Rebase)

작업 브랜치에서 개발하는 동안 다른 팀원이 main 브랜치에 변경사항을 반영했을 수 있습니다. 이를 동기화하기 위해 push 전에 반드시 rebase를 진행합니다.

```bash
# 원격 저장소의 최신 정보 패치
git fetch origin

# 내 기능 브랜치를 최신 main 위에 재배치
git rebase origin/main
```
*   충돌이 없으면 `Successfully rebased and updated refs/heads/브랜치명`과 같이 정상 완료됩니다.
*   이 과정을 통해 오래된 main을 기준으로 작성된 코드가 그대로 PR에 올라가는 것을 방지합니다.

---

## 15. Rebase 충돌(Conflict) 해결 방법

Rebase 도중 충돌이 발생하면 다음과 같이 해결합니다.

1.  `git status`로 충돌 파일을 확인합니다.
2.  충돌 부위를 엽니다.
    ```text
    <<<<<<< HEAD
    원격 main 코드 (서버 측 최신 코드)
    =======
    내 브랜치 코드 (내가 수정한 코드)
    >>>>>>> 내 커밋
    ```
3.  양쪽 코드를 비교하여 최종적으로 필요한 코드만 남기고, 충돌 표시기(`<<<<<<<`, `=======`, `>>>>>>>`)를 삭제한 후 저장합니다.
4.  해결한 파일을 다시 스테이징합니다.
    ```bash
    git add -- "충돌 해결 파일"
    ```
5.  Rebase를 계속 진행합니다.
    ```bash
    git rebase --continue
    ```

> [!CAUTION]
> 충돌을 이해하지 못한 상태에서 한쪽 코드를 무작정 전부 삭제하면 안 됩니다. 해당 코드를 작성한 팀원과 상의하여 해결하는 것이 안전합니다.

---

## 16. 원격 브랜치에 Push

```bash
# 최초 1회 push (원격 브랜치와 연결 설정)
git push -u origin feature/작업명

# 이후에는 간략하게 push 실행
git push
```

> [!CAUTION]
> *   `main` 브랜치에 직접 push하는 행위 금지 (`git push origin main` 금지)
> *   협업 브랜치에서 강제 push 금지 (`git push --force` 금지)

---

## 17. Pull Request (PR) 생성

GitHub 웹 페이지에서 **[Compare & pull request]** 버튼을 클릭하여 생성합니다.
*   **base**: `main` (반영될 대상)
*   **compare**: `feature/작업명` (내가 작업한 브랜치)
*   PR 제목은 변경 목적을 한 눈에 알 수 있게 요약합니다 (예: `Add binary UART communication`).

---

## 18. 좋은 PR 본문 작성 규격

PR 본문에는 요약(Summary), 검증 방법(Verification), 특이 사항(Notes)을 작성하여 팀원이 원활하게 리뷰할 수 있도록 돕습니다.

```markdown
## Summary
- 어떤 기능을 추가/변경했는지 기술
- 변경 사유와 동작 변화 요약

## Verification
- [x] 빌드 성공 여부
- [x] 단위 테스트 완료
- [ ] 하드웨어 검증 완료

## Notes
- 리뷰어가 확인해야 할 특이 사항 및 제한 사항
```
> [!IMPORTANT]
> 테스트하지 않은 항목을 테스트했다고 기입하는 것은 절대 금지합니다.

---

## 19. Draft Pull Request 활용

코드 공유는 필요하지만 아직 작업이나 테스트가 완료되지 않은 경우 **Draft PR**을 사용합니다.
*   하드웨어가 준비되지 않았거나, 전체 빌드/통합 시험이 완료되지 않았을 때 유용합니다.
*   일반 PR을 생성했더라도 GitHub 우측 메뉴에서 **`Convert to draft`**를 선택할 수 있습니다.
*   모든 작업과 검증이 완료되면 **`Ready for review`**를 클릭하여 리뷰 모드로 전환합니다.

---

## 20. Push와 Merge의 차이

*   **Push**: 내 로컬 커밋들을 GitHub 원격 저장소의 내 기능 브랜치에 단순히 업로드하는 작업 (`git push`). `main` 브랜치에는 영향을 주지 않습니다.
*   **Merge**: 기능 브랜치의 코드를 검증 및 리뷰한 후 기준 브랜치인 `main`에 최종 병합하는 작업 (GitHub 웹 페이지에서 **[Merge pull request]** 버튼으로 실행).
  - **Merge 선결 요건**: 코드 리뷰 승인 완료, 빌드 성공, 필수 테스트 통과, 충돌 없음, PR 설명과 실제 코드 일치, 팀 승인 완료.

---

## 21. 리뷰 수정사항 반영

코드 리뷰에서 피드백(수정 요청)이 오면, 다른 브랜치를 만들지 않고 **기존 기능 브랜치에서 코드를 수정**한 후 다시 커밋하여 푸시합니다. 기존 PR에 자동으로 업데이트됩니다.

```bash
git status --short
git diff

# 수정 완료 후
git add -- "수정 파일"
git commit -m "Fix: Address review feedback"
git push
```

---

## 22. PR Merge 후 로컬 정리

PR이 최종 merge된 뒤, 로컬 브랜치를 정리합니다.

```bash
# 1. main 브랜치로 전환
git switch main

# 2. 최신 병합 완료된 main 가져오기
git pull --ff-only origin main

# 3. 로컬 기능 브랜치 삭제
git branch -d feature/작업명

# 4. (선택) 원격 브랜치가 남아있다면 원격 브랜치도 함께 삭제
git push origin --delete feature/작업명
```
> [!CAUTION]
> PR이 최종 Merge 완료되기 전에 브랜치를 삭제하면 절대 안 됩니다.

---

## 23. Windows에서 URL 브라우저로 열기 (PowerShell)

PowerShell에 URL만 바로 입력하면 명령어로 해석되어 오류가 발생하므로 아래 명령어를 사용합니다.
```powershell
Start-Process "https://github.com/example/project"
```

---

## 24. LF와 CRLF 줄바꿈 경고 해결

줄바꿈 형식을 Linux 표준인 LF로 일관되게 고정하려면 프로젝트 루트에 `.gitattributes` 파일을 구성하는 것이 좋습니다.
```text
*.c text eol=lf
*.h text eol=lf
*.cpp text eol=lf
*.py text eol=lf
*.md text eol=lf
```

---

## 25. .gitignore의 역할

빌드 부산물, 임시 파일, 설정 파일 등 Git이 추적하지 말아야 할 경로를 정의합니다.
```text
# Build output
Debug/
Release/
build/

# Compiled files
*.o
*.elf
*.hex
*.bin
*.map

# Logs
*.log

# Environment and secrets
.env
.env.*

# Temporary files
*.tmp
*.bak

# IDE-specific files
.vscode/
.idea/
```
> [!IMPORTANT]
> 이미 Git이 추적하여 커밋된 파일은 `.gitignore`에 추가해도 자동으로 제외되지 않으므로, `git rm --cached <파일>`로 트래킹을 해제해야 합니다.

---

## 26. 팀 저장소 관리자 권장 설정 (Branch Protection)

GitHub Settings -> Branches에서 보호 규칙을 추가합니다.
1.  `main` 브랜치 직접 Push 차단 (PR 필수화)
2.  최소 1명 이상의 리뷰 승인(Approval) 요구
3.  CI 빌드 및 테스트 통과 필수 요구
4.  Force Push 및 브랜치 삭제 비활성화
5.  Conversation Resolution (리뷰 스레드 완료) 필수 요구

---

## 27. 매일 활용하는 표준 작업 흐름 (Daily Checklist)

### 1) 작업 시작
```bash
git switch main
git pull --ff-only origin main
git switch -c feature/작업명
```

### 2) 개발 진행 및 상태 체크
```bash
git status --short
git diff
```

### 3) 커밋 준비 및 검증
```bash
git add -- "관련 파일"
git diff --cached --name-only
git diff --cached --stat
git diff --cached --check
git diff --cached
```

### 4) 커밋
```bash
git commit -m "Add: 작업 내용"
```

### 5) push 전 원격 최신 main 반영 (rebase)
```bash
git fetch origin
git rebase origin/main
```

### 6) push
```bash
git push -u origin feature/작업명
```

### 7) 피드백 반영 시
```bash
git add -- "수정 파일"
git commit -m "Fix: 수정 내용"
git push
```

### 8) merge 완료 후 로컬 브랜치 삭제
```bash
git switch main
git pull --ff-only origin main
git branch -d feature/작업명
```

---

## 28. 팀 협업 체크리스트

*   **작업 시작 전**:
    - [ ] 최신 `main` 브랜치를 안전하게 풀 받았는가?
    - [ ] 목적에 맞는 기능 브랜치를 파서 이동했는가?
    - [ ] 올바른 저장소 디렉토리에서 작업 중인가?
*   **커밋 전**:
    - [ ] `git status`를 확인하여 찌꺼기 파일이 Staging에 없는지 확인했는가?
    - [ ] 비밀번호, API 토큰 키가 코드 내에 하드코딩되지 않았는가?
    - [ ] `git diff --cached`를 통해 작성한 코드를 꼼꼼히 재검토했는가?
    - [ ] 로컬 빌드 혹은 단위 테스트를 수행했는가?
*   **PR 생성 전**:
    - [ ] 최신 `main`으로 rebase하여 충돌이 없는가?
    - [ ] PR 제목과 본문 설명이 실제 코드 변경점과 정확하게 일치하는가?
    - [ ] 아직 보완이 남은 미완성 작업이면 Draft PR로 설정했는가?
*   **Merge 전**:
    - [ ] 동료 리뷰어의 승인(Approve)을 완료받았는가?
    - [ ] CI 빌드 테스트가 실패 없이 완벽히 통과했는가?
    - [ ] 충돌(Conflict)이 존재하지 않는가?

---

## 🏁 마무리
GitHub 협업에서 가장 중요한 핵심 4대 원칙입니다.
1.  **`main` 브랜치에서 직접 작업하지 않는다.**
2.  **커밋할 파일을 커밋하기 전에 반드시 한 번 더 검토한다.**
3.  **Pull Request로 팀원의 교차 검토와 리뷰를 받는다.**
4.  **검증되지 않은 코드는 결코 `main`에 merge하지 않는다.**

이 표준 흐름을 엄격히 지키면 여러 명이 동시에 동일한 저장소에서 개발을 진행하더라도, 소스코드가 유실되거나 엉키는 문제없이 안정적이고 전문성 높은 협업 품질을 상시 유지할 수 있습니다.
