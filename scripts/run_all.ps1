# 初回セッションの残作業を一括実行する(非破壊 + ブランチ作業のみ)
#   1. 作業ブランチ作成  2. contracts/arbiter テスト実行  3. 既存 offline baseline
#   4. 非破壊 inventory  5. WSL 確認  6. テスト全PASSならコミット
# 実機・ネットワーク接続は一切行わない。
$ErrorActionPreference = "Continue"
$repo = Split-Path -Parent $PSScriptRoot
Set-Location $repo
$env:PYTHONDONTWRITEBYTECODE = "1"
$stamp = Get-Date -Format "yyyy-MM-dd_HHmmss"
$outdir = Join-Path $repo "reports"
if (-not (Test-Path $outdir)) { New-Item -ItemType Directory $outdir | Out-Null }
$log = Join-Path $outdir ("session_run_" + $stamp + ".log")

function Log($msg) { $msg | Tee-Object -FilePath $log -Append }

Log "=== GO2 first-session run_all $stamp ==="
Log ("HEAD before: " + (git rev-parse HEAD))

# ---- 1. ブランチ ----
$branch = "claude/phase0-gate0-contracts"
$cur = (git branch --show-current)
if ($cur -ne $branch) {
    git switch -c $branch 2>&1 | Tee-Object -FilePath $log -Append
    if ($LASTEXITCODE -ne 0) { git switch $branch 2>&1 | Tee-Object -FilePath $log -Append }
}
Log ("branch: " + (git branch --show-current))

# ---- 2. contracts / arbiter / stop-transitions テスト ----
Log "`n=== unittest discover -s tests ==="
python -m unittest discover -s tests -v 2>&1 | Tee-Object -FilePath $log -Append
$unittest_ok = ($LASTEXITCODE -eq 0)
Log ("unittest exit ok: " + $unittest_ok)

# ---- 3. 既存 offline baseline(robot 非接続のもののみ)----
Log "`n=== legacy offline baseline ==="
$baselines = @(
    "cockpit.stair",
    "cockpit.test_lidar_pipeline",
    "cockpit.voice",
    "m3_rl.joint_map",
    "m3_rl.test_obs_builder"
)
$baseline_results = @{}
foreach ($m in $baselines) {
    Log "`n--- python -m $m ---"
    python -m $m 2>&1 | Tee-Object -FilePath $log -Append
    $baseline_results[$m] = $LASTEXITCODE
    Log ("exit code: " + $LASTEXITCODE)
}

# ---- 4. 非破壊 inventory ----
Log "`n=== inventory ==="
powershell -ExecutionPolicy Bypass -File (Join-Path $repo "scripts\inventory.ps1") 2>&1 | Tee-Object -FilePath $log -Append

# ---- 5. WSL / Ubuntu ----
Log "`n=== wsl ==="
wsl -l -v 2>&1 | Tee-Object -FilePath $log -Append
Log ""
python --version 2>&1 | Tee-Object -FilePath $log -Append

# ---- 6. コミット(unittest 全PASS のときのみ)----
Log "`n=== git ==="
git status --short | Tee-Object -FilePath $log -Append
if ($unittest_ok) {
    git add reports phase0 contracts mission safety tests scripts
    git commit -m @'
Phase 0/Gate 0: static audit, safety contracts, command arbiter, stop transitions

- reports/: initial static audit + classification (docs/CLAUDE.md sec.14)
- phase0/: hardware manifest / stair registry / API gate report templates (BLOCKED markers)
- contracts/: GoalSpec, StairModel, CommandEnvelope, LocomotionCommand, StopState
  (stdlib only, strict parsers, no-bypass validation via __post_init__)
- mission/command_arbiter.py: 8-level priority arbitration, expiry -> Controlled Stop,
  latching (no auto-recovery), STOP_NOW unconditional accept, clock-jump fail-closed
- safety/stop_transitions.py: 7-state stop transition table, tuple guards,
  undefined transitions rejected (Gate 0)
- tests/: offline unittest suite (49-edge exhaustive, 8x8 priority matrices)
- scripts/: non-destructive inventory + session runner

All offline, robot-disconnected. Adversarially reviewed (2 multi-agent passes).

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>
'@ 2>&1 | Tee-Object -FilePath $log -Append
    Log ("commit exit: " + $LASTEXITCODE)
    git log --oneline -2 | Tee-Object -FilePath $log -Append
} else {
    Log "unittest failed - commit skipped (fix first)"
}

Log "`n=== summary ==="
Log ("branch: " + (git branch --show-current))
Log ("unittest ok: " + $unittest_ok)
foreach ($k in $baselines) { Log ("baseline " + $k + ": exit " + $baseline_results[$k]) }
Log ("log: " + $log)
