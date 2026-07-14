# 初回セッション成果物のコミット(テスト全PASS 確認つき)
$ErrorActionPreference = "Continue"
$repo = Split-Path -Parent $PSScriptRoot
Set-Location $repo
$env:PYTHONDONTWRITEBYTECODE = "1"

python -m unittest discover -s tests 2>&1 | Out-File -Encoding utf8 reports\test_results_2026-07-15.txt
$ok = ($LASTEXITCODE -eq 0)
Get-Content reports\test_results_2026-07-15.txt -Tail 3
if (-not $ok) { Write-Host "unittest FAILED - commit中止"; exit 1 }

git add reports phase0 contracts mission safety tests scripts
git commit -m @'
Phase 0/Gate 0: static audit, safety contracts, command arbiter, stop transitions

- reports/: static audit + classification, platform/GPU/hash inventory
  (RTX 5090 32GB / CUDA 13.2 / WSL2 Ubuntu-24.04; policy artifact SHA-256
  verified against docs/01 records), offline test evidence (137/137 PASS)
- phase0/: hardware manifest / stair registry / API gate report templates
  (all BLOCKED markers; vendor question list included)
- contracts/: GoalSpec, StairModel, CommandEnvelope, LocomotionCommand,
  StopState (stdlib only, strict parsers, no-bypass validation)
- mission/command_arbiter.py: 8-level priority arbitration, expiry ->
  Controlled Stop, latching without auto-recovery, STOP_NOW unconditional
  accept, clock-jump fail-closed
- safety/stop_transitions.py: 7-state stop transition table, tuple guards,
  undefined transitions rejected (Gate 0)
- tests/: 137 offline tests (49-edge exhaustive, 8x8 priority matrices)
- scripts/: non-destructive inventory + session runners

All offline, robot-disconnected. No existing files modified.
Adversarially reviewed (2 multi-agent passes; 19 confirmed findings fixed).

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>
'@
git log --oneline -2
git status --short
