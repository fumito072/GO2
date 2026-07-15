# voice/NL パイプラインのコミット+push(テスト全PASS確認つき)
$ErrorActionPreference = "Continue"
$repo = Split-Path -Parent $PSScriptRoot
Set-Location $repo
$env:PYTHONDONTWRITEBYTECODE = "1"
python -m unittest discover -s tests 2>$null
if ($LASTEXITCODE -ne 0) {
    python -m unittest discover -s tests
    Write-Host "unittest FAILED - commit中止"; exit 1
}
Write-Host "unittest: ALL PASS"
git add contracts voice_gateway perception navigation realtime mission safety tests docs phase0 reports scripts
git commit -F (Join-Path $repo "scripts\commit_msg_voice.txt")
git push origin claude/phase0-gate0-contracts
git log --oneline -3
git status --short
