# Non-destructive platform / Git / GPU / hash inventory (docs/CLAUDE.md sec 6.1)
# Read-only. No network changes, no robot connection.
# Usage: powershell -ExecutionPolicy Bypass -File scripts\inventory.ps1
$ErrorActionPreference = "Continue"
$repo = Split-Path -Parent $PSScriptRoot
$stamp = Get-Date -Format "yyyy-MM-dd_HHmmss"
$out = Join-Path $repo ("reports\inventory_" + $stamp + ".md")
$lines = New-Object System.Collections.Generic.List[string]

function Add-Run($label, $cmd) {
    $lines.Add("")
    $lines.Add("### " + $label)
    $lines.Add('```')
    try {
        $r = Invoke-Expression $cmd 2>&1 | Out-String
        if (-not $r) { $r = "(no output / NOT INSTALLED)" }
        $lines.Add($r.TrimEnd())
    } catch {
        $lines.Add("NOT INSTALLED / FAILED: " + $_.Exception.Message)
    }
    $lines.Add('```')
}

$lines.Add("# Platform / Git / GPU / Hash inventory - " + $stamp)
$lines.Add("host: " + $env:COMPUTERNAME + " / user: " + $env:USERNAME)
$lines.Add("note: read-only. no credentials recorded.")

$lines.Add("")
$lines.Add("## Git")
Add-Run "git status --short" ("git -C '" + $repo + "' status --short")
Add-Run "git rev-parse HEAD" ("git -C '" + $repo + "' rev-parse HEAD")
Add-Run "git branch -a -v" ("git -C '" + $repo + "' branch -a -v")
Add-Run "git log --oneline -5" ("git -C '" + $repo + "' log --oneline -5")

$lines.Add("")
$lines.Add("## OS / CPU / RAM / Disk")
Add-Run "OS" 'Get-CimInstance Win32_OperatingSystem | Select-Object Caption, Version, OSArchitecture, TotalVisibleMemorySize | Format-List | Out-String'
Add-Run "CPU" 'Get-CimInstance Win32_Processor | Select-Object Name, NumberOfCores, NumberOfLogicalProcessors | Format-List | Out-String'
Add-Run "Disk" 'Get-PSDrive -PSProvider FileSystem | Format-Table Name, Used, Free | Out-String'

$lines.Add("")
$lines.Add("## GPU / CUDA")
Add-Run "nvidia-smi query" "nvidia-smi --query-gpu=name,uuid,driver_version,memory.total --format=csv"
Add-Run "nvcc --version" "nvcc --version"

$lines.Add("")
$lines.Add("## Python / WSL / Docker")
Add-Run "python --version" "python --version"
Add-Run "py -0p" "py -0p"
Add-Run "wsl -l -v" "wsl -l -v"
Add-Run "docker --version" "docker --version"

$lines.Add("")
$lines.Add("## Network (read-only)")
Add-Run "ipconfig" "ipconfig"

$lines.Add("")
$lines.Add("## Hashes (policy / config / docs)")
$targets = @("policy\*.pt", "policy\*.onnx", "policy\*.json", "policy\*.yaml",
             "requirements.txt", "docs\*.md")
foreach ($t in $targets) {
    $files = Get-ChildItem -Path (Join-Path $repo $t) -ErrorAction SilentlyContinue
    foreach ($f in $files) {
        $h = (Get-FileHash -Algorithm SHA256 $f.FullName).Hash.ToLower()
        $rel = $f.FullName.Substring($repo.Length + 1)
        $lines.Add("- " + $rel + "  sha256:" + $h + "  (" + $f.Length + " bytes)")
    }
}

$lines -join "`r`n" | Out-File -FilePath $out -Encoding utf8
Write-Host ("inventory written: " + $out)
