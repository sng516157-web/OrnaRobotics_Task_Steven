<#
.SYNOPSIS
  Read-only Windows/WSL preflight for the cable-insertion simulation.

.DESCRIPTION
  Checks WSL availability, the target distribution, WSL 2, Ubuntu 24.04,
  and WSLg support. This script does not install software, enable Windows
  features, change WSL settings, or reboot the machine.

.EXAMPLE
  powershell.exe -ExecutionPolicy Bypass -File .\scripts\preflight_windows.ps1

.EXAMPLE
  powershell.exe -ExecutionPolicy Bypass -File .\scripts\preflight_windows.ps1 -Distro Ubuntu-24.04
#>

[CmdletBinding()]
param(
    [string]$Distro = "Ubuntu-24.04"
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = "Stop"

$script:Failures = 0
$script:Warnings = 0

function Write-Pass([string]$Message) {
    Write-Host "[PASS] $Message" -ForegroundColor Green
}

function Write-Warn([string]$Message) {
    $script:Warnings++
    Write-Host "[WARN] $Message" -ForegroundColor Yellow
}

function Write-Fail([string]$Message) {
    $script:Failures++
    Write-Host "[FAIL] $Message" -ForegroundColor Red
}

function Write-Info([string]$Message) {
    Write-Host "[INFO] $Message" -ForegroundColor Cyan
}

function Invoke-WslCommand {
    param([string[]]$WslArgs)

    $job = $null
    try {
        $job = Start-Job -ScriptBlock {
            param($Arguments)
            $output = (& wsl.exe @Arguments 2>&1 | Out-String).Trim()
            [PSCustomObject]@{
                ExitCode = $LASTEXITCODE
                Output   = $output
            }
        } -ArgumentList (,$WslArgs)

        $finished = Wait-Job -Job $job -Timeout 15
        if ($null -eq $finished) {
            Stop-Job -Job $job -ErrorAction SilentlyContinue
            return [PSCustomObject]@{
                ExitCode = 124
                Output   = "wsl.exe $($WslArgs -join ' ') timed out after 15 seconds."
            }
        }

        return Receive-Job -Job $job
    }
    catch {
        return [PSCustomObject]@{
            ExitCode = 1
            Output   = $_.Exception.Message
        }
    }
    finally {
        if ($null -ne $job) {
            Remove-Job -Job $job -Force -ErrorAction SilentlyContinue
        }
    }
}

function ConvertTo-NormalizedLines([string]$Text) {
    return @(
        ($Text -replace "`0", "") -split "\r?\n" |
            ForEach-Object { $_.Trim() } |
            Where-Object { $_.Length -gt 0 }
    )
}

function Complete-Preflight {
    Write-Host ""
    Write-Host "================ Windows/WSL summary ================"

    if ($script:Failures -gt 0) {
        Write-Host "FAILED: $($script:Failures) failure(s), $($script:Warnings) warning(s)." -ForegroundColor Red
        Write-Host "Resolve the failures, then rerun this script. See Section 0 of AGENT_DETERMINISTIC_WAYPOINT_GUIDE.md."
        exit 1
    }

    if ($script:Warnings -gt 0) {
        Write-Host "PASSED WITH WARNINGS: $($script:Warnings) warning(s)." -ForegroundColor Yellow
    }
    else {
        Write-Host "PASSED: Windows/WSL checks completed successfully." -ForegroundColor Green
    }

    Write-Host "Next: copy the project into the WSL Linux filesystem, then run:"
    Write-Host "  bash scripts/preflight_wsl.sh --bootstrap"
    exit 0
}

Write-Host "Cable insertion: read-only Windows/WSL preflight"
Write-Host "Target distribution: $Distro"
Write-Host ""

if ([System.Environment]::OSVersion.Platform -ne [System.PlatformID]::Win32NT) {
    Write-Fail "This script must run in Windows PowerShell, not inside WSL."
    Complete-Preflight
}

$wslCommand = Get-Command wsl.exe -ErrorAction SilentlyContinue
if ($null -eq $wslCommand) {
    Write-Fail "wsl.exe was not found. Install WSL using the approved guide steps."
    Complete-Preflight
}
Write-Pass "wsl.exe is available."

$version = Invoke-WslCommand -WslArgs @("--version")
if ($version.ExitCode -eq 0) {
    Write-Pass "WSL reports version information."
    if ($version.Output -match "WSLg") {
        Write-Pass "WSLg is listed in the WSL version information."
    }
    else {
        Write-Warn "WSLg was not listed. Run 'wsl --update' after approval if graphical support is unavailable."
    }
}
else {
    Write-Warn "This WSL version does not support 'wsl --version'. Update WSL if WSL 2 or WSLg validation fails."
}

$status = Invoke-WslCommand -WslArgs @("--status")
if ($status.ExitCode -eq 0) {
    Write-Pass "WSL status was retrieved."
}
else {
    Write-Warn "WSL status could not be retrieved: $($status.Output)"
}

$list = Invoke-WslCommand -WslArgs @("--list", "--quiet")
if ($list.ExitCode -ne 0) {
    Write-Fail "Could not list WSL distributions: $($list.Output)"
    Complete-Preflight
}

$distros = ConvertTo-NormalizedLines $list.Output
$distroFound = @($distros | Where-Object { $_ -eq $Distro }).Count -gt 0

if (-not $distroFound) {
    Write-Fail "Distribution '$Distro' is not installed. Available distributions: $($distros -join ', ')"
    Write-Info "With user approval, install it using: wsl --set-default-version 2; wsl --install -d $Distro"
    Complete-Preflight
}
Write-Pass "Distribution '$Distro' is installed."

$verboseList = Invoke-WslCommand -WslArgs @("--list", "--verbose")
if ($verboseList.ExitCode -ne 0) {
    Write-Fail "Could not determine the WSL version for '$Distro'."
}
else {
    $verboseLines = ConvertTo-NormalizedLines $verboseList.Output
    $targetLine = $verboseLines | Where-Object {
        $_ -match "^\*?\s*$([regex]::Escape($Distro))(\s|$)"
    } | Select-Object -First 1

    if ([string]::IsNullOrWhiteSpace($targetLine)) {
        Write-Fail "Could not find '$Distro' in the detailed WSL distribution list."
    }
    elseif ($targetLine -match "\s2\s*$") {
        Write-Pass "Distribution '$Distro' uses WSL 2."
    }
    else {
        Write-Fail "Distribution '$Distro' does not report WSL version 2: $targetLine"
    }
}

$osCheck = Invoke-WslCommand -WslArgs @(
    "-d", $Distro, "--", "/bin/sh", "-c",
    '. /etc/os-release; printf "%s|%s\n" "$ID" "$VERSION_ID"'
)

if ($osCheck.ExitCode -ne 0) {
    Write-Fail "Ubuntu could not be launched or its OS release could not be read: $($osCheck.Output)"
    Write-Info "If this is the first launch, open 'wsl -d $Distro' and let the user complete account setup."
}
elseif ($osCheck.Output.Trim() -eq "ubuntu|24.04") {
    Write-Pass "Distribution '$Distro' runs Ubuntu 24.04."
}
else {
    Write-Fail "Expected ubuntu|24.04 but received '$($osCheck.Output)'."
}

Complete-Preflight
