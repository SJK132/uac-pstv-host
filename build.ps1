[CmdletBinding()]
param(
    [switch]$Logging
)

$ErrorActionPreference = 'Stop'
$projectDir = (Resolve-Path -LiteralPath $PSScriptRoot).Path
$wslProjectDir = (& wsl.exe -e wslpath -a $projectDir).Trim()

if ($LASTEXITCODE -ne 0 -or [string]::IsNullOrWhiteSpace($wslProjectDir)) {
    throw 'Unable to translate the project path through WSL2.'
}

$loggingValue = if ($Logging) { 'ON' } else { 'OFF' }
& wsl.exe -e bash -lc 'cd "$1" && UAC_PSTV_ENABLE_LOGGING="$2" bash ./build.sh' bash $wslProjectDir $loggingValue
if ($LASTEXITCODE -ne 0) {
    throw "WSL2 build failed with exit code $LASTEXITCODE."
}
