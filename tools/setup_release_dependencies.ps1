param(
    [string]$RepoRoot = (Get-Location).Path
)

$ErrorActionPreference = 'Stop'
$RepoRoot = (Resolve-Path -LiteralPath $RepoRoot).Path
$patchPath = Join-Path (Split-Path -Parent $PSScriptRoot) 'patches/luxcore-bikini.patch'
$luxcorePath = Join-Path $RepoRoot 'extern/luxcore'
$windowsLibPath = Join-Path $RepoRoot 'lib/windows_x64'

function Invoke-Git {
    param([string[]]$Arguments)
    & git @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "git command failed with exit code ${LASTEXITCODE}: git $($Arguments -join ' ')"
    }
}

function Ensure-ExternalCheckout {
    param(
        [string]$Path,
        [string]$Url,
        [string]$Commit
    )

    if (Test-Path -LiteralPath $Path) {
        Write-Host "Keeping existing directory: $Path"
        return
    }

    Invoke-Git -Arguments @('clone', '--no-checkout', $Url, $Path)
    Invoke-Git -Arguments @('-C', $Path, 'checkout', '--detach', $Commit)
}

if (-not (Test-Path -LiteralPath (Join-Path $RepoRoot '.gitmodules'))) {
    throw "RepoRoot is not a BIKINI source checkout: $RepoRoot"
}
if (-not (Test-Path -LiteralPath $patchPath)) {
    throw "LuxCore patch missing beside this script: $patchPath"
}

Invoke-Git -Arguments @('-C', $RepoRoot, 'submodule', 'update', '--init', 'extern/luxcore')

& git -C $luxcorePath apply --reverse --check $patchPath 2>$null
if ($LASTEXITCODE -eq 0) {
    Write-Host 'LuxCore patch is already applied.'
}
else {
    Invoke-Git -Arguments @('-C', $luxcorePath, 'apply', '--check', $patchPath)
    Invoke-Git -Arguments @('-C', $luxcorePath, 'apply', $patchPath)
}

$previousSkipSmudge = $env:GIT_LFS_SKIP_SMUDGE
try {
    $env:GIT_LFS_SKIP_SMUDGE = '1'
    Invoke-Git -Arguments @('-C', $RepoRoot, '-c', 'submodule.lib/windows_x64.update=checkout',
        'submodule', 'update', '--progress', '--init', 'lib/windows_x64')
}
finally {
    $env:GIT_LFS_SKIP_SMUDGE = $previousSkipSmudge
}
Invoke-Git -Arguments @('-C', $windowsLibPath, 'lfs', 'pull')

Ensure-ExternalCheckout -Path (Join-Path $windowsLibPath 'spectra') `
    -Url 'https://github.com/yixuan/spectra.git' `
    -Commit 'bdd707b9a872bb17622696ed3cca2a27b743a5b7'
Ensure-ExternalCheckout -Path (Join-Path $windowsLibPath 'optix') `
    -Url 'https://github.com/NVIDIA/optix-dev.git' `
    -Commit 'f60c1e44f18426f426a2ed948f28515b3cf67b8a'
Ensure-ExternalCheckout -Path (Join-Path $windowsLibPath 'dlss') `
    -Url 'https://github.com/NVIDIA/DLSS.git' `
    -Commit '374959484e79a640feaba44c93ac8cfb0a03f5b5'

$dlssPath = Join-Path $windowsLibPath 'dlss'
if (Test-Path -LiteralPath (Join-Path $dlssPath '.git')) {
    Invoke-Git -Arguments @('-C', $dlssPath, 'submodule', 'update', '--init', '--recursive')
}

Write-Host 'Source dependencies are ready. Run make.bat deps in extern/luxcore, then make.bat full 2022.'
