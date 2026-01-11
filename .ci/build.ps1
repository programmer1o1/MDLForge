param(
    [ValidateSet('Debug', 'Release', 'RelWithDebInfo', 'MinSizeRel')]
    [string]$Configuration = 'Release',
    [string]$BuildDir = 'build',
    [string[]]$ConfigureArgs = @(),
    [string[]]$BuildArgs = @()
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$RepoRoot = Resolve-Path (Join-Path $PSScriptRoot '..')
$BuildDirPath = Join-Path $RepoRoot $BuildDir

& cmake -S $RepoRoot -B $BuildDirPath -A Win32 -DSTUDIOMDL_REQUIRE_32BIT=ON @ConfigureArgs
& cmake --build $BuildDirPath --config $Configuration @BuildArgs
