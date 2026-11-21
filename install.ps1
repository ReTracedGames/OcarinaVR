# OcarinaVR installer (Windows). Builds the game with Hookshot from YOUR OWN copy, with the VR mode in it.
#
#   powershell -ExecutionPolicy Bypass -File install.ps1 -Game "C:\path\your-game.nup"
#   powershell -ExecutionPolicy Bypass -File install.ps1 -Romfs "C:\path\romfs" -Exefs "C:\path\exefs"
#   powershell -ExecutionPolicy Bypass -File install.ps1 -CoreOnly      (after an update: rebuilds retraced.dll only)
#
# What it does, all inside this folder (nothing is installed on the system):
#   1. downloads Hookshot's own build tools (its builder parts), llvm-mingw and Khronos's OpenXR loader into
#      _build\downloads, and checks each file's SHA-256;
#   2. downloads Hookshot's source at the revision this mod was written for, into _build\hookshot;
#   3. adds this mod's hook lines to it (hooks.json, tools\apply-hooks.ps1);
#   4. builds the game from your copy with Hookshot's own script (10 to 45 minutes, about 60 GB free needed);
#   5. builds retraced.dll, copies the OpenXR loader, and reads Link's arms and skeleton from your game's files.
# No game file is downloaded, and none leaves your PC.
param(
    [string]$Game = '',
    [string]$Romfs = '',
    [string]$Exefs = '',
    [string]$Language = 'en-US',
    [switch]$CoreOnly
)
$ErrorActionPreference = 'Stop'
$root = $PSScriptRoot
$build = Join-Path $root '_build'
$dl = Join-Path $build 'downloads'
$tools = Join-Path $build 'tools'
$hsRoot = Join-Path $build 'hookshot-src'   # (the source archive is unpacked here: one folder inside)
$hs = ''
$out = Join-Path $root 'game'

$hookshotRev = 'bbdc3704678556a3e6d939b17c00ebacc1e25d27'
$hookshotZip = "https://git.notrexed.dev/Hookshot/Hookshot/archive/$hookshotRev.zip"
$partsUrl = 'https://git.notrexed.dev/Hookshot/Hookshot/releases/download/builder-parts'   # (the parts of Hookshot v2.1.0, the release next to that revision)
$parts = @(
    @{ name = 'python';      file = 'HookshotBuilder-python-a495c4573bc6.zip';      sha = 'a495c4573bc6865a24dedb13e203190f0e145a854ffd03470bcd6a7e5196663b'; url = "$partsUrl/HookshotBuilder-python-a495c4573bc6.zip" },
    @{ name = 'cpm';         file = 'HookshotBuilder-cpm-c30c77e4cf00.zip';         sha = 'c30c77e4cf000ebf11ec82e79ace4bef3206d3698c61146f7477baa428ea7ef0'; url = "$partsUrl/HookshotBuilder-cpm-c30c77e4cf00.zip" },
    @{ name = 'third_party'; file = 'HookshotBuilder-third_party-5817b9f7f2a6.zip'; sha = '5817b9f7f2a60fce478eff07b0b6d891b205e90464d94fd21e98c156f9c4a1a0'; url = "$partsUrl/HookshotBuilder-third_party-5817b9f7f2a6.zip" },
    @{ name = 'wgpu-native'; file = 'HookshotBuilder-wgpu-native-a3b3d8253fba.zip'; sha = 'a3b3d8253fbaedf5975172994c0b45202fe2e4ff89a42ff5bfad53f9468cf498'; url = "$partsUrl/HookshotBuilder-wgpu-native-a3b3d8253fba.zip" },
    @{ name = 'app';         file = 'HookshotBuilder-app-d592ca8a73f4.zip';         sha = 'd592ca8a73f4ad7b881cb85c63d989757dccbc3058b4d9296a59c6442ea7fcfb'; url = "$partsUrl/HookshotBuilder-app-d592ca8a73f4.zip" },
    @{ name = 'llvm-mingw';  file = 'llvm-mingw-20261104-ucrt-x86_64.zip';          sha = '552191b5754c49a5a20723276a6fc56541ea1922c4c50b7b669022333fc1b40f'; url = 'https://github.com/mstorsjo/llvm-mingw/releases/download/20261104/llvm-mingw-20261104-ucrt-x86_64.zip' },
    @{ name = 'openxr';      file = 'OpenXR.Loader.1.1.65.nupkg';                   sha = '05638c526f07ba3a5f1084bbc3308d654d5167f844ffa1e35515325cb72e0c34'; url = 'https://github.com/KhronosGroup/OpenXR-SDK/releases/download/release-1.1.65/OpenXR.Loader.1.1.65.nupkg' }
)

# (Windows' own tar and curl, by their full path: another tar on the PATH, Git's for one, does not unpack zip files)
$tar = Join-Path $env:SystemRoot 'System32\tar.exe'
$curl = Join-Path $env:SystemRoot 'System32\curl.exe'
if (-not (Test-Path $tar) -or -not (Test-Path $curl)) { throw 'Windows 10 (1803 or newer) or Windows 11 is needed: tar.exe and curl.exe were not found in System32.' }

function Say([string]$t) { Write-Host ''; Write-Host "== $t" -ForegroundColor Cyan }
function Fetch([string]$url, [string]$file) {
    & $curl -L --fail --silent --show-error --retry 3 -o $file $url
    if ($LASTEXITCODE -ne 0) { throw "download failed: $url" }
}
function Unpack([string]$file, [string]$dir) {
    New-Item -ItemType Directory -Force $dir | Out-Null
    & $tar -xf $file -C $dir
    if ($LASTEXITCODE -ne 0) { throw "could not unpack $file" }
}

if (-not $CoreOnly -and -not $Game -and -not ($Romfs -and $Exefs)) {
    # (started by a double-click: the game file is asked for in a window)
    Add-Type -AssemblyName System.Windows.Forms
    $dlg = New-Object System.Windows.Forms.OpenFileDialog
    $dlg.Title = 'OcarinaVR: choose YOUR copy of the game (.nup file, version 1.1.1)'
    $dlg.Filter = 'Game file (*.nup)|*.nup|All files (*.*)|*.*'
    if ($dlg.ShowDialog() -ne [System.Windows.Forms.DialogResult]::OK) { throw 'No game file chosen. See README.md.' }
    $Game = $dlg.FileName
}
if ($Game -and -not (Test-Path -LiteralPath $Game)) { throw "game file not found: $Game" }
New-Item -ItemType Directory -Force $dl, $tools | Out-Null

Say 'Build tools (downloaded once, checked by SHA-256)'
foreach ($p in $parts) {
    $file = Join-Path $dl $p.file
    $dir = Join-Path $tools $p.name
    $done = Join-Path $dir '.unpacked'
    if (Test-Path $done) { "  $($p.name): ready"; continue }
    if (-not (Test-Path $file)) { "  $($p.name): downloading"; Fetch $p.url $file }
    $sha = (Get-FileHash -Algorithm SHA256 -LiteralPath $file).Hash.ToLower()
    if ($sha -ne $p.sha) { Move-Item -LiteralPath $file -Destination ($file + '.bad') -Force; throw "$($p.file): wrong SHA-256 ($sha); the file was set aside as .bad, run the installer again" }
    "  $($p.name): unpacking"
    Unpack $file $dir
    Set-Content -LiteralPath $done -Value $p.sha -Encoding ascii
}
$py = Join-Path $tools 'python\HookshotBuilder\tools\python\python.exe'
$mingw = Join-Path $tools 'llvm-mingw\llvm-mingw-20261104-ucrt-x86_64'
$cpm = Join-Path $tools 'cpm\HookshotBuilder\tools\cpm'
$wgpu = Join-Path $tools 'wgpu-native\HookshotBuilder\tools\wgpu-native'
$xr = Join-Path $tools 'openxr'
foreach ($need in $py, (Join-Path $mingw 'bin'), $cpm, $wgpu, (Join-Path $xr 'include')) {
    if (-not (Test-Path $need)) { throw "a build tool is not where it was expected: $need" }
}

if (-not $CoreOnly) {
    Say "Hookshot's source ($($hookshotRev.Substring(0, 7)))"
    $inner = if (Test-Path $hsRoot) { Get-ChildItem $hsRoot -Directory | Select-Object -First 1 } else { $null }
    if (-not $inner -or -not (Test-Path (Join-Path $inner.FullName 'nx2\make_game.py'))) {
        $zip = Join-Path $dl "hookshot-$($hookshotRev.Substring(0, 7)).zip"
        if (-not (Test-Path $zip)) { '  downloading'; Fetch $hookshotZip $zip }
        '  unpacking'
        Unpack $zip $hsRoot
        $inner = Get-ChildItem $hsRoot -Directory | Select-Object -First 1
        $hs = $inner.FullName
    } else { '  already there' }
    $hs = $inner.FullName
    # what a clone does not carry (Hookshot's readme, "Build from source"): prebuilt third-party files and the work folder
    $tp = Join-Path $tools 'third_party\HookshotBuilder\tools\third_party'
    foreach ($f in Get-ChildItem $tp -Recurse -File -Include *.dll, *.exe, *.lib, *.sha1) {
        $to = Join-Path (Join-Path $hs 'third_party') $f.FullName.Substring($tp.Length + 1)
        New-Item -ItemType Directory -Force (Split-Path -Parent $to) | Out-Null
        Copy-Item -LiteralPath $f.FullName -Destination $to -Force
    }
    $wk = Join-Path $tools 'app\HookshotBuilder\tools\work'
    New-Item -ItemType Directory -Force (Join-Path $hs 'work') | Out-Null
    Copy-Item (Join-Path $wk '*') (Join-Path $hs 'work') -Recurse -Force

    Say 'The VR mode''s hook lines'
    & (Join-Path $root 'tools\apply-hooks.ps1') -Hookshot $hs
    if (-not $?) { throw 'the hook lines could not be put in' }

    Say 'Building the game from your copy (10 to 45 minutes; you can use your PC meanwhile)'
    $env:PYTHONIOENCODING = 'utf-8'
    $env:HOOKSHOT_MINGW = $mingw
    $env:RX_CPM = $cpm
    $env:WGPU_NATIVE = $wgpu
    $env:HOOKSHOT_KEEP_BUILD_FILES = '1'
    $env:HOOKSHOT_NO_UPDATE_CHECK = '1'
    $a = @('-I', 'nx2\make_game.py', '--work', (Join-Path $build 'work'), '--log', (Join-Path $build 'build.log'), '--out', $out, '--language', $Language)
    if ($Game) { $a += @('--nup', (Resolve-Path -LiteralPath $Game).Path) } else { $a += @('--romfs', (Resolve-Path -LiteralPath $Romfs).Path, '--exefs', (Resolve-Path -LiteralPath $Exefs).Path) }
    Push-Location $hs
    try { & $py @a; $code = $LASTEXITCODE } finally { Pop-Location }
    if ($code -ne 0) { throw "the game did not build (see _build\build.log)" }
}
if (-not (Test-Path (Join-Path $out 'hookshot.exe'))) { throw "no game in $out yet: run the installer with your game first" }

Say 'The VR mode itself (retraced.dll)'
$vk = Get-ChildItem (Join-Path $cpm 'vulkanheaders') -Directory | Sort-Object Name | Select-Object -Last 1
& (Join-Path $root 'tools\build-core.ps1') -Mingw $mingw -Vulkan (Join-Path $vk.FullName 'include') -OpenXR $xr -Out $out
if ($LASTEXITCODE -ne 0) { throw 'retraced.dll did not build' }

Say "Link's arms and skeleton, read from your game's files"
$env:RETRACED_GAME_DIR = $out
& $py -I (Join-Path $root 'tools\vr-corps.py')
if ($LASTEXITCODE -ne 0) { throw 'tools\vr-corps.py failed' }

Say 'Done'
'Put your headset on, start its PC link (Quest Link, SteamVR...), then double-click "Play OcarinaVR.bat".'
