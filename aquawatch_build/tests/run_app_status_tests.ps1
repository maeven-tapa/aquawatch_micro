$ErrorActionPreference = 'Stop'
$statusSketch = [System.IO.File]::ReadAllText((Join-Path $PSScriptRoot '../aquawatch_build.ino'))
$statusStart = $statusSketch.IndexOf('void sendAppStatus() {')
$statusEnd = $statusSketch.IndexOf('void serviceStrobe() {', $statusStart)
if ($statusStart -lt 0 -or $statusEnd -le $statusStart) { throw 'Status/alarm functions not found' }
$statusHeader = Join-Path $env:TEMP 'app_status_under_test.h'
[System.IO.File]::WriteAllText($statusHeader, $statusSketch.Substring($statusStart, $statusEnd - $statusStart))
$statusExe = Join-Path $env:TEMP 'aquawatch-test-app-status.exe'
$statusObj = Join-Path $env:TEMP 'aquawatch-test-app-status.obj'
& cl /nologo /EHsc /std:c++14 /W4 (Join-Path $PSScriptRoot 'test_app_status.cpp') "/I$env:TEMP" "/Fe:$statusExe" "/Fo:$statusObj"
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $statusExe
exit $LASTEXITCODE
