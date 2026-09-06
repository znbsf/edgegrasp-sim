[CmdletBinding()]
param(
    [string]$Destination = "$env:USERPROFILE\Documents\edgegrasp-tools",
    [ValidatePattern('^4\.5\.\d+$')][string]$Version = '4.5.13'
)
$ErrorActionPreference = 'Stop'
$archiveName = "blender-$Version-windows-x64.zip"
$release = 'https://download.blender.org/release/Blender4.5'
New-Item -ItemType Directory -Force -Path $Destination | Out-Null
$archive = Join-Path $Destination $archiveName
$manifest = Join-Path $Destination "blender-$Version.sha256"
Invoke-WebRequest "$release/blender-$Version.sha256" -OutFile $manifest -UseBasicParsing
$entry = Get-Content $manifest | Where-Object { $_ -match ([regex]::Escape($archiveName) + '$') }
if (@($entry).Count -ne 1) { throw 'Expected one official archive checksum' }
$expected = ($entry -split '\s+')[0].ToLowerInvariant()
if (!(Test-Path -LiteralPath $archive)) {
    & curl.exe --fail --location --retry 3 --output "$archive.part" "$release/$archiveName"
    if ($LASTEXITCODE -ne 0) { throw 'Blender download failed' }
    if ((Get-FileHash "$archive.part" -Algorithm SHA256).Hash.ToLowerInvariant() -ne $expected) {
        throw 'Blender archive checksum mismatch'
    }
    Move-Item -LiteralPath "$archive.part" -Destination $archive
}
if ((Get-FileHash $archive -Algorithm SHA256).Hash.ToLowerInvariant() -ne $expected) {
    throw 'Blender archive checksum mismatch'
}
$directory = Join-Path $Destination "blender-$Version-windows-x64"
if (!(Test-Path -LiteralPath $directory)) { Expand-Archive -LiteralPath $archive -DestinationPath $Destination }
$executable = Join-Path $directory 'blender.exe'
if (!(Test-Path -LiteralPath $executable)) { throw 'Blender executable missing' }
& $executable --version
if ($LASTEXITCODE -ne 0) { throw 'Blender version check failed' }
@{ version=$Version; source="$release/$archiveName"; sha256=$expected; executable=$executable } |
    ConvertTo-Json | Set-Content (Join-Path $Destination 'blender-install.json') -Encoding UTF8
Write-Output $executable
