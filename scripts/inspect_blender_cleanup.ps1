[CmdletBinding()]
param(
    [Parameter(Mandatory)][string]$Plan,
    [string]$Output
)
# Read-only inventory. It deliberately has no deletion or apply mode.
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

function Get-AbsoluteCleanupPath([string]$Value) {
    $expanded = [Environment]::ExpandEnvironmentVariables($Value)
    if ($expanded -match '%[^%]+%' -or ![IO.Path]::IsPathRooted($expanded)) {
        throw "Expected absolute path with resolved variables: $Value"
    }
    return [IO.Path]::GetFullPath($expanded).TrimEnd('\', '/')
}

function Test-PathOverlap([string]$First, [string]$Second) {
    return $First.Equals($Second, [StringComparison]::OrdinalIgnoreCase) -or
        $First.StartsWith($Second + '\', [StringComparison]::OrdinalIgnoreCase) -or
        $Second.StartsWith($First + '\', [StringComparison]::OrdinalIgnoreCase)
}

$spec = Get-Content -LiteralPath $Plan -Raw | ConvertFrom-Json
if ($spec.schema_version -ne 1) { throw 'Unsupported cleanup plan version' }
$protected = @($spec.protected | ForEach-Object { Get-AbsoluteCleanupPath $_ })
if (!$protected.Count) { throw 'Protected paths must be explicit' }
$protected += Get-AbsoluteCleanupPath (Split-Path -Parent $PSScriptRoot)
$rows = @()
$seenPaths = @()
foreach ($candidate in $spec.candidates) {
    $absolute = Get-AbsoluteCleanupPath $candidate.path
    if (@($seenPaths | Where-Object { Test-PathOverlap $absolute $_ }).Count) {
        throw "Overlapping candidates would double-count bytes: $absolute"
    }
    $seenPaths += $absolute
    $blockers = @()
    if (@($protected | Where-Object { Test-PathOverlap $absolute $_ }).Count) {
        $blockers += 'protected_path_overlap'
    }
    if ($absolute -eq [IO.Path]::GetPathRoot($absolute).TrimEnd('\', '/')) {
        $blockers += 'filesystem_root'
    }
    if ($candidate.kind -notin @('download_archive', 'failed_install', 'intermediate_replay')) {
        $blockers += 'unknown_category'
    }
    [long]$bytes = 0
    [long]$files = 0
    $exists = Test-Path -LiteralPath $absolute
    # Never descend into a protected tree or a reparse point while counting.
    if ($exists -and !$blockers.Count) {
        $pending = [Collections.Generic.Stack[string]]::new()
        $pending.Push($absolute)
        while ($pending.Count) {
            $entry = Get-Item -LiteralPath $pending.Pop() -Force
            if ($entry.Attributes -band [IO.FileAttributes]::ReparsePoint) {
                $blockers += 'reparse_point'
                continue
            }
            if ($entry.PSIsContainer) {
                foreach ($child in Get-ChildItem -LiteralPath $entry.FullName -Force) {
                    $pending.Push($child.FullName)
                }
            } else {
                $bytes += $entry.Length
                $files++
                if ($entry.Extension -in @('.mcap', '.db3')) { $blockers += 'raw_recording_present' }
            }
        }
    }
    $rows += [PSCustomObject]@{
        path=$absolute; kind=$candidate.kind; reason=$candidate.reason; exists=$exists
        measured_bytes=$bytes; files=$files; blockers=@($blockers | Select-Object -Unique)
        review_candidate=($exists -and !$blockers.Count)
    }
}
[long]$total = 0
foreach ($row in $rows) { if ($row.review_candidate) { $total += $row.measured_bytes } }
$report = [ordered]@{
    schema_version=1; mode='inventory_only'; deletion_authorized_by_report=$false
    generated_at_utc=[DateTime]::UtcNow.ToString('o'); protected=$protected
    candidates=$rows; candidate_bytes=$total; candidate_gib=[math]::Round($total/1GB,3)
    removed_bytes=0
}
$json = $report | ConvertTo-Json -Depth 6
if ($Output) {
    # Exclusive creation prevents an inventory refresh overwriting previous evidence.
    $stream = [IO.File]::Open([IO.Path]::GetFullPath($Output), [IO.FileMode]::CreateNew)
    try {
        $encoded = [Text.UTF8Encoding]::new($false).GetBytes($json + "`n")
        $stream.Write($encoded, 0, $encoded.Length)
    } finally { $stream.Dispose() }
}
Write-Output $json
