[CmdletBinding()]
param(
    [switch]$IncludeWsl,
    [string]$WslDistribution = "",
    [ValidateRange(3, 60)]
    [int]$WslTimeoutSeconds = 12
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Continue"
$commands = @("python", "cmake", "docker", "colcon", "ros2", "gz", "gazebo", "rviz2")


function Format-DisplayCommand {
    param(
        [Parameter(Mandatory)]
        [string]$FilePath,
        [Parameter(Mandatory)]
        [string[]]$ArgumentList
    )

    $displayArguments = foreach ($argument in $ArgumentList) {
        "'" + $argument.Replace("'", "''") + "'"
    }
    return (($FilePath) + " " + ($displayArguments -join " ")).Trim()
}


function Invoke-BoundedPowerShellJob {
    param(
        [Parameter(Mandatory)]
        [string]$FilePath,
        [Parameter(Mandatory)]
        [string[]]$ArgumentList,
        [Parameter(Mandatory)]
        [int]$TimeoutSeconds
    )

    $displayCommand = Format-DisplayCommand -FilePath $FilePath -ArgumentList $ArgumentList
    $invocation = [pscustomobject]@{
        FilePath = $FilePath
        ArgumentList = @($ArgumentList)
    }
    $job = Start-Job -ScriptBlock {
        param($ExactInvocation)
        try {
            $nativeOutput = @(
                & $ExactInvocation.FilePath @($ExactInvocation.ArgumentList) 2>&1
            )
            [pscustomobject]@{
                ExitCode = $LASTEXITCODE
                Output = ($nativeOutput | Out-String)
                Error = ""
            }
        }
        catch {
            [pscustomobject]@{
                ExitCode = $null
                Output = ""
                Error = $_.Exception.Message
            }
        }
    } -ArgumentList $invocation

    try {
        $completed = Wait-Job -Job $job -Timeout $TimeoutSeconds
        if ($null -eq $completed) {
            Stop-Job -Job $job -ErrorAction SilentlyContinue
            return [ordered]@{
                command = $displayCommand
                status = "UNVERIFIED_TIMEOUT"
                exit_code = $null
                stdout = ""
                stderr = "PowerShell job exceeded the read-only audit timeout."
            }
        }
        $jobResult = Receive-Job -Job $job
        $exitCode = $jobResult.ExitCode
        $output = (($jobResult.Output | Out-String) -replace ([char]0), "").Trim()
        $errorText = (($jobResult.Error | Out-String) -replace ([char]0), "").Trim()
        return [ordered]@{
            command = $displayCommand
            status = if ($exitCode -eq 0) { "PASS" } else { "UNVERIFIED_NONZERO_EXIT" }
            exit_code = $exitCode
            stdout = $output
            stderr = $errorText
        }
    }
    finally {
        Remove-Job -Job $job -Force -ErrorAction SilentlyContinue
    }
}


function Invoke-BoundedReadOnlyProcess {
    param(
        [Parameter(Mandatory)]
        [string]$FilePath,
        [Parameter(Mandatory)]
        [string[]]$ArgumentList,
        [Parameter(Mandatory)]
        [int]$TimeoutSeconds
    )

    $displayCommand = Format-DisplayCommand -FilePath $FilePath -ArgumentList $ArgumentList
    $startInfo = [System.Diagnostics.ProcessStartInfo]::new()
    $startInfo.FileName = $FilePath
    $startInfo.UseShellExecute = $false
    $startInfo.CreateNoWindow = $true
    $startInfo.RedirectStandardOutput = $true
    $startInfo.RedirectStandardError = $true

    if ($null -eq $startInfo.PSObject.Properties["ArgumentList"]) {
        return Invoke-BoundedPowerShellJob -FilePath $FilePath -ArgumentList $ArgumentList -TimeoutSeconds $TimeoutSeconds
    }
    foreach ($argument in $ArgumentList) {
        [void]$startInfo.ArgumentList.Add($argument)
    }

    $process = [System.Diagnostics.Process]::new()
    $process.StartInfo = $startInfo
    try {
        if (-not $process.Start()) {
            throw "process did not start"
        }
        $stdoutTask = $process.StandardOutput.ReadToEndAsync()
        $stderrTask = $process.StandardError.ReadToEndAsync()
        if (-not $process.WaitForExit($TimeoutSeconds * 1000)) {
            try {
                $process.Kill($true)
            }
            catch {
                $process.Kill()
            }
            $process.WaitForExit()
            return [ordered]@{
                command = $displayCommand
                status = "UNVERIFIED_TIMEOUT"
                exit_code = $null
                stdout = ($stdoutTask.GetAwaiter().GetResult() -replace ([char]0), "").Trim()
                stderr = ($stderrTask.GetAwaiter().GetResult() -replace ([char]0), "").Trim()
            }
        }

        $stdout = ($stdoutTask.GetAwaiter().GetResult() -replace ([char]0), "").Trim()
        $stderr = ($stderrTask.GetAwaiter().GetResult() -replace ([char]0), "").Trim()
        return [ordered]@{
            command = $displayCommand
            status = if ($process.ExitCode -eq 0) { "PASS" } else { "UNVERIFIED_NONZERO_EXIT" }
            exit_code = $process.ExitCode
            stdout = $stdout
            stderr = $stderr
        }
    }
    catch {
        return [ordered]@{
            command = $displayCommand
            status = "UNVERIFIED_START_FAILED"
            exit_code = $null
            stdout = ""
            stderr = $_.Exception.Message
        }
    }
    finally {
        $process.Dispose()
    }
}


$operatingSystem = Get-CimInstance Win32_OperatingSystem |
    Select-Object Caption, Version, OSArchitecture
$videoControllers = @(
    Get-CimInstance Win32_VideoController |
        Select-Object Name, DriverVersion, DriverDate, Status
)
$volumes = @(
    Get-Volume |
        Where-Object DriveLetter |
        Select-Object DriveLetter, FileSystem, Size, SizeRemaining
)
$commandAudit = foreach ($name in $commands) {
    $resolved = Get-Command $name -ErrorAction SilentlyContinue
    [ordered]@{
        command = $name
        available = $null -ne $resolved
        path = if ($resolved) { $resolved.Source } else { $null }
    }
}

$report = [ordered]@{
    audited_at = (Get-Date).ToString("o")
    timezone = (Get-TimeZone).Id
    host = [ordered]@{
        operating_system = $operatingSystem
        video_controllers = $videoControllers
        volumes = $volumes
        commands = @($commandAudit)
    }
    wsl_host_metadata = [ordered]@{
        requested = [bool]$IncludeWsl
        status = "NOT_REQUESTED"
    }
    wsl_guest_metadata = [ordered]@{
        status = "NOT_REQUESTED"
        distribution = $null
        probe = $null
    }
}

if ($IncludeWsl) {
    $wslCommand = Get-Command wsl.exe -ErrorAction SilentlyContinue
    if ($null -eq $wslCommand) {
        $report.wsl_host_metadata = [ordered]@{
            requested = $true
            status = "UNVERIFIED_WSL_EXE_NOT_FOUND"
        }
        $report.wsl_guest_metadata.status = "UNVERIFIED_WSL_EXE_NOT_FOUND"
    }
    else {
        $wslPath = $wslCommand.Source
        $statusResult = Invoke-BoundedReadOnlyProcess -FilePath $wslPath -ArgumentList @("--status") -TimeoutSeconds $WslTimeoutSeconds
        $versionResult = Invoke-BoundedReadOnlyProcess -FilePath $wslPath -ArgumentList @("--version") -TimeoutSeconds $WslTimeoutSeconds
        $listVerboseResult = Invoke-BoundedReadOnlyProcess -FilePath $wslPath -ArgumentList @("--list", "--verbose") -TimeoutSeconds $WslTimeoutSeconds
        $listQuietResult = Invoke-BoundedReadOnlyProcess -FilePath $wslPath -ArgumentList @("--list", "--quiet") -TimeoutSeconds $WslTimeoutSeconds

        $registeredDistributions = @()
        if ($listQuietResult.status -eq "PASS") {
            $registeredDistributions = @(
                $listQuietResult.stdout -split "\r?\n" |
                    ForEach-Object { $_.Trim() } |
                    Where-Object { $_ }
            )
        }

        $defaultDistribution = $null
        if ($listVerboseResult.status -eq "PASS") {
            $defaultMatch = [regex]::Match(
                $listVerboseResult.stdout,
                "(?m)^\s*\*\s*(?<name>.+?)\s{2,}(?<state>Running|Stopped)\s+(?<version>[12])\s*$"
            )
            if ($defaultMatch.Success) {
                $candidate = $defaultMatch.Groups["name"].Value.Trim()
                if ($registeredDistributions -contains $candidate) {
                    $defaultDistribution = $candidate
                }
            }
        }
        $selectedDistribution = $null
        $selectionError = $null
        if (-not [string]::IsNullOrWhiteSpace($WslDistribution)) {
            if ($registeredDistributions -contains $WslDistribution) {
                $selectedDistribution = $WslDistribution
            }
            else {
                $selectionError = "UNVERIFIED_REQUESTED_DISTRIBUTION_NOT_REGISTERED"
            }
        }
        elseif ($null -ne $defaultDistribution) {
            $selectedDistribution = $defaultDistribution
        }
        if ($null -eq $selectedDistribution -and $null -eq $selectionError) {
            $selectedDistribution = $registeredDistributions |
                Where-Object { $_ -match "^Ubuntu(?:-|$)" } |
                Select-Object -First 1
        }

        $report.wsl_host_metadata = [ordered]@{
            requested = $true
            status = if (
                $statusResult.status -eq "PASS" -and
                $versionResult.status -eq "PASS" -and
                $listVerboseResult.status -eq "PASS" -and
                $listQuietResult.status -eq "PASS"
            ) { "PASS" } else { "PARTIAL_OR_UNVERIFIED" }
            wsl_status = $statusResult
            wsl_version = $versionResult
            list_verbose = $listVerboseResult
            list_quiet = $listQuietResult
            registered_distributions = $registeredDistributions
            default_distribution = $defaultDistribution
            selected_distribution = $selectedDistribution
        }

        if ($null -ne $selectionError) {
            $report.wsl_guest_metadata = [ordered]@{
                status = $selectionError
                distribution = $WslDistribution
                probe = $null
            }
        }
        elseif ($null -eq $selectedDistribution) {
            $report.wsl_guest_metadata = [ordered]@{
                status = "UNVERIFIED_NO_STARTABLE_UBUNTU_DISTRIBUTION_SELECTED"
                distribution = $null
                probe = $null
            }
        }
        else {
            $guestProbe = Invoke-BoundedReadOnlyProcess -FilePath $wslPath -ArgumentList @(
                "-d",
                [string]$selectedDistribution,
                "--",
                "sh",
                "-lc",
                "cat /etc/os-release; uname -a"
            ) -TimeoutSeconds $WslTimeoutSeconds
            $report.wsl_guest_metadata = [ordered]@{
                status = $guestProbe.status
                distribution = [string]$selectedDistribution
                probe = $guestProbe
            }
        }
    }
}

$report | ConvertTo-Json -Depth 10
