# MCP-CAD Scenario Runner (PowerShell)
# Sends JSON-RPC messages to mcp-cad-server over stdio and logs results/artifacts.

param(
    [Parameter(Mandatory=$false)]
    [string]$ScenarioFile = "",
    [Parameter(Mandatory=$false)]
    [string]$RunId = (Get-Date -Format "yyyyMMdd_HHmmss")
)

$ErrorActionPreference = "Stop"
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$WorkspaceRoot = Split-Path -Parent $ScriptDir
$ServerExe = Join-Path $WorkspaceRoot "target\release\mcp-cad-server.exe"

if (!(Test-Path $ServerExe)) {
    Write-Host "Building release mcp-cad-server..."
    cargo build --release -p mcp-cad-server
}

$ResultsDir = Join-Path $ScriptDir "results"
$ArtifactsDir = Join-Path $ScriptDir "artifacts"
if (!(Test-Path $ResultsDir)) { New-Item -ItemType Directory -Path $ResultsDir | Out-Null }
if (!(Test-Path $ArtifactsDir)) { New-Item -ItemType Directory -Path $ArtifactsDir | Out-Null }

$OutputFile = Join-Path $ResultsDir "$RunId.json"

function Invoke-McpRpc {
    param([string]$RequestJson)

    $psi = New-Object System.Diagnostics.ProcessStartInfo
    $psi.FileName = $ServerExe
    $psi.Arguments = "--mode stdio --headless"
    $psi.UseShellExecute = $false
    $psi.RedirectStandardInput = $true
    $psi.RedirectStandardOutput = $true
    $psi.RedirectStandardError = $true
    $psi.CreateNoWindow = $true

    $proc = [System.Diagnostics.Process]::Start($psi)
    $writer = $proc.StandardInput
    $writer.WriteLine($RequestJson)
    $writer.Flush()
    $writer.Close()

    $stdout = $proc.StandardOutput.ReadToEnd()
    $stderr = $proc.StandardError.ReadToEnd()
    $proc.WaitForExit()

    return @{
        ExitCode = $proc.ExitCode
        Stdout = $stdout.Trim()
        Stderr = $stderr.Trim()
    }
}

Write-Host "MCP-CAD Runner: Executing RunId $RunId"

# Sample ping check
$pingReq = '{"jsonrpc":"2.0","id":1,"method":"ping"}'
$res = Invoke-McpRpc -RequestJson $pingReq
Write-Host "Ping response: $($res.Stdout)"

$report = @{
    RunId = $RunId
    Timestamp = (Get-Date).ToString("o")
    Ping = $res.Stdout
}

$report | ConvertTo-Json -Depth 10 | Set-Content $OutputFile -Encoding UTF8
Write-Host "Results written to $OutputFile"
