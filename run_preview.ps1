Write-Host "Starting MCP-CAD 3D Viewport..." -ForegroundColor Cyan
& "$PSScriptRoot\target\release\mcp-cad-server.exe" --gui --mode sse --port 8080
