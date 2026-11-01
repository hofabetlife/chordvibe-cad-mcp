[Console]::OutputEncoding = [System.Text.Encoding]::UTF8
$Host.UI.RawUI.WindowTitle = "MCP-CAD Studio Launcher"

Write-Host "====================================================================" -ForegroundColor Cyan
Write-Host "  MCP-CAD Engine -- Unified Windows + Web Studio Launcher" -ForegroundColor Cyan
Write-Host "====================================================================" -ForegroundColor Cyan
Write-Host ""

Set-Location $PSScriptRoot
$exePath = Join-Path $PSScriptRoot "target\release\mcp-cad-server.exe"
$npm = Get-Command npm -ErrorAction SilentlyContinue

if ($npm) {
    Write-Host "[OK] Node.js и npm обнаружены!" -ForegroundColor Green

    # Запуск фонового Rust демона на порту 8080 если бинарник скомпилирован
    if (Test-Path $exePath) {
        Write-Host "[OK] Запуск фонового демона MCP-CAD на http://127.0.0.1:8080 ..." -ForegroundColor Yellow
        Start-Process -FilePath $exePath -ArgumentList "--mode", "sse", "--port", "8080" -WindowStyle Hidden
    }

    # Проверка node_modules
    $nodeModules = Join-Path $PSScriptRoot "node_modules"
    if (-not (Test-Path $nodeModules)) {
        Write-Host "[INFO] Установка зависимостей Web Studio (npm install)..." -ForegroundColor Cyan
        npm install
    }

    Write-Host "[OK] Запуск Единого интерфейса MCP-CAD Studio на http://localhost:3000 ..." -ForegroundColor Green
    Start-Job -ScriptBlock {
        Start-Sleep -Seconds 3
        try {
            Start-Process "msedge.exe" -ArgumentList "--app=http://localhost:3000/?bridge=live", "--window-size=1440,900" -ErrorAction Stop
        } catch {
            Start-Process "http://localhost:3000/?bridge=live"
        }
    } | Out-Null
    npm run dev
} else {
    Write-Host "====================================================================" -ForegroundColor Yellow
    Write-Host "  [ВНИМАНИЕ] Node.js не найден в вашей системе Windows!" -ForegroundColor Red
    Write-Host "====================================================================" -ForegroundColor Yellow
    Write-Host ""
    Write-Host "Для запуска интерактивного Web CAD Studio (с деревом параметров DAG," -ForegroundColor Yellow
    Write-Host "оптимизатором операций ИИ, 28 MCP-инструментами и кинематикой)" -ForegroundColor Yellow
    Write-Host "необходим Node.js." -ForegroundColor Yellow
    Write-Host ""
    Write-Host "Как исправить:" -ForegroundColor Green
    Write-Host "  1. Скачайте и установите Node.js (LTS): https://nodejs.org/" -ForegroundColor Green
    Write-Host "  2. При установке нажимайте 'Next' (установка занимает 1 минуту)." -ForegroundColor Green
    Write-Host "  3. После установки перезапустите этот скрипт!" -ForegroundColor Green
    Write-Host ""

    if (Test-Path $exePath) {
        Write-Host "[1] Открыть https://nodejs.org/ для скачивания (Рекомендуется)" -ForegroundColor Cyan
        Write-Host "[2] Запустить автономное базовое окно WGPU (без веб-панелей)" -ForegroundColor Cyan
        $choice = Read-Host "Ваш выбор (1 или 2, по умолчанию 1)"
        if ($choice -eq "2") {
            Write-Host "[INFO] Запуск автономного окна WGPU..." -ForegroundColor Cyan
            & $exePath --gui
            exit
        }
    }

    Start-Process "https://nodejs.org/"
    Write-Host "Браузер открыт на странице скачивания Node.js." -ForegroundColor Cyan
    Write-Host "После установки закройте это окно и запустите скрипт снова." -ForegroundColor Yellow
    Read-Host "Нажмите Enter для завершения..."
}
