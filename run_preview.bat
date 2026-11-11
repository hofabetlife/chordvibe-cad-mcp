@echo off
chcp 65001 >nul
setlocal enabledelayedexpansion

title MCP-CAD Studio Launcher

echo ====================================================================
echo   MCP-CAD Engine -- Unified Windows + Web Studio Launcher
echo ====================================================================
echo.

cd /d "%~dp0"
set "EXE_PATH=%~dp0target\release\mcp-cad-server.exe"

:: 1. Проверяем наличие Node.js (npm)
where npm >nul 2>nul
if %errorlevel% neq 0 goto NO_NODE

echo [OK] Node.js и npm найдены!
echo.

:: 2. Запускаем фоновый Rust MCP-демон, если бинарник скомпилирован
if exist "%EXE_PATH%" (
    echo [OK] Найден Rust CAD kernel daemon: target\release\mcp-cad-server.exe
    echo [INFO] Запуск фонового MCP-CAD SSE демона на порту 8080...
    start "MCP-CAD-Daemon" /min "%EXE_PATH%" --mode sse --port 8080
    timeout /t 1 /nobreak >nul
) else (
    echo [INFO] Скомпилированный mcp-cad-server.exe не найден.
    echo [INFO] Web Studio будет работать на встроенном WGPU-паритетном ядре.
)

:: 3. Проверяем node_modules
if not exist "%~dp0node_modules" (
    echo [INFO] Установка зависимостей Web Studio... Это займет около 30 секунд...
    call npm install
    if %errorlevel% neq 0 (
        echo [ERROR] Ошибка при установке npm пакетов!
        goto FAIL_PAUSE
    )
)

:: 4. Открываем веб-интерфейс в браузере / приложении
echo.
echo [OK] Запуск Web Studio на http://localhost:3000 ...
echo [INFO] Открытие в окне приложения (Edge / Chrome / Браузер по умолчанию)...
start "" cmd /c "timeout /t 3 /nobreak >nul & start msedge --app=http://localhost:3000/?bridge=live --window-size=1440,900 2>nul || start chrome --app=http://localhost:3000/?bridge=live 2>nul || start http://localhost:3000/?bridge=live"

:: 5. Запускаем dev сервер Vite
call npm run dev
if %errorlevel% neq 0 goto FAIL_PAUSE
goto END

:NO_NODE
echo ====================================================================
echo   [ВНИМАНИЕ] Node.js не найден в вашей системе Windows!
echo ====================================================================
echo.
echo Для запуска интерактивного Web CAD Studio (с деревом параметров DAG,
echo оптимизатором операций ИИ, 28 MCP-инструментами и кинематикой)
echo необходим Node.js.
echo.
echo Как исправить:
echo   1. Скачайте и установите Node.js (LTS): https://nodejs.org/
echo   2. При установке нажимайте "Next" (установка занимает 1 минуту).
echo   3. После установки перезапустите run_preview.bat!
echo.
echo ====================================================================
echo.

if exist "%EXE_PATH%" (
    echo [1] Открыть сайт https://nodejs.org/ для загрузки Node.js (Рекомендуется)
    echo [2] Запустить автономное базовое окно WGPU (без веб-панелей Studio)
    echo.
    set "CHOICE="
    set /p "CHOICE=Введите 1 или 2 (Enter = 1): "
    if "!CHOICE!"=="2" (
        echo [INFO] Запуск автономного окна WGPU...
        "%EXE_PATH%" --gui
        goto END
    )
)

start https://nodejs.org/
echo Браузер открыт на странице скачивания Node.js.
echo После установки закройте это окно и запустите run_preview.bat снова.
goto FAIL_PAUSE

:FAIL_PAUSE
echo.
echo Нажмите любую клавишу для закрытия этого окна...
pause >nul

:END
