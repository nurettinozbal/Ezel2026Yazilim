@echo off
setlocal EnableExtensions EnableDelayedExpansion

cd /d "%~dp0"

set "ROOT_DIR=%CD%"
set "BACKEND_DIR=%ROOT_DIR%\backend"
set "BACKEND_VENV=%BACKEND_DIR%\.venv"
set "BACKEND_PY=%BACKEND_VENV%\Scripts\python.exe"
set "REQUIREMENTS_FILE=%BACKEND_DIR%\requirements.txt"
set "REQUIREMENTS_STAMP=%BACKEND_VENV%\.requirements-installed"

if not defined BACKEND_HOST set "BACKEND_HOST=0.0.0.0"
if not defined BACKEND_PORT set "BACKEND_PORT=5000"
if not defined FRONTEND_HOST set "FRONTEND_HOST=0.0.0.0"
if not defined FRONTEND_PORT set "FRONTEND_PORT=5173"
if not defined VITE_WS_URL set "VITE_WS_URL=ws://localhost:%BACKEND_PORT%/ws"

echo [EZEL GCS] Windows launcher starting...

call :find_python
if errorlevel 1 goto :fail

where npm >nul 2>nul
if errorlevel 1 (
  echo [EZEL GCS] ERROR: npm is required but not installed or not in PATH.
  goto :fail
)

call :ensure_port_available "%BACKEND_PORT%" "Backend"
if errorlevel 1 goto :fail
call :ensure_port_available "%FRONTEND_PORT%" "Frontend"
if errorlevel 1 goto :fail

call :sync_tokens
if errorlevel 1 goto :fail

if not defined EZEL_IDA_PORT (
  echo [EZEL GCS] WARNING: EZEL_IDA_PORT is not set. Set it to the correct Windows COM port, for example COM3.
)
if not defined EZEL_IHA_PORT (
  echo [EZEL GCS] WARNING: EZEL_IHA_PORT is not set. Set it to the correct Windows COM port, for example COM4.
)

if not exist "%BACKEND_PY%" (
  echo [EZEL GCS] Creating backend virtual environment...
  %PYTHON_CMD% -m venv "%BACKEND_VENV%"
  if errorlevel 1 goto :fail
)

set "INSTALL_BACKEND_DEPS=0"
if not exist "%REQUIREMENTS_STAMP%" set "INSTALL_BACKEND_DEPS=1"
if "%INSTALL_BACKEND_DEPS%"=="0" (
  powershell -NoProfile -ExecutionPolicy Bypass -Command "if ((Get-Item '%REQUIREMENTS_FILE%').LastWriteTimeUtc -gt (Get-Item '%REQUIREMENTS_STAMP%').LastWriteTimeUtc) { exit 1 } else { exit 0 }" >nul 2>nul
  if errorlevel 1 set "INSTALL_BACKEND_DEPS=1"
)

if "%INSTALL_BACKEND_DEPS%"=="1" (
  echo [EZEL GCS] Installing backend Python dependencies...
  "%BACKEND_PY%" -m pip install --upgrade pip
  if errorlevel 1 goto :fail
  "%BACKEND_PY%" -m pip install -r "%REQUIREMENTS_FILE%"
  if errorlevel 1 goto :fail
  type nul > "%REQUIREMENTS_STAMP%"
)

if not exist "%ROOT_DIR%\node_modules" (
  echo [EZEL GCS] Installing frontend Node dependencies...
  npm ci
  if errorlevel 1 goto :fail
)

if not exist "%ROOT_DIR%\node_modules\.bin\vite.cmd" (
  echo [EZEL GCS] ERROR: Vite command not found after dependency check.
  goto :fail
)

echo [EZEL GCS] Frontend URL: http://localhost:%FRONTEND_PORT%
echo [EZEL GCS] Backend health: http://localhost:%BACKEND_PORT%/health
echo [EZEL GCS] WebSocket URL: %VITE_WS_URL%
echo [EZEL GCS] Opening backend and frontend windows...

start "EZEL GCS Backend" /D "%BACKEND_DIR%" cmd /k ""%BACKEND_PY%" -m uvicorn main:app --host %BACKEND_HOST% --port %BACKEND_PORT%"
start "EZEL GCS Frontend" /D "%ROOT_DIR%" cmd /k ""%ROOT_DIR%\node_modules\.bin\vite.cmd" --host %FRONTEND_HOST% --port %FRONTEND_PORT% --strictPort"

echo.
echo [EZEL GCS] Started. Close the backend/frontend windows or press Ctrl+C in each one to stop.
echo [EZEL GCS] Press any key to close this launcher window.
pause >nul
exit /b 0

:find_python
py -3 --version >nul 2>nul
if not errorlevel 1 (
  set "PYTHON_CMD=py -3"
  exit /b 0
)
python --version >nul 2>nul
if not errorlevel 1 (
  set "PYTHON_CMD=python"
  exit /b 0
)
echo [EZEL GCS] ERROR: Python 3 is required but not installed or not in PATH.
exit /b 1

:ensure_port_available
netstat -ano -p tcp | findstr /R /C:":%~1 .*LISTENING" >nul 2>nul
if not errorlevel 1 (
  echo [EZEL GCS] ERROR: %~2 port %~1 is already in use. Stop the old service or set another port.
  exit /b 1
)
exit /b 0

:sync_tokens
if defined EZEL_WS_TOKEN if defined VITE_WS_TOKEN if not "%EZEL_WS_TOKEN%"=="%VITE_WS_TOKEN%" (
  echo [EZEL GCS] ERROR: EZEL_WS_TOKEN and VITE_WS_TOKEN do not match. Commands would be rejected.
  exit /b 1
)

if not defined EZEL_WS_TOKEN if not defined VITE_WS_TOKEN (
  set "GENERATED_TOKEN="
  for /f "usebackq delims=" %%T in (`powershell -NoProfile -ExecutionPolicy Bypass -Command "[Guid]::NewGuid().ToString('N') + [Guid]::NewGuid().ToString('N').Substring(0,16)" 2^>nul`) do set "GENERATED_TOKEN=%%T"
  if not defined GENERATED_TOKEN set "GENERATED_TOKEN=ezel-%RANDOM%-%RANDOM%-%RANDOM%"
  set "EZEL_WS_TOKEN=!GENERATED_TOKEN!"
  set "VITE_WS_TOKEN=!GENERATED_TOKEN!"
  echo [EZEL GCS] Generated a temporary WebSocket command token for this launch.
  exit /b 0
)

if not defined EZEL_WS_TOKEN set "EZEL_WS_TOKEN=%VITE_WS_TOKEN%"
if not defined VITE_WS_TOKEN set "VITE_WS_TOKEN=%EZEL_WS_TOKEN%"
exit /b 0

:fail
echo.
echo [EZEL GCS] Launcher stopped.
pause
exit /b 1
