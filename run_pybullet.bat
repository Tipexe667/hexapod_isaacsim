@echo off
chcp 65001 >nul
echo ============================================================================
echo   LANCEMENT DU CENTRE DE SIMULATION ROBOTIQUE (MENTORPI ^& HEXAPODE)
echo ============================================================================
cd /d "%~dp0"

set "PY_EXE=python"
if exist "%LOCALAPPDATA%\Programs\Python\Python312\python.exe" (
    set "PY_EXE=%LOCALAPPDATA%\Programs\Python\Python312\python.exe"
) else if exist "%LOCALAPPDATA%\Programs\Python\Python311\python.exe" (
    set "PY_EXE=%LOCALAPPDATA%\Programs\Python\Python311\python.exe"
)

echo [1/2] Verification des librairies (PyBullet + NumPy)...
"%PY_EXE%" -c "import pybullet, numpy" 2>nul
if errorlevel 1 (
    echo Installation de PyBullet et NumPy en cours...
    "%PY_EXE%" -m pip install --upgrade pip
    "%PY_EXE%" -m pip install -r requirements.txt
)

echo [2/2] Ouverture du Menu de Selection de Simulation (MentorPi / Hexapode)...
"%PY_EXE%" sim_launcher.py %*
pause
