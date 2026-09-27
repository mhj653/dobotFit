@echo off
setlocal
cd /d "%~dp0"

set "CONDA_PY=%USERPROFILE%\miniconda3\envs\dobot-sim\python.exe"
if not exist "%CONDA_PY%" (
    echo Missing conda environment: %CONDA_PY%
    echo Create it with: conda env create -f environment.yml
    exit /b 1
)

"%CONDA_PY%" -m PyInstaller --noconfirm RobotAutomationStudio.spec
dist\RobotAutomationStudio\RobotAutomationStudio.exe --self-check
echo Build output: dist\RobotAutomationStudio\RobotAutomationStudio.exe
