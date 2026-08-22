@echo off

set BLENDER_INSTALL_DIRECTORY=%~dp0
set BLENDER_VERSION_FOLDER=%BLENDER_INSTALL_DIRECTORY%5.3
set PYTHON_BIN=%BLENDER_VERSION_FOLDER%\python\bin\python.exe

if exist "%PYTHON_BIN%" (
    "%PYTHON_BIN%" -I "%BLENDER_VERSION_FOLDER%\scripts\modules\_bpy_internal\system_info\url_prefill_startup.py"
    exit /b
)

echo ERROR: Failed to find python executable at: %PYTHON_BIN%
echo Possible causes include:
echo - Your Blender installation is corrupt or missing python.exe.
echo - The location or name of python.exe has changed.
pause
