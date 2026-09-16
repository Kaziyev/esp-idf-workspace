@echo off
where py >nul 2>nul
if errorlevel 1 goto use_python
py -3 "%~dp0scripts\serve_dashboard.py" --open
goto done
:use_python
python "%~dp0scripts\serve_dashboard.py" --open
:done
if errorlevel 1 pause
