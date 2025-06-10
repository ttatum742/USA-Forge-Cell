@echo off
REM Batch file to activate derivative.venv and run torque_derivative.py

SET VENV_NAME=derivative.venv
SET PYTHON_PROGRAM=torque_derivative.py

echo Checking for virtual environment...
if not exist "%VENV_NAME%\Scripts\activate.bat" (
    echo Virtual environment not found at %VENV_NAME%
    echo Creating virtual environment...
    python -m venv %VENV_NAME%
    echo Installing required packages...
    call "%VENV_NAME%\Scripts\activate.bat"
    pip install numpy cpppo
) else (
    echo Found virtual environment at %VENV_NAME%
)

echo Activating virtual environment...
call "%VENV_NAME%\Scripts\activate.bat"

echo Checking for additional requirements...
if exist "requirements.txt" (
    echo Installing from requirements.txt...
    pip install -r requirements.txt
)

echo Running torque derivative calculation...
echo -----------------------------------
python %PYTHON_PROGRAM%
echo -----------------------------------
echo Program finished executing.

pause