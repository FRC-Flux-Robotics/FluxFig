@echo off
setlocal EnableDelayedExpansion
cd /d "%~dp0\.."
set "ROOT_DIR=%CD%"

set "JDK_BIN="
where javac >nul 2>nul
if %ERRORLEVEL% equ 0 (
    set "JAVAC_CMD=javac"
    set "JAVA_CMD=java"
    set "JAR_CMD=jar"
) else (
    for %%Y in (2027 2026 2025) do (
        if exist "C:\Users\Public\wpilib\%%Y\jdk\bin\javac.exe" (
            set "JDK_BIN=C:\Users\Public\wpilib\%%Y\jdk\bin"
            goto :found_jdk
        )
    )
    echo [ERROR] Could not find JDK 17+ in PATH or C:\Users\Public\wpilib\202*\jdk\bin
    exit /b 1
)

:found_jdk
if defined JDK_BIN (
    set "JAVAC_CMD=%JDK_BIN%\javac.exe"
    set "JAVA_CMD=%JDK_BIN%\java.exe"
    set "JAR_CMD=%JDK_BIN%\jar.exe"
)

if exist "build\classes\main" rmdir /s /q "build\classes\main"
if exist "build\classes\test" rmdir /s /q "build\classes\test"
mkdir "build\classes\main"
mkdir "build\classes\test"
if not exist "build\libs" mkdir "build\libs"
if not exist "dist" mkdir "dist"

echo [1/3] Compiling FluxFig (Java 17 bytecode, zero external dependencies)...
dir /s /b src\main\java\*.java > build\sources-main.txt
"%JAVAC_CMD%" --release 17 -encoding UTF-8 -Xlint:all,-serial -Werror -d build\classes\main @build\sources-main.txt
if errorlevel 1 exit /b 1

dir /s /b src\test\java\*.java > build\sources-test.txt
"%JAVAC_CMD%" --release 17 -encoding UTF-8 -Xlint:all,-serial -Werror -cp build\classes\main -d build\classes\test @build\sources-test.txt
if errorlevel 1 exit /b 1

echo [2/3] Running FluxFig unit, safety ^& Google Sheet sync tests...
"%JAVA_CMD%" -ea -cp "build\classes\main;build\classes\test" frc.lib.fluxfig.FluxFigTest
if errorlevel 1 exit /b 1

echo [3/3] Packaging build\libs\fluxfig-1.0.0.jar ^& dist\fluxfig.jar...
"%JAR_CMD%" --create --date=1980-01-01T00:00:02Z --file build\libs\fluxfig-1.0.0.jar --main-class frc.lib.fluxfig.FluxFigMain -C build\classes\main .
copy /y build\libs\fluxfig-1.0.0.jar dist\fluxfig.jar >nul

echo Done! Library ^& CLI JAR: %ROOT_DIR%\dist\fluxfig.jar
