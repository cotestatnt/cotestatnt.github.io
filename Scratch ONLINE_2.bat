SETLOCAL EnableExtensions
set EXE=laragon.exe
FOR /F %%x IN ('tasklist /NH /FI "IMAGENAME eq %EXE%"') DO IF %%x == %EXE% goto FOUND
start %~dp0\Webserver\laragon.exe
goto FIN
:FOUND
echo Webserver is running
:FIN

start %~dp0\Firefox\App\Firefox\firefox.exe "http://localhost/?url=http://cotestatnt.github.io/arduino_xmas.js&lang=it#scratch"
exit 