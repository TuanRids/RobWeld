@echo off
echo Current directory: %CD%

xcopy /Y "%CD%\libs\assimp\assimp-vc142-mtd.dll" "%CD%\..\x64\Debug\"
xcopy /Y "%CD%\robosim_ini.dat" "%CD%\..\x64\Debug\" 
xcopy /Y "%CD%\imgui.ini" "%CD%\..\x64\Debug\" 
xcopy /Y /s "%CD%\libs\YMconnect\*" "%CD%\..\x64\Debug\" 
xcopy /Y /s "%CD%\3D Samples\*" "%CD%\..\x64\Debug\3D Samples\"
xcopy /Y /s "%CD%\assets\*" "%CD%\..\x64\Debug\assets\"
xcopy /Y /s "%CD%\RobotStandard\*" "%CD%\..\x64\Debug\RobotStandard\"
echo Debug files copied successfully.

xcopy /Y "%CD%\libs\assimp\assimp-vc142-mtd.dll" "%CD%\..\x64\Release\"
xcopy /Y "%CD%\robosim_ini.dat" "%CD%\..\x64\Release\" 
xcopy /Y "%CD%\imgui.ini" "%CD%\..\x64\Release\" 
xcopy /Y /s "%CD%\libs\YMconnect\*" "%CD%\..\x64\Release\" 
xcopy /Y /s "%CD%\3D Samples\*" "%CD%\..\x64\Release\3D Samples\"
xcopy /Y /s "%CD%\assets\*" "%CD%\..\x64\Release\assets\"
xcopy /Y /s "%CD%\RobotStandard\*" "%CD%\..\x64\Debug\RobotStandard\"
xcopy /Y /s "%CD%\shaders\*" "%CD%\..\x64\Release\shaders\"
echo Release files copied successfully.