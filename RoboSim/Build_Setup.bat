@echo off
echo Current directory: %CD%

:: Copying files to Debug directory
xcopy /Y "%CD%\libs\assimp\assimp-vc142-mtd.dll" "%CD%\..\x64\Debug\"
xcopy /Y "%CD%\robosim_settings.txt" "%CD%\..\x64\Debug\" 
xcopy /Y "%CD%\imgui.ini" "%CD%\..\x64\Debug\" 
xcopy /Y /s "%CD%\libs\YMconnect\*" "%CD%\..\x64\Debug\" 
xcopy /Y /s "%CD%\3D Samples\*" "%CD%\..\x64\Debug\3D Samples\"
xcopy /Y /s "%CD%\assets\*" "%CD%\..\x64\Debug\assets\"
xcopy /Y /s "%CD%\RobotStandard\*" "%CD%\..\x64\Debug\RobotStandard\"
xcopy /Y /s "%CD%\shaders\*" "%CD%\..\x64\Debug\shaders\"
xcopy /Y /s "%CD%\RobFonts\*" "%CD%\..\x64\Debug\RobFonts\"
echo Debug files copied successfully.

:: Copying files to Release directory
xcopy /Y "%CD%\libs\assimp\assimp-vc142-mtd.dll" "%CD%\..\x64\Release\"
xcopy /Y "%CD%\robosim_settings.txt" "%CD%\..\x64\Release\" 
xcopy /Y "%CD%\imgui.ini" "%CD%\..\x64\Release\" 
xcopy /Y /s "%CD%\libs\YMconnect\*" "%CD%\..\x64\Release\" 
xcopy /Y /s "%CD%\3D Samples\*" "%CD%\..\x64\Release\3D Samples\"
xcopy /Y /s "%CD%\assets\*" "%CD%\..\x64\Release\assets\"
xcopy /Y /s "%CD%\RobotStandard\*" "%CD%\..\x64\Release\RobotStandard\"
xcopy /Y /s "%CD%\shaders\*" "%CD%\..\x64\Release\shaders\"
xcopy /Y /s "%CD%\RobFonts\*" "%CD%\..\x64\Release\RobFonts\"
echo Release files copied successfully.
