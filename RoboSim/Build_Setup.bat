@echo off
echo Current directory: %CD%
xcopy /Y "%CD%\libs\assimp\assimp-vc142-mtd.dll" "%CD%\..\x64\Debug\"
xcopy /Y "%CD%\libs\YMconnect\YMConnect.dll" "%CD%\..\x64\Debug\"
xcopy /Y "%CD%\imgui.ini" "%CD%\..\x64\Debug\" 

mkdir /s "%CD%\..\x64\Debug\3D Samples"
xcopy /Y /s "%CD%\3D Samples\*" "%CD%\..\x64\Debug\3D Samples\"

echo samples files copied successfully.