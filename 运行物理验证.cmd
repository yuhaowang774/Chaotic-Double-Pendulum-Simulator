@echo off
chcp 65001 >nul
cd /d "%~dp0"
echo === 双摆物理验证(node tests\validate_physics.mjs)==
node tests\validate_physics.mjs
echo.
echo === 若上方显示三个 PASS 与“全部物理验证通过”即物理核心验证成功 ===
pause
