@echo off
chcp 65001 >nul
cd /d "%~dp0"
echo === [1/3] 平面物理验证 ===
node tests\validate_physics.mjs
echo.
echo === [2/3] 球面物理验证 ===
node tests\validate_spherical.mjs
echo.
echo === [3/3] 页面冒烟测试(模拟浏览器完整链路)==
node --import ./tests/stubs/register.mjs tests/smoke.mjs
echo.
echo === 全部结束后按任意键退出 ===
pause >nul
