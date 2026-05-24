# PID 调试前置检查与地面闭环验证脚本
# 用法: .\tools\pid_precheck_and_ground_verify.ps1 -Serial COM8
#
# 功能:
#   1. 读取 motor_scale_m1~m4 / motor_offset_m1~m4，确认全为 1.0 / 0.0
#   2. 保存参数到 NVS，避免旧 trim 重启恢复
#   3. 读取当前 rate PID 参数作为基线
#   4. 执行 attitude-ground-round 地面姿态闭环验证
#   5. 打印验证结果

param(
    [string]$Serial = "COM8",
    [float]$TargetDeg = 1.0,
    [float]$BaseDuty = 0.08
)

$ErrorActionPreference = "Continue"
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RepoRoot = Split-Path -Parent $ScriptDir
$ToolsDir = Join-Path $RepoRoot "tools"

Set-Location $ToolsDir

function Run-Cli {
    param([string[]]$Args)
    $cmd = "python -m esp_drone_cli.cli.main --serial $Serial $($Args -join ' ')"
    Write-Host "  [RUN] $cmd" -ForegroundColor DarkGray
    Invoke-Expression $cmd 2>&1
}

function Get-ParamValue {
    param([string]$ParamName)
    $output = python -m esp_drone_cli.cli.main --serial $Serial get $ParamName 2>&1
    $match = ($output | Select-String -Pattern "$ParamName\s*=\s*([-\d.eE+]+)").Matches
    if ($match.Count -gt 0) {
        return $match.Groups[1].Value
    }
    return $null
}

Write-Host ""
Write-Host ("=" * 70)
Write-Host "  PID 调试前置检查 — 地面闭环验证"
Write-Host "  串口: $Serial | 目标角度: $TargetDeg deg | Base Duty: $BaseDuty"
Write-Host "  时间: $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')"
Write-Host ("=" * 70)
Write-Host ""

# ============================================================
# Step 0: 连接并获取设备信息
# ============================================================
Write-Host "[Step 0] 连接设备并获取信息..." -ForegroundColor Cyan
$deviceInfo = Run-Cli @("connect")
Write-Host $deviceInfo
Write-Host ""

# ============================================================
# Step 1: 读取 motor_scale / motor_offset 参数
# ============================================================
Write-Host "[Step 1/5] 读取 motor_scale 和 motor_offset 参数..." -ForegroundColor Cyan

$motorScaleParams = @("motor_scale_m1", "motor_scale_m2", "motor_scale_m3", "motor_scale_m4")
$motorOffsetParams = @("motor_offset_m1", "motor_offset_m2", "motor_offset_m3", "motor_offset_m4")

$allScaleOk = $true
$allOffsetOk = $true

Write-Host ""
Write-Host "  --- motor_scale ---" -ForegroundColor DarkGray
foreach ($p in $motorScaleParams) {
    $v = Get-ParamValue $p
    Write-Host "  $p = $v"
    if ($v -and [math]::Abs([float]$v - 1.0) -gt 0.001) {
        $allScaleOk = $false
    }
}

Write-Host "  --- motor_offset ---" -ForegroundColor DarkGray
foreach ($p in $motorOffsetParams) {
    $v = Get-ParamValue $p
    Write-Host "  $p = $v"
    if ($v -and [math]::Abs([float]$v - 0.0) -gt 0.0001) {
        $allOffsetOk = $false
    }
}

Write-Host ""
if ($allScaleOk -and $allOffsetOk) {
    Write-Host "  ✓ motor_scale 全为 1.0, motor_offset 全为 0.0" -ForegroundColor Green
} else {
    Write-Host "  ⚠ 存在非默认值:" -ForegroundColor Yellow
    if (-not $allScaleOk) { Write-Host "     motor_scale 有值不为 1.0" -ForegroundColor Yellow }
    if (-not $allOffsetOk) { Write-Host "     motor_offset 有值不为 0.0" -ForegroundColor Yellow }
}

# ============================================================
# Step 2: 修正异常参数并保存
# ============================================================
Write-Host ""
Write-Host "[Step 2/5] 修正参数并保存..." -ForegroundColor Cyan

if (-not $allScaleOk) {
    Write-Host "  设置 motor_scale_m1~m4 = 1.0 ..."
    Run-Cli @("set", "motor_scale_m1", "float", "1.0")
    Run-Cli @("set", "motor_scale_m2", "float", "1.0")
    Run-Cli @("set", "motor_scale_m3", "float", "1.0")
    Run-Cli @("set", "motor_scale_m4", "float", "1.0")
}

if (-not $allOffsetOk) {
    Write-Host "  设置 motor_offset_m1~m4 = 0.0 ..."
    Run-Cli @("set", "motor_offset_m1", "float", "0.0")
    Run-Cli @("set", "motor_offset_m2", "float", "0.0")
    Run-Cli @("set", "motor_offset_m3", "float", "0.0")
    Run-Cli @("set", "motor_offset_m4", "float", "0.0")
}

Write-Host "  保存参数到 NVS..."
Run-Cli @("save")
Write-Host "  ✓ 已保存，旧 motor-trim-estimate 补偿结果不会在重启后恢复" -ForegroundColor Green

# ============================================================
# Step 3: 读取 rate PID 参数
# ============================================================
Write-Host ""
Write-Host "[Step 3/5] 读取 rate PID 参数（基线）..." -ForegroundColor Cyan
Write-Host ""

$ratePidParams = @(
    "rate_kp_roll", "rate_ki_roll", "rate_kd_roll",
    "rate_kp_pitch", "rate_ki_pitch", "rate_kd_pitch",
    "rate_kp_yaw", "rate_ki_yaw", "rate_kd_yaw"
)

Write-Host "  Rate PID:" -ForegroundColor DarkGray
foreach ($p in $ratePidParams) {
    $v = Get-ParamValue $p
    Write-Host "  $p = $v"
}

Write-Host ""
Write-Host "  Attitude 外环 + 安全边界:" -ForegroundColor DarkGray
$outerParams = @(
    "ground_att_kp_roll", "ground_att_kp_pitch",
    "ground_att_rate_limit_roll", "ground_att_rate_limit_pitch",
    "ground_att_target_limit_deg", "ground_att_error_deadband_deg",
    "ground_test_base_duty", "ground_test_max_extra_duty",
    "ground_test_motor_balance_limit", "ground_test_ramp_duty_per_s",
    "ground_test_auto_disarm_ms"
)
foreach ($p in $outerParams) {
    $v = Get-ParamValue $p
    Write-Host "  $p = $v"
}

# ============================================================
# Step 4: 地面姿态闭环验证
# ============================================================
Write-Host ""
Write-Host ("=" * 70)
Write-Host "[Step 4/5] 地面姿态闭环验证 — attitude-ground-round" -ForegroundColor Cyan
Write-Host ""
Write-Host "  ⚠ 重要提醒:" -ForegroundColor Yellow
Write-Host "    1. 无人机 +Z 朝上，放置在硬质水平面上"
Write-Host "    2. 桨叶已安装，无人机必须被约束/固定"
Write-Host "    3. 脚本会自动 arm → 执行验证 → disarm"
Write-Host "    4. 电机将旋转，注意安全"
Write-Host "    5. 仅验证 rate PID 控制方向，不调 I/D"
Write-Host ""
Write-Host ("=" * 70)

$confirm = Read-Host "  确认无误，输入 yes 继续"
if ($confirm -ne "yes") {
    Write-Host "  已取消。" -ForegroundColor Yellow
    exit 0
}

Write-Host ""
Write-Host "  执行 attitude-ground-round (target=$TargetDeg deg, base_duty=$BaseDuty)..." -ForegroundColor Yellow
Write-Host ""

# 先确保 disarmed
Write-Host "  确保设备 disarmed..." -ForegroundColor DarkGray
try {
    python -m esp_drone_cli.cli.main --serial $Serial disarm 2>&1 | Out-Null
    Start-Sleep -Milliseconds 500
} catch {
    Write-Host "  disarm 调用失败（可能已经是 disarmed 状态）" -ForegroundColor DarkGray
}

# 执行验证
$sw = [System.Diagnostics.Stopwatch]::StartNew()
python -m esp_drone_cli.cli.main --serial $Serial attitude-ground-round `
    --target-deg $TargetDeg `
    --base-duty $BaseDuty `
    --auto-arm `
    --output-dir logs 2>&1
$exitCode = $LASTEXITCODE
$sw.Stop()

Write-Host ""
Write-Host "  验证耗时: $([math]::Round($sw.Elapsed.TotalSeconds, 1))s" -ForegroundColor DarkGray

# ============================================================
# Step 5: 结果分析
# ============================================================
Write-Host ""
Write-Host ("=" * 70)
Write-Host "[Step 5/5] 结果分析" -ForegroundColor Cyan
Write-Host ""

if ($exitCode -eq 0) {
    Write-Host "  ✓ 验证全部通过（PASS）" -ForegroundColor Green
    Write-Host "    - 所有控制方向 (roll+/roll-/pitch+/pitch-) sign_ok"
    Write-Host "    - validity / safety / yaw 全部正常"
    Write-Host "    - 无 clamp / saturation"
    Write-Host ""
    Write-Host "  → 可以继续 PID 调参" -ForegroundColor Green
} elseif ($exitCode -eq 2) {
    Write-Host "  ✗ 验证未通过（FAIL）" -ForegroundColor Red
    Write-Host "    - 查看上方输出的 segments 详情"
    Write-Host "    - 重点检查 sign_ok / outer_link_ok"
    Write-Host "    - 如果某轴方向越修越偏: 先检查轴向/mixer 符号"
    Write-Host "    - 如果高频振荡: 先降低对应 rate_kp"
    Write-Host ""
    Write-Host "  → 不要直接调大 PID，先排查控制方向" -ForegroundColor Red
} else {
    Write-Host "  ⚠ 运行异常 (exit code = $exitCode)" -ForegroundColor Yellow
}

# 查找最新生成的 CSV
Write-Host ""
Write-Host "  最新日志文件:" -ForegroundColor DarkGray
$latestCsv = Get-ChildItem -Path ".\logs\*_attitude_ground_verify_round.csv" -ErrorAction SilentlyContinue `
    | Sort-Object LastWriteTime -Descending | Select-Object -First 1
if ($latestCsv) {
    Write-Host "  $($latestCsv.FullName) ($([math]::Round($latestCsv.Length/1024, 1)) KB)"
    Write-Host ""
    Write-Host "  查看前 3 行:"
    Get-Content $latestCsv.FullName -TotalCount 3 | ForEach-Object { Write-Host "    $_" }
} else {
    Write-Host "  (未找到日志文件)"
}

Write-Host ""
Write-Host ("=" * 70)
Write-Host "  脚本执行完毕 — $(Get-Date -Format 'HH:mm:ss')" -ForegroundColor Green
Write-Host ("=" * 70)
Write-Host ""
Write-Host "  后续步骤建议:"
Write-Host "    1. 如果验证通过: 可小幅增加 rate_kp (如 0.000700 → 0.000800)"
Write-Host "    2. 如果 roll_pitch 某一方向 sign 不对: 检查 mixer 符号"
Write-Host "    3. 如果振荡: 降低对应 rate_kp (如 0.000700 → 0.000500)"
Write-Host "    4. 如果响应慢: 小幅增加 rate_kp (如 0.000700 → 0.000850)"
Write-Host "    5. 每次改参后重新运行本脚本验证"
Write-Host ""
