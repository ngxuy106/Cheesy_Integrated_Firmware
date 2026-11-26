<# :
@echo off
chcp 65001 >nul
set "BAT_DIR=%~dp0"
set "BAT_FILE=%~f0"
set "BAT_ARG1=%~1"
powershell -NoProfile -ExecutionPolicy Bypass -Command "$c=[IO.File]::ReadAllText($env:BAT_FILE,[Text.Encoding]::UTF8); & ([scriptblock]::Create($c)) $env:BAT_DIR $env:BAT_ARG1"
exit /b %errorlevel%
#>
param(
    [string]$RootDir,
    [string]$Arg1
)

[Console]::OutputEncoding = [System.Text.Encoding]::UTF8
[Console]::InputEncoding  = [System.Text.Encoding]::UTF8
$ErrorActionPreference = "Stop"

Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing
[System.Windows.Forms.Application]::EnableVisualStyles()

$RobotCfgPath    = Join-Path $RootDir "Core\Inc\robot_config.h"
$PurePursuitPath = Join-Path $RootDir "Core\Inc\pure_pursuit.h"
$NapBatPath      = Join-Path $RootDir "NAP_CODE.bat"
$TunerIcoPath    = Join-Path $RootDir "icons\chinh_thong_so.ico"
$NapIcoPath      = Join-Path $RootDir "icons\nap_code.ico"
$LogsDir         = Join-Path $RootDir "logs"

$ParamSpecs = @(
    @{ Id=1;  Tab=0; Key="V1";     Label="Vận tốc Ô 1 - Khe hẹp xuất phát";       Unit="cm/s";   Min=10.0;  Max=30.0;    Inc=0.5;  Dec=1; Type="f1"; File="robot_config"; Macro="V_TARGET_O1_CMS" }
    @{ Id=2;  Tab=0; Key="V2";     Label="Vận tốc Ô 2 - Bo cua trái";             Unit="cm/s";   Min=10.0;  Max=30.0;    Inc=0.5;  Dec=1; Type="f1"; File="robot_config"; Macro="V_TARGET_O2_CMS" }
    @{ Id=3;  Tab=0; Key="V3";     Label="Vận tốc Ô 3 - Đoạn thẳng + bật ToF";    Unit="cm/s";   Min=10.0;  Max=30.0;    Inc=0.5;  Dec=1; Type="f1"; File="robot_config"; Macro="V_TARGET_O3_CMS" }
    @{ Id=4;  Tab=0; Key="V4";     Label="Vận tốc Ô 4 - Bo cua phải + quét ToF";  Unit="cm/s";   Min=10.0;  Max=30.0;    Inc=0.5;  Dec=1; Type="f1"; File="robot_config"; Macro="V_TARGET_O4_CMS" }
    @{ Id=5;  Tab=0; Key="V5";     Label="Vận tốc Ô 5 - Xuôi về đích";            Unit="cm/s";   Min=10.0;  Max=30.0;    Inc=0.5;  Dec=1; Type="f1"; File="robot_config"; Macro="V_TARGET_O5_CMS" }
    @{ Id=6;  Tab=0; Key="VDET";   Label="Trần tốc độ khi né vật cản động";       Unit="cm/s";   Min=10.0;  Max=30.0;    Inc=0.5;  Dec=1; Type="f1"; File="robot_config"; Macro="V_TARGET_DETOUR_MAX_CMS" }
    @{ Id=7;  Tab=0; Key="VMAX";   Label="Giới hạn tốc độ tối đa toàn xe";        Unit="cm/s";   Min=15.0;  Max=35.0;    Inc=0.5;  Dec=1; Type="f1"; File="robot_config"; Macro="ROBOT_MAX_SPEED_CMS" }
    @{ Id=8;  Tab=0; Key="ACC";    Label="Gia tốc tối đa chống trượt bánh";       Unit="cm/s²";  Min=20.0;  Max=150.0;   Inc=5.0;  Dec=1; Type="f1"; File="robot_config"; Macro="ROBOT_ACCEL_MAX_CMS2" }
    @{ Id=9;  Tab=1; Key="KP";     Label="Motor PI - Kp (trái & phải)";           Unit="-";      Min=10.0;  Max=600.0;   Inc=5.0;  Dec=1; Type="f1"; File="robot_config"; Macro="MOTOR_DEFAULT_KP" }
    @{ Id=10; Tab=1; Key="KI";     Label="Motor PI - Ki (tăng 1.5..4 nếu hụt)";   Unit="-";      Min=0.0;   Max=50.0;    Inc=0.1;  Dec=2; Type="f2"; File="robot_config"; Macro="MOTOR_DEFAULT_KI" }
    @{ Id=11; Tab=1; Key="WD";     Label="Đường kính bánh xe";                    Unit="cm";     Min=2.5;   Max=8.0;     Inc=0.05; Dec=2; Type="f2"; File="robot_config"; Macro="ROBOT_WHEEL_DIAM_CM" }
    @{ Id=12; Tab=1; Key="WB";     Label="Khoảng cách tâm 2 bánh";                Unit="cm";     Min=6.0;   Max=16.0;    Inc=0.05; Dec=2; Type="f2"; File="robot_config"; Macro="ROBOT_TRACK_WIDTH_CM" }
    @{ Id=13; Tab=1; Key="PPR";    Label="Số xung Encoder / vòng bánh (4x)";      Unit="ticks";  Min=100.0; Max=10000.0; Inc=10.0; Dec=0; Type="f0"; File="robot_config"; Macro="ROBOT_ENCODER_PPR" }
    @{ Id=14; Tab=2; Key="TOF";    Label="Tầm quét 3× VL53L0X";                   Unit="mm";     Min=150.0; Max=800.0;   Inc=10.0; Dec=0; Type="u";  File="robot_config"; Macro="TOF_SCAN_MAX_MM" }
    @{ Id=15; Tab=2; Key="SCANX";  Label="Tọa độ X bắt đầu bật quét ToF (Ô3=60)"; Unit="cm";     Min=0.0;   Max=120.0;   Inc=5.0;  Dec=1; Type="f1"; File="robot_config"; Macro="FSM_SCAN_TRIGGER_X_CM" }
    @{ Id=16; Tab=2; Key="LD";     Label="Pure Pursuit Lookahead offset";         Unit="cm";     Min=4.0;   Max=12.0;    Inc=0.5;  Dec=1; Type="f1"; File="pure_pursuit"; Macro="PP_LOOKAHEAD_OFFSET_CM" }
    @{ Id=17; Tab=2; Key="YAWINV"; Label="Đảo chiều Yaw MPU6050 (0=Thường,1=Đảo)"; Unit="0|1";    Min=0.0;   Max=1.0;     Inc=1.0;  Dec=0; Type="i";  File="robot_config"; Macro="IMU_DEFAULT_YAW_INVERT" }
)

$StaticPillars = @(
    @{ Name="T1";  X=5.0;   Y=15.0 }
    @{ Name="T2";  X=55.0;  Y=25.0 }
    @{ Name="T3";  X=30.0;  Y=35.0 }
    @{ Name="T4";  X=55.0;  Y=45.0 }
    @{ Name="T5";  X=5.0;   Y=55.0 }
    @{ Name="T6";  X=5.0;   Y=75.0 }
    @{ Name="T7";  X=35.0;  Y=90.0 }
    @{ Name="T8";  X=45.0;  Y=115.0 }
    @{ Name="T9";  X=65.0;  Y=75.0 }
    @{ Name="T10"; X=75.0;  Y=65.0 }
    @{ Name="T11"; X=85.0;  Y=100.0 }
    @{ Name="T12"; X=95.0;  Y=65.0 }
    @{ Name="T13"; X=115.0; Y=115.0 }
    @{ Name="T14"; X=125.0; Y=85.0 }
    @{ Name="T15"; X=145.0; Y=45.0 }
    @{ Name="T16"; X=145.0; Y=15.0 }
    @{ Name="T17"; X=175.0; Y=25.0 }
)

$FsmStateNames = @{
    0 = "FSM_IDLE (Chờ chạy)"
    1 = "FSM_RUN_O1_O2"
    2 = "FSM_RUN_SCAN_O3_O5"
    3 = "FSM_RECOVERY_BACKUP"
    4 = "FSM_MISSION_FINISHED"
    5 = "FSM_EMERGENCY_STOP"
}

function Ensure-Shortcuts {
    try {
        $ws = New-Object -ComObject WScript.Shell
        foreach ($it in @(
            @{ Lnk="NAP_CODE.lnk"; Bat="NAP_CODE.bat"; Ico=$NapIcoPath; Desc="Build va Nap firmware STM32F411CEU6 qua ST-Link" },
            @{ Lnk="CHINH_THONG_SO.lnk"; Bat="CHINH_THONG_SO.bat"; Ico=$TunerIcoPath; Desc="Dashboard xem quy dao 2D + chinh 17 thong so Cheesy Robot" }
        )) {
            $lnkPath = Join-Path $RootDir $it.Lnk
            if (-not (Test-Path $lnkPath)) {
                $sc = $ws.CreateShortcut($lnkPath)
                $sc.TargetPath = Join-Path $RootDir $it.Bat
                $sc.WorkingDirectory = $RootDir
                if (Test-Path $it.Ico) { $sc.IconLocation = "$($it.Ico),0" }
                $sc.Description = $it.Desc
                $sc.Save()
            }
        }
    } catch {}
}

function Format-CfgValue([double]$Val, [string]$Type) {
    $inv = [System.Globalization.CultureInfo]::InvariantCulture
    switch ($Type) {
        "f0" { return ([math]::Round($Val)).ToString("0", $inv) }
        "f1" { return $Val.ToString("0.0", $inv) }
        "f2" { return $Val.ToString("0.00", $inv) }
        "u"  { return ([int][math]::Round($Val)).ToString($inv) }
        "i"  { return ([int][math]::Round($Val)).ToString($inv) }
        default { return $Val.ToString($inv) }
    }
}

function Format-CDefineValue([double]$Val, [string]$Type) {
    $inv = [System.Globalization.CultureInfo]::InvariantCulture
    switch ($Type) {
        "f0" { return $Val.ToString("0.0", $inv) + "f" }
        "f1" { return $Val.ToString("0.0", $inv) + "f" }
        "f2" { return $Val.ToString("0.00", $inv) + "f" }
        "u"  { return ([int][math]::Round($Val)).ToString($inv) + "U" }
        "i"  { return ([int][math]::Round($Val)).ToString($inv) + "U" }
    }
}

function Read-CodeDefaults {
    $vals = [ordered]@{}
    $txtRobot = [IO.File]::ReadAllText($RobotCfgPath, [Text.Encoding]::UTF8)
    $txtPP    = [IO.File]::ReadAllText($PurePursuitPath, [Text.Encoding]::UTF8)
    foreach ($p in $ParamSpecs) {
        $txt = if ($p.File -eq "pure_pursuit") { $txtPP } else { $txtRobot }
        $pat = "(?m)^\s*#define\s+" + [regex]::Escape($p.Macro) + "\s+([+-]?\d+(?:\.\d+)?)[fFuU]*"
        $m = [regex]::Match($txt, $pat)
        if ($m.Success) {
            $vals[$p.Key] = [double]::Parse($m.Groups[1].Value, [System.Globalization.CultureInfo]::InvariantCulture)
        } else {
            throw "Khong doc duoc macro $($p.Macro)"
        }
    }
    return $vals
}

function Write-CodeParam([hashtable]$Spec, [double]$NewVal) {
    $path = if ($Spec.File -eq "pure_pursuit") { $PurePursuitPath } else { $RobotCfgPath }
    $txt  = [IO.File]::ReadAllText($path, [Text.Encoding]::UTF8)
    $cVal = Format-CDefineValue $NewVal $Spec.Type
    $pat  = "(?m)^(\s*#define\s+" + [regex]::Escape($Spec.Macro) + "\s+)([+-]?\d+(?:\.\d+)?[fFuU]*)"
    if (-not [regex]::IsMatch($txt, $pat)) {
        throw "Khong tim thay macro $($Spec.Macro) trong $path"
    }
    $updated = [regex]::Replace($txt, $pat, { param($m) $m.Groups[1].Value + $cVal }, 1)
    [IO.File]::WriteAllText($path, $updated, (New-Object System.Text.UTF8Encoding($false)))
}

function Parse-CfgLine([string]$Line, [System.Collections.IDictionary]$Target) {
    if (-not ($Line.StartsWith('$CFG1,') -or $Line.StartsWith('$CFG2,'))) { return }
    $parts = $Line.Substring(6).Split(',')
    foreach ($kv in $parts) {
        $eq = $kv.IndexOf('=')
        if ($eq -gt 0) {
            $k = $kv.Substring(0, $eq).Trim().ToUpperInvariant()
            $vStr = $kv.Substring($eq + 1).Trim()
            $v = 0.0
            if ([double]::TryParse($vStr, [System.Globalization.NumberStyles]::Float, [System.Globalization.CultureInfo]::InvariantCulture, [ref]$v)) {
                $Target[$k] = $v
            }
        }
    }
}

function Raycast-DemoToF([double]$rx, [double]$ry, [double]$phiRad, [double]$mountCm, [bool]$hasDemoObs) {
    $c = [math]::Cos($phiRad)
    $s = [math]::Sin($phiRad)
    $ox = $rx + $mountCm * $c
    $oy = $ry + $mountCm * $s
    for ($d = 0.5; $d -le 80.0; $d += 0.5) {
        $px = $ox + $d * $c
        $py = $oy + $d * $s
        if ($px -le 0.0 -or $px -ge 180.0 -or $py -ge 120.0) { return [int][math]::Round($d * 10.0) }
        if ($px -ge 60.0 -and $px -le 120.0 -and $py -ge 0.0 -and $py -le 60.0) { return [int][math]::Round($d * 10.0) }
        foreach ($p in $StaticPillars) {
            if ($px -ge ($p.X - 5.0) -and $px -le ($p.X + 5.0) -and $py -ge ($p.Y - 5.0) -and $py -le ($p.Y + 5.0)) {
                return [int][math]::Round($d * 10.0)
            }
        }
        if ($hasDemoObs) {
            if ($px -ge 120.0 -and $px -le 130.0 -and $py -ge 60.0 -and $py -le 70.0) {
                return [int][math]::Round($d * 10.0)
            }
        }
    }
    return 8190
}

function Build-DashboardForm([System.Collections.IDictionary]$InitialValues) {
    $cBg      = [System.Drawing.Color]::FromArgb(15, 23, 42)
    $cPanel   = [System.Drawing.Color]::FromArgb(30, 41, 59)
    $cInput   = [System.Drawing.Color]::FromArgb(17, 24, 39)
    $cFg      = [System.Drawing.Color]::FromArgb(241, 245, 249)
    $cMuted   = [System.Drawing.Color]::FromArgb(148, 163, 184)
    $cCyan    = [System.Drawing.Color]::FromArgb(56, 189, 248)
    $script:cBlue    = [System.Drawing.Color]::FromArgb(37, 99, 235)
    $cGreen   = [System.Drawing.Color]::FromArgb(5, 150, 105)
    $cAmber   = [System.Drawing.Color]::FromArgb(217, 119, 6)
    $cTeal    = [System.Drawing.Color]::FromArgb(13, 148, 136)
    $cRed     = [System.Drawing.Color]::FromArgb(220, 38, 38)
    $cSlate   = [System.Drawing.Color]::FromArgb(71, 85, 105)

    $fontUi    = New-Object System.Drawing.Font("Segoe UI", 9.0)
    $fontBold  = New-Object System.Drawing.Font("Segoe UI", 9.0, [System.Drawing.FontStyle]::Bold)
    $fontSmall = New-Object System.Drawing.Font("Segoe UI", 8.0)
    $fontMono  = New-Object System.Drawing.Font("Consolas", 8.8)

    $form = New-Object System.Windows.Forms.Form
    $form.Text = "Cheesy Robot - Bảng Giám Sát Quỹ Đạo & Chỉnh Thông Số (STM32F411CEU6)"
    $form.Size = New-Object System.Drawing.Size(1240, 660)
    $form.FormBorderStyle = [System.Windows.Forms.FormBorderStyle]::FixedSingle
    $form.MaximizeBox = $false
    $form.StartPosition = [System.Windows.Forms.FormStartPosition]::CenterScreen
    $form.BackColor = $cBg
    $form.ForeColor = $cFg
    $form.Font = $fontUi
    if (Test-Path $TunerIcoPath) {
        $form.Icon = New-Object System.Drawing.Icon($TunerIcoPath)
    }

    # =========================================================================
    # LEFT COLUMN (x=12, w=560): 2D Map + Live Telemetry + Map Controls
    # =========================================================================
    $script:mapW = 560
    $script:mapH = 410
    $script:mapBmp = New-Object System.Drawing.Bitmap($script:mapW, $script:mapH)

    $script:picMap = New-Object System.Windows.Forms.PictureBox
    $script:picMap.Location = New-Object System.Drawing.Point(12, 10)
    $script:picMap.Size = New-Object System.Drawing.Size($script:mapW, $script:mapH)
    $script:picMap.BackColor = $cInput
    $script:picMap.Image = $script:mapBmp
    $script:picMap.BorderStyle = [System.Windows.Forms.BorderStyle]::FixedSingle
    $form.Controls.Add($script:picMap)

    # Map Controls Bar
    $mapBar = New-Object System.Windows.Forms.Panel
    $mapBar.Location = New-Object System.Drawing.Point(12, 424)
    $mapBar.Size = New-Object System.Drawing.Size(560, 34)
    $mapBar.BackColor = $cPanel
    $form.Controls.Add($mapBar)

    $script:btnDemo = New-Object System.Windows.Forms.Button
    $script:btnDemo.Text = "Mô phỏng Demo"
    $script:btnDemo.Location = New-Object System.Drawing.Point(6, 4)
    $script:btnDemo.Size = New-Object System.Drawing.Size(134, 26)
    $script:btnDemo.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $script:btnDemo.BackColor = $script:cBlue
    $script:btnDemo.ForeColor = $cFg
    $script:btnDemo.Font = $fontBold
    $mapBar.Controls.Add($script:btnDemo)

    $btnDemoObs = New-Object System.Windows.Forms.Button
    $btnDemoObs.Text = "Cọc Demo Ô 4: TẮT"
    $btnDemoObs.Location = New-Object System.Drawing.Point(146, 4)
    $btnDemoObs.Size = New-Object System.Drawing.Size(144, 26)
    $btnDemoObs.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnDemoObs.BackColor = $cSlate
    $btnDemoObs.ForeColor = $cFg
    $mapBar.Controls.Add($btnDemoObs)

    $btnClearMap = New-Object System.Windows.Forms.Button
    $btnClearMap.Text = "Xóa quỹ đạo"
    $btnClearMap.Location = New-Object System.Drawing.Point(296, 4)
    $btnClearMap.Size = New-Object System.Drawing.Size(122, 26)
    $btnClearMap.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnClearMap.BackColor = $cSlate
    $btnClearMap.ForeColor = $cFg
    $mapBar.Controls.Add($btnClearMap)

    $btnExportCsv = New-Object System.Windows.Forms.Button
    $btnExportCsv.Text = "Xuất CSV điểm"
    $btnExportCsv.Location = New-Object System.Drawing.Point(424, 4)
    $btnExportCsv.Size = New-Object System.Drawing.Size(130, 26)
    $btnExportCsv.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnExportCsv.BackColor = $cTeal
    $btnExportCsv.ForeColor = $cFg
    $mapBar.Controls.Add($btnExportCsv)

    # Telemetry Readout Panel
    $telPanel = New-Object System.Windows.Forms.Panel
    $telPanel.Location = New-Object System.Drawing.Point(12, 464)
    $telPanel.Size = New-Object System.Drawing.Size(560, 146)
    $telPanel.BackColor = $cPanel
    $form.Controls.Add($telPanel)

    $lblTelTitle = New-Object System.Windows.Forms.Label
    $lblTelTitle.Text = "THÔNG SỐ XE TRỰC TIẾP (EKF + 3× VL53L0X + ĐỘNG CƠ)"
    $lblTelTitle.Location = New-Object System.Drawing.Point(10, 6)
    $lblTelTitle.Size = New-Object System.Drawing.Size(540, 18)
    $lblTelTitle.ForeColor = $cCyan
    $lblTelTitle.Font = $fontBold
    $telPanel.Controls.Add($lblTelTitle)

    $script:lblPose = New-Object System.Windows.Forms.Label
    $script:lblPose.Location = New-Object System.Drawing.Point(10, 30)
    $script:lblPose.Size = New-Object System.Drawing.Size(540, 22)
    $script:lblPose.Font = $fontMono
    $telPanel.Controls.Add($script:lblPose)

    $script:lblState = New-Object System.Windows.Forms.Label
    $script:lblState.Location = New-Object System.Drawing.Point(10, 56)
    $script:lblState.Size = New-Object System.Drawing.Size(540, 22)
    $script:lblState.Font = $fontMono
    $telPanel.Controls.Add($script:lblState)

    $script:lblToF = New-Object System.Windows.Forms.Label
    $script:lblToF.Location = New-Object System.Drawing.Point(10, 82)
    $script:lblToF.Size = New-Object System.Drawing.Size(540, 22)
    $script:lblToF.Font = $fontMono
    $telPanel.Controls.Add($script:lblToF)

    $script:lblWheels = New-Object System.Windows.Forms.Label
    $script:lblWheels.Location = New-Object System.Drawing.Point(10, 108)
    $script:lblWheels.Size = New-Object System.Drawing.Size(540, 22)
    $script:lblWheels.Font = $fontMono
    $telPanel.Controls.Add($script:lblWheels)

    # =========================================================================
    # RIGHT COLUMN (x=584, w=634): BLE Header + 3-Tab Tuner + Buttons + Log
    # =========================================================================
    $hdr = New-Object System.Windows.Forms.Panel
    $hdr.Location = New-Object System.Drawing.Point(584, 10)
    $hdr.Size = New-Object System.Drawing.Size(634, 66)
    $hdr.BackColor = $cPanel
    $form.Controls.Add($hdr)

    $lblCom = New-Object System.Windows.Forms.Label
    $lblCom.Text = "Cổng HC-05:"
    $lblCom.Location = New-Object System.Drawing.Point(10, 12)
    $lblCom.Size = New-Object System.Drawing.Size(90, 22)
    $lblCom.Font = $fontBold
    $hdr.Controls.Add($lblCom)

    $script:cmbCom = New-Object System.Windows.Forms.ComboBox
    $script:cmbCom.Location = New-Object System.Drawing.Point(102, 9)
    $script:cmbCom.Size = New-Object System.Drawing.Size(220, 24)
    $script:cmbCom.DropDownStyle = [System.Windows.Forms.ComboBoxStyle]::DropDownList
    $script:cmbCom.BackColor = $cInput
    $script:cmbCom.ForeColor = $cFg
    $hdr.Controls.Add($script:cmbCom)

    $btnScan = New-Object System.Windows.Forms.Button
    $btnScan.Text = "Quét COM"
    $btnScan.Location = New-Object System.Drawing.Point(328, 8)
    $btnScan.Size = New-Object System.Drawing.Size(82, 26)
    $btnScan.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnScan.BackColor = $cSlate
    $btnScan.ForeColor = $cFg
    $hdr.Controls.Add($btnScan)

    $btnConn = New-Object System.Windows.Forms.Button
    $btnConn.Text = "Kết nối BLE"
    $btnConn.Location = New-Object System.Drawing.Point(416, 8)
    $btnConn.Size = New-Object System.Drawing.Size(102, 26)
    $btnConn.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnConn.BackColor = $script:cBlue
    $btnConn.ForeColor = $cFg
    $btnConn.Font = $fontBold
    $hdr.Controls.Add($btnConn)

    $btnRead = New-Object System.Windows.Forms.Button
    $btnRead.Text = "Đọc từ xe"
    $btnRead.Location = New-Object System.Drawing.Point(524, 8)
    $btnRead.Size = New-Object System.Drawing.Size(100, 26)
    $btnRead.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnRead.BackColor = $cTeal
    $btnRead.ForeColor = $cFg
    $btnRead.Enabled = $false
    $hdr.Controls.Add($btnRead)

    $lblStatus = New-Object System.Windows.Forms.Label
    $lblStatus.Text = "Trạng thái: Offline (nhấn 'Mô phỏng Demo' bên trái để xem thử quỹ đạo)"
    $lblStatus.Location = New-Object System.Drawing.Point(10, 40)
    $lblStatus.Size = New-Object System.Drawing.Size(614, 20)
    $lblStatus.ForeColor = $cCyan
    $hdr.Controls.Add($lblStatus)

    # Tabs (17 Parameters)
    $tabs = New-Object System.Windows.Forms.TabControl
    $tabs.Location = New-Object System.Drawing.Point(584, 84)
    $tabs.Size = New-Object System.Drawing.Size(634, 315)
    $form.Controls.Add($tabs)

    $tabNames = @("1. Vận tốc Ô 1..Ô 5 & Gia tốc", "2. Động cơ PI & Cơ khí", "3. Cảm biến ToF & Bám đường")
    $tabPages = @()
    foreach ($tName in $tabNames) {
        $tp = New-Object System.Windows.Forms.TabPage
        $tp.Text = $tName
        $tp.BackColor = $cPanel
        $tp.ForeColor = $cFg
        $tabs.TabPages.Add($tp)
        $tabPages += $tp
    }

    $script:numControls = @{}
    $tabRowY = @(14, 14, 14)

    foreach ($spec in $ParamSpecs) {
        $tIdx = $spec.Tab
        $tp = $tabPages[$tIdx]
        $y = $tabRowY[$tIdx]

        $lblKey = New-Object System.Windows.Forms.Label
        $lblKey.Text = "[$($spec.Key)]"
        $lblKey.Location = New-Object System.Drawing.Point(10, ($y + 3))
        $lblKey.Size = New-Object System.Drawing.Size(64, 22)
        $lblKey.Font = $fontMono
        $lblKey.ForeColor = $cCyan
        $tp.Controls.Add($lblKey)

        $lblName = New-Object System.Windows.Forms.Label
        $lblName.Text = $spec.Label
        $lblName.Location = New-Object System.Drawing.Point(76, ($y + 2))
        $lblName.Size = New-Object System.Drawing.Size(276, 22)
        $tp.Controls.Add($lblName)

        $num = New-Object System.Windows.Forms.NumericUpDown
        $num.Location = New-Object System.Drawing.Point(358, $y)
        $num.Size = New-Object System.Drawing.Size(96, 24)
        $num.Minimum = [decimal]$spec.Min
        $num.Maximum = [decimal]$spec.Max
        $num.Increment = [decimal]$spec.Inc
        $num.DecimalPlaces = [int]$spec.Dec
        $num.BackColor = $cInput
        $num.ForeColor = $cFg
        $num.Font = $fontBold
        if ($InitialValues.Contains($spec.Key)) {
            $initV = [decimal]$InitialValues[$spec.Key]
            if ($initV -lt $num.Minimum) { $initV = $num.Minimum }
            if ($initV -gt $num.Maximum) { $initV = $num.Maximum }
            $num.Value = $initV
        }
        $tp.Controls.Add($num)
        $script:numControls[$spec.Key] = $num

        $lblRange = New-Object System.Windows.Forms.Label
        $lblRange.Text = "[$($spec.Min)..$($spec.Max)] $($spec.Unit)"
        $lblRange.Location = New-Object System.Drawing.Point(462, ($y + 3))
        $lblRange.Size = New-Object System.Drawing.Size(156, 22)
        $lblRange.ForeColor = $cMuted
        $lblRange.Font = $fontSmall
        $tp.Controls.Add($lblRange)

        $tabRowY[$tIdx] = $y + 30
    }

    # Quick uniform speed row
    $lblAll = New-Object System.Windows.Forms.Label
    $lblAll.Text = "Đặt nhanh chung cả 5 ô (V1..V5):"
    $lblAll.Location = New-Object System.Drawing.Point(76, 263)
    $lblAll.Size = New-Object System.Drawing.Size(260, 22)
    $lblAll.ForeColor = $cCyan
    $lblAll.Font = $fontBold
    $tabPages[0].Controls.Add($lblAll)

    $numAll = New-Object System.Windows.Forms.NumericUpDown
    $numAll.Location = New-Object System.Drawing.Point(358, 260)
    $numAll.Size = New-Object System.Drawing.Size(96, 24)
    $numAll.Minimum = [decimal]10.0
    $numAll.Maximum = [decimal]30.0
    $numAll.Increment = [decimal]0.5
    $numAll.DecimalPlaces = 1
    $numAll.Value = [decimal]20.0
    $numAll.BackColor = $cInput
    $numAll.ForeColor = $cFg
    $tabPages[0].Controls.Add($numAll)

    $btnApplyAll = New-Object System.Windows.Forms.Button
    $btnApplyAll.Text = "Áp cho 5 ô"
    $btnApplyAll.Location = New-Object System.Drawing.Point(462, 259)
    $btnApplyAll.Size = New-Object System.Drawing.Size(110, 26)
    $btnApplyAll.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnApplyAll.BackColor = $script:cBlue
    $btnApplyAll.ForeColor = $cFg
    $tabPages[0].Controls.Add($btnApplyAll)

    # Action Panel
    $actPanel = New-Object System.Windows.Forms.Panel
    $actPanel.Location = New-Object System.Drawing.Point(584, 405)
    $actPanel.Size = New-Object System.Drawing.Size(634, 74)
    $actPanel.BackColor = $cPanel
    $form.Controls.Add($actPanel)

    $btnSendLive = New-Object System.Windows.Forms.Button
    $btnSendLive.Text = "Gửi LIVE Bluetooth"
    $btnSendLive.Location = New-Object System.Drawing.Point(8, 8)
    $btnSendLive.Size = New-Object System.Drawing.Size(150, 28)
    $btnSendLive.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnSendLive.BackColor = $script:cBlue
    $btnSendLive.ForeColor = $cFg
    $btnSendLive.Font = $fontBold
    $actPanel.Controls.Add($btnSendLive)

    $btnSaveFlash = New-Object System.Windows.Forms.Button
    $btnSaveFlash.Text = "Lưu Flash (CFG:SAVE)"
    $btnSaveFlash.Location = New-Object System.Drawing.Point(164, 8)
    $btnSaveFlash.Size = New-Object System.Drawing.Size(164, 28)
    $btnSaveFlash.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnSaveFlash.BackColor = $cGreen
    $btnSaveFlash.ForeColor = $cFg
    $btnSaveFlash.Font = $fontBold
    $actPanel.Controls.Add($btnSaveFlash)

    $btnSaveCode = New-Object System.Windows.Forms.Button
    $btnSaveCode.Text = "Lưu Code (.h)"
    $btnSaveCode.Location = New-Object System.Drawing.Point(334, 8)
    $btnSaveCode.Size = New-Object System.Drawing.Size(142, 28)
    $btnSaveCode.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnSaveCode.BackColor = $cAmber
    $btnSaveCode.ForeColor = $cFg
    $btnSaveCode.Font = $fontBold
    $actPanel.Controls.Add($btnSaveCode)

    $btnResetDef = New-Object System.Windows.Forms.Button
    $btnResetDef.Text = "Khôi phục mặc định"
    $btnResetDef.Location = New-Object System.Drawing.Point(482, 8)
    $btnResetDef.Size = New-Object System.Drawing.Size(144, 28)
    $btnResetDef.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnResetDef.BackColor = $cSlate
    $btnResetDef.ForeColor = $cFg
    $actPanel.Controls.Add($btnResetDef)

    $btnRunAuto = New-Object System.Windows.Forms.Button
    $btnRunAuto.Text = "Chạy tự hành (RUN:2)"
    $btnRunAuto.Location = New-Object System.Drawing.Point(8, 41)
    $btnRunAuto.Size = New-Object System.Drawing.Size(150, 26)
    $btnRunAuto.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnRunAuto.BackColor = $cTeal
    $btnRunAuto.ForeColor = $cFg
    $actPanel.Controls.Add($btnRunAuto)

    $btnStop = New-Object System.Windows.Forms.Button
    $btnStop.Text = "DỪNG KHẨN (STOP)"
    $btnStop.Location = New-Object System.Drawing.Point(164, 41)
    $btnStop.Size = New-Object System.Drawing.Size(146, 26)
    $btnStop.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnStop.BackColor = $cRed
    $btnStop.ForeColor = $cFg
    $btnStop.Font = $fontBold
    $actPanel.Controls.Add($btnStop)

    $btnZero = New-Object System.Windows.Forms.Button
    $btnZero.Text = "Reset tọa độ (ZERO)"
    $btnZero.Location = New-Object System.Drawing.Point(316, 41)
    $btnZero.Size = New-Object System.Drawing.Size(144, 26)
    $btnZero.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnZero.BackColor = $cSlate
    $btnZero.ForeColor = $cFg
    $actPanel.Controls.Add($btnZero)

    $btnFlashStlink = New-Object System.Windows.Forms.Button
    $btnFlashStlink.Text = "Build && Nạp ST-Link"
    $btnFlashStlink.Location = New-Object System.Drawing.Point(466, 41)
    $btnFlashStlink.Size = New-Object System.Drawing.Size(160, 26)
    $btnFlashStlink.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnFlashStlink.BackColor = $cSlate
    $btnFlashStlink.ForeColor = $cCyan
    $actPanel.Controls.Add($btnFlashStlink)

    # Log Box
    $script:txtLog = New-Object System.Windows.Forms.TextBox
    $script:txtLog.Location = New-Object System.Drawing.Point(584, 486)
    $script:txtLog.Size = New-Object System.Drawing.Size(634, 124)
    $script:txtLog.Multiline = $true
    $script:txtLog.ReadOnly = $true
    $script:txtLog.ScrollBars = [System.Windows.Forms.ScrollBars]::Vertical
    $script:txtLog.BackColor = $cInput
    $script:txtLog.ForeColor = $cFg
    $script:txtLog.Font = $fontMono
    $form.Controls.Add($script:txtLog)

    # =========================================================================
    # LIVE TELEMETRY, MAP STATE & DEMO SIMULATOR STATE
    # =========================================================================
    $script:serialPort   = $null
    $script:rxBuffer     = ""
    $script:trail        = New-Object System.Collections.Generic.List[object]
    $script:breadcrumbs  = New-Object System.Collections.Generic.List[object]
    $script:dynObs       = New-Object System.Collections.Generic.List[object]
    $script:demoActive   = $false
    $script:demoHasObs   = $false
    $script:demoWpIdx    = 1
    $script:demoObsAdded = $false
    $script:liveState    = @{
        X = 30.0; Y = -6.0; Heading = 90.0; Vbat = 0.0; Mode = 2
        Dl = 8190; Df = 8190; Dr = 8190
        VlExp = 0.0; VrExp = 0.0; VlReal = 0.0; VrReal = 0.0
        FsmState = 0; Score = 0
    }

    $script:writeLog = {
        param([string]$msg)
        $ts = (Get-Date).ToString("HH:mm:ss")
        $script:txtLog.AppendText("[$ts] $msg`r`n")
    }

    $script:updateTelLabels = {
        $st = $script:liveState
        $fsmTxt = if ($FsmStateNames.ContainsKey([int]$st.FsmState)) { $FsmStateNames[[int]$st.FsmState] } else { "STATE_$($st.FsmState)" }
        $script:lblPose.Text   = ("Tọa độ EKF: X={0,6:F1} cm | Y={1,6:F1} cm | Yaw={2,6:F1}° | Điểm đi qua: {3}" -f $st.X, $st.Y, $st.Heading, $script:breadcrumbs.Count)
        $script:lblState.Text  = ("FSM: {0,-22} | Điểm thi: {1,3} | Pin: {2:F2} V" -f $fsmTxt, $st.Score, $st.Vbat)
        $dlTxt = if ($st.Dl -ge 8190) { "---" } else { "$($st.Dl)" }
        $dfTxt = if ($st.Df -ge 8190) { "---" } else { "$($st.Df)" }
        $drTxt = if ($st.Dr -ge 8190) { "---" } else { "$($st.Dr)" }
        $script:lblToF.Text    = ("3× VL53L0X (mm): Trái(+45°)={0,-4} | Trước(0°)={1,-4} | Phải(-45°)={2,-4}" -f $dlTxt, $dfTxt, $drTxt)
        $script:lblWheels.Text = ("Bánh xe (cm/s): Trái {0,4:F1}/{1,4:F1} | Phải {2,4:F1}/{3,4:F1} (Đặt/Thực)" -f $st.VlExp, $st.VlReal, $st.VrExp, $st.VrReal)
    }

    $script:drawArenaMap = {
        $g = [System.Drawing.Graphics]::FromImage($script:mapBmp)
        try {
            $g.SmoothingMode = [System.Drawing.Drawing2D.SmoothingMode]::AntiAlias
            $g.Clear([System.Drawing.Color]::FromArgb(13, 20, 34))

            $scale = [math]::Min(($script:mapW - 32.0) / 192.0, ($script:mapH - 32.0) / 140.0)
            $offX  = ($script:mapW - 192.0 * $scale) * 0.5
            $offY  = ($script:mapH - 140.0 * $scale) * 0.5

            $wx = { param([double]$x) return [float]($offX + ($x + 6.0) * $scale) }
            $wy = { param([double]$y) return [float]($offY + (126.0 - $y) * $scale) }

            # O 3..O 5 ToF Scan Zone tint (x >= SCANX, y >= 0)
            $scanX = [double]$script:numControls["SCANX"].Value
            $bScan = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(24, 16, 185, 129))
            $g.FillRectangle($bScan, (& $wx $scanX), (& $wy 120.0), [float]((180.0 - $scanX) * $scale), [float](120.0 * $scale))
            $bScan.Dispose()

            # 10cm grid
            $pGrid = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(30, 41, 59), 1.0)
            for ($gx = 0; $gx -le 180; $gx += 10) {
                $px = & $wx $gx
                $g.DrawLine($pGrid, $px, (& $wy 120.0), $px, (& $wy 0.0))
            }
            for ($gy = 0; $gy -le 120; $gy += 10) {
                $py = & $wy $gy
                $g.DrawLine($pGrid, (& $wx 0.0), $py, (& $wx 180.0), $py)
            }
            $pGrid.Dispose()

            # Segment dividers & labels (O 1..O 5)
            $pDash = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(71, 85, 105), 1.0)
            $pDash.DashStyle = [System.Drawing.Drawing2D.DashStyle]::Dash
            $g.DrawLine($pDash, (& $wx 60.0), (& $wy 120.0), (& $wx 60.0), (& $wy 60.0))
            $g.DrawLine($pDash, (& $wx 120.0), (& $wy 120.0), (& $wx 120.0), (& $wy 60.0))
            $g.DrawLine($pDash, (& $wx 0.0), (& $wy 60.0), (& $wx 60.0), (& $wy 60.0))
            $g.DrawLine($pDash, (& $wx 120.0), (& $wy 60.0), (& $wx 180.0), (& $wy 60.0))
            $pDash.Dispose()

            $fZone = New-Object System.Drawing.Font("Segoe UI", 8.0, [System.Drawing.FontStyle]::Bold)
            $bZone = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(100, 148, 163, 184))
            $g.DrawString("Ô 1", $fZone, $bZone, (& $wx 22.0), (& $wy 12.0))
            $g.DrawString("Ô 2", $fZone, $bZone, (& $wx 22.0), (& $wy 116.0))
            $g.DrawString("Ô 3 (Bật ToF)", $fZone, $bZone, (& $wx 70.0), (& $wy 116.0))
            $g.DrawString("Ô 4 (Quét ToF)", $fZone, $bZone, (& $wx 132.0), (& $wy 116.0))
            $g.DrawString("Ô 5 (Đích)", $fZone, $bZone, (& $wx 136.0), (& $wy 56.0))
            $bZone.Dispose()

            # Start & Finish gates
            $pStart = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(56, 189, 248), 2.0)
            $g.DrawRectangle($pStart, (& $wx 18.0), (& $wy 0.0), [float](24.0 * $scale), [float](11.0 * $scale))
            $pStart.Dispose()
            $pFin = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(16, 185, 129), 3.0)
            $g.DrawLine($pFin, (& $wx 120.0), (& $wy 0.0), (& $wx 180.0), (& $wy 0.0))
            $pFin.Dispose()

            # Outer walls (bottom open at O1 & O5)
            $pWall = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(203, 213, 225), 2.2)
            $g.DrawLine($pWall, (& $wx 0.0), (& $wy 0.0), (& $wx 0.0), (& $wy 120.0))
            $g.DrawLine($pWall, (& $wx 0.0), (& $wy 120.0), (& $wx 180.0), (& $wy 120.0))
            $g.DrawLine($pWall, (& $wx 180.0), (& $wy 120.0), (& $wx 180.0), (& $wy 0.0))
            $pWall.Dispose()

            # Forbidden U-block [60..120] x [0..60]
            $bU = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(45, 55, 72))
            $pU = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(148, 163, 184), 1.8)
            $ux = & $wx 60.0; $uy = & $wy 60.0; $uw = [float](60.0 * $scale); $uh = [float](60.0 * $scale)
            $g.FillRectangle($bU, $ux, $uy, $uw, $uh)
            $g.DrawRectangle($pU, $ux, $uy, $uw, $uh)
            $bU.Dispose(); $pU.Dispose()

            # 17 static pillars (10x10 cm)
            $bObs  = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(220, 71, 85, 105))
            $pObs  = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(148, 163, 184), 1.2)
            $bLbl  = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(226, 232, 240))
            $fObs  = New-Object System.Drawing.Font("Consolas", 6.5)
            $boxSz = [float](10.0 * $scale)
            foreach ($p in $StaticPillars) {
                $bx = & $wx ($p.X - 5.0)
                $by = & $wy ($p.Y + 5.0)
                $g.FillRectangle($bObs, $bx, $by, $boxSz, $boxSz)
                $g.DrawRectangle($pObs, $bx, $by, $boxSz, $boxSz)
                $g.DrawString($p.Name, $fObs, $bLbl, ($bx + 2.0), ($by + 4.0))
            }
            $bObs.Dispose(); $pObs.Dispose(); $bLbl.Dispose()

            # Detected Dynamic Obstacles
            $bDyn = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(190, 239, 68, 68))
            $pDyn = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(254, 202, 202), 1.8)
            $bDynTxt = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::White)
            foreach ($d in $script:dynObs) {
                $dx = & $wx ($d.X - 5.0)
                $dy = & $wy ($d.Y + 5.0)
                $g.FillRectangle($bDyn, $dx, $dy, $boxSz, $boxSz)
                $g.DrawRectangle($pDyn, $dx, $dy, $boxSz, $boxSz)
                $g.DrawString("DYN", $fObs, $bDynTxt, ($dx + 1.0), ($dy + 4.0))
            }
            $bDyn.Dispose(); $pDyn.Dispose(); $bDynTxt.Dispose(); $fObs.Dispose(); $fZone.Dispose()

            # Trajectory polyline
            if ($script:trail.Count -ge 2) {
                $pts = New-Object 'System.Drawing.PointF[]' $script:trail.Count
                for ($i = 0; $i -lt $script:trail.Count; $i++) {
                    $pt = $script:trail[$i]
                    $pts[$i] = New-Object System.Drawing.PointF((& $wx $pt.X), (& $wy $pt.Y))
                }
                $pTrail = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(56, 189, 248), 2.3)
                $g.DrawLines($pTrail, $pts)
                $pTrail.Dispose()
            }

            # Breadcrumb visited dots
            $bDot = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(250, 204, 21))
            foreach ($bc in $script:breadcrumbs) {
                $px = & $wx $bc.X
                $py = & $wy $bc.Y
                $g.FillEllipse($bDot, ($px - 2.2), ($py - 2.2), 4.4, 4.4)
            }
            $bDot.Dispose()

            # Live 3x VL53L0X laser rays + Robot footprint (11.46 x 10.0 cm)
            $rx = [double]$script:liveState.X
            $ry = [double]$script:liveState.Y
            $th = [double]$script:liveState.Heading * [math]::PI / 180.0
            $tofMaxCm = [double]$script:numControls["TOF"].Value * 0.1

            $raySpecs = @(
                @{ Ang=45.0;  Off=4.5; DistMm=[int]$script:liveState.Dl }
                @{ Ang=0.0;   Off=5.0; DistMm=[int]$script:liveState.Df }
                @{ Ang=-45.0; Off=4.5; DistMm=[int]$script:liveState.Dr }
            )
            foreach ($rs in $raySpecs) {
                if ($rs.DistMm -gt 20 -and $rs.DistMm -lt 8190) {
                    $dCm = [double]$rs.DistMm * 0.1
                    $phi = $th + ($rs.Ang * [math]::PI / 180.0)
                    $sx  = $rx + $rs.Off * [math]::Cos($phi)
                    $sy  = $ry + $rs.Off * [math]::Sin($phi)
                    $ex  = $sx + $dCm * [math]::Cos($phi)
                    $ey  = $sy + $dCm * [math]::Sin($phi)
                    $col = if ($dCm -lt 12.0) { [System.Drawing.Color]::FromArgb(239, 68, 68) }
                           elseif ($dCm -le $tofMaxCm) { [System.Drawing.Color]::FromArgb(245, 158, 11) }
                           else { [System.Drawing.Color]::FromArgb(34, 197, 94) }
                    $pRay = New-Object System.Drawing.Pen($col, 1.6)
                    $g.DrawLine($pRay, (& $wx $sx), (& $wy $sy), (& $wx $ex), (& $wy $ey))
                    $pRay.Dispose()
                }
            }

            # Robot rotated chassis box (hl=5.0cm, hw=5.73cm)
            $c = [math]::Cos($th); $s = [math]::Sin($th)
            $corners = @(@(5.0, 5.73), @(5.0, -5.73), @(-5.0, -5.73), @(-5.0, 5.73))
            $rPts = New-Object 'System.Drawing.PointF[]' 4
            for ($i = 0; $i -lt 4; $i++) {
                $lx = $corners[$i][0]; $ly = $corners[$i][1]
                $pxW = $rx + $lx * $c - $ly * $s
                $pyW = $ry + $lx * $s + $ly * $c
                $rPts[$i] = New-Object System.Drawing.PointF((& $wx $pxW), (& $wy $pyW))
            }
            $bRob = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(220, 14, 165, 233))
            $pRob = New-Object System.Drawing.Pen([System.Drawing.Color]::White, 1.8)
            $g.FillPolygon($bRob, $rPts)
            $g.DrawPolygon($pRob, $rPts)
            $bRob.Dispose(); $pRob.Dispose()

            # Heading arrow
            $ax = $rx + 8.5 * $c; $ay = $ry + 8.5 * $s
            $pHead = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(250, 204, 21), 2.4)
            $g.DrawLine($pHead, (& $wx $rx), (& $wy $ry), (& $wx $ax), (& $wy $ay))
            $pHead.Dispose()
        } finally {
            $g.Dispose()
        }
        $script:picMap.Invalidate()
    }

    $script:pushTrailPoint = {
        param([double]$x, [double]$y)
        if ($script:trail.Count -eq 0) {
            $script:trail.Add([PSCustomObject]@{ X=$x; Y=$y })
            $script:breadcrumbs.Add([PSCustomObject]@{ X=$x; Y=$y })
            return
        }
        $last = $script:trail[$script:trail.Count - 1]
        $dist = [math]::Sqrt(($x - $last.X)*($x - $last.X) + ($y - $last.Y)*($y - $last.Y))
        if ($dist -ge 1.5 -and $script:trail.Count -lt 3000) {
            $script:trail.Add([PSCustomObject]@{ X=$x; Y=$y })
        }
        $lastBc = $script:breadcrumbs[$script:breadcrumbs.Count - 1]
        $dBc = [math]::Sqrt(($x - $lastBc.X)*($x - $lastBc.X) + ($y - $lastBc.Y)*($y - $lastBc.Y))
        if ($dBc -ge 5.0 -and $script:breadcrumbs.Count -lt 1000) {
            $script:breadcrumbs.Add([PSCustomObject]@{ X=$x; Y=$y })
        }
    }

    $script:handleIncomingLine = {
        param([string]$line)
        $inv = [System.Globalization.CultureInfo]::InvariantCulture
        if ($line.StartsWith('$TEL,')) {
            $f = $line.Split(',')
            if ($f.Length -ge 13) {
                $wheelR = [double]$script:numControls["WD"].Value * 0.5
                $script:liveState.Mode    = [int]$f[1]
                $script:liveState.Vbat    = [double]::Parse($f[2], $inv)
                $script:liveState.X       = [double]::Parse($f[3], $inv)
                $script:liveState.Y       = [double]::Parse($f[4], $inv)
                $script:liveState.Heading = [double]::Parse($f[5], $inv)
                $script:liveState.Dl      = [int]$f[6]
                $script:liveState.Df      = [int]$f[7]
                $script:liveState.Dr      = [int]$f[8]
                $script:liveState.VrExp   = [double]::Parse($f[9],  $inv) * $wheelR
                $script:liveState.VlExp   = [double]::Parse($f[10], $inv) * $wheelR
                $script:liveState.VrReal  = [double]::Parse($f[11], $inv) * $wheelR
                $script:liveState.VlReal  = [double]::Parse($f[12], $inv) * $wheelR
                & $script:pushTrailPoint $script:liveState.X $script:liveState.Y
                & $script:updateTelLabels
                & $script:drawArenaMap
            }
        } elseif ($line.StartsWith('$EVT,FSM,')) {
            $f = $line.Split(',')
            if ($f.Length -ge 4) {
                $script:liveState.FsmState = [int]$f[2]
                $script:liveState.Score    = [int]$f[3]
                & $script:updateTelLabels
                & $script:writeLog "FSM -> State=$($f[2]), Score=$($f[3])"
            }
        } elseif ($line.StartsWith('$EVT,OBS,')) {
            $f = $line.Split(',')
            if ($f.Length -ge 4) {
                $ox = [double]::Parse($f[2], $inv)
                $oy = [double]::Parse($f[3], $inv)
                $script:dynObs.Add([PSCustomObject]@{ X=$ox; Y=$oy })
                & $script:drawArenaMap
                & $script:writeLog "Phát hiện vật cản động tại ($ox, $oy) cm!"
            }
        } elseif ($line.StartsWith('$EVT,')) {
            & $script:writeLog $line
        }
    }

    # --- Serial & Demo Helpers ---
    $script:uiTimer = New-Object System.Windows.Forms.Timer
    $script:uiTimer.Interval = 50

    $script:sendAndWaitAck = {
        param([string]$cmd, [string]$ackPrefix, [int]$timeoutMs = 1500)
        $script:uiTimer.Stop()
        try {
            $port = $script:serialPort
            if ($null -eq $port -or -not $port.IsOpen) { return @{ Ok=$false; Reply="DISCONNECTED" } }
            $script:rxBuffer += $port.ReadExisting()
            $script:rxBuffer = ""
            $port.WriteLine($cmd)
            $sw = [System.Diagnostics.Stopwatch]::StartNew()
            while ($sw.ElapsedMilliseconds -lt $timeoutMs) {
                $chunk = $port.ReadExisting()
                if ($chunk.Length -gt 0) {
                    $script:rxBuffer += $chunk
                    while ($script:rxBuffer.Contains("`n")) {
                        $idx = $script:rxBuffer.IndexOf("`n")
                        $ln  = $script:rxBuffer.Substring(0, $idx).Trim()
                        $script:rxBuffer = $script:rxBuffer.Substring($idx + 1)
                        if ($ln.Length -eq 0) { continue }
                        if ($ln.StartsWith($ackPrefix) -or $ln.StartsWith('$ERR,')) {
                            return @{ Ok=$ln.StartsWith($ackPrefix); Reply=$ln }
                        }
                        & $script:handleIncomingLine $ln
                    }
                }
                [System.Windows.Forms.Application]::DoEvents()
                Start-Sleep -Milliseconds 10
            }
            return @{ Ok=$false; Reply="TIMEOUT" }
        } finally {
            $script:uiTimer.Start()
        }
    }

    $script:fetchBleCfg = {
        param([System.Collections.IDictionary]$target)
        $script:uiTimer.Stop()
        try {
            $port = $script:serialPort
            if ($null -eq $port -or -not $port.IsOpen) { return $false }
            $script:rxBuffer = ""
            [void]$port.ReadExisting()
            $port.WriteLine("CFG:GET")
            $sw = [System.Diagnostics.Stopwatch]::StartNew()
            while ($sw.ElapsedMilliseconds -lt 2200) {
                $chunk = $port.ReadExisting()
                if ($chunk.Length -gt 0) {
                    $script:rxBuffer += $chunk
                    while ($script:rxBuffer.Contains("`n")) {
                        $idx = $script:rxBuffer.IndexOf("`n")
                        $ln  = $script:rxBuffer.Substring(0, $idx).Trim()
                        $script:rxBuffer = $script:rxBuffer.Substring($idx + 1)
                        if ($ln.Length -eq 0) { continue }
                        if ($ln.StartsWith('$CFG1,') -or $ln.StartsWith('$CFG2,')) {
                            Parse-CfgLine $ln $target
                        } elseif ($ln -eq '$CFG,END') {
                            return $true
                        } else {
                            & $script:handleIncomingLine $ln
                        }
                    }
                }
                [System.Windows.Forms.Application]::DoEvents()
                Start-Sleep -Milliseconds 10
            }
            return $false
        } finally {
            $script:uiTimer.Start()
        }
    }

    $refreshComList = {
        $script:cmbCom.Items.Clear()
        $ports = [System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object
        $pnpMap = @{}
        try {
            Get-CimInstance Win32_PnPEntity -Filter "Name LIKE '%(COM%'" -ErrorAction SilentlyContinue | ForEach-Object {
                if ($_.Name -match '\((COM\d+)\)') { $pnpMap[$matches[1]] = $_.Name }
            }
        } catch {}
        foreach ($p in $ports) {
            $desc = if ($pnpMap.ContainsKey($p)) { $pnpMap[$p] } else { $p }
            [void]$script:cmbCom.Items.Add("$p - $desc")
        }
        if ($script:cmbCom.Items.Count -gt 0) {
            $script:cmbCom.SelectedIndex = 0
        } else {
            [void]$script:cmbCom.Items.Add("(Chưa thấy cổng COM Bluetooth)")
            $script:cmbCom.SelectedIndex = 0
        }
    }

    $syncValuesToUi = {
        param([System.Collections.IDictionary]$vals)
        foreach ($spec in $ParamSpecs) {
            if ($vals.Contains($spec.Key)) {
                $num = $script:numControls[$spec.Key]
                $v = [decimal]$vals[$spec.Key]
                if ($v -lt $num.Minimum) { $v = $num.Minimum }
                if ($v -gt $num.Maximum) { $v = $num.Maximum }
                $num.Value = $v
            }
        }
    }

    $script:uiTimer.Add_Tick({
        # 1. Real Bluetooth live stream
        if ($null -ne $script:serialPort -and $script:serialPort.IsOpen) {
            try {
                $chunk = $script:serialPort.ReadExisting()
                if ($chunk.Length -gt 0) {
                    $script:rxBuffer += $chunk
                    while ($script:rxBuffer.Contains("`n")) {
                        $idx = $script:rxBuffer.IndexOf("`n")
                        $ln  = $script:rxBuffer.Substring(0, $idx).Trim()
                        $script:rxBuffer = $script:rxBuffer.Substring($idx + 1)
                        if ($ln.Length -gt 0) { & $script:handleIncomingLine $ln }
                    }
                }
            } catch {}
            return
        }

        # 2. Built-in 20Hz Demo Simulator when enabled
        if ($script:demoActive) {
            $route = if ($script:demoHasObs) {
                @(
                    @(30.0,-6.0), @(30.0,15.0), @(30.0,50.0), @(24.0,74.0), @(20.0,98.0),
                    @(55.0,102.0), @(85.0,82.0), @(105.0,102.0), @(142.0,100.0),
                    @(158.0,72.0), @(158.0,32.0), @(162.0,12.0), @(150.0,-7.0)
                )
            } else {
                @(
                    @(30.0,-6.0), @(30.0,15.0), @(30.0,50.0), @(24.0,74.0), @(20.0,98.0),
                    @(55.0,102.0), @(85.0,82.0), @(105.0,102.0), @(142.0,100.0),
                    @(140.0,68.0), @(135.0,34.0), @(162.0,22.0), @(150.0,-7.0)
                )
            }
            if ($script:demoWpIdx -ge $route.Count) {
                $script:demoActive = $false
                $script:liveState.FsmState = 4
                $script:liveState.Score    = if ($script:demoObsAdded) { 272 } else { 257 }
                $script:liveState.VlExp = 0; $script:liveState.VrExp = 0
                $script:liveState.VlReal = 0; $script:liveState.VrReal = 0
                $script:btnDemo.Text = "Mô phỏng Demo"
                $script:btnDemo.BackColor = $script:cBlue
                & $script:updateTelLabels
                & $script:drawArenaMap
                & $script:writeLog "Demo hoàn thành chặng về đích Ô 5!"
                return
            }

            $tx = $route[$script:demoWpIdx][0]
            $ty = $route[$script:demoWpIdx][1]
            $rx = [double]$script:liveState.X
            $ry = [double]$script:liveState.Y
            $dx = $tx - $rx; $dy = $ty - $ry
            $dist = [math]::Sqrt($dx * $dx + $dy * $dy)

            # Segment speed from right-side tuner!
            $segKey = if ($rx -lt 60.0) { if ($ry -lt 60.0) { "V1" } else { "V2" } }
                      elseif ($rx -lt 120.0) { "V3" }
                      else { if ($ry -ge 60.0) { "V4" } else { "V5" } }
            $vDemo = [double]$script:numControls[$segKey].Value
            $vMax  = [double]$script:numControls["VMAX"].Value
            if ($vDemo -gt $vMax) { $vDemo = $vMax }
            if ($script:demoObsAdded -and $rx -ge 60.0) {
                $vDet = [double]$script:numControls["VDET"].Value
                if ($vDemo -gt $vDet) { $vDemo = $vDet }
            }

            $stepCm = $vDemo * 0.05
            $angRad = [math]::Atan2($dy, $dx)
            if ($dist -le $stepCm) {
                $rx = $tx; $ry = $ty
                $script:demoWpIdx++
            } else {
                $rx += $stepCm * [math]::Cos($angRad)
                $ry += $stepCm * [math]::Sin($angRad)
            }

            $scanX = [double]$script:numControls["SCANX"].Value
            $script:liveState.X       = $rx
            $script:liveState.Y       = $ry
            $script:liveState.Heading = $angRad * 180.0 / [math]::PI
            $script:liveState.Vbat    = 7.98
            $script:liveState.FsmState = if ($rx -ge $scanX -and $ry -ge 55.0) { 2 } else { 1 }
            $script:liveState.Dl = Raycast-DemoToF $rx $ry ($angRad + 0.7854) 4.5 $script:demoHasObs
            $script:liveState.Df = Raycast-DemoToF $rx $ry $angRad 5.0 $script:demoHasObs
            $script:liveState.Dr = Raycast-DemoToF $rx $ry ($angRad - 0.7854) 4.5 $script:demoHasObs
            $script:liveState.VlExp  = $vDemo
            $script:liveState.VrExp  = $vDemo
            $script:liveState.VlReal = $vDemo
            $script:liveState.VrReal = $vDemo

            if ($script:demoHasObs -and -not $script:demoObsAdded -and $rx -ge 120.0 -and $ry -le 95.0) {
                $script:demoObsAdded = $true
                $script:dynObs.Add([PSCustomObject]@{ X=125.0; Y=65.0 })
                & $script:writeLog "Demo 3× ToF phát hiện cọc ngẫu nhiên (125, 65) cm -> A* rẽ né!"
            }

            & $script:pushTrailPoint $rx $ry
            & $script:updateTelLabels
            & $script:drawArenaMap
        }
    })

    # --- Button Handlers ---
    $script:btnDemo.Add_Click({
        if ($null -ne $script:serialPort -and $script:serialPort.IsOpen) {
            [System.Windows.Forms.MessageBox]::Show("Đang kết nối Bluetooth thật — hãy ngắt BLE trước khi chạy Demo.", "Thông báo") | Out-Null
            return
        }
        $script:demoActive = -not $script:demoActive
        if ($script:demoActive) {
            $script:trail.Clear(); $script:breadcrumbs.Clear(); $script:dynObs.Clear()
            $script:demoWpIdx = 1; $script:demoObsAdded = $false
            $script:liveState.X = 30.0; $script:liveState.Y = -6.0; $script:liveState.Heading = 90.0
            $script:liveState.FsmState = 1; $script:liveState.Score = 0
            $script:btnDemo.Text = "Dừng Demo"
            $script:btnDemo.BackColor = $cRed
            & $script:writeLog "Bắt đầu mô phỏng Demo theo vận tốc V1..V5 bạn đang chọn!"
        } else {
            $script:btnDemo.Text = "Mô phỏng Demo"
            $script:btnDemo.BackColor = $script:cBlue
        }
    })

    $btnDemoObs.Add_Click({
        $script:demoHasObs = -not $script:demoHasObs
        $btnDemoObs.Text = if ($script:demoHasObs) { "Cọc Demo Ô 4: BẬT" } else { "Cọc Demo Ô 4: TẮT" }
        $btnDemoObs.BackColor = if ($script:demoHasObs) { $cAmber } else { $cSlate }
    })

    $btnClearMap.Add_Click({
        $script:trail.Clear(); $script:breadcrumbs.Clear(); $script:dynObs.Clear()
        $script:liveState.X = 30.0; $script:liveState.Y = -6.0; $script:liveState.Heading = 90.0
        $script:liveState.Dl = 8190; $script:liveState.Df = 8190; $script:liveState.Dr = 8190
        & $script:updateTelLabels
        & $script:drawArenaMap
        & $script:writeLog "Đã xóa quỹ đạo & đưa robot hiển thị về gốc (30, -6)."
    })

    $btnExportCsv.Add_Click({
        if ($script:trail.Count -eq 0) {
            & $script:writeLog "Chưa có điểm quỹ đạo nào để xuất CSV."
            return
        }
        if (-not (Test-Path $LogsDir)) { [void](New-Item -ItemType Directory -Path $LogsDir) }
        $csvPath = Join-Path $LogsDir ("trajectory_" + (Get-Date).ToString("yyyyMMdd_HHmmss") + ".csv")
        $sb = New-Object System.Text.StringBuilder
        [void]$sb.AppendLine("index,x_cm,y_cm")
        $inv = [System.Globalization.CultureInfo]::InvariantCulture
        for ($i = 0; $i -lt $script:trail.Count; $i++) {
            $p = $script:trail[$i]
            [void]$sb.AppendLine("$i,$($p.X.ToString('0.00', $inv)),$($p.Y.ToString('0.00', $inv))")
        }
        [IO.File]::WriteAllText($csvPath, $sb.ToString(), (New-Object System.Text.UTF8Encoding($false)))
        & $script:writeLog "Đã xuất $($script:trail.Count) điểm ra: $csvPath"
    })

    $btnScan.Add_Click({
        & $refreshComList
        & $script:writeLog "Đã quét danh sách cổng COM."
    })

    $btnApplyAll.Add_Click({
        $v = $numAll.Value
        foreach ($k in @("V1","V2","V3","V4","V5")) { $script:numControls[$k].Value = $v }
        & $script:writeLog "Đã đặt V1..V5 = $v cm/s trên bảng."
    })

    $btnConn.Add_Click({
        if ($null -ne $script:serialPort -and $script:serialPort.IsOpen) {
            $script:serialPort.Close(); $script:serialPort.Dispose(); $script:serialPort = $null
            $btnConn.Text = "Kết nối BLE"; $btnConn.BackColor = $script:cBlue; $btnRead.Enabled = $false
            $lblStatus.Text = "Trạng thái: Đã ngắt kết nối (Offline)"
            & $script:writeLog "Đã ngắt kết nối Bluetooth."
            return
        }
        $sel = [string]$script:cmbCom.SelectedItem
        if (-not ($sel -match '^(COM\d+)')) {
            [System.Windows.Forms.MessageBox]::Show("Chưa chọn cổng COM hợp lệ! Hãy ghép đôi HC-05 trong Windows trước.", "Thông báo") | Out-Null
            return
        }
        $comName = $matches[1]
        try {
            $script:demoActive = $false; $script:btnDemo.Text = "Mô phỏng Demo"; $script:btnDemo.BackColor = $script:cBlue
            $p = New-Object System.IO.Ports.SerialPort($comName, 9600, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
            $p.NewLine = "`r`n"; $p.ReadTimeout = 100; $p.WriteTimeout = 1000
            $p.Open()
            Start-Sleep -Milliseconds 150
            $script:serialPort = $p
            $btnConn.Text = "Ngắt BLE"; $btnConn.BackColor = $cRed; $btnRead.Enabled = $true
            $lblStatus.Text = "Trạng thái: Đã kết nối $comName @ 9600 (đang vẽ quỹ đạo trực tiếp)"
            & $script:writeLog "Đã mở $comName, đang đọc thông số từ xe..."

            $liveVals = [ordered]@{}
            if (& $script:fetchBleCfg $liveVals) {
                & $syncValuesToUi $liveVals
                & $script:writeLog "Đã đồng bộ đủ 17 thông số từ STM32!"
            } else {
                & $script:writeLog "Cảnh báo: Chưa nhận được `$CFG từ xe (kiểm tra nguồn xe / TX-RX HC-05)."
            }
        } catch {
            if ($null -ne $script:serialPort) {
                if ($script:serialPort.IsOpen) { $script:serialPort.Close() }
                $script:serialPort.Dispose(); $script:serialPort = $null
            }
            & $script:writeLog "Lỗi mở ${comName}: $($_.Exception.Message)"
        }
    })

    $btnRead.Add_Click({
        $liveVals = [ordered]@{}
        if (& $script:fetchBleCfg $liveVals) {
            & $syncValuesToUi $liveVals
            & $script:writeLog "Đã đọc lại 17 thông số từ xe."
        } else {
            & $script:writeLog "Lỗi: Không nhận được `$CFG từ xe."
        }
    })

    $btnSendLive.Add_Click({
        if ($null -eq $script:serialPort -or -not $script:serialPort.IsOpen) {
            [System.Windows.Forms.MessageBox]::Show("Hãy kết nối Bluetooth HC-05 trước, hoặc dùng 'Lưu Code (.h)'.", "Chưa kết nối BLE") | Out-Null
            return
        }
        $okCnt = 0
        foreach ($spec in $ParamSpecs) {
            $vStr = Format-CfgValue ([double]$script:numControls[$spec.Key].Value) $spec.Type
            $res = & $script:sendAndWaitAck "CFG:$($spec.Key),$vStr" "`$ACK,CFG,$($spec.Key)" 1200
            if ($res.Ok) { $okCnt++ } else { & $script:writeLog "Lỗi $($spec.Key)=$vStr -> $($res.Reply)" }
        }
        & $script:writeLog "Đã gửi LIVE $okCnt/$($ParamSpecs.Count) thông số! Bấm 'Lưu Flash' nếu muốn giữ sau khi tắt nguồn."
    })

    $btnSaveFlash.Add_Click({
        if ($null -eq $script:serialPort -or -not $script:serialPort.IsOpen) {
            [System.Windows.Forms.MessageBox]::Show("Hãy kết nối Bluetooth HC-05 trước khi lưu Flash.", "Chưa kết nối BLE") | Out-Null
            return
        }
        $res = & $script:sendAndWaitAck "CFG:SAVE" '$ACK,CFG,SAVE' 2500
        if ($res.Ok) { & $script:writeLog "OK: Đã lưu vĩnh viễn vào Flash Sector 7 trên STM32!" }
        else { & $script:writeLog "Lỗi CFG:SAVE ($($res.Reply)) — Bấm DỪNG KHẨN rồi thử lại." }
    })

    $btnSaveCode.Add_Click({
        try {
            foreach ($spec in $ParamSpecs) { Write-CodeParam $spec ([double]$script:numControls[$spec.Key].Value) }
            & $script:writeLog "OK: Đã ghi 17 thông số vào Core/Inc/robot_config.h & pure_pursuit.h!"
        } catch {
            & $script:writeLog "Lỗi ghi file code: $($_.Exception.Message)"
        }
    })

    $btnResetDef.Add_Click({
        if ($null -ne $script:serialPort -and $script:serialPort.IsOpen) {
            $res = & $script:sendAndWaitAck "CFG:RESET" '$ACK,CFG,RESET' 1500
            if ($res.Ok) {
                $liveVals = [ordered]@{}
                [void](& $script:fetchBleCfg $liveVals)
                & $syncValuesToUi $liveVals
                & $script:writeLog "Đã khôi phục mặc định trên xe."
            } else { & $script:writeLog "Lỗi CFG:RESET: $($res.Reply)" }
        } else {
            & $syncValuesToUi (Read-CodeDefaults)
            & $script:writeLog "Đã tải lại giá trị từ file code."
        }
    })

    $btnRunAuto.Add_Click({
        if ($null -eq $script:serialPort -or -not $script:serialPort.IsOpen) { return }
        $script:trail.Clear(); $script:breadcrumbs.Clear(); $script:dynObs.Clear()
        $res = & $script:sendAndWaitAck "RUN:2" '$ACK,RUN,2'
        & $script:writeLog "RUN:2 -> $($res.Reply)"
    })

    $btnStop.Add_Click({
        if ($null -eq $script:serialPort -or -not $script:serialPort.IsOpen) { return }
        $res = & $script:sendAndWaitAck "STOP" '$ACK,STOP'
        & $script:writeLog "STOP -> $($res.Reply)"
    })

    $btnZero.Add_Click({
        if ($null -eq $script:serialPort -or -not $script:serialPort.IsOpen) { return }
        $script:trail.Clear(); $script:breadcrumbs.Clear(); $script:dynObs.Clear()
        $res = & $script:sendAndWaitAck "ZERO" '$ACK,ZERO'
        & $script:drawArenaMap
        & $script:writeLog "ZERO -> $($res.Reply)"
    })

    $btnFlashStlink.Add_Click({
        & $script:writeLog "Đang mở NAP_CODE.bat để build & nạp ST-Link..."
        Start-Process cmd.exe -ArgumentList "/c `"`"$NapBatPath`"`"" -WorkingDirectory $RootDir
    })

    $form.Add_FormClosed({
        $script:uiTimer.Stop(); $script:uiTimer.Dispose()
        if ($null -ne $script:serialPort) {
            if ($script:serialPort.IsOpen) { $script:serialPort.Close() }
            $script:serialPort.Dispose(); $script:serialPort = $null
        }
        if ($null -ne $script:mapBmp) { $script:mapBmp.Dispose() }
    })

    & $refreshComList
    & $script:updateTelLabels
    & $script:drawArenaMap
    & $script:writeLog "Sẵn sàng! Bấm 'Mô phỏng Demo' để xem thử quỹ đạo, hoặc kết nối HC-05 xem xe chạy thật."
    $script:uiTimer.Start()

    return @{ Form=$form; DrawMap=$script:drawArenaMap; PushPt=$script:pushTrailPoint; DynObs=$script:dynObs; LiveState=$script:liveState }
}

$defaults = Read-CodeDefaults

if ($Arg1 -eq "--selftest") {
    $ctx = Build-DashboardForm $defaults
    # Test map rendering with sample trajectory, 3 ToF rays & dynamic obstacle
    & $ctx.PushPt 30.0 -6.0
    & $ctx.PushPt 30.0 50.0
    & $ctx.PushPt 85.0 82.0
    & $ctx.PushPt 135.0 80.0
    $ctx.LiveState.X = 135.0; $ctx.LiveState.Y = 80.0; $ctx.LiveState.Heading = -90.0
    $ctx.LiveState.Dl = 180; $ctx.LiveState.Df = 140; $ctx.LiveState.Dr = 220
    $ctx.DynObs.Add([PSCustomObject]@{ X=125.0; Y=65.0 })
    & $ctx.DrawMap
    $cnt = $script:numControls.Count
    $ctx.Form.Dispose()
    Write-Host "SELFTEST_OK Params=$cnt MapRendered=True IconTuner=$(Test-Path $TunerIcoPath) IconNap=$(Test-Path $NapIcoPath)"
    exit 0
}

Ensure-Shortcuts
$ctx = Build-DashboardForm $defaults
[void]$ctx.Form.ShowDialog()
