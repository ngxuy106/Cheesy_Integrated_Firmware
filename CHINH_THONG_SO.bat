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

function Ensure-Shortcuts {
    try {
        $ws = New-Object -ComObject WScript.Shell
        foreach ($it in @(
            @{ Lnk="NAP_CODE.lnk"; Bat="NAP_CODE.bat"; Ico=$NapIcoPath; Desc="Build va Nap firmware STM32F411CEU6 qua ST-Link" },
            @{ Lnk="CHINH_THONG_SO.lnk"; Bat="CHINH_THONG_SO.bat"; Ico=$TunerIcoPath; Desc="Mo bang chinh 17 thong so Cheesy Robot (BLE Live + Flash)" }
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

function Read-BleResponses([System.IO.Ports.SerialPort]$Port, [int]$TimeoutMs, [scriptblock]$OnLine) {
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    while ($sw.ElapsedMilliseconds -lt $TimeoutMs) {
        try {
            $line = $Port.ReadLine().Trim()
            if ($line.Length -gt 0) {
                $done = & $OnLine $line
                if ($done) { return $true }
            }
        } catch [System.TimeoutException] {}
    }
    return $false
}

function Send-BleAndWaitAck([System.IO.Ports.SerialPort]$Port, [string]$Cmd, [string]$AckPrefix, [int]$TimeoutMs = 1500) {
    while ($Port.BytesToRead -gt 0) { [void]$Port.ReadExisting() }
    $Port.WriteLine($Cmd)
    $script:lastReply = ""
    $ok = Read-BleResponses $Port $TimeoutMs {
        param($line)
        if ($line.StartsWith($AckPrefix)) { $script:lastReply = $line; return $true }
        if ($line.StartsWith('$ERR,'))    { $script:lastReply = $line; return $true }
        return $false
    }
    return @{ Ok = ($ok -and $script:lastReply.StartsWith($AckPrefix)); Reply = $script:lastReply }
}

function Fetch-BleConfig([System.IO.Ports.SerialPort]$Port, [System.Collections.IDictionary]$Values) {
    while ($Port.BytesToRead -gt 0) { [void]$Port.ReadExisting() }
    $Port.WriteLine("CFG:GET")
    return (Read-BleResponses $Port 2200 {
        param($line)
        if ($line.StartsWith('$CFG1,') -or $line.StartsWith('$CFG2,')) {
            Parse-CfgLine $line $Values
            return $false
        }
        if ($line -eq '$CFG,END') { return $true }
        return $false
    })
}

function Build-TunerForm([System.Collections.IDictionary]$InitialValues) {
    $cBg      = [System.Drawing.Color]::FromArgb(15, 23, 42)
    $cPanel   = [System.Drawing.Color]::FromArgb(30, 41, 59)
    $cInput   = [System.Drawing.Color]::FromArgb(17, 24, 39)
    $cFg      = [System.Drawing.Color]::FromArgb(241, 245, 249)
    $cMuted   = [System.Drawing.Color]::FromArgb(148, 163, 184)
    $cCyan    = [System.Drawing.Color]::FromArgb(56, 189, 248)
    $cBlue    = [System.Drawing.Color]::FromArgb(37, 99, 235)
    $cGreen   = [System.Drawing.Color]::FromArgb(5, 150, 105)
    $cAmber   = [System.Drawing.Color]::FromArgb(217, 119, 6)
    $cTeal    = [System.Drawing.Color]::FromArgb(13, 148, 136)
    $cRed     = [System.Drawing.Color]::FromArgb(220, 38, 38)
    $cSlate   = [System.Drawing.Color]::FromArgb(71, 85, 105)

    $fontUi   = New-Object System.Drawing.Font("Segoe UI", 9.5)
    $fontBold = New-Object System.Drawing.Font("Segoe UI", 9.5, [System.Drawing.FontStyle]::Bold)
    $fontMono = New-Object System.Drawing.Font("Consolas", 9.0)

    $form = New-Object System.Windows.Forms.Form
    $form.Text = "Cheesy Robot - Bảng Chỉnh Thông Số (STM32F411CEU6)"
    $form.Size = New-Object System.Drawing.Size(790, 620)
    $form.FormBorderStyle = [System.Windows.Forms.FormBorderStyle]::FixedSingle
    $form.MaximizeBox = $false
    $form.StartPosition = [System.Windows.Forms.FormStartPosition]::CenterScreen
    $form.BackColor = $cBg
    $form.ForeColor = $cFg
    $form.Font = $fontUi
    if (Test-Path $TunerIcoPath) {
        $form.Icon = New-Object System.Drawing.Icon($TunerIcoPath)
    }

    # --- Header Bar (Bluetooth COM) ---
    $hdr = New-Object System.Windows.Forms.Panel
    $hdr.Location = New-Object System.Drawing.Point(12, 10)
    $hdr.Size = New-Object System.Drawing.Size(750, 66)
    $hdr.BackColor = $cPanel
    $form.Controls.Add($hdr)

    $lblCom = New-Object System.Windows.Forms.Label
    $lblCom.Text = "Cổng Bluetooth HC-05:"
    $lblCom.Location = New-Object System.Drawing.Point(12, 12)
    $lblCom.Size = New-Object System.Drawing.Size(145, 22)
    $lblCom.Font = $fontBold
    $hdr.Controls.Add($lblCom)

    $cmbCom = New-Object System.Windows.Forms.ComboBox
    $cmbCom.Location = New-Object System.Drawing.Point(160, 9)
    $cmbCom.Size = New-Object System.Drawing.Size(250, 24)
    $cmbCom.DropDownStyle = [System.Windows.Forms.ComboBoxStyle]::DropDownList
    $cmbCom.BackColor = $cInput
    $cmbCom.ForeColor = $cFg
    $hdr.Controls.Add($cmbCom)

    $btnScan = New-Object System.Windows.Forms.Button
    $btnScan.Text = "Quét COM"
    $btnScan.Location = New-Object System.Drawing.Point(418, 8)
    $btnScan.Size = New-Object System.Drawing.Size(85, 26)
    $btnScan.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnScan.BackColor = $cSlate
    $btnScan.ForeColor = $cFg
    $hdr.Controls.Add($btnScan)

    $btnConn = New-Object System.Windows.Forms.Button
    $btnConn.Text = "Kết nối BLE"
    $btnConn.Location = New-Object System.Drawing.Point(510, 8)
    $btnConn.Size = New-Object System.Drawing.Size(110, 26)
    $btnConn.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnConn.BackColor = $cBlue
    $btnConn.ForeColor = $cFg
    $btnConn.Font = $fontBold
    $hdr.Controls.Add($btnConn)

    $btnRead = New-Object System.Windows.Forms.Button
    $btnRead.Text = "Đọc từ xe"
    $btnRead.Location = New-Object System.Drawing.Point(626, 8)
    $btnRead.Size = New-Object System.Drawing.Size(112, 26)
    $btnRead.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnRead.BackColor = $cTeal
    $btnRead.ForeColor = $cFg
    $btnRead.Enabled = $false
    $hdr.Controls.Add($btnRead)

    $lblStatus = New-Object System.Windows.Forms.Label
    $lblStatus.Text = "Trạng thái: Chế độ Offline (đang hiển thị mặc định từ code .h)"
    $lblStatus.Location = New-Object System.Drawing.Point(12, 40)
    $lblStatus.Size = New-Object System.Drawing.Size(726, 20)
    $lblStatus.ForeColor = $cCyan
    $hdr.Controls.Add($lblStatus)

    # --- TabControl with 3 Tabs ---
    $tabs = New-Object System.Windows.Forms.TabControl
    $tabs.Location = New-Object System.Drawing.Point(12, 84)
    $tabs.Size = New-Object System.Drawing.Size(750, 315)
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
        $lblKey.Location = New-Object System.Drawing.Point(14, ($y + 3))
        $lblKey.Size = New-Object System.Drawing.Size(68, 22)
        $lblKey.Font = $fontMono
        $lblKey.ForeColor = $cCyan
        $tp.Controls.Add($lblKey)

        $lblName = New-Object System.Windows.Forms.Label
        $lblName.Text = $spec.Label
        $lblName.Location = New-Object System.Drawing.Point(86, ($y + 2))
        $lblName.Size = New-Object System.Drawing.Size(330, 22)
        $tp.Controls.Add($lblName)

        $num = New-Object System.Windows.Forms.NumericUpDown
        $num.Location = New-Object System.Drawing.Point(424, $y)
        $num.Size = New-Object System.Drawing.Size(110, 24)
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
        $lblRange.Text = "[$($spec.Min) .. $($spec.Max)] $($spec.Unit)"
        $lblRange.Location = New-Object System.Drawing.Point(544, ($y + 3))
        $lblRange.Size = New-Object System.Drawing.Size(180, 22)
        $lblRange.ForeColor = $cMuted
        $tp.Controls.Add($lblRange)

        $tabRowY[$tIdx] = $y + 31
    }

    # Quick uniform speed row in Tab 0
    $lblAll = New-Object System.Windows.Forms.Label
    $lblAll.Text = "Đặt nhanh chung cả 5 ô (V1..V5):"
    $lblAll.Location = New-Object System.Drawing.Point(86, 265)
    $lblAll.Size = New-Object System.Drawing.Size(260, 22)
    $lblAll.ForeColor = $cCyan
    $lblAll.Font = $fontBold
    $tabPages[0].Controls.Add($lblAll)

    $numAll = New-Object System.Windows.Forms.NumericUpDown
    $numAll.Location = New-Object System.Drawing.Point(424, 262)
    $numAll.Size = New-Object System.Drawing.Size(110, 24)
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
    $btnApplyAll.Location = New-Object System.Drawing.Point(544, 261)
    $btnApplyAll.Size = New-Object System.Drawing.Size(110, 26)
    $btnApplyAll.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnApplyAll.BackColor = $cBlue
    $btnApplyAll.ForeColor = $cFg
    $tabPages[0].Controls.Add($btnApplyAll)

    # --- Action Buttons Panel ---
    $actPanel = New-Object System.Windows.Forms.Panel
    $actPanel.Location = New-Object System.Drawing.Point(12, 405)
    $actPanel.Size = New-Object System.Drawing.Size(750, 74)
    $actPanel.BackColor = $cPanel
    $form.Controls.Add($actPanel)

    $btnSendLive = New-Object System.Windows.Forms.Button
    $btnSendLive.Text = "Gửi LIVE qua Bluetooth"
    $btnSendLive.Location = New-Object System.Drawing.Point(10, 8)
    $btnSendLive.Size = New-Object System.Drawing.Size(178, 28)
    $btnSendLive.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnSendLive.BackColor = $cBlue
    $btnSendLive.ForeColor = $cFg
    $btnSendLive.Font = $fontBold
    $actPanel.Controls.Add($btnSendLive)

    $btnSaveFlash = New-Object System.Windows.Forms.Button
    $btnSaveFlash.Text = "Lưu vào Flash xe (CFG:SAVE)"
    $btnSaveFlash.Location = New-Object System.Drawing.Point(194, 8)
    $btnSaveFlash.Size = New-Object System.Drawing.Size(200, 28)
    $btnSaveFlash.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnSaveFlash.BackColor = $cGreen
    $btnSaveFlash.ForeColor = $cFg
    $btnSaveFlash.Font = $fontBold
    $actPanel.Controls.Add($btnSaveFlash)

    $btnSaveCode = New-Object System.Windows.Forms.Button
    $btnSaveCode.Text = "Lưu vào file Code (.h)"
    $btnSaveCode.Location = New-Object System.Drawing.Point(400, 8)
    $btnSaveCode.Size = New-Object System.Drawing.Size(178, 28)
    $btnSaveCode.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnSaveCode.BackColor = $cAmber
    $btnSaveCode.ForeColor = $cFg
    $btnSaveCode.Font = $fontBold
    $actPanel.Controls.Add($btnSaveCode)

    $btnResetDef = New-Object System.Windows.Forms.Button
    $btnResetDef.Text = "Khôi phục mặc định"
    $btnResetDef.Location = New-Object System.Drawing.Point(584, 8)
    $btnResetDef.Size = New-Object System.Drawing.Size(156, 28)
    $btnResetDef.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnResetDef.BackColor = $cSlate
    $btnResetDef.ForeColor = $cFg
    $actPanel.Controls.Add($btnResetDef)

    $btnRunAuto = New-Object System.Windows.Forms.Button
    $btnRunAuto.Text = "Chạy tự hành (RUN:2)"
    $btnRunAuto.Location = New-Object System.Drawing.Point(10, 41)
    $btnRunAuto.Size = New-Object System.Drawing.Size(178, 26)
    $btnRunAuto.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnRunAuto.BackColor = $cTeal
    $btnRunAuto.ForeColor = $cFg
    $actPanel.Controls.Add($btnRunAuto)

    $btnStop = New-Object System.Windows.Forms.Button
    $btnStop.Text = "DỪNG KHẨN (STOP)"
    $btnStop.Location = New-Object System.Drawing.Point(194, 41)
    $btnStop.Size = New-Object System.Drawing.Size(166, 26)
    $btnStop.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnStop.BackColor = $cRed
    $btnStop.ForeColor = $cFg
    $btnStop.Font = $fontBold
    $actPanel.Controls.Add($btnStop)

    $btnZero = New-Object System.Windows.Forms.Button
    $btnZero.Text = "Reset tọa độ (ZERO)"
    $btnZero.Location = New-Object System.Drawing.Point(366, 41)
    $btnZero.Size = New-Object System.Drawing.Size(164, 26)
    $btnZero.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnZero.BackColor = $cSlate
    $btnZero.ForeColor = $cFg
    $actPanel.Controls.Add($btnZero)

    $btnFlashStlink = New-Object System.Windows.Forms.Button
    $btnFlashStlink.Text = "Build & Nạp ST-Link (NAP_CODE)"
    $btnFlashStlink.Location = New-Object System.Drawing.Point(536, 41)
    $btnFlashStlink.Size = New-Object System.Drawing.Size(204, 26)
    $btnFlashStlink.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnFlashStlink.BackColor = $cSlate
    $btnFlashStlink.ForeColor = $cCyan
    $actPanel.Controls.Add($btnFlashStlink)

    # --- Log Box ---
    $txtLog = New-Object System.Windows.Forms.TextBox
    $txtLog.Location = New-Object System.Drawing.Point(12, 486)
    $txtLog.Size = New-Object System.Drawing.Size(750, 86)
    $txtLog.Multiline = $true
    $txtLog.ReadOnly = $true
    $txtLog.ScrollBars = [System.Windows.Forms.ScrollBars]::Vertical
    $txtLog.BackColor = $cInput
    $txtLog.ForeColor = $cFg
    $txtLog.Font = $fontMono
    $form.Controls.Add($txtLog)

    $script:serialPort = $null

    $writeLog = {
        param([string]$msg)
        $ts = (Get-Date).ToString("HH:mm:ss")
        $txtLog.AppendText("[$ts] $msg`r`n")
    }

    $refreshComList = {
        $cmbCom.Items.Clear()
        $ports = [System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object
        $pnpMap = @{}
        try {
            Get-CimInstance Win32_PnPEntity -Filter "Name LIKE '%(COM%'" -ErrorAction SilentlyContinue | ForEach-Object {
                if ($_.Name -match '\((COM\d+)\)') { $pnpMap[$matches[1]] = $_.Name }
            }
        } catch {}
        foreach ($p in $ports) {
            $desc = if ($pnpMap.ContainsKey($p)) { $pnpMap[$p] } else { $p }
            [void]$cmbCom.Items.Add("$p - $desc")
        }
        if ($cmbCom.Items.Count -gt 0) {
            $cmbCom.SelectedIndex = 0
        } else {
            [void]$cmbCom.Items.Add("(Chưa thấy cổng COM Bluetooth)")
            $cmbCom.SelectedIndex = 0
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

    $btnScan.Add_Click({
        & $refreshComList
        & $writeLog "Đã quét danh sách cổng COM."
    })

    $btnApplyAll.Add_Click({
        $v = $numAll.Value
        foreach ($k in @("V1","V2","V3","V4","V5")) {
            $script:numControls[$k].Value = $v
        }
        & $writeLog "Đã đặt V1..V5 = $v cm/s trên bảng (hãy bấm Gửi LIVE hoặc Lưu vào file Code)."
    })

    $btnConn.Add_Click({
        if ($null -ne $script:serialPort -and $script:serialPort.IsOpen) {
            $script:serialPort.Close()
            $script:serialPort.Dispose()
            $script:serialPort = $null
            $btnConn.Text = "Kết nối BLE"
            $btnConn.BackColor = $cBlue
            $btnRead.Enabled = $false
            $lblStatus.Text = "Trạng thái: Đã ngắt kết nối (chế độ Offline)"
            & $writeLog "Đã ngắt kết nối Bluetooth."
            return
        }

        $sel = [string]$cmbCom.SelectedItem
        if (-not ($sel -match '^(COM\d+)')) {
            [System.Windows.Forms.MessageBox]::Show("Chưa chọn cổng COM hợp lệ! Hãy ghép đôi HC-05 trong Windows trước.", "Thông báo") | Out-Null
            return
        }
        $comName = $matches[1]
        try {
            $p = New-Object System.IO.Ports.SerialPort($comName, 9600, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
            $p.NewLine = "`r`n"
            $p.ReadTimeout = 250
            $p.WriteTimeout = 1000
            $p.Open()
            Start-Sleep -Milliseconds 150
            $script:serialPort = $p
            $btnConn.Text = "Ngắt BLE"
            $btnConn.BackColor = $cRed
            $btnRead.Enabled = $true
            $lblStatus.Text = "Trạng thái: Đã kết nối $comName @ 9600 baud"
            & $writeLog "Đã mở $comName, đang đọc thông số từ xe..."

            $liveVals = [ordered]@{}
            if (Fetch-BleConfig $p $liveVals) {
                & $syncValuesToUi $liveVals
                & $writeLog "Đã đồng bộ đủ 17 thông số trực tiếp từ STM32!"
            } else {
                & $writeLog "Cảnh báo: Chưa nhận được `$CFG từ xe (kiểm tra nguồn xe / TX-RX HC-05)."
            }
        } catch {
            if ($null -ne $script:serialPort) {
                if ($script:serialPort.IsOpen) { $script:serialPort.Close() }
                $script:serialPort.Dispose()
                $script:serialPort = $null
            }
            & $writeLog "Lỗi mở ${comName}: $($_.Exception.Message)"
        }
    })

    $btnRead.Add_Click({
        if ($null -eq $script:serialPort -or -not $script:serialPort.IsOpen) { return }
        $liveVals = [ordered]@{}
        if (Fetch-BleConfig $script:serialPort $liveVals) {
            & $syncValuesToUi $liveVals
            & $writeLog "Đã đọc lại 17 thông số từ xe."
        } else {
            & $writeLog "Lỗi: Không nhận được `$CFG từ xe."
        }
    })

    $btnSendLive.Add_Click({
        if ($null -eq $script:serialPort -or -not $script:serialPort.IsOpen) {
            [System.Windows.Forms.MessageBox]::Show("Hãy kết nối Bluetooth HC-05 trước, hoặc dùng nút 'Lưu vào file Code (.h)'.", "Chưa kết nối BLE") | Out-Null
            return
        }
        $okCnt = 0
        foreach ($spec in $ParamSpecs) {
            $vStr = Format-CfgValue ([double]$script:numControls[$spec.Key].Value) $spec.Type
            $res = Send-BleAndWaitAck $script:serialPort "CFG:$($spec.Key),$vStr" "`$ACK,CFG,$($spec.Key)" 1200
            if ($res.Ok) { $okCnt++ }
            else { & $writeLog "Lỗi gửi $($spec.Key)=$vStr -> $($res.Reply)" }
        }
        & $writeLog "Đã gửi LIVE $okCnt/$($ParamSpecs.Count) thông số xuống xe! Bấm 'Lưu vào Flash xe' nếu muốn giữ sau khi tắt nguồn."
    })

    $btnSaveFlash.Add_Click({
        if ($null -eq $script:serialPort -or -not $script:serialPort.IsOpen) {
            [System.Windows.Forms.MessageBox]::Show("Hãy kết nối Bluetooth HC-05 trước khi lưu Flash từ xa.", "Chưa kết nối BLE") | Out-Null
            return
        }
        $res = Send-BleAndWaitAck $script:serialPort "CFG:SAVE" '$ACK,CFG,SAVE' 2500
        if ($res.Ok) { & $writeLog "OK: Đã lưu vĩnh viễn vào Flash Sector 7 trên STM32! ($($res.Reply))" }
        else { & $writeLog "Lỗi CFG:SAVE ($($res.Reply)) — Hãy bấm DỪNG KHẨN cho xe đứng yên rồi thử lại." }
    })

    $btnSaveCode.Add_Click({
        try {
            foreach ($spec in $ParamSpecs) {
                Write-CodeParam $spec ([double]$script:numControls[$spec.Key].Value)
            }
            & $writeLog "OK: Đã ghi toàn bộ 17 thông số vào Core/Inc/robot_config.h & pure_pursuit.h!"
        } catch {
            & $writeLog "Lỗi ghi file code: $($_.Exception.Message)"
        }
    })

    $btnResetDef.Add_Click({
        if ($null -ne $script:serialPort -and $script:serialPort.IsOpen) {
            $res = Send-BleAndWaitAck $script:serialPort "CFG:RESET" '$ACK,CFG,RESET' 1500
            if ($res.Ok) {
                $liveVals = [ordered]@{}
                [void](Fetch-BleConfig $script:serialPort $liveVals)
                & $syncValuesToUi $liveVals
                & $writeLog "Đã khôi phục mặc định trên xe (chưa lưu Flash cho tới khi bấm CFG:SAVE)."
            } else {
                & $writeLog "Lỗi CFG:RESET: $($res.Reply)"
            }
        } else {
            $defs = Read-CodeDefaults
            & $syncValuesToUi $defs
            & $writeLog "Đã tải lại giá trị hiện có trong file code."
        }
    })

    $btnRunAuto.Add_Click({
        if ($null -eq $script:serialPort -or -not $script:serialPort.IsOpen) { return }
        $res = Send-BleAndWaitAck $script:serialPort "RUN:2" '$ACK,RUN,2'
        & $writeLog "RUN:2 -> $($res.Reply)"
    })

    $btnStop.Add_Click({
        if ($null -eq $script:serialPort -or -not $script:serialPort.IsOpen) { return }
        $res = Send-BleAndWaitAck $script:serialPort "STOP" '$ACK,STOP'
        & $writeLog "STOP -> $($res.Reply)"
    })

    $btnZero.Add_Click({
        if ($null -eq $script:serialPort -or -not $script:serialPort.IsOpen) { return }
        $res = Send-BleAndWaitAck $script:serialPort "ZERO" '$ACK,ZERO'
        & $writeLog "ZERO -> $($res.Reply)"
    })

    $btnFlashStlink.Add_Click({
        & $writeLog "Đang mở NAP_CODE.bat để build & nạp qua ST-Link V2..."
        Start-Process cmd.exe -ArgumentList "/c `"`"$NapBatPath`"`"" -WorkingDirectory $RootDir
    })

    $form.Add_FormClosed({
        if ($null -ne $script:serialPort) {
            if ($script:serialPort.IsOpen) { $script:serialPort.Close() }
            $script:serialPort.Dispose()
            $script:serialPort = $null
        }
    })

    & $refreshComList
    & $writeLog "Sẵn sàng. Chọn cổng COM HC-05 để chỉnh LIVE không cần nạp code, hoặc chỉnh rồi lưu vào file .h."
    return $form
}

$defaults = Read-CodeDefaults

if ($Arg1 -eq "--selftest") {
    $f = Build-TunerForm $defaults
    $cnt = $script:numControls.Count
    $f.Dispose()
    Write-Host "SELFTEST_OK Params=$cnt IconTuner=$(Test-Path $TunerIcoPath) IconNap=$(Test-Path $NapIcoPath)"
    foreach ($p in $ParamSpecs) {
        $v = Format-CfgValue ([double]$defaults[$p.Key]) $p.Type
        Write-Host ("  [{0,2}] {1,-7} = {2,-8} ({3})" -f $p.Id, $p.Key, $v, $p.Label)
    }
    exit 0
}

Ensure-Shortcuts
$mainForm = Build-TunerForm $defaults
[void]$mainForm.ShowDialog()
