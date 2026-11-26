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

$RobotCfgPath   = Join-Path $RootDir "Core\Inc\robot_config.h"
$PurePursuitPath = Join-Path $RootDir "Core\Inc\pure_pursuit.h"
$NapBatPath     = Join-Path $RootDir "NAP_CODE.bat"

$ParamSpecs = @(
    @{ Id=1;  Key="V1";     Label="Van toc O 1 - Khe hep xuat phat";      Unit="cm/s";   Min=10.0;  Max=30.0;   Type="f1"; File="robot_config"; Macro="V_TARGET_O1_CMS" }
    @{ Id=2;  Key="V2";     Label="Van toc O 2 - Bo cua trai";            Unit="cm/s";   Min=10.0;  Max=30.0;   Type="f1"; File="robot_config"; Macro="V_TARGET_O2_CMS" }
    @{ Id=3;  Key="V3";     Label="Van toc O 3 - Doan thang + bat ToF";   Unit="cm/s";   Min=10.0;  Max=30.0;   Type="f1"; File="robot_config"; Macro="V_TARGET_O3_CMS" }
    @{ Id=4;  Key="V4";     Label="Van toc O 4 - Bo cua phai + quet ToF"; Unit="cm/s";   Min=10.0;  Max=30.0;   Type="f1"; File="robot_config"; Macro="V_TARGET_O4_CMS" }
    @{ Id=5;  Key="V5";     Label="Van toc O 5 - Xuoi ve dich";           Unit="cm/s";   Min=10.0;  Max=30.0;   Type="f1"; File="robot_config"; Macro="V_TARGET_O5_CMS" }
    @{ Id=6;  Key="VDET";   Label="Tran toc do khi ne vat can dong";      Unit="cm/s";   Min=10.0;  Max=30.0;   Type="f1"; File="robot_config"; Macro="V_TARGET_DETOUR_MAX_CMS" }
    @{ Id=7;  Key="VMAX";   Label="Gioi han van toc toi da toan xe";      Unit="cm/s";   Min=15.0;  Max=35.0;   Type="f1"; File="robot_config"; Macro="ROBOT_MAX_SPEED_CMS" }
    @{ Id=8;  Key="ACC";    Label="Gia toc toi da chong truot banh";      Unit="cm/s^2"; Min=20.0;  Max=150.0;  Type="f1"; File="robot_config"; Macro="ROBOT_ACCEL_MAX_CMS2" }
    @{ Id=9;  Key="KP";     Label="Motor PI - Kp (trai & phai)";          Unit="-";      Min=10.0;  Max=600.0;  Type="f1"; File="robot_config"; Macro="MOTOR_DEFAULT_KP" }
    @{ Id=10; Key="KI";     Label="Motor PI - Ki (tang 1.5..4 neu hut)";  Unit="-";      Min=0.0;   Max=50.0;   Type="f2"; File="robot_config"; Macro="MOTOR_DEFAULT_KI" }
    @{ Id=11; Key="WD";     Label="Duong kinh banh xe";                   Unit="cm";     Min=2.5;   Max=8.0;    Type="f2"; File="robot_config"; Macro="ROBOT_WHEEL_DIAM_CM" }
    @{ Id=12; Key="WB";     Label="Khoang cach tam 2 banh";               Unit="cm";     Min=6.0;   Max=16.0;   Type="f2"; File="robot_config"; Macro="ROBOT_TRACK_WIDTH_CM" }
    @{ Id=13; Key="PPR";    Label="So xung Encoder / vong banh (4x)";     Unit="ticks";  Min=100.0; Max=10000.0;Type="f0"; File="robot_config"; Macro="ROBOT_ENCODER_PPR" }
    @{ Id=14; Key="TOF";    Label="Tam quet 3x VL53L0X";                  Unit="mm";     Min=150.0; Max=800.0;  Type="u";  File="robot_config"; Macro="TOF_SCAN_MAX_MM" }
    @{ Id=15; Key="SCANX";  Label="Toa do X bat dau quet ToF (O 3 = 60)"; Unit="cm";     Min=0.0;   Max=120.0;  Type="f1"; File="robot_config"; Macro="FSM_SCAN_TRIGGER_X_CM" }
    @{ Id=16; Key="LD";     Label="Pure Pursuit Lookahead offset";        Unit="cm";     Min=4.0;   Max=12.0;   Type="f1"; File="pure_pursuit"; Macro="PP_LOOKAHEAD_OFFSET_CM" }
    @{ Id=17; Key="YAWINV"; Label="Dao chieu Yaw MPU6050 (0/1)";          Unit="0|1";    Min=0.0;   Max=1.0;    Type="i";  File="robot_config"; Macro="IMU_DEFAULT_YAW_INVERT" }
)

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
            throw "Khong doc duoc macro $($p.Macro) trong file $($p.File)"
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

function Show-ParamTable([string]$Title, [System.Collections.IDictionary]$Values) {
    Write-Host ""
    Write-Host "================================================================================" -ForegroundColor Cyan
    Write-Host "  $Title" -ForegroundColor Cyan
    Write-Host "================================================================================" -ForegroundColor Cyan
    Write-Host ("  {0,-4} {1,-7} {2,-40} {3,-10} {4}" -f "STT", "Key", "Thong so", "Hien tai", "Gioi han") -ForegroundColor Yellow
    Write-Host "  ------------------------------------------------------------------------------"
    foreach ($p in $ParamSpecs) {
        $cur = if ($Values.Contains($p.Key)) { Format-CfgValue ([double]$Values[$p.Key]) $p.Type } else { "?" }
        $rng = "[$($p.Min) .. $($p.Max)] $($p.Unit)"
        Write-Host ("  [{0,2}] {1,-7} {2,-40} {3,-10} {4}" -f $p.Id, $p.Key, $p.Label, $cur, $rng)
    }
    Write-Host "================================================================================" -ForegroundColor Cyan
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
        } catch [System.TimeoutException] {
            # tiep tuc cho den het TimeoutMs
        }
    }
    return $false
}

function Send-BleAndWaitAck([System.IO.Ports.SerialPort]$Port, [string]$Cmd, [string]$AckPrefix, [int]$TimeoutMs = 1500) {
    while ($Port.BytesToRead -gt 0) { [void]$Port.ReadExisting() }
    $Port.WriteLine($Cmd)
    $script:lastReply = ""
    $ok = Read-BleResponses $Port $TimeoutMs {
        param($line)
        if ($line.StartsWith($AckPrefix)) {
            $script:lastReply = $line
            return $true
        }
        if ($line.StartsWith('$ERR,')) {
            $script:lastReply = $line
            return $true
        }
        return $false
    }
    return @{ Ok = ($ok -and $script:lastReply.StartsWith($AckPrefix)); Reply = $script:lastReply }
}

function Fetch-BleConfig([System.IO.Ports.SerialPort]$Port, [System.Collections.IDictionary]$Values) {
    while ($Port.BytesToRead -gt 0) { [void]$Port.ReadExisting() }
    $Port.WriteLine("CFG:GET")
    $gotEnd = Read-BleResponses $Port 2200 {
        param($line)
        if ($line.StartsWith('$CFG1,') -or $line.StartsWith('$CFG2,')) {
            Parse-CfgLine $line $Values
            return $false
        }
        if ($line -eq '$CFG,END') {
            return $true
        }
        return $false
    }
    return $gotEnd
}

function Prompt-ParamNumber([hashtable]$Spec) {
    $raw = (Read-Host "Nhap gia tri moi cho [$($Spec.Key)] $($Spec.Label) [$($Spec.Min)..$($Spec.Max)]").Trim().Replace(',', '.')
    $val = 0.0
    if (-not [double]::TryParse($raw, [System.Globalization.NumberStyles]::Float, [System.Globalization.CultureInfo]::InvariantCulture, [ref]$val)) {
        Write-Host "[LOI] Khong phai so hop le!" -ForegroundColor Red
        return $null
    }
    if ($val -lt $Spec.Min -or $val -gt $Spec.Max) {
        Write-Host "[LOI] Ngoai gioi han cho phep [$($Spec.Min) .. $($Spec.Max)]!" -ForegroundColor Red
        return $null
    }
    if ($Spec.Type -eq "i" -or $Spec.Type -eq "u") {
        $val = [math]::Round($val)
    }
    return $val
}

function Run-LiveBluetoothMode {
    $ports = [System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object
    $pnpMap = @{}
    try {
        Get-CimInstance Win32_PnPEntity -Filter "Name LIKE '%(COM%'" -ErrorAction SilentlyContinue | ForEach-Object {
            if ($_.Name -match '\((COM\d+)\)') { $pnpMap[$matches[1]] = $_.Name }
        }
    } catch {}

    Write-Host ""
    Write-Host "Cac cong COM tim thay tren may:" -ForegroundColor Yellow
    if ($ports.Count -eq 0) {
        Write-Host "  (Khong thay cong COM nao! Hay ghep doi Bluetooth HC-05 trong Windows truoc)" -ForegroundColor Red
        Read-Host "Nhan Enter de quay lai"
        return
    }
    foreach ($p in $ports) {
        $desc = if ($pnpMap.ContainsKey($p)) { $pnpMap[$p] } else { $p }
        Write-Host "  - $p : $desc"
    }
    $com = (Read-Host "Nhap ten cong COM cua HC-05 (vd COM5, Enter de quay lai)").Trim().ToUpperInvariant()
    if ([string]::IsNullOrWhiteSpace($com)) { return }
    if ($com -match '^\d+$') { $com = "COM$com" }

    $port = New-Object System.IO.Ports.SerialPort($com, 9600, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
    $port.NewLine = "`r`n"
    $port.ReadTimeout = 250
    $port.WriteTimeout = 1000

    try {
        $port.Open()
        Start-Sleep -Milliseconds 150
        $values = [ordered]@{}
        Write-Host "Dang doc thong so tu STM32 qua $com..." -ForegroundColor Cyan
        if (-not (Fetch-BleConfig $port $values)) {
            Write-Host "[CANH BAO] Chua nhan duoc phan hoi `$CFG tu xe (kiem tra nguon xe / dung cong COM TX-RX HC-05)." -ForegroundColor Yellow
        }

        while ($true) {
            Show-ParamTable "CHE DO 1: CHINH LIVE QUA BLUETOOTH ($com - KHONG CAN NAP CODE)" $values
            Write-Host "  [1..17] Chinh 1 thong so     [A] Dat toc do chung V1..V5"
            Write-Host "  [S] LUU VAO FLASH TREN XE    [R] Doc lai tu xe          [D] Khoi phuc mac dinh"
            Write-Host "  [G] Chay tu hanh (RUN:2)     [X] Dung khan cap (STOP)   [Z] Reset toa do (ZERO)"
            Write-Host "  [0] Ngat ket noi & quay lai"
            $choice = (Read-Host "Chon lenh").Trim().ToUpperInvariant()

            if ($choice -eq "0") { break }
            elseif ($choice -eq "R") {
                if (Fetch-BleConfig $port $values) { Write-Host "[OK] Da doc lai thong so tu xe." -ForegroundColor Green }
                else { Write-Host "[LOI] Khong nhan duoc `$CFG tu xe." -ForegroundColor Red }
            }
            elseif ($choice -eq "S") {
                Write-Host "Dang luu xuong Flash STM32 Sector 7 (xe phai dang dung yen)..." -ForegroundColor Cyan
                $res = Send-BleAndWaitAck $port "CFG:SAVE" '$ACK,CFG,SAVE' 2500
                if ($res.Ok) { Write-Host "[OK] Da luu vinh vien vao Flash tren xe! ($($res.Reply))" -ForegroundColor Green }
                else { Write-Host "[LOI] Luu that bai ($($res.Reply)). Hay bam STOP de xe dung yen roi thu lai." -ForegroundColor Red }
            }
            elseif ($choice -eq "D") {
                $res = Send-BleAndWaitAck $port "CFG:RESET" '$ACK,CFG,RESET' 1500
                if ($res.Ok) {
                    [void](Fetch-BleConfig $port $values)
                    Write-Host "[OK] Da khoi phuc mac dinh (chua luu Flash cho den khi bam [S])." -ForegroundColor Green
                } else {
                    Write-Host "[LOI] Khong reset duoc ($($res.Reply))." -ForegroundColor Red
                }
            }
            elseif ($choice -eq "A") {
                $raw = (Read-Host "Nhap van toc chung cho O 1..O 5 [10..30] cm/s").Trim().Replace(',', '.')
                $v = 0.0
                if ([double]::TryParse($raw, [System.Globalization.NumberStyles]::Float, [System.Globalization.CultureInfo]::InvariantCulture, [ref]$v) -and $v -ge 10.0 -and $v -le 30.0) {
                    $vStr = Format-CfgValue $v "f1"
                    $res = Send-BleAndWaitAck $port "CFG:VALL,$vStr" '$ACK,CFG,VALL'
                    if ($res.Ok) {
                        foreach ($k in @("V1","V2","V3","V4","V5")) { $values[$k] = $v }
                        Write-Host "[OK] Da dat V1..V5 = $vStr cm/s ($($res.Reply))" -ForegroundColor Green
                    } else { Write-Host "[LOI] Xe tu choi ($($res.Reply))" -ForegroundColor Red }
                } else { Write-Host "[LOI] Nhap so trong [10..30]!" -ForegroundColor Red }
            }
            elseif ($choice -eq "G") {
                $res = Send-BleAndWaitAck $port "RUN:2" '$ACK,RUN,2'
                Write-Host "RUN:2 -> $($res.Reply)" -ForegroundColor Cyan
            }
            elseif ($choice -eq "X") {
                $res = Send-BleAndWaitAck $port "STOP" '$ACK,STOP'
                Write-Host "STOP -> $($res.Reply)" -ForegroundColor Yellow
            }
            elseif ($choice -eq "Z") {
                $res = Send-BleAndWaitAck $port "ZERO" '$ACK,ZERO'
                Write-Host "ZERO -> $($res.Reply)" -ForegroundColor Cyan
            }
            elseif ($choice -match '^\d+$') {
                $id = [int]$choice
                $spec = $ParamSpecs | Where-Object { $_.Id -eq $id } | Select-Object -First 1
                if ($null -eq $spec) {
                    Write-Host "[LOI] So thu tu khong hop le!" -ForegroundColor Red
                    continue
                }
                $newVal = Prompt-ParamNumber $spec
                if ($null -eq $newVal) { continue }
                $vStr = Format-CfgValue $newVal $spec.Type
                $res = Send-BleAndWaitAck $port "CFG:$($spec.Key),$vStr" "`$ACK,CFG,$($spec.Key)"
                if ($res.Ok) {
                    $values[$spec.Key] = $newVal
                    Write-Host "[OK] Da cap nhat $($spec.Key) = $vStr ($($res.Reply)) — Bam [S] neu muon luu vao Flash." -ForegroundColor Green
                } else {
                    Write-Host "[LOI] Khong gui duoc lenh ($($res.Reply))" -ForegroundColor Red
                }
            }
        }
    } catch {
        Write-Host "[LOI] Khong mo duoc cong $com : $($_.Exception.Message)" -ForegroundColor Red
        Read-Host "Nhan Enter de tiep tuc"
    } finally {
        if ($port.IsOpen) { $port.Close() }
        $port.Dispose()
    }
}

function Run-CodeEditMode {
    while ($true) {
        $values = Read-CodeDefaults
        Show-ParamTable "CHE DO 2: CHINH TRONG FILE CODE (robot_config.h / pure_pursuit.h)" $values
        Write-Host "  [1..17] Chinh 1 thong so trong file code   [A] Dat toc do chung V1..V5"
        Write-Host "  [F] Chay NAP_CODE.bat (Build & Nap ST-Link ngay)"
        Write-Host "  [0] Quay lai"
        $choice = (Read-Host "Chon lenh").Trim().ToUpperInvariant()

        if ($choice -eq "0") { break }
        elseif ($choice -eq "F") {
            & cmd /c "`"$NapBatPath`""
        }
        elseif ($choice -eq "A") {
            $raw = (Read-Host "Nhap van toc chung cho O 1..O 5 [10..30] cm/s").Trim().Replace(',', '.')
            $v = 0.0
            if ([double]::TryParse($raw, [System.Globalization.NumberStyles]::Float, [System.Globalization.CultureInfo]::InvariantCulture, [ref]$v) -and $v -ge 10.0 -and $v -le 30.0) {
                for ($i = 0; $i -lt 5; $i++) {
                    Write-CodeParam $ParamSpecs[$i] $v
                }
                Write-Host "[OK] Da ghi V1..V5 = $(Format-CfgValue $v 'f1') cm/s vao robot_config.h." -ForegroundColor Green
            } else {
                Write-Host "[LOI] Nhap so trong [10..30]!" -ForegroundColor Red
            }
        }
        elseif ($choice -match '^\d+$') {
            $id = [int]$choice
            $spec = $ParamSpecs | Where-Object { $_.Id -eq $id } | Select-Object -First 1
            if ($null -eq $spec) {
                Write-Host "[LOI] So thu tu khong hop le!" -ForegroundColor Red
                continue
            }
            $newVal = Prompt-ParamNumber $spec
            if ($null -eq $newVal) { continue }
            Write-CodeParam $spec $newVal
            Write-Host "[OK] Da ghi $($spec.Macro) = $(Format-CDefineValue $newVal $spec.Type)." -ForegroundColor Green
        }
    }
}

if ($Arg1 -eq "--selftest") {
    $vals = Read-CodeDefaults
    Show-ParamTable "SELFTEST: PARSED DEFAULTS FROM CODE" $vals
    exit 0
}

while ($true) {
    Write-Host ""
    Write-Host "============================================================" -ForegroundColor Cyan
    Write-Host "  CHEESY - CONG CU CHINH THONG SO DE NHIN (STM32F411CEU6)" -ForegroundColor Cyan
    Write-Host "============================================================" -ForegroundColor Cyan
    Write-Host "  [1] Chinh LIVE qua Bluetooth HC-05 + Luu Flash (KHONG can nap code)"
    Write-Host "  [2] Chinh vao file code (.h) + Build & Nap qua ST-Link"
    Write-Host "  [0] Thoat"
    $m = (Read-Host "Chon che do").Trim()
    if ($m -eq "1") { Run-LiveBluetoothMode }
    elseif ($m -eq "2") { Run-CodeEditMode }
    elseif ($m -eq "0") { break }
}
