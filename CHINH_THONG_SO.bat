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

$script:RootDir         = $RootDir
$script:RobotCfgPath    = Join-Path $RootDir "Core\Inc\robot_config.h"
$script:PurePursuitPath = Join-Path $RootDir "Core\Inc\pure_pursuit.h"
$script:ArenaMapPath    = Join-Path $RootDir "Core\Src\arena_map.c"
$script:NapBatPath      = Join-Path $RootDir "NAP_CODE.bat"
$script:TunerIcoPath    = Join-Path $RootDir "icons\chinh_thong_so.ico"
$script:NapIcoPath      = Join-Path $RootDir "icons\nap_code.ico"
$script:LogsDir         = Join-Path $RootDir "logs"

# =============================================================================
# 1-TO-1 PORT OF Core/Src/arena_map.c + Core/Src/astar_planner.c IN C#
# =============================================================================
if (-not ("CheesyAStar" -as [type])) {
$astarCs = @'
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text.RegularExpressions;

public static class CheesyAStar
{
    public const int MAP_ROWS = 50;
    public const int MAP_COLS = 74;
    public const int MAP_MARGIN = 1;
    public const float GRID_SIZE_CM = 2.5f;
    public const float MAP_Y_OFFSET_CM = -2.5f;
    public const float WALL_NEAR_PENALTY = 0.35f;
    public const byte CELL_FREE = 0;
    public const byte CELL_OBST = 1;
    public const byte CELL_BLOCK = 2;
    public const byte CELL_WALL = 3;
    public const byte CELL_INFLATED = 4;

    private static readonly int[] DIR_R = { -1, 1, 0, 0, -1, -1, 1, 1 };
    private static readonly int[] DIR_C = { 0, 0, -1, 1, -1, 1, -1, 1 };
    private static readonly float[] DIR_W = {
        1.0f, 1.0f, 1.0f, 1.0f,
        1.41421356f, 1.41421356f, 1.41421356f, 1.41421356f
    };

    private static readonly float[,] PILLARS = new float[,] {
        { 5.0f, 15.0f }, { 55.0f, 25.0f }, { 30.0f, 35.0f }, { 55.0f, 45.0f },
        { 5.0f, 55.0f }, { 5.0f, 75.0f }, { 35.0f, 90.0f }, { 45.0f, 115.0f },
        { 65.0f, 75.0f }, { 75.0f, 65.0f }, { 85.0f, 100.0f }, { 95.0f, 65.0f },
        { 115.0f, 115.0f }, { 125.0f, 85.0f }, { 145.0f, 45.0f }, { 145.0f, 15.0f },
        { 175.0f, 25.0f }
    };

    private static byte[,] s_staticGrid = new byte[MAP_ROWS, MAP_COLS];
    private static byte[,] s_cspaceGrid = new byte[MAP_ROWS, MAP_COLS];
    private static bool s_inited = false;

    public static void LoadStaticGridFromCFile(string arenaMapCPath)
    {
        string text = File.ReadAllText(arenaMapCPath);
        Match m = Regex.Match(text, @"s_arena_grid\[MAP_NY\]\[MAP_NX\]\s*=\s*\{(?<body>.*?)\};", RegexOptions.Singleline);
        if (!m.Success) throw new InvalidOperationException("Khong tim thay s_arena_grid trong arena_map.c");
        MatchCollection rows = Regex.Matches(m.Groups["body"].Value, @"\{([0-4,\s]+)\}");
        if (rows.Count != MAP_ROWS)
            throw new InvalidOperationException(string.Format("So hang grid ({0}) != {1}", rows.Count, MAP_ROWS));

        for (int r = 0; r < MAP_ROWS; r++)
        {
            string[] cols = rows[r].Groups[1].Value.Split(new[] { ',' }, StringSplitOptions.RemoveEmptyEntries);
            if (cols.Length != MAP_COLS)
                throw new InvalidOperationException(string.Format("Hang {0} co {1} cot != {2}", r, cols.Length, MAP_COLS));
            for (int c = 0; c < MAP_COLS; c++)
            {
                s_staticGrid[r, c] = byte.Parse(cols[c].Trim(), CultureInfo.InvariantCulture);
            }
        }
        ResetCSpace();
    }

    public static void ResetCSpace()
    {
        Array.Copy(s_staticGrid, s_cspaceGrid, s_staticGrid.Length);
        s_inited = true;
    }

    public static int GetGridCol(float x)
    {
        int col = (int)Math.Floor((x + 2.5f + 1e-3f) / GRID_SIZE_CM);
        if (col < 0) col = 0;
        if (col >= MAP_COLS) col = MAP_COLS - 1;
        return col;
    }

    public static int GetGridRow(float y)
    {
        int row = (int)Math.Floor((y - MAP_Y_OFFSET_CM + 1e-3f) / GRID_SIZE_CM);
        if (row < 0) row = 0;
        if (row >= MAP_ROWS) row = MAP_ROWS - 1;
        return row;
    }

    public static float GetWorldX(int col) { return ((float)col - MAP_MARGIN + 0.5f) * GRID_SIZE_CM; }
    public static float GetWorldY(int row) { return MAP_Y_OFFSET_CM + ((float)row + 0.5f) * GRID_SIZE_CM; }

    private static bool WorldToGrid(float x, float y, out int outR, out int outC)
    {
        outR = 0; outC = 0;
        if (float.IsNaN(x) || float.IsNaN(y) || float.IsInfinity(x) || float.IsInfinity(y)) return false;

        if (x >= -2.5f && x <= 182.5f && y >= -2.5f && y <= 122.5f)
        {
            int c = (int)Math.Floor((x + 2.5f + 1e-3f) / GRID_SIZE_CM);
            int r = (int)Math.Floor((y + 2.5f + 1e-3f) / GRID_SIZE_CM);
            if (c >= MAP_COLS) c = MAP_COLS - 1;
            if (r >= MAP_ROWS) r = MAP_ROWS - 1;
            if (r >= 0 && r < MAP_ROWS && c >= 0 && c < MAP_COLS)
            {
                outR = r; outC = c;
                return true;
            }
            return false;
        }
        if (y >= -10.5f && y < -2.5f && x >= 0.0f && x <= 60.0f)
        {
            int c = (int)Math.Floor((x + 2.5f + 1e-3f) / GRID_SIZE_CM);
            if (c >= 0 && c < MAP_COLS) { outR = 0; outC = c; return true; }
            return false;
        }
        if (y >= -10.5f && y < -2.5f && x >= 120.0f && x <= 180.0f)
        {
            int c = (int)Math.Floor((x + 2.5f + 1e-3f) / GRID_SIZE_CM);
            if (c >= 0 && c < MAP_COLS) { outR = 0; outC = c; return true; }
            return false;
        }
        return false;
    }

    public static void AddDynamicObstacleBox(float cx, float cy, float halfSizeCm, float inflateCm)
    {
        if (!s_inited) ResetCSpace();
        if (halfSizeCm < 0.5f) halfSizeCm = 0.5f;
        if (inflateCm < 0.0f) inflateCm = 0.0f;

        float x0 = cx - halfSizeCm, x1 = cx + halfSizeCm;
        float y0 = cy - halfSizeCm, y1 = cy + halfSizeCm;
        float reach = halfSizeCm + inflateCm + GRID_SIZE_CM;
        int cMin = GetGridCol(cx - reach), cMax = GetGridCol(cx + reach);
        int rMin = GetGridRow(cy - reach), rMax = GetGridRow(cy + reach);

        for (int r = rMin; r <= rMax; r++)
        {
            float wy = GetWorldY(r);
            for (int c = cMin; c <= cMax; c++)
            {
                float wx = GetWorldX(c);
                float dx = (wx < x0) ? (x0 - wx) : (wx > x1 ? (wx - x1) : 0.0f);
                float dy = (wy < y0) ? (y0 - wy) : (wy > y1 ? (wy - y1) : 0.0f);
                float d = (float)Math.Sqrt(dx * dx + dy * dy);
                if (d <= 0.001f) s_cspaceGrid[r, c] = CELL_OBST;
                else if (d <= inflateCm && s_cspaceGrid[r, c] == CELL_FREE) s_cspaceGrid[r, c] = CELL_INFLATED;
            }
        }
    }

    private static float Heuristic(int r1, int c1, int r2, int c2)
    {
        float dr = r1 - r2, dc = c1 - c2;
        return (float)Math.Sqrt(dr * dr + dc * dc);
    }

    private static float CellProximityPenalty(int r, int c)
    {
        for (int i = 0; i < 4; i++)
        {
            int nr = r + DIR_R[i], nc = c + DIR_C[i];
            if (nr < 0 || nr >= MAP_ROWS || nc < 0 || nc >= MAP_COLS || s_cspaceGrid[nr, nc] != CELL_FREE)
                return WALL_NEAR_PENALTY;
        }
        return 0.0f;
    }

    public static bool IsLineClear(float x1, float y1, float x2, float y2)
    {
        if (!s_inited) return false;
        float dx = x2 - x1, dy = y2 - y1;
        float dist = (float)Math.Sqrt(dx * dx + dy * dy);
        int steps = Math.Max(2, (int)Math.Ceiling(dist / 0.5f));
        int prevR = -1, prevC = -1;
        for (int s = 0; s <= steps; s++)
        {
            float t = (float)s / steps;
            int r, c;
            if (!WorldToGrid(x1 + t * dx, y1 + t * dy, out r, out c)) return false;
            if (s_cspaceGrid[r, c] != CELL_FREE) return false;
            if (prevR >= 0 && r != prevR && c != prevC)
            {
                if (s_cspaceGrid[prevR, c] != CELL_FREE || s_cspaceGrid[r, prevC] != CELL_FREE) return false;
            }
            prevR = r; prevC = c;
        }
        return true;
    }

    private static bool SnapToNearestFree(ref int ioR, ref int ioC, int maxRad)
    {
        int r0 = ioR, c0 = ioC;
        if (s_cspaceGrid[r0, c0] == CELL_FREE) return true;
        if (s_cspaceGrid[r0, c0] != CELL_INFLATED) return false;
        int bestR = -1, bestC = -1, bestD2 = 999999;
        for (int rad = 1; rad <= maxRad; rad++)
        {
            for (int dr = -rad; dr <= rad; dr++)
            {
                for (int dc = -rad; dc <= rad; dc++)
                {
                    int rr = r0 + dr, cc = c0 + dc;
                    if (rr < 0 || rr >= MAP_ROWS || cc < 0 || cc >= MAP_COLS) continue;
                    if (s_cspaceGrid[rr, cc] == CELL_FREE)
                    {
                        int d2 = dr * dr + dc * dc;
                        if (d2 < bestD2) { bestD2 = d2; bestR = rr; bestC = cc; }
                    }
                }
            }
            if (bestR >= 0) { ioR = bestR; ioC = bestC; return true; }
        }
        return false;
    }

    private struct HeapNode { public int R, C; public float F; }

    public static List<double[]> PlanPath(float startX, float startY, float goalX, float goalY, bool allowStartSnap)
    {
        var res = new List<double[]>();
        if (!s_inited) ResetCSpace();

        int sr, sc, gr, gc;
        if (!WorldToGrid(startX, startY, out sr, out sc) || !WorldToGrid(goalX, goalY, out gr, out gc))
            return res;

        float effStartX = startX, effStartY = startY;
        if (s_cspaceGrid[sr, sc] != CELL_FREE)
        {
            if (!allowStartSnap || !SnapToNearestFree(ref sr, ref sc, 4)) return res;
            effStartX = GetWorldX(sc);
            effStartY = GetWorldY(sr);
        }
        if (s_cspaceGrid[gr, gc] != CELL_FREE) return res;

        if (sr == gr && sc == gc)
        {
            if (!IsLineClear(effStartX, effStartY, goalX, goalY)) return res;
            res.Add(new double[] { effStartX, effStartY });
            res.Add(new double[] { goalX, goalY });
            return res;
        }

        float[,] gScore = new float[MAP_ROWS, MAP_COLS];
        int[,] parentR = new int[MAP_ROWS, MAP_COLS];
        int[,] parentC = new int[MAP_ROWS, MAP_COLS];
        bool[,] closed = new bool[MAP_ROWS, MAP_COLS];
        for (int r = 0; r < MAP_ROWS; r++)
            for (int c = 0; c < MAP_COLS; c++)
            {
                gScore[r, c] = 1e9f;
                parentR[r, c] = -1;
                parentC[r, c] = -1;
            }

        var heap = new List<HeapNode>(1024);
        Action<int, int, float> push = (r, c, f) =>
        {
            heap.Add(new HeapNode { R = r, C = c, F = f });
            int idx = heap.Count - 1;
            while (idx > 0)
            {
                int p = (idx - 1) / 2;
                if (heap[idx].F < heap[p].F) { var t = heap[idx]; heap[idx] = heap[p]; heap[p] = t; idx = p; }
                else break;
            }
        };
        Func<HeapNode> pop = () =>
        {
            var top = heap[0];
            int last = heap.Count - 1;
            heap[0] = heap[last];
            heap.RemoveAt(last);
            int idx = 0;
            while (true)
            {
                int l = 2 * idx + 1, rgt = 2 * idx + 2, sm = idx;
                if (l < heap.Count && heap[l].F < heap[sm].F) sm = l;
                if (rgt < heap.Count && heap[rgt].F < heap[sm].F) sm = rgt;
                if (sm != idx) { var t = heap[idx]; heap[idx] = heap[sm]; heap[sm] = t; idx = sm; }
                else break;
            }
            return top;
        };

        gScore[sr, sc] = 0.0f;
        push(sr, sc, Heuristic(sr, sc, gr, gc));
        bool found = false;

        while (heap.Count > 0)
        {
            var cur = pop();
            if (cur.R == gr && cur.C == gc) { found = true; break; }
            if (closed[cur.R, cur.C]) continue;
            closed[cur.R, cur.C] = true;

            float curG = gScore[cur.R, cur.C];
            for (int i = 0; i < 8; i++)
            {
                int nr = cur.R + DIR_R[i], nc = cur.C + DIR_C[i];
                if (nr < 0 || nr >= MAP_ROWS || nc < 0 || nc >= MAP_COLS) continue;
                if (s_cspaceGrid[nr, nc] != CELL_FREE) continue;
                if (i >= 4)
                {
                    if (s_cspaceGrid[cur.R + DIR_R[i], cur.C] != CELL_FREE ||
                        s_cspaceGrid[cur.R, cur.C + DIR_C[i]] != CELL_FREE)
                        continue;
                }
                float tentG = curG + DIR_W[i] + CellProximityPenalty(nr, nc);
                if (tentG < gScore[nr, nc])
                {
                    gScore[nr, nc] = tentG;
                    parentR[nr, nc] = cur.R;
                    parentC[nr, nc] = cur.C;
                    push(nr, nc, tentG + Heuristic(nr, nc, gr, gc));
                }
            }
        }

        if (!found) return res;

        var rev = new List<int[]>(256);
        int tr = gr, tc = gc;
        while (rev.Count < 600)
        {
            rev.Add(new[] { tr, tc });
            if (tr == sr && tc == sc) break;
            int pr = parentR[tr, tc], pc = parentC[tr, tc];
            if (pr < 0 || pc < 0) return res;
            tr = pr; tc = pc;
        }
        if (rev.Count == 0 || rev[rev.Count - 1][0] != sr || rev[rev.Count - 1][1] != sc) return res;
        rev.Reverse();

        res.Add(new double[] { effStartX, effStartY });
        int curIdx = 0;
        float cx = effStartX, cy = effStartY;
        while (curIdx < rev.Count - 1)
        {
            if (IsLineClear(cx, cy, goalX, goalY))
            {
                res.Add(new double[] { goalX, goalY });
                break;
            }
            int furth = curIdx + 1;
            for (int ni = rev.Count - 1; ni > curIdx; ni--)
            {
                float nx = (ni == rev.Count - 1) ? goalX : GetWorldX(rev[ni][1]);
                float ny = (ni == rev.Count - 1) ? goalY : GetWorldY(rev[ni][0]);
                if (IsLineClear(cx, cy, nx, ny)) { furth = ni; break; }
            }
            curIdx = furth;
            if (curIdx == rev.Count - 1)
            {
                res.Add(new double[] { goalX, goalY });
                break;
            }
            cx = GetWorldX(rev[curIdx][1]);
            cy = GetWorldY(rev[curIdx][0]);
            res.Add(new double[] { cx, cy });
        }
        return res;
    }

    public static List<double[]> PlanPathToFinish(float startX, float startY, bool allowStartSnap)
    {
        float[] goalXs = { 150.0f, 130.0f, 166.0f, 138.0f, 158.0f };
        for (int i = 0; i < goalXs.Length; i++)
        {
            var p = PlanPath(startX, startY, goalXs[i], -8.0f, allowStartSnap);
            if (p.Count >= 2) return p;
        }
        return new List<double[]>();
    }

    public static bool IsPathBlocked(List<double[]> wps)
    {
        if (wps == null || wps.Count < 2) return false;
        for (int i = 0; i < wps.Count - 1; i++)
        {
            if (!IsLineClear((float)wps[i][0], (float)wps[i][1], (float)wps[i + 1][0], (float)wps[i + 1][1]))
                return true;
        }
        return false;
    }

    private static double DistPointToBox(double px, double py, double x0, double y0, double x1, double y1)
    {
        double dx = (px < x0) ? (x0 - px) : (px > x1 ? (px - x1) : 0.0);
        double dy = (py < y0) ? (y0 - py) : (py > y1 ? (py - y1) : 0.0);
        return Math.Sqrt(dx * dx + dy * dy);
    }

    public static double DistToObstaclesCm(double px, double py, double[] dynX, double[] dynY)
    {
        double minD = 999.0;
        double dLeft  = (px <= 0.0) ? 0.0 : px;
        double dRight = (px >= 180.0) ? 0.0 : (180.0 - px);
        double dTop   = (py >= 120.0) ? 0.0 : (120.0 - py);
        if (dLeft < minD) minD = dLeft;
        if (dRight < minD) minD = dRight;
        if (dTop < minD) minD = dTop;

        double dU = DistPointToBox(px, py, 60.0, 0.0, 120.0, 60.0);
        if (dU < minD) minD = dU;

        for (int i = 0; i < 17; i++)
        {
            double cx = PILLARS[i, 0], cy = PILLARS[i, 1];
            double d = DistPointToBox(px, py, cx - 5.0, cy - 5.0, cx + 5.0, cy + 5.0);
            if (d < minD) minD = d;
        }

        if (dynX != null && dynY != null)
        {
            for (int i = 0; i < dynX.Length; i++)
            {
                double d = DistPointToBox(px, py, dynX[i] - 5.0, dynY[i] - 5.0, dynX[i] + 5.0, dynY[i] + 5.0);
                if (d < minD) minD = d;
            }
        }
        return minD;
    }

    public static double[] ComputeClearances(double rx, double ry, double headingRad, double[] dynX, double[] dynY)
    {
        double centerClr = DistToObstaclesCm(rx, ry, dynX, dynY);
        double c = Math.Cos(headingRad), s = Math.Sin(headingRad);
        double[,] pts = new double[,] {
            { 5.0,  5.73 }, { 5.0, -5.73 }, { -5.0, -5.73 }, { -5.0,  5.73 },
            { 5.0,  0.0  }, { -5.0, 0.0  }, {  0.0,  5.73 }, {  0.0, -5.73 }
        };
        double chassisClr = 999.0;
        for (int i = 0; i < 8; i++)
        {
            double wx = rx + pts[i, 0] * c - pts[i, 1] * s;
            double wy = ry + pts[i, 0] * s + pts[i, 1] * c;
            double d = DistToObstaclesCm(wx, wy, dynX, dynY);
            if (d < chassisClr) chassisClr = d;
        }
        return new double[] { centerClr, chassisClr };
    }
}
'@
    Add-Type -TypeDefinition $astarCs -Language CSharp
}

[CheesyAStar]::LoadStaticGridFromCFile($script:ArenaMapPath)

# =============================================================================
# 17 TUNABLE PARAMETERS & ARENA CONSTANTS
# =============================================================================
$script:ParamSpecs = @(
    @{ Id=1;  Tab=0; Key="V1";     Label="Vận tốc Ô 1 - Khe hẹp xuất phát";       Unit="cm/s";  Min=10.0;  Max=30.0;    Inc=0.5;  Dec=1; Type="f1"; File="robot_config"; Macro="V_TARGET_O1_CMS" }
    @{ Id=2;  Tab=0; Key="V2";     Label="Vận tốc Ô 2 - Bo cua trái";             Unit="cm/s";  Min=10.0;  Max=30.0;    Inc=0.5;  Dec=1; Type="f1"; File="robot_config"; Macro="V_TARGET_O2_CMS" }
    @{ Id=3;  Tab=0; Key="V3";     Label="Vận tốc Ô 3 - Đoạn thẳng + bật ToF";    Unit="cm/s";  Min=10.0;  Max=30.0;    Inc=0.5;  Dec=1; Type="f1"; File="robot_config"; Macro="V_TARGET_O3_CMS" }
    @{ Id=4;  Tab=0; Key="V4";     Label="Vận tốc Ô 4 - Bo cua phải + quét ToF";  Unit="cm/s";  Min=10.0;  Max=30.0;    Inc=0.5;  Dec=1; Type="f1"; File="robot_config"; Macro="V_TARGET_O4_CMS" }
    @{ Id=5;  Tab=0; Key="V5";     Label="Vận tốc Ô 5 - Xuôi về đích";            Unit="cm/s";  Min=10.0;  Max=30.0;    Inc=0.5;  Dec=1; Type="f1"; File="robot_config"; Macro="V_TARGET_O5_CMS" }
    @{ Id=6;  Tab=0; Key="VDET";   Label="Trần tốc độ khi né vật cản động";       Unit="cm/s";  Min=10.0;  Max=30.0;    Inc=0.5;  Dec=1; Type="f1"; File="robot_config"; Macro="V_TARGET_DETOUR_MAX_CMS" }
    @{ Id=7;  Tab=0; Key="VMAX";   Label="Giới hạn tốc độ tối đa toàn xe";        Unit="cm/s";  Min=15.0;  Max=35.0;    Inc=0.5;  Dec=1; Type="f1"; File="robot_config"; Macro="ROBOT_MAX_SPEED_CMS" }
    @{ Id=8;  Tab=0; Key="ACC";    Label="Gia tốc tối đa chống trượt bánh";       Unit="cm/s²"; Min=20.0;  Max=150.0;   Inc=5.0;  Dec=1; Type="f1"; File="robot_config"; Macro="ROBOT_ACCEL_MAX_CMS2" }
    @{ Id=9;  Tab=1; Key="KP";     Label="Motor PI - Kp (trái & phải)";           Unit="-";     Min=10.0;  Max=600.0;   Inc=5.0;  Dec=1; Type="f1"; File="robot_config"; Macro="MOTOR_DEFAULT_KP" }
    @{ Id=10; Tab=1; Key="KI";     Label="Motor PI - Ki (tăng nếu hụt tốc)";      Unit="-";     Min=0.0;   Max=50.0;    Inc=0.1;  Dec=2; Type="f2"; File="robot_config"; Macro="MOTOR_DEFAULT_KI" }
    @{ Id=11; Tab=1; Key="WD";     Label="Đường kính bánh xe";                    Unit="cm";    Min=2.5;   Max=8.0;     Inc=0.05; Dec=2; Type="f2"; File="robot_config"; Macro="ROBOT_WHEEL_DIAM_CM" }
    @{ Id=12; Tab=1; Key="WB";     Label="Khoảng cách tâm 2 bánh";                Unit="cm";    Min=6.0;   Max=16.0;    Inc=0.05; Dec=2; Type="f2"; File="robot_config"; Macro="ROBOT_TRACK_WIDTH_CM" }
    @{ Id=13; Tab=1; Key="PPR";    Label="Số xung Encoder / vòng bánh (4x)";      Unit="ticks"; Min=100.0; Max=10000.0; Inc=10.0; Dec=0; Type="f0"; File="robot_config"; Macro="ROBOT_ENCODER_PPR" }
    @{ Id=14; Tab=2; Key="TOF";    Label="Tầm quét 3× VL53L0X";                   Unit="mm";    Min=150.0; Max=800.0;   Inc=10.0; Dec=0; Type="u";  File="robot_config"; Macro="TOF_SCAN_MAX_MM" }
    @{ Id=15; Tab=2; Key="SCANX";  Label="Tọa độ X bật quét ToF (Ô 3=60)";        Unit="cm";    Min=0.0;   Max=120.0;   Inc=5.0;  Dec=1; Type="f1"; File="robot_config"; Macro="FSM_SCAN_TRIGGER_X_CM" }
    @{ Id=16; Tab=2; Key="LD";     Label="Pure Pursuit Lookahead offset";         Unit="cm";    Min=4.0;   Max=12.0;    Inc=0.5;  Dec=1; Type="f1"; File="pure_pursuit"; Macro="PP_LOOKAHEAD_OFFSET_CM" }
    @{ Id=17; Tab=2; Key="YAWINV"; Label="Đảo chiều Yaw MPU6050 (0=Thường,1=Đảo)"; Unit="0|1";   Min=0.0;   Max=1.0;     Inc=1.0;  Dec=0; Type="i";  File="robot_config"; Macro="IMU_DEFAULT_YAW_INVERT" }
)

$script:StaticPillars = @(
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

$script:FsmStateNames = @{
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
            @{ Lnk="NAP_CODE.lnk"; Bat="NAP_CODE.bat"; Ico=$script:NapIcoPath; Desc="Build va Nap firmware STM32F411CEU6 qua ST-Link" },
            @{ Lnk="CHINH_THONG_SO.lnk"; Bat="CHINH_THONG_SO.bat"; Ico=$script:TunerIcoPath; Desc="Dashboard xem quy dao 2D + chinh 17 thong so Cheesy Robot" }
        )) {
            $lnkPath = Join-Path $script:RootDir $it.Lnk
            if (-not (Test-Path $lnkPath)) {
                $sc = $ws.CreateShortcut($lnkPath)
                $sc.TargetPath = Join-Path $script:RootDir $it.Bat
                $sc.WorkingDirectory = $script:RootDir
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
    $txtRobot = [IO.File]::ReadAllText($script:RobotCfgPath, [Text.Encoding]::UTF8)
    $txtPP    = [IO.File]::ReadAllText($script:PurePursuitPath, [Text.Encoding]::UTF8)
    foreach ($p in $script:ParamSpecs) {
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
    $path = if ($Spec.File -eq "pure_pursuit") { $script:PurePursuitPath } else { $script:RobotCfgPath }
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

function Choose-DemoO4Obstacle([object]$CleanPath) {
    $cands = @(
        @(145.0, 75.0), @(140.0, 75.0), @(150.0, 75.0),
        @(145.0, 70.0), @(140.0, 80.0), @(155.0, 75.0),
        @(135.0, 75.0), @(145.0, 85.0)
    )
    foreach ($cand in $cands) {
        [CheesyAStar]::ResetCSpace()
        [CheesyAStar]::AddDynamicObstacleBox([float]$cand[0], [float]$cand[1], 5.0, 6.75)
        $blocked = [CheesyAStar]::IsPathBlocked($CleanPath)
        $detour  = [CheesyAStar]::PlanPathToFinish(135.0, 95.0, $true)
        if ($blocked -and $detour.Count -ge 2) {
            [CheesyAStar]::ResetCSpace()
            return @{ X=[double]$cand[0]; Y=[double]$cand[1] }
        }
    }
    [CheesyAStar]::ResetCSpace()
    return @{ X=145.0; Y=75.0 }
}

function Raycast-DemoToF([double]$rx, [double]$ry, [double]$phiRad, [double]$mountCm, [bool]$hasDemoObs, [double]$obsX, [double]$obsY) {
    $c = [math]::Cos($phiRad)
    $s = [math]::Sin($phiRad)
    $ox = $rx + $mountCm * $c
    $oy = $ry + $mountCm * $s
    for ($d = 0.5; $d -le 80.0; $d += 0.5) {
        $px = $ox + $d * $c
        $py = $oy + $d * $s
        if ($px -le 0.0 -or $px -ge 180.0 -or $py -ge 120.0) { return [int][math]::Round($d * 10.0) }
        if ($px -ge 60.0 -and $px -le 120.0 -and $py -ge 0.0 -and $py -le 60.0) { return [int][math]::Round($d * 10.0) }
        foreach ($p in $script:StaticPillars) {
            if ($px -ge ($p.X - 5.0) -and $px -le ($p.X + 5.0) -and $py -ge ($p.Y - 5.0) -and $py -le ($p.Y + 5.0)) {
                return [int][math]::Round($d * 10.0)
            }
        }
        if ($hasDemoObs) {
            if ($px -ge ($obsX - 5.0) -and $px -le ($obsX + 5.0) -and $py -ge ($obsY - 5.0) -and $py -le ($obsY + 5.0)) {
                return [int][math]::Round($d * 10.0)
            }
        }
    }
    return 8190
}

function Build-DashboardForm([System.Collections.IDictionary]$InitialValues) {
    # All colors in script scope -> never null inside any event handler
    $script:Theme = @{
        Bg    = [System.Drawing.Color]::FromArgb(15, 23, 42)
        Panel = [System.Drawing.Color]::FromArgb(30, 41, 59)
        Input = [System.Drawing.Color]::FromArgb(17, 24, 39)
        Fg    = [System.Drawing.Color]::FromArgb(241, 245, 249)
        Muted = [System.Drawing.Color]::FromArgb(148, 163, 184)
        Cyan  = [System.Drawing.Color]::FromArgb(56, 189, 248)
        Blue  = [System.Drawing.Color]::FromArgb(37, 99, 235)
        Green = [System.Drawing.Color]::FromArgb(5, 150, 105)
        Amber = [System.Drawing.Color]::FromArgb(217, 119, 6)
        Teal  = [System.Drawing.Color]::FromArgb(13, 148, 136)
        Red   = [System.Drawing.Color]::FromArgb(220, 38, 38)
        Slate = [System.Drawing.Color]::FromArgb(71, 85, 105)
    }

    $script:UI = @{}
    $script:Fn = @{}

    $fontUi    = New-Object System.Drawing.Font("Segoe UI", 8.5)
    $fontBold  = New-Object System.Drawing.Font("Segoe UI", 8.5, [System.Drawing.FontStyle]::Bold)
    $fontSmall = New-Object System.Drawing.Font("Segoe UI", 7.8)
    $fontMono  = New-Object System.Drawing.Font("Consolas", 8.2)

    $form = New-Object System.Windows.Forms.Form
    $form.Text = "Cheesy Robot - Bảng Giám Sát Quỹ Đạo A* & Chỉnh Thông Số (STM32F411CEU6)"
    $form.AutoScaleMode = [System.Windows.Forms.AutoScaleMode]::None
    $form.ClientSize = New-Object System.Drawing.Size(1060, 590)
    $form.FormBorderStyle = [System.Windows.Forms.FormBorderStyle]::FixedSingle
    $form.MaximizeBox = $false
    $form.StartPosition = [System.Windows.Forms.FormStartPosition]::CenterScreen
    $form.BackColor = $script:Theme.Bg
    $form.ForeColor = $script:Theme.Fg
    $form.Font = $fontUi
    if (Test-Path $script:TunerIcoPath) {
        $form.Icon = New-Object System.Drawing.Icon($script:TunerIcoPath)
    }
    $script:UI.Form = $form

    # =========================================================================
    # LEFT COLUMN (x=10, w=480): 2D Map + Map Bar + Telemetry Panel
    # =========================================================================
    $script:mapW = 480
    $script:mapH = 352
    $script:mapBmp = New-Object System.Drawing.Bitmap($script:mapW, $script:mapH)

    $picMap = New-Object System.Windows.Forms.PictureBox
    $picMap.Location = New-Object System.Drawing.Point(10, 8)
    $picMap.Size = New-Object System.Drawing.Size($script:mapW, $script:mapH)
    $picMap.BackColor = $script:Theme.Input
    $picMap.Image = $script:mapBmp
    $picMap.BorderStyle = [System.Windows.Forms.BorderStyle]::FixedSingle
    $form.Controls.Add($picMap)
    $script:UI.PicMap = $picMap

    $mapBar = New-Object System.Windows.Forms.Panel
    $mapBar.Location = New-Object System.Drawing.Point(10, 366)
    $mapBar.Size = New-Object System.Drawing.Size(480, 34)
    $mapBar.BackColor = $script:Theme.Panel
    $form.Controls.Add($mapBar)

    $btnDemo = New-Object System.Windows.Forms.Button
    $btnDemo.Text = "Mô phỏng Demo"
    $btnDemo.Location = New-Object System.Drawing.Point(4, 4)
    $btnDemo.Size = New-Object System.Drawing.Size(118, 26)
    $btnDemo.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnDemo.BackColor = $script:Theme.Blue
    $btnDemo.ForeColor = $script:Theme.Fg
    $btnDemo.Font = $fontBold
    $mapBar.Controls.Add($btnDemo)
    $script:UI.BtnDemo = $btnDemo

    $btnDemoObs = New-Object System.Windows.Forms.Button
    $btnDemoObs.Text = "Cọc Demo Ô 4: TẮT"
    $btnDemoObs.Location = New-Object System.Drawing.Point(126, 4)
    $btnDemoObs.Size = New-Object System.Drawing.Size(134, 26)
    $btnDemoObs.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnDemoObs.BackColor = $script:Theme.Slate
    $btnDemoObs.ForeColor = $script:Theme.Fg
    $mapBar.Controls.Add($btnDemoObs)
    $script:UI.BtnDemoObs = $btnDemoObs

    $btnClearMap = New-Object System.Windows.Forms.Button
    $btnClearMap.Text = "Xóa quỹ đạo"
    $btnClearMap.Location = New-Object System.Drawing.Point(264, 4)
    $btnClearMap.Size = New-Object System.Drawing.Size(102, 26)
    $btnClearMap.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnClearMap.BackColor = $script:Theme.Slate
    $btnClearMap.ForeColor = $script:Theme.Fg
    $mapBar.Controls.Add($btnClearMap)
    $script:UI.BtnClearMap = $btnClearMap

    $btnExportCsv = New-Object System.Windows.Forms.Button
    $btnExportCsv.Text = "Xuất CSV điểm"
    $btnExportCsv.Location = New-Object System.Drawing.Point(370, 4)
    $btnExportCsv.Size = New-Object System.Drawing.Size(106, 26)
    $btnExportCsv.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnExportCsv.BackColor = $script:Theme.Teal
    $btnExportCsv.ForeColor = $script:Theme.Fg
    $mapBar.Controls.Add($btnExportCsv)
    $script:UI.BtnExportCsv = $btnExportCsv

    $telPanel = New-Object System.Windows.Forms.Panel
    $telPanel.Location = New-Object System.Drawing.Point(10, 406)
    $telPanel.Size = New-Object System.Drawing.Size(480, 174)
    $telPanel.BackColor = $script:Theme.Panel
    $form.Controls.Add($telPanel)

    $lblTelTitle = New-Object System.Windows.Forms.Label
    $lblTelTitle.Text = "THÔNG SỐ XE TRỰC TIẾP (EKF + 3× VL53L0X + ĐỘNG CƠ)"
    $lblTelTitle.Location = New-Object System.Drawing.Point(8, 6)
    $lblTelTitle.Size = New-Object System.Drawing.Size(464, 18)
    $lblTelTitle.ForeColor = $script:Theme.Cyan
    $lblTelTitle.Font = $fontBold
    $telPanel.Controls.Add($lblTelTitle)

    $lblPose = New-Object System.Windows.Forms.Label
    $lblPose.Location = New-Object System.Drawing.Point(8, 28)
    $lblPose.Size = New-Object System.Drawing.Size(464, 20)
    $lblPose.Font = $fontMono
    $telPanel.Controls.Add($lblPose)
    $script:UI.LblPose = $lblPose

    $lblState = New-Object System.Windows.Forms.Label
    $lblState.Location = New-Object System.Drawing.Point(8, 50)
    $lblState.Size = New-Object System.Drawing.Size(464, 20)
    $lblState.Font = $fontMono
    $telPanel.Controls.Add($lblState)
    $script:UI.LblState = $lblState

    $lblToF = New-Object System.Windows.Forms.Label
    $lblToF.Location = New-Object System.Drawing.Point(8, 72)
    $lblToF.Size = New-Object System.Drawing.Size(464, 20)
    $lblToF.Font = $fontMono
    $telPanel.Controls.Add($lblToF)
    $script:UI.LblToF = $lblToF

    $lblWheels = New-Object System.Windows.Forms.Label
    $lblWheels.Location = New-Object System.Drawing.Point(8, 94)
    $lblWheels.Size = New-Object System.Drawing.Size(464, 20)
    $lblWheels.Font = $fontMono
    $telPanel.Controls.Add($lblWheels)
    $script:UI.LblWheels = $lblWheels

    $lblClearance = New-Object System.Windows.Forms.Label
    $lblClearance.Location = New-Object System.Drawing.Point(8, 116)
    $lblClearance.Size = New-Object System.Drawing.Size(464, 20)
    $lblClearance.Font = $fontMono
    $lblClearance.ForeColor = $script:Theme.Cyan
    $telPanel.Controls.Add($lblClearance)
    $script:UI.LblClearance = $lblClearance

    $lblLegend = New-Object System.Windows.Forms.Label
    $lblLegend.Text = "Đứt nét xanh: Đường A* dự kiến | Xanh dương: Quỹ đạo xe | Vàng: Điểm qua | Đỏ: Cọc động"
    $lblLegend.Location = New-Object System.Drawing.Point(8, 142)
    $lblLegend.Size = New-Object System.Drawing.Size(464, 26)
    $lblLegend.Font = $fontSmall
    $lblLegend.ForeColor = $script:Theme.Muted
    $telPanel.Controls.Add($lblLegend)

    # =========================================================================
    # RIGHT COLUMN (x=500, w=550, right=1050 <= 1060): Header + Tabs + Actions + Log
    # =========================================================================
    $hdr = New-Object System.Windows.Forms.Panel
    $hdr.Location = New-Object System.Drawing.Point(500, 8)
    $hdr.Size = New-Object System.Drawing.Size(550, 62)
    $hdr.BackColor = $script:Theme.Panel
    $form.Controls.Add($hdr)

    $lblCom = New-Object System.Windows.Forms.Label
    $lblCom.Text = "Cổng HC-05:"
    $lblCom.Location = New-Object System.Drawing.Point(8, 10)
    $lblCom.Size = New-Object System.Drawing.Size(74, 20)
    $lblCom.Font = $fontBold
    $hdr.Controls.Add($lblCom)

    $cmbCom = New-Object System.Windows.Forms.ComboBox
    $cmbCom.Location = New-Object System.Drawing.Point(84, 7)
    $cmbCom.Size = New-Object System.Drawing.Size(192, 23)
    $cmbCom.DropDownStyle = [System.Windows.Forms.ComboBoxStyle]::DropDownList
    $cmbCom.BackColor = $script:Theme.Input
    $cmbCom.ForeColor = $script:Theme.Fg
    $hdr.Controls.Add($cmbCom)
    $script:UI.CmbCom = $cmbCom

    $btnScan = New-Object System.Windows.Forms.Button
    $btnScan.Text = "Quét COM"
    $btnScan.Location = New-Object System.Drawing.Point(282, 6)
    $btnScan.Size = New-Object System.Drawing.Size(74, 26)
    $btnScan.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnScan.BackColor = $script:Theme.Slate
    $btnScan.ForeColor = $script:Theme.Fg
    $hdr.Controls.Add($btnScan)
    $script:UI.BtnScan = $btnScan

    $btnConn = New-Object System.Windows.Forms.Button
    $btnConn.Text = "Kết nối BLE"
    $btnConn.Location = New-Object System.Drawing.Point(360, 6)
    $btnConn.Size = New-Object System.Drawing.Size(94, 26)
    $btnConn.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnConn.BackColor = $script:Theme.Blue
    $btnConn.ForeColor = $script:Theme.Fg
    $btnConn.Font = $fontBold
    $hdr.Controls.Add($btnConn)
    $script:UI.BtnConn = $btnConn

    $btnRead = New-Object System.Windows.Forms.Button
    $btnRead.Text = "Đọc từ xe"
    $btnRead.Location = New-Object System.Drawing.Point(458, 6)
    $btnRead.Size = New-Object System.Drawing.Size(84, 26)
    $btnRead.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnRead.BackColor = $script:Theme.Teal
    $btnRead.ForeColor = $script:Theme.Fg
    $btnRead.Enabled = $false
    $hdr.Controls.Add($btnRead)
    $script:UI.BtnRead = $btnRead

    $lblStatus = New-Object System.Windows.Forms.Label
    $lblStatus.Text = "Trạng thái: Offline (nhấn 'Mô phỏng Demo' bên trái để xem thử quỹ đạo A*)"
    $lblStatus.Location = New-Object System.Drawing.Point(8, 38)
    $lblStatus.Size = New-Object System.Drawing.Size(534, 18)
    $lblStatus.ForeColor = $script:Theme.Cyan
    $hdr.Controls.Add($lblStatus)
    $script:UI.LblStatus = $lblStatus

    # Tabs (17 Parameters)
    $tabs = New-Object System.Windows.Forms.TabControl
    $tabs.Location = New-Object System.Drawing.Point(500, 76)
    $tabs.Size = New-Object System.Drawing.Size(550, 304)
    $form.Controls.Add($tabs)
    $script:UI.Tabs = $tabs

    $tabNames = @("1. Vận tốc Ô 1..Ô 5 & Gia tốc", "2. Động cơ PI & Cơ khí", "3. Cảm biến ToF & Bám đường")
    $tabPages = @()
    foreach ($tName in $tabNames) {
        $tp = New-Object System.Windows.Forms.TabPage
        $tp.Text = $tName
        $tp.BackColor = $script:Theme.Panel
        $tp.ForeColor = $script:Theme.Fg
        $tabs.TabPages.Add($tp)
        $tabPages += $tp
    }

    $script:numControls = @{}
    $tabRowY = @(8, 8, 8)

    foreach ($spec in $script:ParamSpecs) {
        $tIdx = $spec.Tab
        $tp = $tabPages[$tIdx]
        $y = $tabRowY[$tIdx]

        $lblKey = New-Object System.Windows.Forms.Label
        $lblKey.Text = "[$($spec.Key)]"
        $lblKey.Location = New-Object System.Drawing.Point(6, ($y + 3))
        $lblKey.Size = New-Object System.Drawing.Size(58, 20)
        $lblKey.Font = $fontMono
        $lblKey.ForeColor = $script:Theme.Cyan
        $tp.Controls.Add($lblKey)

        $lblName = New-Object System.Windows.Forms.Label
        $lblName.Text = $spec.Label
        $lblName.Location = New-Object System.Drawing.Point(66, ($y + 2))
        $lblName.Size = New-Object System.Drawing.Size(236, 20)
        $tp.Controls.Add($lblName)

        $num = New-Object System.Windows.Forms.NumericUpDown
        $num.Location = New-Object System.Drawing.Point(304, $y)
        $num.Size = New-Object System.Drawing.Size(82, 22)
        $num.Minimum = [decimal]$spec.Min
        $num.Maximum = [decimal]$spec.Max
        $num.Increment = [decimal]$spec.Inc
        $num.DecimalPlaces = [int]$spec.Dec
        $num.BackColor = $script:Theme.Input
        $num.ForeColor = $script:Theme.Fg
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
        $lblRange.Location = New-Object System.Drawing.Point(392, ($y + 3))
        $lblRange.Size = New-Object System.Drawing.Size(142, 20)
        $lblRange.ForeColor = $script:Theme.Muted
        $lblRange.Font = $fontSmall
        $tp.Controls.Add($lblRange)

        $tabRowY[$tIdx] = $y + 29
    }

    # Quick uniform speed row
    $lblAll = New-Object System.Windows.Forms.Label
    $lblAll.Text = "Đặt nhanh chung cả 5 ô (V1..V5):"
    $lblAll.Location = New-Object System.Drawing.Point(66, 245)
    $lblAll.Size = New-Object System.Drawing.Size(232, 20)
    $lblAll.ForeColor = $script:Theme.Cyan
    $lblAll.Font = $fontBold
    $tabPages[0].Controls.Add($lblAll)

    $numAll = New-Object System.Windows.Forms.NumericUpDown
    $numAll.Location = New-Object System.Drawing.Point(304, 242)
    $numAll.Size = New-Object System.Drawing.Size(82, 22)
    $numAll.Minimum = [decimal]10.0
    $numAll.Maximum = [decimal]30.0
    $numAll.Increment = [decimal]0.5
    $numAll.DecimalPlaces = 1
    $numAll.Value = [decimal]20.0
    $numAll.BackColor = $script:Theme.Input
    $numAll.ForeColor = $script:Theme.Fg
    $tabPages[0].Controls.Add($numAll)
    $script:UI.NumAll = $numAll

    $btnApplyAll = New-Object System.Windows.Forms.Button
    $btnApplyAll.Text = "Áp cho 5 ô"
    $btnApplyAll.Location = New-Object System.Drawing.Point(392, 241)
    $btnApplyAll.Size = New-Object System.Drawing.Size(102, 24)
    $btnApplyAll.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnApplyAll.BackColor = $script:Theme.Blue
    $btnApplyAll.ForeColor = $script:Theme.Fg
    $tabPages[0].Controls.Add($btnApplyAll)
    $script:UI.BtnApplyAll = $btnApplyAll

    # Action Panel
    $actPanel = New-Object System.Windows.Forms.Panel
    $actPanel.Location = New-Object System.Drawing.Point(500, 386)
    $actPanel.Size = New-Object System.Drawing.Size(550, 70)
    $actPanel.BackColor = $script:Theme.Panel
    $form.Controls.Add($actPanel)

    $btnSendLive = New-Object System.Windows.Forms.Button
    $btnSendLive.Text = "Gửi LIVE Bluetooth"
    $btnSendLive.Location = New-Object System.Drawing.Point(6, 6)
    $btnSendLive.Size = New-Object System.Drawing.Size(132, 27)
    $btnSendLive.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnSendLive.BackColor = $script:Theme.Blue
    $btnSendLive.ForeColor = $script:Theme.Fg
    $btnSendLive.Font = $fontBold
    $actPanel.Controls.Add($btnSendLive)
    $script:UI.BtnSendLive = $btnSendLive

    $btnSaveFlash = New-Object System.Windows.Forms.Button
    $btnSaveFlash.Text = "Lưu Flash (CFG:SAVE)"
    $btnSaveFlash.Location = New-Object System.Drawing.Point(142, 6)
    $btnSaveFlash.Size = New-Object System.Drawing.Size(138, 27)
    $btnSaveFlash.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnSaveFlash.BackColor = $script:Theme.Green
    $btnSaveFlash.ForeColor = $script:Theme.Fg
    $btnSaveFlash.Font = $fontBold
    $actPanel.Controls.Add($btnSaveFlash)
    $script:UI.BtnSaveFlash = $btnSaveFlash

    $btnSaveCode = New-Object System.Windows.Forms.Button
    $btnSaveCode.Text = "Lưu Code (.h)"
    $btnSaveCode.Location = New-Object System.Drawing.Point(284, 6)
    $btnSaveCode.Size = New-Object System.Drawing.Size(124, 27)
    $btnSaveCode.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnSaveCode.BackColor = $script:Theme.Amber
    $btnSaveCode.ForeColor = $script:Theme.Fg
    $btnSaveCode.Font = $fontBold
    $actPanel.Controls.Add($btnSaveCode)
    $script:UI.BtnSaveCode = $btnSaveCode

    $btnResetDef = New-Object System.Windows.Forms.Button
    $btnResetDef.Text = "Khôi phục mặc định"
    $btnResetDef.Location = New-Object System.Drawing.Point(412, 6)
    $btnResetDef.Size = New-Object System.Drawing.Size(132, 27)
    $btnResetDef.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnResetDef.BackColor = $script:Theme.Slate
    $btnResetDef.ForeColor = $script:Theme.Fg
    $actPanel.Controls.Add($btnResetDef)
    $script:UI.BtnResetDef = $btnResetDef

    $btnRunAuto = New-Object System.Windows.Forms.Button
    $btnRunAuto.Text = "Chạy tự hành (RUN:2)"
    $btnRunAuto.Location = New-Object System.Drawing.Point(6, 38)
    $btnRunAuto.Size = New-Object System.Drawing.Size(132, 26)
    $btnRunAuto.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnRunAuto.BackColor = $script:Theme.Teal
    $btnRunAuto.ForeColor = $script:Theme.Fg
    $actPanel.Controls.Add($btnRunAuto)
    $script:UI.BtnRunAuto = $btnRunAuto

    $btnStop = New-Object System.Windows.Forms.Button
    $btnStop.Text = "DỪNG KHẨN (STOP)"
    $btnStop.Location = New-Object System.Drawing.Point(142, 38)
    $btnStop.Size = New-Object System.Drawing.Size(138, 26)
    $btnStop.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnStop.BackColor = $script:Theme.Red
    $btnStop.ForeColor = $script:Theme.Fg
    $btnStop.Font = $fontBold
    $actPanel.Controls.Add($btnStop)
    $script:UI.BtnStop = $btnStop

    $btnZero = New-Object System.Windows.Forms.Button
    $btnZero.Text = "Reset tọa độ (ZERO)"
    $btnZero.Location = New-Object System.Drawing.Point(284, 38)
    $btnZero.Size = New-Object System.Drawing.Size(124, 26)
    $btnZero.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnZero.BackColor = $script:Theme.Slate
    $btnZero.ForeColor = $script:Theme.Fg
    $actPanel.Controls.Add($btnZero)
    $script:UI.BtnZero = $btnZero

    $btnFlashStlink = New-Object System.Windows.Forms.Button
    $btnFlashStlink.Text = "Build && Nạp ST-Link"
    $btnFlashStlink.Location = New-Object System.Drawing.Point(412, 38)
    $btnFlashStlink.Size = New-Object System.Drawing.Size(132, 26)
    $btnFlashStlink.FlatStyle = [System.Windows.Forms.FlatStyle]::Flat
    $btnFlashStlink.BackColor = $script:Theme.Slate
    $btnFlashStlink.ForeColor = $script:Theme.Cyan
    $actPanel.Controls.Add($btnFlashStlink)
    $script:UI.BtnFlashStlink = $btnFlashStlink

    # Log Box
    $txtLog = New-Object System.Windows.Forms.TextBox
    $txtLog.Location = New-Object System.Drawing.Point(500, 462)
    $txtLog.Size = New-Object System.Drawing.Size(550, 118)
    $txtLog.Multiline = $true
    $txtLog.ReadOnly = $true
    $txtLog.ScrollBars = [System.Windows.Forms.ScrollBars]::Vertical
    $txtLog.BackColor = $script:Theme.Input
    $txtLog.ForeColor = $script:Theme.Fg
    $txtLog.Font = $fontMono
    $form.Controls.Add($txtLog)
    $script:UI.TxtLog = $txtLog

    # =========================================================================
    # LIVE TELEMETRY, C-SPACE A* PLANNER & DEMO STATE (ALL IN $script: SCOPE)
    # =========================================================================
    $script:serialPort   = $null
    $script:rxBuffer     = ""
    $script:trail        = New-Object System.Collections.Generic.List[object]
    $script:breadcrumbs  = New-Object System.Collections.Generic.List[object]
    $script:dynObs       = New-Object System.Collections.Generic.List[object]
    $script:plannedPath  = $null
    $script:demoActive   = $false
    $script:demoHasObs   = $false
    $script:demoObsAdded = $false
    $script:demoWpIdx    = 1
    $script:demoObsTarget = @{ X=145.0; Y=75.0 }
    $script:liveState    = @{
        X = 30.0; Y = -6.0; Heading = 90.0; Vbat = 7.98; Mode = 2
        Dl = 8190; Df = 8190; Dr = 8190
        VlExp = 0.0; VrExp = 0.0; VlReal = 0.0; VrReal = 0.0
        FsmState = 0; Score = 0
        MinCenterClr = 999.0; MinChassisClr = 999.0
    }

    # Precompute clean A* path at startup so planned route renders immediately
    [CheesyAStar]::ResetCSpace()
    $script:plannedPath = [CheesyAStar]::PlanPathToFinish(30.0, -6.0, $false)
    $script:demoObsTarget = Choose-DemoO4Obstacle $script:plannedPath

    $script:Fn.WriteLog = {
        param([string]$msg)
        $ts = (Get-Date).ToString("HH:mm:ss")
        $script:UI.TxtLog.AppendText("[$ts] $msg`r`n")
    }

    $script:Fn.UpdateTelLabels = {
        $st = $script:liveState
        $fsmTxt = if ($script:FsmStateNames.ContainsKey([int]$st.FsmState)) { $script:FsmStateNames[[int]$st.FsmState] } else { "STATE_$($st.FsmState)" }
        $script:UI.LblPose.Text   = ("Tọa độ EKF: X={0,5:F1} cm | Y={1,5:F1} cm | Yaw={2,6:F1}° | Điểm: {3}" -f $st.X, $st.Y, $st.Heading, $script:breadcrumbs.Count)
        $script:UI.LblState.Text  = ("FSM: {0,-21} | Điểm: {1,3} | Pin: {2:F2} V" -f $fsmTxt, $st.Score, $st.Vbat)
        $dlTxt = if ($st.Dl -ge 8190) { "---" } else { "$($st.Dl)" }
        $dfTxt = if ($st.Df -ge 8190) { "---" } else { "$($st.Df)" }
        $drTxt = if ($st.Dr -ge 8190) { "---" } else { "$($st.Dr)" }
        $script:UI.LblToF.Text    = ("3× VL53L0X (mm): Trái(+45°)={0,-4} | Trước={1,-4} | Phải(-45°)={2,-4}" -f $dlTxt, $dfTxt, $drTxt)
        $script:UI.LblWheels.Text = ("Bánh (cm/s): Trái {0,4:F1}/{1,4:F1} | Phải {2,4:F1}/{3,4:F1} (Đặt/Thực)" -f $st.VlExp, $st.VlReal, $st.VrExp, $st.VrReal)

        $wpCnt = if ($null -ne $script:plannedPath) { $script:plannedPath.Count } else { 0 }
        $cenS = if ($st.MinCenterClr -gt 900) { "---" } else { ("{0:F2}" -f $st.MinCenterClr) }
        $chaS = if ($st.MinChassisClr -gt 900) { "---" } else { ("{0:F2}" -f $st.MinChassisClr) }
        $script:UI.LblClearance.Text = ("A* C-Space: {0} nút | Khoảng hở min: Tâm={1} cm, Thân={2} cm" -f $wpCnt, $cenS, $chaS)
    }

    $script:Fn.DrawArenaMap = {
        $g = [System.Drawing.Graphics]::FromImage($script:mapBmp)
        try {
            $g.SmoothingMode = [System.Drawing.Drawing2D.SmoothingMode]::AntiAlias
            $g.Clear([System.Drawing.Color]::FromArgb(13, 20, 34))

            $scale = [math]::Min(($script:mapW - 24.0) / 192.0, ($script:mapH - 24.0) / 140.0)
            $offX  = ($script:mapW - 192.0 * $scale) * 0.5
            $offY  = ($script:mapH - 140.0 * $scale) * 0.5

            $wx = { param([double]$x) return [float]($offX + ($x + 6.0) * $scale) }
            $wy = { param([double]$y) return [float]($offY + (126.0 - $y) * $scale) }

            # O 3..O 5 ToF Scan Zone tint
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

            # Zone dividers & labels
            $pDash = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(71, 85, 105), 1.0)
            $pDash.DashStyle = [System.Drawing.Drawing2D.DashStyle]::Dash
            $g.DrawLine($pDash, (& $wx 60.0), (& $wy 120.0), (& $wx 60.0), (& $wy 60.0))
            $g.DrawLine($pDash, (& $wx 120.0), (& $wy 120.0), (& $wx 120.0), (& $wy 60.0))
            $g.DrawLine($pDash, (& $wx 0.0), (& $wy 60.0), (& $wx 60.0), (& $wy 60.0))
            $g.DrawLine($pDash, (& $wx 120.0), (& $wy 60.0), (& $wx 180.0), (& $wy 60.0))
            $pDash.Dispose()

            $fZone = New-Object System.Drawing.Font("Segoe UI", 7.5, [System.Drawing.FontStyle]::Bold)
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

            # Outer walls
            $pWall = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(203, 213, 225), 2.0)
            $g.DrawLine($pWall, (& $wx 0.0), (& $wy 0.0), (& $wx 0.0), (& $wy 120.0))
            $g.DrawLine($pWall, (& $wx 0.0), (& $wy 120.0), (& $wx 180.0), (& $wy 120.0))
            $g.DrawLine($pWall, (& $wx 180.0), (& $wy 120.0), (& $wx 180.0), (& $wy 0.0))
            $pWall.Dispose()

            # Forbidden U-block
            $bU = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(45, 55, 72))
            $pU = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(148, 163, 184), 1.6)
            $ux = & $wx 60.0; $uy = & $wy 60.0; $uw = [float](60.0 * $scale); $uh = [float](60.0 * $scale)
            $g.FillRectangle($bU, $ux, $uy, $uw, $uh)
            $g.DrawRectangle($pU, $ux, $uy, $uw, $uh)
            $bU.Dispose(); $pU.Dispose()

            # 17 static pillars (10x10 cm)
            $bObs  = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(220, 71, 85, 105))
            $pObs  = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(148, 163, 184), 1.1)
            $bLbl  = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(226, 232, 240))
            $fObs  = New-Object System.Drawing.Font("Consolas", 6.2)
            $boxSz = [float](10.0 * $scale)
            foreach ($p in $script:StaticPillars) {
                $bx = & $wx ($p.X - 5.0)
                $by = & $wy ($p.Y + 5.0)
                $g.FillRectangle($bObs, $bx, $by, $boxSz, $boxSz)
                $g.DrawRectangle($pObs, $bx, $by, $boxSz, $boxSz)
                $g.DrawString($p.Name, $fObs, $bLbl, ($bx + 1.5), ($by + 3.0))
            }
            $bObs.Dispose(); $pObs.Dispose(); $bLbl.Dispose()

            # Pending demo obstacle preview (dashed faint outline before ToF detects it)
            if ($script:demoHasObs -and -not $script:demoObsAdded) {
                $pPend = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(160, 248, 113, 113), 1.4)
                $pPend.DashStyle = [System.Drawing.Drawing2D.DashStyle]::Dash
                $pxO = & $wx ($script:demoObsTarget.X - 5.0)
                $pyO = & $wy ($script:demoObsTarget.Y + 5.0)
                $g.DrawRectangle($pPend, $pxO, $pyO, $boxSz, $boxSz)
                $pPend.Dispose()
            }

            # Detected Dynamic Obstacles
            $bDyn = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(195, 239, 68, 68))
            $pDyn = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(254, 202, 202), 1.6)
            $bDynTxt = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::White)
            foreach ($d in $script:dynObs) {
                $dx = & $wx ($d.X - 5.0)
                $dy = & $wy ($d.Y + 5.0)
                $g.FillRectangle($bDyn, $dx, $dy, $boxSz, $boxSz)
                $g.DrawRectangle($pDyn, $dx, $dy, $boxSz, $boxSz)
                $g.DrawString("DYN", $fObs, $bDynTxt, ($dx + 1.0), ($dy + 3.0))
            }
            $bDyn.Dispose(); $pDyn.Dispose(); $bDynTxt.Dispose(); $fObs.Dispose(); $fZone.Dispose()

            # Planned A* route preview (subtle dashed emerald line)
            if ($null -ne $script:plannedPath -and $script:plannedPath.Count -ge 2) {
                $rPts = New-Object 'System.Drawing.PointF[]' $script:plannedPath.Count
                for ($i = 0; $i -lt $script:plannedPath.Count; $i++) {
                    $wp = $script:plannedPath[$i]
                    $rPts[$i] = New-Object System.Drawing.PointF((& $wx $wp[0]), (& $wy $wp[1]))
                }
                $pPlan = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(150, 52, 211, 153), 1.5)
                $pPlan.DashStyle = [System.Drawing.Drawing2D.DashStyle]::Dash
                $g.DrawLines($pPlan, $rPts)
                $pPlan.Dispose()
            }

            # Trajectory polyline
            if ($script:trail.Count -ge 2) {
                $pts = New-Object 'System.Drawing.PointF[]' $script:trail.Count
                for ($i = 0; $i -lt $script:trail.Count; $i++) {
                    $pt = $script:trail[$i]
                    $pts[$i] = New-Object System.Drawing.PointF((& $wx $pt.X), (& $wy $pt.Y))
                }
                $pTrail = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(56, 189, 248), 2.2)
                $g.DrawLines($pTrail, $pts)
                $pTrail.Dispose()
            }

            # Breadcrumb dots
            $bDot = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(250, 204, 21))
            foreach ($bc in $script:breadcrumbs) {
                $px = & $wx $bc.X
                $py = & $wy $bc.Y
                $g.FillEllipse($bDot, ($px - 2.0), ($py - 2.0), 4.0, 4.0)
            }
            $bDot.Dispose()

            # 3x VL53L0X laser rays + Robot footprint (11.46 x 10.0 cm)
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
                    $pRay = New-Object System.Drawing.Pen($col, 1.4)
                    $g.DrawLine($pRay, (& $wx $sx), (& $wy $sy), (& $wx $ex), (& $wy $ey))
                    $pRay.Dispose()
                }
            }

            $c = [math]::Cos($th); $s = [math]::Sin($th)
            $corners = @(@(5.0, 5.73), @(5.0, -5.73), @(-5.0, -5.73), @(-5.0, 5.73))
            $robPts = New-Object 'System.Drawing.PointF[]' 4
            for ($i = 0; $i -lt 4; $i++) {
                $lx = $corners[$i][0]; $ly = $corners[$i][1]
                $pxW = $rx + $lx * $c - $ly * $s
                $pyW = $ry + $lx * $s + $ly * $c
                $robPts[$i] = New-Object System.Drawing.PointF((& $wx $pxW), (& $wy $pyW))
            }
            $bRob = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(220, 14, 165, 233))
            $pRob = New-Object System.Drawing.Pen([System.Drawing.Color]::White, 1.6)
            $g.FillPolygon($bRob, $robPts)
            $g.DrawPolygon($pRob, $robPts)
            $bRob.Dispose(); $pRob.Dispose()

            $ax = $rx + 8.0 * $c; $ay = $ry + 8.0 * $s
            $pHead = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(250, 204, 21), 2.2)
            $g.DrawLine($pHead, (& $wx $rx), (& $wy $ry), (& $wx $ax), (& $wy $ay))
            $pHead.Dispose()
        } finally {
            $g.Dispose()
        }
        $script:UI.PicMap.Invalidate()
    }

    $script:Fn.PushTrailPoint = {
        param([double]$x, [double]$y)
        $thRad = [double]$script:liveState.Heading * [math]::PI / 180.0
        $dXs = [double[]]@(); $dYs = [double[]]@()
        if ($script:dynObs.Count -gt 0) {
            $dXs = [double[]]($script:dynObs | ForEach-Object { [double]$_.X })
            $dYs = [double[]]($script:dynObs | ForEach-Object { [double]$_.Y })
        }
        $clr = [CheesyAStar]::ComputeClearances($x, $y, $thRad, $dXs, $dYs)
        if ($clr[0] -lt $script:liveState.MinCenterClr)  { $script:liveState.MinCenterClr  = $clr[0] }
        if ($clr[1] -lt $script:liveState.MinChassisClr) { $script:liveState.MinChassisClr = $clr[1] }

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

    $script:Fn.HandleIncomingLine = {
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
                & $script:Fn.PushTrailPoint $script:liveState.X $script:liveState.Y
                & $script:Fn.UpdateTelLabels
                & $script:Fn.DrawArenaMap
            }
        } elseif ($line.StartsWith('$EVT,FSM,')) {
            $f = $line.Split(',')
            if ($f.Length -ge 4) {
                $script:liveState.FsmState = [int]$f[2]
                $script:liveState.Score    = [int]$f[3]
                & $script:Fn.UpdateTelLabels
                & $script:Fn.WriteLog "FSM -> State=$($f[2]), Score=$($f[3])"
            }
        } elseif ($line.StartsWith('$EVT,OBS,')) {
            $f = $line.Split(',')
            if ($f.Length -ge 4) {
                $ox = [double]::Parse($f[2], $inv)
                $oy = [double]::Parse($f[3], $inv)
                $script:dynObs.Add([PSCustomObject]@{ X=$ox; Y=$oy })
                & $script:Fn.DrawArenaMap
                & $script:Fn.WriteLog "Phát hiện vật cản động tại ($ox, $oy) cm!"
            }
        } elseif ($line.StartsWith('$EVT,')) {
            & $script:Fn.WriteLog $line
        }
    }

    $script:uiTimer = New-Object System.Windows.Forms.Timer
    $script:uiTimer.Interval = 50

    $script:Fn.SendAndWaitAck = {
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
                        & $script:Fn.HandleIncomingLine $ln
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

    $script:Fn.FetchBleCfg = {
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
                            & $script:Fn.HandleIncomingLine $ln
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

    $script:Fn.RefreshComList = {
        $script:UI.CmbCom.Items.Clear()
        $ports = [System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object
        $pnpMap = @{}
        try {
            Get-CimInstance Win32_PnPEntity -Filter "Name LIKE '%(COM%'" -ErrorAction SilentlyContinue | ForEach-Object {
                if ($_.Name -match '\((COM\d+)\)') { $pnpMap[$matches[1]] = $_.Name }
            }
        } catch {}
        foreach ($p in $ports) {
            $desc = if ($pnpMap.ContainsKey($p)) { $pnpMap[$p] } else { $p }
            [void]$script:UI.CmbCom.Items.Add("$p - $desc")
        }
        if ($script:UI.CmbCom.Items.Count -gt 0) {
            $script:UI.CmbCom.SelectedIndex = 0
        } else {
            [void]$script:UI.CmbCom.Items.Add("(Chưa thấy cổng COM Bluetooth)")
            $script:UI.CmbCom.SelectedIndex = 0
        }
    }

    $script:Fn.SyncValuesToUi = {
        param([System.Collections.IDictionary]$vals)
        foreach ($spec in $script:ParamSpecs) {
            if ($vals.Contains($spec.Key)) {
                $num = $script:numControls[$spec.Key]
                $v = [decimal]$vals[$spec.Key]
                if ($v -lt $num.Minimum) { $v = $num.Minimum }
                if ($v -gt $num.Maximum) { $v = $num.Maximum }
                $num.Value = $v
            }
        }
    }

    $script:Fn.StartDemoPlan = {
        $script:trail.Clear()
        $script:breadcrumbs.Clear()
        $script:dynObs.Clear()
        [CheesyAStar]::ResetCSpace()
        $script:plannedPath = [CheesyAStar]::PlanPathToFinish(30.0, -6.0, $false)
        $script:demoObsTarget = Choose-DemoO4Obstacle $script:plannedPath
        $script:demoWpIdx = 1
        $script:demoObsAdded = $false
        $script:liveState.X = 30.0
        $script:liveState.Y = -6.0
        $script:liveState.Heading = 90.0
        $script:liveState.FsmState = 1
        $script:liveState.Score = 0
        $script:liveState.MinCenterClr = 999.0
        $script:liveState.MinChassisClr = 999.0
        $script:liveState.Dl = 8190; $script:liveState.Df = 8190; $script:liveState.Dr = 8190
        & $script:Fn.PushTrailPoint 30.0 -6.0
        & $script:Fn.UpdateTelLabels
        & $script:Fn.DrawArenaMap
    }

    $script:Fn.StepDemoTick = {
        if (-not $script:demoActive) { return }
        if ($null -eq $script:plannedPath -or $script:demoWpIdx -ge $script:plannedPath.Count) {
            $script:demoActive = $false
            $script:liveState.FsmState = 4
            $script:liveState.Score    = if ($script:demoObsAdded) { 272 } else { 257 }
            $script:liveState.VlExp = 0; $script:liveState.VrExp = 0
            $script:liveState.VlReal = 0; $script:liveState.VrReal = 0
            $script:UI.BtnDemo.Text = "Mô phỏng Demo"
            $script:UI.BtnDemo.BackColor = $script:Theme.Blue
            & $script:Fn.UpdateTelLabels
            & $script:Fn.DrawArenaMap
            & $script:Fn.WriteLog ("Demo hoàn thành Ô 5! (Khoảng hở min: Tâm={0:F2} cm, Thân={1:F2} cm)" -f $script:liveState.MinCenterClr, $script:liveState.MinChassisClr)
            return
        }

        $tx = [double]$script:plannedPath[$script:demoWpIdx][0]
        $ty = [double]$script:plannedPath[$script:demoWpIdx][1]
        $rx = [double]$script:liveState.X
        $ry = [double]$script:liveState.Y
        $dx = $tx - $rx; $dy = $ty - $ry
        $dist = [math]::Sqrt($dx * $dx + $dy * $dy)

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
        $angRad = if ($dist -gt 1e-4) { [math]::Atan2($dy, $dx) } else { [double]$script:liveState.Heading * [math]::PI / 180.0 }
        if ($dist -le $stepCm) {
            $rx = $tx; $ry = $ty
            $script:demoWpIdx++
        } else {
            $rx += $stepCm * [math]::Cos($angRad)
            $ry += $stepCm * [math]::Sin($angRad)
        }

        $scanX = [double]$script:numControls["SCANX"].Value
        $tofMaxMm = [double]$script:numControls["TOF"].Value
        $tofMaxCm = $tofMaxMm * 0.1
        $obsX = [double]$script:demoObsTarget.X
        $obsY = [double]$script:demoObsTarget.Y

        $dl = Raycast-DemoToF $rx $ry ($angRad + 0.7854) 4.5 $script:demoHasObs $obsX $obsY
        $df = Raycast-DemoToF $rx $ry $angRad 5.0 $script:demoHasObs $obsX $obsY
        $dr = Raycast-DemoToF $rx $ry ($angRad - 0.7854) 4.5 $script:demoHasObs $obsX $obsY

        $script:liveState.X       = $rx
        $script:liveState.Y       = $ry
        $script:liveState.Heading = $angRad * 180.0 / [math]::PI
        $script:liveState.Vbat    = 7.98
        $script:liveState.FsmState = if ($rx -ge $scanX -and $ry -ge 55.0) { 2 } else { 1 }
        $script:liveState.Dl = $dl
        $script:liveState.Df = $df
        $script:liveState.Dr = $dr
        $script:liveState.VlExp  = $vDemo
        $script:liveState.VrExp  = $vDemo
        $script:liveState.VlReal = $vDemo
        $script:liveState.VrReal = $vDemo

        # Live ToF obstacle detection + real A* replanning when in scan zone
        if ($script:demoHasObs -and -not $script:demoObsAdded -and $rx -ge $scanX -and $ry -ge 55.0) {
            $dToObs = [math]::Sqrt(($rx - $obsX)*($rx - $obsX) + ($ry - $obsY)*($ry - $obsY))
            if ($dToObs -le [math]::Min($tofMaxCm + 6.0, 38.0) -and $dToObs -ge 13.0) {
                $script:demoObsAdded = $true
                $script:dynObs.Add([PSCustomObject]@{ X=$obsX; Y=$obsY })
                [CheesyAStar]::AddDynamicObstacleBox([float]$obsX, [float]$obsY, 5.0, 6.75)
                $replan = [CheesyAStar]::PlanPathToFinish([float]$rx, [float]$ry, $true)
                if ($replan.Count -ge 2) {
                    $script:plannedPath = $replan
                    $script:demoWpIdx = 1
                    & $script:Fn.WriteLog ("3× ToF phát hiện cọc động ({0:F0},{1:F0}) cm -> A* tìm đường tránh ({2} nút)!" -f $obsX, $obsY, $replan.Count)
                }
            }
        }

        & $script:Fn.PushTrailPoint $rx $ry
        & $script:Fn.UpdateTelLabels
        & $script:Fn.DrawArenaMap
    }

    $script:uiTimer.Add_Tick({
        if ($null -ne $script:serialPort -and $script:serialPort.IsOpen) {
            try {
                $chunk = $script:serialPort.ReadExisting()
                if ($chunk.Length -gt 0) {
                    $script:rxBuffer += $chunk
                    while ($script:rxBuffer.Contains("`n")) {
                        $idx = $script:rxBuffer.IndexOf("`n")
                        $ln  = $script:rxBuffer.Substring(0, $idx).Trim()
                        $script:rxBuffer = $script:rxBuffer.Substring($idx + 1)
                        if ($ln.Length -gt 0) { & $script:Fn.HandleIncomingLine $ln }
                    }
                }
            } catch {}
            return
        }
        if ($script:demoActive) {
            & $script:Fn.StepDemoTick
        }
    })

    # --- Button Handlers (100% $script:UI / $script:Theme / $script:Fn) ---
    $script:UI.BtnDemo.Add_Click({
        if ($null -ne $script:serialPort -and $script:serialPort.IsOpen) {
            [System.Windows.Forms.MessageBox]::Show("Đang kết nối Bluetooth thật — hãy ngắt BLE trước khi chạy Demo.", "Thông báo") | Out-Null
            return
        }
        $script:demoActive = -not $script:demoActive
        if ($script:demoActive) {
            & $script:Fn.StartDemoPlan
            $script:UI.BtnDemo.Text = "Dừng Demo"
            $script:UI.BtnDemo.BackColor = $script:Theme.Red
            & $script:Fn.WriteLog "Bắt đầu mô phỏng Demo theo A* C-Space & vận tốc V1..V5 bạn chọn!"
        } else {
            $script:UI.BtnDemo.Text = "Mô phỏng Demo"
            $script:UI.BtnDemo.BackColor = $script:Theme.Blue
        }
    })

    $script:UI.BtnDemoObs.Add_Click({
        $script:demoHasObs = -not $script:demoHasObs
        $script:UI.BtnDemoObs.Text = if ($script:demoHasObs) { "Cọc Demo Ô 4: BẬT" } else { "Cọc Demo Ô 4: TẮT" }
        $script:UI.BtnDemoObs.BackColor = if ($script:demoHasObs) { $script:Theme.Amber } else { $script:Theme.Slate }
        if (-not $script:demoActive) { & $script:Fn.DrawArenaMap }
    })

    $script:UI.BtnClearMap.Add_Click({
        $script:demoActive = $false
        $script:UI.BtnDemo.Text = "Mô phỏng Demo"
        $script:UI.BtnDemo.BackColor = $script:Theme.Blue
        $script:trail.Clear(); $script:breadcrumbs.Clear(); $script:dynObs.Clear()
        $script:demoObsAdded = $false
        [CheesyAStar]::ResetCSpace()
        $script:plannedPath = [CheesyAStar]::PlanPathToFinish(30.0, -6.0, $false)
        $script:demoObsTarget = Choose-DemoO4Obstacle $script:plannedPath
        $script:liveState.X = 30.0; $script:liveState.Y = -6.0; $script:liveState.Heading = 90.0
        $script:liveState.FsmState = 0; $script:liveState.Score = 0
        $script:liveState.Dl = 8190; $script:liveState.Df = 8190; $script:liveState.Dr = 8190
        $script:liveState.MinCenterClr = 999.0; $script:liveState.MinChassisClr = 999.0
        & $script:Fn.UpdateTelLabels
        & $script:Fn.DrawArenaMap
        & $script:Fn.WriteLog "Đã xóa quỹ đạo & đặt lại robot tại gốc (30, -6)."
    })

    $script:UI.BtnExportCsv.Add_Click({
        if ($script:trail.Count -eq 0) {
            & $script:Fn.WriteLog "Chưa có điểm quỹ đạo nào để xuất CSV."
            return
        }
        if (-not (Test-Path $script:LogsDir)) { [void](New-Item -ItemType Directory -Path $script:LogsDir) }
        $csvPath = Join-Path $script:LogsDir ("trajectory_" + (Get-Date).ToString("yyyyMMdd_HHmmss") + ".csv")
        $sb = New-Object System.Text.StringBuilder
        [void]$sb.AppendLine("index,x_cm,y_cm")
        $inv = [System.Globalization.CultureInfo]::InvariantCulture
        for ($i = 0; $i -lt $script:trail.Count; $i++) {
            $p = $script:trail[$i]
            [void]$sb.AppendLine("$i,$($p.X.ToString('0.00', $inv)),$($p.Y.ToString('0.00', $inv))")
        }
        [IO.File]::WriteAllText($csvPath, $sb.ToString(), (New-Object System.Text.UTF8Encoding($false)))
        & $script:Fn.WriteLog "Đã xuất $($script:trail.Count) điểm ra: $csvPath"
    })

    $script:UI.BtnScan.Add_Click({
        & $script:Fn.RefreshComList
        & $script:Fn.WriteLog "Đã quét danh sách cổng COM."
    })

    $script:UI.BtnApplyAll.Add_Click({
        $v = $script:UI.NumAll.Value
        foreach ($k in @("V1","V2","V3","V4","V5")) { $script:numControls[$k].Value = $v }
        & $script:Fn.WriteLog "Đã đặt V1..V5 = $v cm/s trên bảng."
    })

    $script:UI.BtnConn.Add_Click({
        if ($null -ne $script:serialPort -and $script:serialPort.IsOpen) {
            $script:serialPort.Close(); $script:serialPort.Dispose(); $script:serialPort = $null
            $script:UI.BtnConn.Text = "Kết nối BLE"
            $script:UI.BtnConn.BackColor = $script:Theme.Blue
            $script:UI.BtnRead.Enabled = $false
            $script:UI.LblStatus.Text = "Trạng thái: Đã ngắt kết nối (Offline)"
            & $script:Fn.WriteLog "Đã ngắt kết nối Bluetooth."
            return
        }
        $sel = [string]$script:UI.CmbCom.SelectedItem
        if (-not ($sel -match '^(COM\d+)')) {
            [System.Windows.Forms.MessageBox]::Show("Chưa chọn cổng COM hợp lệ! Hãy ghép đôi HC-05 trong Windows trước.", "Thông báo") | Out-Null
            return
        }
        $comName = $matches[1]
        try {
            $script:demoActive = $false
            $script:UI.BtnDemo.Text = "Mô phỏng Demo"
            $script:UI.BtnDemo.BackColor = $script:Theme.Blue
            $p = New-Object System.IO.Ports.SerialPort($comName, 9600, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
            $p.NewLine = "`r`n"; $p.ReadTimeout = 100; $p.WriteTimeout = 1000
            $p.Open()
            Start-Sleep -Milliseconds 150
            $script:serialPort = $p
            $script:UI.BtnConn.Text = "Ngắt BLE"
            $script:UI.BtnConn.BackColor = $script:Theme.Red
            $script:UI.BtnRead.Enabled = $true
            $script:UI.LblStatus.Text = "Trạng thái: Đã kết nối $comName @ 9600 (đang vẽ quỹ đạo trực tiếp)"
            & $script:Fn.WriteLog "Đã mở $comName, đang đọc thông số từ xe..."

            $liveVals = [ordered]@{}
            if (& $script:Fn.FetchBleCfg $liveVals) {
                & $script:Fn.SyncValuesToUi $liveVals
                & $script:Fn.WriteLog "Đã đồng bộ đủ 17 thông số từ STM32!"
            } else {
                & $script:Fn.WriteLog "Cảnh báo: Chưa nhận được `$CFG từ xe (kiểm tra nguồn xe / TX-RX HC-05)."
            }
        } catch {
            if ($null -ne $script:serialPort) {
                if ($script:serialPort.IsOpen) { $script:serialPort.Close() }
                $script:serialPort.Dispose(); $script:serialPort = $null
            }
            & $script:Fn.WriteLog "Lỗi mở ${comName}: $($_.Exception.Message)"
        }
    })

    $script:UI.BtnRead.Add_Click({
        $liveVals = [ordered]@{}
        if (& $script:Fn.FetchBleCfg $liveVals) {
            & $script:Fn.SyncValuesToUi $liveVals
            & $script:Fn.WriteLog "Đã đọc lại 17 thông số từ xe."
        } else {
            & $script:Fn.WriteLog "Lỗi: Không nhận được `$CFG từ xe."
        }
    })

    $script:UI.BtnSendLive.Add_Click({
        if ($null -eq $script:serialPort -or -not $script:serialPort.IsOpen) {
            [System.Windows.Forms.MessageBox]::Show("Hãy kết nối Bluetooth HC-05 trước, hoặc dùng 'Lưu Code (.h)'.", "Chưa kết nối BLE") | Out-Null
            return
        }
        $okCnt = 0
        foreach ($spec in $script:ParamSpecs) {
            $vStr = Format-CfgValue ([double]$script:numControls[$spec.Key].Value) $spec.Type
            $res = & $script:Fn.SendAndWaitAck "CFG:$($spec.Key),$vStr" "`$ACK,CFG,$($spec.Key)" 1200
            if ($res.Ok) { $okCnt++ } else { & $script:Fn.WriteLog "Lỗi $($spec.Key)=$vStr -> $($res.Reply)" }
        }
        & $script:Fn.WriteLog "Đã gửi LIVE $okCnt/$($script:ParamSpecs.Count) thông số! Bấm 'Lưu Flash' nếu muốn giữ sau khi tắt nguồn."
    })

    $script:UI.BtnSaveFlash.Add_Click({
        if ($null -eq $script:serialPort -or -not $script:serialPort.IsOpen) {
            [System.Windows.Forms.MessageBox]::Show("Hãy kết nối Bluetooth HC-05 trước khi lưu Flash.", "Chưa kết nối BLE") | Out-Null
            return
        }
        $res = & $script:Fn.SendAndWaitAck "CFG:SAVE" '$ACK,CFG,SAVE' 2500
        if ($res.Ok) { & $script:Fn.WriteLog "OK: Đã lưu vĩnh viễn vào Flash Sector 7 trên STM32!" }
        else { & $script:Fn.WriteLog "Lỗi CFG:SAVE ($($res.Reply)) — Bấm DỪNG KHẨN rồi thử lại." }
    })

    $script:UI.BtnSaveCode.Add_Click({
        try {
            foreach ($spec in $script:ParamSpecs) { Write-CodeParam $spec ([double]$script:numControls[$spec.Key].Value) }
            & $script:Fn.WriteLog "OK: Đã ghi 17 thông số vào Core/Inc/robot_config.h & pure_pursuit.h!"
        } catch {
            & $script:Fn.WriteLog "Lỗi ghi file code: $($_.Exception.Message)"
        }
    })

    $script:UI.BtnResetDef.Add_Click({
        if ($null -ne $script:serialPort -and $script:serialPort.IsOpen) {
            $res = & $script:Fn.SendAndWaitAck "CFG:RESET" '$ACK,CFG,RESET' 1500
            if ($res.Ok) {
                $liveVals = [ordered]@{}
                [void](& $script:Fn.FetchBleCfg $liveVals)
                & $script:Fn.SyncValuesToUi $liveVals
                & $script:Fn.WriteLog "Đã khôi phục mặc định trên xe."
            } else { & $script:Fn.WriteLog "Lỗi CFG:RESET: $($res.Reply)" }
        } else {
            & $script:Fn.SyncValuesToUi (Read-CodeDefaults)
            & $script:Fn.WriteLog "Đã tải lại giá trị từ file code."
        }
    })

    $script:UI.BtnRunAuto.Add_Click({
        if ($null -eq $script:serialPort -or -not $script:serialPort.IsOpen) { return }
        $script:trail.Clear(); $script:breadcrumbs.Clear(); $script:dynObs.Clear()
        $res = & $script:Fn.SendAndWaitAck "RUN:2" '$ACK,RUN,2'
        & $script:Fn.WriteLog "RUN:2 -> $($res.Reply)"
    })

    $script:UI.BtnStop.Add_Click({
        if ($null -eq $script:serialPort -or -not $script:serialPort.IsOpen) { return }
        $res = & $script:Fn.SendAndWaitAck "STOP" '$ACK,STOP'
        & $script:Fn.WriteLog "STOP -> $($res.Reply)"
    })

    $script:UI.BtnZero.Add_Click({
        if ($null -eq $script:serialPort -or -not $script:serialPort.IsOpen) { return }
        $script:trail.Clear(); $script:breadcrumbs.Clear(); $script:dynObs.Clear()
        $res = & $script:Fn.SendAndWaitAck "ZERO" '$ACK,ZERO'
        & $script:Fn.DrawArenaMap
        & $script:Fn.WriteLog "ZERO -> $($res.Reply)"
    })

    $script:UI.BtnFlashStlink.Add_Click({
        & $script:Fn.WriteLog "Đang mở NAP_CODE.bat để build & nạp ST-Link..."
        Start-Process cmd.exe -ArgumentList "/c `"`"$($script:NapBatPath)`"`"" -WorkingDirectory $script:RootDir
    })

    $form.Add_FormClosed({
        $script:uiTimer.Stop(); $script:uiTimer.Dispose()
        if ($null -ne $script:serialPort) {
            if ($script:serialPort.IsOpen) { $script:serialPort.Close() }
            $script:serialPort.Dispose(); $script:serialPort = $null
        }
        if ($null -ne $script:mapBmp) { $script:mapBmp.Dispose() }
    })

    & $script:Fn.RefreshComList
    & $script:Fn.UpdateTelLabels
    & $script:Fn.DrawArenaMap
    & $script:Fn.WriteLog "Sẵn sàng! Bấm 'Mô phỏng Demo' để xem quỹ đạo A* thực tế, hoặc kết nối HC-05 xem xe chạy thật."
    $script:uiTimer.Start()
    return $form
}

$defaults = Read-CodeDefaults

if ($Arg1 -eq "--selftest") {
    $form = Build-DashboardForm $defaults
    $form.StartPosition = [System.Windows.Forms.FormStartPosition]::Manual
    $form.Location = New-Object System.Drawing.Point(-3000, -3000)
    $form.ShowInTaskbar = $false
    $form.Show()
    [System.Windows.Forms.Application]::DoEvents()

    # 1. Bounds audit across all controls & realized tab pages
    function Check-Bounds([System.Windows.Forms.Control]$parent) {
        $cw = $parent.ClientSize.Width; $ch = $parent.ClientSize.Height
        foreach ($c in $parent.Controls) {
            if ($c.Left -lt 0 -or $c.Top -lt 0 -or $c.Right -gt $cw -or $c.Bottom -gt $ch) {
                throw "Clipping control '$($c.Text)' ($($c.Left),$($c.Top),$($c.Right),$($c.Bottom)) inside '$($parent.GetType().Name):$($parent.Text)' (${cw}x${ch})"
            }
            if ($c -is [System.Windows.Forms.TabControl]) {
                foreach ($tp in $c.TabPages) {
                    $c.SelectedTab = $tp
                    [System.Windows.Forms.Application]::DoEvents()
                    Check-Bounds $tp
                }
                $c.SelectedIndex = 0
            } elseif ($c.Controls.Count -gt 0) {
                Check-Bounds $c
            }
        }
    }
    Check-Bounds $form

    # 2. Button click audit (catch any scoping null errors)
    $script:UI.BtnDemoObs.PerformClick()
    $script:UI.BtnDemoObs.PerformClick()
    $script:UI.BtnApplyAll.PerformClick()
    $script:UI.BtnScan.PerformClick()
    $script:UI.BtnClearMap.PerformClick()
    $script:UI.BtnResetDef.PerformClick()
    $script:UI.BtnDemo.PerformClick()
    $script:UI.BtnDemo.PerformClick()

    # 3. Run A: Clean A* Demo to finish
    $script:demoHasObs = $false
    $script:demoActive = $true
    & $script:Fn.StartDemoPlan
    for ($t = 0; $t -lt 900 -and $script:demoActive; $t++) { & $script:Fn.StepDemoTick }
    if ($script:liveState.FsmState -ne 4) { throw "Clean Demo did not reach finish! Fsm=$($script:liveState.FsmState) X=$($script:liveState.X) Y=$($script:liveState.Y)" }
    $cleanSummary = ("X={0:F1},Y={1:F1},CenMin={2:F2},ChaMin={3:F2}" -f $script:liveState.X, $script:liveState.Y, $script:liveState.MinCenterClr, $script:liveState.MinChassisClr)
    if ($script:liveState.MinCenterClr -lt 5.75 -or $script:liveState.MinChassisClr -lt 0.30) {
        throw "Clean Demo collision/clearance violation: $cleanSummary"
    }

    # 4. Run B: Dynamic Obstacle in O4 + ToF replanning Demo to finish
    $script:UI.BtnDemoObs.PerformClick()
    if (-not $script:demoHasObs) { throw "BtnDemoObs failed to toggle demoHasObs" }
    $script:demoActive = $true
    & $script:Fn.StartDemoPlan
    for ($t = 0; $t -lt 900 -and $script:demoActive; $t++) { & $script:Fn.StepDemoTick }
    if ($script:liveState.FsmState -ne 4 -or -not $script:demoObsAdded) {
        throw "DynObs Demo failed! Fsm=$($script:liveState.FsmState) ObsAdded=$($script:demoObsAdded) X=$($script:liveState.X) Y=$($script:liveState.Y)"
    }
    $dynSummary = ("Obs=({0:F0},{1:F0}),X={2:F1},Y={3:F1},CenMin={4:F2},ChaMin={5:F2}" -f $script:demoObsTarget.X, $script:demoObsTarget.Y, $script:liveState.X, $script:liveState.Y, $script:liveState.MinCenterClr, $script:liveState.MinChassisClr)
    if ($script:liveState.MinCenterClr -lt 5.75 -or $script:liveState.MinChassisClr -lt 0.30) {
        throw "DynObs Demo collision/clearance violation: $dynSummary"
    }

    $cnt = $script:numControls.Count
    $form.Dispose()
    Write-Host "SELFTEST_OK Params=$cnt BoundsOK=True ClicksOK=True CleanRun=[$cleanSummary] DynObsRun=[$dynSummary]"
    exit 0
}

Ensure-Shortcuts
$mainForm = Build-DashboardForm $defaults
[void]$mainForm.ShowDialog()
