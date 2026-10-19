# ============================================================================
# gen_cjk16.ps1 -- generate 16x16 CJK dot-matrix bitmaps (SSD1306 page format)
#
# Usage : powershell -ExecutionPolicy Bypass -File gen_cjk16.ps1
#         optional: -Only 0x51B2,0x51FB  (render just these code points)
# Output: C array text + per-glyph ASCII art for visual check
#
# NOTE  : This file must stay ASCII-only. Windows PowerShell 5.1 parses
#         BOM-less files as ANSI/GBK, so non-ASCII comments corrupt parsing.
#         Target characters are given as Unicode code points instead.
#
# Glyphs are rendered from the local SimSun (simsun.ttc) with grid-fit
# hinting, then ink-centred in the 16x16 cell.
#
# Target set (33 chars):
#   base 14 (title + states):
#     U+8DCC U+5012 U+68C0 U+6D4B U+57FA U+7AD9 U+6B63 U+5E38
#     U+884C U+8D70 U+7591 U+4F3C U+544A U+8B66
#   ui pages 19 (impact / telemetry / settings / demo pages):
#     U+51B2 U+51FB U+8BBE U+7F6E U+6F14 U+793A U+6A21 U+5F0F
#     U+573A U+666F U+9759 U+6B62 U+706F U+81EA U+5267 U+672C
#     U+9065 U+7CFB U+7EDF
# ============================================================================

param(
    [string]$Only = "",
    [switch]$Big = $false
)

$script:UseBig = [bool]$Big

$Cps = @(0x8DCC, 0x5012, 0x68C0, 0x6D4B, 0x57FA, 0x7AD9, 0x6B63, 0x5E38,
         0x884C, 0x8D70, 0x7591, 0x4F3C, 0x544A, 0x8B66,
         0x51B2, 0x51FB, 0x8BBE, 0x7F6E, 0x6F14, 0x793A, 0x6A21, 0x5F0F,
         0x573A, 0x666F, 0x9759, 0x6B62, 0x706F, 0x81EA, 0x5267, 0x672C,
         0x9065, 0x7CFB, 0x7EDF)

if ($Only -ne "") {
    $Cps = $Only.Split(',') | ForEach-Object { [Convert]::ToInt64($_.Trim(), 16) }
}

Add-Type -AssemblyName System.Drawing

function Get-InkBox($bmp) {
    $minX = 99; $maxX = -1; $minY = 99; $maxY = -1
    for ($y = 0; $y -lt 16; $y++) {
        for ($x = 0; $x -lt 16; $x++) {
            if ($bmp.GetPixel($x, $y).R -gt 127) {
                if ($x -lt $minX) { $minX = $x }
                if ($x -gt $maxX) { $maxX = $x }
                if ($y -lt $minY) { $minY = $y }
                if ($y -gt $maxY) { $maxY = $y }
            }
        }
    }
    return @{ minX = $minX; maxX = $maxX; minY = $minY; maxY = $maxY; empty = ($maxX -lt 0) }
}

function Render-At([int]$cp, [int]$emSize, [int]$dx, [int]$dy) {
    $bmp = New-Object System.Drawing.Bitmap 16, 16
    $g = [System.Drawing.Graphics]::FromImage($bmp)
    $g.Clear([System.Drawing.Color]::Black)
    $g.TextRenderingHint = [System.Drawing.Text.TextRenderingHint]::SingleBitPerPixelGridFit
    $font = New-Object System.Drawing.Font('SimSun', $emSize, [System.Drawing.FontStyle]::Regular, [System.Drawing.GraphicsUnit]::Pixel)
    $sf = New-Object System.Drawing.StringFormat
    $sf.Alignment = [System.Drawing.StringAlignment]::Near
    $sf.LineAlignment = [System.Drawing.StringAlignment]::Near
    $s = [string][char]$cp
    $g.DrawString($s, $font, [System.Drawing.Brushes]::White, [single]$dx, [single]$dy, $sf)
    $g.Dispose()
    $font.Dispose()
    return $bmp
}

function Render-Glyph([int]$cp) {
    # -Big：为数字/字母这类"矮字形"用更大字号渲染，使其墨迹填满 16 行，
    # 与中文 16x16 同高（否则数字只占上半 8 行，与中文混排时明显偏小）。
    $sizes = @(16, 15, 14, 13, 12)
    if ($script:UseBig) { $sizes = @(24, 23, 22, 21, 20, 19, 18) }
    foreach ($size in $sizes) {
        $probe = Render-At $cp $size 0 0
        $ink = Get-InkBox $probe
        if ($ink.empty) { $probe.Dispose(); continue }
        $w = $ink.maxX - $ink.minX + 1
        $h = $ink.maxY - $ink.minY + 1
        $dx = [int][math]::Floor((16 - $w) / 2) - $ink.minX
        $dy = [int][math]::Floor((16 - $h) / 2) - $ink.minY
        $probe.Dispose()

        $bmp = Render-At $cp $size $dx $dy
        $ink2 = Get-InkBox $bmp
        if (-not $ink2.empty) {
            if ($ink2.minX -ge 0 -and $ink2.maxX -le 15 -and
                $ink2.minY -ge 0 -and $ink2.maxY -le 15) {
                return @{ bmp = $bmp; size = $size
                          w = ($ink2.maxX - $ink2.minX + 1)
                          h = ($ink2.maxY - $ink2.minY + 1) }
            }
        }
        $bmp.Dispose()
    }
    throw ("cannot render U+{0:X4}" -f $cp)
}

Write-Host ("glyph count = {0}" -f $Cps.Count)

$glyphs = @()
foreach ($cp in $Cps) {
    $r = Render-Glyph $cp
    $glyphs += [pscustomobject]@{ cp = $cp; bmp = $r.bmp; size = $r.size; w = $r.w; h = $r.h }
}
Write-Host ("rendered = {0}" -f $glyphs.Count)

Write-Host ''
Write-Host '=== DOT MATRIX (visual check: every glyph must be readable) ==='
foreach ($g in $glyphs) {
    Write-Host ("--- U+{0:X4}  (em={1}px, ink {2}x{3})" -f $g.cp, $g.size, $g.w, $g.h)
    for ($y = 0; $y -lt 16; $y++) {
        $line = ''
        for ($x = 0; $x -lt 16; $x++) {
            if ($g.bmp.GetPixel($x, $y).R -gt 127) { $line += '#' } else { $line += '.' }
        }
        Write-Host ("    {0}" -f $line)
    }
}

Write-Host ''
Write-Host '=== C ARRAY (page format: 16 cols top page, then 16 cols bottom page) ==='
foreach ($g in $glyphs) {
    $bytes = @()
    for ($half = 0; $half -lt 2; $half++) {
        for ($x = 0; $x -lt 16; $x++) {
            $b = 0
            for ($r = 0; $r -lt 8; $r++) {
                $y = $half * 8 + $r
                if ($g.bmp.GetPixel($x, $y).R -gt 127) { $b = $b -bor (1 -shl $r) }
            }
            $bytes += $b
        }
    }
    $hex = ($bytes | ForEach-Object { '0x{0:X2}' -f $_ }) -join ', '
    Write-Host ("    {{ 0x{0:X4}, {{ {1} }} }}," -f $g.cp, $hex)
}

foreach ($g in $glyphs) { $g.bmp.Dispose() }
