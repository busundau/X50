param(
    [Parameter(Mandatory=$true)]
    [string]$HexFile
)

function Get-Crc32([string]$path) {
    $table = @()
    for ($i = 0; $i -lt 256; $i++) {
        $crc = $i
        for ($j = 0; $j -lt 8; $j++) {
            if ($crc -band 1) { $crc = (0xEDB88320 -bxor ($crc -shr 1)) }
            else { $crc = ($crc -shr 1) }
        }
        $table += $crc
    }
    $crc32 = 0xffffffff
    foreach ($b in [System.IO.File]::ReadAllBytes($path)) {
        $crc32 = ($crc32 -shr 8) -bxor $table[($crc32 -bxor $b) -band 0xff]
    }
    return "{0:X8}" -f (-bnot $crc32)
}

$fullpath = [System.IO.Path]::GetFullPath($HexFile)

if (-Not (Test-Path -LiteralPath $fullpath)) {
    Write-Output "[Keil Build] HEX file not found: $fullpath"
    exit 1
}

$crc = Get-Crc32 $fullpath
Write-Output "[Keil Build] CRC32 = 0x$crc"

# === 新檔名處理 ===
$dir     = Split-Path $fullpath -Parent
$name    = [System.IO.Path]::GetFileNameWithoutExtension($fullpath)
$ext     = [System.IO.Path]::GetExtension($fullpath)

# 確保 Output2 資料夾存在
$outdir  = Join-Path $dir "..\Output2"
if (-Not (Test-Path $outdir)) {
    New-Item -ItemType Directory -Path $outdir | Out-Null
}

# 產生時間戳 (yyyyMMdd-HHmmss)
$timestamp = Get-Date -Format "yyyyMMdd-HHmmss"

# === 檔名順序：原名 + 時間戳 + CRC ===
$newfile = Join-Path $outdir ($name + "_" + $timestamp + "_crc" + $crc + $ext)

Copy-Item $fullpath $newfile -Force
Write-Output "[Keil Build] Copied to: $newfile"