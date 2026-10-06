# Общие функции установщиков (install_to_blender.ps1, install_native_to_blender.ps1): подключаются точкой.
#
# ЗАЧЕМ. Установка, которая сначала стирает старую копию, а потом копирует новую, оставляет Blender без аддона, если стирание
# оборвалось на файле, который держит чужой процесс (пул воркеров открытого Blender держит .pyc ядра): штамп установки исчез,
# аддон наполовину стёрт, а владелец узнаёт об этом по пустой панели. Здесь установка устроена как подмена каталога:
#
#   1. новая копия собирается и сверяется В СТОРОНЕ (скрытый каталог `scripts\.cftuv_staging`, Blender его не читает);
#   2. занятые другим процессом файлы названы ДО подмены (Get-LockedFiles);
#   3. старый каталог УХОДИТ ОДНИМ ПЕРЕИМЕНОВАНИЕМ (Directory.Move на том же томе: либо весь каталог, либо ничего), новый
#      встаёт на его место тем же переименованием; сбой на любом шаге возвращает всё как было (Undo-Install);
#   4. отложенные старые копии удаляются ПОСЛЕ успеха (Commit-Install), и их сбой не портит установку.
#
# Только то, что есть в Windows PowerShell 5.1 (см. тест `test_windows_scripts_stay_within_powershell_5_1`).

function Get-LockedFiles([string]$Root) {
    # Файлы каталога, которые нельзя открыть на запись с исключительным доступом: их держит другой процесс
    # (воркер пула, Blender с подгруженным .pyd). Пустой каталог и отсутствующий путь - пустой список.
    $locked = New-Object System.Collections.ArrayList
    if (-not (Test-Path -LiteralPath $Root)) { return ,$locked }
    foreach ($file in Get-ChildItem -LiteralPath $Root -Recurse -File -Force -ErrorAction SilentlyContinue) {
        try {
            $stream = [System.IO.File]::Open($file.FullName, [System.IO.FileMode]::Open, [System.IO.FileAccess]::ReadWrite, [System.IO.FileShare]::None)
            $stream.Close()
        } catch {
            [void]$locked.Add($file.FullName)
        }
    }
    return ,$locked
}

function Get-TreeFingerprint([string]$Root) {
    # sha256 по относительным путям и содержимому ВСЕХ файлов каталога, кроме __pycache__ (то, что ставят, а не то, что
    # Python накопил при запуске). Окончания строк не нормализуются: проверяется побайтовое равенство копии.
    $lines = New-Object System.Collections.ArrayList
    $base = (Resolve-Path -LiteralPath $Root).Path.TrimEnd('\')
    foreach ($file in Get-ChildItem -LiteralPath $Root -Recurse -File -Force) {
        if ($file.FullName -match '\\__pycache__\\') { continue }
        $relative = $file.FullName.Substring($base.Length + 1).Replace('\', '/')
        $hash = (Get-FileHash -LiteralPath $file.FullName -Algorithm SHA256).Hash
        [void]$lines.Add("$relative $hash")
    }
    $lines.Sort([System.StringComparer]::Ordinal)
    $text = [string]::Join("`n", $lines.ToArray())
    $sha = [System.Security.Cryptography.SHA256]::Create()
    $bytes = $sha.ComputeHash([System.Text.Encoding]::UTF8.GetBytes($text))
    return ([System.BitConverter]::ToString($bytes).Replace('-', '').ToLower().Substring(0, 16))
}

function Remove-Quietly([string]$Path) {
    # Удаление, чей сбой не ошибка: возвращает $true, если пути больше нет.
    if (-not (Test-Path -LiteralPath $Path)) { return $true }
    try {
        Remove-Item -LiteralPath $Path -Recurse -Force -ErrorAction Stop
        return $true
    } catch {
        return $false
    }
}

function Remove-Staging([string]$StagingRoot) {
    # Каталог стороны и, если он опустел, общий родитель `.cftuv_staging`; $true - стороны больше нет.
    $removed = Remove-Quietly $StagingRoot
    $parent = Split-Path -Parent $StagingRoot
    if ($removed -and (Test-Path -LiteralPath $parent) -and -not (Get-ChildItem -LiteralPath $parent -Force)) {
        Remove-Item -LiteralPath $parent -Force -ErrorAction SilentlyContinue
    }
    return $removed
}

function Install-Directories([object[]]$Pairs, [string]$Graveyard) {
    # Подмена каталогов. Каждая пара: @{ Staged = <новая копия>; Target = <куда встаёт> }; Staged = $null - только убрать
    # Target (устаревшая копия). Старые каталоги уходят в $Graveyard. Результат - запись для Undo-Install / Commit-Install.
    # Любой сбой возвращает уже сделанное и бросает исключение с именем каталога и причиной.
    $done = New-Object System.Collections.ArrayList
    try {
        foreach ($pair in $Pairs) {
            $entry = @{ Target = $pair.Target; Staged = $pair.Staged; Old = $null }
            if (Test-Path -LiteralPath $pair.Target) {
                New-Item -ItemType Directory -Force -Path $Graveyard | Out-Null
                $old = Join-Path $Graveyard ([guid]::NewGuid().ToString('N').Substring(0, 8) + '_' + (Split-Path -Leaf $pair.Target))
                try {
                    [System.IO.Directory]::Move($pair.Target, $old)
                } catch {
                    throw "cannot move the installed copy aside: $($pair.Target): $($_.Exception.Message)"
                }
                $entry.Old = $old
                [void]$done.Add($entry)
            }
            if ($pair.Staged) {
                try {
                    [System.IO.Directory]::Move($pair.Staged, $pair.Target)
                } catch {
                    throw "cannot put the new copy in place: $($pair.Target): $($_.Exception.Message)"
                }
                if ($entry.Old -eq $null) { [void]$done.Add($entry) }
            }
        }
    } catch {
        $message = $_.Exception.Message
        Undo-Install $done
        throw $message
    }
    return ,$done
}

function Undo-Install($Done) {
    # Возвращает каталоги на прежние места в обратном порядке: новая копия - обратно в сторону, старая - на место.
    for ($index = $Done.Count - 1; $index -ge 0; $index--) {
        $entry = $Done[$index]
        if ($entry.Staged -and (Test-Path -LiteralPath $entry.Target)) {
            try { [System.IO.Directory]::Move($entry.Target, $entry.Staged) } catch { Write-Host "  could not take the new copy away from $($entry.Target): $($_.Exception.Message)" -ForegroundColor Red }
        }
        if ($entry.Old) {
            try { [System.IO.Directory]::Move($entry.Old, $entry.Target) } catch { Write-Host "  COULD NOT RESTORE $($entry.Target) from $($entry.Old): $($_.Exception.Message)" -ForegroundColor Red }
        }
    }
}

function Commit-Install($Done, [string]$StagingRoot) {
    # Установка удалась: старые копии и сторона удаляются. Сбой удаления - предупреждение, а не отказ: Blender их не читает.
    $stuck = @()
    foreach ($entry in $Done) {
        if ($entry.Old -and -not (Remove-Quietly $entry.Old)) { $stuck += $entry.Old }
    }
    if (-not (Remove-Staging $StagingRoot)) { $stuck += $StagingRoot }
    foreach ($path in $stuck) {
        Write-Host "  left behind (safe to delete by hand): $path" -ForegroundColor Yellow
    }
}

function Get-LockMessage([string]$What, $Locked) {
    # Сообщение на английском намеренно: PowerShell 5.1 читает файл без BOM в кодовой странице системы.
    $shown = @($Locked | Select-Object -First 5) -join "`n    "
    $more = ''
    if ($Locked.Count -gt 5) { $more = "`n    ... and " + ($Locked.Count - 5) + " more" }
    return ("$What is held by another process (Blender or its domain pool workers keep these files open):`n    $shown$more`n" +
        "  Close Blender (its pool workers are the python.exe processes started with -c) and run this again. Nothing was changed.")
}
