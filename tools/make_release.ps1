param(
    [string]$TrainingDist = "D:\Samsung\realsense\dist\YoloSegTrainingUtility",
    [string]$OutputRoot = ".\release\DobotMG400Studio"
)

$ErrorActionPreference = "Stop"
$repo = Resolve-Path (Join-Path $PSScriptRoot "..")
$mainDist = Join-Path $repo "dist\RobotAutomationStudio"
$output = Join-Path $repo $OutputRoot
$zipPath = "$output.zip"
$releaseRoot = Join-Path $repo "release"

if (-not (Test-Path $mainDist)) {
    throw "RobotAutomationStudio dist was not found: $mainDist"
}
if (-not (Test-Path $TrainingDist)) {
    throw "YoloTrainingUtility dist was not found: $TrainingDist"
}

if (Test-Path $output) {
    $resolvedOutput = Resolve-Path $output
    $resolvedReleaseRoot = Resolve-Path $releaseRoot
    if (-not $resolvedOutput.Path.StartsWith($resolvedReleaseRoot.Path, [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "Refusing to delete outside release root: $resolvedOutput"
    }
    Remove-Item -LiteralPath $resolvedOutput.Path -Recurse -Force
}
if (Test-Path $zipPath) {
    Remove-Item -LiteralPath $zipPath -Force
}

New-Item -ItemType Directory -Path $output | Out-Null
Copy-Item -LiteralPath $mainDist -Destination (Join-Path $output "RobotAutomationStudio") -Recurse
Copy-Item -LiteralPath $TrainingDist -Destination (Join-Path $output "YoloTrainingUtility") -Recurse

foreach ($path in @(
    "RobotAutomationStudio\models",
    "RobotAutomationStudio\projects",
    "RobotAutomationStudio\configs",
    "YoloTrainingUtility\datasets",
    "YoloTrainingUtility\runs",
    "YoloTrainingUtility\pretrained_models"
)) {
    New-Item -ItemType Directory -Path (Join-Path $output $path) -Force | Out-Null
}

$sourcePretrained = "D:\Samsung\realsense\app_training\models"
if (Test-Path $sourcePretrained) {
    Get-ChildItem -LiteralPath $sourcePretrained | Copy-Item -Destination (Join-Path $output "YoloTrainingUtility\pretrained_models") -Recurse -Force
}

$sampleModel = "D:\Samsung\realsense\app_main\models\best.pt"
if (Test-Path $sampleModel) {
    Copy-Item -LiteralPath $sampleModel -Destination (Join-Path $output "RobotAutomationStudio\models\best.pt") -Force
}

Compress-Archive -LiteralPath $output -DestinationPath $zipPath -Force
Write-Host "Release folder: $output"
Write-Host "Release zip: $zipPath"
