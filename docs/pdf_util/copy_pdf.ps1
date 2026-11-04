# 把外部的参考 PDF 复制为 pdf_util/main.pdf
# 用法： .\copy_pdf.ps1 -Source "<你的 PDF 路径>"
param(
    [Parameter(Mandatory = $true)][string]$Source
)

Copy-Item -LiteralPath $Source -Destination (Join-Path $PSScriptRoot "main.pdf") -Force
Write-Output "done"
