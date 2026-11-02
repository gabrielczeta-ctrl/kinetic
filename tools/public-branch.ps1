<#
Makes the public branch: the release tree as one commit on the Jolt
integration merge, without the private development notes. The development
history (local paths, e-mails, test commits) stays here; OpenMW's history and
the Jolt merge request's commits keep their authors.

usage: powershell -File tools/public-branch.ps1 -Message "KINETIC TECH-DEMO RELEASE 0.1" [-Tag name]
         [-Branch public/kinetic] [-Author "Name <address>"]
The first run makes the branch on a copy of the Jolt integration merge with
the public author; each later run adds one commit on the branch's tip.
Nothing is pushed. Package the release from the result with
tools/package-kinetic.ps1 -SourceCommit <branch>.
#>
param(
    [Parameter(Mandatory = $true)][string]$Message,
    [string]$Branch = 'public/kinetic',
    [string]$Tag = '',
    [string]$Author = 'gabrielczeta-ctrl <229237346+gabrielczeta-ctrl@users.noreply.github.com>'
)
$ErrorActionPreference = 'Stop'
$source = Split-Path -Parent $PSScriptRoot
# In-house (user 11-02: project documentation stays offline): the plan, the
# player's build log and verdicts, the agents' rules, the designs, research
# and audits (version-manifest.json stays: the tools read it), the store page
# draft, and the Mixamo-derived reference loops (release audit 10-06).
$private = @('PLAN.md', 'PLAYTEST.md', 'AGENTS.md', 'tools/workspace', 'docs/rewrite', 'docs/NEXUS-PAGE.txt',
    'docs/ragdoll', ':!docs/ragdoll/version-manifest.json', 'files/data/ragdoll/reference-loops.txt')

if ($Author -notmatch '^(.+?) <(.+)>$') { throw "Author must be 'Name <address>': $Author" }
$env:GIT_AUTHOR_NAME = $env:GIT_COMMITTER_NAME = $Matches[1]
$env:GIT_AUTHOR_EMAIL = $env:GIT_COMMITTER_EMAIL = $Matches[2]
# git.exe: a function named Git would call itself (names ignore case).
function Git { $out = & git.exe -C $source @args; if ($LASTEXITCODE) { throw "git $args failed" }; $out }

if (Git status --porcelain --untracked-files=no) { throw 'Commit the working tree first: the public commit is made from HEAD.' }

# The release tree without the private files, through a scratch index.
$index = Join-Path ([IO.Path]::GetTempPath()) "kinetic-public-$PID.index"
$env:GIT_INDEX_FILE = $index
try {
    Git read-tree HEAD | Out-Null
    Git rm -r -q --cached --ignore-unmatch -- @private | Out-Null
    $tree = (Git write-tree).Trim()
}
finally { Remove-Item -LiteralPath $index -ErrorAction SilentlyContinue; Remove-Item Env:GIT_INDEX_FILE }

$manifest = Get-Content -Raw (Join-Path $source 'docs\ragdoll\version-manifest.json') | ConvertFrom-Json
$merge = $manifest.source.jolt_integration_merge

# Nothing personal may leave: this machine's profile path or an e-mail from
# the development history.
$leaks = @("Users[\\/]+$([regex]::Escape($env:USERNAME))[\\/]")
foreach ($mail in @(Git config user.email) + @(Git log --format=%ae "$merge..HEAD") | Sort-Object -Unique) {
    if ($mail -and $mail -notmatch 'noreply|\.invalid$') { $leaks += [regex]::Escape($mail) }
}
$found = & git.exe -C $source grep -nIiE ($leaks -join '|') $tree -- . ':!AUTHORS.md'
if ($found) { $found | Select-Object -First 20 | Write-Output; throw 'Personal paths or addresses in the public tree (above).' }

$tip = & git.exe -C $source rev-parse --verify --quiet "refs/heads/$Branch"
if (!$tip) {
    # The Jolt integration merge again, under the public author.
    $tip = (Git commit-tree "$merge^{tree}" -p $manifest.source.openmw_upstream -p $manifest.source.jolt_merge_request `
        -m (Git log -1 --format=%B $merge | Out-String).Trim()).Trim()
}
if ((Git rev-parse "$tip^{tree}").Trim() -eq $tree) { throw "The public branch already has this tree ($tip)." }
$commit = (Git commit-tree $tree -p $tip -m $Message).Trim()
Git update-ref "refs/heads/$Branch" $commit | Out-Null
if ($Tag) { Git tag $Tag $commit | Out-Null }
Write-Output "$Branch = $commit (from $((Git rev-parse --short=10 HEAD).Trim()))$(if ($Tag) { ", tag $Tag" })"
