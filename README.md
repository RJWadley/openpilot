# IQ.Pilot GitHub bootstrap

This branch installs [IQ.Pilot](https://iqlvbs.com). It contains no openpilot code.

Installers that only take a GitHub fork, like `installer.comma.ai/<user>/<branch>`, clone this branch and boot
`launch_openpilot.sh`. That script downloads the IQ.Pilot installer from `installer.iqlvbs.com` and reinstalls the
device from `git.konn3kt.com`, where all future updates come from.

The branch name picks the IQ.Pilot channel (`release`, `beta`, ...). A branch with no matching channel installs
`release`. If the install fails, the script retries every few seconds, and rebooting retries too.

`common/version.h` is only there for `installer.comma.ai`, which refuses to serve a comma four a branch without a
recent enough openpilot version. Nothing on the device reads it.

You can skip GitHub entirely by entering `installer.iqlvbs.com/release` as the custom software URL.

Maintained in the IQ monorepo at `pilot/github-bootstrap/` and published as a single-commit branch of a GitHub
repo named `openpilot`. Edit it there, not on GitHub.
