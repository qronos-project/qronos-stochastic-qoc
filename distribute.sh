#!/bin/bash

# create ZIP of this git repository

set -ex
./git-archive-all.sh/git-archive-all.sh --format tar

# Windows git-bash has no zip
which zip 2>/dev/null || { echo "zip not found. Please run this from a linux shell or from cygwin."; exit 1; }

# convert tar to zip, because --format zip is broken at least under cygwin
# (TODO fix this issue in git-archive-all)
wd="$(pwd)"
dir="$(mktemp -d)"
rev="$(date -I)-$(git describe --always --dirty)"
cd "$dir";
mkdir "${dir}/qronos-qoc"
cd "${dir}/qronos-qoc"
tar xf "${wd}/qronos-qoc.tar"
cd ..
zip "${wd}/qronos-qoc-$rev.zip" -m -r qronos-qoc
cd "$wd"
rmdir "$dir"
rm "${wd}/qronos-qoc.tar"
