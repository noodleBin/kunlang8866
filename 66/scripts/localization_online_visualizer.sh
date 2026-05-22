#! /bin/bash
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${DIR}/.."

source "${DIR}/century_base.sh"

cyber_launch start /century/modules/localization/launch/msf_visualizer.launch
