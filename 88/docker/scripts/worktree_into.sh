#!/usr/bin/env bash

###############################################################################
# Enter worktree container interactively.
#
# Usage:
#   ./worktree_into.sh <worktree-alias> [--user <user>]
#   ./worktree_into.sh -n <worktree-alias> [--user <user>]
#
# Examples:
#   ./worktree_into.sh freespace
#   ./worktree_into.sh -n segmentation
#   ./worktree_into.sh refactor --user root
###############################################################################

CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
source "${CURR_DIR}/worktree_manager.sh"

DOCKER_USER="${USER}"
WORKTREE_ALIAS=""

function usage() {
    sed -n '2,13p' "${BASH_SOURCE[0]}"
}

function parse_arguments() {
    while [[ $# -gt 0 ]]; do
        local opt="$1"
        shift
        case "${opt}" in
            -n | --name)
                WORKTREE_ALIAS="$1"
                shift
                ;;
            --user)
                DOCKER_USER="$1"
                shift
                ;;
            -h | --help | help)
                usage
                exit 0
                ;;
            *)
                if [[ -z "${WORKTREE_ALIAS}" ]]; then
                    WORKTREE_ALIAS="${opt}"
                else
                    error "Unknown argument: ${opt}"
                    usage
                    exit 1
                fi
                ;;
        esac
    done
}

function restart_stopped_container() {
    local container="$1"
    if docker ps -f status=exited -f name="^${container}$" --format '{{.Names}}' | grep -qx "${container}"; then
        info "Starting stopped container [${container}] ..."
        docker start "${container}" >/dev/null
    fi
}

function main() {
    parse_arguments "$@"

    if [[ -z "${WORKTREE_ALIAS}" ]]; then
        error "Usage: $0 <worktree-alias>"
        list_worktrees
        exit 1
    fi

    local alias
    alias="$(resolve_alias "${WORKTREE_ALIAS}")" || exit 1

    local container
    container="$(container_name "${alias}")"

    xhost +local:root 1>/dev/null 2>&1
    restart_stopped_container "${container}"

    if ! container_running "${container}"; then
        xhost -local:root 1>/dev/null 2>&1
        error "Container [${container}] is not running."
        echo "Start it first: ${CURR_DIR}/worktree_manager.sh start ${alias}"
        exit 1
    fi

    info "Entering container [${container}] (branch: ${WORKTREE_BRANCH[$alias]}) ..."
    docker exec \
        -u "${DOCKER_USER}" \
        -e HISTFILE=/century/.dev_bash_hist \
        -it "${container}" \
        /bin/bash

    xhost -local:root 1>/dev/null 2>&1
}

main "$@"
