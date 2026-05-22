#!/usr/bin/env bash

###############################################################################
# Worktree Container Manager
# Unified script to manage Docker containers for each git worktree branch.
# Each worktree gets its own isolated container with the same image.
#
# Usage:
#   ./worktree_manager.sh [command] [worktree]
#
# Commands:
#   start   [worktree|all]   Start container(s)
#   stop    [worktree|all]   Stop container(s)
#   restart [worktree|all]   Restart container(s)
#   into    <worktree>       Enter a running container (interactive)
#   status                   Show status of all worktree containers
#   list                     List available worktrees
#
# Examples:
#   ./worktree_manager.sh start all
#   ./worktree_manager.sh start freespace
#   ./worktree_manager.sh into segmentation
#   ./worktree_manager.sh stop all
#   ./worktree_manager.sh status
###############################################################################

CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
source "${CURR_DIR}/docker_base.sh"

CENTURY_IMAGE="century:centuryV7"
SHM_SIZE="2G"
DEV_INSIDE="in-dev-docker"
WORKSPACE_BASE="/mnt/kunlang/workspace/project"

###############################################################################
# Worktree definitions: alias -> (dir_suffix, branch_hint)
# Container name will be: century_dev_${USER}_${alias}
###############################################################################
declare -A WORKTREE_DIR=(
    [main]="century"
    [freespace]="century-ai-freespace"
    [refactor]="century-ai-refactor-lidar"
    [segmentation]="century-ai-segmentation"
)

declare -A WORKTREE_BRANCH=(
    [main]="wangrui_dev"
    [freespace]="ai/feat-freespace"
    [refactor]="ai/refactor-lidar-thread"
    [segmentation]="ai/feat-segmentation"
)

WORKTREE_ALIASES=(main freespace refactor segmentation)

###############################################################################
# Helper functions
###############################################################################
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
BOLD='\033[1m'
NC='\033[0m'

info()    { echo -e "${BLUE}[INFO]${NC} $*"; }
ok()      { echo -e "${GREEN}[OK]${NC}   $*"; }
warning() { echo -e "${YELLOW}[WARN]${NC} $*"; }
error()   { echo -e "${RED}[ERR]${NC}  $*" >&2; }

function container_name() {
    local alias="$1"
    if [[ "${alias}" == "main" ]]; then
        echo "century_dev_${USER}"
    else
        echo "century_dev_${USER}_${alias}"
    fi
}

function container_exists() {
    docker ps -a --format '{{.Names}}' | grep -q "^$1$"
}

function container_running() {
    docker ps --format '{{.Names}}' | grep -q "^$1$"
}

function resolve_alias() {
    local input="$1"
    # exact match first
    for alias in "${WORKTREE_ALIASES[@]}"; do
        if [[ "${alias}" == "${input}" ]]; then
            echo "${alias}"
            return 0
        fi
    done
    error "Unknown worktree alias: '${input}'"
    echo "Available: ${WORKTREE_ALIASES[*]}"
    return 1
}

function setup_worktree_volumes() {
    local alias="$1"
    local __retval="$2"
    local -n volumes_ref="${__retval}"
    local dir="${WORKSPACE_BASE}/${WORKTREE_DIR[$alias]}"

    if [[ -f "${dir}/scripts/century_base.sh" ]]; then
        # Reuse the same host device preparation flow as dev_start.sh.
        # shellcheck disable=SC1090
        source "${dir}/scripts/century_base.sh"
        setup_device
    else
        warning "century_base.sh not found under ${dir}/scripts, skip setup_device"
    fi

    local volume_args="-v ${dir}:/century"

    # Share the main worktree's data directory across all containers to avoid duplication.
    local main_data="${WORKSPACE_BASE}/${WORKTREE_DIR[main]}/data"
    if [[ "${alias}" != "main" && -d "${main_data}" ]]; then
        volume_args="${volume_args} -v ${main_data}:/century/data"
        info "    Data dir:  ${main_data} -> /century/data (shared)"
    fi

    volume_args="${volume_args} -v /dev:/dev"
    volume_args="${volume_args} -v /media:/media"
    volume_args="${volume_args} -v /tmp/.X11-unix:/tmp/.X11-unix:rw"
    volume_args="${volume_args} -v /etc/localtime:/etc/localtime:ro"
    volume_args="${volume_args} -v /usr/src:/usr/src"
    volume_args="${volume_args} -v /lib/modules:/lib/modules"
    volume_args="${volume_args} -v /home/${USER}/disk6/wangrui:/mnt/disk6"
    volume_args="${volume_args} -v /home/${USER}/KLPC:/mnt/disk"
    volumes_ref="$(tr -s ' ' <<<"${volume_args}")"
}

###############################################################################
# Start a single worktree container
###############################################################################
function start_worktree() {
    local alias="$1"
    local dir="${WORKSPACE_BASE}/${WORKTREE_DIR[$alias]}"
    local branch="${WORKTREE_BRANCH[$alias]}"
    local container
    container="$(container_name "${alias}")"

    if [[ ! -d "${dir}" ]]; then
        error "Worktree directory not found: ${dir}"
        return 1
    fi

    info "=== Starting container for [${alias}] (branch: ${branch}) ==="
    info "    Dir:       ${dir}"
    info "    Container: ${container}"

    # Remove existing container if present
    if container_exists "${container}"; then
        warning "Removing existing container: ${container}"
        docker stop "${container}" >/dev/null 2>&1
        docker rm -f "${container}" >/dev/null 2>&1
    fi

    # Detect GPU
    determine_gpu_use_host
    info "    USE_GPU_HOST: ${USE_GPU_HOST}"
    local docker_run_cmd="${DOCKER_RUN_CMD}"
    info "    DOCKER_RUN_CMD: ${docker_run_cmd}"

    local local_host
    local_host="$(hostname)"
    local display="${DISPLAY:-:0}"
    local uid
    uid="$(id -u)"
    local group
    group="$(id -g -n)"
    local gid
    gid="$(id -g)"

    local volumes=
    setup_worktree_volumes "${alias}" volumes

    set -x
    ${docker_run_cmd} -itd \
        --privileged \
        --name "${container}" \
        -e DISPLAY="${display}" \
        -e DOCKER_USER="${USER}" \
        -e USER="${USER}" \
        -e DOCKER_USER_ID="${uid}" \
        -e DOCKER_GRP="${group}" \
        -e DOCKER_GRP_ID="${gid}" \
        -e DOCKER_IMG="${CENTURY_IMAGE}" \
        -e USE_GPU_HOST="${USE_GPU_HOST}" \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=compute,video,graphics,utility \
        ${volumes} \
        --net host \
        --pid host \
        -w /century \
        --add-host "${DEV_INSIDE}:127.0.0.1" \
        --add-host "${local_host}:127.0.0.1" \
        --hostname "${DEV_INSIDE}" \
        --shm-size "${SHM_SIZE}" \
        -v /dev/null:/dev/raw1394 \
        "${CENTURY_IMAGE}" \
        /bin/bash
    local ret=$?
    set +x

    if [[ ${ret} -ne 0 ]]; then
        error "Failed to start container: ${container}"
        return 1
    fi

    # Setup user inside container
    postrun_start_user "${container}"
    rename_docker_user_in "${container}"
    ensure_container_user_in "${container}"
    ensure_bazel_gpu_config "${container}"

    ok "Container [${container}] started. Enter with:"
    ok "  $0 into ${alias}"
}

###############################################################################
# Rename user inside container (same as original rename_docker_user)
###############################################################################
function rename_docker_user_in() {
    local container="$1"
    local files=(/etc/shadow- /etc/subgid /etc/subuid /etc/passwd /etc/group /etc/group- /etc/shadow)
    for f in "${files[@]}"; do
        docker exec -u root "${container}" bash -c "sed -i 's#admin#${USER}#g' ${f}" 2>/dev/null
    done
    docker exec -u root "${container}" bash -c \
        "[ -d /home/admin ] && mv /home/admin/ /home/${USER}/ || true" 2>/dev/null
}

function ensure_container_user_in() {
    local container="$1"

    info "Ensuring container user ${USER}:${USER} exists in ${container} ..."
    docker exec -u root \
        -e TARGET_USER="${USER}" \
        -e TARGET_UID="$(id -u)" \
        -e TARGET_GROUP="$(id -g -n)" \
        -e TARGET_GID="$(id -g)" \
        "${container}" bash -lc '
            set -e

            existing_user="$(getent passwd "${TARGET_UID}" | cut -d: -f1 || true)"
            existing_group="$(getent group "${TARGET_GID}" | cut -d: -f1 || true)"

            if [[ -n "${existing_group}" && "${existing_group}" != "${TARGET_GROUP}" ]]; then
                groupmod -n "${TARGET_GROUP}" "${existing_group}"
            elif ! getent group "${TARGET_GROUP}" >/dev/null; then
                addgroup --gid "${TARGET_GID}" "${TARGET_GROUP}"
            fi

            if getent passwd "${TARGET_USER}" >/dev/null; then
                usermod -u "${TARGET_UID}" -g "${TARGET_GID}" "${TARGET_USER}" || true
            elif [[ -n "${existing_user}" && "${existing_user}" != "${TARGET_USER}" ]]; then
                usermod -l "${TARGET_USER}" -d "/home/${TARGET_USER}" -m "${existing_user}"
                usermod -g "${TARGET_GID}" "${TARGET_USER}"
            else
                adduser --disabled-password --force-badname --gecos "" \
                    "${TARGET_USER}" --uid "${TARGET_UID}" --gid "${TARGET_GID}"
            fi

            usermod -aG sudo "${TARGET_USER}" || true
            usermod -aG video "${TARGET_USER}" || true
            usermod -aG audio "${TARGET_USER}" || true
            mkdir -p "/home/${TARGET_USER}"
            chown -R "${TARGET_UID}:${TARGET_GID}" "/home/${TARGET_USER}"
        '
}

function ensure_bazel_gpu_config() {
    local container="$1"

    info "Ensuring Bazel GPU config in ${container} ..."
    docker exec -u "${USER}" "${container}" bash -lc '
        cd /century || exit 1

        if [[ -f .century.bazelrc ]] && grep -q "^build:cuda " .century.bazelrc; then
            exit 0
        fi

        rm -f .century.bazelrc
        source cyber/setup.bash
        bash scripts/century_config.sh --noninteractive
    '
    local ret=$?

    if [[ ${ret} -ne 0 ]]; then
        warning "Failed to regenerate .century.bazelrc in ${container}"
        return ${ret}
    fi

    ok "Bazel GPU config is ready in ${container}"
}

###############################################################################
# Stop a single worktree container
###############################################################################
function stop_worktree() {
    local alias="$1"
    local container
    container="$(container_name "${alias}")"

    if ! container_exists "${container}"; then
        warning "Container not found: ${container}"
        return 0
    fi

    info "Stopping [${alias}] container: ${container}"
    docker stop "${container}" >/dev/null && \
        docker rm -f "${container}" >/dev/null 2>&1 && \
        ok "Stopped: ${container}" || \
        error "Failed to stop: ${container}"
}

###############################################################################
# Enter a running container interactively
###############################################################################
function into_worktree() {
    local alias="$1"
    local container
    container="$(container_name "${alias}")"

    if ! container_running "${container}"; then
        error "Container [${container}] is not running."
        echo "Start it first: $0 start ${alias}"
        return 1
    fi

    info "Entering container [${container}] (branch: ${WORKTREE_BRANCH[$alias]}) ..."
    docker exec -it "${container}" /bin/bash
}

###############################################################################
# Show status of all worktree containers
###############################################################################
function show_status() {
    printf "\n${BOLD}%-16s %-40s %-32s %-12s${NC}\n" "ALIAS" "CONTAINER" "BRANCH" "STATUS"
    printf '%s\n' "$(printf '%.0s-' {1..104})"

    for alias in "${WORKTREE_ALIASES[@]}"; do
        local container
        container="$(container_name "${alias}")"
        local branch="${WORKTREE_BRANCH[$alias]}"
        local dir="${WORKSPACE_BASE}/${WORKTREE_DIR[$alias]}"

        local status
        if container_running "${container}"; then
            status="${GREEN}RUNNING${NC}"
        elif container_exists "${container}"; then
            status="${YELLOW}STOPPED${NC}"
        else
            status="${RED}NOT FOUND${NC}"
        fi

        local dir_ok=""
        [[ ! -d "${dir}" ]] && dir_ok=" ${RED}[dir missing]${NC}"

        printf "%-16s %-40s %-32s " "${alias}" "${container}" "${branch}"
        echo -e "${status}${dir_ok}"
    done
    echo
}

###############################################################################
# List available worktrees
###############################################################################
function list_worktrees() {
    printf "\n${BOLD}%-16s %-40s %-32s${NC}\n" "ALIAS" "DIRECTORY" "BRANCH"
    printf '%s\n' "$(printf '%.0s-' {1..88})"
    for alias in "${WORKTREE_ALIASES[@]}"; do
        local dir="${WORKSPACE_BASE}/${WORKTREE_DIR[$alias]}"
        local branch="${WORKTREE_BRANCH[$alias]}"
        local exists=""
        [[ -d "${dir}" ]] && exists="${GREEN}✓${NC}" || exists="${RED}✗${NC}"
        printf "%-16s %-40s %-32s " "${alias}" "${WORKTREE_DIR[$alias]}" "${branch}"
        echo -e "${exists}"
    done
    echo
}

###############################################################################
# Main dispatcher
###############################################################################
function main() {
    local cmd="${1:-status}"
    local target="${2:-}"

    case "${cmd}" in
        start)
            if [[ "${target}" == "all" || -z "${target}" ]]; then
                for alias in "${WORKTREE_ALIASES[@]}"; do
                    start_worktree "${alias}"
                    echo
                done
            else
                local alias
                alias="$(resolve_alias "${target}")" || exit 1
                start_worktree "${alias}"
            fi
            ;;
        stop)
            if [[ "${target}" == "all" || -z "${target}" ]]; then
                for alias in "${WORKTREE_ALIASES[@]}"; do
                    stop_worktree "${alias}"
                done
            else
                local alias
                alias="$(resolve_alias "${target}")" || exit 1
                stop_worktree "${alias}"
            fi
            ;;
        restart)
            if [[ "${target}" == "all" || -z "${target}" ]]; then
                for alias in "${WORKTREE_ALIASES[@]}"; do
                    stop_worktree "${alias}"
                    start_worktree "${alias}"
                    echo
                done
            else
                local alias
                alias="$(resolve_alias "${target}")" || exit 1
                stop_worktree "${alias}"
                start_worktree "${alias}"
            fi
            ;;
        into)
            if [[ -z "${target}" ]]; then
                error "Usage: $0 into <worktree>"
                list_worktrees
                exit 1
            fi
            local alias
            alias="$(resolve_alias "${target}")" || exit 1
            into_worktree "${alias}"
            ;;
        status)
            show_status
            ;;
        list)
            list_worktrees
            ;;
        -h | --help | help)
            sed -n '2,20p' "${BASH_SOURCE[0]}"
            ;;
        *)
            error "Unknown command: ${cmd}"
            echo "Commands: start | stop | restart | into | status | list"
            exit 1
            ;;
    esac
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    main "$@"
fi
