import argparse
from dataclasses import dataclass
import fnmatch
import os
import re
import subprocess
import sys

import lizard
import pandas as pd

try:
    from tree_sitter import Language, Parser
    import tree_sitter_bash
except ImportError:
    Language = None
    Parser = None
    tree_sitter_bash = None


SHELL_EXTENSIONS = {'.sh', '.bash'}
EXCLUDE_PATTERNS = [
    '*/third_party/*',
    '*/dreamview/frontend/*',
    '*/.cache/*',
    '*test*',
]
SKIP_LIST = {
    (
        'modules/planning/tasks/optimizers/piecewise_jerk_path/'
        'piecewise_jerk_path_optimizer.cc',
        'century::planning::PiecewiseJerkPathOptimizer::GetSelfPathEndStateL',
    ),
    (
        'modules/planning/on_lane_planning.cc',
        'century::planning::OnLanePlanning::RunOnce',
    ),
    (
        'modules/planning/on_lane_planning.cc',
        'century::planning::OnLanePlanning::CheckEnableBorrow',
    ),
}


@dataclass
class FunctionInfo:
    name: str
    start_line: int
    end_line: int

    @property
    def length(self):
        return self.end_line - self.start_line + 1


def get_actual_modified_lines(file_path, target_branch):
    modified_lines = set()
    try:
        cmd = [
            'git', 'diff', '-U0', '--ignore-all-space', '--ignore-blank-lines',
            f'origin/{target_branch}...HEAD', '--', file_path,
        ]
        result = subprocess.check_output(
            cmd, encoding='utf-8', stderr=subprocess.STDOUT
        )

        for line in result.splitlines():
            if line.startswith('@@'):
                match = re.search(r'\+(\d+)(,(\d+))?', line)
                if match:
                    start = int(match.group(1))
                    length = int(match.group(3)) if match.group(3) else 1
                    if length > 0:
                        for line_no in range(start, start + length):
                            modified_lines.add(line_no)
    except Exception:
        return set()
    return modified_lines


def should_skip_file(file_path, rel_path):
    return any(
        fnmatch.fnmatch(file_path, pattern) or fnmatch.fnmatch(rel_path, pattern)
        for pattern in EXCLUDE_PATTERNS
    )


def analyze_with_lizard(file_path):
    file_info = lizard.analyze_file(file_path)
    return [
        FunctionInfo(
            name=func.name,
            start_line=func.start_line,
            end_line=func.end_line,
        )
        for func in file_info.function_list
    ]


def get_shell_parser():
    if Language is None or Parser is None or tree_sitter_bash is None:
        raise RuntimeError(
            'Shell parser dependencies are missing. '
            'Please install tree-sitter and tree-sitter-bash.'
        )

    parser = Parser()
    parser.language = Language(tree_sitter_bash.language())
    return parser


def analyze_shell_functions(file_path):
    parser = get_shell_parser()
    source = open(file_path, 'rb').read()
    tree = parser.parse(source)
    root = tree.root_node
    if root.has_error:
        raise RuntimeError(f'Failed to parse shell script: {file_path}')

    functions = []
    stack = [root]
    while stack:
        node = stack.pop()
        if node.type == 'function_definition':
            name_node = next(
                (child for child in node.children if child.type == 'word'),
                None,
            )
            if name_node is not None:
                name = source[name_node.start_byte:name_node.end_byte].decode(
                    'utf-8', errors='ignore'
                )
                functions.append(
                    FunctionInfo(
                        name=name,
                        start_line=node.start_point.row + 1,
                        end_line=node.end_point.row + 1,
                    )
                )
        stack.extend(reversed(node.children))

    return functions


def analyze_functions(file_path):
    extension = os.path.splitext(file_path)[1].lower()
    if extension in SHELL_EXTENSIONS:
        return analyze_shell_functions(file_path)
    return analyze_with_lizard(file_path)


def analyze_modified_functions(
    targets, target_branch, line_threshold=200, excel_path=None
):
    total_functions_scanned = 0
    violation_functions = []
    all_data = []

    print(f'CI Checking: Functions touched in this MR vs {target_branch}')
    print(f'Threshold: {line_threshold} lines')
    print('-' * 80)

    for file_path in targets:
        if not os.path.exists(file_path):
            continue
        rel_path = os.path.relpath(file_path)

        if should_skip_file(file_path, rel_path):
            continue

        touched_lines = get_actual_modified_lines(file_path, target_branch)
        if not touched_lines:
            continue

        try:
            function_infos = analyze_functions(file_path)
            for func in function_infos:
                if (rel_path, func.name) in SKIP_LIST:
                    continue

                func_range = set(range(func.start_line, func.end_line + 1))
                if func_range.isdisjoint(touched_lines):
                    continue

                total_functions_scanned += 1
                if func.length > line_threshold:
                    violation_functions.append({
                        'name': func.name,
                        'length': func.length,
                        'file': rel_path,
                        'line': func.start_line,
                    })
                    all_data.append({
                        'Function': func.name,
                        'Length': func.length,
                        'File': rel_path,
                        'Line': func.start_line,
                    })
        except Exception as exc:
            print(f'Error analyzing {rel_path}: {exc}')

    print('\n' + '=' * 80)
    print('STATISTICS SUMMARY')
    print(f'Actually Modified Functions: {total_functions_scanned}')
    print(f'Threshold Violations: {len(violation_functions)}')
    print('=' * 80)

    if violation_functions:
        violation_functions.sort(key=lambda item: item['length'], reverse=True)
        for function_info in violation_functions:
            print(
                f"{function_info['length']:>4} lines | "
                f"{function_info['name']} | "
                f"{function_info['file']}:{function_info['line']}"
            )
    else:
        print('No modified functions exceed the length threshold.')

    if excel_path and all_data:
        pd.DataFrame(all_data).to_excel(excel_path, index=False)

    return len(violation_functions)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('paths', nargs='*')
    parser.add_argument('--threshold', type=int, default=200)
    parser.add_argument(
        '--branch', required=True,
        help='Target branch for comparison, e.g., main',
    )
    parser.add_argument('--excel', type=str)
    args = parser.parse_args()

    target_files = args.paths
    if not target_files:
        try:
            cmd = ['git', 'diff', '--name-only', f'origin/{args.branch}...HEAD']
            target_files = subprocess.check_output(cmd, encoding='utf-8').splitlines()
        except Exception:
            target_files = []

    count = analyze_modified_functions(
        target_files, args.branch, args.threshold, args.excel
    )
    sys.exit(1 if count > 0 else 0)
