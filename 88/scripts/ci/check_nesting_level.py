import os
import argparse
import sys
import subprocess
import re
import lizard

def get_modified_lines_dict(branch):
    lines_dict = {}
    try:
        cmd = ["git", "diff", "-U0", f"origin/{branch}...HEAD"]
        output = subprocess.check_output(cmd, encoding='utf-8')
        
        current_file = None
        for line in output.splitlines():
            if line.startswith('+++ b/'):
                current_file = os.path.abspath(line[6:])
                lines_dict[current_file] = set()
            elif line.startswith('@@') and current_file:
                match = re.search(r'\+(\d+)(?:,(\d+))?', line)
                if match:
                    start = int(match.group(1))
                    length = int(match.group(2)) if match.group(2) else 1
                    for i in range(start, start + length):
                        lines_dict[current_file].add(i)
    except Exception as e:
        print(f"⚠️ Error getting diff lines: {e}")
    return lines_dict

def calculate_visual_depth(lines):
    code = "".join(lines)
    code = re.sub(r'//.*', '', code)
    code = re.sub(r'/\*.*?\*/', '', code, flags=re.DOTALL)
    code = re.sub(r'".*?"', '""', code)
    code = re.sub(r"'.*?'", "''", code)

    # Only braces after these patterns count as nesting
    logic_pattern = re.compile(
        r'(\b(if|else\s+if|else|for|while|switch|do|try|catch)\b|\))\s*$'
    )

    max_depth = 0
    current_depth = 0
    brace_stack = []

    pending_text = ""
    for char in code:
        if char == '{':
            is_logic = bool(logic_pattern.search(pending_text))
            brace_stack.append(is_logic)
            if is_logic:
                current_depth += 1
                max_depth = max(max_depth, current_depth)
            pending_text = ""
        elif char == '}':
            if brace_stack:
                was_logic = brace_stack.pop()
                if was_logic:
                    current_depth -= 1
            pending_text = ""
        else:
            pending_text += char

    return max(0, max_depth - 1)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('paths', nargs='*')
    parser.add_argument('--threshold', type=int, default=5)
    parser.add_argument('--branch', required=True)
    args = parser.parse_args()

    modified_data = get_modified_lines_dict(args.branch)
    target_files = [f for f in modified_data.keys() if f.lower().endswith(('.cc', '.cpp', '.h', '.hpp')) and os.path.exists(f)]

    print(f"🚀 Selective Nesting Checker | Threshold: {args.threshold}")
    print(f"Analyzing only modified functions in {len(target_files)} files...")
    print("-" * 75)

    total_scanned = 0
    total_violations = 0
    skip_list = {
        ('modules/planning/on_lane_planning.cc',
         'century::planning::OnLanePlanning::CheckEnableBorrow'),
    }

    for file_path in target_files:
        rel_path = os.path.relpath(file_path)
        changed_lines = modified_data.get(file_path, set())
        
        try:
            analysis = lizard.analyze_file(file_path)
            with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
                all_lines = f.readlines()
            
            for func in analysis.function_list:
                if (rel_path, func.name) in skip_list:
                    continue
                func_range = set(range(func.start_line, func.end_line + 1))
                if not (func_range & changed_lines):
                    continue  
                total_scanned += 1
                func_lines = all_lines[func.start_line - 1 : func.end_line]
                
                manual_depth = calculate_visual_depth(func_lines)
                depth = max(manual_depth, func.max_nesting_depth)

                status = "❌" if depth > args.threshold else "✅"
                print(f"{status} Depth: {depth:>2} | {func.name} | {rel_path}:{func.start_line}")
                
                if depth > args.threshold:
                    total_violations += 1
        except Exception as e:
            print(f"⚠️ Error processing {rel_path}: {e}")

    print(f"\n📊 SUMMARY: Scanned {total_scanned} modified functions, Found {total_violations} violations.")
    sys.exit(1 if total_violations > 0 else 0)

if __name__ == "__main__":
    main()
