import os
import re
import fnmatch
import argparse
import sys

def has_chinese(text):
    pattern = re.compile(r'[\u4e00-\u9fa5]')
    return bool(pattern.search(text))

def scan_files(file_list, root_dir="."):
    exclude_dirs = {'third_party', '.cache', '.git', 'bazel-bin', 'bazel-out', 'bazel-testlogs'}
    exclude_file_pattern = '*test*' 

    results = []
    print(f"Starting to check files, totaling: {len(file_list)} files to be selected")

    for file_path in file_list:
        try:
            rel_path = os.path.relpath(file_path, root_dir)
            
            path_parts = rel_path.split(os.sep)
            if any(part in exclude_dirs for part in path_parts) or any(part.startswith('.') for part in path_parts):
                continue

            if fnmatch.fnmatch(os.path.basename(rel_path).lower(), exclude_file_pattern):
                continue

            if not os.path.isfile(file_path):
                continue

            with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
                for line_num, line in enumerate(f, 1):
                    if has_chinese(line):
                        results.append({
                            'File': rel_path,
                            'Line': line_num,
                            'Content': line.strip()
                        })
                        print(f"Found Chinese: {rel_path}:{line_num} -> {line.strip()[:50]}")
                        
        except Exception as e:
            print(f"Error opening {file_path}: {e}")

    return results

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Check Chinese in all files")
    parser.add_argument('files', nargs='*', help='List of files to check')
    args = parser.parse_args()

    if args.files:
        targets = args.files
    else:
        targets = []
        print("No file specified, collecting all files in the current directory...")
        for root, dirs, files in os.walk("."):
            dirs[:] = [d for d in dirs if d not in {'.cache', 'third_party', '.git', 'bazel-bin'}]
            for f in files:
                targets.append(os.path.join(root, f))

    results = scan_files(targets)

    print("\n" + "="*50)
    if len(results) > 0:
        print(f"Total {len(results)} Chinese instances found. Please fix them.")
        sys.exit(1)
    else:
        print("No Chinese content found.")
        sys.exit(0)