import os
import sys
import re
import subprocess
import shutil
from urllib.parse import urlparse
import stat
import time

def remove_readonly(func, path, excinfo):
    try:
        os.chmod(path, stat.S_IWRITE)
        func(path)
    except Exception:
        pass

def normalize_git_url(url):
    url = url.split('?')[0].strip()
    url = re.sub(r'/blob/[^/]+/.*$', '', url)
    url = re.sub(r'/tree/[^/]+/?.*$', '', url)
    if url.endswith('.git'):
        url = url[:-4]
    url = url.rstrip('/')
    return url.lower()

def extract_roll_and_name(folder_name, content):
    rolls = []
    # Broaden regex to find any roll
    for m in re.finditer(r'(TCR(?:24|23|25|22)CS\d{3})', content, re.IGNORECASE):
        roll = m.group(1).upper()
        if roll not in rolls:
            rolls.append(roll)
            
    # Try guessing roll from folder name if not in content
    if not rolls:
        for m in re.finditer(r'(TCR(?:24|23|25|22)CS\d{3})', folder_name, re.IGNORECASE):
            roll = m.group(1).upper()
            if roll not in rolls:
                rolls.append(roll)

    names_found = []
    lines = content.split('\n')
    
    for line in lines:
        m = re.search(r'(?:Name|Student\s*Name)[\s\*\_]*[:\-]+[\s\*\_]*([A-Za-z\s\.]+)', line, re.IGNORECASE)
        if m:
            cand = m.group(1).strip().title()
            cand = re.sub(r'[^a-zA-Z\s\.]', '', cand).strip()
            # Ignore false positives
            if len(cand) > 2 and not any(x in cand.lower() for x in ['hmm', 'algorithm', 'register', 'university', 'student', 'project']):
                if cand not in names_found:
                    names_found.append(cand)
                    
    # Broad multi-line search if not found
    if not names_found and rolls:
        for roll in rolls:
            for line in lines:
                if roll.lower() in line.lower():
                    clean_line = re.sub(roll, '', line, flags=re.IGNORECASE)
                    # aggressive scrub
                    for word in ['Name', 'Roll', 'Number', 'No', 'is', 'student', 'Registration', 'University', 'Register', 'The', 'By', 'Submitted']:
                        clean_line = re.sub(r'\b' + word + r'\b', ' ', clean_line, flags=re.IGNORECASE)
                    clean_line = re.sub(r'[^a-zA-Z\s]', ' ', clean_line)
                    cand = ' '.join(clean_line.split()).title().strip()
                    if len(cand) > 2 and cand.lower() not in ['student', 'project'] and cand not in names_found:
                        names_found.append(cand)
    
    # Very aggressive fallback: look around 'Name' keyword if we missed the colon
    if not names_found:
        for line in lines:
            if 'name' in line.lower() and len(line) < 50:
                clean_line = re.sub(r'name', '', line, flags=re.IGNORECASE)
                clean_line = re.sub(r'[^a-zA-Z\s]', ' ', clean_line)
                cand = ' '.join(clean_line.split()).title().strip()
                if len(cand) > 2 and cand not in names_found:
                    names_found.append(cand)

    final_pairs = []
    
    # Match pairs or fill unknowns
    if rolls:
        for i, roll in enumerate(rolls):
            if i < len(names_found):
                name = names_found[i]
            else:
                name = "Unknown"
            final_pairs.append((roll, name))
    elif names_found:
        for name in names_found:
            final_pairs.append(("Unknown", name))
            
    return final_pairs

def get_existing_repos(base_dir):
    mapping = {}
    for entry in os.listdir(base_dir):
        full_path = os.path.join(base_dir, entry)
        info_file = os.path.join(full_path, '.repo_info')
        if os.path.isdir(full_path) and os.path.exists(info_file):
            with open(info_file, 'r', encoding='utf-8') as f:
                url = f.read().strip()
                n_url = normalize_git_url(url)
                mapping[n_url] = full_path
    return mapping

def check_and_append_deploy_link(dir_path, deploy_url):
    if not deploy_url: return
    
    readme_path = None
    for file in os.listdir(dir_path):
        if file.lower() == 'readme.md':
            readme_path = os.path.join(dir_path, file)
            break
            
    if not readme_path:
        readme_path = os.path.join(dir_path, 'README.md')
        
    try:
        content = ""
        if os.path.exists(readme_path):
            with open(readme_path, 'r', encoding='utf-8', errors='ignore') as f:
                content = f.read()
                
        if deploy_url not in content:
            with open(readme_path, 'a', encoding='utf-8') as f:
                f.write(f"\n\n## Live Deployment\n[Live Link]({deploy_url})\n")
    except Exception as e:
        print(f"Failed to append deploy link to {readme_path}: {e}")

def main():
    base_dir = os.getcwd()
    txt_file = 'PR assignment.txt'
    if not os.path.exists(txt_file):
        print(f"File {txt_file} not found!")
        sys.exit(1)
        
    groups_mode = False
    entries = []
    
    with open(txt_file, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            line = line.strip()
            if not line: continue
            if '--GROUPS--' in line:
                groups_mode = True
                continue
                
            parts = [p.strip() for p in line.split(',')]
            repo_url = parts[0]
            deploy_url = parts[1] if len(parts) > 1 else None
            
            if 'github.com' not in repo_url:
                continue
                
            # If deployed url happens to be missing http protocol
            if deploy_url and not deploy_url.startswith('http'):
                deploy_url = 'https://' + deploy_url
                
            entries.append({
                'repo': repo_url,
                'deploy': deploy_url,
                'is_group': groups_mode
            })
            
    print(f"Found {len(entries)} repository entries to process.")
    
    existing_repos = get_existing_repos(base_dir)
    print(f"Found {len(existing_repos)} already cloned repositories.")
    
    for entry in entries:
        raw_url = entry['repo']
        n_url = normalize_git_url(raw_url)
        deploy_url = entry['deploy']
        is_group = entry['is_group']
        print(f"Processing: {n_url}")
        
        target_dir = existing_repos.get(n_url)
        
        if target_dir and os.path.exists(target_dir):
            print(f"  -> Already exists at {os.path.basename(target_dir)}. Refetching...")
            username_repo = raw_url.split('github.com/')[-1].split('?')[0].replace('/', '_').replace('.git', '')
            username_repo = re.sub(r'[\\/*?:"<>|.]+$', '', username_repo)
            temp_dir = os.path.join(base_dir, f"temp_{username_repo}")
            
            if os.path.exists(temp_dir):
                shutil.rmtree(temp_dir, onerror=remove_readonly)
                
            clone_url = n_url + ".git" if not n_url.endswith('.git') else n_url
            res = subprocess.run(['git', 'clone', clone_url, temp_dir], capture_output=True, text=True)
            if res.returncode != 0:
                print(f"  -> Clone failed: {res.stderr}")
                continue
                
            # Clear target dir except .repo_info
            for item in os.listdir(target_dir):
                if item == '.repo_info': continue
                p = os.path.join(target_dir, item)
                try:
                    if os.path.isdir(p):
                        shutil.rmtree(p, onerror=remove_readonly)
                    else:
                        os.remove(p)
                except Exception as e:
                    pass
                    
            # Move new items to target_dir
            for item in os.listdir(temp_dir):
                src = os.path.join(temp_dir, item)
                dst = os.path.join(target_dir, item)
                try:
                    shutil.move(src, dst)
                except Exception:
                    pass
                
            # Remove inner .git and __pycache__
            for bad_dir in ['.git', '__pycache__']:
                bad_path = os.path.join(target_dir, bad_dir)
                if os.path.exists(bad_path):
                    shutil.rmtree(bad_path, onerror=remove_readonly)
            
            if os.path.exists(temp_dir):
                shutil.rmtree(temp_dir, onerror=remove_readonly)
                
            check_and_append_deploy_link(target_dir, deploy_url)
            print("  -> Updated successfully.")
            continue
            
        # Needs clone
        username_repo = raw_url.split('github.com/')[-1].split('?')[0].replace('/', '_').replace('.git', '')
        username_repo = re.sub(r'[\\/*?:"<>|.]+$', '', username_repo)
        temp_dir = os.path.join(base_dir, f"temp_{username_repo}")
        
        if os.path.exists(temp_dir):
            shutil.rmtree(temp_dir, onerror=remove_readonly)
            
        print(f"  -> Cloning...")
        clone_url = n_url + ".git" if not n_url.endswith('.git') else n_url
        res = subprocess.run(['git', 'clone', clone_url, temp_dir], capture_output=True, text=True)
        if res.returncode != 0:
            print(f"  -> Clone failed: {res.stderr}")
            continue
            
        # Read README to extract Roll & Name
        readme_content = ""
        for file in os.listdir(temp_dir):
            if file.lower() == 'readme.md' or file.lower() == 'readme.txt':
                with open(os.path.join(temp_dir, file), 'r', encoding='utf-8', errors='ignore') as f:
                    readme_content = f.read()
                break
                
        students = extract_roll_and_name(username_repo, readme_content)
        
        if not students:
            # Try exploring deeper if README has nothing
            # Or just unknown
            final_name = f"[UNKNOWN] - {username_repo}"
        else:
            if is_group:
                parts = []
                for roll, name in students:
                    parts.append(roll)
                final_name = f"[Group] " + " ".join(parts)
            else:
                # Individual format: TCR24CS029 - Name
                roll, name = students[0]
                final_name = f"{roll} - {name}"
                
        # Handle invalid chars in folder name
        final_name = re.sub(r'[\\/*?:"<>|]', "", final_name)
        
        final_path = os.path.join(base_dir, final_name)
        
        # Ensure uniqueness if folder already exists
        counter = 1
        original_final_path = final_path
        while os.path.exists(final_path):
            final_path = f"{original_final_path}_{counter}"
            counter += 1
            
        # Rename
        time.sleep(1)
        try:
            shutil.move(temp_dir, final_path)
            print(f"  -> Renamed to: {os.path.basename(final_path)}")
            # Append deploy link
            check_and_append_deploy_link(final_path, deploy_url)
            
            # Write .repo_info and delete .git and __pycache__
            with open(os.path.join(final_path, '.repo_info'), 'w', encoding='utf-8') as f:
                f.write(n_url)
            for bad_dir in ['.git', '__pycache__']:
                bad_path = os.path.join(final_path, bad_dir)
                if os.path.exists(bad_path):
                    shutil.rmtree(bad_path, onerror=remove_readonly)
        except Exception as e:
            print(f"  -> Failed to rename {temp_dir} to {final_path}: {e}")

    print("Finished processing all repositories.")
    print("Generating master README.md...")
    generate_readme(base_dir)

def generate_readme(base_dir):
    entries = []
    
    for d in os.listdir(base_dir):
        if not os.path.isdir(d):
            continue
            
        if d in ['.git', '__pycache__'] or d.startswith('.'):
            continue
            
        if d.startswith('[Group]'):
            rolls = []
            for m in re.finditer(r'(TCR.*?CS\d{3})', d, re.IGNORECASE):
                roll = m.group(1).upper()
                if roll not in rolls:
                    rolls.append(roll)
                    
            if rolls:
                readme_content = ""
                for file in os.listdir(os.path.join(base_dir, d)):
                    if file.lower() in ['readme.md', 'readme.txt']:
                        try:
                            with open(os.path.join(base_dir, d, file), 'r', encoding='utf-8', errors='ignore') as f:
                                readme_content = f.read()
                        except: pass
                        break
                        
                students = extract_roll_and_name(d, readme_content)
                roll_to_name = {r: n for r, n in students}
                
                for roll in rolls:
                    name = roll_to_name.get(roll, "Group Member")
                    entries.append((roll, name, d))
            else:
                entries.append(("UNKNOWN", "Group Member", d))
        else:
            m = re.search(r'(TCR.*?CS\d{3}|\[UNKNOWN\])\s*-\s*(.*)', d, re.IGNORECASE)
            if m:
                entries.append((m.group(1).upper(), m.group(2).strip(), d))
                
    entries.sort(key=lambda x: x[0] if x[0] != '[UNKNOWN]' else 'ZZZ')
    
    readme_path = os.path.join(base_dir, 'README.md')
    
    try:
        with open(readme_path, 'w', encoding='utf-8') as f:
            f.write("# Pattern Recognition - S4 Assignments\n\n")
            f.write("This repository contains all the student submissions for the HMM Baum-Welch Algorithm assignment.\n\n")
            f.write("## Submissions\n\n")
            f.write("| Registration Number | Student Name | Directory Link |\n")
            f.write("| :--- | :--- | :--- |\n")
            
            for reg, name, directory in entries:
                clean_dir = directory.replace(' ', '%20')
                f.write(f"| {reg} | {name} | [{directory}](./{clean_dir}) |\n")
                
        print(f"Generated README.md with {len(entries)} student entries.")
    except Exception as e:
        print(f"Failed to generate README.md: {e}")

if __name__ == '__main__':
    main()
